#include "rm_serial_driver/rm_serial_node.hpp"

#include <tf2_ros/buffer_interface.h>
#include <tf2_ros/transform_listener.h>

#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "rm_serial_driver/packet.hpp"


namespace rm_serial_driver
{

// Params implementation
template <typename T>
static T declare(
  rclcpp::Node * node, const std::string & name, const T & default_val)
{
  return node->declare_parameter(name, default_val);
}

Params::Params(rclcpp::Node * node)
{
  left_config.device_name =
    declare(node, "left.device_name", std::string("/dev/ttyACM0"));
  left_config.baudrate = declare(node, "left.baud_rate", 115200);
  left_config.hardware_flow = declare(node, "left.hardware_flow", false);
  left_config.parity = declare(node, "left.parity", false);
  left_config.stop_bits = declare(node, "left.stop_bits", 1);
  left_config.timeout_ms = declare(node, "left.timeout_ms", 1000);

  right_config.device_name =
    declare(node, "right.device_name", std::string("/dev/ttyACM1"));
  right_config.baudrate = declare(node, "right.baud_rate", 115200);
  right_config.hardware_flow = declare(node, "right.hardware_flow", false);
  right_config.parity = declare(node, "right.parity", false);
  right_config.stop_bits = declare(node, "right.stop_bits", 1);
  right_config.timeout_ms = declare(node, "right.timeout_ms", 1000);

  is_debug = declare(node, "is_debug", false);
}

// Publishers implementation
Publishers::Publishers(rclcpp::Node * node) : tf_broadcaster(node) {}

// Subscribers implementation
Subscribers::Subscribers(rclcpp::Node * node, RMSerialDriver * parent)
{
  tf_buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);
  target_sub = node->create_subscription<auto_aim_interfaces::msg::Target>(
    "target_topic", 10,
    std::bind(&RMSerialDriver::handleMsg, parent, std::placeholders::_1));
}

// Clients implementation
Clients::Clients(rclcpp::Node * node) : node_ptr(node)
{
  detector_client =
    std::make_shared<rclcpp::AsyncParametersClient>(node_ptr, "armor_detector");
  rune_client =
    std::make_shared<rclcpp::AsyncParametersClient>(node_ptr, "rune_detector");
  reset_tracker_srv =
    node_ptr->create_client<std_srvs::srv::Trigger>("/reset_tracker");
}

void Clients::setParam(const rclcpp::Parameter & param)
{
  setParamInternal(param, detector_client, detector_future, "Armor");
  setParamInternal(param, rune_client, rune_future, "Rune", true);
}

void Clients::setParamInternal(
  const rclcpp::Parameter & param,
  const rclcpp::AsyncParametersClient::SharedPtr & client,
  ResultFuturePtr & future, const std::string & name, bool invert)
{
  if (!client->service_is_ready()) {
    RCLCPP_WARN(
      node_ptr->get_logger(), "%s service not ready, skipping", name.c_str());
    return;
  }
  if (
    !future.valid() ||
    future.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
    int value = invert ? (1 - param.as_int()) : param.as_int();
    rclcpp::Parameter new_param(param.get_name(), value);
    RCLCPP_INFO(
      node_ptr->get_logger(), "Setting %s to %d", name.c_str(), value);
    future = client->set_parameters(
      {new_param}, [this, name, value](const ResultFuturePtr & results) {
        for (auto & res : results.get()) {
          if (!res.successful) {
            RCLCPP_ERROR(
              node_ptr->get_logger(), "%s set failed: %s", name.c_str(),
              res.reason.c_str());
            return;
          }
        }
        RCLCPP_INFO(
          node_ptr->get_logger(), "%s set to %d succeeded", name.c_str(),
          value);
      });
  }
}

// RMSerialDriver implementation
RMSerialDriver::RMSerialDriver(const rclcpp::NodeOptions & opts)
: Node("rm_serial_driver", opts),
  prev_color_(0),
  params_(this),
  pubs_(this),
  subs_(this, this),
  clis_(this),
  left_serial_parser_(get_logger()),
  right_serial_parser_(get_logger())
{
  left_serial_parser_.registerHandler<ReceiveImuData>(
    0x01, [this](auto & pkt) { handlePacket(pkt, DoubleEnd::LEFT); });
  left_serial_parser_.registerHandler<ReceiveTargetInfoData>(
    0x09, [this](auto & pkt) { handlePacket(pkt, DoubleEnd::LEFT); });
  right_serial_parser_.registerHandler<ReceiveImuData>(
    0x01, [this](auto & pkt) { handlePacket(pkt, DoubleEnd::RIGHT); });
  right_serial_parser_.registerHandler<ReceiveTargetInfoData>(
    0x09, [this](auto & pkt) { handlePacket(pkt, DoubleEnd::RIGHT); });
  left_port_ = std::make_shared<SerialPort>(params_.left_config);
  right_port_ = std::make_shared<SerialPort>(params_.right_config);
  left_port_->open();
  right_port_->open();
  left_thread_ = std::thread([this] { receiveLoop(DoubleEnd::LEFT); });
  right_thread_ = std::thread([this] { receiveLoop(DoubleEnd::RIGHT); });
}

RMSerialDriver::~RMSerialDriver()
{
  rclcpp::shutdown();
  if (left_thread_.joinable()) left_thread_.join();
  if (right_thread_.joinable()) right_thread_.join();
}

void RMSerialDriver::receiveLoop(DoubleEnd end)
{
  auto & port = (end == DoubleEnd::LEFT ? left_port_ : right_port_);
  auto & parser =
    (end == DoubleEnd::LEFT ? left_serial_parser_ : right_serial_parser_);
  uint8_t * buf = (end == DoubleEnd::LEFT ? left_buf_ : right_buf_);
  std::string end_str = end==DoubleEnd::LEFT ? "left" : "right";

  while (rclcpp::ok()) {
    ssize_t n = port->read(buf, BUFFER_SIZE);
    if (n > 0) {
      // 真正读到数据，喂给解析器
      parser.feed(buf, static_cast<size_t>(n));
    } else if (n == 0 || (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK))) {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    } else {
      // 真正的读错误，打印并退出循环
      RCLCPP_ERROR(
        get_logger(), "%s port read error (%s)",
        (end == DoubleEnd::LEFT ? "Left" : "Right"), std::strerror(errno));
      break;
    }
  }
}

void RMSerialDriver::handlePacket(const ReceiveImuData & pkt, DoubleEnd end)
{
  tf2::Quaternion q;
  q.setRPY(pkt.data.roll, pkt.data.pitch, pkt.data.yaw);
  q.normalize();
  if (params_.is_debug) {
    std::cout << "roll:" << pkt.data.roll << " pitch: " << pkt.data.pitch << " yaw: " << pkt.data.yaw << std::endl;
  }
  geometry_msgs::msg::TransformStamped t;
  t.header.stamp = now();
  t.header.frame_id = "gimbal_big_link";
  t.child_frame_id =
    (end == DoubleEnd::LEFT ? "gimbal_left_link" : "gimbal_right_link");
  t.transform.rotation = tf2::toMsg(q);
  pubs_.tf_broadcaster.sendTransform(t);
}

void RMSerialDriver::handlePacket(
  const ReceiveTargetInfoData & pkt, DoubleEnd end)
{
  if (pkt.data.reset_tracker && clis_.reset_tracker_srv->service_is_ready()) {
    clis_.reset_tracker_srv->async_send_request(
      std::make_shared<std_srvs::srv::Trigger::Request>());
  }
  if (prev_color_ != pkt.data.detect_color) {
    clis_.setParam(rclcpp::Parameter(
      "detect_color", static_cast<bool>(pkt.data.detect_color)));
    prev_color_ = pkt.data.detect_color;
  }
}

void RMSerialDriver::handleMsg(auto_aim_interfaces::msg::Target::SharedPtr msg)
{
  // 1. 填 packet（先用原始 odom 坐标）
  SendVisionData pkt{};
  auto & pkt_data = pkt.data;
  pkt_data.tracking = msg->tracking;
  pkt_data.x = msg->position.x;
  pkt_data.y = msg->position.y;
  pkt_data.z = msg->position.z;
  pkt_data.yaw = msg->yaw;
  pkt_data.vx = msg->velocity.x;
  pkt_data.vy = msg->velocity.y;
  pkt_data.vz = msg->velocity.z;
  pkt_data.v_yaw = msg->v_yaw;
  pkt_data.r1 = msg->radius_1;
  pkt_data.r2 = msg->radius_2;
  pkt_data.dz = msg->dz;

  // 2. 构造待变换的 PointStamped
  geometry_msgs::msg::PointStamped pt_in;
  pt_in.header.stamp = now();
  pt_in.header.frame_id = "odom";
  pt_in.point.x = pkt_data.x;
  pt_in.point.y = pkt_data.y;
  pt_in.point.z = pkt_data.z;

  // 3. 根据 side_flag 做 TF 变换
  if (msg->gimbal_side_flag == 0 || msg->gimbal_side_flag == 1) {
    const std::string target_frame =
      (msg->gimbal_side_flag == 0) ? "gimbal_left_link" : "gimbal_right_link";
    try {
      auto pt_out = subs_.tf_buffer->transform(pt_in, target_frame);
      pkt_data.x = pt_out.point.x;
      pkt_data.y = pt_out.point.y;
      pkt_data.z = pt_out.point.z;
    } catch (const tf2::TransformException & e) {
      RCLCPP_WARN(get_logger(), "TF transform failed: %s", e.what());
    }
  } else {
    RCLCPP_WARN(
      get_logger(), "unknown gimbal_side_flag: %d", msg->gimbal_side_flag);
  }

  // 4. 打包并发送到对应串口
  auto data = pack(pkt);
  size_t len = data.size();
  auto port = (msg->gimbal_side_flag == 0 ? left_port_ : right_port_);
  if (static_cast<ssize_t>(port->write(data.data(), len)) < 0) {
    RCLCPP_WARN(get_logger(), "数据发送失败");
  }
}

}  // namespace rm_serial_driver

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(rm_serial_driver::RMSerialDriver)