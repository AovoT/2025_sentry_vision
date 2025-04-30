#include "rm_serial_driver/rm_serial_node.hpp"

#include <tf2_ros/buffer_interface.h>
#include <tf2_ros/transform_listener.h>

#include <auto_aim_interfaces/msg/detail/tracker_info__struct.hpp>
#include <exception>
#include <rclcpp/logging.hpp>
#include <rclcpp/qos.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2/time.hpp>
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
  config[LEFT].device_name =
    declare(node, "left.device_name", std::string("/dev/ttyACM0"));
  config[LEFT].baudrate = declare(node, "left.baud_rate", 115200);
  config[LEFT].hardware_flow = declare(node, "left.hardware_flow", false);
  config[LEFT].parity = declare(node, "left.parity", false);
  config[LEFT].stop_bits = declare(node, "left.stop_bits", 1);
  config[LEFT].timeout_ms = declare(node, "left.timeout_ms", 1000);

  config[RIGHT].device_name =
    declare(node, "right.device_name", std::string("/dev/ttyACM1"));
  config[RIGHT].baudrate = declare(node, "right.baud_rate", 115200);
  config[RIGHT].hardware_flow = declare(node, "right.hardware_flow", false);
  config[RIGHT].parity = declare(node, "right.parity", false);
  config[RIGHT].stop_bits = declare(node, "right.stop_bits", 1);
  config[RIGHT].timeout_ms = declare(node, "right.timeout_ms", 1000);

  is_debug = declare(node, "debug", false);
}

// Publishers implementation
Publishers::Publishers(rclcpp::Node * node) : tf_broadcaster(node) {}

// Subscribers implementation
Subscribers::Subscribers(rclcpp::Node * node, RMSerialDriver * parent)
{
  tf_buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);
  target_sub = node->create_subscription<auto_aim_interfaces::msg::Target>(
    "/tracker/target", rclcpp::SensorDataQoS(),
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
  clis_(this)
{
  serial_parser_[LEFT] = std::make_unique<SerialParser>(this->get_logger());
  serial_parser_[RIGHT] = std::make_unique<SerialParser>(this->get_logger());
  serial_parser_[LEFT]->registerHandler<ReceiveImuData>(
    0x01, [this](auto & pkt) { handlePacket(pkt, DoubleEnd::LEFT); });
  serial_parser_[LEFT]->registerHandler<ReceiveTargetInfoData>(
    0x09, [this](auto & pkt) { handlePacket(pkt, DoubleEnd::LEFT); });
  serial_parser_[RIGHT]->registerHandler<ReceiveImuData>(
    0x01, [this](auto & pkt) { handlePacket(pkt, DoubleEnd::RIGHT); });
  serial_parser_[RIGHT]->registerHandler<ReceiveTargetInfoData>(
    0x09, [this](auto & pkt) { handlePacket(pkt, DoubleEnd::RIGHT); });
  port_[LEFT] = std::make_unique<SerialPort>(params_.config[LEFT]);
  port_[RIGHT] = std::make_unique<SerialPort>(params_.config[RIGHT]);
  this->openPortWithRetry(LEFT);
  this->openPortWithRetry(RIGHT);
  thread_[LEFT] = std::thread([this] { receiveLoop(DoubleEnd::LEFT); });
  thread_[RIGHT] = std::thread([this] { receiveLoop(DoubleEnd::RIGHT); });
}

// === 新增: 打开串口并自动重连 ===
void RMSerialDriver::openPortWithRetry(DoubleEnd de)
{
  const char * which = (de == LEFT ? "left" : "right");
  while (rclcpp::ok()) {
    try {
      port_[de]->open();
      RCLCPP_INFO(get_logger(), "%s serial port opened", which);
      return;  // 成功 → 直接返回
    } catch (const std::exception & e) {
      RCLCPP_WARN(
        get_logger(), "%s serial open failed: %s; retry in 1 s …", which,
        e.what());
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }
  }
}

RMSerialDriver::~RMSerialDriver()
{
  rclcpp::shutdown();
  if (thread_[LEFT].joinable()) thread_[LEFT].join();
  if (thread_[RIGHT].joinable()) thread_[RIGHT].join();
}

void RMSerialDriver::receiveLoop(DoubleEnd de)
{
  std::string end_str = de == DoubleEnd::LEFT ? "left" : "right";

  while (rclcpp::ok()) {
    ssize_t n = port_[de]->read(data_buf_[de], BUFFER_SIZE);
    if (n > 0) {
      serial_parser_[de]->feed(data_buf_[de], static_cast<size_t>(n));
    } else if (n == 0) {
      // EOF: 对端关闭，设备可能掉了
      RCLCPP_WARN(
        get_logger(), "%s port returned 0 (EOF), 尝试重连…", end_str.c_str());
      port_[de]->close();
      openPortWithRetry(de);
    } else if (errno == EAGAIN || errno == EWOULDBLOCK) {
      // 非阻塞模式下没数据，跳过
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    } else {
      RCLCPP_ERROR(
        get_logger(), "%s port read error (%s), 尝试重连…", end_str.c_str(),
        std::strerror(errno));
      port_[de]->close();
      openPortWithRetry(de);
    }
  }
}

void RMSerialDriver::handlePacket(const ReceiveImuData & pkt, DoubleEnd end)
{
  tf2::Quaternion q;
  q.setRPY(pkt.data.roll, pkt.data.pitch, pkt.data.yaw);
  q.normalize();
  if (
    std::isnan(q.x()) || std::isnan(q.y()) || std::isnan(q.z()) ||
    std::isnan(q.w())) {
    RCLCPP_WARN(get_logger(), "roll, pitch or yaw is invalid nan");
    return;
  }
  geometry_msgs::msg::TransformStamped t;
  t.header.stamp = now();
  t.header.frame_id =
    (end == DoubleEnd::LEFT ? "gimbal_left_link_offset"
                            : "gimbal_right_link_offset");
  t.child_frame_id =
    (end == DoubleEnd::LEFT ? "gimbal_left_link" : "gimbal_right_link");
  t.transform.rotation = tf2::toMsg(q);
  pubs_.tf_broadcaster.sendTransform(t);
}

void RMSerialDriver::handleMsg(auto_aim_interfaces::msg::Target::SharedPtr msg)
{
  std::map<std::string, int> name_to_id = {
    {"", 0},  {"1", 1},       {"2", 2},     {"3", 3},    {"4", 4},
    {"5", 5}, {"outpost", 6}, {"guard", 7}, {"base", 8},
  };

  bool is_valid = msg->header.frame_id == "odom";
  if (!is_valid) {
    RCLCPP_WARN(
      get_logger(), "invalid target frame id : %s",
      msg->header.frame_id.c_str());
    return;
  }

  // ========== 准备共用的基础数据 ==========
  SendVisionData base_pkt{};
  base_pkt.frame_header.sof = 0x5A;
  base_pkt.frame_header.id = 0x03;
  base_pkt.time_stamp = std::chrono::duration_cast<std::chrono::milliseconds>(
                          std::chrono::steady_clock::now().time_since_epoch())
                          .count();

  auto & d = base_pkt.data;
  d.tracking = msg->tracking;
  d.armors_num = msg->armors_num;
  d.id = name_to_id[msg->id];
  d.yaw = msg->yaw;
  d.vx = msg->velocity.x;
  d.vy = msg->velocity.y;
  d.vz = msg->velocity.z;
  d.v_yaw = msg->v_yaw;
  d.r1 = msg->radius_1;
  d.r2 = msg->radius_2;
  d.dz = msg->dz;

  // ========== 准备待转换点 ==========
  geometry_msgs::msg::PointStamped pt_in;
  pt_in.header = msg->header;
  pt_in.point.x = msg->position.x;
  pt_in.point.y = msg->position.y;
  pt_in.point.z = msg->position.z;
  pt_in.header.stamp = rclcpp::Time(0);  // 使用最新 TF

  // ========== 转换 & 发送左右各自的包 ==========
  const std::array<DoubleEnd, 2> ends = {LEFT, RIGHT};
  const std::array<const char *, 2> frames = {
    "gimbal_left_link_offset", "gimbal_right_link_offset"};

  for (int i = 0; i < 2; ++i) {
    DoubleEnd de = ends[i];
    const char * frame = frames[i];

    geometry_msgs::msg::PointStamped pt_out;
    try {
      pt_out =
        subs_.tf_buffer->transform(pt_in, frame, tf2::durationFromSec(0.1));
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(
        get_logger(), "TF transform to %s failed: %s", frame, ex.what());
      continue;
    }

    SendVisionData pkt = base_pkt;
    pkt.data.x = pt_out.point.x;
    pkt.data.y = pt_out.point.y;
    pkt.data.z = pt_out.point.z;

    auto buf = pack(pkt);
    if (params_.is_debug) {
      RCLCPP_INFO(
        get_logger(), "%s port debug info", de == LEFT ? "left" : "right");
      print(pkt);
    }

    if (static_cast<ssize_t>(port_[de]->write(buf.data(), buf.size())) < 0) {
      RCLCPP_WARN(
        get_logger(), "Failed to send vision data to %s port",
        de == LEFT ? "left" : "right");
    }
  }
}

void RMSerialDriver::handlePacket(
  const ReceiveTargetInfoData & pkt, DoubleEnd de)
{
  (void)pkt;
  (void)de;
}
}  // namespace rm_serial_driver

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(rm_serial_driver::RMSerialDriver)