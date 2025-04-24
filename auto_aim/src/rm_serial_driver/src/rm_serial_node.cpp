#include "rm_serial_driver/rm_serial_node.hpp"

#include <tf2_ros/buffer_interface.h>

#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace rm_serial_driver
{

// Params implementation
template<typename T>
static T declare(
  rclcpp::Node * node, const std::string & name, const T & default_val)
{
  return node->declare_parameter(name, default_val);
}

Params::Params(rclcpp::Node * node)
{
  left_config.device =
    declare(node, "left.device_name", std::string("/dev/ttyACM0"));
  left_config.baudrate = declare(node, "left.baud_rate", 115200);
  left_config.hardware_flow = declare(node, "left.hardware_flow", false);
  left_config.parity = declare(node, "left.parity", false);
  left_config.stop_bits = declare(node, "left.stop_bits", 1);
  left_config.timeout_ms = declare(node, "left.timeout_ms", 1000);

  right_config.device =
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
  clis_(this)
{
  CRC::init_table();
  left_port_ = std::make_shared<SerialPort>(params_.left_config);
  right_port_ = std::make_shared<SerialPort>(params_.right_config);
  left_port_->open();
  right_port_->open();
  left_thread_ = std::thread([this] { receiveLoop(DoubleEnd::LEFT); });
  right_thread_ = std::thread([this] { receiveLoop(DoubleEnd::RIGHT); });
}

RMSerialDriver::~RMSerialDriver()
{
  if (left_thread_.joinable()) left_thread_.join();
  if (right_thread_.joinable()) right_thread_.join();
  left_port_->close();
  right_port_->close();
}

void RMSerialDriver::receiveLoop(DoubleEnd end)
{
  auto port = (end == DoubleEnd::LEFT ? left_port_ : right_port_);
  auto buf = (end == DoubleEnd::LEFT ? left_buf_ : right_buf_);
  while (rclcpp::ok()) {
    auto n = port->read(buf, BUFFER_SIZE);
    if (n > 0) {
      handlePacket(unpack(buf, n), end);
    } else if (n == 0) {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    } else {
      RCLCPP_ERROR(
        get_logger(), "%s port read error",
        (end == DoubleEnd::LEFT ? "Left" : "Right"));
      break;
    }
  }
}

void RMSerialDriver::handlePacket(const ReceivePacket & pkt, DoubleEnd end)
{
  if (CRC::crc8(getPacketBytesWithoutCRC(pkt)) != pkt.crc) {
    RCLCPP_ERROR(get_logger(), "CRC mismatch");
    return;
  }
  if (pkt.reset_tracker && clis_.reset_tracker_srv->service_is_ready()) {
    clis_.reset_tracker_srv->async_send_request(
      std::make_shared<std_srvs::srv::Trigger::Request>());
  }
  tf2::Quaternion q;
  q.setRPY(pkt.roll, pkt.pitch, pkt.yaw);
  geometry_msgs::msg::TransformStamped t;
  t.header.stamp = now();
  t.header.frame_id = "gimbal_big_link";
  t.child_frame_id =
    (end == DoubleEnd::LEFT ? "gimbal_left_link" : "gimbal_right_link");
  t.transform.rotation = tf2::toMsg(q);
  pubs_.tf_broadcaster.sendTransform(t);
  if (prev_color_ != pkt.detect_color) {
    clis_.setParam(
      rclcpp::Parameter("detect_color", static_cast<bool>(pkt.detect_color)));
    prev_color_ = pkt.detect_color;
  }
}

void RMSerialDriver::handleMsg(auto_aim_interfaces::msg::Target::SharedPtr msg)
{
  const static std::map<std::string,
    uint8_t> ID_MAP = {{"", 0},  {"outpost", 0}, {"1", 1},
                      {"2", 2}, {"3", 3},       {"4", 4},
                      {"5", 5}, {"guard", 6},   {"base", 7}};
  SendPacket packet{};
  packet.tracking = msg->tracking;
  packet.x = msg->position.x;
  packet.y = msg->position.y;
  packet.z = msg->position.z;
  packet.yaw = msg->yaw;
  packet.vx = msg->velocity.x;
  packet.vy = msg->velocity.y;
  packet.vz = msg->velocity.z;
  packet.v_yaw = msg->v_yaw;
  packet.r1 = msg->radius_1;
  packet.r2 = msg->radius_2;
  packet.dz = msg->dz;
  auto data = pack(packet);
}

void RMSerialDriver::testSendPacket()
{ /* ... */
}

}  // namespace rm_serial_driver

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(rm_serial_driver::RMSerialDriver)