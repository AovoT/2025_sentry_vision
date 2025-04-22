// rm_serial_driver.cpp
#include "rm_serial_driver/rm_serial_node.hpp"
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace rm_serial_driver
{

RMSerialDriver::RMSerialDriver(const rclcpp::NodeOptions & opts)
: Node("rm_serial_driver", opts),
  prev_color_(0),
  params_(this),
  pubs_(this),
  subs_(this),
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
  // 通知退出
  rclcpp::shutdown();
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
      auto pkt = unpack(buf, static_cast<size_t>(n));
      handlePacket(pkt, end);
    } else if (n == 0) {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    } else {
      RCLCPP_ERROR(
        get_logger(), "%s port read error",
        end == DoubleEnd::LEFT ? "Left" : "Right");
      break;
    }
  }
}

void RMSerialDriver::handlePacket(const ReceivePacket & pkt, DoubleEnd end)
{
  // CRC 校验
  if (CRC::crc8(getPacketBytesWithoutCRC(pkt)) != pkt.crc) {
    RCLCPP_ERROR(get_logger(), "CRC mismatch");
    return;
  }
  // 重置 Tracker
  if (pkt.reset_tracker && clis_.reset_tracker_srv->service_is_ready()) {
    clis_.reset_tracker_srv->async_send_request(
      std::make_shared<std_srvs::srv::Trigger::Request>());
  }
  // 发布 TF
  tf2::Quaternion q;
  q.setRPY(pkt.roll, pkt.pitch, pkt.yaw);
  geometry_msgs::msg::TransformStamped t;
  t.header.stamp = now();
  t.header.frame_id = end == DoubleEnd::LEFT ? "left" : "right";
  
  t.transform.rotation = tf2::toMsg(q);
  pubs_.tf_broadcaster.sendTransform(t);
  // 动态参数设置
  if (prev_color_ != pkt.detect_color) {
    clis_.setParam(
      rclcpp::Parameter("detect_color", static_cast<int>(pkt.detect_color)));
    prev_color_ = pkt.detect_color;
  }
}

}  // namespace rm_serial_driver
