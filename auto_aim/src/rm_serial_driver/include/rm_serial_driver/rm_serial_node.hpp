// rm_serial_driver.hpp
#pragma once

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "auto_aim_interfaces/msg/target.hpp"
#include "packet.hpp"
#include "serial_port.hpp"
#include "util/serial_parser.hpp"

namespace rm_serial_driver
{

class RMSerialDriver;
enum class DoubleEnd { LEFT, RIGHT };

struct Params
{
  SerialConfig left_config;
  SerialConfig right_config;
  bool is_debug;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr
    param_cb_handle;

  explicit Params(rclcpp::Node * node);
};

struct Publishers
{
  tf2_ros::TransformBroadcaster tf_broadcaster;

  explicit Publishers(rclcpp::Node * node);
};

struct Subscribers
{
  rclcpp::Subscription<auto_aim_interfaces::msg::Target>::SharedPtr target_sub;

  tf2_ros::Buffer::SharedPtr tf_buffer;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener;
  explicit Subscribers(rclcpp::Node * node, RMSerialDriver * parent);
};

struct Clients
{
  friend class RMSerialDriver;
  using ResultFuturePtr =
    std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>>;

  explicit Clients(rclcpp::Node * node);
  void setParam(const rclcpp::Parameter & param);

private:
  void setParamInternal(
    const rclcpp::Parameter & param,
    const rclcpp::AsyncParametersClient::SharedPtr & client,
    ResultFuturePtr & future, const std::string & name, bool invert = false);

  rclcpp::Node * node_ptr;
  bool initial_set_param{false};
  rclcpp::AsyncParametersClient::SharedPtr detector_client;
  rclcpp::AsyncParametersClient::SharedPtr rune_client;
  ResultFuturePtr detector_future;
  ResultFuturePtr rune_future;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr reset_tracker_srv;
};

class RMSerialDriver : public rclcpp::Node
{
public:
  explicit RMSerialDriver(const rclcpp::NodeOptions & options);
  ~RMSerialDriver();

  void handleMsg(auto_aim_interfaces::msg::Target::SharedPtr msg);

private:
  static constexpr int BUFFER_SIZE = 128;
  uint8_t prev_color_{0};
  std::thread left_thread_, right_thread_;

  Params params_;
  Publishers pubs_;
  Subscribers subs_;
  Clients clis_;

  std::shared_ptr<SerialPort> left_port_, right_port_;
  SerialParser left_serial_parser_, right_serial_parser_;
  uint8_t left_buf_[BUFFER_SIZE], right_buf_[BUFFER_SIZE];

  void receiveLoop(DoubleEnd end);
  void handlePacket(const ReceiveImuData & pkt, DoubleEnd end);
  void handlePacket(const ReceiveTargetInfoData & pkt, DoubleEnd end);
};
}  // namespace rm_serial_driver