// rm_serial_driver.hpp
#pragma once

#include <tf2_ros/transform_broadcaster.h>

#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <thread>

#include "auto_aim_interfaces/msg/target.hpp"
#include "packet.hpp"
#include "serial_port.hpp"

namespace rm_serial_driver
{

enum class DoubleEnd { LEFT, RIGHT };

struct Params
{
  SerialConfig left_config;
  SerialConfig right_config;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr
    param_cb_handle;

  explicit Params(rclcpp::Node * node)
  {
    // 左串口参数
    left_config.device =
      node->declare_parameter<std::string>("left.device_name", "/dev/ttyACM0");
    left_config.baudrate =
      node->declare_parameter<int>("left.baud_rate", 115200);
    left_config.hardware_flow =
      node->declare_parameter<bool>("left.hardware_flow", false);
    left_config.parity = node->declare_parameter<bool>("left.parity", false);
    left_config.stop_bits = node->declare_parameter<int>("left.stop_bits", 1);
    left_config.timeout_ms =
      node->declare_parameter<int>("left.timeout_ms", 1000);

    // 右串口参数
    right_config.device =
      node->declare_parameter<std::string>("right.device_name", "/dev/ttyACM1");
    right_config.baudrate =
      node->declare_parameter<int>("right.baud_rate", 115200);
    right_config.hardware_flow =
      node->declare_parameter<bool>("right.hardware_flow", false);
    right_config.parity = node->declare_parameter<bool>("right.parity", false);
    right_config.stop_bits = node->declare_parameter<int>("right.stop_bits", 1);
    right_config.timeout_ms =
      node->declare_parameter<int>("right.timeout_ms", 1000);
  }
};

struct Publishers
{
  tf2_ros::TransformBroadcaster tf_broadcaster;

  explicit Publishers(rclcpp::Node * node) : tf_broadcaster(node) {}
};

struct Subscribers
{
  rclcpp::Subscription<auto_aim_interfaces::msg::Target>::SharedPtr target_sub;

  explicit Subscribers(rclcpp::Node * node)
  {
    target_sub = node->create_subscription<auto_aim_interfaces::msg::Target>(
      "target_topic", 10, [](auto) { /* 处理目标消息 */ });
  }
};

struct Clients
{
  rclcpp::Node * node_ptr;
  using ResultFuturePtr =
    std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>>;
  bool initial_set_param = false;
  rclcpp::AsyncParametersClient::SharedPtr detector_client;
  rclcpp::AsyncParametersClient::SharedPtr rune_client;
  ResultFuturePtr detector_future;
  ResultFuturePtr rune_future;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr reset_tracker_srv;

  explicit Clients(rclcpp::Node * node) : node_ptr(node)
  {
    detector_client = std::make_shared<rclcpp::AsyncParametersClient>(
      node_ptr, "armor_detector");
    rune_client = std::make_shared<rclcpp::AsyncParametersClient>(
      node_ptr, "rune_detector");
    reset_tracker_srv =
      node_ptr->create_client<std_srvs::srv::Trigger>("/reset_tracker");
  }

  void setParam(const rclcpp::Parameter & param)
  {
    setParamInternal(param, detector_client, detector_future, "Armor");
    setParamInternal(param, rune_client, rune_future, "Rune", true);
  }

private:
  void setParamInternal(
    const rclcpp::Parameter & param,
    const rclcpp::AsyncParametersClient::SharedPtr & client,
    ResultFuturePtr & future, const std::string & name, bool invert = false)
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
          for (const auto & res : results.get()) {
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
};

class RMSerialDriver : public rclcpp::Node
{
public:
  explicit RMSerialDriver(const rclcpp::NodeOptions & options);
  ~RMSerialDriver();

private:
  static constexpr int BUFFER_SIZE = 128;
  uint8_t prev_color_;
  std::thread left_thread_, right_thread_;

  Params params_;
  Publishers pubs_;
  Subscribers subs_;
  Clients clis_;

  std::shared_ptr<SerialPort> left_port_, right_port_;
  uint8_t left_buf_[BUFFER_SIZE], right_buf_[BUFFER_SIZE];

  void receiveLoop(DoubleEnd end);
  void handlePacket(const ReceivePacket & pkt, DoubleEnd end);
};
}  // namespace rm_serial_driver