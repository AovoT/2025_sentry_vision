// camera_core.hpp
#pragma once

#include <hikcamera/image_capturer.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>


namespace camera
{

struct Params
{
  std::string left_camera_name;
  std::string right_camera_name;
  std::string left_cam_info_url;
  std::string right_cam_info_url;
  hikcamera::ImageCapturer::CameraProfile left_camera_profile;
  hikcamera::ImageCapturer::CameraProfile right_camera_profile;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_handle;

  explicit Params(rclcpp::Node * node)
  {
    // Left camera params
    left_camera_name = node->declare_parameter("left_camera.name", "left_camera");
    left_cam_info_url = node->declare_parameter("left_camera.cam_info_url", "package://camera_core/config/left_cam_info.yaml");
    float exp_us = node->declare_parameter("left_camera.exposure_time", 5000.0f);
    left_camera_profile.exposure_time = std::chrono::duration<float, std::micro>(exp_us);
    left_camera_profile.gain = node->declare_parameter("left_camera.gain", 8.0f);
    left_camera_profile.invert_image = node->declare_parameter("left_camera.invert", false);
    left_camera_profile.trigger_mode = node->declare_parameter("left_camera.trigger_mode", false);
    left_camera_profile.gain = std::clamp(left_camera_profile.gain, 0.0f, 16.0f);

    // Right camera params
    right_camera_name = node->declare_parameter("right_camera.name", "right_camera");
    right_cam_info_url = node->declare_parameter("right_camera.cam_info_url", "package://camera_core/config/right_cam_info.yaml");
    exp_us = node->declare_parameter("right_camera.exposure_time", 5000.0f);
    right_camera_profile.exposure_time = std::chrono::duration<float, std::micro>(exp_us);
    right_camera_profile.gain = node->declare_parameter("right_camera.gain", 8.0f);
    right_camera_profile.invert_image = node->declare_parameter("right_camera.invert", false);
    right_camera_profile.trigger_mode = node->declare_parameter("right_camera.trigger_mode", false);
    right_camera_profile.gain = std::clamp(right_camera_profile.gain, 0.0f, 16.0f);
  }
};

struct Publishers
{
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr left_image_pub;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr right_image_pub;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr left_cam_info_pub;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr right_cam_info_pub;

  explicit Publishers(rclcpp::Node * node)
  {
    left_image_pub = node->create_publisher<sensor_msgs::msg::Image>(
      "left/image_raw", rclcpp::QoS(1).reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT));
    right_image_pub = node->create_publisher<sensor_msgs::msg::Image>(
      "right/image_raw", rclcpp::QoS(1).reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT));
    left_cam_info_pub = node->create_publisher<sensor_msgs::msg::CameraInfo>("left/cam_info", 10);
    right_cam_info_pub = node->create_publisher<sensor_msgs::msg::CameraInfo>("right/cam_info", 10);
  }
};

struct Subscribers
{
  explicit Subscribers(rclcpp::Node * /*node*/)
  {
    // Add subscribers here if needed
  }
};

class CameraCore final : public rclcpp::Node
{
public:
  explicit CameraCore(const rclcpp::NodeOptions & options);

private:
  void captureAndPubLeft();
  void captureAndPubRight();
  rcl_interfaces::msg::SetParametersResult set_param_cb(
    const std::vector<rclcpp::Parameter> & params);

  Params params_;
  Publishers pubs_;
  Subscribers subs_;

  std::unique_ptr<hikcamera::ImageCapturer> left_camera_;
  std::unique_ptr<hikcamera::ImageCapturer> right_camera_;

  rclcpp::TimerBase::SharedPtr left_timer_;
  rclcpp::TimerBase::SharedPtr right_timer_;
};
}  // namespace camera