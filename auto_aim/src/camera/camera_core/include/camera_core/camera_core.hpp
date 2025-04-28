// camera_core.hpp
#pragma once

#include <cv_bridge/cv_bridge.h>

#include <camera_info_manager/camera_info_manager.hpp>
#include <chrono>
#include <hikcamera/image_capturer.hpp>
#include <memory>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/callback_group.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <thread>

namespace camera
{
enum DoubleEnd { LEFT = 0, RIGHT = 1, DoubleEndMax = 2 };

struct Params
{
  int rate;  // hz
  std::string left_camera_name;
  std::string right_camera_name;
  std::string left_cam_info_url;
  std::string right_cam_info_url;
  hikcamera::ImageCapturer::CameraProfile left_camera_profile;
  hikcamera::ImageCapturer::CameraProfile right_camera_profile;

  explicit Params(rclcpp::Node * node)
  {
    rcl_interfaces::msg::ParameterDescriptor desc;
    desc.read_only = true;
    rate = node->declare_parameter<int>("rate", 200, desc);

    left_camera_name =
      node->declare_parameter("left.name", "left_camera", desc);
    left_cam_info_url = node->declare_parameter(
      "left.cam_info_url", "package://camera_core/config/left_cam_info.yaml",
      desc);
    {
      int exp_us =
        node->declare_parameter<int>("left.exposure_time", 5000, desc);
      left_camera_profile.exposure_time =
        std::chrono::duration<float, std::micro>(exp_us);
    }
    left_camera_profile.gain = node->declare_parameter("left.gain", 8.0f, desc);
    left_camera_profile.invert_image =
      node->declare_parameter("left.invert", false, desc);
    left_camera_profile.trigger_mode =
      node->declare_parameter("left.trigger_mode", false, desc);
    left_camera_profile.gain =
      std::clamp(left_camera_profile.gain, 0.0f, 16.0f);

    right_camera_name =
      node->declare_parameter("right.name", "right_camera", desc);
    right_cam_info_url = node->declare_parameter(
      "right.cam_info_url", "package://camera_core/config/right_cam_info.yaml",
      desc);
    {
      int exp_us =
        node->declare_parameter<int>("right.exposure_time", 5000, desc);
      right_camera_profile.exposure_time =
        std::chrono::duration<float, std::micro>(exp_us);
    }
    right_camera_profile.gain =
      node->declare_parameter("right.gain", 8.0f, desc);
    right_camera_profile.invert_image =
      node->declare_parameter("right.invert", false, desc);
    right_camera_profile.trigger_mode =
      node->declare_parameter("right.trigger_mode", false, desc);
    right_camera_profile.gain =
      std::clamp(right_camera_profile.gain, 0.0f, 16.0f);
  }
};

struct Publishers
{
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub[DoubleEndMax];
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr
    cam_info_pub[DoubleEndMax];

  explicit Publishers(rclcpp::Node * node)
  {
    auto sensor_qos = rclcpp::QoS(rclcpp::KeepLast(1))
                        .reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT)
                        .durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);

    image_pub[LEFT] = node->create_publisher<sensor_msgs::msg::Image>(
      "/left/image_raw", sensor_qos);
    image_pub[RIGHT] = node->create_publisher<sensor_msgs::msg::Image>(
      "/right/image_raw", sensor_qos);

    cam_info_pub[LEFT] = node->create_publisher<sensor_msgs::msg::CameraInfo>(
      "/left/cam_info", 10);
    cam_info_pub[RIGHT] = node->create_publisher<sensor_msgs::msg::CameraInfo>(
      "/right/cam_info", 10);
  }
};

class CameraCore final : public rclcpp::Node
{
public:
  explicit CameraCore(const rclcpp::NodeOptions & options);
  ~CameraCore();

private:
  void cameraLoop(DoubleEnd de);
  void captureAndPub(DoubleEnd de);

  Params params_;
  Publishers pubs_;

  std::unique_ptr<hikcamera::ImageCapturer> camera_[DoubleEndMax];
  std::unique_ptr<camera_info_manager::CameraInfoManager>
    cam_info_manager_[DoubleEndMax];
  std::thread capture_thread_[DoubleEndMax];
};

}  // namespace camera
