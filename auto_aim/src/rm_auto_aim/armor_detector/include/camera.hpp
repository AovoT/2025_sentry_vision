#pragma once
#include <cv_bridge/cv_bridge.h>

#include <camera_info_manager/camera_info_manager.hpp>
#include <hikcamera/image_capturer.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>


#include <string>
namespace camera
{

struct HikCameraParams
{
  int rate;  // hz
  std::string left_camera_name;
  std::string right_camera_name;
  std::string left_cam_info_url;
  std::string right_cam_info_url;
  hikcamera::ImageCapturer::CameraProfile left_camera_profile;
  hikcamera::ImageCapturer::CameraProfile right_camera_profile;

  explicit HikCameraParams(rclcpp::Node * node)
  {
    rcl_interfaces::msg::ParameterDescriptor desc;
    desc.read_only = true;
    rate = node->declare_parameter<int>("rate", 200, desc);

    left_camera_name =
      node->declare_parameter("left.name", "left_camera", desc);
    left_cam_info_url = node->declare_parameter(
      "left.cam_info_url", "package://rm_vision_bringup/config/camera_info_left.yaml",
      desc);
    {
      int exp_us =
        node->declare_parameter<int>("left.exposure_time", 2200, desc);
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
      "right.cam_info_url", "package://rm_vision_bringup/config/camera_info_right.yaml",
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
} // namespace camera