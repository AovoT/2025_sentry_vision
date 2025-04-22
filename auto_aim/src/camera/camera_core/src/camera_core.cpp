// camera_core.cpp
#include "camera_core/camera_core.hpp"

#include <cv_bridge/cv_bridge.h>

namespace camera
{

CameraCore::CameraCore(const rclcpp::NodeOptions & options)
: Node("camera_node", options), params_(this), pubs_(this), subs_(this)
{
  // Create cameras
  left_camera_ = std::make_unique<hikcamera::ImageCapturer>(
    params_.left_camera_profile, params_.left_camera_name.c_str());
  right_camera_ = std::make_unique<hikcamera::ImageCapturer>(
    params_.right_camera_profile, params_.right_camera_name.c_str());

  // Parameter callback
  params_.param_cb_handle = this->add_on_set_parameters_callback(
    std::bind(&CameraCore::set_param_cb, this, std::placeholders::_1));

  // Timers
  left_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(1), std::bind(&CameraCore::captureAndPubLeft, this));
  right_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(1), std::bind(&CameraCore::captureAndPubRight, this));
}

void CameraCore::captureAndPubLeft()
{
  auto img = left_camera_->read();
  auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", img).toImageMsg();
  pubs_.left_image_pub->publish(*msg);
}

void CameraCore::captureAndPubRight()
{
  auto img = right_camera_->read();
  auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", img).toImageMsg();
  pubs_.right_image_pub->publish(*msg);
}

rcl_interfaces::msg::SetParametersResult CameraCore::set_param_cb(
  const std::vector<rclcpp::Parameter> & params)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "Parameters updated successfully";

  try {
    for (const auto & p : params) {
      const auto & n = p.get_name();
      if (n == "left_camera.exposure_time") {
        int v = p.as_int();
        params_.left_camera_profile.exposure_time = std::chrono::duration<float, std::micro>(v);
        RCLCPP_INFO(get_logger(), "Updated left exposure to %d us", v);
      } else if (n == "left_camera.gain") {
        double v = p.as_double();
        params_.left_camera_profile.gain = std::clamp(v, 0.0, 16.0);
        RCLCPP_INFO(get_logger(), "Updated left gain to %.2f", params_.left_camera_profile.gain);
      } else if (n == "left_camera.invert") {
        params_.left_camera_profile.invert_image = p.as_bool();
        RCLCPP_INFO(
          get_logger(), "Left invert set to %s",
          params_.left_camera_profile.invert_image ? "true" : "false");
      } else if (n == "left_camera.trigger_mode") {
        params_.left_camera_profile.trigger_mode = p.as_bool();
        RCLCPP_INFO(
          get_logger(), "Left trigger set to %s",
          params_.left_camera_profile.trigger_mode ? "true" : "false");
      } else if (n == "left_camera.cam_info_url") {
        params_.right_cam_info_url = p.as_string();
        RCLCPP_INFO(get_logger(), "Left cam info url set to %s", p.as_string().c_str());
      } else if (n == "right_camera.exposure_time") {
        int v = p.as_int();
        params_.right_camera_profile.exposure_time = std::chrono::duration<float, std::micro>(v);
        RCLCPP_INFO(get_logger(), "Updated right exposure to %d us", v);
      } else if (n == "right_camera.gain") {
        double v = p.as_double();
        params_.right_camera_profile.gain = std::clamp(v, 0.0, 16.0);
        RCLCPP_INFO(get_logger(), "Updated right gain to %.2f", params_.right_camera_profile.gain);
      } else if (n == "right_camera.invert") {
        params_.right_camera_profile.invert_image = p.as_bool();
        RCLCPP_INFO(
          get_logger(), "Right invert set to %s",
          params_.right_camera_profile.invert_image ? "true" : "false");
      } else if (n == "right_camera.trigger_mode") {
        params_.right_camera_profile.trigger_mode = p.as_bool();
        RCLCPP_INFO(
          get_logger(), "Right trigger set to %s",
          params_.right_camera_profile.trigger_mode ? "true" : "false");
      } else if (n == "right_camera.cam_info_url") {
        params_.right_cam_info_url = p.as_string();
        RCLCPP_INFO(get_logger(), "Right cam info url set to %s", p.as_string().c_str());
      }
    }
  } catch (const std::exception & e) {
    result.successful = false;
    result.reason = e.what();
    RCLCPP_ERROR(get_logger(), "Failed to update parameters: %s", e.what());
  }

  return result;
}

}  // namespace camera

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(camera::CameraCore)