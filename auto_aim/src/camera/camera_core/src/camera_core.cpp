// camera_core.cpp
#include "camera_core/camera_core.hpp"

#include <cv_bridge/cv_bridge.h>
#include <camera_info_manager/camera_info_manager.hpp>
#include <rclcpp/rate.hpp>
#include <std_msgs/msg/detail/header__struct.hpp>

namespace camera
{

CameraCore::CameraCore(const rclcpp::NodeOptions & options)
: Node("camera_node", options), params_(this), pubs_(this), subs_(this)
{
  // Create cameras
  camera_[LEFT] = std::make_unique<hikcamera::ImageCapturer>(
    params_.left_camera_profile, params_.left_camera_name.c_str());
  camera_[RIGHT] = std::make_unique<hikcamera::ImageCapturer>(
    params_.right_camera_profile, params_.right_camera_name.c_str());
  cam_info_manager_[LEFT] =
    std::make_unique<camera_info_manager::CameraInfoManager>(
      this, params_.left_camera_name, params_.left_cam_info_url
  );
  cam_info_manager_[RIGHT] =
    std::make_unique<camera_info_manager::CameraInfoManager>(
      this, params_.right_camera_name, params_.right_cam_info_url
  );

  capture_thread_[LEFT] = std::thread([this] { this->cameraLoop(LEFT); });
  capture_thread_[RIGHT] = std::thread([this] { this->cameraLoop(RIGHT); });
}
CameraCore::~CameraCore()
{
  rclcpp::shutdown();
  if (capture_thread_[LEFT].joinable()) {
    capture_thread_[LEFT].join();
  }
  if (capture_thread_[RIGHT].joinable()) {
    capture_thread_[RIGHT].join();
  }
}

void CameraCore::cameraLoop(DoubleEnd de)
{
  rclcpp::WallRate rate(params_.rate);
  while (rclcpp::ok()) {
    captureAndPub(de);
    rate.sleep();
  }
}

void CameraCore::captureAndPub(DoubleEnd de)
{
  auto img = camera_[de]->read();
  std_msgs::msg::Header hdr;
  hdr.stamp = this->now();
  std::string de_str = de == LEFT ? "left" : "right";
  hdr.frame_id = de_str + "_camera_optical_frame";
  auto img_msg =
    cv_bridge::CvImage(hdr, "bgr8", img).toImageMsg();
  auto info_msg = cam_info_manager_[de]->getCameraInfo();
  info_msg.header = hdr;
  pubs_.image[de].publish(img_msg);
  pubs_.cam_info[de]->publish(info_msg);
}

}  // namespace camera

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(camera::CameraCore)