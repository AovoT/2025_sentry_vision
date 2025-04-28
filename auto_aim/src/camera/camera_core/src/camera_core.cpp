// camera_core.cpp
#include "camera_core/camera_core.hpp"

#include <cv_bridge/cv_bridge.h>

#include <rclcpp/rate.hpp>

namespace camera
{

CameraCore::CameraCore(const rclcpp::NodeOptions & options)
: Node("camera_node", options), params_(this), pubs_(this)
{
  // 初始化摄像头
  camera_[LEFT] = std::make_unique<hikcamera::ImageCapturer>(
    params_.left_camera_profile, params_.left_camera_name.c_str());
  camera_[RIGHT] = std::make_unique<hikcamera::ImageCapturer>(
    params_.right_camera_profile, params_.right_camera_name.c_str());

  // 初始化 CameraInfoManager
  cam_info_manager_[LEFT] =
    std::make_unique<camera_info_manager::CameraInfoManager>(
      this, params_.left_camera_name, params_.left_cam_info_url);
  cam_info_manager_[RIGHT] =
    std::make_unique<camera_info_manager::CameraInfoManager>(
      this, params_.right_camera_name, params_.right_cam_info_url);

  // 启动采集线程
  capture_thread_[LEFT] = std::thread([this] { cameraLoop(LEFT); });
  capture_thread_[RIGHT] = std::thread([this] { cameraLoop(RIGHT); });
}

CameraCore::~CameraCore()
{
  rclcpp::shutdown();
  for (int i = 0; i < DoubleEndMax; ++i) {
    if (capture_thread_[i].joinable()) {
      capture_thread_[i].join();
    }
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
  // 1. 读帧
  auto img = camera_[de]->read();

  // 2. 构造 Header
  auto hdr = std_msgs::msg::Header();
  hdr.stamp = this->now();
  hdr.frame_id =
    (de == LEFT ? "left_camera_optical_frame" : "right_camera_optical_frame");

  // 3. 转成 ROS 消息
  auto img_msg = cv_bridge::CvImage(hdr, "bgr8", img).toImageMsg();
  auto info_msg = cam_info_manager_[de]->getCameraInfo();
  info_msg.header = hdr;

  // 4. 发布
  pubs_.image_pub[de]->publish(*img_msg);
  pubs_.cam_info_pub[de]->publish(info_msg);
}

}  // namespace camera

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(camera::CameraCore)
