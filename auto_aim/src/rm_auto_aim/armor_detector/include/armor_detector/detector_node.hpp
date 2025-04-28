// Copyright 2022 Chen Jun
// Licensed under the MIT License.

#ifndef ARMOR_DETECTOR__DETECTOR_NODE_HPP_
#define ARMOR_DETECTOR__DETECTOR_NODE_HPP_

// ROS
#include <image_transport/image_transport.hpp>
#include <image_transport/publisher.hpp>
#include <image_transport/subscriber_filter.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

// STD
#include <memory>
#include <vector>

#include "armor_detector/detector.hpp"
#include "armor_detector/pnp_solver.hpp"
#include "auto_aim_interfaces/msg/armors.hpp"


namespace rm_auto_aim
{

enum DoubleEnd : uint8_t {
  LEFT = 0,
  RIGHT = 1,
  DoubleEndMax = 2
};

class ArmorDetectorNode : public rclcpp::Node
{
public:
  ArmorDetectorNode(const rclcpp::NodeOptions & options);

private:
  
  void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr img_msg, DoubleEnd de);
  void camInfoCallback(sensor_msgs::msg::CameraInfo::ConstSharedPtr, DoubleEnd de);

  std::unique_ptr<Detector> initDetector();
  std::vector<Armor> detectArmors(const sensor_msgs::msg::Image::ConstSharedPtr & img_msg, DoubleEnd de);

  void createDebugPublishers();
  void destroyDebugPublishers();

  void publishMarkers();
  

  // Armor Detector
  std::unique_ptr<Detector> detector_;

  // Detected armors publisher
  auto_aim_interfaces::msg::Armors armors_msg_;
  rclcpp::Publisher<auto_aim_interfaces::msg::Armors>::SharedPtr armors_pub_;

  // Visualization marker publisher
  visualization_msgs::msg::Marker armor_marker_;
  visualization_msgs::msg::Marker text_marker_;
  visualization_msgs::msg::MarkerArray marker_array_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

  // Camera info part
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_sub_[DoubleEndMax];
  cv::Point2f cam_center_[DoubleEndMax];
  std::shared_ptr<sensor_msgs::msg::CameraInfo> cam_info_[DoubleEndMax];
  std::unique_ptr<PnPSolver> pnp_solver_[DoubleEndMax];

  // Image subscrpition
  // rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_[DoubleEndMax];

  // Debug information
  bool debug_;
  std::shared_ptr<rclcpp::ParameterEventHandler> debug_param_sub_;
  std::shared_ptr<rclcpp::ParameterCallbackHandle> debug_cb_handle_;
  rclcpp::Publisher<auto_aim_interfaces::msg::DebugLights>::SharedPtr lights_data_pub_;
  rclcpp::Publisher<auto_aim_interfaces::msg::DebugArmors>::SharedPtr armors_data_pub_;
  image_transport::Publisher binary_img_pub_[DoubleEndMax];
  image_transport::Publisher number_img_pub_;
  image_transport::Publisher result_img_pub_;

  //原子变量
  std::atomic<bool> m_left_find_ = false;
  std::atomic<bool> m_right_find_ = false; 

  std::shared_ptr<rclcpp::AsyncParametersClient> param_client_;

  // 用于保存 add_on_set_parameters_callback 的句柄，防止回调被 GC
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;

  // 新增：参数修改时的回调
  rcl_interfaces::msg::SetParametersResult onParametersSet(
    const std::vector<rclcpp::Parameter> & params);
};

}  // namespace rm_auto_aim

#endif  // ARMOR_DETECTOR__DETECTOR_NODE_HPP_
