/*
 * Refactored ArmorDetectorNode with optimized QoS for image subs,
 * dynamic parameter callback handling,
 * multi-threaded CallbackGroups for parallel image callbacks,
 * including publishArmorsAndMarkers implementation.
 */

// detector_node.hpp
#ifndef ARMOR_DETECTOR__DETECTOR_NODE_HPP_
#define ARMOR_DETECTOR__DETECTOR_NODE_HPP_

#include <atomic>
#include <image_transport/image_transport.hpp>
#include <memory>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "armor_detector/detector.hpp"
#include "armor_detector/pnp_solver.hpp"
#include "auto_aim_interfaces/msg/armors.hpp"

namespace rm_auto_aim
{

enum DoubleEnd : uint8_t { LEFT = 0, RIGHT = 1, DoubleEndMax = 2 };

struct DebugPublishers
{
  rclcpp::Publisher<auto_aim_interfaces::msg::DebugLights>::SharedPtr lights;
  rclcpp::Publisher<auto_aim_interfaces::msg::DebugArmors>::SharedPtr armors;
  image_transport::Publisher binary_left;
  image_transport::Publisher binary_right;
  image_transport::Publisher numbers;
  image_transport::Publisher result;
};

class ArmorDetectorNode : public rclcpp::Node
{
public:
  explicit ArmorDetectorNode(const rclcpp::NodeOptions & options);

private:
  void imageCallback(
    const sensor_msgs::msg::Image::ConstSharedPtr img, DoubleEnd de);
  void camInfoCallback(
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr info, DoubleEnd de);
  bool shouldDetect(DoubleEnd de);  // now non-const to allow state update

  std::unique_ptr<Detector> initDetector();
  std::vector<Armor> detectArmors(
    const sensor_msgs::msg::Image::ConstSharedPtr & img, DoubleEnd de);
  void publishArmorsAndMarkers(
    const std::vector<Armor> & armors,
    const sensor_msgs::msg::Image::ConstSharedPtr & img, DoubleEnd de);
  rcl_interfaces::msg::SetParametersResult onParametersSet(
    const std::vector<rclcpp::Parameter> & params);

  // Publishers & Subscribers
  rclcpp::Publisher<auto_aim_interfaces::msg::Armors>::SharedPtr armors_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
    marker_pub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr
    cam_info_sub_[DoubleEndMax];
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr
    img_sub_[DoubleEndMax];
  rclcpp::CallbackGroup::SharedPtr img_cb_group_[DoubleEndMax];

  // Detection state
  static constexpr int kMissTolerance = 3;
  bool last_find_[DoubleEndMax] = {false, false};
  int miss_count_[DoubleEndMax] = {kMissTolerance, kMissTolerance};
  int owner_ = -1;
  std::mutex find_mutex_;  // non-const methods

  // Detector & PnP
  std::unique_ptr<Detector> detector_;
  std::shared_ptr<PnPSolver> pnp_solver_[DoubleEndMax];

  // Camera info
  cv::Point2d cam_center_[DoubleEndMax];

  // Debug
  std::atomic<bool> debug_enabled_{false};
  std::mutex debug_mutex_;
  std::shared_ptr<DebugPublishers> debug_pubs_;
  void createDebugPublishers();
  void destroyDebugPublishers();

  // Parameter client
  std::shared_ptr<rclcpp::AsyncParametersClient> param_client_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr
    param_cb_handle_;
};

}  // namespace rm_auto_aim
#endif  // ARMOR_DETECTOR__DETECTOR_NODE_HPP_
