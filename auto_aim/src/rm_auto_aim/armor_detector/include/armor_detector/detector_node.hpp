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
#include <camera_info_manager/camera_info_manager.hpp>
#include <hikcamera/image_capturer.hpp>
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
#include "camera.hpp"

namespace rm_auto_aim
{

enum DoubleEnd : uint8_t { LEFT = 0, RIGHT = 1, DOUBLE_END_MAX = 2 };

struct DebugPublishers
{
  rclcpp::Publisher<auto_aim_interfaces::msg::DebugLights>::SharedPtr lights;
  rclcpp::Publisher<auto_aim_interfaces::msg::DebugArmors>::SharedPtr armors;
  image_transport::Publisher binary[DOUBLE_END_MAX];
  image_transport::Publisher numbers;
  image_transport::Publisher result;
  image_transport::Publisher img_raw[DOUBLE_END_MAX];
};

class ArmorDetectorNode : public rclcpp::Node
{
public:
  explicit ArmorDetectorNode(const rclcpp::NodeOptions & options);
  ~ArmorDetectorNode() override;

private:
  void mainLoop(DoubleEnd de);
  void handleOnce(DoubleEnd de);
  bool shouldDetect(DoubleEnd de);  // now non-const to allow state update

  void initDetectors();
  std::vector<Armor> detectArmors(const cv::Mat & img, DoubleEnd de);
  void publishArmorsAndMarkers(const std::vector<Armor> & armors, DoubleEnd de);
  rcl_interfaces::msg::SetParametersResult onParametersSet(
    const std::vector<rclcpp::Parameter> & params);

  // Publishers & Subscribers
  rclcpp::Publisher<auto_aim_interfaces::msg::Armors>::SharedPtr armors_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
    marker_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr
    img_sub_[DOUBLE_END_MAX];

  // Detection state
  static constexpr int K_MISS_TOLERANCE = 10;
  bool last_find_[DOUBLE_END_MAX] = {false, false};
  int miss_count_[DOUBLE_END_MAX] = {K_MISS_TOLERANCE, K_MISS_TOLERANCE};
  int owner_ = -1;
  std::mutex find_mutex_;  // non-const methods
  // Camera
  std::unique_ptr<hikcamera::ImageCapturer> img_capture_[DOUBLE_END_MAX];
  camera::HikCameraParams hik_camera_params_;
  // Detector & PnP
  std::unique_ptr<Detector> detector_[DOUBLE_END_MAX];
  std::shared_ptr<PnPSolver> pnp_solver_[DOUBLE_END_MAX];
  camera_info_manager::CameraInfoManager cam_info_manager_;

  // thread
  std::unique_ptr<std::thread> loop_thread[DOUBLE_END_MAX];

  // Camera info
  cv::Point2d cam_center_[DOUBLE_END_MAX];

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
