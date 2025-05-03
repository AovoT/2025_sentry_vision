// Copyright (C) 2022 ChenJun
// Copyright (C) 2024 Zheng Yu
// Licensed under the MIT License.

#ifndef ARMOR_PROCESSOR__PROCESSOR_NODE_HPP_
#define ARMOR_PROCESSOR__PROCESSOR_NODE_HPP_

// ROS
#include <message_filters/subscriber.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/create_timer_ros.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_listener.h>

#include <rclcpp/callback_group.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

// STD
#include <memory>
#include <string>
#include <vector>
#include <cstdint>

#include "armor_tracker/tracker.hpp"
#include "auto_aim_interfaces/msg/armors.hpp"
#include "auto_aim_interfaces/msg/target.hpp"
#include "auto_aim_interfaces/msg/tracker_info.hpp"

namespace rm_auto_aim
{

enum DoubleEnd : uint8_t {
  LEFT,
  RIGHT,
  DOUBLE_END_MAX
};

using tf2_filter = tf2_ros::MessageFilter<auto_aim_interfaces::msg::Armors>;
class ArmorTrackerNode : public rclcpp::Node
{
public:
  explicit ArmorTrackerNode(const rclcpp::NodeOptions & options);

private:
  void armorsCallback(
    const auto_aim_interfaces::msg::Armors::SharedPtr armors_ptr, DoubleEnd de);

  void publishMarkers(const auto_aim_interfaces::msg::Target & target_msg, DoubleEnd de);

  std::atomic<bool> debug = false;

  // Maximum allowable armor distance in the XOY plane
  double max_armor_distance_;

  // The time when the last message was received
  rclcpp::Time last_time_[DOUBLE_END_MAX];
  double dt_[DOUBLE_END_MAX];

  // Armor tracker
  double s2qxyz_max_, s2qxyz_min_, s2qyaw_max_, s2qyaw_min_, s2qr_;
  double r_xyz_factor, r_yaw;
  double lost_time_thres_;
  std::unique_ptr<Tracker> tracker_[DOUBLE_END_MAX];

  // Reset tracker service
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_tracker_srv_[DOUBLE_END_MAX];

  // Change target service
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr change_target_srv_[DOUBLE_END_MAX];

  // Subscriber with tf2 message_filter
  std::string target_frame_[DOUBLE_END_MAX];
  std::shared_ptr<tf2_ros::Buffer> tf2_buffer_[DOUBLE_END_MAX];
  std::shared_ptr<tf2_ros::TransformListener> tf2_listener_[DOUBLE_END_MAX];
  rclcpp::CallbackGroup::SharedPtr cb_group[DOUBLE_END_MAX];
  message_filters::Subscriber<auto_aim_interfaces::msg::Armors> armors_sub_[DOUBLE_END_MAX];
  
  std::shared_ptr<tf2_filter> tf2_filter_[DOUBLE_END_MAX];

  // Tracker info publisher
  rclcpp::Publisher<auto_aim_interfaces::msg::TrackerInfo>::SharedPtr info_pub_[DOUBLE_END_MAX];

  // Publisher
  rclcpp::Publisher<auto_aim_interfaces::msg::Target>::SharedPtr target_pub_[DOUBLE_END_MAX];

  // Visualization marker publisher
  visualization_msgs::msg::Marker position_marker_[DOUBLE_END_MAX];
  visualization_msgs::msg::Marker linear_v_marker_[DOUBLE_END_MAX];
  visualization_msgs::msg::Marker angular_v_marker_[DOUBLE_END_MAX];
  visualization_msgs::msg::Marker armor_marker_[DOUBLE_END_MAX];
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
    marker_pub_[DOUBLE_END_MAX];
};

}  // namespace rm_auto_aim

#endif  // ARMOR_PROCESSOR__PROCESSOR_NODE_HPP_