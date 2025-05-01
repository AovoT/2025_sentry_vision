// Copyright (C) 2022 ChenJun
// Copyright (C) 2024 Zheng Yu
// Licensed under the MIT License.

#include "armor_tracker/tracker_node.hpp"

// STD
#include <cmath>
#include <memory>
#include <vector>

namespace rm_auto_aim
{
ArmorTrackerNode::ArmorTrackerNode(const rclcpp::NodeOptions & options)
: Node("armor_tracker", options)
{
  RCLCPP_INFO(this->get_logger(), "Starting TrackerNode!");

  // Maximum allowable armor distance in the XOY plane
  max_armor_distance_ = this->declare_parameter("max_armor_distance", 10.0);

  // Tracker
  double max_match_distance =
    this->declare_parameter("tracker.max_match_distance", 0.15);
  double max_match_yaw_diff =
    this->declare_parameter("tracker.max_match_yaw_diff", 1.0);
  tracker_[LEFT] =
    std::make_unique<Tracker>(max_match_distance, max_match_yaw_diff);
  tracker_[RIGHT] =
    std::make_unique<Tracker>(max_match_distance, max_match_yaw_diff);
  int tracking_thres = this->declare_parameter("tracker.tracking_thres", 5);
  tracker_[LEFT]->tracking_thres = tracking_thres;
  tracker_[RIGHT]->tracking_thres = tracking_thres;
  lost_time_thres_ = this->declare_parameter("tracker.lost_time_thres", 0.3);

  // EKF
  // xa = x_armor, xc = x_robot_center
  // state: xc, v_xc, yc, v_yc, za, v_za, yaw, v_yaw, r
  // measurement: xa, ya, za, yaw
  // f - Process function
  auto f_left = [this](const Eigen::VectorXd & x) {
    Eigen::VectorXd x_new = x;
    x_new(0) += x(1) * dt_[LEFT];
    x_new(2) += x(3) * dt_[LEFT];
    x_new(4) += x(5) * dt_[LEFT];
    x_new(6) += x(7) * dt_[LEFT];
    return x_new;
  };
  auto f_right = [this](const Eigen::VectorXd & x) {
    Eigen::VectorXd x_new = x;
    x_new(0) += x(1) * dt_[RIGHT];
    x_new(2) += x(3) * dt_[RIGHT];
    x_new(4) += x(5) * dt_[RIGHT];
    x_new(6) += x(7) * dt_[RIGHT];
    return x_new;
  };
  // J_f - Jacobian of process function
  auto j_f_left = [this](const Eigen::VectorXd &) {
    Eigen::MatrixXd f(9, 9);
    // clang-format off
    f <<  1,   dt_[LEFT], 0,   0,   0,   0,   0,   0,   0,
          0,   1,   0,   0,   0,   0,   0,   0,   0,
          0,   0,   1,   dt_[LEFT], 0,   0,   0,   0,   0, 
          0,   0,   0,   1,   0,   0,   0,   0,   0,
          0,   0,   0,   0,   1,   dt_[LEFT], 0,   0,   0,
          0,   0,   0,   0,   0,   1,   0,   0,   0,
          0,   0,   0,   0,   0,   0,   1,   dt_[LEFT], 0,
          0,   0,   0,   0,   0,   0,   0,   1,   0,
          0,   0,   0,   0,   0,   0,   0,   0,   1;
    // clang-format on
    return f;
  };
  auto j_f_right = [this](const Eigen::VectorXd &) {
    Eigen::MatrixXd f(9, 9);
    // clang-format off
    f <<  1,   dt_[RIGHT], 0,   0,   0,   0,   0,   0,   0,
          0,   1,   0,   0,   0,   0,   0,   0,   0,
          0,   0,   1,   dt_[RIGHT], 0,   0,   0,   0,   0, 
          0,   0,   0,   1,   0,   0,   0,   0,   0,
          0,   0,   0,   0,   1,   dt_[RIGHT], 0,   0,   0,
          0,   0,   0,   0,   0,   1,   0,   0,   0,
          0,   0,   0,   0,   0,   0,   1,   dt_[RIGHT], 0,
          0,   0,   0,   0,   0,   0,   0,   1,   0,
          0,   0,   0,   0,   0,   0,   0,   0,   1;
    // clang-format on
    return f;
  };
  // h - Observation function
  auto h = [](const Eigen::VectorXd & x) {
    Eigen::VectorXd z(4);
    double xc = x(0), yc = x(2), yaw = x(6), r = x(8);
    z(0) = xc - r * cos(yaw);  // xa
    z(1) = yc - r * sin(yaw);  // ya
    z(2) = x(4);               // za
    z(3) = x(6);               // yaw
    return z;
  };
  // J_h - Jacobian of observation function
  auto j_h = [](const Eigen::VectorXd & x) {
    Eigen::MatrixXd h(4, 9);
    double yaw = x(6), r = x(8);
    // clang-format off
    //    xc   v_xc yc   v_yc za   v_za yaw         v_yaw r
    h <<  1,   0,   0,   0,   0,   0,   r*sin(yaw), 0,   -cos(yaw),
          0,   0,   1,   0,   0,   0,   -r*cos(yaw),0,   -sin(yaw),
          0,   0,   0,   0,   1,   0,   0,          0,   0,
          0,   0,   0,   0,   0,   0,   1,          0,   0;
    // clang-format on
    return h;
  };
  // update_Q - process noise covariance matrix
  s2qxyz_max_ = declare_parameter("ekf.sigma2_q_xyz_max", 0.1);
  s2qxyz_min_ = declare_parameter("ekf.sigma2_q_xyz_min", 0.05);
  s2qyaw_max_ = declare_parameter("ekf.sigma2_q_yaw_max", 10.0);
  s2qyaw_min_ = declare_parameter("ekf.sigma2_q_yaw_min", 5.0);
  s2qr_ = declare_parameter("ekf.sigma2_q_r", 80.0);
  auto u_q_left = [this](const Eigen::VectorXd & x_p) {
    double vx = x_p(1), vy = x_p(3), v_yaw = x_p(7);
    double dx = pow(pow(vx, 2) + pow(vy, 2), 0.5);
    double dy = abs(v_yaw);
    Eigen::MatrixXd q(9, 9);
    double x, y;
    x = exp(-dy) * (s2qxyz_max_ - s2qxyz_min_) + s2qxyz_min_;
    y = exp(-dx) * (s2qyaw_max_ - s2qyaw_min_) + s2qyaw_min_;
    double t = dt_[LEFT], r = s2qr_;
    double q_x_x = pow(t, 4) / 4 * x, q_x_vx = pow(t, 3) / 2 * x,
           q_vx_vx = pow(t, 2) * x;
    double q_y_y = pow(t, 4) / 4 * y, q_y_vy = pow(t, 3) / 2 * x,
           q_vy_vy = pow(t, 2) * y;
    double q_r = pow(t, 4) / 4 * r;
    // clang-format off
    //    xc      v_xc    yc      v_yc    za      v_za    yaw     v_yaw   r
    q <<  q_x_x,  q_x_vx, 0,      0,      0,      0,      0,      0,      0,
          q_x_vx, q_vx_vx,0,      0,      0,      0,      0,      0,      0,
          0,      0,      q_x_x,  q_x_vx, 0,      0,      0,      0,      0,
          0,      0,      q_x_vx, q_vx_vx,0,      0,      0,      0,      0,
          0,      0,      0,      0,      q_x_x,  q_x_vx, 0,      0,      0,
          0,      0,      0,      0,      q_x_vx, q_vx_vx,0,      0,      0,
          0,      0,      0,      0,      0,      0,      q_y_y,  q_y_vy, 0,
          0,      0,      0,      0,      0,      0,      q_y_vy, q_vy_vy,0,
          0,      0,      0,      0,      0,      0,      0,      0,      q_r;
    // clang-format on
    return q;
  };
  // update_R - measurement noise covariance matrix
  r_xyz_factor = declare_parameter("ekf.r_xyz_factor", 0.05);
  r_yaw = declare_parameter("ekf.r_yaw", 0.02);
  auto u_r = [this](const Eigen::VectorXd & z) {
    Eigen::DiagonalMatrix<double, 4> r;
    double x = r_xyz_factor;
    r.diagonal() << abs(x * z[0]), abs(x * z[1]), abs(x * z[2]), r_yaw;
    return r;
  };
  // P - error estimate covariance matrix
  Eigen::DiagonalMatrix<double, 9> p0;
  p0.setIdentity();
  tracker_[LEFT]->ekf =
    ExtendedKalmanFilter{f_left, h, j_f_left, j_h, u_q_left, u_r, p0};
  tracker_[RIGHT]->ekf =
    ExtendedKalmanFilter{f_right, h, j_f_right, j_h, u_q_left, u_r, p0};

  // Reset tracker service
  using std::placeholders::_1;
  using std::placeholders::_2;
  using std::placeholders::_3;
  reset_tracker_srv_[LEFT] = this->create_service<std_srvs::srv::Trigger>(
    "/tracker/left/reset",
    [this](
      const std_srvs::srv::Trigger::Request::SharedPtr,
      std_srvs::srv::Trigger::Response::SharedPtr response) {
      tracker_[LEFT]->tracker_state = Tracker::LOST;
      response->success = true;
      RCLCPP_INFO(this->get_logger(), "Left Tracker reset!");
      return;
    });
  reset_tracker_srv_[RIGHT] = this->create_service<std_srvs::srv::Trigger>(
    "/tracker/left/reset",
    [this](
      const std_srvs::srv::Trigger::Request::SharedPtr,
      std_srvs::srv::Trigger::Response::SharedPtr response) {
      tracker_[RIGHT]->tracker_state = Tracker::LOST;
      response->success = true;
      RCLCPP_INFO(this->get_logger(), "Right Tracker reset!");
      return;
    });

  // Change target service
  change_target_srv_[LEFT] = this->create_service<std_srvs::srv::Trigger>(
    "/tracker/left/change",
    [this](
      const std_srvs::srv::Trigger::Request::SharedPtr,
      std_srvs::srv::Trigger::Response::SharedPtr response) {
      tracker_[LEFT]->tracker_state = Tracker::CHANGE_TARGET;
      response->success = true;
      RCLCPP_INFO(this->get_logger(), "Target change!");
      return;
    });
  change_target_srv_[RIGHT] = this->create_service<std_srvs::srv::Trigger>(
    "/tracker/right/change",
    [this](
      const std_srvs::srv::Trigger::Request::SharedPtr,
      std_srvs::srv::Trigger::Response::SharedPtr response) {
      tracker_[RIGHT]->tracker_state = Tracker::CHANGE_TARGET;
      response->success = true;
      RCLCPP_INFO(this->get_logger(), "Target change!");
      return;
    });

  // Subscriber with tf2 message_filter
  // tf2 relevant
  tf2_buffer_[LEFT] = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf2_buffer_[RIGHT] = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  // Create the timer interface before call to waitForTransform,
  // to avoid a tf2_ros::CreateTimerInterfaceException exception
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
    this->get_node_base_interface(), this->get_node_timers_interface());
  tf2_buffer_[LEFT]->setCreateTimerInterface(timer_interface);
  tf2_buffer_[RIGHT]->setCreateTimerInterface(timer_interface);
  tf2_listener_[LEFT] =
    std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_[LEFT]);
  tf2_listener_[RIGHT] =
    std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_[RIGHT]);
  // subscriber and filter
  armors_sub_[LEFT].subscribe(
    this, "/detector/left/armors", rmw_qos_profile_sensor_data);
  armors_sub_[RIGHT].subscribe(
    this, "/detector/right/armors", rmw_qos_profile_sensor_data);
  target_frame_[LEFT] = this->declare_parameter("left.target_frame", "odom");
  target_frame_[RIGHT] = this->declare_parameter("right.target_frame", "odom");
  tf2_filter_[LEFT] = std::make_shared<tf2_filter>(
    armors_sub_[LEFT], *tf2_buffer_[LEFT], target_frame_[LEFT], 10,
    this->get_node_logging_interface(), this->get_node_clock_interface(),
    std::chrono::duration<int>(1));
  tf2_filter_[RIGHT] = std::make_shared<tf2_filter>(
    armors_sub_[RIGHT], *tf2_buffer_[RIGHT], target_frame_[RIGHT], 10,
    this->get_node_logging_interface(), this->get_node_clock_interface(),
    std::chrono::duration<int>(1));
  // Register a callback with tf2_ros::MessageFilter to be called when transforms are available
  tf2_filter_[LEFT]->registerCallback(
    [this](std::shared_ptr<const auto_aim_interfaces::msg::Armors> armors_ptr)
      -> void {
      return this->armorsCallback(
        std::const_pointer_cast<auto_aim_interfaces::msg::Armors>(armors_ptr),
        LEFT);
    });
  tf2_filter_[RIGHT]->registerCallback(
    [this](std::shared_ptr<const auto_aim_interfaces::msg::Armors> armors_ptr)
      -> void {
      return this->armorsCallback(
        std::const_pointer_cast<auto_aim_interfaces::msg::Armors>(armors_ptr),
        RIGHT);
    });

  // Measurement publisher (for debug usage)
  info_pub_[LEFT] =
    this->create_publisher<auto_aim_interfaces::msg::TrackerInfo>(
      "/tracker/left/info", 10);
  info_pub_[RIGHT] =
    this->create_publisher<auto_aim_interfaces::msg::TrackerInfo>(
      "/tracker/right/info", 10);

  // Publisher
  target_pub_[LEFT] = this->create_publisher<auto_aim_interfaces::msg::Target>(
    "/tracker/left/target", rclcpp::SensorDataQoS());
  target_pub_[RIGHT] = this->create_publisher<auto_aim_interfaces::msg::Target>(
    "/tracker/right/target", rclcpp::SensorDataQoS());

  // Visualization Marker Publisher
  // See http://wiki.ros.org/rviz/DisplayTypes/Marker
  position_marker_[LEFT].ns = "position";
  position_marker_[LEFT].type = visualization_msgs::msg::Marker::SPHERE;
  position_marker_[LEFT].scale.x = position_marker_[LEFT].scale.y =
    position_marker_[LEFT].scale.z = 0.1;
  position_marker_[LEFT].color.a = 1.0;
  position_marker_[LEFT].color.g = 1.0;
  position_marker_[RIGHT] = position_marker_[LEFT];
  linear_v_marker_[LEFT].type = visualization_msgs::msg::Marker::ARROW;
  linear_v_marker_[LEFT].ns = "linear_v";
  linear_v_marker_[LEFT].scale.x = 0.03;
  linear_v_marker_[LEFT].scale.y = 0.05;
  linear_v_marker_[LEFT].color.a = 1.0;
  linear_v_marker_[LEFT].color.r = 1.0;
  linear_v_marker_[LEFT].color.g = 1.0;
  linear_v_marker_[RIGHT] = linear_v_marker_[LEFT];
  angular_v_marker_[LEFT].type = visualization_msgs::msg::Marker::ARROW;
  angular_v_marker_[LEFT].ns = "angular_v";
  angular_v_marker_[LEFT].scale.x = 0.03;
  angular_v_marker_[LEFT].scale.y = 0.05;
  angular_v_marker_[LEFT].color.a = 1.0;
  angular_v_marker_[LEFT].color.b = 1.0;
  angular_v_marker_[LEFT].color.g = 1.0;
  angular_v_marker_[RIGHT] = angular_v_marker_[LEFT];
  armor_marker_[LEFT].ns = "armors";
  armor_marker_[LEFT].type = visualization_msgs::msg::Marker::CUBE;
  armor_marker_[LEFT].scale.x = 0.03;
  armor_marker_[LEFT].scale.z = 0.125;
  armor_marker_[LEFT].color.a = 1.0;
  armor_marker_[LEFT].color.r = 1.0;
  armor_marker_[RIGHT] = armor_marker_[LEFT];
  marker_pub_[LEFT] =
    this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "/tracker/left/marker", 10);
  marker_pub_[RIGHT] =
    this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "/tracker/right/marker", 10);
}

void ArmorTrackerNode::armorsCallback(
  const auto_aim_interfaces::msg::Armors::SharedPtr armors_msg, DoubleEnd de)
{
  // Tranform armor position from image frame to world coordinate
  for (auto & armor : armors_msg->armors) {
    geometry_msgs::msg::PoseStamped ps;
    ps.header = armors_msg->header;
    ps.pose = armor.pose;
    try {
      armor.pose = tf2_buffer_[de]->transform(ps, target_frame_[de]).pose;
    } catch (const tf2::ExtrapolationException & ex) {
      RCLCPP_ERROR(get_logger(), "Error while transforming %s", ex.what());
      return;
    }
  }

  // Filter abnormal armors
  armors_msg->armors.erase(
    std::remove_if(
      armors_msg->armors.begin(), armors_msg->armors.end(),
      [this](const auto_aim_interfaces::msg::Armor & armor) {
        return abs(armor.pose.position.z) > 1.2 ||
               Eigen::Vector2d(armor.pose.position.x, armor.pose.position.y)
                   .norm() > max_armor_distance_;
      }),
    armors_msg->armors.end());

  // Init message
  auto_aim_interfaces::msg::TrackerInfo info_msg;
  auto_aim_interfaces::msg::Target target_msg;
  rclcpp::Time time = armors_msg->header.stamp;
  target_msg.header.stamp = time;
  target_msg.header.frame_id = target_frame_[de];

  // Update tracker
  if (tracker_[de]->tracker_state == Tracker::LOST) {
    tracker_[de]->init(armors_msg);
    target_msg.tracking = false;
  } else {
    dt_[de] = (time - last_time_[de]).seconds();
    tracker_[de]->lost_thres = static_cast<int>(lost_time_thres_ / dt_[de]);

    tracker_[de]->update(armors_msg);

    // Publish Info
    info_msg.position_diff = tracker_[de]->info_position_diff;
    info_msg.yaw_diff = tracker_[de]->info_yaw_diff;
    info_msg.position.x = tracker_[de]->measurement(0);
    info_msg.position.y = tracker_[de]->measurement(1);
    info_msg.position.z = tracker_[de]->measurement(2);
    info_msg.yaw = tracker_[de]->measurement(3);
    info_pub_[de]->publish(info_msg);

    if (tracker_[de]->tracker_state == Tracker::DETECTING) {
      target_msg.tracking = false;
    } else if (
      tracker_[de]->tracker_state == Tracker::TRACKING ||
      tracker_[de]->tracker_state == Tracker::TEMP_LOST) {
      target_msg.tracking = true;
      // Fill target message
      const auto & state = tracker_[de]->target_state;
      target_msg.id = tracker_[de]->tracked_id;
      target_msg.armors_num =
        static_cast<int>(tracker_[de]->tracked_armors_num);
      target_msg.position.x = state(0);
      target_msg.velocity.x = state(1);
      target_msg.position.y = state(2);
      target_msg.velocity.y = state(3);
      target_msg.position.z = state(4);
      target_msg.velocity.z = state(5);
      target_msg.yaw = state(6);
      target_msg.v_yaw = state(7);
      target_msg.radius_1 = state(8);
      target_msg.radius_2 = tracker_[de]->another_r;
      target_msg.dz = tracker_[de]->dz;
    } else if (tracker_[de]->tracker_state == Tracker::CHANGE_TARGET) {
      target_msg.tracking = false;
    }
  }

  last_time_[de] = time;

  target_pub_[de]->publish(target_msg);

  publishMarkers(target_msg, de);
}

void ArmorTrackerNode::publishMarkers(
  const auto_aim_interfaces::msg::Target & target_msg, DoubleEnd de)
{
  position_marker_[de].header = target_msg.header;
  linear_v_marker_[de].header = target_msg.header;
  angular_v_marker_[de].header = target_msg.header;
  armor_marker_[de].header = target_msg.header;

  visualization_msgs::msg::MarkerArray marker_array;
  if (target_msg.tracking) {
    double yaw = target_msg.yaw, r1 = target_msg.radius_1,
           r2 = target_msg.radius_2;
    double xc = target_msg.position.x, yc = target_msg.position.y,
           za = target_msg.position.z;
    double vx = target_msg.velocity.x, vy = target_msg.velocity.y,
           vz = target_msg.velocity.z;
    double dz = target_msg.dz;

    position_marker_[de].action = visualization_msgs::msg::Marker::ADD;
    position_marker_[de].pose.position.x = xc;
    position_marker_[de].pose.position.y = yc;
    position_marker_[de].pose.position.z = za + dz / 2;

    linear_v_marker_[de].action = visualization_msgs::msg::Marker::ADD;
    linear_v_marker_[de].points.clear();
    linear_v_marker_[de].points.emplace_back(
      position_marker_[de].pose.position);
    geometry_msgs::msg::Point arrow_end = position_marker_[de].pose.position;
    arrow_end.x += vx;
    arrow_end.y += vy;
    arrow_end.z += vz;
    linear_v_marker_[de].points.emplace_back(arrow_end);

    angular_v_marker_[de].action = visualization_msgs::msg::Marker::ADD;
    angular_v_marker_[de].points.clear();
    angular_v_marker_[de].points.emplace_back(
      position_marker_[de].pose.position);
    arrow_end = position_marker_[de].pose.position;
    arrow_end.z += target_msg.v_yaw / M_PI;
    angular_v_marker_[de].points.emplace_back(arrow_end);

    armor_marker_[de].action = visualization_msgs::msg::Marker::ADD;
    armor_marker_[de].scale.y =
      tracker_[de]->tracked_armor.type == "small" ? 0.135 : 0.23;
    bool is_current_pair = true;
    size_t a_n = target_msg.armors_num;
    geometry_msgs::msg::Point p_a;
    double r = 0;
    for (size_t i = 0; i < a_n; i++) {
      double tmp_yaw = yaw + i * (2 * M_PI / a_n);
      // Only 4 armors has 2 radius and height
      if (a_n == 4) {
        r = is_current_pair ? r1 : r2;
        p_a.z = za + (is_current_pair ? 0 : dz);
        is_current_pair = !is_current_pair;
      } else {
        r = r1;
        p_a.z = za;
      }
      p_a.x = xc - r * cos(tmp_yaw);
      p_a.y = yc - r * sin(tmp_yaw);

      armor_marker_[de].id = i;
      armor_marker_[de].pose.position = p_a;
      tf2::Quaternion q;
      q.setRPY(0, target_msg.id == "outpost" ? -0.26 : 0.26, tmp_yaw);
      armor_marker_[de].pose.orientation = tf2::toMsg(q);
      marker_array.markers.emplace_back(armor_marker_[de]);
    }
  } else {
    position_marker_[de].action = visualization_msgs::msg::Marker::DELETEALL;
    linear_v_marker_[de].action = visualization_msgs::msg::Marker::DELETEALL;
    angular_v_marker_[de].action = visualization_msgs::msg::Marker::DELETEALL;

    armor_marker_[de].action = visualization_msgs::msg::Marker::DELETEALL;
    marker_array.markers.emplace_back(armor_marker_[de]);
  }

  marker_array.markers.emplace_back(position_marker_[de]);
  marker_array.markers.emplace_back(linear_v_marker_[de]);
  marker_array.markers.emplace_back(angular_v_marker_[de]);
  marker_pub_[de]->publish(marker_array);
}

}  // namespace rm_auto_aim

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(rm_auto_aim::ArmorTrackerNode)