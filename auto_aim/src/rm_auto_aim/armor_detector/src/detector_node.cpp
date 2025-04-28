// Copyright 2022 Chen Jun
// Licensed under the MIT License.

#include <cv_bridge/cv_bridge.h>
#include <rmw/qos_profiles.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/convert.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <image_transport/image_transport.hpp>
#include <mutex>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/qos.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// STD
#include <algorithm>
#include <memory>
#include <string>
#include <vector>

#include "armor_detector/armor.hpp"
#include "armor_detector/detector_node.hpp"

namespace rm_auto_aim
{
ArmorDetectorNode::ArmorDetectorNode(const rclcpp::NodeOptions & options)
: Node("armor_detector", options)
{
  RCLCPP_INFO(this->get_logger(), "Starting DetectorNode!");
  last_find_[LEFT] = false;
  last_find_[RIGHT] = false;

  // Detector
  detector_ = initDetector();

  param_client_ =
    std::make_shared<rclcpp::AsyncParametersClient>(this, "armor_detector");
  // Armors Publisher
  armors_pub_ = this->create_publisher<auto_aim_interfaces::msg::Armors>(
    "/detector/armors", rclcpp::SensorDataQoS());

  // Visualization Marker Publisher
  // See http://wiki.ros.org/rviz/DisplayTypes/Marker
  armor_marker_.ns = "armors";
  armor_marker_.action = visualization_msgs::msg::Marker::ADD;
  armor_marker_.type = visualization_msgs::msg::Marker::CUBE;
  armor_marker_.scale.x = 0.05;
  armor_marker_.scale.z = 0.125;
  armor_marker_.color.a = 1.0;
  armor_marker_.color.g = 0.5;
  armor_marker_.color.b = 1.0;
  armor_marker_.lifetime = rclcpp::Duration::from_seconds(0.1);

  text_marker_.ns = "classification";
  text_marker_.action = visualization_msgs::msg::Marker::ADD;
  text_marker_.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
  text_marker_.scale.z = 0.1;
  text_marker_.color.a = 1.0;
  text_marker_.color.r = 1.0;
  text_marker_.color.g = 1.0;
  text_marker_.color.b = 1.0;
  text_marker_.lifetime = rclcpp::Duration::from_seconds(0.1);

  marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
    "/detector/marker", 10);

  // Debug Publishers
  debug_ = this->declare_parameter("debug", false);
  if (debug_) {
    createDebugPublishers();
  }

  // Debug param change moniter
  debug_param_sub_ = std::make_shared<rclcpp::ParameterEventHandler>(this);
  debug_cb_handle_ = debug_param_sub_->add_parameter_callback(
    "debug", [this](const rclcpp::Parameter & p) {
      debug_ = p.as_bool();
      debug_ ? createDebugPublishers() : destroyDebugPublishers();
    });

  cam_info_sub_[LEFT] = this->create_subscription<sensor_msgs::msg::CameraInfo>(
    "/left/cam_info", rclcpp::SensorDataQoS(),
    [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr camera_info) {
      this->camInfoCallback(camera_info, LEFT);
    });
  cam_info_sub_[RIGHT] =
    this->create_subscription<sensor_msgs::msg::CameraInfo>(
      "/right/cam_info", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr camera_info) {
        this->camInfoCallback(camera_info, RIGHT);
      });

  img_sub_[LEFT] = this->create_subscription<sensor_msgs::msg::Image>(
    "/left/image_raw", rclcpp::SensorDataQoS(),
    [this](std::shared_ptr<const sensor_msgs::msg::Image> msg) {
      this->imageCallback(msg, LEFT);
    });

  img_sub_[RIGHT] = this->create_subscription<sensor_msgs::msg::Image>(
    "/right/image_raw", rclcpp::SensorDataQoS(),
    [this](std::shared_ptr<const sensor_msgs::msg::Image> msg) {
      this->imageCallback(msg, RIGHT);
    });
  param_cb_handle_ = this->add_on_set_parameters_callback(std::bind(
    &ArmorDetectorNode::onParametersSet, this, std::placeholders::_1));
}

void ArmorDetectorNode::camInfoCallback(
  sensor_msgs::msg::CameraInfo::ConstSharedPtr camera_info, DoubleEnd de)
{
  cam_center_[de] = cv::Point2f(camera_info->k[2], camera_info->k[5]);
  cam_info_[de] = std::make_shared<sensor_msgs::msg::CameraInfo>(*camera_info);
  pnp_solver_[de] = std::make_unique<PnPSolver>(camera_info->k, camera_info->d);
  cam_info_sub_[de].reset();
}
bool ArmorDetectorNode::shouldDetect(DoubleEnd de)
{
  std::lock_guard<std::mutex> lk(find_mtx_);

  // 如果没人持有——两侧都可检测
  if (owner_ == -1) {
    return true;
  }

  // 如果我是持有者
  if (owner_ == de) {
    // 漏检未超限，则继续由我检测
    if (miss_count_[de] < kMissTolerance) {
      return true;
    }
    // 漏检超限，释放权利，让双方都有机会抢占
    owner_ = -1;
    miss_count_[de] = kMissTolerance;
    return true;  // 本帧也让它检测一次，用于下一步抢占
  }

  // 如果我不是持有者，则跳过检测
  return false;
}

void ArmorDetectorNode::imageCallback(
  const sensor_msgs::msg::Image::ConstSharedPtr img_msg, DoubleEnd de)
{
  bool do_detect = shouldDetect(de);
  std::vector<Armor> armors;
  bool found = false;
  if (do_detect) {
    armors = detectArmors(img_msg, de);
    found = !armors.empty();
  }
  // 2) 在 shouldDetect 之外，再做一次持有者的状态更新
  if (do_detect) {
    std::lock_guard<std::mutex> lk(find_mtx_);
    if (found) {
      // 抢占：只有当没人持有时才设置
      int expected = -1;
      owner_.compare_exchange_strong(expected, de);
      if (owner_ == de) {
        // 成为持有者，漏检计数器清零
        miss_count_[de] = 0;
      }
    } else if (owner_ == de) {
      // 我是持有者但漏检
      miss_count_[de]++;
    }
  }

  // 3) 只有当我是持有者 && 本帧检测到 才发布
  {
    std::lock_guard<std::mutex> lk(find_mtx_);
    if (owner_ != de || !found) {
      return;
    }
  }

  if (pnp_solver_[de] != nullptr) {
    armors_msg_.header = armor_marker_.header = text_marker_.header =
      img_msg->header;
    armors_msg_.armors.clear();
    marker_array_.markers.clear();
    armor_marker_.id = 0;
    text_marker_.id = 0;

    auto_aim_interfaces::msg::Armor armor_msg;
    for (const auto & armor : armors) {
      cv::Mat rvec, tvec;
      bool success = pnp_solver_[de]->solvePnP(armor, rvec, tvec);
      if (success) {
        // Fill basic info
        armor_msg.type = ARMOR_TYPE_STR[static_cast<int>(armor.type)];
        armor_msg.number = armor.number;

        // Fill pose
        armor_msg.pose.position.x = tvec.at<double>(0);
        armor_msg.pose.position.y = tvec.at<double>(1);
        armor_msg.pose.position.z = tvec.at<double>(2);
        // rvec to 3x3 rotation matrix
        cv::Mat rotation_matrix;
        cv::Rodrigues(rvec, rotation_matrix);
        // rotation matrix to quaternion
        tf2::Matrix3x3 tf2_rotation_matrix(
          rotation_matrix.at<double>(0, 0), rotation_matrix.at<double>(0, 1),
          rotation_matrix.at<double>(0, 2), rotation_matrix.at<double>(1, 0),
          rotation_matrix.at<double>(1, 1), rotation_matrix.at<double>(1, 2),
          rotation_matrix.at<double>(2, 0), rotation_matrix.at<double>(2, 1),
          rotation_matrix.at<double>(2, 2));
        tf2::Quaternion tf2_q;
        tf2_rotation_matrix.getRotation(tf2_q);
        armor_msg.pose.orientation = tf2::toMsg(tf2_q);

        // Fill the distance to image center
        armor_msg.distance_to_image_center =
          pnp_solver_[de]->calculateDistanceToCenter(armor.center);

        // Fill the markers
        armor_marker_.id++;
        armor_marker_.scale.y = armor.type == ArmorType::SMALL ? 0.135 : 0.23;
        armor_marker_.pose = armor_msg.pose;
        text_marker_.id++;
        text_marker_.pose.position = armor_msg.pose.position;
        text_marker_.pose.position.y -= 0.1;
        text_marker_.text = armor.classfication_result;
        armors_msg_.armors.emplace_back(armor_msg);
        marker_array_.markers.emplace_back(armor_marker_);
        marker_array_.markers.emplace_back(text_marker_);
      } else {
        RCLCPP_WARN(this->get_logger(), "PnP failed!");
      }
    }

    // Publishing detected armors
    // armors_msg_.header.stamp = this->now();
    armors_pub_->publish(armors_msg_);

    // Publishing marker
    publishMarkers();
  }
}

std::unique_ptr<Detector> ArmorDetectorNode::initDetector()
{
  rcl_interfaces::msg::ParameterDescriptor param_desc;
  param_desc.integer_range.resize(1);
  param_desc.integer_range[0].step = 1;
  param_desc.integer_range[0].from_value = 0;
  param_desc.integer_range[0].to_value = 255;
  int binary_thres = declare_parameter("binary_thres", 160, param_desc);

  param_desc.description = "0-RED, 1-BLUE";
  param_desc.integer_range[0].from_value = 0;
  param_desc.integer_range[0].to_value = 1;
  auto detect_color = declare_parameter("detect_color", RED, param_desc);

  Detector::LightParams l_params = {
    .min_ratio = declare_parameter("light.min_ratio", 0.1),
    .max_ratio = declare_parameter("light.max_ratio", 0.4),
    .max_angle = declare_parameter("light.max_angle", 40.0)};

  Detector::ArmorParams a_params = {
    .min_light_ratio = declare_parameter("armor.min_light_ratio", 0.7),
    .min_small_center_distance =
      declare_parameter("armor.min_small_center_distance", 0.8),
    .max_small_center_distance =
      declare_parameter("armor.max_small_center_distance", 3.2),
    .min_large_center_distance =
      declare_parameter("armor.min_large_center_distance", 3.2),
    .max_large_center_distance =
      declare_parameter("armor.max_large_center_distance", 5.5),
    .max_angle = declare_parameter("armor.max_angle", 35.0)};

  auto detector =
    std::make_unique<Detector>(binary_thres, detect_color, l_params, a_params);

  // Init classifier
  auto pkg_path =
    ament_index_cpp::get_package_share_directory("armor_detector");
  auto model_path = pkg_path + "/model/mlp.onnx";
  auto label_path = pkg_path + "/model/label.txt";
  double threshold = this->declare_parameter("classifier_threshold", 0.7);
  std::vector<std::string> ignore_classes = this->declare_parameter(
    "ignore_classes", std::vector<std::string>{"negative"});
  detector->classifier = std::make_unique<NumberClassifier>(
    model_path, label_path, threshold, ignore_classes);

  return detector;
}

std::vector<Armor> ArmorDetectorNode::detectArmors(
  const sensor_msgs::msg::Image::ConstSharedPtr & img_msg, DoubleEnd de)
{
  // Convert ROS img to cv::Mat
  auto img = cv_bridge::toCvShare(img_msg, "rgb8")->image;

  auto armors = detector_->detect(img);

  auto final_time = this->now();
  auto latency = (final_time - img_msg->header.stamp).seconds() * 1000;
  RCLCPP_DEBUG_STREAM(this->get_logger(), "Latency: " << latency << "ms");

  // Publish debug info
  if (debug_) {
    binary_img_pub_[de].publish(
      cv_bridge::CvImage(img_msg->header, "mono8", detector_->binary_img)
        .toImageMsg());

    // Sort lights and armors data by x coordinate
    std::sort(
      detector_->debug_lights.data.begin(), detector_->debug_lights.data.end(),
      [](const auto & l1, const auto & l2) {
        return l1.center_x < l2.center_x;
      });
    std::sort(
      detector_->debug_armors.data.begin(), detector_->debug_armors.data.end(),
      [](const auto & a1, const auto & a2) {
        return a1.center_x < a2.center_x;
      });

    lights_data_pub_->publish(detector_->debug_lights);
    armors_data_pub_->publish(detector_->debug_armors);

    if (!armors.empty()) {
      auto all_num_img = detector_->getAllNumbersImage();
      number_img_pub_.publish(
        *cv_bridge::CvImage(img_msg->header, "mono8", all_num_img)
           .toImageMsg());
    }

    detector_->drawResults(img);
    // Draw camera center
    cv::circle(img, cam_center_[de], 5, cv::Scalar(255, 0, 0), 2);
    // Draw latency
    std::stringstream latency_ss;
    latency_ss << "Latency: " << std::fixed << std::setprecision(2) << latency
               << "ms";
    auto latency_s = latency_ss.str();
    cv::putText(
      img, latency_s, cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 1.0,
      cv::Scalar(0, 255, 0), 2);
    result_img_pub_.publish(
      cv_bridge::CvImage(img_msg->header, "rgb8", img).toImageMsg());
  }

  return armors;
}

void ArmorDetectorNode::createDebugPublishers()
{
  lights_data_pub_ =
    this->create_publisher<auto_aim_interfaces::msg::DebugLights>(
      "/detector/debug_lights", 10);
  armors_data_pub_ =
    this->create_publisher<auto_aim_interfaces::msg::DebugArmors>(
      "/detector/debug_armors", 10);

  binary_img_pub_[LEFT] =
    image_transport::create_publisher(this, "/detector/left/binary_img");
  binary_img_pub_[RIGHT] =
    image_transport::create_publisher(this, "/detector/right/binary_img");
  number_img_pub_ =
    image_transport::create_publisher(this, "/detector/number_img");
  result_img_pub_ =
    image_transport::create_publisher(this, "/detector/result_img");
}

void ArmorDetectorNode::destroyDebugPublishers()
{
  lights_data_pub_.reset();
  armors_data_pub_.reset();

  binary_img_pub_[LEFT].shutdown();
  binary_img_pub_[RIGHT].shutdown();
  number_img_pub_.shutdown();
  result_img_pub_.shutdown();
}

void ArmorDetectorNode::publishMarkers()
{
  using Marker = visualization_msgs::msg::Marker;
  armor_marker_.action =
    armors_msg_.armors.empty() ? Marker::DELETE : Marker::ADD;
  marker_array_.markers.emplace_back(armor_marker_);
  marker_pub_->publish(marker_array_);
}

// detector_node.cpp

rcl_interfaces::msg::SetParametersResult ArmorDetectorNode::onParametersSet(
  const std::vector<rclcpp::Parameter> & params)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";

  for (const auto & p : params) {
    const auto & name = p.get_name();
    // 二值化阈值
    if (name == "binary_thres") {
      detector_->binary_thres = p.as_int();

      // 红蓝色选择
    } else if (name == "detect_color") {
      detector_->detect_color = p.as_int();

      // 分类器置信度阈值
    } else if (name == "classifier_threshold") {
      detector_->classifier->threshold = p.as_double();

      // LightParams
    } else if (name == "light.min_ratio") {
      detector_->l.min_ratio = p.as_double();
    } else if (name == "light.max_ratio") {
      detector_->l.max_ratio = p.as_double();
    } else if (name == "light.max_angle") {
      detector_->l.max_angle = p.as_double();

      // ArmorParams
    } else if (name == "armor.min_light_ratio") {
      detector_->a.min_light_ratio = p.as_double();
    } else if (name == "armor.min_small_center_distance") {
      detector_->a.min_small_center_distance = p.as_double();
    } else if (name == "armor.max_small_center_distance") {
      detector_->a.max_small_center_distance = p.as_double();
    } else if (name == "armor.min_large_center_distance") {
      detector_->a.min_large_center_distance = p.as_double();
    } else if (name == "armor.max_large_center_distance") {
      detector_->a.max_large_center_distance = p.as_double();
    } else if (name == "armor.max_angle") {
      detector_->a.max_angle = p.as_double();
    }
  }

  return result;
}

}  // namespace rm_auto_aim

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(rm_auto_aim::ArmorDetectorNode)
