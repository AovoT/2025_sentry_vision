// detector_node.cpp
#include "armor_detector/detector_node.hpp"

#include <cv_bridge/cv_bridge.h>
#include <rmw/qos_profiles.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <memory>
#include <rclcpp/qos.hpp>
#include <sensor_msgs/msg/detail/camera_info__struct.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace rm_auto_aim
{

ArmorDetectorNode::ArmorDetectorNode(const rclcpp::NodeOptions & options)
: Node("armor_detector", options)
{
  RCLCPP_INFO(get_logger(), "Starting DetectorNode (refactored with QoS)...");

  // Initialize detector
  detector_ = initDetector();
  // Armors & marker publishers
  armors_pub_ = create_publisher<auto_aim_interfaces::msg::Armors>(
    "/detector/armors", rclcpp::SensorDataQoS());
  marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
    "/detector/marker", rclcpp::QoS(10));
  // QoS for high-frequency image subscriptions: drop old if busy
  auto sensor_qos = rclcpp::QoS(rclcpp::KeepLast(1))
                      .reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
  // Camera Info subscriptions
  for (int i = 0; i < DoubleEndMax; ++i) {
    cam_info_sub_[i] = create_subscription<sensor_msgs::msg::CameraInfo>(
      i == LEFT ? "/left/cam_info" : "/right/cam_info", rclcpp::SensorDataQoS(),
      [this, i](sensor_msgs::msg::CameraInfo::ConstSharedPtr msg) { camInfoCallback(msg, static_cast<DoubleEnd>(i)); });
  }
  // Image subscriptions with optimized QoS
  for (int i = 0; i < DoubleEndMax; ++i) {
    img_sub_[i] = create_subscription<sensor_msgs::msg::Image>(
      i == LEFT ? "/left/image_raw" : "/right/image_raw", sensor_qos,
      [this, i](sensor_msgs::msg::Image::ConstSharedPtr msg) { imageCallback(msg, static_cast<DoubleEnd>(i)); });
  }
  // Parameter callback for dynamic updates
  debug_enabled_ = declare_parameter("debug", false);
  param_cb_handle_ = add_on_set_parameters_callback(std::bind(
    &ArmorDetectorNode::onParametersSet, this, std::placeholders::_1));
  if (debug_enabled_) {
    createDebugPublishers();
  }
}
void ArmorDetectorNode::createDebugPublishers()
{
  if (!debug_pubs_) {
    debug_pubs_ = std::make_shared<DebugPublishers>();
  }
  debug_pubs_->lights =
    this->create_publisher<auto_aim_interfaces::msg::DebugLights>("/detector/debug_lights", 10);
  debug_pubs_->armors =
    this->create_publisher<auto_aim_interfaces::msg::DebugArmors>("/detector/debug_armors", 10);

  debug_pubs_->binary_left = image_transport::create_publisher(this, "/detector/left/binary_img");
  debug_pubs_->binary_right = image_transport::create_publisher(this, "/detector/right/binary_img");
  debug_pubs_->numbers = image_transport::create_publisher(this, "/detector/number_img");
  debug_pubs_->result = image_transport::create_publisher(this, "/detector/result_img");
}
void ArmorDetectorNode::destroyDebugPublishers()
{
  debug_pubs_->binary_left.shutdown();
  debug_pubs_->binary_right.shutdown();
  debug_pubs_->numbers.shutdown();
  debug_pubs_->result.shutdown();
  debug_pubs_.reset();
}

bool ArmorDetectorNode::shouldDetect(DoubleEnd de)
{
  std::lock_guard lock(find_mutex_);
  if (owner_ == -1) return true;
  if (owner_ == static_cast<int>(de) && miss_count_[de] < kMissTolerance)
    return true;
  if (owner_ != static_cast<int>(de)) return false;
  // Release after tolerance reached
  owner_ = -1;
  return true;
}

void ArmorDetectorNode::imageCallback(
  const sensor_msgs::msg::Image::ConstSharedPtr img, DoubleEnd de)
{
  if (!shouldDetect(de)) return;
  auto armors = detectArmors(img, de);
  bool found = !armors.empty();

  {  // Update ownership and miss count
    std::lock_guard lock(find_mutex_);
    if (found) {
      if (owner_ == -1) owner_ = static_cast<int>(de);
      if (owner_ == static_cast<int>(de)) miss_count_[de] = 0;
    } else if (owner_ == static_cast<int>(de)) {
      ++miss_count_[de];
    }
  }

  if (owner_ != static_cast<int>(de) || !found) return;
  publishArmorsAndMarkers(armors, img, de);
}

void ArmorDetectorNode::camInfoCallback(
  const sensor_msgs::msg::CameraInfo::ConstSharedPtr info, DoubleEnd de)
{
  cam_center_[de] = {info->k[2], info->k[5]};
  pnp_solver_[de] = std::make_shared<PnPSolver>(info->k, info->d);
  cam_info_sub_[de].reset();
}

std::unique_ptr<Detector> ArmorDetectorNode::initDetector()
{
  int bin_th = declare_parameter("binary_thres", 160);
  int color = declare_parameter("detect_color", RED);
  Detector::LightParams lp{
    declare_parameter("light.min_ratio", 0.1),
    declare_parameter("light.max_ratio", 0.4),
    declare_parameter("light.max_angle", 40.0)};
  Detector::ArmorParams ap{
    declare_parameter("armor.min_light_ratio", 0.7),
    declare_parameter("armor.min_small_center_distance", 0.8),
    declare_parameter("armor.max_small_center_distance", 3.2),
    declare_parameter("armor.min_large_center_distance", 3.2),
    declare_parameter("armor.max_large_center_distance", 5.5),
    declare_parameter("armor.max_angle", 35.0)};
  auto det = std::make_unique<Detector>(bin_th, color, lp, ap);
  // Classifier threshold & ignore classes
  double thr = declare_parameter("classifier_threshold", 0.7);
  det->classifier->threshold = thr;
  return det;
}

std::vector<Armor> ArmorDetectorNode::detectArmors(
  const sensor_msgs::msg::Image::ConstSharedPtr & img, DoubleEnd de)
{
  auto cvimg = cv_bridge::toCvShare(img, "rgb8")->image;
  auto res = detector_->detect(cvimg);

  if (debug_enabled_) {
    std::shared_ptr<DebugPublishers> dbg;
    {
      std::lock_guard lock(debug_mutex_);
      dbg = debug_pubs_;
    }
    if (dbg) {
      dbg->binary_left.publish(
        cv_bridge::CvImage(img->header, "mono8", detector_->binary_img)
          .toImageMsg());
      dbg->lights->publish(detector_->debug_lights);
      dbg->armors->publish(detector_->debug_armors);
      dbg->numbers.publish(
        *cv_bridge::CvImage(
           img->header, "mono8", detector_->getAllNumbersImage())
           .toImageMsg());
      // Draw on image & publish result
      detector_->drawResults(cvimg);
      cv::circle(cvimg, cam_center_[de], 5, cv::Scalar(255, 0, 0), 2);
      dbg->result.publish(
        cv_bridge::CvImage(img->header, "rgb8", cvimg).toImageMsg());
    }
  }
  return res;
}

void ArmorDetectorNode::publishArmorsAndMarkers(
  const std::vector<Armor> & armors,
  const sensor_msgs::msg::Image::ConstSharedPtr & img, DoubleEnd de)
{
  auto header = img->header;

  // Armors message
  auto_aim_interfaces::msg::Armors msg;
  msg.header = header;


  int id = 0;
  for (const auto & armor : armors) {
    cv::Mat rvec, tvec;
    bool ok = pnp_solver_[de] && pnp_solver_[de]->solvePnP(armor, rvec, tvec);
    if (!ok) {
      RCLCPP_WARN(get_logger(), "PnP failed for armor");
      continue;
    }
    auto amsg = auto_aim_interfaces::msg::Armor();
    amsg.type = ARMOR_TYPE_STR[static_cast<int>(armor.type)];
    amsg.number = armor.number;
    geometry_msgs::msg::Pose pose;
    pose.position.x = tvec.at<double>(0);
    pose.position.y = tvec.at<double>(1);
    pose.position.z = tvec.at<double>(2);
    cv::Mat rot;
    cv::Rodrigues(rvec, rot);
    tf2::Matrix3x3 m(
      rot.at<double>(0, 0), rot.at<double>(0, 1), rot.at<double>(0, 2),
      rot.at<double>(1, 0), rot.at<double>(1, 1), rot.at<double>(1, 2),
      rot.at<double>(2, 0), rot.at<double>(2, 1), rot.at<double>(2, 2));
    tf2::Quaternion q;
    m.getRotation(q);
    pose.orientation = tf2::toMsg(q);
    amsg.pose = pose;
    amsg.distance_to_image_center =
      pnp_solver_[de]->calculateDistanceToCenter(armor.center);
    msg.armors.push_back(amsg);
  // MarkerArray
  if (debug_enabled_) {
    visualization_msgs::msg::MarkerArray marray;
    visualization_msgs::msg::Marker cube, text;
    cube.header = header;
    cube.ns = "armors";
    cube.type = visualization_msgs::msg::Marker::CUBE;
    cube.action = visualization_msgs::msg::Marker::ADD;
    cube.scale.x = 0.05;
    cube.scale.z = 0.125;
    cube.color.a = 1.0f;
    cube.color.g = 0.5f;
    cube.color.b = 1.0f;
    cube.lifetime = rclcpp::Duration::from_seconds(0.1);

    text.header = header;
    text.ns = "classification";
    text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    text.action = visualization_msgs::msg::Marker::ADD;
    text.scale.z = 0.1;
    text.color.a = 1.0f;
    text.color.r = 1.0f;
    text.color.g = 1.0f;
    text.color.b = 1.0f;
    text.lifetime = cube.lifetime;
    cube.id = id++;
    cube.pose = pose;
    marray.markers.push_back(cube);

    text.id = id++;
    text.text = armor.classfication_result;
    text.pose = pose;
    text.pose.position.y -= 0.1;
    marray.markers.push_back(text);
    marker_pub_->publish(marray);
  }
  }

  armors_pub_->publish(msg);
}

rcl_interfaces::msg::SetParametersResult ArmorDetectorNode::onParametersSet(
  const std::vector<rclcpp::Parameter> & params)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "";

  for (const auto & p : params) {
    const auto & name = p.get_name();
    if (name == "debug") {
      bool en = p.as_bool();
      debug_enabled_ = en;
      if (en)
        createDebugPublishers();
      else
        destroyDebugPublishers();
    } else if (name == "binary_thres") {
      detector_->binary_thres = p.as_int();
    } else if (name == "detect_color") {
      detector_->detect_color = p.as_int();
    } else if (name == "classifier_threshold") {
      detector_->classifier->threshold = p.as_double();
    } else if (name == "light.min_ratio") {
      detector_->l.min_ratio = p.as_double();
    } else if (name == "light.max_ratio") {
      detector_->l.max_ratio = p.as_double();
    } else if (name == "light.max_angle") {
      detector_->l.max_angle = p.as_double();
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
RCLCPP_COMPONENTS_REGISTER_NODE(rm_auto_aim::ArmorDetectorNode)
