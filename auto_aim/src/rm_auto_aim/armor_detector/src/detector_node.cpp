// detector_node.cpp
#include "armor_detector/detector_node.hpp"

#include <cv_bridge/cv_bridge.h>
#include <rmw/qos_profiles.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <auto_aim_interfaces/msg/detail/debug_armors__struct.hpp>
#include <auto_aim_interfaces/msg/detail/debug_lights__struct.hpp>
#include <hikcamera/image_capturer.hpp>
#include <image_transport/image_transport.hpp>
#include <memory>
#include <rclcpp/qos.hpp>
#include <sensor_msgs/msg/detail/camera_info__struct.hpp>
#include <std_msgs/msg/detail/header__struct.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "armor_detector/pnp_solver.hpp"

namespace rm_auto_aim
{

ArmorDetectorNode::ArmorDetectorNode(const rclcpp::NodeOptions & options)
: Node("armor_detector", options),
  hik_camera_params_(this),
  cam_info_manager_(this)
{
  RCLCPP_INFO(get_logger(), "Starting ArmorDetectorNode...");
  img_capture_[LEFT] = std::make_unique<hikcamera::ImageCapturer>(
    hik_camera_params_.left_camera_profile,
    hik_camera_params_.left_camera_name.c_str());
  img_capture_[RIGHT] = std::make_unique<hikcamera::ImageCapturer>(
    hik_camera_params_.right_camera_profile,
    hik_camera_params_.right_camera_name.c_str());
  RCLCPP_INFO(get_logger(), "Camera initialized.");

  initDetectors();
  RCLCPP_INFO(get_logger(), "Detector initialized.");

  armors_pub_[LEFT] = create_publisher<auto_aim_interfaces::msg::Armors>(
    "/detector/left/armors", rclcpp::SensorDataQoS());
  armors_pub_[RIGHT] = create_publisher<auto_aim_interfaces::msg::Armors>(
    "/detector/right/armors", rclcpp::SensorDataQoS());
  RCLCPP_INFO(get_logger(), "Armors publisher created.");

  marker_pub_[LEFT] = create_publisher<visualization_msgs::msg::MarkerArray>(
    "/detector/left/marker", rclcpp::QoS(10));
  marker_pub_[RIGHT] = create_publisher<visualization_msgs::msg::MarkerArray>(
    "/detector/right/marker", rclcpp::QoS(10));
  RCLCPP_INFO(get_logger(), "Marker publisher created.");

  // Parameter callback for dynamic updates
  debug_enabled_ = declare_parameter("debug", false);

  cam_info_manager_.setCameraName(hik_camera_params_.left_camera_name);
  cam_info_manager_.loadCameraInfo(hik_camera_params_.left_cam_info_url);
  auto left_ci = cam_info_manager_.getCameraInfo();
  cam_info_manager_.setCameraName(hik_camera_params_.right_camera_name);
  cam_info_manager_.loadCameraInfo(hik_camera_params_.right_cam_info_url);
  auto right_ci = cam_info_manager_.getCameraInfo();
  pnp_solver_[LEFT] = std::make_shared<PnPSolver>(left_ci.k, left_ci.d);
  pnp_solver_[RIGHT] = std::make_shared<PnPSolver>(right_ci.k, right_ci.d);
  cam_center_[LEFT] = cv::Point2f(left_ci.k[2], left_ci.k[5]);
  cam_center_[RIGHT] = cv::Point2f(right_ci.k[2], right_ci.k[5]);

  param_cb_handle_ = add_on_set_parameters_callback(std::bind(
    &ArmorDetectorNode::onParametersSet, this, std::placeholders::_1));
  for (int i = 0; i < DOUBLE_END_MAX; i++) {
    const char * end_str = i == LEFT ? "left" : "right";
    if (debug_enabled_) {
      debug_pubs_[i] =
        std::make_shared<DebugPublishers>(this, static_cast<DoubleEnd>(i));
    }
    capture_thread[i] = std::make_unique<std::thread>([this, end_str, i]() {
      RCLCPP_INFO(get_logger(), "%s capture loop thread running", end_str);
      captureLoop(static_cast<DoubleEnd>(i));
    });
    detect_thread[i] = std::make_unique<std::thread>([this, end_str, i]() {
      RCLCPP_INFO(get_logger(), "%s detect loop thread running", end_str);
      detectLoop(static_cast<DoubleEnd>(i));
    });
  }
}

ArmorDetectorNode::~ArmorDetectorNode()
{
  for (auto & thread : detect_thread) {
    if (thread && thread->joinable()) {
      thread->join();
      thread.reset();
    }
  }
  rclcpp::shutdown();
}

void ArmorDetectorNode::detectLoop(DoubleEnd de)
{
  while (rclcpp::ok()) {
    detectOnce(de);
    asm volatile("");
  }
}

void ArmorDetectorNode::captureLoop(DoubleEnd de)
{
  while (rclcpp::ok()) {
    if (!img_capture_[de]) [[unlikely]] {
      RCLCPP_ERROR(get_logger(), "img_capture_[%d] is null!", (int)de);
      return;
    }

    cv::Mat raw_img = img_capture_[de]->read();
    if (raw_img.empty()) [[unlikely]] {
      RCLCPP_WARN(get_logger(), "Captured empty image from camera %d", (int)de);
      return;
    }
    if (debug_enabled_) {
      if (debug_enabled_) {
        debug_pubs_[de]->img_raw.publish(
          *cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", raw_img)
             .toImageMsg());
      } else {
        RCLCPP_ERROR(get_logger(), "debug_pubs_ is null when debug enabled!");
      }
    }
    img_queue_[de].push(raw_img);
  }
}

void ArmorDetectorNode::detectOnce(DoubleEnd de)
{
  cv::Mat raw_img;
  if (!img_queue_[de].waitAndPop(raw_img)) {
    RCLCPP_WARN(get_logger(), "Failed to fetch the stream from the queue");
    return;
  }
  auto armors = detector_[de]->detect(raw_img);
  if (debug_enabled_) {
    cv::Mat dbg_img = raw_img.clone();
    std_msgs::msg::Header hdr;
    hdr.stamp = this->now();
    hdr.frame_id =
      de == LEFT ? "left_camera_optical_frame" : "right_camera_optical_frame";
    debug_pubs_[de]->binary.publish(
      cv_bridge::CvImage(hdr, "mono8", detector_[de]->binary_img).toImageMsg());
    debug_pubs_[de]->lights->publish(detector_[de]->debug_lights);
    debug_pubs_[de]->armors->publish(detector_[de]->debug_armors);
    debug_pubs_[de]->numbers.publish(
      *cv_bridge::CvImage(
         std_msgs::msg::Header(), "mono8", detector_[de]->getAllNumbersImage())
         .toImageMsg());
    // Draw on image & publish result
    detector_[de]->drawResults(dbg_img);
    cv::circle(dbg_img, cam_center_[de], 5, cv::Scalar(255, 0, 0), 2);
    debug_pubs_[de]->result.publish(
      cv_bridge::CvImage(hdr, "bgr8", dbg_img).toImageMsg());
  }
  if (armors.empty()) return;

  publishArmorsAndMarkers(armors, de);
}

void ArmorDetectorNode::initDetectors()
{
  rcl_interfaces::msg::ParameterDescriptor pd;
  pd.integer_range.resize(1);
  pd.integer_range[0].step = 1;
  pd.integer_range[0].from_value = 0;
  pd.integer_range[0].to_value = 255;

  int binary_thres = declare_parameter("binary_thres", 160, pd);

  pd.description = "0-RED, 1-BLUE";
  pd.integer_range[0].to_value = 1;
  auto detect_color = declare_parameter("detect_color", RED, pd);

  Detector::LightParams l_params{
    .min_ratio = declare_parameter("light.min_ratio", 0.1),
    .max_ratio = declare_parameter("light.max_ratio", 0.4),
    .max_angle = declare_parameter("light.max_angle", 40.0)};

  Detector::ArmorParams a_params{
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

  std::string pkg_path =
    ament_index_cpp::get_package_share_directory("armor_detector");
  std::string model_path = pkg_path + "/model/mlp.onnx";
  std::string label_path = pkg_path + "/model/label.txt";
  double threshold = declare_parameter("classifier_threshold", 0.7);
  auto ignore_classes =
    declare_parameter("ignore_classes", std::vector<std::string>{"negative"});

  for (auto & detct : detector_) {
    detct = std::make_unique<Detector>(
      binary_thres, detect_color, l_params, a_params);
    detct->classifier = std::make_unique<NumberClassifier>(
      model_path, label_path, threshold, ignore_classes);
  }
}

void ArmorDetectorNode::publishArmorsAndMarkers(
  const std::vector<Armor> & armors, DoubleEnd de)
{
  std_msgs::msg::Header header;
  header.stamp = this->now();
  header.frame_id =
    de == LEFT ? "left_camera_optical_frame" : "right_camera_optical_frame";

  // Armors message
  auto_aim_interfaces::msg::Armors msg;
  msg.header = header;

  int id = 0;
  visualization_msgs::msg::MarkerArray marray;
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
      visualization_msgs::msg::Marker cube, text;
      cube.header = msg.header;
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
    }
  }

  if (debug_enabled_) {
    marker_pub_[de]->publish(marray);
  }
  armors_pub_[de]->publish(msg);
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
      if (en) {
        debug_pubs_[LEFT] = std::make_shared<DebugPublishers>(this, LEFT);
        debug_pubs_[RIGHT] = std::make_shared<DebugPublishers>(this, RIGHT);
      } else {
        debug_pubs_[LEFT].reset();
        debug_pubs_[RIGHT].reset();
      }
    } else if (name == "binary_thres") {
      detector_[LEFT]->binary_thres = p.as_int();
      detector_[RIGHT]->binary_thres = p.as_int();
    } else if (name == "detect_color") {
      detector_[LEFT]->detect_color = p.as_int();
      detector_[RIGHT]->detect_color = p.as_int();
    } else if (name == "classifier_threshold") {
      detector_[LEFT]->classifier->threshold = p.as_double();
      detector_[RIGHT]->classifier->threshold = p.as_double();
    } else if (name == "light.min_ratio") {
      detector_[LEFT]->l.min_ratio = p.as_double();
      detector_[RIGHT]->l.min_ratio = p.as_double();
    } else if (name == "light.max_ratio") {
      detector_[LEFT]->l.max_ratio = p.as_double();
      detector_[RIGHT]->l.max_ratio = p.as_double();
    } else if (name == "light.max_angle") {
      detector_[LEFT]->l.max_angle = p.as_double();
      detector_[RIGHT]->l.max_angle = p.as_double();
    } else if (name == "armor.min_light_ratio") {
      detector_[LEFT]->a.min_light_ratio = p.as_double();
      detector_[RIGHT]->a.min_light_ratio = p.as_double();
    } else if (name == "armor.min_small_center_distance") {
      detector_[LEFT]->a.min_small_center_distance = p.as_double();
      detector_[RIGHT]->a.min_small_center_distance = p.as_double();
    } else if (name == "armor.max_small_center_distance") {
      detector_[LEFT]->a.max_small_center_distance = p.as_double();
      detector_[RIGHT]->a.max_small_center_distance = p.as_double();
    } else if (name == "armor.min_large_center_distance") {
      detector_[LEFT]->a.min_large_center_distance = p.as_double();
      detector_[RIGHT]->a.min_large_center_distance = p.as_double();
    } else if (name == "armor.max_large_center_distance") {
      detector_[LEFT]->a.max_large_center_distance = p.as_double();
      detector_[RIGHT]->a.max_large_center_distance = p.as_double();
    } else if (name == "armor.max_angle") {
      detector_[LEFT]->a.max_angle = p.as_double();
      detector_[RIGHT]->a.max_angle = p.as_double();
    }
  }
  return result;
}
DebugPublishers::DebugPublishers(rclcpp::Node * node_ptr, DoubleEnd de)
{
  std::string end_str = de == LEFT ? "left" : "right";
  this->lights =
    node_ptr->create_publisher<auto_aim_interfaces::msg::DebugLights>(
      "/detector/" + end_str + "/debug_lights", 10);
  this->armors = node_ptr->create_publisher<auto_aim_interfaces::msg::DebugArmors>(
    "/detector/" + end_str + "/armors", 10);
  this->binary = image_transport::create_publisher(
    node_ptr, "/detector/" + end_str + "/binary_img");
  this->img_raw = image_transport::create_publisher(
    node_ptr, "/detector/" + end_str + "/image_raw");
  this->numbers = image_transport::create_publisher(
    node_ptr, "/detector/" + end_str + "/number_img");
  this->result = image_transport::create_publisher(
    node_ptr, "/detector/" + end_str + "/result_img");
}
DebugPublishers::~DebugPublishers()
{
  binary.shutdown();
  img_raw.shutdown();
  numbers.shutdown();
  result.shutdown();
}

}  // namespace rm_auto_aim

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(rm_auto_aim::ArmorDetectorNode)
