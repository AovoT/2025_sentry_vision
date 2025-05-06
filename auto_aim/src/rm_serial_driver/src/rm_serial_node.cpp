#include "rm_serial_driver/rm_serial_node.hpp"

#include <tf2_ros/buffer_interface.h>
#include <tf2_ros/transform_listener.h>

#include <auto_aim_interfaces/msg/detail/tracker_info__struct.hpp>
#include <exception>
#include <mutex>
#include <rcl_interfaces/msg/detail/parameter_descriptor__struct.hpp>
#include <rclcpp/callback_group.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/subscription_options.hpp>
#include <string>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2/time.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <visualization_msgs/msg/detail/marker__struct.hpp>

#include "rm_serial_driver/packet.hpp"
#include "util/mathit.hpp"
#include "util/serial_parser.hpp"

namespace rm_serial_driver
{

// Params implementation
template <typename T>
static T declare(
  rclcpp::Node * node, const std::string & name, const T & default_val,
  const rcl_interfaces::msg::ParameterDescriptor & desc =
    rcl_interfaces::msg::ParameterDescriptor())
{
  return node->declare_parameter(name, default_val, desc);
}

Params::Params(rclcpp::Node * node)
{
  rcl_interfaces::msg::ParameterDescriptor desc;
  desc.set__read_only(true);
  config[LEFT].device_name =
    declare(node, "left.device_name", std::string("/dev/ttyACM0"), desc);
  config[LEFT].baudrate = declare(node, "left.baud_rate", 115200, desc);
  config[LEFT].hardware_flow = declare(node, "left.hardware_flow", false, desc);
  config[LEFT].parity = declare(node, "left.parity", false, desc);
  config[LEFT].stop_bits = declare(node, "left.stop_bits", 1, desc);
  config[LEFT].timeout_ms = declare(node, "left.timeout_ms", 1000, desc);

  config[RIGHT].device_name =
    declare(node, "right.device_name", std::string("/dev/ttyACM1"), desc);
  config[RIGHT].baudrate = declare(node, "right.baud_rate", 115200, desc);
  config[RIGHT].hardware_flow =
    declare(node, "right.hardware_flow", false, desc);
  config[RIGHT].parity = declare(node, "right.parity", false, desc);
  config[RIGHT].stop_bits = declare(node, "right.stop_bits", 1, desc);
  config[RIGHT].timeout_ms = declare(node, "right.timeout_ms", 1000, desc);

  debug = declare(node, "debug", false);
}

// Publishers implementation
Publishers::Publishers(rclcpp::Node * node) : tf_broadcaster(node)
{
  marker_p =
    node->create_publisher<visualization_msgs::msg::Marker>("aiming_point", 10);
}

// Subscribers implementation
Subscribers::Subscribers(rclcpp::Node * node, RMSerialDriver * parent)
{
  cb_group[LEFT] =
    node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  cb_group[RIGHT] =
    node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  rclcpp::SubscriptionOptions options[DOUBLE_END_MAX];
  options[LEFT].callback_group = cb_group[LEFT];
  options[RIGHT].callback_group = cb_group[RIGHT];
  tf_buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);
  target_sub[LEFT] =
    node->create_subscription<auto_aim_interfaces::msg::Target>(
      "/tracker/left/target", rclcpp::SensorDataQoS(),
      std::bind(&RMSerialDriver::handleLeftMsg, parent, std::placeholders::_1),
      options[LEFT]);
  target_sub[RIGHT] =
    node->create_subscription<auto_aim_interfaces::msg::Target>(
      "/tracker/right/target", rclcpp::SensorDataQoS(),
      std::bind(&RMSerialDriver::handleRightMsg, parent, std::placeholders::_1),
      options[RIGHT]);
}

// Clients implementation
Clients::Clients(rclcpp::Node * node) : node_ptr(node)
{
  detector_client =
    std::make_shared<rclcpp::AsyncParametersClient>(node_ptr, "armor_detector");
  reset_tracker_srv =
    node_ptr->create_client<std_srvs::srv::Trigger>("/reset_tracker");
}

void Clients::setParam(const rclcpp::Parameter & param)
{
  setParamInternal(param, detector_client, "detect_color");
}

void Clients::setParamInternal(
  const rclcpp::Parameter & param,
  const rclcpp::AsyncParametersClient::SharedPtr & client,
  const std::string & name, bool invert)
{
  if (!client || !client->service_is_ready()) {
    RCLCPP_WARN(
      node_ptr->get_logger(), "%s service not ready, skipping", name.c_str());
    return;
  }

  if (!param_mutex.try_lock()) {
    RCLCPP_DEBUG(node_ptr->get_logger(), "%s param busy, skip", name.c_str());
    return;
  }
  std::lock_guard<std::mutex> lock(param_mutex, std::adopt_lock);

  int value = invert ? (1 - param.as_int()) : param.as_int();
  rclcpp::Parameter new_param(param.get_name(), value);

  RCLCPP_INFO(node_ptr->get_logger(), "Setting %s to %d", name.c_str(), value);

  auto sync_future = client->set_parameters({new_param});
  if (
    sync_future.wait_for(std::chrono::milliseconds(100)) ==
    std::future_status::ready) {
    try {
      auto results = sync_future.get();
      for (const auto & res : results) {
        if (!res.successful) {
          RCLCPP_ERROR(
            node_ptr->get_logger(), "%s set failed: %s", name.c_str(),
            res.reason.c_str());
          return;
        }
      }
      RCLCPP_INFO(
        node_ptr->get_logger(), "%s set to %d succeeded", name.c_str(), value);
    } catch (const std::exception & e) {
      RCLCPP_ERROR(
        node_ptr->get_logger(), "Exception while setting %s: %s", name.c_str(),
        e.what());
    }
  } else {
    RCLCPP_WARN(node_ptr->get_logger(), "%s set timeout", name.c_str());
  }
}

// RMSerialDriver implementation
RMSerialDriver::RMSerialDriver(const rclcpp::NodeOptions & opts)
: Node("rm_serial_driver", opts),
  prev_color_(0),
  params_(this),
  pubs_(this),
  subs_(this, this),
  clis_(this)
{
  serial_parser_[LEFT] = std::make_unique<SerialParser>(this->get_logger());
  serial_parser_[RIGHT] = std::make_unique<SerialParser>(this->get_logger());
  serial_parser_[LEFT]->registerHandler<ReceiveImuData>(
    0x01, [this](auto & pkt) { handlePacket(pkt, DoubleEnd::LEFT); });
  serial_parser_[LEFT]->registerHandler<ReceiveTargetInfoData>(
    0x09, [this](auto & pkt) { handlePacket(pkt, DoubleEnd::LEFT); });
  serial_parser_[RIGHT]->registerHandler<ReceiveImuData>(
    0x01, [this](auto & pkt) { handlePacket(pkt, DoubleEnd::RIGHT); });
  serial_parser_[RIGHT]->registerHandler<ReceiveTargetInfoData>(
    0x09, [this](auto & pkt) { handlePacket(pkt, DoubleEnd::RIGHT); });
  port_[LEFT] = std::make_unique<SerialPort>(params_.config[LEFT]);
  port_[RIGHT] = std::make_unique<SerialPort>(params_.config[RIGHT]);
  this->openPortWithRetry(LEFT);
  this->openPortWithRetry(RIGHT);
  params_.param_cb_handle = add_on_set_parameters_callback(
    (std::bind(&RMSerialDriver::paramCallback, this, std::placeholders::_1)));
  thread_[LEFT] = std::thread([this] { receiveLoop(DoubleEnd::LEFT); });
  thread_[RIGHT] = std::thread([this] { receiveLoop(DoubleEnd::RIGHT); });
}

// === 新增: 打开串口并自动重连 ===
void RMSerialDriver::openPortWithRetry(DoubleEnd de)
{
  const char * which = (de == LEFT ? "left" : "right");
  while (rclcpp::ok()) {
    try {
      port_[de]->open();
      RCLCPP_INFO(get_logger(), "%s serial port opened", which);
      return;  // 成功 → 直接返回
    } catch (const std::exception & e) {
      RCLCPP_WARN(
        get_logger(), "%s serial open failed: %s; retry in 1 s …", which,
        e.what());
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }
  }
}

RMSerialDriver::~RMSerialDriver()
{
  rclcpp::shutdown();
  if (thread_[LEFT].joinable()) thread_[LEFT].join();
  if (thread_[RIGHT].joinable()) thread_[RIGHT].join();
}

void RMSerialDriver::receiveLoop(DoubleEnd de)
{
  std::string end_str = de == DoubleEnd::LEFT ? "left" : "right";

  while (rclcpp::ok()) {
    ssize_t n = port_[de]->read(data_buf_[de], BUFFER_SIZE);
    if (n > 0) {
      serial_parser_[de]->feed(data_buf_[de], static_cast<size_t>(n));
    } else if (n == 0) {
      // EOF: 对端关闭，设备可能掉了
      RCLCPP_WARN(
        get_logger(), "%s port returned 0 (EOF), 尝试重连…", end_str.c_str());
      port_[de]->close();
      openPortWithRetry(de);
    } else if (errno == EAGAIN || errno == EWOULDBLOCK) {
      // 非阻塞模式下没数据，跳过
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    } else {
      RCLCPP_ERROR(
        get_logger(), "%s port read error (%s), 尝试重连…", end_str.c_str(),
        std::strerror(errno));
      port_[de]->close();
      openPortWithRetry(de);
    }
  }
}

void RMSerialDriver::handlePacket(const ReceiveImuData & pkt, DoubleEnd de)
{
  tf2::Quaternion q;
  q.setRPY(pkt.data.roll, pkt.data.pitch, pkt.data.yaw);
  q.normalize();
  if (
    std::isnan(q.x()) || std::isnan(q.y()) || std::isnan(q.z()) ||
    std::isnan(q.w())) {
    RCLCPP_WARN(get_logger(), "roll, pitch or yaw is invalid nan");
    return;
  }
  geometry_msgs::msg::TransformStamped t;
  t.header.stamp = now();
  t.header.frame_id =
    (de == DoubleEnd::LEFT ? "gimbal_left_link_offset"
                           : "gimbal_right_link_offset");
  t.child_frame_id =
    (de == DoubleEnd::LEFT ? "gimbal_left_link" : "gimbal_right_link");
  t.transform.rotation = tf2::toMsg(q);
  pubs_.tf_broadcaster.sendTransform(t);
}

void RMSerialDriver::handleMsg(
  auto_aim_interfaces::msg::Target::SharedPtr msg, DoubleEnd de)
{
  std::map<std::string, int> name_to_id = {
    {"", 0},  {"1", 1},       {"2", 2},     {"3", 3},    {"4", 4},
    {"5", 5}, {"outpost", 6}, {"guard", 7}, {"base", 8},
  };

  bool is_valid = msg->header.frame_id == "gimbal_left_link_offset" ||
                  msg->header.frame_id == "gimbal_right_link_offset" ||
                  msg->header.frame_id == "odom";
  if (!is_valid) {
    RCLCPP_WARN(
      get_logger(), "invalid target frame id : %s",
      msg->header.frame_id.c_str());
    return;
  }

  // ========== 准备共用的基础数据 ==========
  SendVisionData base_pkt{};
  base_pkt.frame_header.sof = 0x5A;
  base_pkt.frame_header.id = 0x03;
  base_pkt.time_stamp = std::chrono::duration_cast<std::chrono::milliseconds>(
                          std::chrono::steady_clock::now().time_since_epoch())
                          .count();

  auto & d = base_pkt.data;
  d.tracking = msg->tracking;
  d.armors_num = msg->armors_num;
  d.id = name_to_id[msg->id];
  d.yaw = msg->yaw;
  d.x = msg->position.x;
  d.y = msg->position.y;
  d.z = msg->position.z;
  d.vx = msg->velocity.x;
  d.vy = msg->velocity.y;
  d.vz = msg->velocity.z;
  d.v_yaw = msg->v_yaw;
  d.r1 = msg->radius_1;
  d.r2 = msg->radius_2;
  d.dz = msg->dz;

  geometry_msgs::msg::PointStamped pt_out;

  auto buf = pack(base_pkt);
  if (params_.debug) {
    RCLCPP_INFO(
      get_logger(), "%s port debug info", de == LEFT ? "left" : "right");
    print(base_pkt);
  }

  if (static_cast<ssize_t>(port_[de]->write(buf.data(), buf.size())) < 0) {
    RCLCPP_WARN(get_logger(), "Failed to send vision data to port");
  }
}

void RMSerialDriver::handlePacket(
  const ReceiveTargetInfoData & pkt, DoubleEnd de)
{
  auto d = pkt.data;
  std::array<float, 3> point = {d.aim_x, d.aim_y, d.aim_z};
  auto distance = norm(point);

  if (distance > 0.01 && params_.debug) {
    visualization_msgs::msg::Marker aiming_point;
    aiming_point.header.frame_id =
      de == LEFT ? "gimbal_left_link_offset" : "gimbal_right_link_offset";
    aiming_point.header.stamp = this->now();
    aiming_point.lifetime = rclcpp::Duration::from_seconds(1);
    aiming_point.pose.position.x = d.aim_x;
    aiming_point.pose.position.x = d.aim_y;
    aiming_point.pose.position.x = d.aim_z;
    pubs_.marker_p->publish(aiming_point);
  }
  static std::atomic<uint8_t> aiming_color = UNKNOWN;
  if (d.detect_color != RED && d.detect_color != BLUE) [[unlikely]] {
    RCLCPP_ERROR(
      get_logger(),
      "invalid color value: %d, expected color value is 0 - RED, 1 - BLUE",
      d.detect_color);
    return;
  }
  if (d.detect_color != aiming_color || aiming_color == UNKNOWN) [[unlikely]] {
    clis_.setParam(rclcpp::Parameter("detect_color", d.detect_color));
    aiming_color.store(d.detect_color);
  }
}

rcl_interfaces::msg::SetParametersResult RMSerialDriver::paramCallback(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";

  for (const auto & param : parameters) {
    if (param.get_name() == "debug") {
      params_.debug = param.as_bool();
      RCLCPP_INFO(
        this->get_logger(), "Updated debug: %s",
        param.as_bool() ? "true" : "false");
    } else {
      result.successful = false;
      result.reason = "Unsupported parameter update: " + param.get_name();
    }
  }

  return result;
}

}  // namespace rm_serial_driver

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(rm_serial_driver::RMSerialDriver)