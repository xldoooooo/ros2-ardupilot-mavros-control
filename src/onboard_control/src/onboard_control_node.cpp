/**
 * @file onboard_control_node.cpp
 * @brief 机载高层协议、控制权租约、MAVROS 编排、航点和固定周期输出实现。
 */
#include "onboard_control/onboard_control_node.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <sstream>
#include <utility>

#include <tf2/utils.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace onboard_control
{
namespace
{

constexpr char kInterfaceVersion[] = "1.0";
constexpr std::uint32_t kMinimumTtlMs = 50;
constexpr std::uint32_t kMaximumTtlMs = 10000;
constexpr std::uint32_t kMinimumLeaseMs = 300;
constexpr std::uint32_t kMaximumLeaseMs = 5000;
constexpr std::size_t kMaximumWaypoints = 256;
constexpr double kPi = 3.14159265358979323846;

const std::array<std::pair<std::uint32_t, float>, 3> kMessageIntervals{{
  {32U, 100.0F},   // LOCAL_POSITION_NED
  {31U, 100.0F},   // ATTITUDE_QUATERNION
  {105U, 100.0F},  // HIGHRES_IMU
}};

bool finite_waypoint(const guided_interfaces::msg::Waypoint & waypoint)
{
  return std::isfinite(waypoint.position.x) && std::isfinite(waypoint.position.y) &&
         std::isfinite(waypoint.position.z) && std::isfinite(waypoint.yaw);
}

}  // namespace

OnboardControlNode::OnboardControlNode(const rclcpp::NodeOptions & options)
: Node("onboard_control_node", options),
  controller_parameters_(),
  controller_(controller_parameters_)
{
  control_frequency_hz_ = declare_parameter<double>("control_frequency", 100.0);
  status_frequency_hz_ = declare_parameter<double>("status_frequency", 10.0);
  pose_timeout_seconds_ = declare_parameter<double>("pose_timeout_seconds", 0.3);
  state_timeout_seconds_ = declare_parameter<double>("state_timeout_seconds", 2.0);
  link_loss_land_timeout_seconds_ =
    declare_parameter<double>("link_loss_land_timeout_seconds", 10.0);
  takeoff_timeout_seconds_ = declare_parameter<double>("takeoff_timeout_seconds", 45.0);
  waypoint_tolerance_ = declare_parameter<double>("waypoint_tolerance", 0.3);
  waypoint_hold_seconds_ = declare_parameter<double>("waypoint_hold_seconds", 1.0);
  max_velocity_xy_ = declare_parameter<double>("max_velocity_xy", 1.5);
  max_velocity_z_ = declare_parameter<double>("max_velocity_z", 0.8);
  max_yaw_rate_ = declare_parameter<double>("max_yaw_rate", 1.0);
  max_reference_error_xy_ = declare_parameter<double>("max_reference_error_xy", 0.8);
  max_reference_error_z_ = declare_parameter<double>("max_reference_error_z", 0.5);
  max_clock_skew_seconds_ = declare_parameter<double>("max_clock_skew_seconds", 2.0);
  mavros_prefix_ = declare_parameter<std::string>("mavros_prefix", "/mavros");
  interface_prefix_ =
    declare_parameter<std::string>("interface_prefix", "/onboard_control");

  controller_parameters_.wn_xy = declare_parameter<double>("hover_wn_xy", 2.236);
  controller_parameters_.zeta_xy = declare_parameter<double>("hover_zeta_xy", 0.8);
  controller_parameters_.wn_z = declare_parameter<double>("hover_wn_z", 2.236);
  controller_parameters_.zeta_z = declare_parameter<double>("hover_zeta_z", 0.6);
  controller_parameters_.observer_xy = declare_parameter<double>("dob_L_xy", 1.5);
  controller_parameters_.observer_z = declare_parameter<double>("dob_L_z", 0.6);
  controller_parameters_.hover_throttle =
    declare_parameter<double>("hover_throttle", 0.39);
  controller_parameters_.thrust_ratio = declare_parameter<double>("thrust_ratio", 2.5);
  controller_parameters_.gravity = declare_parameter<double>("gravity", 9.8);
  controller_parameters_.max_acceleration_xy =
    declare_parameter<double>("max_acceleration_xy", 4.0);
  controller_parameters_.max_acceleration_z =
    declare_parameter<double>("max_acceleration_z", 4.0);
  controller_parameters_.max_disturbance =
    declare_parameter<double>("max_disturbance", 3.0);
  const double max_tilt_degrees = declare_parameter<double>("max_tilt_degrees", 25.0);
  controller_parameters_.max_tilt_rad = max_tilt_degrees * kPi / 180.0;
  controller_parameters_.min_total_acceleration_z =
    declare_parameter<double>("min_total_acceleration_z", 2.0);

  if (control_frequency_hz_ < 20.0 || control_frequency_hz_ > 400.0 ||
    status_frequency_hz_ <= 0.0 || pose_timeout_seconds_ <= 0.0 ||
    link_loss_land_timeout_seconds_ <= 0.0 || max_velocity_xy_ <= 0.0 ||
    max_velocity_z_ <= 0.0 || controller_parameters_.hover_throttle <= 0.0 ||
    controller_parameters_.hover_throttle >= 1.0 || controller_parameters_.thrust_ratio <= 1.0)
  {
    throw std::invalid_argument("机载控制参数超出安全范围");
  }
  controller_ = DobController(controller_parameters_);

  attitude_topic_ = mavros_prefix_ + "/setpoint_raw/attitude";
  attitude_publisher_ = create_publisher<mavros_msgs::msg::AttitudeTarget>(
    attitude_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).best_effort());
  origin_publisher_ = create_publisher<geographic_msgs::msg::GeoPointStamped>(
    mavros_prefix_ + "/global_position/set_gp_origin", rclcpp::QoS(10).reliable());
  status_publisher_ = create_publisher<guided_interfaces::msg::ControlStatus>(
    interface_prefix_ + "/status", rclcpp::QoS(rclcpp::KeepLast(1)).best_effort());
  result_publisher_ = create_publisher<guided_interfaces::msg::CommandResult>(
    interface_prefix_ + "/command_result", rclcpp::QoS(50).reliable());

  state_subscription_ = create_subscription<mavros_msgs::msg::State>(
    mavros_prefix_ + "/state", rclcpp::QoS(10).reliable(),
    std::bind(&OnboardControlNode::on_fcu_state, this, std::placeholders::_1));
  pose_subscription_ = create_subscription<geometry_msgs::msg::PoseStamped>(
    mavros_prefix_ + "/local_position/pose", rclcpp::SensorDataQoS().keep_last(1),
    std::bind(&OnboardControlNode::on_pose, this, std::placeholders::_1));
  velocity_subscription_ = create_subscription<geometry_msgs::msg::TwistStamped>(
    mavros_prefix_ + "/local_position/velocity_local", rclcpp::SensorDataQoS().keep_last(1),
    std::bind(&OnboardControlNode::on_velocity, this, std::placeholders::_1));
  heartbeat_subscription_ = create_subscription<guided_interfaces::msg::ControlHeartbeat>(
    interface_prefix_ + "/heartbeat", rclcpp::QoS(10).reliable(),
    std::bind(&OnboardControlNode::on_heartbeat, this, std::placeholders::_1));
  motion_subscription_ = create_subscription<guided_interfaces::msg::MotionIntent>(
    interface_prefix_ + "/motion_intent", rclcpp::QoS(20).reliable(),
    std::bind(&OnboardControlNode::on_motion_intent, this, std::placeholders::_1));

  acquire_service_ = create_service<AcquireControl>(
    interface_prefix_ + "/acquire_control",
    std::bind(
      &OnboardControlNode::on_acquire_control, this,
      std::placeholders::_1, std::placeholders::_2));
  flight_command_service_ = create_service<FlightCommand>(
    interface_prefix_ + "/flight_command",
    std::bind(
      &OnboardControlNode::on_flight_command, this,
      std::placeholders::_1, std::placeholders::_2));
  waypoint_service_ = create_service<ExecuteWaypoints>(
    interface_prefix_ + "/execute_waypoints",
    std::bind(
      &OnboardControlNode::on_execute_waypoints, this,
      std::placeholders::_1, std::placeholders::_2));
  origin_service_ = create_service<SetGpsOrigin>(
    interface_prefix_ + "/set_gps_origin",
    std::bind(
      &OnboardControlNode::on_set_gps_origin, this,
      std::placeholders::_1, std::placeholders::_2));

  set_mode_client_ = create_client<mavros_msgs::srv::SetMode>(mavros_prefix_ + "/set_mode");
  arming_client_ = create_client<mavros_msgs::srv::CommandBool>(mavros_prefix_ + "/cmd/arming");
  takeoff_client_ = create_client<mavros_msgs::srv::CommandTOL>(mavros_prefix_ + "/cmd/takeoff");
  message_interval_client_ = create_client<mavros_msgs::srv::MessageInterval>(
    mavros_prefix_ + "/set_message_interval");
  fcu_parameter_client_ = std::make_shared<rclcpp::AsyncParametersClient>(
    this, mavros_prefix_ + "/param");

  const auto control_period = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(1.0 / control_frequency_hz_));
  const auto status_period = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(1.0 / status_frequency_hz_));
  // 独立回调组允许 100 Hz 控制与图查询、服务响应在多线程执行器中并行调度。
  control_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  control_timer_ = create_wall_timer(
    control_period, std::bind(&OnboardControlNode::control_tick, this),
    control_callback_group_);
  status_timer_ = create_wall_timer(status_period, std::bind(&OnboardControlNode::status_tick, this));

  RCLCPP_INFO(
    get_logger(),
    "机载控制服务 %s 启动：控制 %.1f Hz，接口 %s，MAVROS %s",
    kInterfaceVersion, control_frequency_hz_, interface_prefix_.c_str(), mavros_prefix_.c_str());
}

void OnboardControlNode::on_fcu_state(const mavros_msgs::msg::State::SharedPtr message)
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  const bool was_armed = armed_;
  fcu_connected_ = message->connected;
  armed_ = message->armed;
  autopilot_mode_ = message->mode;
  last_state_time_ = SteadyClock::now();
  if (!fcu_connected_) {
    thrust_mode_verified_ = false;
  }

  if (was_armed && !armed_) {
    controller_engaged_ = false;
    active_task_ = ActiveTask::kNone;
    waypoints_.clear();
    reference_.velocity.setZero();
    target_yaw_rate_ = 0.0;
    control_mode_ = guided_interfaces::msg::ControlStatus::MODE_IDLE;
    controller_.reset();
    link_loss_started_.reset();
    failsafe_land_requested_ = false;
    set_status_message("飞行器已解除武装，机载控制回到待机");
  }
}

void OnboardControlNode::on_pose(const geometry_msgs::msg::PoseStamped::SharedPtr message)
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  vehicle_.position = Eigen::Vector3d(
    message->pose.position.x, message->pose.position.y, message->pose.position.z);
  yaw_ = tf2::getYaw(message->pose.orientation);
  pose_valid_ = vehicle_.position.allFinite() && std::isfinite(yaw_);
  last_pose_time_ = SteadyClock::now();
}

void OnboardControlNode::on_velocity(const geometry_msgs::msg::TwistStamped::SharedPtr message)
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  vehicle_.velocity = Eigen::Vector3d(
    message->twist.linear.x, message->twist.linear.y, message->twist.linear.z);
  velocity_valid_ = vehicle_.velocity.allFinite();
  last_velocity_time_ = SteadyClock::now();
}

bool OnboardControlNode::validate_envelope(
  const builtin_interfaces::msg::Time & stamp,
  const std::uint32_t ttl_ms,
  const std::string & source,
  std::string & reason) const
{
  if (source.empty()) {
    reason = "命令来源不能为空";
    return false;
  }
  if (ttl_ms < kMinimumTtlMs || ttl_ms > kMaximumTtlMs) {
    reason = "命令有效期超出允许范围";
    return false;
  }
  const rclcpp::Time sent_time(stamp);
  if (sent_time.nanoseconds() <= 0) {
    reason = "命令时间戳无效";
    return false;
  }
  const double age_seconds = (get_clock()->now() - sent_time).seconds();
  if (age_seconds < -max_clock_skew_seconds_) {
    reason = "命令时间戳来自未来，请检查两端时间同步";
    return false;
  }
  if (age_seconds * 1000.0 > static_cast<double>(ttl_ms)) {
    reason = "命令已过期";
    return false;
  }
  return true;
}

bool OnboardControlNode::lease_active_locked() const
{
  return !lease_owner_.empty() && SteadyClock::now() < lease_deadline_;
}

std::uint32_t OnboardControlNode::lease_remaining_ms_locked() const
{
  if (!lease_active_locked()) {
    return 0;
  }
  const auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(
    lease_deadline_ - SteadyClock::now()).count();
  return static_cast<std::uint32_t>(std::max<std::int64_t>(0, remaining));
}

bool OnboardControlNode::has_control_locked(const std::string & source) const
{
  return lease_active_locked() && lease_owner_ == source;
}

void OnboardControlNode::on_acquire_control(
  const std::shared_ptr<AcquireControl::Request> request,
  std::shared_ptr<AcquireControl::Response> response)
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  std::string reason;
  if (request->lease_duration_ms < kMinimumLeaseMs ||
    request->lease_duration_ms > kMaximumLeaseMs ||
    !validate_envelope(request->stamp, request->lease_duration_ms, request->source_id, reason))
  {
    response->message = reason.empty() ? "租约时长超出允许范围" : reason;
    response->owner = lease_owner_;
    response->lease_remaining_ms = lease_remaining_ms_locked();
    return;
  }

  auto & last_sequence = last_lease_sequence_[request->source_id];
  if (request->sequence <= last_sequence) {
    response->message = "租约请求序号重复或乱序";
    response->owner = lease_owner_;
    response->lease_remaining_ms = lease_remaining_ms_locked();
    return;
  }
  last_sequence = request->sequence;

  if (request->release) {
    if (lease_owner_ != request->source_id) {
      response->message = "当前客户端不是控制权持有者";
    } else {
      lease_owner_.clear();
      lease_deadline_ = SteadyTime{};
      response->granted = true;
      response->message = "控制权已主动释放";
      set_status_message(response->message);
    }
    response->owner = lease_owner_;
    response->lease_remaining_ms = lease_remaining_ms_locked();
    return;
  }

  if (lease_active_locked() && lease_owner_ != request->source_id) {
    response->message = "控制权正由另一客户端持有";
    response->owner = lease_owner_;
    response->lease_remaining_ms = lease_remaining_ms_locked();
    return;
  }

  lease_owner_ = request->source_id;
  lease_deadline_ = SteadyClock::now() +
    std::chrono::milliseconds(request->lease_duration_ms);
  response->granted = true;
  response->owner = lease_owner_;
  response->lease_remaining_ms = lease_remaining_ms_locked();
  response->message = "控制权已授予";
  set_status_message("地面站控制权已授予 " + lease_owner_);
}

void OnboardControlNode::on_heartbeat(
  const guided_interfaces::msg::ControlHeartbeat::SharedPtr message)
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  if (message->lease_duration_ms < kMinimumLeaseMs ||
    message->lease_duration_ms > kMaximumLeaseMs || lease_owner_ != message->source_id)
  {
    return;
  }
  std::string reason;
  if (!validate_envelope(
      message->header.stamp, message->lease_duration_ms, message->source_id, reason))
  {
    return;
  }
  auto & last_sequence = last_lease_sequence_[message->source_id];
  if (message->sequence <= last_sequence) {
    return;
  }
  last_sequence = message->sequence;
  lease_deadline_ = SteadyClock::now() +
    std::chrono::milliseconds(message->lease_duration_ms);
}

bool OnboardControlNode::authorize_flight_sequence(
  const std::string & source,
  const std::uint64_t sequence,
  std::string & reason)
{
  if (!has_control_locked(source)) {
    reason = lease_active_locked() ? "当前客户端没有控制权" : "控制权租约未建立或已过期";
    return false;
  }
  auto & last_sequence = last_flight_sequence_[source];
  if (sequence <= last_sequence) {
    reason = "命令序号重复或乱序";
    return false;
  }
  last_sequence = sequence;
  return true;
}

void OnboardControlNode::on_motion_intent(
  const guided_interfaces::msg::MotionIntent::SharedPtr message)
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  CommandIdentity command{message->source_id, message->sequence, "motion"};
  std::string reason;
  if (!validate_envelope(
      message->header.stamp, message->ttl_ms, message->source_id, reason) ||
    !authorize_flight_sequence(message->source_id, message->sequence, reason))
  {
    publish_result(
      command, guided_interfaces::msg::CommandResult::STATUS_REJECTED, true, reason);
    return;
  }
  if (!armed_ || autopilot_mode_ != "GUIDED" || !pose_valid_) {
    publish_result(
      command, guided_interfaces::msg::CommandResult::STATUS_REJECTED, true,
      "运动意图要求飞行器已武装、处于 GUIDED 且本地位置有效");
    return;
  }
  if (!thrust_mode_verified_) {
    publish_result(
      command, guided_interfaces::msg::CommandResult::STATUS_REJECTED, true,
      "尚未确认 GUID_OPTIONS=8，拒绝姿态/推力控制");
    return;
  }
  const Eigen::Vector3d delta(
    message->velocity_delta.x, message->velocity_delta.y, message->velocity_delta.z);
  if (!delta.allFinite() || !std::isfinite(message->yaw_rate_delta)) {
    publish_result(
      command, guided_interfaces::msg::CommandResult::STATUS_REJECTED, true,
      "运动意图包含非有限数值");
    return;
  }
  if (active_task_ == ActiveTask::kTakeoff || active_task_ == ActiveTask::kLand) {
    publish_result(
      command, guided_interfaces::msg::CommandResult::STATUS_REJECTED, true,
      "起降流程执行期间不接受方向意图");
    return;
  }

  const bool entering_motion =
    control_mode_ != guided_interfaces::msg::ControlStatus::MODE_MOTION;
  cancel_active_task("航点任务已被新的键盘运动意图覆盖");
  if (entering_motion) {
    reference_.position = vehicle_.position;
    reference_.velocity.setZero();
    reference_.yaw = yaw_;
    target_yaw_rate_ = 0.0;
    controller_.reset();
  }

  reference_.velocity += delta;
  const double horizontal_speed = reference_.velocity.head<2>().norm();
  if (horizontal_speed > max_velocity_xy_) {
    reference_.velocity.head<2>() *= max_velocity_xy_ / horizontal_speed;
  }
  reference_.velocity.z() = std::clamp(
    reference_.velocity.z(), -max_velocity_z_, max_velocity_z_);
  target_yaw_rate_ = std::clamp(
    target_yaw_rate_ + message->yaw_rate_delta, -max_yaw_rate_, max_yaw_rate_);
  active_command_ = command;
  control_mode_ = guided_interfaces::msg::ControlStatus::MODE_MOTION;
  controller_engaged_ = true;
  clear_failsafe_locked();

  std::ostringstream stream;
  stream << "运动意图已接受：V=(" << reference_.velocity.x() << ", "
         << reference_.velocity.y() << ", " << reference_.velocity.z()
         << ") m/s, yaw_rate=" << target_yaw_rate_ << " rad/s";
  set_status_message(stream.str());
  publish_result(
    command, guided_interfaces::msg::CommandResult::STATUS_SUCCEEDED, true, stream.str());
}

void OnboardControlNode::on_flight_command(
  const std::shared_ptr<FlightCommand::Request> request,
  std::shared_ptr<FlightCommand::Response> response)
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  std::string reason;
  if (!validate_envelope(request->stamp, request->ttl_ms, request->source_id, reason) ||
    !authorize_flight_sequence(request->source_id, request->sequence, reason))
  {
    response->message = reason;
    return;
  }

  CommandIdentity command{request->source_id, request->sequence, "unknown"};
  switch (request->command) {
    case FlightCommand::Request::COMMAND_TAKEOFF:
      command.name = "takeoff";
      if (!fcu_connected_ || !pose_valid_) {
        response->message = "飞控或本地位置尚未就绪";
        return;
      }
      if (!thrust_mode_verified_) {
        response->message = "尚未确认 GUID_OPTIONS=8，拒绝起飞后切入姿态/推力控制";
        return;
      }
      if (!std::isfinite(request->value) || request->value <= 0.0 || request->value > 20.0) {
        response->message = "起飞高度必须在 (0, 20] 米范围内";
        return;
      }
      start_takeoff(command, request->value);
      response->accepted = true;
      response->message = "起飞命令已由机载服务接收";
      break;
    case FlightCommand::Request::COMMAND_LAND:
      command.name = "land";
      if (!fcu_connected_) {
        response->message = "飞控未连接";
        return;
      }
      start_land(command, false);
      response->accepted = true;
      response->message = "降落命令已由机载服务接收";
      break;
    case FlightCommand::Request::COMMAND_HOVER:
      command.name = "hover";
      if (!armed_ || autopilot_mode_ != "GUIDED" || !pose_valid_) {
        response->message = "悬停要求飞行器已武装、处于 GUIDED 且本地位置有效";
        return;
      }
      if (!thrust_mode_verified_) {
        response->message = "尚未确认 GUID_OPTIONS=8，拒绝姿态/推力控制";
        return;
      }
      cancel_active_task("当前任务已被悬停命令覆盖");
      active_command_ = command;
      enter_hover("PD+DOB 悬停已在机载端接管");
      clear_failsafe_locked();
      publish_result(
        command, guided_interfaces::msg::CommandResult::STATUS_SUCCEEDED, true,
        "PD+DOB 悬停已在机载端接管");
      response->accepted = true;
      response->message = "悬停命令已执行";
      break;
    case FlightCommand::Request::COMMAND_CANCEL:
      command.name = "cancel";
      cancel_active_task("任务已由地面站取消");
      active_command_ = command;
      if (armed_ && autopilot_mode_ == "GUIDED" && pose_valid_) {
        enter_hover("任务已取消，机载端保持当前位置");
      } else {
        control_mode_ = guided_interfaces::msg::ControlStatus::MODE_IDLE;
        controller_engaged_ = false;
      }
      clear_failsafe_locked();
      publish_result(
        command, guided_interfaces::msg::CommandResult::STATUS_SUCCEEDED, true,
        "任务已取消");
      response->accepted = true;
      response->message = "取消命令已执行";
      break;
    case FlightCommand::Request::COMMAND_CONFIGURE_RATES:
      command.name = "set_rates";
      if (!fcu_connected_ || !message_interval_client_->service_is_ready()) {
        response->message = "MAVROS 消息频率服务尚未就绪";
        return;
      }
      if (message_rate_configuration_active_) {
        response->message = "消息频率配置已在执行";
        return;
      }
      start_message_rate_configuration(command);
      response->accepted = true;
      response->message = "消息频率配置已启动";
      break;
    default:
      response->message = "未知飞行命令";
      break;
  }
}

void OnboardControlNode::on_execute_waypoints(
  const std::shared_ptr<ExecuteWaypoints::Request> request,
  std::shared_ptr<ExecuteWaypoints::Response> response)
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  std::string reason;
  if (!validate_envelope(request->stamp, request->ttl_ms, request->source_id, reason) ||
    !authorize_flight_sequence(request->source_id, request->sequence, reason))
  {
    response->message = reason;
    return;
  }
  if (!fcu_connected_ || !pose_valid_) {
    response->message = "飞控或本地位置尚未就绪";
    return;
  }
  if (!thrust_mode_verified_) {
    response->message = "尚未确认 GUID_OPTIONS=8，拒绝航点姿态/推力控制";
    return;
  }
  if (request->waypoints.empty() || request->waypoints.size() > kMaximumWaypoints) {
    response->message = "航点数量必须在 1 到 256 之间";
    return;
  }
  for (const auto & waypoint : request->waypoints) {
    if (!finite_waypoint(waypoint) || waypoint.position.z < 0.1 || waypoint.position.z > 50.0) {
      response->message = "航点包含非法数值，Z 必须在 [0.1, 50] 米范围内";
      return;
    }
  }

  CommandIdentity command{request->source_id, request->sequence, "waypoints"};
  cancel_active_task("旧任务已被新的航点任务覆盖");
  start_waypoint_task(command, request->waypoints);
  response->accepted = true;
  response->message = "航点任务已上传至机载执行器";
}

void OnboardControlNode::on_set_gps_origin(
  const std::shared_ptr<SetGpsOrigin::Request> request,
  std::shared_ptr<SetGpsOrigin::Response> response)
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  std::string reason;
  if (!validate_envelope(request->stamp, request->ttl_ms, request->source_id, reason) ||
    !authorize_flight_sequence(request->source_id, request->sequence, reason))
  {
    response->message = reason;
    return;
  }
  const auto & origin = request->origin;
  if (!std::isfinite(origin.latitude) || !std::isfinite(origin.longitude) ||
    !std::isfinite(origin.altitude) || origin.latitude < -90.0 || origin.latitude > 90.0 ||
    origin.longitude < -180.0 || origin.longitude > 180.0)
  {
    response->message = "GPS 原点经纬高非法";
    return;
  }

  geographic_msgs::msg::GeoPointStamped message;
  message.header.frame_id = "map";
  message.position = origin;
  for (int attempt = 0; attempt < 5; ++attempt) {
    message.header.stamp = get_clock()->now();
    origin_publisher_->publish(message);
  }
  CommandIdentity command{request->source_id, request->sequence, "set_gp_origin"};
  std::ostringstream stream;
  stream << "GPS 原点已由机载端发布 (lat=" << origin.latitude
         << ", lon=" << origin.longitude << ", alt=" << origin.altitude << ")";
  set_status_message(stream.str());
  publish_result(
    command, guided_interfaces::msg::CommandResult::STATUS_SUCCEEDED, true, stream.str());
  response->accepted = true;
  response->message = stream.str();
}

void OnboardControlNode::start_takeoff(
  const CommandIdentity & command, const double altitude)
{
  cancel_active_task("旧任务已被起飞命令覆盖");
  active_command_ = command;
  active_task_ = ActiveTask::kTakeoff;
  active_task_started_ = SteadyClock::now();
  takeoff_altitude_ = altitude;
  control_mode_ = guided_interfaces::msg::ControlStatus::MODE_TAKEOFF;
  controller_engaged_ = true;
  clear_failsafe_locked();
  publish_result(
    command, guided_interfaces::msg::CommandResult::STATUS_RUNNING, false,
    "机载端正在切换 GUIDED、武装并起飞");
  ensure_guided_and_armed(command, [this, command]() {send_takeoff_request(command);});
}

void OnboardControlNode::start_waypoint_task(
  const CommandIdentity & command,
  const std::vector<guided_interfaces::msg::Waypoint> & waypoints)
{
  active_command_ = command;
  active_task_ = ActiveTask::kWaypoint;
  active_task_started_ = SteadyClock::now();
  waypoints_ = waypoints;
  waypoint_index_ = 0;
  waypoint_arrival_started_.reset();
  controller_engaged_ = true;
  clear_failsafe_locked();
  publish_result(
    command, guided_interfaces::msg::CommandResult::STATUS_RUNNING, false,
    "机载端正在准备航点任务", 0, static_cast<std::uint32_t>(waypoints_.size()));

  ensure_guided_and_armed(
    command,
    [this, command]() {
      if (!active_task_matches(command) || waypoints_.empty()) {
        return;
      }
      const auto & waypoint = waypoints_.front();
      reference_.position = Eigen::Vector3d(
        waypoint.position.x, waypoint.position.y, waypoint.position.z);
      reference_.velocity.setZero();
      reference_.yaw = normalize_angle(waypoint.yaw);
      target_yaw_rate_ = 0.0;
      controller_.reset();
      control_mode_ = guided_interfaces::msg::ControlStatus::MODE_WAYPOINT;
      std::ostringstream stream;
      stream << "航点任务已在机载端启动，共 " << waypoints_.size() << " 个航点";
      set_status_message(stream.str());
      publish_result(
        command, guided_interfaces::msg::CommandResult::STATUS_RUNNING, false,
        stream.str(), 1, static_cast<std::uint32_t>(waypoints_.size()));
    });
}

void OnboardControlNode::start_land(const CommandIdentity & command, const bool failsafe)
{
  cancel_active_task(failsafe ? "任务因机载安全保护取消" : "当前任务已被降落命令覆盖");
  active_command_ = command;
  active_task_ = ActiveTask::kLand;
  active_task_started_ = SteadyClock::now();
  control_mode_ = guided_interfaces::msg::ControlStatus::MODE_LAND;
  reference_.velocity.setZero();
  target_yaw_rate_ = 0.0;
  controller_.reset();
  if (!failsafe) {
    clear_failsafe_locked();
  }
  publish_result(
    command, guided_interfaces::msg::CommandResult::STATUS_RUNNING, false,
    failsafe ? "机载失联/健康保护正在切换 LAND" : "机载端正在切换 LAND");
  send_land_mode_request(command, failsafe);
}

void OnboardControlNode::enter_hover(const std::string & reason, const std::uint8_t mode)
{
  reference_.position = vehicle_.position;
  reference_.velocity.setZero();
  reference_.yaw = yaw_;
  target_yaw_rate_ = 0.0;
  control_mode_ = mode;
  controller_engaged_ = armed_;
  controller_.reset();
  set_status_message(reason);
}

void OnboardControlNode::cancel_active_task(const std::string & reason)
{
  if (active_task_ == ActiveTask::kTakeoff || active_task_ == ActiveTask::kWaypoint) {
    publish_result(
      active_command_, guided_interfaces::msg::CommandResult::STATUS_CANCELLED, true,
      reason, static_cast<std::uint32_t>(waypoint_index_ + 1),
      static_cast<std::uint32_t>(waypoints_.size()));
  }
  active_task_ = ActiveTask::kNone;
  waypoints_.clear();
  waypoint_index_ = 0;
  waypoint_arrival_started_.reset();
}

void OnboardControlNode::fail_active_task(const std::string & reason, const bool request_land)
{
  const CommandIdentity failed = active_command_;
  active_task_ = ActiveTask::kNone;
  waypoints_.clear();
  waypoint_arrival_started_.reset();
  publish_result(
    failed, guided_interfaces::msg::CommandResult::STATUS_FAILED, true, reason);
  set_status_message(reason);
  if (request_land && armed_) {
    trigger_failsafe_land(reason);
  } else if (armed_ && pose_valid_ && autopilot_mode_ == "GUIDED") {
    enter_hover("命令失败，机载端保持当前位置");
  } else {
    control_mode_ = guided_interfaces::msg::ControlStatus::MODE_IDLE;
    controller_engaged_ = false;
  }
}

void OnboardControlNode::clear_failsafe_locked()
{
  failsafe_reason_.clear();
  link_loss_started_.reset();
  failsafe_land_requested_ = false;
}

bool OnboardControlNode::active_task_matches(const CommandIdentity & command) const
{
  return active_task_ != ActiveTask::kNone && active_command_.source == command.source &&
         active_command_.sequence == command.sequence;
}

void OnboardControlNode::ensure_guided_and_armed(
  const CommandIdentity & command,
  const std::function<void()> & on_ready)
{
  if (!active_task_matches(command)) {
    return;
  }
  if (autopilot_mode_ == "GUIDED") {
    ensure_armed(command, on_ready);
    return;
  }
  if (!set_mode_client_->service_is_ready()) {
    fail_active_task("MAVROS 模式服务不可用", false);
    return;
  }

  auto request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
  request->custom_mode = "GUIDED";
  set_mode_client_->async_send_request(
    request,
    [this, command, on_ready](rclcpp::Client<mavros_msgs::srv::SetMode>::SharedFuture future) {
      std::lock_guard<std::recursive_mutex> lock(mutex_);
      if (!active_task_matches(command)) {
        return;
      }
      try {
        const auto response = future.get();
        if (!response || !response->mode_sent) {
          fail_active_task("飞控拒绝 GUIDED 模式", false);
          return;
        }
      } catch (const std::exception & error) {
        fail_active_task(std::string("GUIDED 服务异常: ") + error.what(), false);
        return;
      }
      ensure_armed(command, on_ready);
    });
}

void OnboardControlNode::ensure_armed(
  const CommandIdentity & command,
  const std::function<void()> & on_ready)
{
  if (!active_task_matches(command)) {
    return;
  }
  if (armed_) {
    on_ready();
    return;
  }
  if (!arming_client_->service_is_ready()) {
    fail_active_task("MAVROS 武装服务不可用", false);
    return;
  }

  auto request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
  request->value = true;
  arming_client_->async_send_request(
    request,
    [this, command, on_ready](rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedFuture future) {
      std::lock_guard<std::recursive_mutex> lock(mutex_);
      if (!active_task_matches(command)) {
        return;
      }
      try {
        const auto response = future.get();
        if (!response || !response->success) {
          fail_active_task("飞控拒绝武装，请检查 PreArm 状态", false);
          return;
        }
      } catch (const std::exception & error) {
        fail_active_task(std::string("武装服务异常: ") + error.what(), false);
        return;
      }
      on_ready();
    });
}

void OnboardControlNode::send_takeoff_request(const CommandIdentity & command)
{
  if (!active_task_matches(command)) {
    return;
  }
  if (!takeoff_client_->service_is_ready()) {
    fail_active_task("MAVROS 起飞服务不可用", true);
    return;
  }
  auto request = std::make_shared<mavros_msgs::srv::CommandTOL::Request>();
  request->altitude = static_cast<float>(takeoff_altitude_);
  request->min_pitch = 0.0F;
  request->yaw = 0.0F;
  request->latitude = 0.0F;
  request->longitude = 0.0F;
  takeoff_client_->async_send_request(
    request,
    [this, command](rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedFuture future) {
      std::lock_guard<std::recursive_mutex> lock(mutex_);
      if (!active_task_matches(command)) {
        return;
      }
      try {
        const auto response = future.get();
        if (!response || !response->success) {
          fail_active_task("飞控拒绝起飞指令", true);
          return;
        }
      } catch (const std::exception & error) {
        fail_active_task(std::string("起飞服务异常: ") + error.what(), true);
        return;
      }
      active_task_started_ = SteadyClock::now();
      set_status_message("起飞指令已接受，机载端正在确认高度");
    });
}

void OnboardControlNode::send_land_mode_request(
  const CommandIdentity & command, const bool failsafe)
{
  if (!set_mode_client_->service_is_ready()) {
    publish_result(
      command, guided_interfaces::msg::CommandResult::STATUS_FAILED, true,
      "MAVROS 模式服务不可用，无法发送 LAND");
    active_task_ = ActiveTask::kNone;
    if (failsafe) {
      failsafe_land_requested_ = false;
    }
    return;
  }
  auto request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
  request->custom_mode = "LAND";
  set_mode_client_->async_send_request(
    request,
    [this, command, failsafe](rclcpp::Client<mavros_msgs::srv::SetMode>::SharedFuture future) {
      std::lock_guard<std::recursive_mutex> lock(mutex_);
      if (!active_task_matches(command)) {
        return;
      }
      bool success = false;
      std::string failure;
      try {
        const auto response = future.get();
        success = response && response->mode_sent;
        if (!success) {
          failure = "飞控拒绝 LAND 模式";
        }
      } catch (const std::exception & error) {
        failure = std::string("LAND 服务异常: ") + error.what();
      }
      active_task_ = ActiveTask::kNone;
      if (!success) {
        publish_result(
          command, guided_interfaces::msg::CommandResult::STATUS_FAILED, true, failure);
        set_status_message(failure);
        if (failsafe) {
          failsafe_land_requested_ = false;
        }
        return;
      }
      control_mode_ = guided_interfaces::msg::ControlStatus::MODE_LAND;
      set_status_message(failsafe ? "机载安全保护已切换 LAND" : "降落指令已发送 — LAND 模式");
      publish_result(
        command, guided_interfaces::msg::CommandResult::STATUS_SUCCEEDED, true,
        failsafe ? "机载安全保护已切换 LAND" : "降落指令已发送 — LAND 模式");
    });
}

void OnboardControlNode::start_message_rate_configuration(const CommandIdentity & command)
{
  message_rate_configuration_active_ = true;
  message_rates_configured_ = false;
  message_rate_command_ = command;
  message_rate_index_ = 0;
  publish_result(
    command, guided_interfaces::msg::CommandResult::STATUS_RUNNING, false,
    "机载端正在配置 MAVLink 消息频率");
  send_next_message_rate();
}

void OnboardControlNode::send_next_message_rate()
{
  if (!message_rate_configuration_active_) {
    return;
  }
  if (message_rate_index_ >= kMessageIntervals.size()) {
    message_rate_configuration_active_ = false;
    message_rates_configured_ = true;
    publish_result(
      message_rate_command_, guided_interfaces::msg::CommandResult::STATUS_SUCCEEDED, true,
      "MAVLink 本地位置/姿态/IMU 消息频率已配置为 100 Hz");
    set_status_message("MAVLink 高频遥测配置完成");
    return;
  }

  const auto [message_id, rate] = kMessageIntervals[message_rate_index_];
  auto request = std::make_shared<mavros_msgs::srv::MessageInterval::Request>();
  request->message_id = message_id;
  request->message_rate = rate;
  message_interval_client_->async_send_request(
    request,
    [this, message_id](rclcpp::Client<mavros_msgs::srv::MessageInterval>::SharedFuture future) {
      std::lock_guard<std::recursive_mutex> lock(mutex_);
      if (!message_rate_configuration_active_) {
        return;
      }
      bool success = false;
      try {
        const auto response = future.get();
        success = response && response->success;
      } catch (const std::exception &) {
        success = false;
      }
      if (!success) {
        message_rate_configuration_active_ = false;
        std::ostringstream stream;
        stream << "飞控拒绝消息 " << message_id << " 的频率配置";
        publish_result(
          message_rate_command_, guided_interfaces::msg::CommandResult::STATUS_FAILED,
          true, stream.str());
        set_status_message(stream.str());
        return;
      }
      ++message_rate_index_;
      send_next_message_rate();
    });
}

void OnboardControlNode::check_thrust_mode_parameter()
{
  if (thrust_mode_check_inflight_ || !fcu_parameter_client_->service_is_ready()) {
    return;
  }
  thrust_mode_check_inflight_ = true;
  last_thrust_mode_check_ = SteadyClock::now();
  fcu_parameter_client_->get_parameters(
    {"GUID_OPTIONS", "MOT_THST_HOVER"},
    [this](std::shared_future<std::vector<rclcpp::Parameter>> future) {
      std::lock_guard<std::recursive_mutex> lock(mutex_);
      thrust_mode_check_inflight_ = false;
      try {
        const auto parameters = future.get();
        if (parameters.size() != 2U || parameters.front().get_type() !=
          rclcpp::ParameterType::PARAMETER_INTEGER || parameters[1].get_type() !=
          rclcpp::ParameterType::PARAMETER_DOUBLE)
        {
          // 已验证后的单次参数读取故障不应在飞行中制造伪故障；FCU 断开会单独撤销验证。
          if (!thrust_mode_verified_) {
            set_status_message(
              "无法同时读取 GUID_OPTIONS 与 MOT_THST_HOVER，姿态/推力控制保持禁用");
          }
          return;
        }
        const bool was_verified = thrust_mode_verified_;
        const std::int64_t options = parameters.front().as_int();
        const double hover_throttle = parameters[1].as_double();
        const bool hover_valid = controller_.set_hover_throttle(hover_throttle);
        if (hover_valid) {
          controller_parameters_.hover_throttle = hover_throttle;
        }
        thrust_mode_verified_ = (options & 8) != 0 && hover_valid;
        if (thrust_mode_verified_ && !was_verified) {
          std::ostringstream stream;
          stream << "已确认 GUID_OPTIONS bit 3，并同步 MOT_THST_HOVER="
                 << hover_throttle;
          set_status_message(stream.str());
        } else if ((options & 8) == 0) {
          set_status_message("GUID_OPTIONS bit 3 未启用：请设置 GUID_OPTIONS=8");
        } else if (!hover_valid) {
          set_status_message("飞控 MOT_THST_HOVER 非法，姿态/推力控制保持禁用");
        }
      } catch (const std::exception & error) {
        if (!thrust_mode_verified_) {
          set_status_message(std::string("读取飞控推力参数失败: ") + error.what());
        }
      }
    });
}

void OnboardControlNode::control_tick()
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  const SteadyTime now = SteadyClock::now();
  const double expected_period = 1.0 / control_frequency_hz_;
  double dt_seconds = expected_period;
  if (last_control_tick_ != SteadyTime{}) {
    dt_seconds = std::chrono::duration<double>(now - last_control_tick_).count();
    if (dt_seconds > 0.0) {
      const double instantaneous_rate = 1.0 / dt_seconds;
      measured_control_rate_hz_ = measured_control_rate_hz_ == 0.0 ? instantaneous_rate :
        0.98 * measured_control_rate_hz_ + 0.02 * instantaneous_rate;
      const double jitter_ms = std::abs(dt_seconds - expected_period) * 1000.0;
      max_jitter_ms_ = std::max(max_jitter_ms_, jitter_ms);
      if (dt_seconds > expected_period * 1.5) {
        ++deadline_miss_count_;
      }
    }
  }
  last_control_tick_ = now;

  enforce_safety(now);

  if (active_task_ == ActiveTask::kTakeoff) {
    const double threshold = std::max(0.1, takeoff_altitude_ - 0.1);
    if (armed_ && pose_valid_ && vehicle_.position.z() >= threshold) {
      const CommandIdentity completed = active_command_;
      active_task_ = ActiveTask::kNone;
      enter_hover("起飞完成，机载 PD+DOB 已进入悬停");
      // 起飞服务只负责把飞机送入安全高度，最终请求高度仍由同一 PD+DOB 精确保持。
      reference_.position.z() = takeoff_altitude_;
      publish_result(
        completed, guided_interfaces::msg::CommandResult::STATUS_SUCCEEDED, true,
        "起飞成功，机载 PD+DOB 已进入悬停");
    } else if (
      std::chrono::duration<double>(now - active_task_started_).count() >
      takeoff_timeout_seconds_)
    {
      fail_active_task("起飞高度确认超时", true);
    }
  }

  if (control_mode_ == guided_interfaces::msg::ControlStatus::MODE_MOTION) {
    update_motion_reference(dt_seconds);
  } else if (control_mode_ == guided_interfaces::msg::ControlStatus::MODE_WAYPOINT) {
    update_waypoint_executor(now);
  }

  if (control_mode_ == guided_interfaces::msg::ControlStatus::MODE_MOTION ||
    control_mode_ == guided_interfaces::msg::ControlStatus::MODE_HOVER ||
    control_mode_ == guided_interfaces::msg::ControlStatus::MODE_WAYPOINT ||
    control_mode_ == guided_interfaces::msg::ControlStatus::MODE_FAILSAFE_HOLD)
  {
    publish_attitude_setpoint(dt_seconds);
  }
}

void OnboardControlNode::update_motion_reference(const double dt_seconds)
{
  reference_.position += reference_.velocity * dt_seconds;
  reference_.yaw = normalize_angle(reference_.yaw + target_yaw_rate_ * dt_seconds);

  // 虚拟目标不允许无限跑离机体，以免网络抖动后产生过大的追赶加速度。
  Eigen::Vector2d horizontal_error =
    reference_.position.head<2>() - vehicle_.position.head<2>();
  if (horizontal_error.norm() > max_reference_error_xy_) {
    horizontal_error *= max_reference_error_xy_ / horizontal_error.norm();
    reference_.position.x() = vehicle_.position.x() + horizontal_error.x();
    reference_.position.y() = vehicle_.position.y() + horizontal_error.y();
  }
  reference_.position.z() = std::clamp(
    reference_.position.z(),
    vehicle_.position.z() - max_reference_error_z_,
    vehicle_.position.z() + max_reference_error_z_);
  reference_.position.z() = std::max(reference_.position.z(), 0.1);
}

void OnboardControlNode::update_waypoint_executor(const SteadyTime & now)
{
  if (active_task_ != ActiveTask::kWaypoint || waypoint_index_ >= waypoints_.size()) {
    return;
  }
  const auto & waypoint = waypoints_[waypoint_index_];
  const Eigen::Vector3d target(
    waypoint.position.x, waypoint.position.y, waypoint.position.z);
  reference_.position = target;
  reference_.velocity.setZero();
  reference_.yaw = normalize_angle(waypoint.yaw);
  target_yaw_rate_ = 0.0;

  const double distance = (vehicle_.position - target).norm();
  if (distance >= waypoint_tolerance_) {
    waypoint_arrival_started_.reset();
    return;
  }
  if (!waypoint_arrival_started_.has_value()) {
    waypoint_arrival_started_ = now;
    return;
  }
  if (std::chrono::duration<double>(now - *waypoint_arrival_started_).count() <
    waypoint_hold_seconds_)
  {
    return;
  }

  ++waypoint_index_;
  waypoint_arrival_started_.reset();
  if (waypoint_index_ >= waypoints_.size()) {
    const CommandIdentity completed = active_command_;
    const std::uint32_t count = static_cast<std::uint32_t>(waypoints_.size());
    active_task_ = ActiveTask::kNone;
    waypoint_index_ = waypoints_.size() - 1;
    control_mode_ = guided_interfaces::msg::ControlStatus::MODE_HOVER;
    reference_.velocity.setZero();
    target_yaw_rate_ = 0.0;
    set_status_message("航点任务完成，机载端保持末航点");
    publish_result(
      completed, guided_interfaces::msg::CommandResult::STATUS_SUCCEEDED, true,
      "航点任务完成，机载端保持末航点", count, count);
    return;
  }

  const auto & next = waypoints_[waypoint_index_];
  std::ostringstream stream;
  stream << "前往航点 " << waypoint_index_ + 1 << "/" << waypoints_.size()
         << " (" << next.position.x << ", " << next.position.y << ", "
         << next.position.z << ")";
  set_status_message(stream.str());
  publish_result(
    active_command_, guided_interfaces::msg::CommandResult::STATUS_RUNNING, false,
    stream.str(), static_cast<std::uint32_t>(waypoint_index_ + 1),
    static_cast<std::uint32_t>(waypoints_.size()));
}

void OnboardControlNode::enforce_safety(const SteadyTime & now)
{
  if (!controller_engaged_ || !armed_) {
    return;
  }

  const bool raw_control_mode =
    control_mode_ == guided_interfaces::msg::ControlStatus::MODE_MOTION ||
    control_mode_ == guided_interfaces::msg::ControlStatus::MODE_HOVER ||
    control_mode_ == guided_interfaces::msg::ControlStatus::MODE_WAYPOINT ||
    control_mode_ == guided_interfaces::msg::ControlStatus::MODE_FAILSAFE_HOLD;

  if (setpoint_conflict_ && raw_control_mode) {
    trigger_failsafe_land("检测到多个姿态 setpoint 发布者");
    return;
  }
  if (raw_control_mode &&
    (!fcu_connected_ || last_state_time_ == SteadyTime{} ||
    std::chrono::duration<double>(now - last_state_time_).count() > state_timeout_seconds_))
  {
    trigger_failsafe_land("飞控状态遥测超时或连接中断");
    return;
  }
  if (!thrust_mode_verified_ && raw_control_mode) {
    trigger_failsafe_land("GUID_OPTIONS bit 3 校验失效");
    return;
  }
  if (raw_control_mode &&
    (!pose_valid_ || !velocity_valid_ || last_pose_time_ == SteadyTime{} ||
    last_velocity_time_ == SteadyTime{} ||
    std::chrono::duration<double>(now - last_pose_time_).count() > pose_timeout_seconds_ ||
    std::chrono::duration<double>(now - last_velocity_time_).count() > pose_timeout_seconds_))
  {
    trigger_failsafe_land("本地位置或速度遥测无效/超时");
    return;
  }
  if (raw_control_mode && autopilot_mode_ != "GUIDED") {
    controller_engaged_ = false;
    control_mode_ = guided_interfaces::msg::ControlStatus::MODE_IDLE;
    active_task_ = ActiveTask::kNone;
    controller_.reset();
    failsafe_reason_ = "飞控模式被外部切换，机载服务已停止发送 setpoint";
    set_status_message(failsafe_reason_);
    return;
  }

  if (!lease_active_locked()) {
    if (!link_loss_started_.has_value()) {
      trigger_link_loss_hold(now);
    }
    if (link_loss_started_.has_value() &&
      std::chrono::duration<double>(now - *link_loss_started_).count() >=
      link_loss_land_timeout_seconds_)
    {
      trigger_failsafe_land("地面站控制租约超时，悬停等待期结束");
    }
  }
}

void OnboardControlNode::trigger_link_loss_hold(const SteadyTime & now)
{
  cancel_active_task("任务因地面站控制租约超时而取消");
  link_loss_started_ = now;
  failsafe_reason_ = "地面站控制租约超时";
  if (pose_valid_ && autopilot_mode_ == "GUIDED") {
    enter_hover(
      "地面站失联：机载端独立悬停，超时后 LAND",
      guided_interfaces::msg::ControlStatus::MODE_FAILSAFE_HOLD);
  } else {
    trigger_failsafe_land(failsafe_reason_);
  }
}

void OnboardControlNode::trigger_failsafe_land(const std::string & reason)
{
  if (failsafe_land_requested_ || control_mode_ == guided_interfaces::msg::ControlStatus::MODE_LAND) {
    return;
  }
  failsafe_reason_ = reason;
  failsafe_land_requested_ = true;
  const auto sequence = static_cast<std::uint64_t>(get_clock()->now().nanoseconds());
  CommandIdentity command{"onboard-failsafe", sequence, "failsafe_land"};
  start_land(command, true);
}

void OnboardControlNode::publish_attitude_setpoint(const double dt_seconds)
{
  const SteadyTime now = SteadyClock::now();
  if (!armed_ || autopilot_mode_ != "GUIDED" || !pose_valid_ || !velocity_valid_ ||
    last_pose_time_ == SteadyTime{} || last_velocity_time_ == SteadyTime{} ||
    std::chrono::duration<double>(now - last_pose_time_).count() > pose_timeout_seconds_ ||
    std::chrono::duration<double>(now - last_velocity_time_).count() > pose_timeout_seconds_)
  {
    return;
  }

  const ControlOutput output = controller_.compute(vehicle_, reference_, dt_seconds);
  if (!output.valid) {
    trigger_failsafe_land("PD+DOB 产生非有限控制输出");
    return;
  }
  mavros_msgs::msg::AttitudeTarget message;
  message.header.stamp = get_clock()->now();
  message.orientation.x = output.attitude.x();
  message.orientation.y = output.attitude.y();
  message.orientation.z = output.attitude.z();
  message.orientation.w = output.attitude.w();
  message.thrust = static_cast<float>(output.thrust);
  message.type_mask =
    mavros_msgs::msg::AttitudeTarget::IGNORE_ROLL_RATE |
    mavros_msgs::msg::AttitudeTarget::IGNORE_PITCH_RATE |
    mavros_msgs::msg::AttitudeTarget::IGNORE_YAW_RATE;
  attitude_publisher_->publish(message);
}

void OnboardControlNode::status_tick()
{
  // ROS 图查询可能发生调度抖动，必须放在控制状态互斥锁之外。
  const std::size_t publisher_count = count_publishers(attitude_topic_);
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  const SteadyTime now = SteadyClock::now();
  if (fcu_connected_ && !thrust_mode_check_inflight_ &&
    (last_thrust_mode_check_ == SteadyTime{} ||
    std::chrono::duration<double>(now - last_thrust_mode_check_).count() >= 5.0))
  {
    check_thrust_mode_parameter();
  }
  if (publisher_count > 1U) {
    ++conflict_observations_;
  } else {
    conflict_observations_ = 0;
  }
  setpoint_conflict_ = conflict_observations_ >= 3U;

  guided_interfaces::msg::ControlStatus message;
  message.header.stamp = get_clock()->now();
  message.interface_version = kInterfaceVersion;
  message.fcu_connected = fcu_connected_ && last_state_time_ != SteadyTime{} &&
    std::chrono::duration<double>(now - last_state_time_).count() <= state_timeout_seconds_;
  message.armed = armed_;
  message.autopilot_mode = autopilot_mode_;
  message.local_position_valid = pose_valid_ && velocity_valid_ &&
    last_pose_time_ != SteadyTime{} && last_velocity_time_ != SteadyTime{} &&
    std::chrono::duration<double>(now - last_pose_time_).count() <= pose_timeout_seconds_ &&
    std::chrono::duration<double>(now - last_velocity_time_).count() <= pose_timeout_seconds_;
  message.position.x = vehicle_.position.x();
  message.position.y = vehicle_.position.y();
  message.position.z = vehicle_.position.z();
  message.velocity.x = vehicle_.velocity.x();
  message.velocity.y = vehicle_.velocity.y();
  message.velocity.z = vehicle_.velocity.z();
  message.yaw = yaw_;
  message.control_mode = control_mode_;
  message.control_mode_label = mode_label(control_mode_);
  message.controller_active = controller_engaged_;
  message.target_position.x = reference_.position.x();
  message.target_position.y = reference_.position.y();
  message.target_position.z = reference_.position.z();
  message.target_velocity.x = reference_.velocity.x();
  message.target_velocity.y = reference_.velocity.y();
  message.target_velocity.z = reference_.velocity.z();
  message.target_yaw = reference_.yaw;
  message.target_yaw_rate = target_yaw_rate_;
  message.lease_owner = lease_owner_;
  message.lease_active = lease_active_locked();
  message.lease_remaining_ms = lease_remaining_ms_locked();
  message.active_command_sequence = active_command_.sequence;
  message.waypoint_index = waypoints_.empty() ? 0U :
    static_cast<std::uint32_t>(std::min(waypoint_index_ + 1, waypoints_.size()));
  message.waypoint_count = static_cast<std::uint32_t>(waypoints_.size());
  message.message_rates_configured = message_rates_configured_;
  message.thrust_mode_verified = thrust_mode_verified_;
  message.hover_throttle = controller_.hover_throttle();
  message.setpoint_conflict = setpoint_conflict_;
  message.failsafe_reason = failsafe_reason_;
  message.status_message = status_message_;
  message.control_rate_hz = measured_control_rate_hz_;
  message.max_jitter_ms = max_jitter_ms_;
  message.deadline_miss_count = deadline_miss_count_;
  status_publisher_->publish(message);
}

void OnboardControlNode::publish_result(
  const CommandIdentity & command,
  const std::uint8_t status,
  const bool final,
  const std::string & message,
  const std::uint32_t waypoint_index,
  const std::uint32_t waypoint_count)
{
  guided_interfaces::msg::CommandResult result;
  result.header.stamp = get_clock()->now();
  result.source_id = command.source;
  result.sequence = command.sequence;
  result.command = command.name;
  result.status = status;
  result.final = final;
  result.message = message;
  result.waypoint_index = waypoint_index;
  result.waypoint_count = waypoint_count;
  result_publisher_->publish(result);
}

void OnboardControlNode::set_status_message(const std::string & message)
{
  status_message_ = message;
  RCLCPP_INFO(get_logger(), "%s", message.c_str());
}

std::string OnboardControlNode::mode_label(const std::uint8_t mode)
{
  switch (mode) {
    case guided_interfaces::msg::ControlStatus::MODE_TAKEOFF:
      return "起飞";
    case guided_interfaces::msg::ControlStatus::MODE_MOTION:
      return "键盘运动 PD+DOB";
    case guided_interfaces::msg::ControlStatus::MODE_HOVER:
      return "悬停 PD+DOB";
    case guided_interfaces::msg::ControlStatus::MODE_WAYPOINT:
      return "航点 PD+DOB";
    case guided_interfaces::msg::ControlStatus::MODE_LAND:
      return "降落";
    case guided_interfaces::msg::ControlStatus::MODE_FAILSAFE_HOLD:
      return "失联保护悬停";
    default:
      return "待机";
  }
}

double OnboardControlNode::normalize_angle(double angle)
{
  while (angle > kPi) {
    angle -= 2.0 * kPi;
  }
  while (angle < -kPi) {
    angle += 2.0 * kPi;
  }
  return angle;
}

}  // namespace onboard_control
