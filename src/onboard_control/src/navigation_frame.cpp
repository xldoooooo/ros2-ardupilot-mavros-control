/** @file navigation_frame.cpp
 * @brief Observe Odin localization, adapt immutable map missions, and guard mode restarts.
 */
#include "onboard_control/onboard_control_node.hpp"

#include <fcntl.h>
#include <spawn.h>
#include <sys/wait.h>
#include <unistd.h>
#include <algorithm>
#include <filesystem>
#include <fstream>
#include <cstdlib>
#include <cerrno>
#include <limits>
#include <thread>
#include <nlohmann/json.hpp>

extern char ** environ;

namespace onboard_control
{
namespace
{
// Odin status is 2 Hz; extnav status has the same cadence, independent of FCU telemetry.
constexpr double kExtnavStatusTimeout = 3.0;

/** Match the restart script's atomic failure marker without creating another ROS participant. */
std::string switch_result_path()
{
  const char * path = std::getenv("NAVIGATION_SWITCH_RESULT_FILE");
  return path ? path : "/tmp/ros2_ardupilot_navigation_switch.result";
}

/** Convert an Eigen point to an existing message field without changing its frame. */
template<typename Point>
void write_vector(Point & point, const Eigen::Vector3d & value)
{
  point.x = value.x(); point.y = value.y(); point.z = value.z();
}

/** Keep map identity explicit across Odin restarts and operator mode changes. */
std::string navigation_identity(const std::string & frame, const std::string & map, const std::string & session)
{
  return frame + "\n" + map + "\n" + session;
}
}  // namespace

/** Create navigation observations, restart service and correction limits. */
void OnboardControlNode::initialize_navigation()
{
  navigation_switch_script_ = declare_parameter<std::string>("navigation_switch_script", "");
  navigation_status_timeout_ = declare_parameter<double>("navigation_status_timeout_seconds", 3.0);
  navigation_tf_timeout_ = declare_parameter<double>("navigation_tf_timeout_seconds", 5.0);
  navigation_max_jump_m_ = declare_parameter<double>("navigation_max_jump_m", 2.0);
  navigation_max_jump_rad_ = declare_parameter<double>("navigation_max_jump_degrees", 45.0) * std::acos(-1.0) / 180.0;
  navigation_hold_timeout_ = declare_parameter<double>("navigation_hold_timeout_seconds", 15.0);
  navigation_retarget_distance_ = declare_parameter<double>("navigation_retarget_distance_m", 0.05);
  navigation_retarget_yaw_ = declare_parameter<double>("navigation_retarget_degrees", 2.0) * std::acos(-1.0) / 180.0;
  navigation_retarget_interval_ = declare_parameter<double>("navigation_retarget_interval_seconds", 1.0);
  navigation_retarget_blend_ = declare_parameter<double>("navigation_retarget_blend_seconds", 1.0);
  for (double value : {navigation_status_timeout_, navigation_tf_timeout_, navigation_max_jump_m_,
    navigation_max_jump_rad_, navigation_hold_timeout_, navigation_retarget_distance_,
    navigation_retarget_yaw_, navigation_retarget_interval_, navigation_retarget_blend_})
  {
    if (!std::isfinite(value) || value <= 0.0) {
      throw std::invalid_argument("navigation timeouts and correction limits must be finite and positive");
    }
  }
  navigation_status_subscription_ = create_subscription<std_msgs::msg::String>(
    "/odin1/navigation_status", rclcpp::QoS(1).reliable().transient_local(),
    std::bind(&OnboardControlNode::on_navigation_status, this, std::placeholders::_1));
  navigation_tf_subscription_ = create_subscription<tf2_msgs::msg::TFMessage>(
    "/tf", rclcpp::SensorDataQoS(),
    std::bind(&OnboardControlNode::on_navigation_tf, this, std::placeholders::_1));
  navigation_local_tf_publisher_ = create_publisher<tf2_msgs::msg::TFMessage>(
    "/tf_static", rclcpp::QoS(1).reliable().transient_local());
  navigation_extnav_subscription_ = create_subscription<correction_interfaces::msg::ExtnavCorrectionStatus>(
    "/extnav/correction_status", rclcpp::QoS(10),
    [this](correction_interfaces::msg::ExtnavCorrectionStatus::SharedPtr message) {
      std::lock_guard<std::recursive_mutex> lock(mutex_);
      navigation_extnav_ = *message;
      navigation_extnav_time_ = SteadyClock::now();
      if (navigation_frame_ == "map" && !message->correction_valid &&
        message->final_sample_reference_mode == "local_identity_origin" &&
        std::isfinite(message->lever_arm_x_m) && std::isfinite(message->lever_arm_y_m) &&
        std::isfinite(message->lever_arm_z_m))
      {
        // Odin owns the real map TF. Give FCU-local previews their own unambiguous frame.
        geometry_msgs::msg::TransformStamped transform;
        transform.header.stamp = get_clock()->now();
        transform.header.frame_id = "odom";
        transform.child_frame_id = "fcu_local";
        transform.transform.translation.x = -message->lever_arm_x_m;
        transform.transform.translation.y = -message->lever_arm_y_m;
        transform.transform.translation.z = -message->lever_arm_z_m;
        transform.transform.rotation.w = 1.0;
        tf2_msgs::msg::TFMessage tf;
        tf.transforms.push_back(transform);
        navigation_local_tf_publisher_->publish(tf);
      }
    });
  navigation_mode_service_ = create_service<SetNavigationMode>(
    interface_prefix_ + "/set_navigation_mode",
    std::bind(&OnboardControlNode::on_set_navigation_mode, this, std::placeholders::_1, std::placeholders::_2));
}

/** Track the actual driver mode and connection identity. */
void OnboardControlNode::on_navigation_status(const std_msgs::msg::String::SharedPtr message)
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  try {
    const auto status = nlohmann::json::parse(message->data);
    const std::string mode = status.at("mode").get<std::string>();
    const std::string session = status.at("session_id").get<std::string>();
    const std::string map = status.at("map_path").get<std::string>();
    if ((mode != "odom" && mode != "map" && mode != "unsupported") || session.empty()) {
      throw std::invalid_argument("invalid Odin navigation identity");
    }
    const bool changed = navigation_identity(mode, map, session) !=
      navigation_identity(navigation_frame_, navigation_map_, navigation_session_);
    if (changed && navigation_driver_seen_) {
      if (armed_ || airborne_ || active_task_ != ActiveTask::kNone) {
        navigation_fault_ = "Odin 工作模式、地图或设备会话在飞行任务中改变，请落地后重新连接";
      } else {
        navigation_fault_.clear();
        navigation_hold_started_.reset();
      }
      navigation_tf_valid_ = false;
    }
    if (armed_navigation_identity_ &&
      *armed_navigation_identity_ != navigation_identity(mode, map, session))
    {
      navigation_fault_ = "飞行中禁止改变 Odin 工作模式、地图或设备会话";
    }
    navigation_frame_ = mode;
    navigation_map_ = map;
    navigation_session_ = session;
    navigation_localized_ = status.at("localized").get<bool>() && status.value("connected", true);
    const auto & tf_age = status.at("tf_age_seconds");
    navigation_driver_tf_age_ = tf_age.is_number() ? tf_age.get<double>() : std::numeric_limits<double>::infinity();
    if (!std::isfinite(navigation_driver_tf_age_) || navigation_driver_tf_age_ < 0.0) {
      navigation_driver_tf_age_ = std::numeric_limits<double>::infinity();
    }
    navigation_driver_seen_ = true;
    navigation_driver_time_ = SteadyClock::now();
  } catch (const std::exception & error) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Invalid Odin navigation status: %s", error.what());
  }
}

/** Accept finite native map transforms and detect discontinuities. */
void OnboardControlNode::on_navigation_tf(const tf2_msgs::msg::TFMessage::SharedPtr message)
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  for (const auto & tf : message->transforms) {
    if (tf.header.frame_id != "odom" || tf.child_frame_id != "map") {continue;}
    // An old external driver with a map TF is detectable, but lacks localization health evidence.
    if (!navigation_driver_seen_) {navigation_frame_ = "map";}
    if (navigation_frame_ != "map") {continue;}
    const auto & q = tf.transform.rotation;
    const auto & p = tf.transform.translation;
    NavigationTransform candidate{{q.w, q.x, q.y, q.z}, {p.x, p.y, p.z}};
    if (!candidate.normalize()) {
      navigation_tf_valid_ = false;
      continue;
    }
    if (navigation_tf_valid_) {
      Eigen::Vector3d local_origin = Eigen::Vector3d::Zero();
      if (navigation_extnav_) {
        local_origin = {navigation_extnav_->lever_arm_x_m, navigation_extnav_->lever_arm_y_m,
          navigation_extnav_->lever_arm_z_m};
      }
      const Eigen::Vector3d odom_position = vehicle_.position - local_origin;
      const Eigen::Vector3d map_position = odom_from_map_.inverse().point(odom_position);
      const double jump = (candidate.point(map_position) - odom_position).norm();
      const double rotation_jump = candidate.rotation.angularDistance(odom_from_map_.rotation);
      if (jump > navigation_max_jump_m_ || rotation_jump > navigation_max_jump_rad_) {
        navigation_fault_ = "Odin 重定位修正跳变超过配置范围，请落地后重新确认定位";
        navigation_tf_valid_ = false;
        continue;
      }
    }
    if (!armed_ && active_task_ == ActiveTask::kNone) {navigation_fault_.clear();}
    odom_from_map_ = candidate;
    navigation_tf_time_ = SteadyClock::now();
    navigation_tf_valid_ = true;
  }
}

/** Explain whether the selected task frame can currently be used. */
bool OnboardControlNode::navigation_ready(const SteadyTime & now, std::string & reason) const
{
  if (navigation_switch_pending_) {reason = "定位模式正在重启切换，等待机载服务重新连接"; return false;}
  if (!navigation_fault_.empty()) {reason = navigation_fault_; return false;}
  if (navigation_hold_started_) {reason = "定位保护已取消任务，请降落并解除武装后重新确认"; return false;}
  if (navigation_frame_ == "odom") {reason = "odom 本地任务坐标"; return true;}
  if (navigation_frame_ != "map") {reason = "当前 Odin 工作模式不支持飞行，请选择 odom 或重定位"; return false;}
  const auto age = [&now](SteadyTime time) {return std::chrono::duration<double>(now - time).count();};
  if (!navigation_driver_seen_ || age(navigation_driver_time_) > navigation_status_timeout_) {
    reason = "等待 Odin 重定位运行状态"; return false;
  }
  if (navigation_map_.empty() || !navigation_localized_) {reason = "Odin 地图尚未定位成功"; return false;}
  if (!navigation_tf_valid_ || age(navigation_tf_time_) > navigation_tf_timeout_ ||
    navigation_driver_tf_age_ + age(navigation_driver_time_) > navigation_tf_timeout_)
  {
    reason = "Odin 地图定位关系无效或超时"; return false;
  }
  // TODO(odom-map-tag): compose Tag correction and revalidate map/avoidance before enabling it.
  if (!navigation_extnav_ || age(navigation_extnav_time_) > kExtnavStatusTimeout ||
    navigation_extnav_->interface_version != "2.0" || !navigation_extnav_->service_available ||
    !navigation_extnav_->odin_available || navigation_extnav_->odin_session_id.empty() ||
    !navigation_extnav_->final_sample_available || navigation_extnav_->correction_valid ||
    navigation_extnav_->final_sample_correction_valid ||
    navigation_extnav_->final_sample_revision != navigation_extnav_->revision ||
    navigation_extnav_->final_sample_odin_session_id != navigation_extnav_->odin_session_id ||
    navigation_extnav_->final_sample_reference_mode != "local_identity_origin")
  {
    reason = "map 需要有效的原始 extnav 坐标；本次尚不支持 Odin-Tag 修正"; return false;
  }
  for (double value : {navigation_extnav_->lever_arm_x_m, navigation_extnav_->lever_arm_y_m,
    navigation_extnav_->lever_arm_z_m})
  {
    if (!std::isfinite(value)) {reason = "Odin 外参原点平移无效"; return false;}
  }
  reason = "map 重定位就绪";
  return true;
}

/** Convert an immutable task waypoint into the continuous FCU local frame. */
guided_interfaces::msg::Waypoint OnboardControlNode::local_waypoint(
  const guided_interfaces::msg::Waypoint & waypoint) const
{
  auto local = waypoint;
  if (navigation_frame_ != "map" || waypoint_coordinates_local_) {return local;}
  write_vector(local.position, task_to_local_.point({waypoint.position.x, waypoint.position.y, waypoint.position.z}));
  local.yaw = task_to_local_.yaw(waypoint.yaw);
  return local;
}

/** Retarget ordinary corrections or brake locally after localization failure. */
void OnboardControlNode::navigation_tick(const SteadyTime & now, const double dt_seconds)
{
  if (navigation_switch_pending_) {
    std::ifstream result(switch_result_path());
    std::string failure;
    if (std::getline(result, failure) && !failure.empty()) {
      navigation_switch_pending_ = false;
      set_status_message("定位切换失败：" + failure, StatusLogLevel::kError);
    }
  }
  task_to_local_ = NavigationTransform{};
  if (navigation_frame_ == "map" && navigation_tf_valid_ && navigation_extnav_) {
    task_to_local_ = odom_from_map_;
    // b=T_FI is the existing local origin offset, not the current rotated physical lever arm.
    task_to_local_.translation += Eigen::Vector3d(navigation_extnav_->lever_arm_x_m,
      navigation_extnav_->lever_arm_y_m, navigation_extnav_->lever_arm_z_m);
  }
  std::string reason;
  bool ready = navigation_ready(now, reason);
  if (ready && navigation_frame_ == "map" && !waypoint_coordinates_local_ &&
    active_task_ == ActiveTask::kWaypoint && !task_waypoints_.empty())
  {
    for (std::size_t index = 0; index < task_waypoints_.size(); ++index) {
      const auto local = local_waypoint(task_waypoints_[index]);
      if (index >= waypoint_index_ && (local.position.z < 0.1 || local.position.z > 50.0 ||
        !std::isfinite(local.position.x) || !std::isfinite(local.position.y) || !std::isfinite(local.position.z)))
      {
        ready = false;
        reason = "重定位后航点转换到本地系的高度超出 [0.1, 50] 米";
        break;
      }
      waypoints_[index] = local;
    }
    if (ready && generator_waypoint_initialized_ && waypoint_index_ < waypoints_.size()) {
      const auto & goal = waypoints_[waypoint_index_];
      const Eigen::Vector3d target(goal.position.x, goal.position.y, goal.position.z);
      const bool moved = (target - navigation_last_goal_).norm() > navigation_retarget_distance_ ||
        std::abs(normalize_angle(goal.yaw - navigation_last_goal_yaw_)) > navigation_retarget_yaw_;
      if (moved && std::chrono::duration<double>(now - navigation_last_retarget_).count() >= navigation_retarget_interval_) {
        navigation_last_retarget_ = now;
        navigation_last_goal_ = target;
        navigation_last_goal_yaw_ = goal.yaw;
        waypoint_arrival_tracker_.reset();
        if (waypoint_flight_strategy_ == 0) {
          navigation_reference_join_.retarget(target - navigation_segment_target_,
            normalize_angle(goal.yaw - navigation_segment_yaw_), navigation_retarget_blend_);
        } else if (waypoint_flight_strategy_ == 2) {
          // Continue the previously cleared segment until the new endpoint has its own check.
          // Normal relocalization does not force a stop or repeatedly restart obstacle waits.
          ++avoidance_task_revision_;
          avoidance_pending_.reset();
          avoidance_clear_since_.reset();
          navigation_line_retarget_pending_ = true;
        } else {
          // Drop old target identity before any old polynomial can be executed against a new goal.
          const auto wait_started = avoidance_wait_started_;
          const auto brake_start = avoidance_brake_start_;
          const auto brake_acceleration = avoidance_brake_acceleration_;
          reset_avoidance();
          begin_avoidance_wait(now, "map 重定位已更新目标，等待新的规划/通道检查");
          if (wait_started) {
            avoidance_wait_started_ = wait_started;
            avoidance_brake_start_ = brake_start;
            avoidance_brake_acceleration_ = brake_acceleration;
          }
        }
      }
    }
  }
  const bool map_flight = navigation_frame_ != "odom" || navigation_hold_started_ ||
    (armed_navigation_identity_ && armed_navigation_identity_->rfind("odom\n", 0) != 0) || !navigation_fault_.empty();
  if (!map_flight || active_task_ == ActiveTask::kLand || (!armed_ && active_task_ == ActiveTask::kNone)) {return;}
  if (!ready && !navigation_hold_started_) {
    cancel_active_task("地图定位保护：" + reason);
    if (!armed_) {set_status_message("拒绝飞行：" + reason, StatusLogLevel::kError); return;}
    navigation_hold_started_ = now;
    enter_hover("地图定位异常：本地制动悬停，超时后 LAND", guided_interfaces::msg::ControlStatus::MODE_FAILSAFE_HOLD,
      StatusLogLevel::kWarn);
    navigation_brake_start_ = reference_;
    navigation_brake_start_.velocity = vehicle_.velocity;
    failsafe_reason_ = reason;
  }
  if (navigation_hold_started_ && armed_) {
    const double elapsed = std::chrono::duration<double>(now - *navigation_hold_started_).count();
    reference_ = navigation_brake_start_;
    const double horizontal_speed = navigation_brake_start_.velocity.head<2>().norm();
    const double horizontal_deceleration = reference_generator_parameters_.trapezoidal.max_deceleration_xy;
    const double vertical_deceleration = reference_generator_parameters_.trapezoidal.max_deceleration_z;
    for (int axis = 0; axis < 3; ++axis) {
      const double speed = navigation_brake_start_.velocity[axis];
      const double stopping_time = axis < 2 ? horizontal_speed / horizontal_deceleration : std::abs(speed) / vertical_deceleration;
      const double t = std::min(elapsed, stopping_time);
      const double acceleration = stopping_time > 0.0 ? -speed / stopping_time : 0.0;
      reference_.position[axis] += speed * t + 0.5 * acceleration * t * t;
      reference_.velocity[axis] = elapsed < stopping_time ? speed + acceleration * t : 0.0;
      reference_.acceleration[axis] = elapsed < stopping_time ? acceleration : 0.0;
    }
    if (elapsed >= navigation_hold_timeout_) {trigger_failsafe_land("地图定位保护等待超时：" + failsafe_reason_);}
  }
  (void)dt_seconds;
}

/** Expose task coordinates while preserving existing local status fields. */
void OnboardControlNode::fill_navigation_status(guided_interfaces::msg::ControlStatus & message, const SteadyTime & now) const
{
  message.navigation_frame = navigation_frame_;
  message.navigation_map = navigation_map_;
  message.navigation_ready = navigation_ready(now, message.navigation_detail);
  const auto & transform = task_to_local_;
  write_vector(message.task_to_local.translation, transform.translation);
  message.task_to_local.rotation.x = transform.rotation.x();
  message.task_to_local.rotation.y = transform.rotation.y();
  message.task_to_local.rotation.z = transform.rotation.z();
  message.task_to_local.rotation.w = transform.rotation.w();
  const auto inverse = transform.inverse();
  write_vector(message.task_position, inverse.point(vehicle_.position));
  write_vector(message.task_velocity, inverse.rotation * vehicle_.velocity);
  message.task_yaw = inverse.yaw(yaw_);
}

/** Validate ground-only authority before spawning the independent restart worker. */
void OnboardControlNode::on_set_navigation_mode(const std::shared_ptr<SetNavigationMode::Request> request,
  std::shared_ptr<SetNavigationMode::Response> response)
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  std::string reason;
  if (!validate_envelope(request->stamp, request->ttl_ms, request->source_id, reason) ||
    !authorize_flight_sequence(request->source_id, request->sequence, reason))
  {
    response->message = reason; return;
  }
  if (request->mode != "odom" && request->mode != "map") {response->message = "定位模式必须为 odom 或 map"; return;}
  if (armed_ || !fresh_on_ground(SteadyClock::now()) || active_task_ != ActiveTask::kNone || reboot_active_) {
    response->message = "仅在落地、解除武装且无任务时允许切换定位模式"; return;
  }
  if (navigation_switch_pending_) {response->message = "已有定位切换正在执行"; return;}
  if (navigation_switch_script_.empty() || !std::filesystem::is_regular_file(navigation_switch_script_)) {
    response->message = "当前启动入口未配置定位切换脚本"; return;
  }
  std::error_code marker_error;
  std::filesystem::remove(switch_result_path(), marker_error);
  if (marker_error) {response->message = "无法清除上次定位切换结果：" + marker_error.message(); return;}
  posix_spawn_file_actions_t actions;
  posix_spawn_file_actions_init(&actions);
  posix_spawn_file_actions_addopen(&actions, STDIN_FILENO, "/dev/null", O_RDONLY, 0);
  posix_spawn_file_actions_addopen(&actions, STDOUT_FILENO, "/tmp/ros2_ardupilot_navigation_switch.log", O_WRONLY | O_CREAT | O_APPEND, 0644);
  posix_spawn_file_actions_adddup2(&actions, STDOUT_FILENO, STDERR_FILENO);
  std::vector<std::string> arguments{"setsid", "--fork", "bash", navigation_switch_script_, request->mode};
  std::vector<char *> argv;
  for (auto & argument : arguments) {argv.push_back(argument.data());}
  argv.push_back(nullptr);
  pid_t pid{};
  const int result = posix_spawnp(&pid, "setsid", &actions, nullptr, argv.data(), environ);
  posix_spawn_file_actions_destroy(&actions);
  if (result != 0) {response->message = "无法启动定位切换工作进程：" + std::to_string(result); return;}
  // Reap only our short-lived setsid parent; the detached worker owns its own lifecycle.
  std::thread([pid]() {while (waitpid(pid, nullptr, 0) < 0 && errno == EINTR) {}}).detach();
  navigation_switch_pending_ = true;
  response->accepted = true;
  response->message = "正在重启定位模式为 " + request->mode + "，请等待机载服务重新连接";
  set_status_message(response->message);
}
}  // namespace onboard_control
