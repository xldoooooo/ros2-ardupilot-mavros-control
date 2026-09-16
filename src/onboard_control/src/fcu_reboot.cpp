/** @file fcu_reboot.cpp
 * @brief Ground-only FCU reboot and passive boot-clock recovery; never arms or takes off.
 */
#include "onboard_control/onboard_control_node.hpp"

namespace onboard_control
{
namespace
{
// Long enough for ArduPilot boot, parameter/telemetry setup and EKF origin recovery.
constexpr auto kRecoveryTimeout = std::chrono::seconds(90);
constexpr auto kGroundFreshness = std::chrono::seconds(3);
constexpr auto kStableRecovery = std::chrono::seconds(2);
constexpr std::uint16_t kRebootCommand = 246;  // MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN
}

void OnboardControlNode::initialize_reboot()
{
  reboot_client_ = create_client<mavros_msgs::srv::CommandLong>(mavros_prefix_ + "/cmd/command");
  boot_clock_subscription_ = create_subscription<mavros_msgs::msg::TimesyncStatus>(
    mavros_prefix_ + "/timesync_status", rclcpp::SensorDataQoS(),
    [this](mavros_msgs::msg::TimesyncStatus::ConstSharedPtr message) {
      observe_fcu_clock(*message);
    });
}

bool OnboardControlNode::fresh_on_ground(const SteadyTime now) const
{
  return fcu_connected_ && !armed_ && on_ground_ &&
         last_state_time_ != SteadyTime{} &&
         std::chrono::duration<double>(now - last_state_time_).count() <= state_timeout_seconds_ &&
         last_extended_state_time_ != SteadyTime{} &&
         now - last_extended_state_time_ <= kGroundFreshness;
}

void OnboardControlNode::start_fcu_reboot(
  const CommandIdentity & command, std::shared_ptr<FlightCommand::Response> response)
{
  const auto now = SteadyClock::now();
  if (!fresh_on_ground(now) || active_task_ != ActiveTask::kNone || controller_engaged_ ||
    control_mode_ != guided_interfaces::msg::ControlStatus::MODE_IDLE || origin_confirmation_active_)
  {
    response->message = "拒绝重启：飞控须在线、未解锁、新鲜落地状态、待机且无正在执行的任务";
    return;
  }
  if (!reboot_client_->service_is_ready() || boot_clock_ns_ == 0 ||
    now - last_boot_clock_time_ > kGroundFreshness)
  {
    response->message = "拒绝重启：MAVROS 命令服务或飞控启动时钟不可用";
    return;
  }
  reboot_active_ = true;
  reboot_requested_ = true;
  reboot_observed_ = false;
  reboot_command_ = command;
  reboot_started_ = now;
  reboot_ready_since_ = SteadyTime{};
  ++reboot_generation_;
  reboot_sent_ = false;
  reboot_origin_prepared_ = false;
  last_reboot_origin_query_ = SteadyTime{};
  query_reboot_origin();
  response->accepted = true;
  response->message = "已接受飞控重启请求，正在读取原点";
  publish_result(command, guided_interfaces::msg::CommandResult::STATUS_RUNNING, false, response->message);
  set_status_message(response->message);
}

void OnboardControlNode::query_reboot_origin()
{
  // MAVROS need not receive GPS_GLOBAL_ORIGIN spontaneously after a short reboot.
  // Request message 49 explicitly; its ACK alone never confirms the origin.
  auto request = std::make_shared<mavros_msgs::srv::CommandLong::Request>();
  request->command = 512;  // MAV_CMD_REQUEST_MESSAGE
  request->param1 = 49;  // GPS_GLOBAL_ORIGIN
  last_reboot_origin_query_ = SteadyClock::now();
  reboot_client_->async_send_request(request,
    [](rclcpp::Client<mavros_msgs::srv::CommandLong>::SharedFuture) {});
}

void OnboardControlNode::send_fcu_reboot()
{
  const auto generation = reboot_generation_;
  reboot_sent_ = true;
  auto request = std::make_shared<mavros_msgs::srv::CommandLong::Request>();
  request->command = kRebootCommand;
  request->param1 = 1.0;  // Normal autopilot reboot; no force, bootloader or companion reboot.
  try {
    reboot_client_->async_send_request(request,
      [this, generation](rclcpp::Client<mavros_msgs::srv::CommandLong>::SharedFuture future) {
        std::lock_guard<std::recursive_mutex> lock(mutex_);
        if (!reboot_active_ || generation != reboot_generation_ || reboot_observed_) {return;}
        try {
          const auto reply = future.get();
          // A lost ACK is inconclusive: continue observing the FCU clock, never retry reboot.
          if (!reply->success) {
            set_status_message("飞控重启未获成功 ACK，继续等待启动时钟证据（不重发）", StatusLogLevel::kWarn);
          }
        } catch (const std::exception & error) {
          set_status_message(std::string("重启 ACK 异常，继续观察：") + error.what(), StatusLogLevel::kWarn);
        }
      });
  } catch (const std::exception & error) {
    finish_reboot(false, std::string("发送飞控重启失败：") + error.what());
    return;
  }
  const std::string message = "已发送飞控重启指令，等待实际重启和控制链路恢复";
  publish_result(reboot_command_, guided_interfaces::msg::CommandResult::STATUS_RUNNING, false, message);
  set_status_message(message);
}

void OnboardControlNode::observe_fcu_clock(const mavros_msgs::msg::TimesyncStatus & message)
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  const auto value = message.remote_timestamp_ns;
  if (value == 0) {return;}
  const auto previous = boot_clock_ns_;
  boot_clock_ns_ = value;
  last_boot_clock_time_ = SteadyClock::now();
  // ArduPilot TIMESYNC uses monotonic boot nanoseconds. Require >1 s rollback into
  // the first 30 s, ignoring minor packet reordering and host/NTP clock changes.
  if (previous <= value || previous - value < 1000000000ULL || value > 30000000000ULL) {return;}
  if (reboot_observed_ && reboot_active_) {return;}
  if (!reboot_active_) {
    reboot_active_ = true;
    reboot_requested_ = false;
    reboot_started_ = last_boot_clock_time_;
    ++reboot_generation_;
  }
  reboot_observed_ = true;
  reboot_detected_ = last_boot_clock_time_;
  reboot_ready_since_ = SteadyTime{};
  last_reboot_origin_send_ = SteadyTime{};
  last_reboot_origin_query_ = SteadyTime{};
  set_status_message("飞控重启成功：已检测到飞控启动时钟回退；开始恢复控制链路");
  if (reboot_requested_) {
    publish_result(reboot_command_, guided_interfaces::msg::CommandResult::STATUS_RUNNING,
      false, "飞控重启成功，等待控制链路恢复");
  }
  // Invalidate all session-dependent evidence, including short reboots that MAVROS
  // never represented as disconnected. Late interval replies cannot advance a new chain.
  ++session_generation_;
  ++fcu_parameter_revision_;
  thrust_mode_verified_ = false;
  reboot_guid_seen_ = false;
  reboot_hover_seen_ = false;
  fcu_guid_options_.reset();
  fcu_hover_throttle_.reset();
  fcu_parameter_pull_requested_ = false;
  fcu_parameter_sync_started_ = reboot_detected_;
  priority_parameter_rounds_ = 0;
  last_priority_parameter_request_ = SteadyTime{};
  last_thrust_mode_check_ = SteadyTime{};
  message_rates_configured_ = false;
  message_rate_configuration_active_ = false;
  message_rate_index_ = 0;
  last_automatic_message_rate_attempt_ = SteadyTime{};
  global_origin_observed_ = false;
  on_ground_ = false;
  pose_valid_ = false;
  velocity_valid_ = false;
  battery_present_ = false;
  if (armed_ || active_task_ != ActiveTask::kNone || controller_engaged_) {
    finish_reboot(false, "检测到飞控重启但未处于未解锁待机状态，停止自动恢复");
  }
}

void OnboardControlNode::reboot_tick(const SteadyTime now)
{
  if (!reboot_active_) {return;}
  if (armed_) {
    finish_reboot(false, "重启恢复期间检测到已解锁，停止自动恢复；请人工检查");
    return;
  }
  if (now - reboot_started_ > kRecoveryTimeout) {
    finish_reboot(false, reboot_observed_ ?
      "飞控已重启，但控制链路恢复超时（检查原点、定位、参数和消息流）" :
      "飞控重启超时：未观察到启动时钟回退，不能确认重启成功");
    return;
  }
  if (!reboot_observed_) {
    if (reboot_requested_ && !reboot_sent_ &&
      (reboot_origin_prepared_ || now - reboot_started_ >= std::chrono::seconds(3)))
    {
      if (!fresh_on_ground(now) || now - last_boot_clock_time_ > kGroundFreshness) {
        finish_reboot(false, "重启发送前飞控/落地状态已变化，取消重启");
        return;
      }
      send_fcu_reboot();
    }
    return;
  }
  if (!fresh_on_ground(now)) {return;}
  if (!global_origin_observed_ &&
    (last_reboot_origin_query_ == SteadyTime{} || now - last_reboot_origin_query_ > std::chrono::seconds(2)))
  {
    query_reboot_origin();
  }
  // Restore only the previously observed FCU origin. Never invent an origin or
  // restore flight tasks. A new gp_origin callback must confirm the value.
  if (reboot_origin_saved_ && (!global_origin_observed_ ||
    !origins_match(reboot_origin_, last_global_origin_)) &&
    (last_reboot_origin_send_ == SteadyTime{} || now - last_reboot_origin_send_ > std::chrono::seconds(2)))
  {
    geographic_msgs::msg::GeoPointStamped origin;
    origin.header.stamp = get_clock()->now();
    origin.position = reboot_origin_;
    origin_publisher_->publish(origin);
    last_reboot_origin_send_ = now;
  }
  const bool ready = last_state_time_ > reboot_detected_ &&
    last_extended_state_time_ > reboot_detected_ &&
    global_origin_observed_ && (!reboot_origin_saved_ || origins_match(reboot_origin_, last_global_origin_)) &&
    pose_valid_ && velocity_valid_ && last_pose_time_ > reboot_detected_ &&
    last_velocity_time_ > reboot_detected_ &&
    std::chrono::duration<double>(now - last_pose_time_).count() <= pose_timeout_seconds_ &&
    std::chrono::duration<double>(now - last_velocity_time_).count() <= pose_timeout_seconds_ &&
    reboot_guid_seen_ && reboot_hover_seen_ &&
    message_rates_configured_ && thrust_mode_verified_ && !setpoint_conflict_ &&
    now - last_boot_clock_time_ <= kGroundFreshness;
  if (!ready) {reboot_ready_since_ = SteadyTime{}; return;}
  if (reboot_ready_since_ == SteadyTime{}) {reboot_ready_since_ = now;}
  if (now - reboot_ready_since_ >= kStableRecovery) {
    finish_reboot(true, "控制链路成功恢复：飞控未解锁、已落地，原点回读、定位/速度、推力参数和消息配置均已就绪");
  }
}

void OnboardControlNode::finish_reboot(bool success, const std::string & message)
{
  reboot_active_ = false;
  set_status_message(message, success ? StatusLogLevel::kInfo : StatusLogLevel::kError);
  if (reboot_requested_) {
    publish_result(reboot_command_, success ? guided_interfaces::msg::CommandResult::STATUS_SUCCEEDED :
      guided_interfaces::msg::CommandResult::STATUS_FAILED, true, message);
  }
}
}  // namespace onboard_control
