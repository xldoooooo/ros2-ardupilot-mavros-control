/** @file control_diagnostics.cpp
 * @brief Bounded, deferred flight timing evidence; never participates in flight decisions.
 */
#include "onboard_control/onboard_control_node.hpp"

#include <algorithm>
#include <cinttypes>
#include <cerrno>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <unistd.h>
#include <type_traits>

namespace onboard_control
{

/** Create a separate file so a stalled diagnostic sink cannot take the global ROS logging lock. */
void OnboardControlNode::initialize_control_diagnostics()
{
  const char * directory = std::getenv("ROS_LOG_DIR");
  const char * ros_home = std::getenv("ROS_HOME");
  const char * home = std::getenv("HOME");
  const auto log_root = directory && *directory ? std::filesystem::path(directory) :
    (ros_home && *ros_home ? std::filesystem::path(ros_home) :
    std::filesystem::path(home && *home ? home : ".") / ".ros") / "log";
  std::error_code error;
  std::filesystem::create_directories(log_root, error);
  const auto run_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::system_clock::now().time_since_epoch()).count();
  const auto path = log_root / ("onboard-control-diagnostics-" + std::to_string(getpid()) + "-" +
    std::to_string(run_ns) + ".log");
  if (!error) {diagnostic_log_.reset(std::fopen(path.c_str(), "ae"));}
  if (!diagnostic_log_) {
    RCLCPP_WARN(get_logger(), "异常诊断文件不可用（不影响飞行）：%s: %s", path.c_str(),
      error ? error.message().c_str() : std::strerror(errno));
    return;
  }
  diagnostic_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  diagnostic_timer_ = create_wall_timer(std::chrono::seconds(1),
    std::bind(&OnboardControlNode::flush_control_diagnostics, this), diagnostic_callback_group_);
  RCLCPP_INFO(get_logger(), "异常诊断已启用：独立1Hz文件 %s（不更改保护阈值）", path.c_str());
}

/** Retain constant-size metadata after the original state callback has finished its updates. */
void OnboardControlNode::record_receive(
  ReceiveDiagnostic & sample, const rclcpp::MessageInfo & info,
  const builtin_interfaces::msg::Time & stamp, SteadyTime entered, SteadyTime acquired)
{
  sample.gap_ms = sample.entered == SteadyTime{} ? 0.0 :
    std::chrono::duration<double, std::milli>(entered - sample.entered).count();
  sample.entered = entered;
  sample.lock_wait_ms = std::chrono::duration<double, std::milli>(acquired - entered).count();
  ++sample.count;
  sample.header_ns = static_cast<std::int64_t>(stamp.sec) * 1000000000LL + stamp.nanosec;
  const auto & middleware = info.get_rmw_message_info();
  sample.dds_source_ns = middleware.source_timestamp;
  sample.dds_receive_ns = middleware.received_timestamp;
  sample.callback_wall_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::system_clock::now().time_since_epoch()).count();
}

/** Capture the first pending fault and coalesce timing events into one worst snapshot. */
void OnboardControlNode::record_control_diagnostic(
  const char * event, bool fault, const SteadyTime & now)
{
  static_assert(std::is_trivially_copyable<ControlDiagnostic>::value);
  if (fault && pending_fault_diagnostic_) {return;}
  const auto age_ms = [&now](SteadyTime received) {
      // -1 explicitly distinguishes never-received data from an old valid sample.
      return received == SteadyTime{} ? -1.0 :
             std::chrono::duration<double, std::milli>(now - received).count();
    };
  ControlDiagnostic sample;
  sample.event = event;
  sample.captured_at = now;
  sample.event_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::system_clock::now().time_since_epoch()).count();
  sample.sequence = active_command_.sequence;
  sample.fcu_session = session_generation_;
  sample.values = {vehicle_.position.x(), vehicle_.position.y(), vehicle_.position.z(),
    vehicle_.velocity.x(), vehicle_.velocity.y(), vehicle_.velocity.z(), roll_, pitch_, yaw_};
  sample.waypoint = static_cast<std::uint32_t>(waypoint_index_ + 1);
  sample.mode = control_mode_;
  sample.calibration_stage = static_cast<unsigned>(calibration_stage_);
  sample.pose_valid = pose_valid_;
  sample.velocity_valid = velocity_valid_;
  sample.connected = fcu_connected_;
  sample.armed = armed_;
  sample.pose_age_ms = age_ms(last_pose_time_);
  sample.velocity_age_ms = age_ms(last_velocity_time_);
  sample.state_age_ms = age_ms(last_state_time_);
  sample.timeout_ms = pose_timeout_seconds_ * 1000.0;
  sample.state_timeout_ms = state_timeout_seconds_ * 1000.0;
  sample.period_ms = diagnostic_period_ms_;
  sample.lock_wait_ms = diagnostic_lock_wait_ms_;
  sample.previous_work_ms = previous_control_work_ms_;
  sample.pose = pose_diagnostic_;
  sample.velocity = velocity_diagnostic_;
  sample.slow = slow_operation_;
  if (fault) {
    pending_fault_diagnostic_ = sample;
    post_fault_pose_.reset();
    post_fault_velocity_.reset();
  } else {
    ++pending_timing_count_;
    const auto severity = [](const ControlDiagnostic & s) {
        return std::max(s.period_ms, s.slow.finished == s.captured_at ? s.slow.duration_ms : 0.0);
      };
    if (!pending_timing_diagnostic_ || severity(sample) > severity(*pending_timing_diagnostic_)) {
      pending_timing_diagnostic_ = sample;
    }
  }
}

/** Measure only named existing code regions; the threshold controls logging, not execution. */
void OnboardControlNode::record_slow_operation(
  const char * name, SteadyTime started, SteadyTime finished)
{
  const double elapsed = std::chrono::duration<double, std::milli>(finished - started).count();
  if (elapsed < kDiagnosticSlowMs) {return;}
  slow_operation_ = SlowOperation{name, elapsed, finished};
  record_control_diagnostic("slow_operation", false, finished);
}

/** Try-copy two fixed slots, release the flight lock, then emit at most one batch per second. */
void OnboardControlNode::flush_control_diagnostics()
{
  // Fixed-size capture only; all derived metrics, formatting and I/O stay outside mutex_.
  struct FusionLog
  {
    std::int64_t event_ns;
    std::uint64_t revision;
    Eigen::Vector3d position, reference, velocity, external_velocity, control_velocity;
    geometry_msgs::msg::Pose external_pose;
    double elapsed, yaw, reference_yaw, frame_yaw, stable_seconds;
    std::array<double, 5> ages;  // FCU pose/velocity, extnav pose/velocity, raw odometry, in ms.
    std::array<double, 2> post_apply;  // extnav pose/velocity stamp minus application stamp, in ms.
    bool pose_valid, velocity_valid;
  };
  std::optional<FusionLog> fusion;
  std::optional<ControlDiagnostic> fault, timing;
  std::optional<ReceiveDiagnostic> after_pose, after_velocity;
  std::uint64_t timing_count = 0;
  {
    // Logging must neither wait for nor retain the flight-state lock during any I/O.
    std::unique_lock<std::recursive_mutex> lock(mutex_, std::try_to_lock);
    if (!lock.owns_lock()) {return;}
    fault.swap(pending_fault_diagnostic_);
    timing.swap(pending_timing_diagnostic_);
    after_pose.swap(post_fault_pose_);
    after_velocity.swap(post_fault_velocity_);
    timing_count = pending_timing_count_;
    pending_timing_count_ = 0;
    if (calibration_stage_ == CalibrationStage::kFusion) {
      const auto now = SteadyClock::now();
      const auto milliseconds = [&now](SteadyTime t) {
          return t == SteadyTime{} ? -1.0 :
                 std::chrono::duration<double, std::milli>(now - t).count();
        };
      fusion = FusionLog{
        std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::system_clock::now().time_since_epoch()).count(),
        navigation_extnav_ ? navigation_extnav_->revision : 0,
        vehicle_.position, reference_.position, vehicle_.velocity, calibration_external_velocity_,
        calibration_control_velocity_, calibration_external_pose_.pose,
        std::chrono::duration<double>(now - calibration_started_).count(), yaw_, reference_.yaw,
        calibration_control_frame_yaw_, calibration_stable_since_ == SteadyTime{} ? 0.0 :
        milliseconds(calibration_stable_since_) / 1000.0,
        {milliseconds(last_pose_time_), milliseconds(last_velocity_time_),
          milliseconds(calibration_pose_time_), milliseconds(calibration_velocity_time_),
          milliseconds(calibration_odometry_time_)},
        {(calibration_pose_stamp_ - calibration_application_stamp_) / 1.e6,
          (calibration_velocity_stamp_ - calibration_application_stamp_) / 1.e6},
        pose_valid_, velocity_valid_};
    }
  }
  const auto log_snapshot = [this](const char * label, const ControlDiagnostic & s,
      std::uint64_t count) {
      const double slow_age_ms = s.slow.finished == SteadyTime{} ? -1.0 :
        std::chrono::duration<double, std::milli>(s.captured_at - s.slow.finished).count();
      std::fprintf(diagnostic_log_.get(),
        "[%s] event_ns=%" PRId64 " event=%s count=%" PRIu64
        " mode=%u armed=%d connected=%d command_seq=%" PRIu64 " fcu_session=%" PRIu64
        " waypoint=%u calibration_stage=%u"
        " pose_valid=%d velocity_valid=%d pose_age_ms=%.3f velocity_age_ms=%.3f state_age_ms=%.3f"
        " pose_timeout_ms=%.3f state_timeout_ms=%.3f control_period_ms=%.3f control_lock_wait_ms=%.3f previous_work_ms=%.3f"
        " slow_operation=%s slow_ms=%.3f slow_age_at_event_ms=%.3f"
        " position=(%.6f,%.6f,%.6f) velocity=(%.6f,%.6f,%.6f) rpy=(%.6f,%.6f,%.6f)\n",
        label, s.event_ns, s.event, count, s.mode, s.armed, s.connected, s.sequence, s.fcu_session,
        s.waypoint, s.calibration_stage, s.pose_valid, s.velocity_valid,
        s.pose_age_ms, s.velocity_age_ms, s.state_age_ms, s.timeout_ms, s.state_timeout_ms,
        s.period_ms, s.lock_wait_ms, s.previous_work_ms,
        s.slow.name, s.slow.duration_ms, slow_age_ms,
        s.values[0], s.values[1], s.values[2], s.values[3], s.values[4], s.values[5],
        s.values[6], s.values[7], s.values[8]);
    };
  const auto log_receive = [this](std::int64_t event_ns, const char * phase, const char * stream,
      const ReceiveDiagnostic & s) {
      std::fprintf(diagnostic_log_.get(),
        "[TELEMETRY_RX] event_ns=%" PRId64 " phase=%s stream=%s count=%" PRIu64
        " header_ns=%" PRId64 " dds_source_ns=%" PRId64 " dds_receive_ns=%" PRId64
        " callback_wall_ns=%" PRId64 " callback_gap_ms=%.3f callback_lock_wait_ms=%.3f\n",
        event_ns, phase, stream, s.count, s.header_ns, s.dds_source_ns, s.dds_receive_ns,
        s.callback_wall_ns, s.gap_ms, s.lock_wait_ms);
    };
  if (fault) {
    log_snapshot("TELEMETRY_FAULT", *fault, 1);
    log_receive(fault->event_ns, "before", "pose", fault->pose);
    log_receive(fault->event_ns, "before", "velocity", fault->velocity);
    if (after_pose) {log_receive(fault->event_ns, "first_after", "pose", *after_pose);}
    if (after_velocity) {log_receive(fault->event_ns, "first_after", "velocity", *after_velocity);}
  }
  if (timing) {log_snapshot("CONTROL_TIMING", *timing, timing_count);}
  if (fusion) {
    const auto & f = *fusion;
    const auto & p = f.external_pose.position;
    const auto & q = f.external_pose.orientation;
    const double external_yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
      1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    std::fprintf(diagnostic_log_.get(),
      "[CALIBRATION_FUSION] event_ns=%" PRId64 " revision=%" PRIu64
      " elapsed_s=%.3f stable_s=%.3f valid=(%d,%d)"
      " ages_ms=(%.3f,%.3f,%.3f,%.3f,%.3f) post_apply_ms=(%.3f,%.3f)"
      " xy_error_m=%.6f yaw_error_rad=%.6f speeds_mps=(%.6f,%.6f,%.6f)"
      " position=(%.6f,%.6f,%.6f) target=(%.6f,%.6f,%.6f)"
      " yaw_rad=%.6f target_yaw_rad=%.6f raw_to_fcu_yaw_rad=%.6f control_vxy=(%.6f,%.6f)\n",
      f.event_ns, f.revision, f.elapsed, f.stable_seconds, f.pose_valid, f.velocity_valid,
      f.ages[0], f.ages[1], f.ages[2], f.ages[3], f.ages[4], f.post_apply[0], f.post_apply[1],
      std::hypot(f.position.x() - p.x, f.position.y() - p.y), normalize_angle(f.yaw - external_yaw),
      f.velocity.norm(), f.external_velocity.head<2>().norm(),
      (f.velocity - f.external_velocity).head<2>().norm(),
      f.position.x(), f.position.y(), f.position.z(), f.reference.x(), f.reference.y(), f.reference.z(),
      f.yaw, f.reference_yaw, f.frame_yaw, f.control_velocity.x(), f.control_velocity.y());
  }
  if (fault || timing || fusion) {std::fflush(diagnostic_log_.get());}
}
}  // namespace onboard_control
