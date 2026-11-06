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
#include <sys/resource.h>

namespace onboard_control
{

/** Read per-thread CPU/context/fault counters without formatting or heap allocation. */
OnboardControlNode::ThreadTiming OnboardControlNode::sample_thread_timing()
{
  ThreadTiming sample;
  rusage usage{};
  if (::getrusage(RUSAGE_THREAD, &usage) == 0) {
    sample.cpu_ms = (usage.ru_utime.tv_sec + usage.ru_stime.tv_sec) * 1000.0 +
      (usage.ru_utime.tv_usec + usage.ru_stime.tv_usec) / 1000.0;
    sample.counters = {usage.ru_nvcsw, usage.ru_nivcsw, usage.ru_minflt, usage.ru_majflt};
  }
  sample.wall = SteadyClock::now();
  return sample;
}

/** Retain fixed-size stage durations; unavailable CPU samples stay explicitly negative. */
OnboardControlNode::OutputDiagnostic OnboardControlNode::make_output_diagnostic(
  const OutputInFlight & identity, const std::array<ThreadTiming, 5> & samples)
{
  OutputDiagnostic result;
  result.identity = identity;
  for (std::size_t i = 0; i < result.wall_ms.size(); ++i) {
    result.wall_ms[i] = std::chrono::duration<double, std::milli>(
      samples[i + 1].wall - samples[i].wall).count();
    result.cpu_ms[i] = samples[i].cpu_ms < 0.0 || samples[i + 1].cpu_ms < 0.0 ? -1.0 :
      samples[i + 1].cpu_ms - samples[i].cpu_ms;
    result.counters[i] = samples.front().cpu_ms < 0.0 || samples.back().cpu_ms < 0.0 ? -1 :
      samples.back().counters[i] - samples.front().counters[i];
  }
  return result;
}

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
  sample.status_ms = status_diagnostic_ms_;
  sample.execution_wait_ms = diagnostic_execution_wait_ms_;
  sample.output = output_diagnostic_;
  sample.raw.receive = raw_diagnostic_;
  sample.raw.age_ms = age_ms(calibration_odometry_time_);
  sample.raw.follow_fusion = calibration_follow_fusion_;
  sample.raw.revision = navigation_extnav_ ? navigation_extnav_->revision : 0;
  const auto copy_text = [](auto & destination, const std::string & source) {
      const auto length = std::min(destination.size() - 1, source.size());
      std::memcpy(destination.data(), source.data(), length);
      return source.size();
    };
  sample.raw.job_length = copy_text(sample.raw.job, calibration_job_);
  if (calibration_odometry_) {
    const auto & odometry = *calibration_odometry_;
    const auto & p = odometry.pose.pose.position;
    const auto & v = odometry.twist.twist.linear;
    const auto & q = odometry.pose.pose.orientation;
    sample.raw.present = true;
    sample.raw.values = {p.x, p.y, p.z, v.x, v.y, v.z, q.x, q.y, q.z, q.w};
    sample.raw.frame_length = copy_text(sample.raw.frame, odometry.header.frame_id);
    sample.raw.child_length = copy_text(sample.raw.child, odometry.child_frame_id);
  }
  if (fault) {
    pending_fault_diagnostic_ = sample;
    post_fault_pose_.reset();
    post_fault_velocity_.reset();
    // Do not discard a post-fault sample still waiting for the diagnostic timer.
    if (raw_fault_event_ns_ == 0) {raw_fault_event_ns_ = sample.event_ns;}
  } else {
    ++pending_timing_count_;
    const auto severity = [](const ControlDiagnostic & s) {
        return std::max(s.period_ms,
          std::strcmp(s.event, "slow_operation") == 0 ? s.slow.duration_ms : 0.0);
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
  // Inputs may have advanced while publishing; snapshot their ages at capture time.
  record_control_diagnostic("slow_operation", false, SteadyClock::now());
}

/** Try-copy bounded slots, release the flight lock, then emit at most one batch per second. */
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
  std::array<std::optional<OutputInFlight>, 2> inflight;
  std::array<double, 2> inflight_age_ms{};
  std::array<double, 3> input_age_ms{};  // Ages when the active-publication snapshot was captured.
  std::optional<ControlDiagnostic> fault, timing;
  std::optional<ReceiveDiagnostic> after_pose, after_velocity, after_raw;
  std::int64_t raw_event_ns = 0;
  std::uint64_t timing_count = 0;
  {
    // Logging must neither wait for nor retain the flight-state lock during any I/O.
    std::unique_lock<std::recursive_mutex> lock(mutex_, std::try_to_lock);
    if (!lock.owns_lock()) {return;}
    const auto captured = SteadyClock::now();
    const auto age = [&captured](SteadyTime received) {
        return received == SteadyTime{} ? -1.0 :
          std::chrono::duration<double, std::milli>(captured - received).count();
      };
    input_age_ms = {age(last_pose_time_), age(last_velocity_time_), age(calibration_odometry_time_)};
    inflight = {attitude_inflight_, status_inflight_};
    for (std::size_t i = 0; i < inflight.size(); ++i) {
      if (!inflight[i]) {continue;}
      inflight_age_ms[i] = std::chrono::duration<double, std::milli>(
        captured - inflight[i]->started).count();
      if (inflight_age_ms[i] < kDiagnosticSlowMs) {inflight[i].reset();}
    }
    fault.swap(pending_fault_diagnostic_);
    timing.swap(pending_timing_diagnostic_);
    after_pose.swap(post_fault_pose_);
    after_velocity.swap(post_fault_velocity_);
    after_raw.swap(post_fault_raw_);
    if (after_raw) {
      raw_event_ns = raw_fault_event_ns_;
      raw_fault_event_ns_ = 0;
    }
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
        " execution_lock_wait_ms=%.3f slow_operation=%s slow_ms=%.3f slow_age_at_event_ms=%.3f"
        " status_ms=(%.3f,%.3f,%.3f,%.3f,%.3f,%.3f)"
        " position=(%.6f,%.6f,%.6f) velocity=(%.6f,%.6f,%.6f) rpy=(%.6f,%.6f,%.6f)\n",
        label, s.event_ns, s.event, count, s.mode, s.armed, s.connected, s.sequence, s.fcu_session,
        s.waypoint, s.calibration_stage, s.pose_valid, s.velocity_valid,
        s.pose_age_ms, s.velocity_age_ms, s.state_age_ms, s.timeout_ms, s.state_timeout_ms,
        s.period_ms, s.lock_wait_ms, s.previous_work_ms,
        s.execution_wait_ms, s.slow.name, s.slow.duration_ms, slow_age_ms,
        s.status_ms[0], s.status_ms[1], s.status_ms[2], s.status_ms[3], s.status_ms[4], s.status_ms[5],
        s.values[0], s.values[1], s.values[2], s.values[3], s.values[4], s.values[5],
        s.values[6], s.values[7], s.values[8]);
      const auto & o = s.output;
      if (o.identity.wall_ns != 0) {
        std::fprintf(diagnostic_log_.get(),
          "[CONTROL_OUTPUT] event_ns=%" PRId64 " publish_begin_ns=%" PRId64
          " command_seq=%" PRIu64 " tid=%d phase_order=compute,message,publish,relock"
          " wall_ms=(%.3f,%.3f,%.3f,%.3f) cpu_ms=(%.3f,%.3f,%.3f,%.3f)"
          " voluntary_switches=%ld involuntary_switches=%ld minor_faults=%ld major_faults=%ld\n",
          s.event_ns, o.identity.wall_ns, o.identity.sequence, o.identity.tid,
          o.wall_ms[0], o.wall_ms[1], o.wall_ms[2], o.wall_ms[3],
          o.cpu_ms[0], o.cpu_ms[1], o.cpu_ms[2], o.cpu_ms[3],
          o.counters[0], o.counters[1], o.counters[2], o.counters[3]);
      }
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
  const auto log_raw = [this, &log_receive](const ControlDiagnostic & s) {
      const auto & raw = s.raw;
      const auto & v = raw.values;
      const double qnorm_squared = v[6]*v[6] + v[7]*v[7] + v[8]*v[8] + v[9]*v[9];
      std::fprintf(diagnostic_log_.get(),
        "[RAW_ODOMETRY] event_ns=%" PRId64 " event=%s present=%d age_ms=%.3f timeout_ms=%.3f"
        " frame=%s child=%s job=%s text_lengths=(%zu,%zu,%zu) revision=%" PRIu64 " follow_fusion=%d"
        " position=(%.6f,%.6f,%.6f) body_velocity=(%.6f,%.6f,%.6f)"
        " quaternion_xyzw=(%.9f,%.9f,%.9f,%.9f) quaternion_norm_squared=%.9f\n",
        s.event_ns, s.event, raw.present, raw.age_ms, s.timeout_ms,
        raw.frame.data(), raw.child.data(), raw.job.data(), raw.frame_length, raw.child_length,
        raw.job_length, raw.revision, raw.follow_fusion,
        v[0], v[1], v[2], v[3], v[4], v[5], v[6], v[7], v[8], v[9], qnorm_squared);
      log_receive(s.event_ns, "before", "raw_odin", raw.receive);
    };
  if (fault) {
    log_snapshot("TELEMETRY_FAULT", *fault, 1);
    log_raw(*fault);
    log_receive(fault->event_ns, "before", "pose", fault->pose);
    log_receive(fault->event_ns, "before", "velocity", fault->velocity);
    if (after_pose) {log_receive(fault->event_ns, "first_after", "pose", *after_pose);}
    if (after_velocity) {log_receive(fault->event_ns, "first_after", "velocity", *after_velocity);}
  }
  if (timing) {
    log_snapshot("CONTROL_TIMING", *timing, timing_count);
    log_raw(*timing);
  }
  if (after_raw) {log_receive(raw_event_ns, "first_after", "raw_odin", *after_raw);}
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
  for (std::size_t i = 0; i < inflight.size(); ++i) {
    if (!inflight[i]) {continue;}
    const auto & active = *inflight[i];
    std::fprintf(diagnostic_log_.get(),
      "[PUBLISH_INFLIGHT] publish_begin_ns=%" PRId64 " operation=%s command_seq=%" PRIu64
      " tid=%d elapsed_ms=%.3f scope=publish_or_relock input_age_ms=(%.3f,%.3f,%.3f)\n",
      active.wall_ns, i == 0 ? "attitude_publish" : "status_publish",
      active.sequence, active.tid, inflight_age_ms[i], input_age_ms[0], input_age_ms[1], input_age_ms[2]);
  }
  if (fault || timing || fusion || after_raw || inflight[0] || inflight[1]) {
    std::fflush(diagnostic_log_.get());
  }
}
}  // namespace onboard_control
