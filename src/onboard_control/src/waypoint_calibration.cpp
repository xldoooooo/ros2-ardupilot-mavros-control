/** @file waypoint_calibration.cpp
 * @brief Nonblocking Tag arrival actions using the existing calibration and LAND paths.
 */
#include "onboard_control/onboard_control_node.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace onboard_control
{
namespace
{
// Service/pose observations may miss a few messages, but never reuse an offline history.
constexpr double kCalibrationStatusTimeout = 3.0;
// Ownership renewal leaves ample margin inside the service's 15 second lease.
constexpr double kOwnerRenewInterval = 2.0;
// A yaw mismatch above five degrees cannot certify that FCU fusion has settled.
constexpr double kFusionYawTolerance = 5.0 * 3.14159265358979323846 / 180.0;

/** Inspect an asynchronous response without blocking the 100 Hz control thread. */
template<typename Future>
bool ready(const Future & future)
{
  return future.valid() && future.wait_for(std::chrono::seconds(0)) == std::future_status::ready;
}
}  // namespace

/** Wire passive evidence and the existing transaction services. */
void OnboardControlNode::initialize_calibration()
{
  calibration_timeout_seconds_ = declare_parameter("calibration_timeout_seconds", 120.0);
  if (!std::isfinite(calibration_timeout_seconds_) || calibration_timeout_seconds_ <= 0.0) {
    throw std::invalid_argument("calibration_timeout_seconds must be positive");
  }
  // Create the subscription at startup, not in the airborne control loop. Retain one sample only.
  calibration_odometry_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
    declare_parameter<std::string>("calibration_odometry_topic", "/odin1/odometry_highfreq"),
    rclcpp::SensorDataQoS().keep_last(1),
    [this](nav_msgs::msg::Odometry::ConstSharedPtr message, const rclcpp::MessageInfo & info) {
      const auto entered = SteadyClock::now();
      std::lock_guard<std::recursive_mutex> lock(mutex_);
      const auto acquired = SteadyClock::now();
      calibration_odometry_ = std::move(message);
      calibration_odometry_time_ = SteadyClock::now();
      record_receive(raw_diagnostic_, info, calibration_odometry_->header.stamp, entered, acquired);
      if (raw_fault_event_ns_ != 0 && !post_fault_raw_) {post_fault_raw_ = raw_diagnostic_;}
    }, telemetry_subscription_options_);
  calibration_owner_client_ = create_client<CalibrationOwner>("/correction_service/set_calibration_owner");
  calibration_clear_client_ = create_client<CalibrationClear>("/correction_service/clear_window");
  calibration_start_client_ = create_client<CalibrationStart>("/correction_service/start");
  calibration_stop_client_ = create_client<CalibrationStop>("/correction_service/stop");
  calibration_subscription_ = create_subscription<correction_interfaces::msg::CorrectionStatus>(
    "/correction_service/status", rclcpp::QoS(10),
    [this](correction_interfaces::msg::CorrectionStatus::SharedPtr message) {
      std::lock_guard<std::recursive_mutex> lock(mutex_);
      calibration_status_ = *message;
      calibration_status_time_ = SteadyClock::now();
    }, telemetry_subscription_options_);
  calibration_pose_subscription_ = create_subscription<geometry_msgs::msg::PoseStamped>(
    "/extnav/pose_fcu", rclcpp::QoS(10),
    [this](geometry_msgs::msg::PoseStamped::SharedPtr message) {
      std::lock_guard<std::recursive_mutex> lock(mutex_);
      calibration_external_pose_ = *message;
      calibration_pose_time_ = SteadyClock::now();
      calibration_pose_stamp_ = rclcpp::Time(message->header.stamp).nanoseconds();
    }, telemetry_subscription_options_);
  calibration_velocity_subscription_ = create_subscription<geometry_msgs::msg::TwistStamped>(
    "/extnav/velocity_fcu", rclcpp::QoS(10),
    [this](geometry_msgs::msg::TwistStamped::SharedPtr message) {
      std::lock_guard<std::recursive_mutex> lock(mutex_);
      const auto & velocity = message->twist.linear;
      calibration_external_velocity_ = {velocity.x, velocity.y, velocity.z};
      calibration_velocity_time_ = SteadyClock::now();
      calibration_velocity_stamp_ = rclcpp::Time(message->header.stamp).nanoseconds();
    }, telemetry_subscription_options_);
}

/** Preserve physical error in raw Odin, then rotate it into the FCU's evolving local frame. */
bool OnboardControlNode::update_calibration_hold(const SteadyTime & now, const bool update_reference)
{
  // Keep the original checks and order; retain the reason only for a terminal failure snapshot.
  const auto invalid = [this](const char * reason) {
      calibration_hold_failure_ = reason;
      return false;
    };
  if (!pose_valid_ || !std::isfinite(yaw_)) {return invalid("calibration_fcu_pose_invalid");}
  if (!calibration_odometry_) {return invalid("calibration_raw_missing");}
  if (std::chrono::duration<double>(now - calibration_odometry_time_).count() > pose_timeout_seconds_) {
    return invalid("calibration_raw_stale");
  }
  const auto & odometry = *calibration_odometry_;
  if (odometry.header.frame_id != "odom" || odometry.child_frame_id != "fcu") {
    return invalid("calibration_raw_frame");
  }
  const auto & p = odometry.pose.pose.position;
  const auto & q = odometry.pose.pose.orientation;
  const auto & v = odometry.twist.twist.linear;
  const Eigen::Vector2d raw_position(p.x, p.y);
  const Eigen::Vector3d body_velocity(v.x, v.y, v.z);
  Eigen::Quaterniond raw_attitude(q.w, q.x, q.y, q.z);
  if (!raw_position.allFinite()) {return invalid("calibration_raw_xy_nonfinite");}
  if (!body_velocity.allFinite()) {return invalid("calibration_raw_velocity_nonfinite");}
  if (!raw_attitude.coeffs().allFinite() || !std::isfinite(raw_attitude.squaredNorm()) ||
    raw_attitude.squaredNorm() <= 0.0)
  {
    return invalid("calibration_raw_quaternion_invalid");
  }
  raw_attitude.normalize();
  const auto raw_rotation = raw_attitude.toRotationMatrix();
  const double raw_yaw = std::atan2(raw_rotation(1, 0), raw_rotation(0, 0));
  const double frame_yaw = normalize_angle(yaw_ - raw_yaw);
  const Eigen::Rotation2Dd raw_to_fcu(frame_yaw);
  if (!calibration_hold_initialized_) {
    // Preserve the existing hover error, not the vehicle's slightly displaced current position.
    calibration_hold_position_ = raw_position + raw_to_fcu.inverse() *
      (reference_.position - vehicle_.position).head<2>();
    calibration_hold_yaw_ = normalize_angle(raw_yaw + reference_.yaw - yaw_);
    calibration_control_frame_yaw_ = frame_yaw;
    calibration_hold_initialized_ = true;
  }
  if (calibration_follow_fusion_) {
    // Translation/yaw estimator changes move the coordinate representation, never the anchor.
    if (update_reference) {
      reference_.position.head<2>() = vehicle_.position.head<2>() +
        raw_to_fcu * (calibration_hold_position_ - raw_position);
      reference_.yaw = normalize_angle(yaw_ + calibration_hold_yaw_ - raw_yaw);
      reference_.velocity.setZero();
      reference_.acceleration.setZero();
      target_yaw_rate_ = 0.0;
    }
    const Eigen::Vector3d raw_velocity = raw_rotation * body_velocity;
    calibration_control_velocity_ = vehicle_.velocity;
    calibration_control_velocity_.head<2>() = raw_to_fcu * raw_velocity.head<2>();
    controller_.rotate_horizontal_frame(normalize_angle(frame_yaw - calibration_control_frame_yaw_));
    if (!calibration_using_raw_velocity_) {
      controller_.rebase_horizontal_velocity(calibration_control_velocity_.head<2>());
      calibration_using_raw_velocity_ = true;
    }
  }
  calibration_control_frame_yaw_ = frame_yaw;
  return true;
}

/** Validate the complete proposed window sequence before accepting any flight task. */
bool OnboardControlNode::validate_calibration_task(
  const ExecuteWaypoints::Request & request, std::string & reason) const
{
  const auto now = SteadyClock::now();
  bool has_calibration = false;
  for (const auto & waypoint : request.waypoints) {
    if (waypoint.calibration_tag_id < 0) {reason = "校准 Tag ID 必须为非负整数"; return false;}
    has_calibration = has_calibration || waypoint.is_calibration;
  }
  if (!has_calibration) {return true;}
  // TODO(task12-map-avoidance): revalidate transformed world targets and planner maps.
  if (navigation_frame_ != "odom" || request.local_coordinates ||
    request.flight_strategy == ExecuteWaypoints::Request::STRATEGY_AVOID)
  {
    reason = "校准点仅支持 odom 绝对坐标＋直线/遇障悬停，不支持 map、自主避障或 local_coordinates";
    return false;
  }
  if (!calibration_status_ || !calibration_status_->service_available ||
    std::chrono::duration<double>(now - calibration_status_time_).count() > kCalibrationStatusTimeout ||
    !calibration_owner_client_->service_is_ready() || !calibration_start_client_->service_is_ready() ||
    !calibration_clear_client_->service_is_ready())
  {
    reason = "校准服务或新版自动校准接口不可用"; return false;
  }
  const auto & status = *calibration_status_;
  if (status.active || !status.automatic_owner.empty() ||
    status.application_state == status.APPLICATION_UNKNOWN)
  {
    reason = "校准服务正在操作或应用结果未知，请先完成手动操作/对账"; return false;
  }
  std::vector<int> window;
  for (std::size_t index = 0; index < request.waypoints.size(); ++index) {
    const auto & waypoint = request.waypoints[index];
    if (!waypoint.is_calibration) {continue;}
    if (!waypoint.continue_window) {window.clear();}
    const int tag = waypoint.calibration_tag_id;
    if (std::find(status.configured_tag_ids.begin(), status.configured_tag_ids.end(), tag) ==
      status.configured_tag_ids.end())
    {
      reason = "航点 " + std::to_string(index + 1) + " 使用未知 Tag " + std::to_string(tag); return false;
    }
    if (std::find(window.begin(), window.end(), tag) != window.end()) {
      reason = "航点 " + std::to_string(index + 1) + " 的 Tag 仍在沿用窗口内，请取消沿窗或更换 Tag";
      return false;
    }
    window.push_back(tag);
    if (window.size() > status.window_size) {window.erase(window.begin());}
  }
  return true;
}

/** Begin exactly once, keeping the point index and the immutable remaining goals. */
void OnboardControlNode::start_waypoint_calibration(const SteadyTime & now)
{
  if (!calibration_status_ || !navigation_extnav_) {
    fail_active_task("自动校准缺少权威服务/extnav状态", true); return;
  }
  const auto & waypoint = waypoints_[waypoint_index_];
  const auto & status = *calibration_status_;
  const auto & extnav = *navigation_extnav_;
  const bool next = task_has_calibrated_ && waypoint.continue_window;
  if (next && (status.service_instance_id != task_calibration_instance_ ||
    extnav.odin_session_id != task_calibration_session_ ||
    status.window_revision != task_calibration_window_revision_ ||
    status.window_count == 0 || status.window_state == "stale" ||
    status.window_state == "application_unknown"))
  {
    fail_active_task("自动校准必需窗口已失效/被修改，不能静默改为首次校准", true); return;
  }
  if (!status.service_available || status.active ||
    std::chrono::duration<double>(now - calibration_status_time_).count() > kCalibrationStatusTimeout ||
    !extnav.service_available || !extnav.odin_available || extnav.odin_session_id.empty() ||
    std::chrono::duration<double>(now - navigation_extnav_time_).count() > kCalibrationStatusTimeout ||
    !calibration_owner_client_->service_is_ready())
  {
    fail_active_task("自动校准服务/extnav不可用或存在并发操作", true); return;
  }
  calibration_started_ = now;
  calibration_renewed_ = now;
  calibration_instance_ = status.service_instance_id;
  calibration_session_ = extnav.odin_session_id;
  calibration_window_revision_ = status.window_revision;
  calibration_base_revision_ = extnav.revision;
  calibration_source_ = "onboard-calibration-" + std::to_string(get_clock()->now().nanoseconds());
  calibration_job_.clear();
  calibration_follow_fusion_ = false;
  calibration_hold_initialized_ = false;
  calibration_using_raw_velocity_ = false;
  calibration_application_stamp_ = 0;
  calibration_stable_since_ = SteadyTime{};
  reference_.velocity.setZero();
  reference_.acceleration.setZero();
  target_yaw_rate_ = 0.0;
  calibration_detail_ = "航点 " + std::to_string(waypoint_index_ + 1) + " 自动校准：申请独占";
  set_status_message(calibration_detail_);
  auto request = std::make_shared<CalibrationOwner::Request>();
  request->source_id = calibration_source_;
  request->expected_service_instance_id = calibration_instance_;
  request->acquire = true;
  auto handle = calibration_owner_client_->async_send_request(request);
  calibration_owner_future_ = handle.future.share();
  calibration_owner_request_ = handle.request_id;
  calibration_stage_ = CalibrationStage::kClaim;
}

/** Reconcile application with new bridge samples and the actual MAVROS EKF output. */
void OnboardControlNode::update_waypoint_calibration(const SteadyTime & now)
{
  using Status = correction_interfaces::msg::CorrectionStatus;
  const auto fail = [this](const std::string & reason) {
      calibration_detail_ = "自动校准失败：" + reason;
      fail_active_task(calibration_detail_, true);
    };
  if (std::chrono::duration<double>(now - calibration_started_).count() > calibration_timeout_seconds_) {
    fail("阶段总体超时（含应用与飞控融合确认）"); return;
  }
  if (!calibration_status_ || !navigation_extnav_ ||
    std::chrono::duration<double>(now - calibration_status_time_).count() > kCalibrationStatusTimeout ||
    std::chrono::duration<double>(now - navigation_extnav_time_).count() > kCalibrationStatusTimeout)
  {
    fail("校准服务/extnav掉线"); return;
  }
  const auto & status = *calibration_status_;
  const auto & extnav = *navigation_extnav_;
  if (!status.service_available || status.service_instance_id != calibration_instance_ ||
    !extnav.service_available || !extnav.odin_available || extnav.odin_session_id != calibration_session_)
  {
    fail("服务重启或Odin session失效"); return;
  }
  if (calibration_stage_ != CalibrationStage::kClaim && status.automatic_owner != calibration_source_) {
    // The claim response can precede the next status heartbeat; do not treat that old snapshot as loss.
    if (calibration_status_time_ > calibration_renewed_) {fail("自动校准独占权丢失"); return;}
  }
  if (ready(calibration_renew_future_)) {
    try {
      if (!calibration_renew_future_.get()->accepted) {fail("自动校准独占续租被拒绝"); return;}
    } catch (const std::exception & error) {fail(error.what()); return;}
    calibration_renew_future_ = {};
    calibration_renew_request_ = 0;
  }
  if (calibration_stage_ != CalibrationStage::kClaim && !calibration_renew_future_.valid() &&
    std::chrono::duration<double>(now - calibration_renewed_).count() >= kOwnerRenewInterval)
  {
    if (!calibration_owner_client_->service_is_ready()) {fail("独占服务掉线"); return;}
    auto request = std::make_shared<CalibrationOwner::Request>();
    request->source_id = calibration_source_;
    request->expected_service_instance_id = calibration_instance_;
    request->acquire = true;
    auto handle = calibration_owner_client_->async_send_request(request);
    calibration_renew_future_ = handle.future.share();
    calibration_renew_request_ = handle.request_id;
    calibration_renewed_ = now;
  }
  try {
    if (calibration_stage_ == CalibrationStage::kClaim && ready(calibration_owner_future_)) {
      const auto response = calibration_owner_future_.get();
      calibration_owner_future_ = {};
      calibration_owner_request_ = 0;
      if (!response->accepted) {fail(response->message); return;}
      calibration_renewed_ = now;
      const bool next = task_has_calibrated_ && waypoints_[waypoint_index_].continue_window;
      if (next) {calibration_stage_ = CalibrationStage::kStart;}
      else {
        auto request = std::make_shared<CalibrationClear::Request>();
        request->stamp = get_clock()->now();
        request->source_id = calibration_source_;
        request->sequence = ++calibration_sequence_;
        request->expected_service_instance_id = calibration_instance_;
        request->expected_window_revision = calibration_window_revision_;
        auto handle = calibration_clear_client_->async_send_request(request);
        calibration_clear_future_ = handle.future.share();
        calibration_clear_request_ = handle.request_id;
        calibration_stage_ = CalibrationStage::kClear;
        calibration_detail_ = "自动校准：清空服务残留窗口（保留已生效修正）";
      }
    }
    if (calibration_stage_ == CalibrationStage::kClear && ready(calibration_clear_future_)) {
      const auto response = calibration_clear_future_.get();
      calibration_clear_future_ = {};
      calibration_clear_request_ = 0;
      if (!response->accepted || response->service_instance_id != calibration_instance_) {
        fail(response->message); return;
      }
      calibration_window_revision_ = response->window_revision;
      calibration_stage_ = CalibrationStage::kStart;
    }
    if (calibration_stage_ == CalibrationStage::kStart && !calibration_start_future_.valid()) {
      // Capture the raw-frame anchor before any service can apply a correction.
      if (!update_calibration_hold(now)) {
        if (std::chrono::duration<double>(now - calibration_started_).count() > kCalibrationStatusTimeout) {
          record_control_diagnostic(calibration_hold_failure_, true, now);
          fail("原始Odin位姿/速度不可用，无法固定校准悬停点");
        }
        return;
      }
      const bool next = task_has_calibrated_ && waypoints_[waypoint_index_].continue_window;
      auto request = std::make_shared<CalibrationStart::Request>();
      request->stamp = get_clock()->now();
      request->source_id = calibration_source_;
      request->sequence = ++calibration_sequence_;
      request->operation = next ? Status::OPERATION_NEXT : Status::OPERATION_FIRST;
      request->expected_tag_id = waypoints_[waypoint_index_].calibration_tag_id;
      request->window_size = status.window_size;
      request->expected_service_instance_id = calibration_instance_;
      request->expected_window_revision = calibration_window_revision_;
      request->expected_extnav_revision = calibration_base_revision_;
      request->apply = true;
      auto handle = calibration_start_client_->async_send_request(request);
      calibration_start_future_ = handle.future.share();
      calibration_start_request_ = handle.request_id;
      calibration_detail_ = std::string("自动校准：") + (next ? "NEXT" : "FIRST") +
        " Tag " + std::to_string(request->expected_tag_id) + "，采样并自动应用";
      set_status_message(calibration_detail_);
    }
    if (calibration_stage_ == CalibrationStage::kStart && ready(calibration_start_future_)) {
      const auto response = calibration_start_future_.get();
      calibration_start_future_ = {};
      calibration_start_request_ = 0;
      if (!response->accepted || response->job_id.empty() ||
        response->service_instance_id != calibration_instance_) {fail(response->message); return;}
      calibration_job_ = response->job_id;
      calibration_stage_ = CalibrationStage::kRun;
    }
  } catch (const std::exception & error) {fail(error.what()); return;}

  if (calibration_stage_ != CalibrationStage::kRun && calibration_stage_ != CalibrationStage::kFusion) {return;}
  if (status.job_id != calibration_job_) {return;}
  // STOPPING also means normal capture cleanup before applying a frozen candidate.
  // Wait for the terminal result; the existing total timeout still covers stalled cleanup.
  if (status.state == Status::STATE_FAILED ||
    status.application_state == Status::APPLICATION_REJECTED ||
    status.application_state == Status::APPLICATION_UNKNOWN)
  {
    fail(status.last_error.empty() ? status.message : status.last_error); return;
  }
  if (status.state == Status::STATE_APPLYING || extnav.revision != calibration_base_revision_) {
    calibration_follow_fusion_ = true;
  }
  if (calibration_follow_fusion_) {
    if (calibration_velocity_time_ == SteadyTime{} ||
      std::chrono::duration<double>(now - calibration_velocity_time_).count() > pose_timeout_seconds_ ||
      !calibration_external_velocity_.allFinite())
    {
      fail("修正后 extnav 速度不可用，无法安全衔接飞控融合"); return;
    }
  }
  if (!update_calibration_hold(now)) {
    record_control_diagnostic(calibration_hold_failure_, true, now);
    fail("原始Odin位姿/速度无效或超时，无法保持校准悬停点"); return;
  }
  const bool applied = status.state == Status::STATE_SUCCEEDED && status.resources_released &&
    status.window_saved && status.extnav_applied &&
    (status.application_state == Status::APPLICATION_APPLIED_ACK ||
    status.application_state == Status::APPLICATION_APPLIED_STATUS) &&
    extnav.correction_valid && extnav.applied_job_id == calibration_job_ &&
    extnav.revision == status.extnav_revision && extnav.revision > calibration_base_revision_ &&
    extnav.final_sample_available && extnav.final_sample_correction_valid &&
    extnav.final_sample_revision == extnav.revision &&
    extnav.final_sample_odin_session_id == calibration_session_;
  if (!applied) {return;}
  if (calibration_stage_ != CalibrationStage::kFusion) {
    calibration_stage_ = CalibrationStage::kFusion;
    calibration_application_stamp_ = get_clock()->now().nanoseconds();
    calibration_detail_ = "自动校准已应用，等待本次修正后的飞控融合输出与悬停稳定";
    set_status_message(calibration_detail_);
  }
  const auto & pose = calibration_external_pose_.pose;
  tf2::Quaternion attitude;
  tf2::fromMsg(pose.orientation, attitude);
  double roll, pitch, yaw;
  tf2::Matrix3x3(attitude).getRPY(roll, pitch, yaw);
  const double distance = std::hypot(vehicle_.position.x() - pose.position.x,
    vehicle_.position.y() - pose.position.y);
  const bool fused = calibration_pose_stamp_ > calibration_application_stamp_ &&
    calibration_velocity_stamp_ > calibration_application_stamp_ &&
    last_pose_time_ > calibration_started_ && last_velocity_time_ > calibration_started_ &&
    std::chrono::duration<double>(now - calibration_pose_time_).count() < pose_timeout_seconds_ &&
    pose_valid_ && velocity_valid_ && std::isfinite(distance) &&
    distance <= waypoint_tolerance_ && std::abs(normalize_angle(yaw_ - yaw)) <= kFusionYawTolerance &&
    vehicle_.velocity.norm() <= waypoint_arrival_speed_tolerance_ &&
    calibration_external_velocity_.head<2>().norm() <= waypoint_arrival_speed_tolerance_ &&
    (vehicle_.velocity - calibration_external_velocity_).head<2>().norm() <=
    waypoint_arrival_speed_tolerance_;
  if (!fused) {calibration_stable_since_ = SteadyTime{}; return;}
  if (calibration_stable_since_ == SteadyTime{}) {calibration_stable_since_ = now;}
  if (std::chrono::duration<double>(now - calibration_stable_since_).count() < waypoint_hold_seconds_) {return;}
  task_has_calibrated_ = true;
  task_calibration_instance_ = calibration_instance_;
  task_calibration_session_ = calibration_session_;
  task_calibration_window_revision_ = status.window_revision;
  calibration_detail_ = "自动校准完成，飞控已融合；后续绝对航点保持不变";
  stop_waypoint_calibration();
  reset_avoidance();
  navigation_reference_join_ = NavigationReferenceJoin{};
  generator_waypoint_initialized_ = false;
  set_status_message(calibration_detail_);
  complete_waypoint();
}

/** Cancel outstanding work without waiting for camera shutdown, service replies or LAND. */
void OnboardControlNode::stop_waypoint_calibration()
{
  if (calibration_stage_ == CalibrationStage::kIdle) {return;}
  if (calibration_using_raw_velocity_) {
    // A pose callback can precede cancellation but follow the last control tick.
    // Reframe history without overwriting a new hover reference; stale raw must not block cancel.
    update_calibration_hold(SteadyClock::now(), false);
    if (vehicle_.velocity.head<2>().allFinite()) {
      controller_.rebase_horizontal_velocity(vehicle_.velocity.head<2>());
    }
  }
  calibration_using_raw_velocity_ = false;
  calibration_hold_initialized_ = false;
  calibration_follow_fusion_ = false;
  calibration_stage_ = CalibrationStage::kIdle;
  if (calibration_stop_client_->service_is_ready()) {
    auto request = std::make_shared<CalibrationStop::Request>();
    request->stamp = get_clock()->now();
    request->source_id = calibration_source_;
    request->sequence = ++calibration_sequence_;
    request->job_id = calibration_job_;
    request->expected_service_instance_id = calibration_instance_;
    calibration_stop_client_->async_send_request(request);
  }
  if (calibration_owner_client_->service_is_ready()) {
    auto request = std::make_shared<CalibrationOwner::Request>();
    request->source_id = calibration_source_;
    request->expected_service_instance_id = calibration_instance_;
    request->acquire = false;
    calibration_owner_client_->async_send_request(request);
  }
  if (calibration_owner_request_) {calibration_owner_client_->remove_pending_request(calibration_owner_request_);}
  if (calibration_clear_request_) {calibration_clear_client_->remove_pending_request(calibration_clear_request_);}
  if (calibration_start_request_) {calibration_start_client_->remove_pending_request(calibration_start_request_);}
  if (calibration_renew_request_) {calibration_owner_client_->remove_pending_request(calibration_renew_request_);}
  calibration_owner_future_ = {};
  calibration_clear_future_ = {};
  calibration_start_future_ = {};
  calibration_renew_future_ = {};
  calibration_owner_request_ = calibration_clear_request_ = calibration_start_request_ = calibration_renew_request_ = 0;
}
}  // namespace onboard_control
