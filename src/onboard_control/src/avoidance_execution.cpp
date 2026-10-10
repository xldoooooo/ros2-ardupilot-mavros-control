/**
 * @file avoidance_execution.cpp
 * @brief 航点避障请求关联、暂停恢复和三次轨迹执行；不包含地图或第二套控制器。
 */
#include "onboard_control/onboard_control_node.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace onboard_control
{
namespace
{
using Request = guided_interfaces::msg::AvoidanceRequest;
using Result = guided_interfaces::msg::AvoidanceResult;

geometry_msgs::msg::Point point(const Eigen::Vector3d & v)
{
  geometry_msgs::msg::Point p;
  p.x = v.x(); p.y = v.y(); p.z = v.z();
  return p;
}
}  // namespace

bool OnboardControlNode::validate_waypoint_configuration(
  std::uint8_t strategy, std::uint8_t generator, std::uint8_t tracking,
  std::string & reason) const
{
  if (strategy > 2 || generator > 3 || tracking > 1) {
    reason = "未知避障策略、参考生成器或跟踪器";
    return false;
  }
  if (strategy != 0 && generator == 0) {
    reason = "避障任务需要平滑参考，不能选择位置阶跃";
    return false;
  }
  if (strategy == 1 && tracking != 1) {
    reason = "自主避障必须选择轨迹PD+DOB以执行规划p/v/a参考";
    return false;
  }
  return true;
}

bool OnboardControlNode::avoidance_ready(const SteadyTime & now) const
{
  // Heartbeat is capability information only; it never grants a trajectory execution permit.
  return avoidance_component_ready_ && !avoidance_bridge_session_.empty() &&
    avoidance_heartbeat_time_ != SteadyTime{} &&
    std::chrono::duration<double>(now - avoidance_heartbeat_time_).count() <= 2.0;
}

void OnboardControlNode::reset_avoidance()
{
  ++avoidance_task_revision_;
  avoidance_pending_.reset();
  avoidance_trajectory_.clear();
  avoidance_trajectory_started_.reset();
  avoidance_wait_started_.reset();
  avoidance_clear_since_.reset();
  avoidance_last_valid_.reset();
  avoidance_state_ = 0;
  avoidance_detail_.clear();
  publish_avoidance_path();
}

void OnboardControlNode::begin_avoidance_wait(
  const SteadyTime & now, const std::string & reason)
{
  if (!avoidance_wait_started_) {
    avoidance_wait_started_ = now;
    // 独立等待不调用enter_hover：保持业务任务、方法锁和航点索引。
    reference_.position = vehicle_.position;
    reference_.velocity.setZero();
    reference_.acceleration.setZero();
    target_yaw_rate_ = 0.0;
    waypoint_arrival_tracker_.reset();
    set_status_message("避障制动等待：" + reason, StatusLogLevel::kWarn);
    publish_result(active_command_, guided_interfaces::msg::CommandResult::STATUS_RUNNING,
      false, "避障制动等待：" + reason,
      static_cast<std::uint32_t>(waypoint_index_ + 1),
      static_cast<std::uint32_t>(waypoints_.size()));
  }
  avoidance_state_ = vehicle_.velocity.norm() > waypoint_arrival_speed_tolerance_ ? 3 : 1;
  avoidance_detail_ = reason;
  avoidance_last_valid_.reset();
  avoidance_clear_since_.reset();
  avoidance_trajectory_.clear();
  avoidance_trajectory_started_.reset();
  active_reference_phase_ = ReferencePhase::kIdle;
  publish_avoidance_path();
}

void OnboardControlNode::avoidance_tick()
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  // 地面preview按接收超时撤销显示；重复发布也让飞行期间新打开的RViz看到当前轨迹。
  publish_avoidance_path();
  const auto steady_now = SteadyClock::now();
  if (avoidance_land_latched_ && armed_ && fcu_connected_ && autopilot_mode_ != "LAND") {
    // A negative ACK or temporarily missing service must not strand a latched timeout in GUIDED.
    // Only this avoidance timeout path retries; original non-avoidance LAND semantics remain intact.
    if (avoidance_land_service_request_ &&
      std::chrono::duration<double>(steady_now - avoidance_last_land_attempt_).count() > 2.0)
    {
      set_mode_client_->remove_pending_request(*avoidance_land_service_request_);
      avoidance_land_service_request_.reset();
      ++avoidance_land_request_generation_;
    }
    if (!avoidance_land_service_request_ &&
      std::chrono::duration<double>(steady_now - avoidance_last_land_attempt_).count() >= 1.0)
    {
      avoidance_last_land_attempt_ = steady_now;
      if (active_task_ == ActiveTask::kLand) {send_land_mode_request(active_command_, true);}
      else {start_land(CommandIdentity{"onboard-avoidance-land",
          ++avoidance_request_sequence_, "failsafe_land"}, true);}
    }
    return;
  }
  if (waypoint_flight_strategy_ == 0 || active_task_ != ActiveTask::kWaypoint ||
    control_mode_ != guided_interfaces::msg::ControlStatus::MODE_WAYPOINT ||
    waypoint_index_ >= waypoints_.size() || avoidance_land_latched_)
  {
    return;
  }
  if (avoidance_pending_) {
    if (std::chrono::duration<double>(steady_now - avoidance_pending_->sent).count() <=
      avoidance_result_timeout_seconds_) {return;}
    avoidance_pending_.reset();
    begin_avoidance_wait(steady_now, "规划请求超时或组件断流");
  }
  Request request;
  request.header.stamp = get_clock()->now();
  request.header.frame_id = "map";
  request.controller_session = avoidance_controller_session_;
  request.task_revision = avoidance_task_revision_;
  request.waypoint_index = static_cast<std::uint32_t>(waypoint_index_ + 1);
  request.request_id = ++avoidance_request_sequence_;
  request.mode = waypoint_flight_strategy_ == 2 ? Request::CHECK_LINE :
    (avoidance_trajectory_started_ ? Request::VALIDATE_TRAJECTORY : Request::PLAN);
  request.start = point(vehicle_.position);
  request.velocity.x = vehicle_.velocity.x();
  request.velocity.y = vehicle_.velocity.y();
  request.velocity.z = vehicle_.velocity.z();
  request.goal = waypoints_[waypoint_index_].position;
  if (request.mode == Request::VALIDATE_TRAJECTORY) {
    request.trajectory_elapsed = std::min(avoidance_trajectory_.duration(),
      std::chrono::duration<double>(steady_now - *avoidance_trajectory_started_).count());
    request.segments.reserve(avoidance_trajectory_.segments().size());
    for (const auto & segment : avoidance_trajectory_.segments()) {
      guided_interfaces::msg::PolynomialSegment wire;
      wire.duration = segment.duration;
      for (std::size_t k = 0; k < 4; ++k) {
        wire.x[k] = segment.coefficients(0, k);
        wire.y[k] = segment.coefficients(1, k);
        wire.z[k] = segment.coefficients(2, k);
      }
      request.segments.push_back(wire);
    }
  }
  avoidance_pending_ = AvoidancePending{request.request_id, request.mode, steady_now};
  avoidance_request_publisher_->publish(request);
}

void OnboardControlNode::on_avoidance_result(const Result::SharedPtr message)
{
  // Polynomial validation occurs outside the control mutex; message work cannot hold a tick.
  AvoidanceTrajectory candidate;
  std::string trajectory_error;
  std::vector<AvoidancePolynomial> segments;
  if (message->mode == Request::PLAN && message->status == Result::REACH_END) {
    if (message->segments.size() <= 256) {
      segments.reserve(message->segments.size());
      for (const auto & wire : message->segments) {
        AvoidancePolynomial s;
        s.duration = wire.duration;
        for (std::size_t k = 0; k < 4; ++k) {
          s.coefficients(0, k) = wire.x[k];
          s.coefficients(1, k) = wire.y[k];
          s.coefficients(2, k) = wire.z[k];
        }
        segments.push_back(s);
      }
    }
  }
  // Read only the current target under the mutex, then check numerical envelope independently.
  Eigen::Vector3d target = Eigen::Vector3d::Zero();
  {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    if (waypoint_index_ < waypoints_.size()) {
      const auto & p = waypoints_[waypoint_index_].position;
      target = {p.x, p.y, p.z};
    }
  }
  if (message->mode == Request::PLAN && message->status == Result::REACH_END) {
    candidate.assign(std::move(segments), target, avoidance_limits_, trajectory_error);
  }

  std::lock_guard<std::recursive_mutex> lock(mutex_);
  const auto now = SteadyClock::now();
  if (message->request_id == 0) {
    if (message->bridge_session.empty() || message->header.frame_id != "map") {return;}
    const bool changed = !avoidance_bridge_session_.empty() &&
      (avoidance_bridge_session_ != message->bridge_session ||
      avoidance_coordinate_revision_ != message->coordinate_revision ||
      (!avoidance_planner_session_.empty() && !message->planner_session.empty() &&
      avoidance_planner_session_ != message->planner_session));
    avoidance_bridge_session_ = message->bridge_session;
    avoidance_coordinate_revision_ = message->coordinate_revision;
    if (!message->planner_session.empty()) {avoidance_planner_session_ = message->planner_session;}
    avoidance_component_ready_ = message->ready;
    avoidance_heartbeat_time_ = now;
    if (changed && waypoint_flight_strategy_ != 0 && active_task_ == ActiveTask::kWaypoint) {
      avoidance_pending_.reset();
      begin_avoidance_wait(now, "规划进程/坐标版本变化，撤销旧轨迹");
    }
    return;
  }
  if (active_task_ != ActiveTask::kWaypoint || waypoint_flight_strategy_ == 0 ||
    avoidance_land_latched_ || !avoidance_pending_ ||
    message->controller_session != avoidance_controller_session_ ||
    message->task_revision != avoidance_task_revision_ ||
    message->waypoint_index != waypoint_index_ + 1 ||
    message->request_id != avoidance_pending_->id || message->mode != avoidance_pending_->mode)
  {
    return;
  }
  const auto sent = avoidance_pending_->sent;
  avoidance_pending_.reset();
  // Result callbacks must honor the same deadline as the control tick; a late success
  // cannot clear the wait just before the next tick would otherwise latch LAND.
  if (avoidance_wait_started_ &&
    std::chrono::duration<double>(now - *avoidance_wait_started_).count() >=
    avoidance_wait_timeout_seconds_)
  {
    avoidance_land_latched_ = true;
    fail_active_task("避障等待超时，当前航点失败，锁存原地LAND", true);
    return;
  }
  const bool fresh = std::chrono::duration<double>(now - sent).count() <=
    avoidance_result_timeout_seconds_;
  const bool identity = message->bridge_session == avoidance_bridge_session_ &&
    message->coordinate_revision == avoidance_coordinate_revision_ &&
    message->header.frame_id == "map" && !message->planner_session.empty();
  if (!fresh || !identity || !message->ready || message->status != Result::REACH_END) {
    begin_avoidance_wait(now, !fresh ? "规划结果超龄" :
      (!identity ? "规划坐标或会话不匹配" : message->detail));
    return;
  }
  if (!avoidance_planner_session_.empty() &&
    avoidance_planner_session_ != message->planner_session)
  {
    avoidance_planner_session_ = message->planner_session;
    begin_avoidance_wait(now, "规划器重启，等待重新接入");
    return;
  }
  avoidance_planner_session_ = message->planner_session;
  if (message->mode == Request::PLAN) {
    // 用接收时最新 FCU 位置投影到连续曲线，裁掉求解/DDS 延迟期间已越过的前缀。
    candidate.start_at_closest(vehicle_.position);
    if (candidate.empty() || !candidate.can_join(vehicle_, avoidance_join_position_tolerance_,
      avoidance_join_velocity_tolerance_))
    {
      begin_avoidance_wait(now, candidate.empty() ? trajectory_error : "规划最近点p/v无法平稳接入");
      return;
    }
  }
  avoidance_last_valid_ = sent;
  if (avoidance_wait_started_) {
    const bool stopped = vehicle_.velocity.norm() <= waypoint_arrival_speed_tolerance_;
    if (!stopped) {avoidance_clear_since_.reset(); return;}
    if (!avoidance_clear_since_) {avoidance_clear_since_ = now; return;}
    if (std::chrono::duration<double>(now - *avoidance_clear_since_).count() <
      avoidance_resume_seconds_) {return;}
    if (message->mode == Request::PLAN) {
      avoidance_trajectory_ = std::move(candidate);
      avoidance_trajectory_started_ = now;  // 裁剪后的最近点是零秒，所有消费者共享此时间原点。
    } else {
      reference_generator_->reset(vehicle_.position, reference_.yaw, target,
        normalize_angle(waypoints_[waypoint_index_].yaw));
    }
    avoidance_wait_started_.reset();
    avoidance_clear_since_.reset();
    avoidance_state_ = 2;
    avoidance_detail_ = "当前航点参考有效，继续原任务";
    waypoint_arrival_tracker_.reset();
    set_status_message("避障等待结束，继续同一航点");
    publish_avoidance_path();
  } else if (message->mode == Request::PLAN) {
    avoidance_trajectory_ = std::move(candidate);
    avoidance_trajectory_started_ = now;
    avoidance_state_ = 2;
    publish_avoidance_path();
  }
}

bool OnboardControlNode::update_avoidance_reference(const SteadyTime & now, double dt)
{
  if (!avoidance_ready(now) || !avoidance_last_valid_ ||
    std::chrono::duration<double>(now - *avoidance_last_valid_).count() >
    avoidance_result_timeout_seconds_)
  {
    // Repeated failures do not move the hover position or restart the waiting deadline.
    if (!avoidance_wait_started_ || avoidance_last_valid_) {
      begin_avoidance_wait(now, "避障输入/结果未就绪或超龄");
    }
  }
  if (avoidance_wait_started_) {
    if (std::chrono::duration<double>(now - *avoidance_wait_started_).count() >=
      avoidance_wait_timeout_seconds_)
    {
      avoidance_land_latched_ = true;
      fail_active_task("避障等待超时，当前航点失败，锁存原地LAND", true);
      return false;
    }
    avoidance_state_ = vehicle_.velocity.norm() > waypoint_arrival_speed_tolerance_ ? 3 : 1;
    return false;
  }
  if (waypoint_flight_strategy_ == 2) {
    const auto generated = reference_generator_->update(dt);
    reference_ = generated.control;
    target_yaw_rate_ = generated.yaw_rate;
    active_reference_phase_ = generated.phase;
  } else if (avoidance_trajectory_started_ && !avoidance_trajectory_.empty()) {
    const double elapsed = std::chrono::duration<double>(now - *avoidance_trajectory_started_).count();
    const double previous_yaw = reference_.yaw;
    reference_ = avoidance_trajectory_.sample(elapsed);
    const double difference = normalize_angle(waypoints_[waypoint_index_].yaw - previous_yaw);
    target_yaw_rate_ = std::clamp(difference / std::max(dt, 1e-6), -max_yaw_rate_, max_yaw_rate_);
    reference_.yaw = normalize_angle(previous_yaw + target_yaw_rate_ * dt);
    active_reference_phase_ = elapsed >= avoidance_trajectory_.duration() ?
      ReferencePhase::kComplete : ReferencePhase::kCruising;
  } else {
    begin_avoidance_wait(now, "无完整可执行轨迹");
    return false;
  }
  // 大跟踪误差使原规划从实际位置脱离，必须制动而非继续追逐曲线。
  if ((vehicle_.position - reference_.position).norm() > max_reference_error_xy_) {
    begin_avoidance_wait(now, "轨迹跟踪误差超过现有参考误差上限");
    return false;
  }
  return true;
}

void OnboardControlNode::publish_avoidance_path()
{
  if (!avoidance_path_publisher_) {return;}
  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  path.header.stamp = get_clock()->now();
  if (avoidance_state_ == 2 && !avoidance_trajectory_.empty()) {
    constexpr double sample_step = 0.1;  // RViz only; the controller evaluates exact coefficients.
    const std::size_t count = static_cast<std::size_t>(
      std::ceil(avoidance_trajectory_.duration() / sample_step));
    path.poses.reserve(count + 1);
    for (std::size_t i = 0; i <= count; ++i) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = path.header;
      pose.pose.position = point(avoidance_trajectory_.sample(
        std::min(i * sample_step, avoidance_trajectory_.duration())).position);
      pose.pose.orientation.w = 1.0;
      path.poses.push_back(pose);
    }
  }
  // Empty paths explicitly revoke a previous success on failure, cancel, or a new task.
  avoidance_path_publisher_->publish(path);
}
}  // namespace onboard_control
