/**
 * @file waypoint_arrival_tracker.hpp
 * @brief 航点启动与入点阶段的定时重试、保持和异常阈值跟踪器。
 */
#pragma once

#include <algorithm>
#include <chrono>
#include <cmath>
#include <optional>
#include <stdexcept>

namespace onboard_control
{

/** 平滑航点启动速度检查的一次更新结果。 */
enum class WaypointStartState
{
  kWaiting,
  kReady,
  kAbnormal,
};

/**
 * 按固定间隔累计平滑航点启动速度失败，成功或显式重置时清零。
 *
 * 第一次失败立即计数；达到阈值后保持异常锁存，防止稍后速度偶然下降时
 * 又启动原巡检任务。新的航点任务会显式 reset 后重新判定。
 */
class WaypointStartTracker
{
public:
  using Clock = std::chrono::steady_clock;
  using TimePoint = Clock::time_point;

  WaypointStartTracker(const double retry_interval_seconds, const int failure_limit)
  : retry_interval_seconds_(retry_interval_seconds), failure_limit_(failure_limit)
  {
    if (!std::isfinite(retry_interval_seconds_) || retry_interval_seconds_ <= 0.0 ||
      failure_limit_ < 1)
    {
      throw std::invalid_argument("航点启动重试参数非法");
    }
  }

  /** 记录本次速度检查；合格立即返回 ready，失败按间隔计数。 */
  WaypointStartState update(const TimePoint & now, const bool speed_satisfied)
  {
    if (abnormal_) {
      return WaypointStartState::kAbnormal;
    }
    if (speed_satisfied) {
      reset();
      return WaypointStartState::kReady;
    }
    if (!last_failed_attempt_.has_value() ||
      std::chrono::duration<double>(now - *last_failed_attempt_).count() >=
      retry_interval_seconds_)
    {
      failure_count_ = std::min(failure_count_ + 1, failure_limit_);
      last_failed_attempt_ = now;
      abnormal_ = failure_count_ >= failure_limit_;
    }
    return abnormal_ ? WaypointStartState::kAbnormal : WaypointStartState::kWaiting;
  }

  /** 新任务、取消或异常清除时清除本次启动判定历史。 */
  void reset()
  {
    failure_count_ = 0;
    abnormal_ = false;
    last_failed_attempt_.reset();
  }

  int failure_count() const {return failure_count_;}

private:
  double retry_interval_seconds_;
  int failure_limit_;
  int failure_count_{0};
  bool abnormal_{false};
  std::optional<TimePoint> last_failed_attempt_;
};

/** 一次入点更新后的权威状态。 */
enum class WaypointArrivalState
{
  kApproaching,
  kHolding,
  kReached,
  kAbnormal,
};

/**
 * 只在飞机首次进入航点候选区后开始计数。
 *
 * 进入候选区前的正常航行不计失败；候选区内速度过大或随后漂出
 * 时，首次立即计数，之后按可配间隔采样。真正持续稳定后才清零。
 */
class WaypointArrivalTracker
{
public:
  using Clock = std::chrono::steady_clock;
  using TimePoint = Clock::time_point;

  WaypointArrivalTracker(const double retry_interval_seconds, const int failure_limit)
  : retry_interval_seconds_(retry_interval_seconds), failure_limit_(failure_limit)
  {
    if (!std::isfinite(retry_interval_seconds_) || retry_interval_seconds_ <= 0.0 ||
      failure_limit_ < 1)
    {
      throw std::invalid_argument("航点入点重试参数非法");
    }
  }

  /** 用当前入点条件推进计数和持续稳定计时。 */
  WaypointArrivalState update(
    const TimePoint & now,
    const bool attempt_candidate,
    const bool arrival_satisfied,
    const double hold_seconds)
  {
    if (!std::isfinite(hold_seconds) || hold_seconds < 0.0) {
      throw std::invalid_argument("航点入点保持时间非法");
    }
    if (!attempt_started_) {
      if (!attempt_candidate) {
        return WaypointArrivalState::kApproaching;
      }
      attempt_started_ = true;
    }

    if (!arrival_satisfied) {
      hold_started_.reset();
      if (!last_failed_attempt_.has_value() ||
        std::chrono::duration<double>(now - *last_failed_attempt_).count() >=
        retry_interval_seconds_)
      {
        failure_count_ = std::min(failure_count_ + 1, failure_limit_);
        last_failed_attempt_ = now;
      }
      return failure_count_ >= failure_limit_ ?
             WaypointArrivalState::kAbnormal : WaypointArrivalState::kApproaching;
    }

    if (!hold_started_.has_value()) {
      hold_started_ = now;
      return hold_seconds == 0.0 ? complete() : WaypointArrivalState::kHolding;
    }
    if (std::chrono::duration<double>(now - *hold_started_).count() < hold_seconds) {
      return WaypointArrivalState::kHolding;
    }
    return complete();
  }

  /** 新航点、新任务或取消时清除本次入点历史。 */
  void reset()
  {
    attempt_started_ = false;
    failure_count_ = 0;
    last_failed_attempt_.reset();
    hold_started_.reset();
  }

  int failure_count() const {return failure_count_;}

private:
  WaypointArrivalState complete()
  {
    reset();
    return WaypointArrivalState::kReached;
  }

  double retry_interval_seconds_;
  int failure_limit_;
  bool attempt_started_{false};
  int failure_count_{0};
  std::optional<TimePoint> last_failed_attempt_;
  std::optional<TimePoint> hold_started_;
};

}  // namespace onboard_control
