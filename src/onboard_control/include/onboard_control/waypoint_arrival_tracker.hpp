/**
 * @file waypoint_arrival_tracker.hpp
 * @brief 航点入点保持、1 Hz 失败采样与异常阈值跟踪器。
 */
#pragma once

#include <algorithm>
#include <chrono>
#include <cmath>
#include <optional>
#include <stdexcept>

namespace onboard_control
{

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
