// Original V4L2 timestamp mapping and single capture-time scheduler.
#pragma once
#include <linux/videodev2.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <utility>

namespace correction_native {
// Translate monotonic capture age into ROS time without estimating it from fps.
inline std::pair<std::int64_t, std::int64_t> capture_stamp(const v4l2_buffer &buffer,
                                                           std::int64_t monotonic,
                                                           std::int64_t ros) {
  if ((buffer.flags & V4L2_BUF_FLAG_TIMESTAMP_MASK) != V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC)
    throw std::runtime_error("UVC driver did not provide a CLOCK_MONOTONIC timestamp");
  const auto stamp = buffer.timestamp.tv_sec * 1'000'000'000LL + buffer.timestamp.tv_usec * 1000;
  const auto age = monotonic - stamp;
  if (stamp <= 0 || age < 0)
    throw std::runtime_error("UVC driver returned an invalid/future capture timestamp");
  return {ros - age, age};
}
class CaptureRateLimiter {
 public:
  // Zero retains the standalone camera tool's unlimited output mode.
  explicit CaptureRateLimiter(double fps) {
    if (!std::isfinite(fps) || fps < 0)
      throw std::invalid_argument("publish_fps must be finite and nonnegative (0 = unlimited)");
    period_ = fps > 0 ? std::max<std::int64_t>(1, std::nearbyint(1e9 / fps)) : 0;
  }
  // Skip missed periods without emitting a backlog or accumulating frame-rounding drift.
  bool accept(std::int64_t capture) {
    if (!period_) return true;
    if (!initialized_) {
      next_ = capture + period_;
      initialized_ = true;
      return true;
    }
    if (capture < next_) return false;
    next_ += ((capture - next_) / period_ + 1) * period_;
    return true;
  }

 private:
  std::int64_t period_ = 0, next_ = 0;
  bool initialized_ = false;
};
}  // namespace correction_native
