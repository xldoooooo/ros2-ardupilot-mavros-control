/**
 * @file avoidance_trajectory.hpp
 * @brief 有界三次规划轨迹求值和接入校验；不改变空间曲线或 PD+DOB。
 */
#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <string>
#include <vector>

#include <Eigen/Core>
#include "onboard_control/dob_controller.hpp"

namespace onboard_control
{
/** 每段独立使用局部秒数，系数列依次为常数、一次、二次、三次项。 */
struct AvoidancePolynomial
{
  double duration{0.0};
  Eigen::Matrix<double, 3, 4> coefficients{Eigen::Matrix<double, 3, 4>::Zero()};

  ControlReference sample(double t) const
  {
    t = std::clamp(t, 0.0, duration);
    ControlReference out;
    out.position = coefficients * Eigen::Vector4d(1.0, t, t * t, t * t * t);
    out.velocity = coefficients * Eigen::Vector4d(0.0, 1.0, 2.0 * t, 3.0 * t * t);
    out.acceleration = coefficients * Eigen::Vector4d(0.0, 0.0, 2.0, 6.0 * t);
    return out;
  }
};

/** 与当前梯形航点参考一致的首版规划包线。 */
struct AvoidanceLimits
{
  double velocity_xy{1.0}, velocity_z{0.20};  // m/s，水平采用合速度。
  double acceleration_xy{0.35}, acceleration_z{0.15};  // m/s²。
  double duration{60.0};  // 单条完整计划最大时长；超限拒绝，不截短。
};

/** 只消费完整 REACH_END 轨迹，末点静止保持不依赖不断重规划。 */
class AvoidanceTrajectory
{
public:
  bool assign(
    std::vector<AvoidancePolynomial> segments, const Eigen::Vector3d & goal,
    const AvoidanceLimits & limits, std::string & reason)
  {
    clear();
    if (segments.empty() || segments.size() > 256 || !goal.allFinite()) {
      reason = "empty/oversized trajectory or invalid goal";
      return false;
    }
    double duration = 0.0;
    for (std::size_t i = 0; i < segments.size(); ++i) {
      const auto & s = segments[i];
      if (!std::isfinite(s.duration) || s.duration <= 0.0 ||
        !s.coefficients.allFinite() || (duration += s.duration) > limits.duration)
      {
        reason = "invalid coefficients or duration";
        return false;
      }
      const auto first = s.sample(0.0), last = s.sample(s.duration);
      if (i > 0) {
        const auto previous = segments[i - 1].sample(segments[i - 1].duration);
        if ((previous.position - first.position).norm() > 1e-5 ||
          (previous.velocity - first.velocity).norm() > 1e-5)
        {
          reason = "trajectory is not C1 continuous";
          return false;
        }
      }
      // Acceleration is linear: convex norms reach their maximum at an endpoint.
      if (!within(first.acceleration, limits.acceleration_xy, limits.acceleration_z) ||
        !within(last.acceleration, limits.acceleration_xy, limits.acceleration_z))
      {
        reason = "acceleration exceeds flight envelope";
        return false;
      }
      // Quadratic velocity Bezier hulls enclose all velocities, including between samples.
      const Eigen::Vector3d middle =
        s.coefficients.col(1) + s.coefficients.col(2) * s.duration;
      if (!velocity_hull(first.velocity, middle, last.velocity,
          limits.velocity_xy, limits.velocity_z, 0))
      {
        reason = "velocity exceeds flight envelope";
        return false;
      }
    }
    const auto end = segments.back().sample(segments.back().duration);
    if ((end.position - goal).norm() > 0.01 || end.velocity.norm() > 0.02) {
      reason = "trajectory has no stationary endpoint at requested waypoint";
      return false;
    }
    segments_ = std::move(segments);
    duration_ = duration;
    return true;
  }

  bool can_join(const VehicleKinematics & actual, double position_tolerance,
    double velocity_tolerance) const
  {
    if (segments_.empty()) {return false;}
    const auto first = segments_.front().sample(0.0);
    return actual.position.allFinite() && actual.velocity.allFinite() &&
      (actual.position - first.position).norm() <= position_tolerance &&
      (actual.velocity - first.velocity).norm() <= velocity_tolerance;
  }

  ControlReference sample(double elapsed) const
  {
    if (segments_.empty()) {return ControlReference{};}
    if (elapsed >= duration_) {
      auto end = segments_.back().sample(segments_.back().duration);
      end.velocity.setZero();
      end.acceleration.setZero();
      return end;
    }
    elapsed = std::max(0.0, elapsed);
    for (const auto & segment : segments_) {
      if (elapsed <= segment.duration) {return segment.sample(elapsed);}
      elapsed -= segment.duration;
    }
    return segments_.back().sample(segments_.back().duration);
  }

  bool empty() const {return segments_.empty();}
  double duration() const {return duration_;}
  const std::vector<AvoidancePolynomial> & segments() const {return segments_;}
  void clear() {segments_.clear(); duration_ = 0.0;}

private:
  static bool within(const Eigen::Vector3d & vector, double xy, double z)
  {
    constexpr double epsilon = 1e-7;  // 仅浮点算术容差，不扩大物理规划上限。
    return vector.allFinite() && vector.head<2>().norm() <= xy + epsilon &&
      std::abs(vector.z()) <= z + epsilon;
  }

  static bool velocity_hull(
    const Eigen::Vector3d & a, const Eigen::Vector3d & b, const Eigen::Vector3d & c,
    double xy, double z, unsigned depth)
  {
    if (!within(a, xy, z) || !within(c, xy, z)) {return false;}
    if (within(b, xy, z)) {return true;}
    // De Casteljau subdivision tightens the enclosure; unresolved hulls fail conservatively.
    if (depth >= 16) {return false;}
    const Eigen::Vector3d ab = 0.5 * (a + b), bc = 0.5 * (b + c), mid = 0.5 * (ab + bc);
    return velocity_hull(a, ab, mid, xy, z, depth + 1) &&
      velocity_hull(mid, bc, c, xy, z, depth + 1);
  }

  std::vector<AvoidancePolynomial> segments_;
  double duration_{0.0};
};
}  // namespace onboard_control
