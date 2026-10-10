/**
 * @file avoidance_trajectory.hpp
 * @brief 有界三次轨迹求值、最近点接入和校验；裁掉过期前缀，不改变空间曲线或 PD+DOB。
 */
#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
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

  /** 全部连续段的三维最近点；等距时选择较晚时刻，避免重走已越过的重合路径。 */
  double closest_time(const Eigen::Vector3d & position) const
  {
    if (empty() || !position.allFinite()) {return 0.0;}
    double best_time = 0.0, best_distance = (sample(0.0).position - position).squaredNorm();
    double offset = 0.0;
    for (const auto & s : segments_) {
      // 在 u=t/duration 的 [0,1] 上求 (p(u)-actual)·p'(u)=0，最高五次。
      auto c = s.coefficients;
      double scale = 1.0;
      for (int i = 1; i < 4; ++i) {scale *= s.duration; c.col(i) *= scale;}
      c.col(0) -= position;
      std::vector<double> derivative(6, 0.0);
      for (int i = 0; i < 4; ++i) {
        for (int j = 1; j < 4; ++j) {derivative[i + j - 1] += j * c.col(i).dot(c.col(j));}
      }
      auto times = unit_interval_roots(std::move(derivative));
      times.push_back(0.0); times.push_back(1.0);
      for (double u : times) {
        const double t = offset + u * s.duration;
        const double distance = (s.sample(u * s.duration).position - position).squaredNorm();
        // ARM 的 FMA 与 x86 在重合点可差几个 ulp；仅用浮点距离容差判定等距。
        const double epsilon = 64.0 * std::numeric_limits<double>::epsilon() *
          std::max({1.0, distance, best_distance});
        if (distance < best_distance - epsilon ||
          (std::abs(distance - best_distance) <= epsilon && t > best_time))
        {
          best_distance = distance; best_time = t;
        }
      }
      offset += s.duration;
    }
    return best_time;
  }

  /** 将最近点设为新的零秒；解析平移首段系数，保留 p/v/a 和后续段的原始时间尺度。 */
  double start_at_closest(const Eigen::Vector3d & position)
  {
    const double elapsed = closest_time(position);
    if (empty() || elapsed <= 0.0) {return elapsed;}
    double local = elapsed;
    std::size_t first = 0;
    while (first + 1 < segments_.size() && local >= segments_[first].duration) {
      local -= segments_[first++].duration;
    }
    segments_.erase(segments_.begin(), segments_.begin() + first);
    auto & s = segments_.front();
    const auto start = s.sample(local);
    s.coefficients.col(0) = start.position;
    s.coefficients.col(1) = start.velocity;
    s.coefficients.col(2) += 3.0 * local * s.coefficients.col(3);
    s.duration = std::max(0.0, s.duration - local);
    duration_ = std::max(0.0, duration_ - elapsed);
    // 已在终点时保持静止；后续 VALIDATE 请求仍需要一个合法的正时长常值段。
    if (duration_ == 0.0) {
      s.coefficients.rightCols<3>().setZero();
      s.duration = duration_ = 1e-6;  // 1 微秒终点保持，不引入新的空间路径。
    }
    return elapsed;
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
  /** 导数根将多项式切为单调区间；二分求根并保留重根处的分区点，含降阶/常值段。 */
  static std::vector<double> unit_interval_roots(std::vector<double> c)
  {
    while (c.size() > 1 && c.back() == 0.0) {c.pop_back();}
    if (c.size() <= 1) {return {};}
    const auto value = [&c](double u) {
        double result = 0.0;
        for (auto i = c.rbegin(); i != c.rend(); ++i) {result = result * u + *i;}
        return result;
      };
    std::vector<double> derivative;
    for (std::size_t i = 1; i < c.size(); ++i) {derivative.push_back(i * c[i]);}
    auto cuts = unit_interval_roots(std::move(derivative));
    // 分区点包含驻点，直接作为候选也能覆盖偶数重根，不依赖浮点零判定。
    std::vector<double> roots = cuts;
    cuts.insert(cuts.begin(), 0.0); cuts.push_back(1.0);
    for (std::size_t i = 1; i < cuts.size(); ++i) {
      double lo = cuts[i - 1], hi = cuts[i], left = value(lo), right = value(hi);
      if (left == 0.0) {roots.push_back(lo);}
      if (right == 0.0) {roots.push_back(hi);}
      if (left == 0.0 || right == 0.0 || std::signbit(left) == std::signbit(right)) {continue;}
      for (int step = 0; step < 50; ++step) {  // [0,1] 时间误差 < 9e-16。
        const double mid = 0.5 * (lo + hi), middle = value(mid);
        if (middle == 0.0) {lo = hi = mid; break;}
        if (std::signbit(middle) == std::signbit(left)) {lo = mid; left = middle;}
        else {hi = mid;}
      }
      roots.push_back(0.5 * (lo + hi));
    }
    std::sort(roots.begin(), roots.end());
    roots.erase(std::unique(roots.begin(), roots.end()), roots.end());
    return roots;
  }

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
