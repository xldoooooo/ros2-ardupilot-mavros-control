/**
 * @file avoidance_trajectory.hpp
 * @brief Native cubic references and exact closest-point trimming for PD+DOB tracking.
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

  /** Evaluate a cubic segment and its first two time derivatives. */
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

/** Preserve native timing for complete and receding-horizon trajectories. */
class AvoidanceTrajectory
{
public:
  /** Validate numerical integrity; native planning owns velocity and acceleration limits. */
  bool assign(
    std::vector<AvoidancePolynomial> segments, const Eigen::Vector3d & goal,
    bool reaches_goal, std::string & reason)
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
        !s.coefficients.allFinite() || !std::isfinite(duration += s.duration))
      {
        reason = "invalid coefficients or duration";
        return false;
      }
      const auto first = s.sample(0.0), last = s.sample(s.duration);
      if (!first.position.allFinite() || !first.velocity.allFinite() ||
        !first.acceleration.allFinite() || !last.position.allFinite() ||
        !last.velocity.allFinite() || !last.acceleration.allFinite())
      {
        reason = "non-finite polynomial evaluation";
        return false;
      }
      if (i > 0) {
        const auto previous = segments[i - 1].sample(segments[i - 1].duration);
        if ((previous.position - first.position).norm() > 1e-5 ||
          (previous.velocity - first.velocity).norm() > 1e-5)
        {
          reason = "trajectory is not C1 continuous";
          return false;
        }
      }
    }
    const auto end = segments.back().sample(segments.back().duration);
    if (reaches_goal && ((end.position - goal).norm() > 0.01 || end.velocity.norm() > 0.02)) {
      reason = "trajectory has no stationary endpoint at requested waypoint";
      return false;
    }
    segments_ = std::move(segments);
    duration_ = duration;
    reaches_goal_ = reaches_goal;
    return true;
  }

  /** Check reference continuity against the latest actual vehicle state. */
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
    // Complete endpoint holds are stationary; horizon endpoints retain native derivatives.
    if (duration_ == 0.0) {
      if (reaches_goal_) {s.coefficients.rightCols<3>().setZero();}
      s.duration = duration_ = 1e-6;  // 1 微秒终点保持，不引入新的空间路径。
    }
    return elapsed;
  }

  /** Evaluate the exact native p/v/a reference, holding completed endpoints. */
  ControlReference sample(double elapsed) const
  {
    if (segments_.empty()) {return ControlReference{};}
    if (elapsed >= duration_) {
      auto end = segments_.back().sample(segments_.back().duration);
      if (reaches_goal_) {end.velocity.setZero(); end.acceleration.setZero();}
      return end;
    }
    elapsed = std::max(0.0, elapsed);
    for (const auto & segment : segments_) {
      if (elapsed <= segment.duration) {return segment.sample(elapsed);}
      elapsed -= segment.duration;
    }
    return segments_.back().sample(segments_.back().duration);
  }

  /** Report whether any polynomial segments are available. */
  bool empty() const {return segments_.empty();}
  /** Return the remaining native trajectory duration in seconds. */
  double duration() const {return duration_;}
  /** Expose native segments for visualization and review. */
  const std::vector<AvoidancePolynomial> & segments() const {return segments_;}
  /** Revoke the current trajectory and endpoint classification. */
  void clear() {segments_.clear(); duration_ = 0.0; reaches_goal_ = false;}
  /** Distinguish a final goal trajectory from an intermediate native plan. */
  bool reaches_goal() const {return reaches_goal_;}

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

  std::vector<AvoidancePolynomial> segments_;
  double duration_{0.0};
  bool reaches_goal_{false};
};
}  // namespace onboard_control
