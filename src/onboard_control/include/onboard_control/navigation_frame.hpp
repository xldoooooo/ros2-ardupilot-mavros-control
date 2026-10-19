/** @file navigation_frame.hpp
 * @brief Rigid task/local transforms and continuous reference joins for relocalization.
 */
#pragma once

#include <cmath>
#include <stdexcept>
#include <Eigen/Geometry>
#include "onboard_control/reference_generator.hpp"

namespace onboard_control
{

/** A rigid transform maps child coordinates into the parent world. */
struct NavigationTransform
{
  Eigen::Quaterniond rotation{Eigen::Quaterniond::Identity()};
  Eigen::Vector3d translation{Eigen::Vector3d::Zero()};

  /** Reject non-finite/zero rotations and normalize valid device quaternions. */
  bool normalize()
  {
    if (!translation.allFinite() || !rotation.coeffs().allFinite() ||
      !std::isfinite(rotation.norm()) || rotation.norm() < 1.0e-9) {return false;}
    rotation.normalize();
    return true;
  }

  /** Apply translation to a point, while velocities use rotation alone. */
  Eigen::Vector3d point(const Eigen::Vector3d & value) const
  {
    return translation + rotation * value;
  }

  /** Reverse the parent/child relationship without changing world conventions. */
  NavigationTransform inverse() const
  {
    const auto inverse_rotation = rotation.conjugate();
    return {inverse_rotation, -(inverse_rotation * translation)};
  }

  /** Transform a horizontal heading and project it onto the destination XY plane. */
  double yaw(double value) const
  {
    const Eigen::Vector3d heading = rotation * Eigen::Vector3d(std::cos(value), std::sin(value), 0.0);
    return std::atan2(heading.y(), heading.x());
  }
};

/** Add a smooth, persistent goal correction without restarting the original flight segment. */
class NavigationReferenceJoin
{
public:
  /** Replan only the small offset from its current P/V/A to the corrected endpoint. */
  void retarget(const Eigen::Vector3d & displacement, double yaw_delta, double duration)
  {
    duration_ = duration;
    elapsed_ = 0.0;
    target_.head<3>() = displacement;
    target_.w() = position_.w() + std::remainder(yaw_delta - position_.w(), 2.0 * std::acos(-1.0));
    coefficients_.col(0) = position_;
    coefficients_.col(1) = velocity_ * duration;
    coefficients_.col(2) = 0.5 * acceleration_ * duration * duration;
    // Coefficients are in normalized time to keep short joins well conditioned.
    const auto delta = (target_ - coefficients_.col(0)).eval();
    const auto v = coefficients_.col(1).eval();
    const auto a = coefficients_.col(2).eval();
    coefficients_.col(3) = 10.0 * delta - 6.0 * v - 3.0 * a;
    coefficients_.col(4) = -15.0 * delta + 8.0 * v + 3.0 * a;
    coefficients_.col(5) = 6.0 * delta - 3.0 * v - a;
  }

  /** Apply the remaining join to one generated sample, including its derivatives. */
  GeneratedReference update(GeneratedReference sample, double dt)
  {
    elapsed_ += std::max(0.0, dt);
    position_ = target_;
    velocity_.setZero();
    acceleration_.setZero();
    if (elapsed_ < duration_) {
      const double u = elapsed_ / duration_;
      position_.setZero();
      for (int k = 0; k < 6; ++k) {
        position_ += coefficients_.col(k) * std::pow(u, k);
        if (k >= 1) {velocity_ += k * coefficients_.col(k) * std::pow(u, k - 1) / duration_;}
        if (k >= 2) {acceleration_ += k * (k - 1) * coefficients_.col(k) * std::pow(u, k - 2) / (duration_ * duration_);}
      }
      sample.finished = false;
    }
    sample.control.position += position_.head<3>();
    sample.control.velocity += velocity_.head<3>();
    sample.control.acceleration += acceleration_.head<3>();
    sample.control.yaw += position_.w();
    sample.yaw_rate += velocity_.w();
    return sample;
  }

  /** Drop a join at a new mission, waypoint, or explicit hover boundary. */
  void clear()
  {
    duration_ = 0.0; elapsed_ = 0.0;
    target_.setZero(); position_.setZero(); velocity_.setZero(); acceleration_.setZero();
  }

private:
  // Normalized quintic coefficients for XYZ and yaw; inactive until reset.
  Eigen::Matrix<double, 4, 6> coefficients_{Eigen::Matrix<double, 4, 6>::Zero()};
  Eigen::Vector4d target_{Eigen::Vector4d::Zero()}, position_{Eigen::Vector4d::Zero()};
  Eigen::Vector4d velocity_{Eigen::Vector4d::Zero()}, acceleration_{Eigen::Vector4d::Zero()};
  double duration_{0.0};
  double elapsed_{0.0};
};
}  // namespace onboard_control
