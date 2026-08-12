/**
 * @file dob_controller.cpp
 * @brief 三轴位置 PD、一阶 DOB、姿态生成和推力安全映射实现。
 */
#include "onboard_control/dob_controller.hpp"

#include <algorithm>
#include <cmath>

namespace onboard_control
{
namespace
{

constexpr double kEpsilon = 1.0e-8;

/** 按向量模长限幅，避免改变未饱和方向。 */
Eigen::Vector2d clamp_norm(const Eigen::Vector2d & value, const double maximum)
{
  const double norm = value.norm();
  if (norm <= maximum || norm < kEpsilon) {
    return value;
  }
  return value * (maximum / norm);
}

}  // namespace

DobController::DobController(const ControllerParameters & parameters)
: parameters_(parameters)
{
  reset();
}

void DobController::reset()
{
  observer_state_.setZero();
  disturbance_.setZero();
  previous_input_.setZero();
}

bool DobController::set_hover_throttle(const double hover_throttle) noexcept
{
  // ArduPilot 对 MOT_THST_HOVER 的有效约束更窄；这里保留通用 (0, 1) 接口。
  if (!std::isfinite(hover_throttle) || hover_throttle <= 0.0 || hover_throttle >= 1.0) {
    return false;
  }
  parameters_.hover_throttle = hover_throttle;
  return true;
}

bool DobController::set_gain_profile(const ControllerParameters & profile) noexcept
{
  const bool valid = std::isfinite(profile.wn_xy) && profile.wn_xy > 0.0 &&
    std::isfinite(profile.zeta_xy) && profile.zeta_xy > 0.0 &&
    std::isfinite(profile.wn_z) && profile.wn_z > 0.0 &&
    std::isfinite(profile.zeta_z) && profile.zeta_z > 0.0 &&
    std::isfinite(profile.observer_xy) && profile.observer_xy >= 0.0 &&
    std::isfinite(profile.observer_z) && profile.observer_z >= 0.0;
  if (!valid) {
    return false;
  }
  parameters_.wn_xy = profile.wn_xy;
  parameters_.zeta_xy = profile.zeta_xy;
  parameters_.wn_z = profile.wn_z;
  parameters_.zeta_z = profile.zeta_z;
  parameters_.observer_xy = profile.observer_xy;
  parameters_.observer_z = profile.observer_z;
  reset();
  return true;
}

ControlOutput DobController::compute(
  const VehicleKinematics & state,
  const ControlReference & reference,
  const double dt_seconds,
  const bool use_acceleration_feedforward)
{
  ControlOutput output;
  if (!state.position.allFinite() || !state.velocity.allFinite() ||
    !reference.position.allFinite() || !reference.velocity.allFinite() ||
    !reference.acceleration.allFinite() ||
    !std::isfinite(reference.yaw) || !std::isfinite(dt_seconds))
  {
    return output;
  }

  // 控制器允许有限调度抖动，但不会把异常长周期注入观测器。
  const double dt = std::clamp(dt_seconds, 0.001, 0.05);
  const double kp_xy = parameters_.wn_xy * parameters_.wn_xy;
  const double kd_xy = 2.0 * parameters_.zeta_xy * parameters_.wn_xy;
  const double kp_z = parameters_.wn_z * parameters_.wn_z;
  const double kd_z = 2.0 * parameters_.zeta_z * parameters_.wn_z;

  const Eigen::Vector3d position_error = reference.position - state.position;
  const Eigen::Vector3d velocity_error = reference.velocity - state.velocity;
  Eigen::Vector3d nominal;
  nominal.x() = kp_xy * position_error.x() + kd_xy * velocity_error.x();
  nominal.y() = kp_xy * position_error.y() + kd_xy * velocity_error.y();
  nominal.z() = kp_z * position_error.z() + kd_z * velocity_error.z();
  if (use_acceleration_feedforward) {
    // 轨迹控制只比既有位置 PD 多这一项；基线分支保持原数值路径不变。
    nominal += reference.acceleration;
  }

  // 一阶 DOB 的半隐式离散形式与原项目算法一致，并对估计值增加安全限幅。
  const Eigen::Vector3d observer_gain(
    parameters_.observer_xy, parameters_.observer_xy, parameters_.observer_z);
  for (Eigen::Index axis = 0; axis < 3; ++axis) {
    const double gain = observer_gain(axis);
    const double denominator = 1.0 + gain * dt;
    observer_state_(axis) =
      (observer_state_(axis) - gain * gain * dt * state.velocity(axis) -
      gain * previous_input_(axis) * dt) / denominator;
    disturbance_(axis) = std::clamp(
      observer_state_(axis) + gain * state.velocity(axis),
      -parameters_.max_disturbance,
      parameters_.max_disturbance);
  }

  Eigen::Vector3d motion_acceleration = nominal - disturbance_;
  const Eigen::Vector2d limited_xy = clamp_norm(
    motion_acceleration.head<2>(), parameters_.max_acceleration_xy);
  motion_acceleration.x() = limited_xy.x();
  motion_acceleration.y() = limited_xy.y();
  motion_acceleration.z() = std::clamp(
    motion_acceleration.z(),
    -parameters_.max_acceleration_z,
    parameters_.max_acceleration_z);

  // 推力方向需包含重力补偿；最小竖直分量和倾角约束防止翻转命令。
  Eigen::Vector3d total_acceleration = motion_acceleration;
  total_acceleration.z() += parameters_.gravity;
  total_acceleration.z() = std::max(
    total_acceleration.z(), parameters_.min_total_acceleration_z);
  const double tilt_limited_xy = total_acceleration.z() * std::tan(parameters_.max_tilt_rad);
  const Eigen::Vector2d attitude_xy = clamp_norm(
    total_acceleration.head<2>(),
    std::min(parameters_.max_acceleration_xy, tilt_limited_xy));
  total_acceleration.x() = attitude_xy.x();
  total_acceleration.y() = attitude_xy.y();

  const double acceleration_norm = total_acceleration.norm();
  if (acceleration_norm < kEpsilon || !std::isfinite(acceleration_norm)) {
    return output;
  }

  const Eigen::Vector3d body_z = total_acceleration / acceleration_norm;
  const Eigen::Vector3d yaw_heading(std::cos(reference.yaw), std::sin(reference.yaw), 0.0);
  Eigen::Vector3d body_x = yaw_heading - yaw_heading.dot(body_z) * body_z;
  if (body_x.norm() < kEpsilon) {
    body_x = Eigen::Vector3d::UnitX();
  }
  body_x.normalize();
  Eigen::Vector3d body_y = body_z.cross(body_x);
  if (body_y.norm() < kEpsilon) {
    return output;
  }
  body_y.normalize();
  body_x = body_y.cross(body_z).normalized();

  Eigen::Matrix3d rotation;
  rotation.col(0) = body_x;
  rotation.col(1) = body_y;
  rotation.col(2) = body_z;
  output.attitude = Eigen::Quaterniond(rotation).normalized();

  // 质量在重量与推力中约去；保留原项目分段线性油门映射。
  const double ratio = acceleration_norm / parameters_.gravity;
  if (ratio <= 1.0) {
    output.thrust = parameters_.hover_throttle * ratio;
  } else {
    const double denominator = std::max(parameters_.thrust_ratio - 1.0, kEpsilon);
    output.thrust = parameters_.hover_throttle +
      (1.0 - parameters_.hover_throttle) * (ratio - 1.0) / denominator;
  }
  output.thrust = std::clamp(output.thrust, 0.0, 1.0);
  output.acceleration = motion_acceleration;
  output.disturbance = disturbance_;
  output.valid = output.attitude.coeffs().allFinite() && std::isfinite(output.thrust);
  previous_input_ = motion_acceleration;
  return output;
}

}  // namespace onboard_control
