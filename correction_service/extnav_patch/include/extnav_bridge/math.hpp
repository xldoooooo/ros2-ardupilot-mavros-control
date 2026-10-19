// Fixed-size arithmetic matching the Python extnav coordinate and covariance contract.
#pragma once

#include <array>
#include <cmath>
#include <stdexcept>

namespace extnav_bridge
{
using Vector3 = std::array<double, 3>;
using Quaternion = std::array<double, 4>;
using Matrix3 = std::array<double, 9>;
using Matrix6 = std::array<double, 36>;

// Preserve the former quaternion rejection boundary and NumPy allclose defaults.
constexpr double kMinimumQuaternionNorm = 1e-9;
constexpr double kIdempotentAbsoluteTolerance = 1e-12;
constexpr double kIdempotentRelativeTolerance = 1e-5;

struct Components
{
  Vector3 position;
  Quaternion quaternion;
  Vector3 velocity;
};

struct Installation
{
  Vector3 lever;
  Quaternion q_fi;
  Quaternion q_if;
  Matrix3 rotation_fi;
};

struct Correction
{
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
  double cosine = 1.0;
  double sine = 0.0;
  Quaternion quaternion{0.0, 0.0, 0.0, 1.0};
  Matrix6 pose_transform{};
  Matrix6 twist_transform{};
};

/** Normalize xyzw and retain the original finite/zero-norm rejection. */
inline Quaternion normalize(Quaternion q)
{
  double squared_norm = 0.0;
  for (double value : q) {
    if (!std::isfinite(value)) {throw std::invalid_argument("invalid quaternion");}
    squared_norm += value * value;
  }
  const double norm = std::sqrt(squared_norm);
  if (!std::isfinite(norm) || norm < kMinimumQuaternionNorm) {
    throw std::invalid_argument("invalid quaternion");
  }
  for (double & value : q) {value /= norm;}
  return q;
}

/** Hamilton product, in the same xyzw component order as ROS. */
inline Quaternion multiply(const Quaternion & a, const Quaternion & b)
{
  const auto [x1, y1, z1, w1] = a;
  const auto [x2, y2, z2, w2] = b;
  return {
    w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
    w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
    w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
    w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2};
}

/** Convert ZYX roll/pitch/yaw without changing the multiplication convention. */
inline Quaternion rpy_quaternion(double roll, double pitch, double yaw)
{
  const double cr = std::cos(roll / 2.0), sr = std::sin(roll / 2.0);
  const double cp = std::cos(pitch / 2.0), sp = std::sin(pitch / 2.0);
  const double cy = std::cos(yaw / 2.0), sy = std::sin(yaw / 2.0);
  return {
    sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy,
    cr * cp * sy - sr * sp * cy, cr * cp * cy + sr * sp * sy};
}

/** Build the original rotation matrix from an already normalized quaternion. */
inline Matrix3 rotation_matrix(const Quaternion & q)
{
  const auto [x, y, z, w] = q;
  return {
    1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w),
    2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w),
    2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)};
}

/** Multiply a fixed 3x3 matrix and vector without temporary heap allocations. */
inline Vector3 rotate(const Matrix3 & r, const Vector3 & v)
{
  return {
    r[0] * v[0] + r[1] * v[1] + r[2] * v[2],
    r[3] * v[0] + r[4] * v[1] + r[5] * v[2],
    r[6] * v[0] + r[7] * v[1] + r[8] * v[2]};
}

/** Cache installation rotation while retaining the original lever validation. */
inline Installation installation(const Vector3 & lever, double roll, double pitch, double yaw)
{
  Installation result;
  result.lever = lever;
  result.q_fi = normalize(rpy_quaternion(roll, pitch, yaw));
  const auto [x, y, z, w] = result.q_fi;
  result.q_if = {-x, -y, -z, w};
  result.rotation_fi = rotation_matrix(normalize(result.q_fi));
  for (double value : lever) {
    if (!std::isfinite(value)) {throw std::invalid_argument("non-finite Odin lever arm");}
  }
  return result;
}

/** Cache revision-constant yaw and covariance matrices instead of rebuilding per raw. */
inline Correction correction(double x, double y, double yaw)
{
  Correction result;
  result.x = x;
  result.y = y;
  result.yaw = yaw;
  result.cosine = std::cos(yaw);
  result.sine = std::sin(yaw);
  result.quaternion = rpy_quaternion(0.0, 0.0, yaw);
  const Matrix3 r{
    result.cosine, -result.sine, 0.0,
    result.sine, result.cosine, 0.0,
    0.0, 0.0, 1.0};
  for (std::size_t row = 0; row < 3; ++row) {
    for (std::size_t col = 0; col < 3; ++col) {
      result.pose_transform[row * 6 + col] = r[row * 3 + col];
      result.pose_transform[(row + 3) * 6 + col + 3] = r[row * 3 + col];
      result.twist_transform[row * 6 + col] = r[row * 3 + col];
    }
    result.twist_transform[(row + 3) * 6 + row + 3] = 1.0;
  }
  return result;
}

/** Left-multiply the planar correction, preserving body angular velocity separately. */
inline Components planar_components(const Components & raw, const Correction & c)
{
  if (!std::isfinite(c.x) || !std::isfinite(c.y) || !std::isfinite(c.yaw)) {
    throw std::invalid_argument("non-finite correction");
  }
  return {
    {c.cosine * raw.position[0] - c.sine * raw.position[1] + c.x,
      c.sine * raw.position[0] + c.cosine * raw.position[1] + c.y, raw.position[2]},
    normalize(multiply(c.quaternion, normalize(raw.quaternion))),
    {c.cosine * raw.velocity[0] - c.sine * raw.velocity[1],
      c.sine * raw.velocity[0] + c.cosine * raw.velocity[1], raw.velocity[2]}};
}

/** Convert IMU to FCU using the current installation, origin and local-z formulas. */
inline Components fcu_components(
  const Components & raw, const Components * corrected,
  const Vector3 & angular, const Installation & mount, bool correction_valid)
{
  const Components & source = corrected ? *corrected : raw;
  const Quaternion q_odom = corrected ? source.quaternion : normalize(raw.quaternion);
  const Quaternion q_fcu = normalize(multiply(q_odom, mount.q_if));
  const Matrix3 r_fcu = rotation_matrix(q_fcu);
  // Match the original checks; velocity/angular NaN policy is intentionally unchanged.
  for (std::size_t index = 0; index < 3; ++index) {
    if (!std::isfinite(raw.position[index]) || !std::isfinite(source.position[index]) ||
      !std::isfinite(mount.lever[index]))
    {
      throw std::invalid_argument("non-finite FCU reference transform");
    }
  }
  for (double value : r_fcu) {
    if (!std::isfinite(value)) {throw std::invalid_argument("non-finite FCU reference transform");}
  }
  const Vector3 arm = rotate(r_fcu, mount.lever);
  Vector3 position;
  for (std::size_t index = 0; index < 3; ++index) {
    position[index] = correction_valid ? source.position[index] - arm[index] :
      raw.position[index] + mount.lever[index] - arm[index];
  }
  if (correction_valid) {position[2] = raw.position[2] + mount.lever[2] - arm[2];}
  const Vector3 omega = rotate(mount.rotation_fi, angular);
  const Vector3 cross{
    omega[1] * mount.lever[2] - omega[2] * mount.lever[1],
    omega[2] * mount.lever[0] - omega[0] * mount.lever[2],
    omega[0] * mount.lever[1] - omega[1] * mount.lever[0]};
  const Vector3 velocity_arm = rotate(r_fcu, cross);
  return {position, q_fcu,
    {source.velocity[0] - velocity_arm[0], source.velocity[1] - velocity_arm[1],
      source.velocity[2] - velocity_arm[2]}};
}

/** Rotate every covariance block; preserve negative-unknown and nonfinite arrays. */
inline Matrix6 rotate_covariance(const Matrix6 & covariance, const Matrix6 & transform)
{
  if (covariance[0] < 0.0) {return covariance;}
  for (double value : covariance) {
    if (!std::isfinite(value)) {return covariance;}
  }
  Matrix6 intermediate{}, result{};
  for (std::size_t row = 0; row < 6; ++row) {
    for (std::size_t col = 0; col < 6; ++col) {
      for (std::size_t index = 0; index < 6; ++index) {
        intermediate[row * 6 + col] += transform[row * 6 + index] * covariance[index * 6 + col];
      }
    }
  }
  for (std::size_t row = 0; row < 6; ++row) {
    for (std::size_t col = 0; col < 6; ++col) {
      for (std::size_t index = 0; index < 6; ++index) {
        result[row * 6 + col] += intermediate[row * 6 + index] * transform[col * 6 + index];
      }
    }
  }
  return result;
}

/** Match np.allclose(requested, active, atol=1e-12), including its default rtol. */
inline bool idempotent_close(double requested, double active)
{
  return std::abs(requested - active) <=
         kIdempotentAbsoluteTolerance + kIdempotentRelativeTolerance * std::abs(active);
}
}  // namespace extnav_bridge
