/**
 * @file reference_generator.cpp
 * @brief 位置阶跃、二阶滤波、梯形速度和限 jerk S 曲线解析实现。
 */
#include "onboard_control/reference_generator.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

namespace onboard_control
{
namespace
{

constexpr double kEpsilon = 1.0e-9;
constexpr double kPi = 3.14159265358979323846;

double normalize_angle(double angle)
{
  while (angle > kPi) {
    angle -= 2.0 * kPi;
  }
  while (angle < -kPi) {
    angle += 2.0 * kPi;
  }
  return angle;
}

bool finite_positive(const double value)
{
  return std::isfinite(value) && value > 0.0;
}

/** 把 XY 模长与 Z 分量限制换算为沿三维航段的标量限制。 */
double path_limit(
  const Eigen::Vector3d & direction,
  const double maximum_xy,
  const double maximum_z)
{
  double limit = std::numeric_limits<double>::infinity();
  const double horizontal = direction.head<2>().norm();
  if (horizontal > kEpsilon) {
    limit = std::min(limit, maximum_xy / horizontal);
  }
  if (std::abs(direction.z()) > kEpsilon) {
    limit = std::min(limit, maximum_z / std::abs(direction.z()));
  }
  return std::isfinite(limit) ? limit : std::min(maximum_xy, maximum_z);
}

/** 保存直线几何，只让派生类规划标量路径进度。 */
class StraightSegmentGenerator : public ReferenceGenerator
{
public:
  void reset_geometry(
    const Eigen::Vector3d & start_position,
    const double start_yaw,
    const Eigen::Vector3d & target_position,
    const double target_yaw)
  {
    start_position_ = start_position;
    target_position_ = target_position;
    const Eigen::Vector3d displacement = target_position - start_position;
    distance_ = displacement.norm();
    if (distance_ > kEpsilon) {
      direction_ = displacement / distance_;
    } else {
      direction_.setZero();
    }
    start_yaw_ = normalize_angle(start_yaw);
    yaw_delta_ = normalize_angle(target_yaw - start_yaw_);
  }

protected:
  GeneratedReference make_sample(
    const double path_position,
    const double path_velocity,
    const double path_acceleration,
    const ReferencePhase phase,
    const bool finished) const
  {
    const double bounded_position = std::clamp(path_position, 0.0, distance_);
    const double ratio = distance_ > kEpsilon ? bounded_position / distance_ : 1.0;
    GeneratedReference output;
    output.control.position = start_position_ + direction_ * bounded_position;
    output.control.velocity = direction_ * path_velocity;
    output.control.acceleration = direction_ * path_acceleration;
    output.control.yaw = normalize_angle(start_yaw_ + yaw_delta_ * ratio);
    output.yaw_rate = distance_ > kEpsilon ? yaw_delta_ * path_velocity / distance_ : 0.0;
    output.phase = phase;
    output.finished = finished;
    if (finished) {
      output.control.position = target_position_;
      output.control.velocity.setZero();
      output.control.acceleration.setZero();
      output.control.yaw = normalize_angle(start_yaw_ + yaw_delta_);
      output.yaw_rate = 0.0;
    }
    return output;
  }

  Eigen::Vector3d start_position_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d target_position_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d direction_{Eigen::Vector3d::Zero()};
  double distance_{0.0};
  double start_yaw_{0.0};
  double yaw_delta_{0.0};
};

class StepPositionGenerator final : public StraightSegmentGenerator
{
public:
  void reset(
    const Eigen::Vector3d & start_position,
    const double start_yaw,
    const Eigen::Vector3d & target_position,
    const double target_yaw) override
  {
    reset_geometry(start_position, start_yaw, target_position, target_yaw);
  }

  GeneratedReference update(const double) override {return sample();}

  GeneratedReference sample() const override
  {
    return make_sample(distance_, 0.0, 0.0, ReferencePhase::kStep, true);
  }
};

class SecondOrderFilterGenerator final : public StraightSegmentGenerator
{
public:
  explicit SecondOrderFilterGenerator(const SecondOrderFilterParameters & parameters)
  : parameters_(parameters) {}

  void reset(
    const Eigen::Vector3d & start_position,
    const double start_yaw,
    const Eigen::Vector3d & target_position,
    const double target_yaw) override
  {
    reset_geometry(start_position, start_yaw, target_position, target_yaw);
    path_position_ = 0.0;
    path_velocity_ = 0.0;
    path_acceleration_ = 0.0;
    finished_ = distance_ <= kEpsilon;
    const bool horizontal = direction_.head<2>().norm() > kEpsilon;
    const bool vertical = std::abs(direction_.z()) > kEpsilon;
    natural_frequency_ = horizontal ? parameters_.wn_xy : parameters_.wn_z;
    damping_ratio_ = horizontal ? parameters_.zeta_xy : parameters_.zeta_z;
    if (horizontal && vertical) {
      natural_frequency_ = std::min(parameters_.wn_xy, parameters_.wn_z);
      damping_ratio_ = std::max(parameters_.zeta_xy, parameters_.zeta_z);
    }
    max_velocity_ = path_limit(
      direction_, parameters_.max_velocity_xy, parameters_.max_velocity_z);
    max_acceleration_ = path_limit(
      direction_, parameters_.max_acceleration_xy, parameters_.max_acceleration_z);
  }

  GeneratedReference update(const double dt_seconds) override
  {
    if (finished_) {
      return sample();
    }
    const double dt = std::clamp(dt_seconds, 0.0, 0.05);
    const double error = distance_ - path_position_;
    path_acceleration_ = natural_frequency_ * natural_frequency_ * error -
      2.0 * damping_ratio_ * natural_frequency_ * path_velocity_;
    path_acceleration_ = std::clamp(
      path_acceleration_, -max_acceleration_, max_acceleration_);
    path_velocity_ = std::clamp(
      path_velocity_ + path_acceleration_ * dt, 0.0, max_velocity_);
    path_position_ += path_velocity_ * dt;
    if (path_position_ >= distance_ ||
      (distance_ - path_position_ <= parameters_.completion_position_tolerance &&
      path_velocity_ <= parameters_.completion_velocity_tolerance))
    {
      path_position_ = distance_;
      path_velocity_ = 0.0;
      path_acceleration_ = 0.0;
      finished_ = true;
    }
    return sample();
  }

  GeneratedReference sample() const override
  {
    return make_sample(
      path_position_, path_velocity_, path_acceleration_,
      finished_ ? ReferencePhase::kComplete : ReferencePhase::kFiltering,
      finished_);
  }

private:
  SecondOrderFilterParameters parameters_;
  double natural_frequency_{1.0};
  double damping_ratio_{1.0};
  double max_velocity_{0.0};
  double max_acceleration_{0.0};
  double path_position_{0.0};
  double path_velocity_{0.0};
  double path_acceleration_{0.0};
  bool finished_{true};
};

class TrapezoidalProfileGenerator final : public StraightSegmentGenerator
{
public:
  explicit TrapezoidalProfileGenerator(const TrapezoidalProfileParameters & parameters)
  : parameters_(parameters) {}

  void reset(
    const Eigen::Vector3d & start_position,
    const double start_yaw,
    const Eigen::Vector3d & target_position,
    const double target_yaw) override
  {
    reset_geometry(start_position, start_yaw, target_position, target_yaw);
    elapsed_ = 0.0;
    max_acceleration_ = path_limit(
      direction_, parameters_.max_acceleration_xy, parameters_.max_acceleration_z);
    max_deceleration_ = path_limit(
      direction_, parameters_.max_deceleration_xy, parameters_.max_deceleration_z);
    peak_velocity_ = path_limit(
      direction_, parameters_.max_velocity_xy, parameters_.max_velocity_z);
    const double full_speed_distance = peak_velocity_ * peak_velocity_ *
      (0.5 / max_acceleration_ + 0.5 / max_deceleration_);
    if (distance_ < full_speed_distance) {
      peak_velocity_ = std::sqrt(
        2.0 * distance_ * max_acceleration_ * max_deceleration_ /
        (max_acceleration_ + max_deceleration_));
    }
    acceleration_time_ = peak_velocity_ / max_acceleration_;
    deceleration_time_ = peak_velocity_ / max_deceleration_;
    acceleration_distance_ = 0.5 * peak_velocity_ * acceleration_time_;
    deceleration_distance_ = 0.5 * peak_velocity_ * deceleration_time_;
    cruise_distance_ = std::max(0.0, distance_ - acceleration_distance_ - deceleration_distance_);
    cruise_time_ = peak_velocity_ > kEpsilon ? cruise_distance_ / peak_velocity_ : 0.0;
    total_time_ = acceleration_time_ + cruise_time_ + deceleration_time_;
  }

  GeneratedReference update(const double dt_seconds) override
  {
    elapsed_ = std::min(total_time_, elapsed_ + std::clamp(dt_seconds, 0.0, 0.05));
    return sample();
  }

  GeneratedReference sample() const override
  {
    if (distance_ <= kEpsilon || elapsed_ >= total_time_ - kEpsilon) {
      return make_sample(distance_, 0.0, 0.0, ReferencePhase::kComplete, true);
    }
    if (elapsed_ < acceleration_time_) {
      const double velocity = max_acceleration_ * elapsed_;
      const double position = 0.5 * max_acceleration_ * elapsed_ * elapsed_;
      return make_sample(
        position, velocity, max_acceleration_, ReferencePhase::kAccelerating, false);
    }
    if (elapsed_ < acceleration_time_ + cruise_time_) {
      const double cruise_elapsed = elapsed_ - acceleration_time_;
      return make_sample(
        acceleration_distance_ + peak_velocity_ * cruise_elapsed,
        peak_velocity_, 0.0, ReferencePhase::kCruising, false);
    }
    const double deceleration_elapsed = elapsed_ - acceleration_time_ - cruise_time_;
    const double velocity = peak_velocity_ - max_deceleration_ * deceleration_elapsed;
    const double position = acceleration_distance_ + cruise_distance_ +
      peak_velocity_ * deceleration_elapsed -
      0.5 * max_deceleration_ * deceleration_elapsed * deceleration_elapsed;
    return make_sample(
      position, std::max(0.0, velocity), -max_deceleration_,
      ReferencePhase::kDecelerating, false);
  }

private:
  TrapezoidalProfileParameters parameters_;
  double elapsed_{0.0};
  double max_acceleration_{0.0};
  double max_deceleration_{0.0};
  double peak_velocity_{0.0};
  double acceleration_time_{0.0};
  double cruise_time_{0.0};
  double deceleration_time_{0.0};
  double acceleration_distance_{0.0};
  double cruise_distance_{0.0};
  double deceleration_distance_{0.0};
  double total_time_{0.0};
};

struct TransitionShape
{
  double jerk_time{0.0};
  double constant_acceleration_time{0.0};
  double peak_acceleration{0.0};

  double total_time() const {return 2.0 * jerk_time + constant_acceleration_time;}
};

TransitionShape transition_shape(
  const double velocity_change,
  const double acceleration_limit,
  const double jerk_limit)
{
  TransitionShape shape;
  const double full_acceleration_velocity = acceleration_limit * acceleration_limit / jerk_limit;
  if (velocity_change >= full_acceleration_velocity) {
    shape.jerk_time = acceleration_limit / jerk_limit;
    shape.constant_acceleration_time = velocity_change / acceleration_limit - shape.jerk_time;
    shape.peak_acceleration = acceleration_limit;
  } else if (velocity_change > kEpsilon) {
    shape.jerk_time = std::sqrt(velocity_change / jerk_limit);
    shape.peak_acceleration = jerk_limit * shape.jerk_time;
  }
  return shape;
}

double transition_distance(
  const double velocity,
  const double acceleration_limit,
  const double deceleration_limit,
  const double jerk_limit)
{
  const TransitionShape accelerating = transition_shape(
    velocity, acceleration_limit, jerk_limit);
  const TransitionShape decelerating = transition_shape(
    velocity, deceleration_limit, jerk_limit);
  return 0.5 * velocity * (accelerating.total_time() + decelerating.total_time());
}

class JerkLimitedSCurveGenerator final : public StraightSegmentGenerator
{
public:
  explicit JerkLimitedSCurveGenerator(const JerkLimitedSCurveParameters & parameters)
  : parameters_(parameters) {}

  void reset(
    const Eigen::Vector3d & start_position,
    const double start_yaw,
    const Eigen::Vector3d & target_position,
    const double target_yaw) override
  {
    reset_geometry(start_position, start_yaw, target_position, target_yaw);
    elapsed_ = 0.0;
    phases_.clear();
    const double acceleration_limit = path_limit(
      direction_, parameters_.max_acceleration_xy, parameters_.max_acceleration_z);
    const double deceleration_limit = path_limit(
      direction_, parameters_.max_deceleration_xy, parameters_.max_deceleration_z);
    const double jerk_limit = path_limit(
      direction_, parameters_.max_jerk_xy, parameters_.max_jerk_z);
    double peak_velocity = path_limit(
      direction_, parameters_.max_velocity_xy, parameters_.max_velocity_z);

    if (transition_distance(
        peak_velocity, acceleration_limit, deceleration_limit, jerk_limit) > distance_)
    {
      double low = 0.0;
      double high = peak_velocity;
      // 60 次只在航段切换执行，双精度收敛远超毫米级参考需求。
      for (int iteration = 0; iteration < 60; ++iteration) {
        const double middle = 0.5 * (low + high);
        if (transition_distance(
            middle, acceleration_limit, deceleration_limit, jerk_limit) <= distance_)
        {
          low = middle;
        } else {
          high = middle;
        }
      }
      peak_velocity = low;
    }

    const TransitionShape accelerating = transition_shape(
      peak_velocity, acceleration_limit, jerk_limit);
    const TransitionShape decelerating = transition_shape(
      peak_velocity, deceleration_limit, jerk_limit);
    const double shaped_distance = 0.5 * peak_velocity *
      (accelerating.total_time() + decelerating.total_time());
    const double cruise_time = peak_velocity > kEpsilon ?
      std::max(0.0, distance_ - shaped_distance) / peak_velocity : 0.0;

    add_phase(accelerating.jerk_time, jerk_limit, ReferencePhase::kAccelerating);
    add_phase(
      accelerating.constant_acceleration_time, 0.0, ReferencePhase::kAccelerating);
    add_phase(accelerating.jerk_time, -jerk_limit, ReferencePhase::kAccelerating);
    add_phase(cruise_time, 0.0, ReferencePhase::kCruising);
    add_phase(decelerating.jerk_time, -jerk_limit, ReferencePhase::kDecelerating);
    add_phase(
      decelerating.constant_acceleration_time, 0.0, ReferencePhase::kDecelerating);
    add_phase(decelerating.jerk_time, jerk_limit, ReferencePhase::kDecelerating);
    total_time_ = phases_.empty() ? 0.0 :
      phases_.back().start_time + phases_.back().duration;
  }

  GeneratedReference update(const double dt_seconds) override
  {
    elapsed_ = std::min(total_time_, elapsed_ + std::clamp(dt_seconds, 0.0, 0.05));
    return sample();
  }

  GeneratedReference sample() const override
  {
    if (distance_ <= kEpsilon || elapsed_ >= total_time_ - kEpsilon) {
      return make_sample(distance_, 0.0, 0.0, ReferencePhase::kComplete, true);
    }
    for (const Phase & phase : phases_) {
      if (elapsed_ <= phase.start_time + phase.duration + kEpsilon) {
        const double time = std::clamp(elapsed_ - phase.start_time, 0.0, phase.duration);
        const double acceleration = phase.start_acceleration + phase.jerk * time;
        const double velocity = phase.start_velocity + phase.start_acceleration * time +
          0.5 * phase.jerk * time * time;
        const double position = phase.start_position + phase.start_velocity * time +
          0.5 * phase.start_acceleration * time * time +
          phase.jerk * time * time * time / 6.0;
        return make_sample(position, velocity, acceleration, phase.phase, false);
      }
    }
    return make_sample(distance_, 0.0, 0.0, ReferencePhase::kComplete, true);
  }

private:
  struct Phase
  {
    double start_time{0.0};
    double duration{0.0};
    double jerk{0.0};
    double start_position{0.0};
    double start_velocity{0.0};
    double start_acceleration{0.0};
    ReferencePhase phase{ReferencePhase::kIdle};
  };

  void add_phase(
    const double duration,
    const double jerk,
    const ReferencePhase reference_phase)
  {
    if (duration <= kEpsilon) {
      return;
    }
    Phase phase;
    phase.duration = duration;
    phase.jerk = jerk;
    phase.phase = reference_phase;
    if (!phases_.empty()) {
      const Phase & previous = phases_.back();
      phase.start_time = previous.start_time + previous.duration;
      const double time = previous.duration;
      phase.start_acceleration = previous.start_acceleration + previous.jerk * time;
      phase.start_velocity = previous.start_velocity + previous.start_acceleration * time +
        0.5 * previous.jerk * time * time;
      phase.start_position = previous.start_position + previous.start_velocity * time +
        0.5 * previous.start_acceleration * time * time +
        previous.jerk * time * time * time / 6.0;
    }
    phases_.push_back(phase);
  }

  JerkLimitedSCurveParameters parameters_;
  std::vector<Phase> phases_;
  double elapsed_{0.0};
  double total_time_{0.0};
};

}  // namespace

bool valid_reference_generator_parameters(
  const ReferenceGeneratorParameters & parameters) noexcept
{
  const auto & filter = parameters.second_order;
  const auto & trapezoidal = parameters.trapezoidal;
  const auto & s_curve = parameters.s_curve;
  const std::array<double, 24> values{{
    filter.wn_xy, filter.zeta_xy, filter.wn_z, filter.zeta_z,
    filter.max_velocity_xy, filter.max_velocity_z,
    filter.max_acceleration_xy, filter.max_acceleration_z,
    filter.completion_position_tolerance, filter.completion_velocity_tolerance,
    trapezoidal.max_velocity_xy, trapezoidal.max_velocity_z,
    trapezoidal.max_acceleration_xy, trapezoidal.max_acceleration_z,
    trapezoidal.max_deceleration_xy, trapezoidal.max_deceleration_z,
    s_curve.max_velocity_xy, s_curve.max_velocity_z,
    s_curve.max_acceleration_xy, s_curve.max_acceleration_z,
    s_curve.max_deceleration_xy, s_curve.max_deceleration_z,
    s_curve.max_jerk_xy, s_curve.max_jerk_z,
  }};
  return std::all_of(values.begin(), values.end(), finite_positive);
}

std::unique_ptr<ReferenceGenerator> make_reference_generator(
  const ReferenceGeneratorType type,
  const ReferenceGeneratorParameters & parameters)
{
  if (!valid_reference_generator_parameters(parameters)) {
    throw std::invalid_argument("航点参考生成参数必须为有限正数");
  }
  switch (type) {
    case ReferenceGeneratorType::kStepPosition:
      return std::make_unique<StepPositionGenerator>();
    case ReferenceGeneratorType::kSecondOrderFilter:
      return std::make_unique<SecondOrderFilterGenerator>(parameters.second_order);
    case ReferenceGeneratorType::kTrapezoidalProfile:
      return std::make_unique<TrapezoidalProfileGenerator>(parameters.trapezoidal);
    case ReferenceGeneratorType::kJerkLimitedSCurve:
      return std::make_unique<JerkLimitedSCurveGenerator>(parameters.s_curve);
    default:
      throw std::invalid_argument("未知航点参考生成方法");
  }
}

}  // namespace onboard_control
