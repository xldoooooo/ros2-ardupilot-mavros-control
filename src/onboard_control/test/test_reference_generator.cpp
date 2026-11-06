/**
 * @file test_reference_generator.cpp
 * @brief 平移限额、统一偏航限速与原地/短航段参考完成判定。
 */
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include "onboard_control/reference_generator.hpp"

namespace onboard_control
{
namespace
{

/** 以实际 100 Hz 周期运行一段参考，直到生成器报告完成。 */
std::vector<GeneratedReference> run_segment(
  ReferenceGenerator & generator,
  const Eigen::Vector3d & target,
  const double maximum_seconds = 120.0)
{
  generator.reset(Eigen::Vector3d::Zero(), 0.0, target, 0.0);
  std::vector<GeneratedReference> samples;
  for (int index = 0; index < static_cast<int>(maximum_seconds * 100.0); ++index) {
    samples.push_back(generator.update(0.01));
    if (samples.back().finished) {
      break;
    }
  }
  return samples;
}

}  // namespace

TEST(ReferenceGenerator, StepPositionPreservesPositionStepWhileYawSlews)
{
  ReferenceGeneratorParameters parameters;
  auto generator = make_reference_generator(
    ReferenceGeneratorType::kStepPosition, parameters);
  generator->reset(
    Eigen::Vector3d(1.0, 2.0, 3.0), 0.2,
    Eigen::Vector3d(8.0, -1.0, 4.0), -0.3);

  const GeneratedReference output = generator->sample();
  EXPECT_FALSE(output.finished);
  EXPECT_EQ(output.phase, ReferencePhase::kStep);
  EXPECT_TRUE(output.control.position.isApprox(Eigen::Vector3d(8.0, -1.0, 4.0)));
  EXPECT_DOUBLE_EQ(output.control.velocity.norm(), 0.0);
  EXPECT_DOUBLE_EQ(output.control.acceleration.norm(), 0.0);
  EXPECT_DOUBLE_EQ(output.control.yaw, 0.2);
}

TEST(ReferenceGenerator, AllMethodsBoundYawForStationaryAndShortSegments)
{
  ReferenceGeneratorParameters parameters;
  parameters.max_yaw_rate = 0.7;
  const double pi = std::acos(-1.0);
  for (int method = 0; method < 4; ++method) {
    for (double distance : {0.0, 0.01, 1.0}) {
      SCOPED_TRACE(::testing::Message() << "method=" << method << ", distance=" << distance);
      auto generator = make_reference_generator(static_cast<ReferenceGeneratorType>(method), parameters);
      const Eigen::Vector3d target(distance, 0.0, 0.0);
      generator->reset(Eigen::Vector3d::Zero(), 0.0, target, pi);
      auto sample = generator->sample();
      EXPECT_DOUBLE_EQ(sample.control.yaw, 0.0);
      EXPECT_FALSE(sample.finished);
      double previous_yaw = sample.control.yaw;
      double elapsed = 0.0;
      for (int tick = 0; tick < 12000 && !sample.finished; ++tick) {
        sample = generator->update(0.01);
        elapsed += 0.01;
        const double delta = std::remainder(sample.control.yaw - previous_yaw, 2.0 * pi);
        EXPECT_LE(std::abs(delta), parameters.max_yaw_rate * 0.01 + 1.0e-12);
        EXPECT_NEAR(sample.yaw_rate, delta / 0.01, 1.0e-10);
        if (elapsed < pi / parameters.max_yaw_rate - 1.0e-9) {
          EXPECT_FALSE(sample.finished);
        }
        previous_yaw = sample.control.yaw;
      }
      EXPECT_TRUE(sample.finished);
      EXPECT_TRUE(sample.control.position.isApprox(target));
      EXPECT_NEAR(std::remainder(sample.control.yaw - pi, 2.0 * pi), 0.0, 1.0e-9);
    }
  }
}

TEST(ReferenceGenerator, YawUsesShortestPathAndContinuesFromLastOutput)
{
  ReferenceGeneratorParameters parameters;
  const double pi = std::acos(-1.0);
  for (int method = 0; method < 4; ++method) {
    auto generator = make_reference_generator(static_cast<ReferenceGeneratorType>(method), parameters);
    const double start = 179.0 * pi / 180.0;
    generator->reset(Eigen::Vector3d::Zero(), start, Eigen::Vector3d::Zero(), -start);
    EXPECT_DOUBLE_EQ(generator->update(0.0).control.yaw, start);
    const auto first = generator->update(0.01);
    EXPECT_NEAR(first.yaw_rate, parameters.max_yaw_rate, 1.0e-12);
    generator->reset(Eigen::Vector3d::Zero(), first.control.yaw, Eigen::Vector3d::Zero(), 0.0);
    EXPECT_DOUBLE_EQ(generator->sample().control.yaw, first.control.yaw);
    EXPECT_NEAR(generator->update(0.01).yaw_rate, -parameters.max_yaw_rate, 1.0e-12);
  }
}

TEST(ReferenceGenerator, CommonYawLimiterBoundsJoinedHeadingAndValidatesRate)
{
  const auto waiting = limit_yaw_reference(0.0, 0.2, 0.01, 1.0);
  EXPECT_NEAR(waiting.yaw, 0.01, 1.0e-12);
  EXPECT_DOUBLE_EQ(waiting.yaw_rate, 1.0);
  EXPECT_FALSE(waiting.finished);
  const auto stationary = limit_yaw_reference(waiting.yaw, 0.2, 0.0, 1.0);
  EXPECT_DOUBLE_EQ(stationary.yaw, waiting.yaw);
  EXPECT_DOUBLE_EQ(stationary.yaw_rate, 0.0);
  EXPECT_FALSE(stationary.finished);
  const auto arrived = limit_yaw_reference(0.195, 0.2, 0.01, 1.0);
  EXPECT_NEAR(arrived.yaw, 0.2, 1.0e-12);
  EXPECT_TRUE(arrived.finished);
  ReferenceGeneratorParameters parameters;
  for (double rate : {0.0, -1.0, std::numeric_limits<double>::infinity()}) {
    parameters.max_yaw_rate = rate;
    EXPECT_FALSE(valid_reference_generator_parameters(parameters));
  }
}

TEST(ReferenceGenerator, TrapezoidalProfileHasBoundedLongCruise)
{
  ReferenceGeneratorParameters parameters;
  auto generator = make_reference_generator(
    ReferenceGeneratorType::kTrapezoidalProfile, parameters);
  const auto samples = run_segment(*generator, Eigen::Vector3d(5.0, 0.0, 0.0));

  ASSERT_FALSE(samples.empty());
  ASSERT_TRUE(samples.back().finished);
  EXPECT_TRUE(samples.back().control.position.isApprox(Eigen::Vector3d(5.0, 0.0, 0.0)));
  double maximum_velocity = 0.0;
  int cruise_samples = 0;
  for (const auto & sample : samples) {
    maximum_velocity = std::max(maximum_velocity, sample.control.velocity.norm());
    cruise_samples += sample.phase == ReferencePhase::kCruising ? 1 : 0;
  }
  EXPECT_LE(maximum_velocity, parameters.trapezoidal.max_velocity_xy + 1.0e-9);
  EXPECT_GT(cruise_samples, 500);
}

TEST(ReferenceGenerator, JerkLimitedSCurveBoundsVelocityAccelerationAndJerk)
{
  ReferenceGeneratorParameters parameters;
  auto generator = make_reference_generator(
    ReferenceGeneratorType::kJerkLimitedSCurve, parameters);
  const auto samples = run_segment(*generator, Eigen::Vector3d(5.0, 0.0, 0.0));

  ASSERT_GT(samples.size(), 10U);
  ASSERT_TRUE(samples.back().finished);
  int cruise_samples = 0;
  double maximum_velocity = 0.0;
  double maximum_acceleration = 0.0;
  double maximum_jerk = 0.0;
  for (std::size_t index = 0; index < samples.size(); ++index) {
    const auto & sample = samples[index];
    maximum_velocity = std::max(maximum_velocity, sample.control.velocity.norm());
    maximum_acceleration = std::max(
      maximum_acceleration, sample.control.acceleration.norm());
    cruise_samples += sample.phase == ReferencePhase::kCruising ? 1 : 0;
    if (index > 0 && !sample.finished) {
      maximum_jerk = std::max(
        maximum_jerk,
        (sample.control.acceleration - samples[index - 1].control.acceleration).norm() /
        0.01);
    }
  }
  EXPECT_LE(maximum_velocity, parameters.s_curve.max_velocity_xy + 1.0e-8);
  EXPECT_LE(
    maximum_acceleration,
    std::max(
      parameters.s_curve.max_acceleration_xy,
      parameters.s_curve.max_deceleration_xy) + 1.0e-8);
  EXPECT_LE(maximum_jerk, parameters.s_curve.max_jerk_xy + 1.0e-6);
  EXPECT_GT(cruise_samples, 500);
}

TEST(ReferenceGenerator, JerkLimitedSCurveShortSegmentLowersPeakVelocity)
{
  ReferenceGeneratorParameters parameters;
  auto generator = make_reference_generator(
    ReferenceGeneratorType::kJerkLimitedSCurve, parameters);
  const auto samples = run_segment(*generator, Eigen::Vector3d(0.05, 0.0, 0.0));

  ASSERT_FALSE(samples.empty());
  ASSERT_TRUE(samples.back().finished);
  double maximum_velocity = 0.0;
  for (const auto & sample : samples) {
    maximum_velocity = std::max(maximum_velocity, sample.control.velocity.norm());
  }
  EXPECT_LT(maximum_velocity, parameters.s_curve.max_velocity_xy);
  EXPECT_TRUE(samples.back().control.position.isApprox(Eigen::Vector3d(0.05, 0.0, 0.0)));
}

TEST(ReferenceGenerator, MixedXyzSegmentsRespectIndependentHorizontalAndVerticalLimits)
{
  ReferenceGeneratorParameters parameters;
  // 与 run_segment 的 100 Hz 更新保持一致，用于计算相邻样本的 jerk。
  constexpr double sample_seconds = 0.01;
  for (int method = 1; method < 4; ++method) {
    for (double height : {-1.5, 1.5}) {
      SCOPED_TRACE(::testing::Message() << "method=" << method << ", height=" << height);
      const Eigen::Vector3d target(1.2, -1.6, height);
      auto generator = make_reference_generator(
        static_cast<ReferenceGeneratorType>(method), parameters);
      const auto samples = run_segment(*generator, target);
      ASSERT_FALSE(samples.empty());
      ASSERT_TRUE(samples.back().finished);
      EXPECT_TRUE(samples.back().control.position.isApprox(target));
      const double velocity_xy = method == 1 ? parameters.second_order.max_velocity_xy :
        method == 2 ? parameters.trapezoidal.max_velocity_xy : parameters.s_curve.max_velocity_xy;
      const double velocity_z = method == 1 ? parameters.second_order.max_velocity_z :
        method == 2 ? parameters.trapezoidal.max_velocity_z : parameters.s_curve.max_velocity_z;
      const double acceleration_xy = method == 1 ? parameters.second_order.max_acceleration_xy :
        method == 2 ? std::max(parameters.trapezoidal.max_acceleration_xy,
          parameters.trapezoidal.max_deceleration_xy) :
        std::max(parameters.s_curve.max_acceleration_xy, parameters.s_curve.max_deceleration_xy);
      const double acceleration_z = method == 1 ? parameters.second_order.max_acceleration_z :
        method == 2 ? std::max(parameters.trapezoidal.max_acceleration_z,
          parameters.trapezoidal.max_deceleration_z) :
        std::max(parameters.s_curve.max_acceleration_z, parameters.s_curve.max_deceleration_z);
      for (std::size_t index = 0; index < samples.size(); ++index) {
        const auto & control = samples[index].control;
        EXPECT_LE(control.velocity.head<2>().norm(), velocity_xy + 1.0e-8);
        EXPECT_LE(std::abs(control.velocity.z()), velocity_z + 1.0e-8);
        EXPECT_LE(control.acceleration.head<2>().norm(), acceleration_xy + 1.0e-8);
        EXPECT_LE(std::abs(control.acceleration.z()), acceleration_z + 1.0e-8);
        EXPECT_NEAR(control.position.cross(target).norm(), 0.0, 1.0e-8);
        if (method == 3 && index > 0 && !samples[index].finished) {
          const auto jerk = (control.acceleration - samples[index - 1].control.acceleration) /
            sample_seconds;
          EXPECT_LE(jerk.head<2>().norm(), parameters.s_curve.max_jerk_xy + 1.0e-6);
          EXPECT_LE(std::abs(jerk.z()), parameters.s_curve.max_jerk_z + 1.0e-6);
        }
      }
    }
  }
}

TEST(ReferenceGenerator, SecondOrderFilterConvergesWithinIndependentLimits)
{
  ReferenceGeneratorParameters parameters;
  auto generator = make_reference_generator(
    ReferenceGeneratorType::kSecondOrderFilter, parameters);
  const auto samples = run_segment(*generator, Eigen::Vector3d(3.0, 0.0, 0.0));

  ASSERT_FALSE(samples.empty());
  ASSERT_TRUE(samples.back().finished);
  for (const auto & sample : samples) {
    EXPECT_LE(
      sample.control.velocity.norm(), parameters.second_order.max_velocity_xy + 1.0e-9);
    EXPECT_LE(
      sample.control.acceleration.norm(),
      parameters.second_order.max_acceleration_xy + 1.0e-9);
  }
  EXPECT_TRUE(samples.back().control.position.isApprox(Eigen::Vector3d(3.0, 0.0, 0.0)));
}

}  // namespace onboard_control
