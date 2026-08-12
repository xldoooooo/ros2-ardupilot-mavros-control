/**
 * @file test_reference_generator.cpp
 * @brief 航点命令生成器限速、连续性、匀速阶段与短航段回归测试。
 */
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <vector>

#include "onboard_control/reference_generator.hpp"

namespace onboard_control
{
namespace
{

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

TEST(ReferenceGenerator, StepPositionPreservesExistingReferenceSemantics)
{
  ReferenceGeneratorParameters parameters;
  auto generator = make_reference_generator(
    ReferenceGeneratorType::kStepPosition, parameters);
  generator->reset(
    Eigen::Vector3d(1.0, 2.0, 3.0), 0.2,
    Eigen::Vector3d(8.0, -1.0, 4.0), -0.3);

  const GeneratedReference output = generator->sample();
  EXPECT_TRUE(output.finished);
  EXPECT_EQ(output.phase, ReferencePhase::kStep);
  EXPECT_TRUE(output.control.position.isApprox(Eigen::Vector3d(8.0, -1.0, 4.0)));
  EXPECT_DOUBLE_EQ(output.control.velocity.norm(), 0.0);
  EXPECT_DOUBLE_EQ(output.control.acceleration.norm(), 0.0);
  EXPECT_DOUBLE_EQ(output.control.yaw, -0.3);
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
