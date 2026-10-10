/** @file test_avoidance_trajectory.cpp @brief 校验连续轨迹、最近点裁剪、包线和接入拒绝。 */
#include <gtest/gtest.h>
#include <limits>
#include "onboard_control/avoidance_trajectory.hpp"

using namespace onboard_control;

namespace
{
// x=3s²-2s³ over six seconds reaches one metre with stationary ends.
AvoidancePolynomial segment()
{
  AvoidancePolynomial s;
  s.duration = 6.0;
  s.coefficients(0, 2) = 3.0 / 36.0;
  s.coefficients(0, 3) = -2.0 / 216.0;
  s.coefficients(2, 0) = 1.0;
  return s;
}
}

TEST(AvoidanceTrajectory, ExactTimedReferenceAndTerminalHold)
{
  AvoidanceTrajectory t;
  std::string reason;
  ASSERT_TRUE(t.assign({segment()}, {1, 0, 1}, {}, reason)) << reason;
  const auto middle = t.sample(3.0);
  EXPECT_NEAR(middle.position.x(), 0.5, 1e-12);
  EXPECT_NEAR(middle.velocity.x(), 0.25, 1e-12);
  EXPECT_NEAR(middle.acceleration.x(), 0.0, 1e-12);
  const auto end = t.sample(20);
  EXPECT_NEAR(end.position.x(), 1.0, 1e-12);
  EXPECT_EQ(end.velocity.norm(), 0);
  EXPECT_EQ(end.acceleration.norm(), 0);
}

TEST(AvoidanceTrajectory, RejectsMalformedPartialAndDiscontinuousCurves)
{
  AvoidanceTrajectory t;
  std::string reason;
  auto s = segment();
  EXPECT_FALSE(t.assign({}, {1, 0, 1}, {}, reason));
  EXPECT_FALSE(t.assign({s}, {2, 0, 1}, {}, reason));
  auto second = s;
  second.coefficients(0, 0) = 1.1;
  EXPECT_FALSE(t.assign({s, second}, {2.1, 0, 1}, {}, reason));
  second.coefficients(0, 0) = 1.0;
  ASSERT_TRUE(t.assign({s, second}, {2, 0, 1}, {}, reason)) << reason;
  EXPECT_NEAR(t.sample(9).position.x(), 1.5, 1e-12);
  s.duration = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(t.assign({s}, {1, 0, 1}, {}, reason));
  EXPECT_TRUE(t.empty());
}

TEST(AvoidanceTrajectory, RejectsHorizontalNormAndVerticalEnvelope)
{
  AvoidanceTrajectory t;
  std::string reason;
  auto s = segment();
  AvoidanceLimits limits;
  limits.velocity_xy = 0.3;
  s.coefficients.row(1) = s.coefficients.row(0);
  EXPECT_FALSE(t.assign({s}, {1, 1, 1}, limits, reason));  // Each axis .25, norm .354.
  s = segment();
  s.coefficients(2, 2) = 3.0 / 36;
  s.coefficients(2, 3) = -2.0 / 216;
  EXPECT_FALSE(t.assign({s}, {1, 0, 2}, {}, reason));
}

TEST(AvoidanceTrajectory, InteriorVelocityPeakCannotEscapeCheck)
{
  AvoidanceTrajectory t;
  std::string reason;
  auto s = segment();
  AvoidanceLimits limits;
  limits.velocity_xy = 0.2499;
  EXPECT_FALSE(t.assign({s}, {1, 0, 1}, limits, reason));
  limits.velocity_xy = 0.25;
  EXPECT_TRUE(t.assign({s}, {1, 0, 1}, limits, reason)) << reason;
}

TEST(AvoidanceTrajectory, JoinRequiresActualPositionAndVelocity)
{
  AvoidanceTrajectory t;
  std::string reason;
  ASSERT_TRUE(t.assign({segment()}, {1, 0, 1}, {}, reason));
  VehicleKinematics state;
  state.position = {0, 0, 1};
  state.velocity.setZero();
  EXPECT_TRUE(t.can_join(state, .08, .08));
  state.position.x() = .09;
  EXPECT_FALSE(t.can_join(state, .08, .08));
  state.position.x() = 0;
  state.velocity.x() = .09;
  EXPECT_FALSE(t.can_join(state, .08, .08));
}

TEST(AvoidanceTrajectory, DelayedStartKeepsExactRemainingPositionVelocityAcceleration)
{
  AvoidanceTrajectory t;
  std::string reason;
  auto second = segment();
  second.coefficients(0, 0) = 1.0;
  ASSERT_TRUE(t.assign({segment(), second}, {2, 0, 1}, {}, reason)) << reason;
  const auto old = t;
  VehicleKinematics actual;
  actual.position = old.sample(8.1).position;
  actual.position.y() += 0.03;  // 最近点按三维距离投影，允许小横向接入误差。
  actual.velocity = old.sample(8.1).velocity;
  EXPECT_FALSE(t.can_join(actual, .08, .08));
  EXPECT_NEAR(t.start_at_closest(actual.position), 8.1, 1e-10);
  ASSERT_TRUE(t.can_join(actual, .08, .08));
  ASSERT_EQ(t.segments().size(), 1u);
  EXPECT_NEAR(t.duration(), 3.9, 1e-10);
  // 避开末点加速度置零的浮点边界，分别检查内部和明确越过终点的保持。
  for (double dt : {0.0, 0.5, 2.0, 3.8, 5.0}) {
    const auto original = old.sample(8.1 + dt), trimmed = t.sample(dt);
    EXPECT_NEAR((trimmed.position - original.position).norm(), 0.0, 1e-10);
    EXPECT_NEAR((trimmed.velocity - original.velocity).norm(), 0.0, 1e-10);
    EXPECT_NEAR((trimmed.acceleration - original.acceleration).norm(), 0.0, 1e-10);
  }
  EXPECT_GE(t.sample(.01).position.x(), actual.position.x());
}

TEST(AvoidanceTrajectory, GlobalNearestIncludesCurvedInteriorVerticalAndEndpoints)
{
  AvoidanceTrajectory t;
  std::string reason;
  auto s = segment();
  // 在轨迹 xy 正交平面中增加弯曲；最近时刻不能只投影到端点连线。
  s.coefficients(1, 2) = .02;
  s.coefficients(1, 3) = -.02 / 9;
  s.coefficients(2, 2) = .01;
  s.coefficients(2, 3) = -.01 / 9;
  const auto goal = s.sample(s.duration).position;
  ASSERT_TRUE(t.assign({s}, goal, {}, reason)) << reason;
  const Eigen::Vector3d actual{.47, .32, 1.17};
  const double nearest = t.closest_time(actual);
  ASSERT_GT(nearest, 0.0); ASSERT_LT(nearest, t.duration());
  const auto ref = t.sample(nearest);
  EXPECT_NEAR((ref.position - actual).dot(ref.velocity), 0.0, 1e-12);
  for (double time = 0.0; time <= t.duration(); time += .001) {
    EXPECT_LE((ref.position - actual).squaredNorm(),
      (t.sample(time).position - actual).squaredNorm() + 1e-12);
  }
  ASSERT_TRUE(t.assign({segment()}, {1, 0, 1}, {}, reason));
  EXPECT_DOUBLE_EQ(t.closest_time({-2, 0, 1}), 0.0);
  EXPECT_DOUBLE_EQ(t.closest_time({2, 0, 1}), 6.0);
  EXPECT_NEAR(t.start_at_closest({2, 0, 1}), 6.0, 1e-12);
  EXPECT_EQ(t.sample(0).position.x(), 1.0);
  EXPECT_EQ(t.sample(0).velocity.norm(), 0.0);
  EXPECT_EQ(t.sample(0).acceleration.norm(), 0.0);
  AvoidanceTrajectory validated_hold;
  EXPECT_TRUE(validated_hold.assign(t.segments(), {1, 0, 1}, {}, reason)) << reason;
}

TEST(AvoidanceTrajectory, MultipleMinimaAndConstantSegmentsChooseLatestEqualPoint)
{
  AvoidanceTrajectory t;
  std::string reason;
  // 三个不同的零距离最近点；五次距离导数有多个根，单次局部优化不够。
  AvoidancePolynomial loop;
  loop.duration = 1.0;
  loop.coefficients.row(0) << 0.0, 1.0, -3.0, 2.0;
  loop.coefficients(2, 0) = 1.0;
  AvoidanceLimits loose;
  loose.velocity_xy = 10.0; loose.acceleration_xy = 10.0;
  auto stop = segment();  // 零距离后继续到静止终点，确保原始曲线合法。
  stop.coefficients(0, 1) = 1.0;
  stop.coefficients(0, 2) = -1.0 / 3.0;
  stop.coefficients(0, 3) = 1.0 / 36.0;
  ASSERT_TRUE(t.assign({loop, stop}, {0, 0, 1}, loose, reason)) << reason;
  EXPECT_DOUBLE_EQ(t.closest_time({0, 0, 1}), 7.0);
  AvoidancePolynomial constant;
  constant.duration = 2;
  constant.coefficients(2, 0) = 1;
  ASSERT_TRUE(t.assign({constant, segment()}, {1, 0, 1}, {}, reason));
  EXPECT_DOUBLE_EQ(t.start_at_closest({0, 0, 1}), 2.0);
  EXPECT_EQ(t.segments().size(), 1u);
  EXPECT_NEAR(t.duration(), 6.0, 1e-12);
  EXPECT_DOUBLE_EQ(t.closest_time({std::numeric_limits<double>::quiet_NaN(), 0, 1}), 0.0);
}
