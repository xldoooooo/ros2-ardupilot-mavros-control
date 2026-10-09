/** @file test_avoidance_trajectory.cpp @brief 独立校验轨迹连续性、时序、包线和接入拒绝。 */
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
