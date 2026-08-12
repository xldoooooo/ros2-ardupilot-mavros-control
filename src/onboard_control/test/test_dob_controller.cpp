/**
 * @file test_dob_controller.cpp
 * @brief 纯 C++ PD+DOB 输出、限幅和复位行为单元测试。
 */
#include <gtest/gtest.h>

#include <cmath>

#include "onboard_control/dob_controller.hpp"

namespace onboard_control
{

TEST(DobController, HoverProducesLevelFiniteOutput)
{
  DobController controller(ControllerParameters{});
  const auto output = controller.compute(VehicleKinematics{}, ControlReference{}, 0.01);

  ASSERT_TRUE(output.valid);
  EXPECT_NEAR(output.attitude.x(), 0.0, 1.0e-9);
  EXPECT_NEAR(output.attitude.y(), 0.0, 1.0e-9);
  EXPECT_NEAR(output.attitude.z(), 0.0, 1.0e-9);
  EXPECT_NEAR(output.attitude.w(), 1.0, 1.0e-9);
  EXPECT_NEAR(output.thrust, 0.39, 1.0e-9);
}

TEST(DobController, HoverThrottleCanOnlyBeUpdatedWithFiniteNormalizedValue)
{
  DobController controller(ControllerParameters{});
  EXPECT_TRUE(controller.set_hover_throttle(0.42));
  EXPECT_DOUBLE_EQ(controller.hover_throttle(), 0.42);
  EXPECT_FALSE(controller.set_hover_throttle(-0.1));
  EXPECT_FALSE(controller.set_hover_throttle(std::nan("")));
  EXPECT_DOUBLE_EQ(controller.hover_throttle(), 0.42);
}

TEST(DobController, LargeErrorRespectsTiltAndThrustLimits)
{
  ControllerParameters parameters;
  parameters.max_tilt_rad = 20.0 * M_PI / 180.0;
  DobController controller(parameters);
  VehicleKinematics state;
  ControlReference reference;
  reference.position = Eigen::Vector3d(100.0, -100.0, 100.0);

  const auto output = controller.compute(state, reference, 0.01);
  ASSERT_TRUE(output.valid);
  EXPECT_GE(output.thrust, 0.0);
  EXPECT_LE(output.thrust, 1.0);
  const Eigen::Vector3d body_z = output.attitude * Eigen::Vector3d::UnitZ();
  const double tilt = std::acos(std::clamp(body_z.z(), -1.0, 1.0));
  EXPECT_LE(tilt, parameters.max_tilt_rad + 1.0e-9);
}

TEST(DobController, ResetClearsObserverState)
{
  DobController controller(ControllerParameters{});
  VehicleKinematics moving;
  moving.velocity = Eigen::Vector3d(1.0, -1.0, 0.5);
  controller.compute(moving, ControlReference{}, 0.01);
  EXPECT_GT(controller.disturbance().norm(), 0.0);

  controller.reset();
  EXPECT_DOUBLE_EQ(controller.disturbance().norm(), 0.0);
}

TEST(DobController, TrajectoryModeAddsAccelerationFeedforwardOnlyWhenRequested)
{
  DobController controller(ControllerParameters{});
  ControlReference reference;
  reference.acceleration = Eigen::Vector3d(0.5, 0.0, 0.0);

  const auto baseline = controller.compute(VehicleKinematics{}, reference, 0.01, false);
  controller.reset();
  const auto trajectory = controller.compute(VehicleKinematics{}, reference, 0.01, true);

  ASSERT_TRUE(baseline.valid);
  ASSERT_TRUE(trajectory.valid);
  EXPECT_NEAR(baseline.acceleration.x(), 0.0, 1.0e-12);
  EXPECT_NEAR(trajectory.acceleration.x(), 0.5, 1.0e-12);
}

TEST(DobController, GainProfileSwitchPreservesHoverThrottleAndRejectsInvalidValues)
{
  DobController controller(ControllerParameters{});
  ASSERT_TRUE(controller.set_hover_throttle(0.42));
  ControllerParameters low_bandwidth;
  low_bandwidth.wn_xy = 1.2;
  low_bandwidth.zeta_xy = 1.1;
  low_bandwidth.observer_xy = 0.5;
  EXPECT_TRUE(controller.set_gain_profile(low_bandwidth));
  EXPECT_DOUBLE_EQ(controller.hover_throttle(), 0.42);

  low_bandwidth.wn_xy = -1.0;
  EXPECT_FALSE(controller.set_gain_profile(low_bandwidth));
  EXPECT_DOUBLE_EQ(controller.hover_throttle(), 0.42);
}

}  // namespace onboard_control
