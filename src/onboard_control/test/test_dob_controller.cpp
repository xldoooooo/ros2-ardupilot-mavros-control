/**
 * @file test_dob_controller.cpp
 * @brief PD+DOB output, selectable jerk bounds, gain transitions and reset behavior.
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
  VehicleKinematics moving;
  moving.velocity = Eigen::Vector3d(0.4, -0.2, 0.3);
  controller.compute(moving, ControlReference{}, 0.01);
  EXPECT_GT(controller.disturbance().norm(), 0.0);
  EXPECT_TRUE(controller.set_gain_profile(low_bandwidth));
  EXPECT_DOUBLE_EQ(controller.disturbance().norm(), 0.0);
  EXPECT_DOUBLE_EQ(controller.hover_throttle(), 0.42);

  low_bandwidth.wn_xy = -1.0;
  EXPECT_FALSE(controller.set_gain_profile(low_bandwidth));
  EXPECT_DOUBLE_EQ(controller.hover_throttle(), 0.42);
}

TEST(DobController, ContinuousGainSwitchPreservesDisturbanceAndPreviousInputWhileMoving)
{
  DobController controller(ControllerParameters{});
  VehicleKinematics moving;
  moving.velocity = Eigen::Vector3d(0.4, -0.2, 0.3);
  ControlReference reference;
  reference.velocity = moving.velocity;
  controller.compute(moving, reference, 0.01);
  controller.compute(moving, reference, 0.01);
  const Eigen::Vector3d disturbance = controller.disturbance();
  ASSERT_GT(disturbance.norm(), 0.0);

  ControllerParameters next;
  next.wn_xy = next.wn_z = 1.2;
  next.zeta_xy = next.zeta_z = 1.1;
  next.observer_xy = 0.2;
  next.observer_z = 0.9;
  // Repeat activation as waypoint preparation and actual execution do.
  for (int activation = 0; activation < 2; ++activation) {
    ASSERT_TRUE(controller.set_gain_profile(next, &moving.velocity));
    EXPECT_LT((controller.disturbance() - disturbance).norm(), 1e-12);
    const auto output = controller.compute(moving, reference, 0.01, true);
    ASSERT_TRUE(output.valid);
    // With zero nominal input, preserving previous_input keeps the next estimate continuous too.
    EXPECT_LT((output.disturbance - disturbance).norm(), 1e-12);
    EXPECT_LT((output.acceleration + disturbance).norm(), 1e-12);
  }
}

/** Enabled slew bounds apply to feedforward, PD changes and reversing commands. */
TEST(DobController, SharedOutputSlewBoundsStepsAndReversals)
{
  ControllerParameters parameters;
  parameters.max_jerk_xy = 6.0;
  parameters.max_jerk_z = 3.0;
  parameters.observer_xy = parameters.observer_z = 0.0;
  DobController controller(parameters);
  Eigen::Vector3d previous = Eigen::Vector3d::Zero();
  for (int cycle = 0; cycle < 60; ++cycle) {
    ControlReference reference;
    const bool feedforward = cycle < 30;
    const double direction = cycle < 15 || cycle >= 45 ? 1.0 : -1.0;
    if (feedforward) {reference.acceleration = direction * Eigen::Vector3d(1.0, 1.0, 0.5);}
    else {reference.position = direction * Eigen::Vector3d(0.2, 0.2, 0.1);}
    const auto output = controller.compute(VehicleKinematics{}, reference, 0.01, feedforward, true);
    ASSERT_TRUE(output.valid);
    EXPECT_LE((output.acceleration - previous).head<2>().norm(), 0.06 + 1e-12);
    EXPECT_LE(std::abs(output.acceleration.z() - previous.z()), 0.03 + 1e-12);
    previous = output.acceleration;
  }
}

/** Gain-only activation preserves the ramp; ending control resets it to zero. */
TEST(DobController, SlewHistorySurvivesGainChangesButNotExplicitReset)
{
  ControllerParameters parameters;
  parameters.max_jerk_xy = 6.0;
  parameters.observer_xy = parameters.observer_z = 0.0;
  DobController controller(parameters);
  ControlReference reference;
  reference.acceleration.x() = 1.0;
  ControlOutput previous;
  for (int cycle = 0; cycle < 20; ++cycle) {
    previous = controller.compute(VehicleKinematics{}, reference, 0.01, true, true);
  }
  ASSERT_NEAR(previous.acceleration.x(), 1.0, 1e-12);
  parameters.wn_xy = 1.2;
  ASSERT_TRUE(controller.set_gain_profile(parameters));
  reference.acceleration.x() = -1.0;
  const auto switched = controller.compute(VehicleKinematics{}, reference, 0.01, true, true);
  EXPECT_NEAR(switched.acceleration.x(), 0.94, 1e-12);
  controller.reset();
  const auto restarted = controller.compute(VehicleKinematics{}, reference, 0.01, true, true);
  EXPECT_NEAR(restarted.acceleration.x(), -0.06, 1e-12);
}

/** The hard tilt bound supplies both reported output and the next slew/observer input. */
TEST(DobController, SlewHistoryUsesTiltLimitedIssuedAcceleration)
{
  ControllerParameters parameters;
  parameters.max_jerk_xy = 6.0;
  parameters.max_tilt_rad = 0.001;
  DobController controller(parameters);
  ControlReference reference;
  reference.acceleration.x() = 1.0;
  const auto first = controller.compute(VehicleKinematics{}, reference, 0.01, true, true);
  ASSERT_TRUE(first.valid);
  const double issued = parameters.gravity * std::tan(parameters.max_tilt_rad);
  EXPECT_NEAR(first.acceleration.x(), issued, 1e-12);
  const auto body_z = first.attitude * Eigen::Vector3d::UnitZ();
  EXPECT_NEAR(parameters.gravity * body_z.x() / body_z.z(), issued, 1e-12);
  reference.acceleration.x() = -1.0;
  VehicleKinematics state;
  state.velocity.x() = issued * 0.001;  // Ideal plant applies the issued input for this cycle.
  const auto second = controller.compute(state, reference, 0.001, true, true);
  EXPECT_NEAR(second.acceleration.x(), issued - 0.006, 1e-12);
  EXPECT_LT(second.disturbance.norm(), 1e-12);
}

/** Bypassing slew responds immediately; re-enabling starts from the last issued command. */
TEST(DobController, OutputSlewSelectionPreservesHistoryAcrossBypass)
{
  ControllerParameters parameters;
  parameters.max_jerk_xy = 10.0;
  parameters.max_jerk_z = 8.0;
  parameters.observer_xy = parameters.observer_z = 0.0;
  DobController controller(parameters);
  ControlReference reference;
  reference.acceleration = Eigen::Vector3d(1.0, 0.0, 0.5);
  const auto bypassed = controller.compute(VehicleKinematics{}, reference, 0.01, true, false);
  ASSERT_TRUE(bypassed.valid);
  EXPECT_LT((bypassed.acceleration - reference.acceleration).norm(), 1e-12);

  reference.acceleration = -reference.acceleration;
  const auto enabled = controller.compute(VehicleKinematics{}, reference, 0.01, true, true);
  ASSERT_TRUE(enabled.valid);
  EXPECT_NEAR(enabled.acceleration.x(), 0.9, 1e-12);
  EXPECT_NEAR(enabled.acceleration.z(), 0.42, 1e-12);
  const auto released = controller.compute(VehicleKinematics{}, reference, 0.01, true, false);
  EXPECT_LT((released.acceleration - reference.acceleration).norm(), 1e-12);
}

/** A bypassed tilt-limited command remains the physical input for both DOB and the next ramp. */
TEST(DobController, BypassedOutputKeepsTiltLimitedObserverInput)
{
  ControllerParameters parameters;
  parameters.max_jerk_xy = 10.0;
  parameters.max_tilt_rad = 0.001;
  DobController controller(parameters);
  ControlReference reference;
  reference.acceleration.x() = 1.0;
  const auto first = controller.compute(VehicleKinematics{}, reference, 0.01, true, false);
  ASSERT_TRUE(first.valid);
  const double issued = parameters.gravity * std::tan(parameters.max_tilt_rad);
  EXPECT_NEAR(first.acceleration.x(), issued, 1e-12);

  VehicleKinematics state;
  state.velocity.x() = issued * 0.001;
  reference.acceleration.x() = -1.0;
  const auto next = controller.compute(state, reference, 0.001, true, true);
  EXPECT_NEAR(next.acceleration.x(), issued - 0.01, 1e-12);
  EXPECT_LT(next.disturbance.norm(), 1e-12);
}

}  // namespace onboard_control
