/** @file test_navigation_frame.cpp
 * @brief Map/local geometry and continuous small relocalization corrections.
 */
#include <gtest/gtest.h>
#include <cmath>
#include "onboard_control/navigation_frame.hpp"

namespace onboard_control
{

/** Keep a remote map origin, nonzero world height, and the local b offset distinct. */
TEST(NavigationFrame, RemoteMapWaypointUsesFullTransformAndFixedLocalOrigin)
{
  const double half_pi = std::acos(-1.0) / 2.0;
  const Eigen::Vector3d local_origin(0.06, -0.03, 0.05);
  NavigationTransform odom_from_map{
    Eigen::Quaterniond(Eigen::AngleAxisd(half_pi, Eigen::Vector3d::UnitZ())),
    Eigen::Vector3d(120.0, -47.0, 6.0)};
  ASSERT_TRUE(odom_from_map.normalize());
  NavigationTransform local_from_map = odom_from_map;
  local_from_map.translation += local_origin;
  const Eigen::Vector3d map_waypoint(4.0, -3.0, 2.0);
  const auto local_waypoint = local_from_map.point(map_waypoint);
  EXPECT_NEAR((local_waypoint - Eigen::Vector3d(123.06, -43.03, 8.05)).norm(),
    0.0, 1.0e-12);
  EXPECT_NEAR((local_from_map.inverse().point(local_waypoint) - map_waypoint).norm(),
    0.0, 1.0e-12);
  EXPECT_NEAR(local_from_map.yaw(0.25), half_pi + 0.25, 1.0e-12);
  EXPECT_NEAR(local_from_map.inverse().yaw(half_pi + 0.25), 0.25, 1.0e-12);

  // A velocity has no map-origin or FCU-origin translation component.
  const Eigen::Vector3d map_velocity(0.2, -0.7, 0.1);
  EXPECT_NEAR((local_from_map.rotation * map_velocity -
    Eigen::Vector3d(0.7, 0.2, 0.1)).norm(), 0.0, 1.0e-12);
  EXPECT_NEAR((local_from_map.inverse().rotation *
    (local_from_map.rotation * map_velocity) - map_velocity).norm(), 0.0, 1.0e-12);
}

/** Updating an unfinished correction preserves all three translation derivatives. */
TEST(NavigationFrame, RetargetKeepsCurrentOffsetPvaAndPersistsAtTheNewEndpoint)
{
  GeneratedReference base;
  base.control.position = {5.0, -2.0, 1.0};
  base.control.velocity = {1.0, 0.2, -0.1};
  base.control.acceleration = {0.1, -0.05, 0.02};
  base.control.yaw = 3.1;
  base.yaw_rate = 0.2;
  base.finished = true;
  NavigationReferenceJoin correction;
  correction.retarget({0.03, -0.02, 0.01}, 0.04, 1.0);
  const auto before = correction.update(base, 0.37);
  EXPECT_FALSE(before.finished);
  const Eigen::Vector3d new_offset(-0.02, 0.01, 0.0);
  correction.retarget(new_offset, -0.03, 0.8);
  const auto after = correction.update(base, 0.0);
  EXPECT_NEAR((after.control.position - before.control.position).norm(), 0.0, 1.0e-12);
  EXPECT_NEAR((after.control.velocity - before.control.velocity).norm(), 0.0, 1.0e-12);
  EXPECT_NEAR((after.control.acceleration - before.control.acceleration).norm(), 0.0, 1.0e-12);
  EXPECT_NEAR(after.control.yaw, before.control.yaw, 1.0e-12);
  EXPECT_NEAR(after.yaw_rate, before.yaw_rate, 1.0e-12);

  // The new goal difference is absolute, not added to the preceding correction.
  for (double dt : {0.8, 1.0}) {
    const auto endpoint = correction.update(base, dt);
    EXPECT_TRUE(endpoint.finished);
    EXPECT_NEAR((endpoint.control.position - base.control.position - new_offset).norm(),
      0.0, 1.0e-12);
    EXPECT_NEAR((endpoint.control.velocity - base.control.velocity).norm(), 0.0, 1.0e-12);
    EXPECT_NEAR((endpoint.control.acceleration - base.control.acceleration).norm(), 0.0, 1.0e-12);
    EXPECT_NEAR(endpoint.control.yaw, base.control.yaw - 0.03, 1.0e-12);
    EXPECT_NEAR(endpoint.yaw_rate, base.yaw_rate, 1.0e-12);
  }
  correction.clear();
  const auto cleared = correction.update(base, 0.01);
  EXPECT_NEAR((cleared.control.position - base.control.position).norm(), 0.0, 1.0e-12);
  EXPECT_NEAR(cleared.control.yaw, base.control.yaw, 1.0e-12);
}

/** A normal three-centimetre correction cannot restart or reverse a cruising segment. */
TEST(NavigationFrame, SmallForwardAndBackwardCorrectionsKeepOneMetrePerSecondCruise)
{
  for (double displacement : {-0.03, 0.03}) {
    NavigationReferenceJoin correction;
    correction.retarget({displacement, 0.0, 0.0}, 0.0, 1.0);
    for (int index = 0; index <= 150; ++index) {
      const double time = index * 0.01;
      GeneratedReference cruise;
      cruise.control.position = {time, 0.0, 1.0};
      cruise.control.velocity = {1.0, 0.0, 0.0};
      cruise.control.acceleration.setZero();
      const auto actual = correction.update(cruise, index == 0 ? 0.0 : 0.01);
      EXPECT_GE(actual.control.velocity.x(), 0.94);
      EXPECT_LE(actual.control.velocity.x(), 1.06);
      EXPECT_LE(std::abs(actual.control.acceleration.x()), 0.18);
      if (time >= 1.0) {
        EXPECT_NEAR(actual.control.position.x(), time + displacement, 1.0e-12);
        EXPECT_NEAR(actual.control.velocity.x(), 1.0, 1.0e-12);
        EXPECT_NEAR(actual.control.acceleration.x(), 0.0, 1.0e-12);
      }
    }
  }
}

/** A map heading correction cannot add excess yaw rate or skip the final catch-up. */
TEST(NavigationFrame, JoinedYawRemainsBoundedWhenBaseAlreadyTurnsAtTheLimit)
{
  ReferenceGeneratorParameters parameters;
  auto generator = make_reference_generator(ReferenceGeneratorType::kStepPosition, parameters);
  generator->reset(Eigen::Vector3d::Zero(), 0.0, Eigen::Vector3d::Zero(), 0.5);
  NavigationReferenceJoin correction;
  correction.retarget(Eigen::Vector3d::Zero(), 0.2, 0.2);
  double previous_yaw = 0.0;
  bool caught_up = false;
  bool delayed_completion = false;
  for (int tick = 0; tick < 100; ++tick) {
    const auto joined = correction.update(generator->update(0.01), 0.01);
    const auto limited = limit_yaw_reference(previous_yaw, joined.control.yaw, 0.01, parameters.max_yaw_rate);
    EXPECT_LE(std::abs(limited.yaw - previous_yaw), parameters.max_yaw_rate * 0.01 + 1.0e-12);
    if (joined.finished && !limited.finished) {delayed_completion = true;}
    caught_up = joined.finished && limited.finished;
    previous_yaw = limited.yaw;
    if (caught_up) {break;}
  }
  EXPECT_TRUE(delayed_completion);
  EXPECT_TRUE(caught_up);
  EXPECT_NEAR(previous_yaw, 0.7, 1.0e-12);
}

}  // namespace onboard_control
