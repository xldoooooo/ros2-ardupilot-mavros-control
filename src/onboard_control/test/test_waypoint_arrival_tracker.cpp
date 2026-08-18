/**
 * @file test_waypoint_arrival_tracker.cpp
 * @brief 航点入点重试频率、异常阈值和成功清零回归测试。
 */
#include <gtest/gtest.h>

#include <chrono>

#include "onboard_control/waypoint_arrival_tracker.hpp"

namespace onboard_control
{
namespace
{

using Seconds = std::chrono::duration<double>;

WaypointArrivalTracker::TimePoint at(const double seconds)
{
  return WaypointArrivalTracker::TimePoint{} +
         std::chrono::duration_cast<WaypointArrivalTracker::Clock::duration>(Seconds(seconds));
}

}  // namespace

TEST(WaypointArrivalTracker, NormalApproachBeforeCandidateDoesNotCountFailures)
{
  WaypointArrivalTracker tracker(1.0, 10);
  for (int second = 0; second < 30; ++second) {
    EXPECT_EQ(
      tracker.update(at(second), false, false, 0.3),
      WaypointArrivalState::kApproaching);
  }
  EXPECT_EQ(tracker.failure_count(), 0);
}

TEST(WaypointArrivalTracker, FailedEntryIsSampledOncePerSecondAndBecomesAbnormal)
{
  WaypointArrivalTracker tracker(1.0, 10);
  EXPECT_EQ(
    tracker.update(at(0.0), true, false, 0.3),
    WaypointArrivalState::kApproaching);
  EXPECT_EQ(tracker.failure_count(), 1);
  tracker.update(at(0.99), true, false, 0.3);
  EXPECT_EQ(tracker.failure_count(), 1);
  for (int second = 1; second <= 8; ++second) {
    tracker.update(at(second), true, false, 0.3);
  }
  EXPECT_EQ(tracker.failure_count(), 9);
  EXPECT_EQ(
    tracker.update(at(9.0), true, false, 0.3),
    WaypointArrivalState::kAbnormal);
  EXPECT_EQ(tracker.failure_count(), 10);
}

TEST(WaypointArrivalTracker, StableHoldCompletesAndClearsPreviousFailures)
{
  WaypointArrivalTracker tracker(1.0, 10);
  tracker.update(at(0.0), true, false, 0.3);
  tracker.update(at(1.0), false, false, 0.3);
  ASSERT_EQ(tracker.failure_count(), 2);

  EXPECT_EQ(
    tracker.update(at(1.1), true, true, 0.3),
    WaypointArrivalState::kHolding);
  EXPECT_EQ(
    tracker.update(at(1.41), true, true, 0.3),
    WaypointArrivalState::kReached);
  EXPECT_EQ(tracker.failure_count(), 0);
}

}  // namespace onboard_control
