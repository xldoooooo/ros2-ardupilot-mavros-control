/** @file telemetry_diagnostics.hpp
 * Bounded timing probes for the MAVROS receive-to-local-position path.
 */
#pragma once

#include <cstdint>
#include <cstring>

namespace mavros::telemetry_diagnostics
{
// Each boundary has one in-flight slot and one coalesced slow/gap event.
enum class Stage { link_receive, position_receive, router_publish, uas_dispatch,
  uas_position, position_handler, odom_publish, pose_publish, body_velocity_publish,
  velocity_publish, count };

struct Identity
{
  uint32_t message{0}, boot_ms{0};
  uint8_t sequence{0}, system{0}, component{0};
  int framing{0};
  int64_t bridge_stamp_ns{0};
};

/** Copy wire identity; LOCAL_POSITION_NED starts with little-endian time_boot_ms. */
template<class Message>
Identity identify(const Message * msg, int framing = 1, int64_t stamp = 0)
{
  Identity value{msg->msgid, 0, msg->seq, msg->sysid, msg->compid, framing, stamp};
  if (msg->msgid == 32 && msg->len >= 4) {
    std::memcpy(&value.boot_ms, msg->payload64, sizeof(value.boot_ms));
  }
  return value;
}

struct Sample
{
  int64_t steady_ns{0}, wall_ns{0}, cpu_ns{-1};
  long voluntary{-1}, involuntary{-1}, minor{-1}, major{-1};
  int tid{0};
};

/** Prepare the sink before opening the link, outside its receive callback. */
void initialize();

/** Observe a synchronous boundary without changing message processing or exceptions. */
class Scope
{
public:
  Scope(Stage stage, Identity identity, bool enabled = true);
  ~Scope();
  Scope(const Scope &) = delete;
  Scope & operator=(const Scope &) = delete;
private:
  Stage stage_;
  Identity identity_;
  Sample started_;
  bool active_{false};
};
}  // namespace mavros::telemetry_diagnostics
