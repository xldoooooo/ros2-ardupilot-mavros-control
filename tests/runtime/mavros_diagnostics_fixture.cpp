/** @file mavros_diagnostics_fixture.cpp
 * Exercise the standalone recorder; the alternate build delays only its writer flush.
 */
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <thread>
#include <unistd.h>

#ifdef SLOW_SINK
#include <atomic>
#include <dlfcn.h>
#include <fcntl.h>

/** Leave startup alone, then expose one slow diagnostic flush to the test process. */
extern "C" int fflush(FILE * stream)
{
  static const auto original = reinterpret_cast<int (*)(FILE *)>(dlsym(RTLD_NEXT, "fflush"));
  static std::atomic<unsigned> calls{0};
  if (stream && ++calls == 2) {
    const int fd = ::open(std::getenv("SLOW_SINK_MARKER"), O_CREAT | O_WRONLY, 0600);
    if (fd >= 0) {::close(fd);}
    std::this_thread::sleep_for(std::chrono::milliseconds(2500));
  }
  return original(stream);
}
#else
#include "mavros/telemetry_diagnostics.hpp"
#include <algorithm>
#include <filesystem>
#include <string>
#include <sys/resource.h>

namespace td = mavros::telemetry_diagnostics;
using Clock = std::chrono::steady_clock;
using namespace std::chrono_literals;

/** Return this process's CPU use, including its diagnostic writer. */
double cpu_ms()
{
  rusage usage{};
  ::getrusage(RUSAGE_SELF, &usage);
  return (usage.ru_utime.tv_sec + usage.ru_stime.tv_sec) * 1000.0 +
         (usage.ru_utime.tv_usec + usage.ru_stime.tv_usec) / 1000.0;
}

/** Observe an empty boundary using a realistic position-message identity. */
void pulse(td::Stage stage, uint32_t boot_ms = 1000, bool enabled = true)
{
  td::Scope observed(stage, td::Identity{32, boot_ms, 7, 1, 1, 1, 123456789}, enabled);
}

/** Run one isolated scenario so recorder lifetime and sink faults cannot leak between tests. */
int main(int argc, char ** argv)
{
  if (argc != 2) {return 2;}
  td::initialize();
  const std::string mode(argv[1]);
  if (mode == "timing") {
    pulse(td::Stage::position_receive, 1000);
    std::this_thread::sleep_for(350ms);
    pulse(td::Stage::position_receive, 1350);
    {
      td::Scope sleeping(td::Stage::pose_publish, {});
      std::this_thread::sleep_for(450ms);
    }
    // The following arrival gap must not overwrite the slow call's CPU evidence.
    pulse(td::Stage::pose_publish);
    {
      td::Scope busy(td::Stage::velocity_publish, {});
      const auto until = Clock::now() + 450ms;
      while (Clock::now() < until) {}
    }
    pulse(td::Stage::velocity_publish);
    std::this_thread::sleep_for(1100ms);
  } else if (mode == "inflight") {
    td::Scope stalled(td::Stage::pose_publish, {});
    std::this_thread::sleep_for(1300ms);
  } else if (mode == "slow_sink") {
    {
      td::Scope sleeping(td::Stage::pose_publish, {});
      std::this_thread::sleep_for(150ms);
    }
    const auto deadline = Clock::now() + 4s;
    while (!std::filesystem::exists(std::getenv("SLOW_SINK_MARKER"))) {
      if (Clock::now() >= deadline) {return 3;}
      std::this_thread::sleep_for(1ms);
    }
    const auto start = Clock::now();
    for (int i = 0; i < 20000; ++i) {pulse(td::Stage::pose_publish);}
    std::printf("{\"probe_ms\":%.3f,\"probes\":20000}\n",
      std::chrono::duration<double, std::milli>(Clock::now() - start).count());
  } else if (mode == "benchmark" || mode == "baseline") {
    const bool enabled = mode == "benchmark";
    const auto start = Clock::now();
    const double initial_cpu = cpu_ms();
    double max_burst_ms = 0;
    // Five seconds: ten position stages at 100 Hz, plus two other-message stages at 500 Hz.
    for (int tick = 0; tick < 2500; ++tick) {
      std::this_thread::sleep_until(start + tick * 2ms);
      const auto burst = Clock::now();
      pulse(td::Stage::link_receive, tick * 2, enabled);
      pulse(td::Stage::uas_dispatch, tick * 2, enabled);
      if (tick % 5 == 0) {
        for (int stage = 0; stage < static_cast<int>(td::Stage::count); ++stage) {
          pulse(static_cast<td::Stage>(stage), tick * 2, enabled);
        }
      }
      max_burst_ms = std::max(max_burst_ms,
        std::chrono::duration<double, std::milli>(Clock::now() - burst).count());
    }
    const double elapsed = std::chrono::duration<double, std::milli>(Clock::now() - start).count();
    std::printf("{\"mode\":\"%s\",\"probes\":10000,\"wall_ms\":%.3f,\"cpu_ms\":%.3f,\"max_burst_ms\":%.3f}\n",
      mode.c_str(), elapsed, cpu_ms() - initial_cpu, max_burst_ms);
  } else {return 2;}
}
#endif
