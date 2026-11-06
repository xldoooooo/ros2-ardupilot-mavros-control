/** @file telemetry_diagnostics.cpp
 * Fixed-size probes; an independent thread formats abnormal snapshots at most once a second.
 */
#include "mavros/telemetry_diagnostics.hpp"

#include <array>
#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <mutex>
#include <thread>
#include <sys/resource.h>
#include <unistd.h>

namespace mavros::telemetry_diagnostics
{
namespace
{
// Diagnostic selection only; never a flight or message acceptance threshold.
constexpr int64_t kSlowNs = 100000000;
// Names and storage are bounded independently of traffic volume.
constexpr std::array<const char *, static_cast<size_t>(Stage::count)> kNames = {
  "link_receive", "position_receive", "router_publish", "uas_dispatch", "uas_position",
  "position_handler", "odom_publish", "pose_publish", "body_velocity_publish", "velocity_publish"};

/** Sample current-thread time and scheduling counters without allocation. */
Sample sample()
{
  Sample s;
  rusage usage{};
  if (::getrusage(RUSAGE_THREAD, &usage) == 0) {
    s.cpu_ns = (usage.ru_utime.tv_sec + usage.ru_stime.tv_sec) * 1000000000LL +
      (usage.ru_utime.tv_usec + usage.ru_stime.tv_usec) * 1000LL;
    s.voluntary = usage.ru_nvcsw;
    s.involuntary = usage.ru_nivcsw;
    s.minor = usage.ru_minflt;
    s.major = usage.ru_majflt;
  }
  s.steady_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
  s.wall_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::system_clock::now().time_since_epoch()).count();
  s.tid = ::gettid();
  return s;
}

struct Event
{
  Identity identity;
  Sample start, finish;
  int64_t gap_ns{0}, previous_wall_ns{0};
  uint32_t previous_boot_ms{0};
};

struct Slot
{
  Event latest, pending;
  uint64_t count{0}, anomalies{0}, overlap{0}, skipped{0};
  bool active{false};
};

/** Own bounded snapshots and a private sink; no disk operation holds the snapshot mutex. */
class Recorder
{
public:
  /** Open the per-process sink once, before the first observed callback. */
  Recorder()
  {
    try {
      const char * directory = std::getenv("ROS_LOG_DIR");
      const char * home = std::getenv("HOME");
      auto root = directory ? std::filesystem::path(directory) :
        std::filesystem::path(home ? home : "/tmp") / ".ros/log";
      std::filesystem::create_directories(root);
      auto path = root / ("mavros-telemetry-" + std::to_string(::getpid()) + "-" +
        std::to_string(sample().wall_ns) + ".log");
      file_ = std::fopen(path.c_str(), "ae");
      if (file_) {
        std::fprintf(file_, "[MAVROS_DIAGNOSTICS] version=1 pid=%d slow_ms=100 period_ms=1000\n", ::getpid());
        std::fflush(file_);
        worker_ = std::thread([this]() {run();});
      }
    } catch (...) {
      // Diagnostics must not prevent MAVROS startup if the sink/thread is unavailable.
      if (file_) {std::fclose(file_); file_ = nullptr;}
    }
    if (!file_) {std::fprintf(stderr, "MAVROS telemetry diagnostic sink unavailable\n");}
  }

  /** Stop the writer before releasing its sink. */
  ~Recorder()
  {
    {std::lock_guard<std::mutex> lock(wait_mutex_); stop_ = true;}
    wake_.notify_one();
    if (worker_.joinable()) {worker_.join();}
    if (file_) {std::fclose(file_);}
  }

  /** Never wait for a diagnostic lock; contention only loses an explicitly counted sample. */
  bool begin(Stage stage, Identity identity, Sample started)
  {
    if (!file_) {return false;}
    const auto index = static_cast<size_t>(stage);
    std::unique_lock<std::mutex> lock(mutex_, std::try_to_lock);
    if (!lock.owns_lock()) {++skipped_[index]; return false;}
    auto & slot = slots_[index];
    if (active_[index].load()) {++slot.overlap; return false;}
    const auto previous = slot.latest;
    slot.latest = Event{identity, started, {},
      slot.count ? started.steady_ns - previous.start.steady_ns : 0,
      previous.start.wall_ns, previous.identity.boot_ms};
    ++slot.count;
    active_[index] = true;
    return true;
  }

  /** Coalesce completed slow work or arrival gaps, keeping the largest event for this boundary. */
  void end(Stage stage, Sample finished)
  {
    const auto index = static_cast<size_t>(stage);
    std::unique_lock<std::mutex> lock(mutex_, std::try_to_lock);
    if (!lock.owns_lock()) {
      ++skipped_[index];
      active_[index] = false;
      return;
    }
    auto & slot = slots_[index];
    slot.latest.finish = finished;
    active_[index] = false;
    const auto duration = [](const Event & e) {
        return e.finish.steady_ns - e.start.steady_ns;
      };
    const auto work = duration(slot.latest);
    if (std::max(slot.latest.gap_ns, work) < kSlowNs) {return;}
    // A long call also delays the next entry: never replace its CPU evidence with that later gap.
    const auto prior_work = duration(slot.pending);
    const bool prefer = work >= kSlowNs ? work > prior_work :
      prior_work < kSlowNs && slot.latest.gap_ns > slot.pending.gap_ns;
    if (slot.anomalies++ == 0 || prefer) {
      slot.pending = slot.latest;
    }
  }

private:
  /** Write one bounded record with both wire identity and host timing. */
  void write(const char * kind, size_t index, const Event & e, const Slot & s, int64_t now)
  {
    const auto & a = e.start;
    const auto & b = e.finish;
    const bool done = b.steady_ns != 0;
    const bool cpu = done && a.cpu_ns >= 0 && b.cpu_ns >= 0;
    std::fprintf(file_,
      "[MAVROS_%s] stage=%s event_ns=%lld tid=%d msg=%u seq=%u source=%u.%u framing=%d"
      " boot_ms=%u previous_boot_ms=%u bridge_stamp_ns=%lld previous_event_ns=%lld"
      " gap_ms=%.3f duration_ms=%.3f age_ms=%.3f cpu_ms=%.3f"
      " voluntary=%ld involuntary=%ld minor_faults=%ld major_faults=%ld count=%llu coalesced=%llu overlap=%llu skipped=%llu\n",
      kind, kNames[index], static_cast<long long>(a.wall_ns), a.tid, e.identity.message,
      e.identity.sequence, e.identity.system, e.identity.component, e.identity.framing,
      e.identity.boot_ms, e.previous_boot_ms, static_cast<long long>(e.identity.bridge_stamp_ns),
      static_cast<long long>(e.previous_wall_ns), e.gap_ns / 1e6,
      done ? (b.steady_ns - a.steady_ns) / 1e6 : (s.active ? (now - a.steady_ns) / 1e6 : -1.0),
      (now - a.steady_ns) / 1e6,
      cpu ? (b.cpu_ns - a.cpu_ns) / 1e6 : -1.0,
      cpu ? b.voluntary - a.voluntary : -1, cpu ? b.involuntary - a.involuntary : -1,
      cpu ? b.minor - a.minor : -1, cpu ? b.major - a.major : -1,
      static_cast<unsigned long long>(s.count), static_cast<unsigned long long>(s.anomalies),
      static_cast<unsigned long long>(s.overlap), static_cast<unsigned long long>(s.skipped));
  }

  /** Copy snapshots first, then perform all formatting/I/O on this independent thread. */
  void run()
  {
    std::unique_lock<std::mutex> wait_lock(wait_mutex_);
    while (!wake_.wait_for(wait_lock, std::chrono::seconds(1), [this]() {return stop_;})) {
      std::array<Slot, kNames.size()> snapshot;
      {
        std::unique_lock<std::mutex> lock(mutex_, std::try_to_lock);
        if (!lock.owns_lock()) {continue;}
        snapshot = slots_;
        for (size_t i = 0; i < slots_.size(); ++i) {
          slots_[i].anomalies = 0;
          snapshot[i].active = active_[i].load();
          snapshot[i].skipped = skipped_[i].load();
        }
      }
      const auto now = sample().steady_ns;
      bool abnormal = false;
      for (size_t i = 0; i < snapshot.size(); ++i) {
        const auto & s = snapshot[i];
        // An optional publication may legitimately disappear; only watch incoming position silence.
        const bool watch_age = s.active || i == static_cast<size_t>(Stage::position_receive);
        abnormal |= s.anomalies != 0 ||
          (watch_age && s.count && now - s.latest.start.steady_ns >= kSlowNs);
      }
      if (!abnormal) {continue;}
      for (size_t i = 0; i < snapshot.size(); ++i) {
        const auto & s = snapshot[i];
        if (s.anomalies) {write("EVENT", i, s.pending, s, now);}
        if (s.count) {write(s.active ? "INFLIGHT" : "LATEST", i, s.latest, s, now);}
      }
      std::fflush(file_);
    }
  }

  std::array<Slot, kNames.size()> slots_{};
  // Completion remains observable even when a timing snapshot is skipped under contention.
  std::array<std::atomic<bool>, kNames.size()> active_{};
  std::array<std::atomic<uint64_t>, kNames.size()> skipped_{};
  std::mutex mutex_, wait_mutex_;
  std::condition_variable wake_;
  std::thread worker_;
  FILE * file_{nullptr};
  bool stop_{false};
};

/** Share one recorder between the MAVROS library and plugin shared object. */
Recorder & recorder()
{
  static Recorder instance;
  return instance;
}
}  // namespace

/** Initialize before transport startup so first-message handling performs no file setup. */
void initialize()
{
  (void)recorder();
}

/** Register a scope only when this message belongs to the selected boundary. */
Scope::Scope(Stage stage, Identity identity, bool enabled)
: stage_(stage), identity_(identity)
{
  if (enabled) {
    auto & sink = recorder();
    started_ = sample();
    active_ = sink.begin(stage_, identity_, started_);
  }
}

/** Record completion even during normal exception unwinding. */
Scope::~Scope()
{
  if (active_) {recorder().end(stage_, sample());}
}
}  // namespace mavros::telemetry_diagnostics
