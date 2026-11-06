// Receive and validate every Odin sample without entering the Python interpreter.
#include "correction_native/odometry.hpp"

#include <cmath>
#include <cstring>
#include <iomanip>
#include <limits>
#include <rclcpp/serialization.hpp>
#include <sstream>

namespace correction_native {
// Nanoseconds per second for source stamps and the original time gates.
constexpr std::int64_t SECOND = 1'000'000'000LL;
// Preserve the former Python callback's session discontinuity gates.
constexpr double RECEIVE_GAP_SECONDS = 2.;
constexpr std::int64_t CLOCK_ROLLBACK_NS = 250'000'000LL;
// Participant discovery happens at service startup, outside the unchanged 2s raw-data wait.
constexpr double DISCOVERY_WARMUP_SECONDS = 3.;

// Use the same Linux monotonic clock as Python time.monotonic().
static double monotonic_now() {
  timespec value{};
  clock_gettime(CLOCK_MONOTONIC, &value);
  return value.tv_sec + value.tv_nsec * 1e-9;
}

// Keep the same configurable history and matching gates as the Python synchronizer.
OdometryCapture::OdometryCapture(double history, double header_ms, double arrival_ms, double epoch,
                                 std::string session_seed, std::function<void()> notify)
    : history_(history * SECOND),
      header_limit_(header_ms * 1e6),
      arrival_limit_(arrival_ms * 1e6),
      epoch_(epoch * SECOND),
      seed_(std::move(session_seed)),
      notify_(std::move(notify)) {}

// Normal Python cleanup closes explicitly before this final ownership release.
OdometryCapture::~OdometryCapture() { shutdown(); }

// Keep a discovery participant while idle, just as the Python service keeps its ROS context.
void OdometryCapture::prepare(const std::string &topic, const std::string &ns, std::size_t domain,
                              bool sim_time) {
  if (context_) return;
  try {
    context_ = std::make_shared<rclcpp::Context>();
    rclcpp::InitOptions init;
    init.set_domain_id(domain);
    context_->init(0, nullptr, init);
    rclcpp::NodeOptions options;
    options.context(context_)
        .use_global_arguments(false)
        .start_parameter_services(false)
        .start_parameter_event_publisher(false)
        .parameter_overrides({rclcpp::Parameter("use_sim_time", sim_time)});
    node_ = std::make_shared<rclcpp::Node>("correction_odometry_capture", ns, options);
    rclcpp::ExecutorOptions executor_options;
    executor_options.context = context_;
    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>(executor_options);
    executor_->add_node(node_);
    const double deadline = monotonic_now() + DISCOVERY_WARMUP_SECONDS;
    while (node_->count_publishers(topic) == 0 && monotonic_now() < deadline)
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
  } catch (...) {
    shutdown();
    throw;
  }
}

// Create only a task subscription; reuse discovered peers so the original raw timeout remains
// valid.
void OdometryCapture::start(const std::string &topic, const std::string &ns, std::size_t domain,
                            bool sim_time) {
  if (subscription_) throw std::runtime_error("Odin task subscription is already running");
  prepare(topic, ns, domain, sim_time);
  {
    std::lock_guard<std::mutex> lock(mutex_);
    records_.clear();
    faults_.clear();
    state_ = {};
    ++generation_;
  }
  try {
    subscription_ = node_->create_subscription<nav_msgs::msg::Odometry>(
        topic, rclcpp::SensorDataQoS(), [this](nav_msgs::msg::Odometry::ConstSharedPtr message) {
          const double monotonic = monotonic_now();
          const auto arrival = node_->now().nanoseconds();
          add(std::move(message), arrival, monotonic);
        });
    thread_ = std::thread([this]() {
      try {
        executor_->spin();
      } catch (const std::exception &error) {
        {
          std::lock_guard<std::mutex> lock(mutex_);
          faults_.push_back({std::string("Odin native receiver failed: ") + error.what(), false});
        }
        notify_();
      }
    });
  } catch (...) {
    close();
    throw;
  }
}

// Join callbacks and remove every high-rate resource while retaining only DDS discovery.
void OdometryCapture::close() {
  if (executor_) executor_->cancel();
  if (thread_.joinable()) thread_.join();
  subscription_.reset();
  std::lock_guard<std::mutex> lock(mutex_);
  records_.clear();
  // Leave queued faults available until Python has completed task cleanup.
  state_ = {};
}

// Service shutdown also releases its otherwise idle native discovery participant.
void OdometryCapture::shutdown() {
  close();
  if (executor_ && node_) executor_->remove_node(node_);
  executor_.reset();
  node_.reset();
  if (context_ && context_->is_valid()) context_->shutdown("correction task released");
  context_.reset();
}

// Validate every source message and report rare faults independently of sampled observations.
void OdometryCapture::add(nav_msgs::msg::Odometry::ConstSharedPtr message, std::int64_t arrival,
                          double monotonic) {
  bool notify = false;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto header = static_cast<std::int64_t>(message->header.stamp.sec) * SECOND +
                        message->header.stamp.nanosec;
    const auto fault = [&](std::string reason, bool invalidate) {
      faults_.push_back({std::move(reason), invalidate});
      notify = true;
    };
    if (message->header.frame_id != "odom" || message->child_frame_id != "fcu") {
      fault("原始里程计必须为 odom/fcu，收到 ('" + message->header.frame_id + "', '" +
                message->child_frame_id + "')",
            true);
    } else {
      std::string discontinuity;
      if (state_.received > 0 && monotonic - state_.received > RECEIVE_GAP_SECONDS) {
        std::ostringstream reason;
        reason << "任务 Odin 断流 " << std::fixed << std::setprecision(3)
               << monotonic - state_.received << "s";
        discontinuity = reason.str();
      } else if (state_.header > 0 && header < state_.header - CLOCK_ROLLBACK_NS) {
        discontinuity = "任务 Odin header 时间戳回退";
      } else if (state_.arrival > 0 && arrival < state_.arrival - CLOCK_ROLLBACK_NS) {
        discontinuity = "主机 ROS 时钟回退";
      }
      if (!discontinuity.empty()) {
        fault(discontinuity, true);
        records_.clear();
        ++generation_;
      }
      if (state_.session.empty() || !discontinuity.empty())
        state_.session =
            "local-" + std::to_string(header) + "-" + seed_ + "-" + std::to_string(generation_);
      const auto &p = message->pose.pose.position;
      const auto &q = message->pose.pose.orientation;
      const double norm = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
      if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {
        fault("原始 Odin 位姿无效：位姿平移含非有限值", false);
      } else if (!std::isfinite(norm) || norm < 1e-9) {
        fault("原始 Odin 位姿无效：四元数无效", false);
      } else {
        const double x = q.x / norm, y = q.y / norm, z = q.z / norm, w = q.w / norm;
        state_.transform = {1 - 2 * (y * y + z * z),
                            2 * (x * y - z * w),
                            2 * (x * z + y * w),
                            p.x,
                            2 * (x * y + z * w),
                            1 - 2 * (x * x + z * z),
                            2 * (y * z - x * w),
                            p.y,
                            2 * (x * z - y * w),
                            2 * (y * z + x * w),
                            1 - 2 * (x * x + y * y),
                            p.z,
                            0,
                            0,
                            0,
                            1};
        state_.valid = true;
        state_.received = monotonic;
        state_.header = header;
        state_.arrival = arrival;
        records_.push_back({std::move(message), header, arrival});
        while (!records_.empty() && records_.front().arrival < arrival - history_)
          records_.pop_front();
      }
    }
  }
  if (notify) notify_();
}

// Prefer sensor header time; use arrival history only for incompatible epochs.
Match OdometryCapture::match(std::int64_t stamp) const {
  std::lock_guard<std::mutex> lock(mutex_);
  if (records_.empty()) return {};
  const Record *header = nullptr, *arrival = nullptr;
  std::int64_t dh = std::numeric_limits<std::int64_t>::max(), da = dh;
  for (const auto &record : records_) {
    const auto h = std::llabs(record.header - stamp), a = std::llabs(record.arrival - stamp);
    if (h < dh) {
      dh = h;
      header = &record;
    }
    if (a < da) {
      da = a;
      arrival = &record;
    }
  }
  if (dh <= epoch_)
    return dh <= header_limit_ ? Match{header->message, dh / 1e6, "header"} : Match{};
  return da <= arrival_limit_ ? Match{arrival->message, da / 1e6, "arrival_history"} : Match{};
}

// Copy a coherent latest state while the native receiver continues independently.
RawState OdometryCapture::snapshot() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return state_;
}

// Transfer rare faults to the Python state machine exactly once.
std::vector<Fault> OdometryCapture::faults() {
  std::lock_guard<std::mutex> lock(mutex_);
  std::vector<Fault> result;
  result.swap(faults_);
  return result;
}

// Preserve the original synchronizer's history-only clear operation.
void OdometryCapture::clear() {
  std::lock_guard<std::mutex> lock(mutex_);
  records_.clear();
}

// Expose actual resource ownership to lifecycle checks.
bool OdometryCapture::running() const { return subscription_ != nullptr; }

// Serialize only the selected observation, preserving covariance and twist fields.
std::string serialize(const nav_msgs::msg::Odometry &message) {
  rclcpp::SerializedMessage bytes;
  rclcpp::Serialization<nav_msgs::msg::Odometry>().serialize_message(&message, &bytes);
  const auto &raw = bytes.get_rcl_serialized_message();
  return {reinterpret_cast<const char *>(raw.buffer), raw.buffer_length};
}

// Feed diagnostic replay through the same native validator as live DDS messages.
nav_msgs::msg::Odometry::SharedPtr deserialize(const std::string &bytes) {
  rclcpp::SerializedMessage data(bytes.size());
  auto &raw = data.get_rcl_serialized_message();
  std::memcpy(raw.buffer, bytes.data(), bytes.size());
  raw.buffer_length = bytes.size();
  auto result = std::make_shared<nav_msgs::msg::Odometry>();
  rclcpp::Serialization<nav_msgs::msg::Odometry>().deserialize_message(&data, result.get());
  return result;
}
}  // namespace correction_native
