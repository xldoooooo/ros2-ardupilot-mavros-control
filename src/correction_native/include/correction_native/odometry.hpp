// Full-rate native Odin history, fault notifications and on-demand Python snapshots.
#pragma once
#include <array>
#include <deque>
#include <functional>
#include <mutex>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <thread>

namespace correction_native {
struct RawState {
  double received = 0;
  std::int64_t header = 0, arrival = 0;
  std::array<double, 16> transform{};
  std::string session;
  bool valid = false;
};
struct Fault {
  std::string reason;
  bool invalidate = false;
};
struct Match {
  nav_msgs::msg::Odometry::ConstSharedPtr message;
  double delta_ms = 0;
  std::string time_source;
};
class OdometryCapture {
 public:
  // Keep the original four synchronization settings and an opaque session seed.
  OdometryCapture(double history, double header_ms, double arrival_ms, double epoch,
                  std::string session_seed, std::function<void()> notify);
  // Stop the private executor before releasing its subscriptions or context.
  ~OdometryCapture();
  // Warm DDS discovery at service startup without subscribing to high-rate data.
  void prepare(const std::string &topic, const std::string &ns, std::size_t domain, bool sim_time);
  // Start only this task's subscription, respecting the caller's ROS domain and clock.
  void start(const std::string &topic, const std::string &ns, std::size_t domain, bool sim_time);
  // Join callbacks and clear task data, retaining queued faults for Python cleanup.
  void close();
  // Release the discovery participant only when the owning service exits.
  void shutdown();
  // Accept a sample with its actual callback arrival, also used by isolated replay tests.
  void add(nav_msgs::msg::Odometry::ConstSharedPtr message, std::int64_t arrival, double monotonic);
  // Select the oldest equal-distance match on the original preferred time axis.
  Match match(std::int64_t stamp) const;
  // Return the latest valid state without converting all 400Hz messages to Python.
  RawState snapshot() const;
  // Drain rare faults for the existing Python executor's guard condition.
  std::vector<Fault> faults();
  // Clear history without retiming or changing the last valid raw state.
  void clear();
  // Expose lifecycle state for resource verification.
  bool running() const;

 private:
  struct Record {
    nav_msgs::msg::Odometry::ConstSharedPtr message;
    std::int64_t header, arrival;
  };
  std::int64_t history_, header_limit_, arrival_limit_, epoch_;
  std::string seed_;
  std::uint64_t generation_ = 0;
  std::function<void()> notify_;
  mutable std::mutex mutex_;
  std::deque<Record> records_;
  std::vector<Fault> faults_;
  RawState state_;
  rclcpp::Context::SharedPtr context_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::thread thread_;
};
// Preserve complete ROS messages only for samples actually requested by Python.
std::string serialize(const nav_msgs::msg::Odometry &message);
// Decode replay data through the same ROS type support used by live reception.
nav_msgs::msg::Odometry::SharedPtr deserialize(const std::string &bytes);
}  // namespace correction_native
