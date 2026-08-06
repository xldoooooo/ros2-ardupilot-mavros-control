/**
 * @file onboard_control_node.hpp
 * @brief 机载命令仲裁、任务执行、MAVROS 网关、租约与失联保护节点。
 */
#pragma once

#include <chrono>
#include <cstdint>
#include <functional>
#include <future>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include <Eigen/Core>

#include <geographic_msgs/msg/geo_point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <guided_interfaces/msg/command_result.hpp>
#include <guided_interfaces/msg/control_heartbeat.hpp>
#include <guided_interfaces/msg/control_status.hpp>
#include <guided_interfaces/msg/motion_intent.hpp>
#include <guided_interfaces/msg/waypoint.hpp>
#include <guided_interfaces/srv/acquire_control.hpp>
#include <guided_interfaces/srv/execute_waypoints.hpp>
#include <guided_interfaces/srv/flight_command.hpp>
#include <guided_interfaces/srv/set_gps_origin.hpp>
#include <mavros_msgs/msg/attitude_target.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/command_tol.hpp>
#include <mavros_msgs/srv/message_interval.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/parameter_client.hpp>

#include "onboard_control/dob_controller.hpp"

namespace onboard_control
{

/** 单进程组合节点；高频控制不依赖地面站或图形界面生命周期。 */
class OnboardControlNode : public rclcpp::Node
{
public:
  explicit OnboardControlNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  using SteadyClock = std::chrono::steady_clock;
  using SteadyTime = SteadyClock::time_point;
  using FlightCommand = guided_interfaces::srv::FlightCommand;
  using ExecuteWaypoints = guided_interfaces::srv::ExecuteWaypoints;
  using AcquireControl = guided_interfaces::srv::AcquireControl;
  using SetGpsOrigin = guided_interfaces::srv::SetGpsOrigin;

  enum class ActiveTask
  {
    kNone,
    kTakeoff,
    kWaypoint,
    kLand,
  };

  struct CommandIdentity
  {
    std::string source;
    std::uint64_t sequence{0};
    std::string name;
  };

  // ROS subscription callbacks update the authoritative vehicle state.
  void on_fcu_state(const mavros_msgs::msg::State::SharedPtr message);
  void on_pose(const geometry_msgs::msg::PoseStamped::SharedPtr message);
  void on_velocity(const geometry_msgs::msg::TwistStamped::SharedPtr message);
  void on_heartbeat(const guided_interfaces::msg::ControlHeartbeat::SharedPtr message);
  void on_motion_intent(const guided_interfaces::msg::MotionIntent::SharedPtr message);

  // High-level services are the only ground-station command entry points.
  void on_acquire_control(
    const std::shared_ptr<AcquireControl::Request> request,
    std::shared_ptr<AcquireControl::Response> response);
  void on_flight_command(
    const std::shared_ptr<FlightCommand::Request> request,
    std::shared_ptr<FlightCommand::Response> response);
  void on_execute_waypoints(
    const std::shared_ptr<ExecuteWaypoints::Request> request,
    std::shared_ptr<ExecuteWaypoints::Response> response);
  void on_set_gps_origin(
    const std::shared_ptr<SetGpsOrigin::Request> request,
    std::shared_ptr<SetGpsOrigin::Response> response);

  // Fixed-rate and diagnostic timers remain entirely onboard.
  void control_tick();
  void status_tick();

  // Command validation, arbitration and state transitions.
  bool validate_envelope(
    const builtin_interfaces::msg::Time & stamp,
    std::uint32_t ttl_ms,
    const std::string & source,
    std::string & reason) const;
  bool authorize_flight_sequence(
    const std::string & source,
    std::uint64_t sequence,
    std::string & reason);
  bool has_control_locked(const std::string & source) const;
  bool lease_active_locked() const;
  std::uint32_t lease_remaining_ms_locked() const;
  void start_takeoff(const CommandIdentity & command, double altitude);
  void start_waypoint_task(
    const CommandIdentity & command,
    const std::vector<guided_interfaces::msg::Waypoint> & waypoints);
  void start_land(const CommandIdentity & command, bool failsafe);
  void enter_hover(
    const std::string & reason,
    std::uint8_t mode = guided_interfaces::msg::ControlStatus::MODE_HOVER);
  void cancel_active_task(const std::string & reason);
  void fail_active_task(const std::string & reason, bool request_land);
  void clear_failsafe_locked();

  // MAVROS asynchronous service chains never block the 100 Hz timer.
  void ensure_guided_and_armed(
    const CommandIdentity & command,
    const std::function<void()> & on_ready);
  void ensure_armed(
    const CommandIdentity & command,
    const std::function<void()> & on_ready);
  void send_takeoff_request(const CommandIdentity & command);
  void send_land_mode_request(const CommandIdentity & command, bool failsafe);
  bool active_task_matches(const CommandIdentity & command) const;

  // Maintenance services are serialized independently of flight tasks.
  void start_message_rate_configuration(const CommandIdentity & command);
  void send_next_message_rate();
  void check_thrust_mode_parameter();

  // Onboard task, safety and output helpers.
  void update_waypoint_executor(const SteadyTime & now);
  void update_motion_reference(double dt_seconds);
  void enforce_safety(const SteadyTime & now);
  void trigger_link_loss_hold(const SteadyTime & now);
  void trigger_failsafe_land(const std::string & reason);
  void publish_attitude_setpoint(double dt_seconds);
  void publish_result(
    const CommandIdentity & command,
    std::uint8_t status,
    bool final,
    const std::string & message,
    std::uint32_t waypoint_index = 0,
    std::uint32_t waypoint_count = 0);
  void set_status_message(const std::string & message);

  static std::string mode_label(std::uint8_t mode);
  static double normalize_angle(double angle);

  mutable std::recursive_mutex mutex_;

  // Parameters with safety significance are declared once and reported in logs.
  double control_frequency_hz_{100.0};
  double status_frequency_hz_{10.0};
  double pose_timeout_seconds_{0.3};
  double state_timeout_seconds_{2.0};
  double link_loss_land_timeout_seconds_{10.0};
  double takeoff_timeout_seconds_{45.0};
  double waypoint_tolerance_{0.3};
  double waypoint_hold_seconds_{1.0};
  double max_velocity_xy_{1.5};
  double max_velocity_z_{0.8};
  double max_yaw_rate_{1.0};
  double max_reference_error_xy_{0.8};
  double max_reference_error_z_{0.5};
  double max_clock_skew_seconds_{2.0};
  std::string mavros_prefix_{"/mavros"};
  std::string interface_prefix_{"/onboard_control"};
  ControllerParameters controller_parameters_;
  DobController controller_;

  // Latest MAVROS state and freshness markers.
  bool fcu_connected_{false};
  bool armed_{false};
  std::string autopilot_mode_;
  bool pose_valid_{false};
  bool velocity_valid_{false};
  VehicleKinematics vehicle_;
  double yaw_{0.0};
  SteadyTime last_state_time_{};
  SteadyTime last_pose_time_{};
  SteadyTime last_velocity_time_{};

  // Single onboard control state shared by motion, hover and waypoint execution.
  std::uint8_t control_mode_{guided_interfaces::msg::ControlStatus::MODE_IDLE};
  bool controller_engaged_{false};
  ControlReference reference_;
  double target_yaw_rate_{0.0};
  std::string status_message_{"机载控制服务已启动"};
  std::string failsafe_reason_;
  CommandIdentity active_command_;
  ActiveTask active_task_{ActiveTask::kNone};
  SteadyTime active_task_started_{};
  double takeoff_altitude_{0.0};

  // Waypoint queue and arrival dwell state live onboard, never in the GUI process.
  std::vector<guided_interfaces::msg::Waypoint> waypoints_;
  std::size_t waypoint_index_{0};
  std::optional<SteadyTime> waypoint_arrival_started_;

  // Lease, replay protection and link-loss state.
  std::string lease_owner_;
  SteadyTime lease_deadline_{};
  std::unordered_map<std::string, std::uint64_t> last_lease_sequence_;
  std::unordered_map<std::string, std::uint64_t> last_flight_sequence_;
  std::optional<SteadyTime> link_loss_started_;
  bool failsafe_land_requested_{false};

  // MAVLink message-rate setup runs independently from flight-mode transitions.
  bool message_rates_configured_{false};
  bool message_rate_configuration_active_{false};
  CommandIdentity message_rate_command_;
  std::size_t message_rate_index_{0};
  bool thrust_mode_verified_{false};
  bool thrust_mode_check_inflight_{false};
  SteadyTime last_thrust_mode_check_{};

  // Timer diagnostics and publisher-conflict protection.
  SteadyTime last_control_tick_{};
  double measured_control_rate_hz_{0.0};
  double max_jitter_ms_{0.0};
  std::uint64_t deadline_miss_count_{0};
  bool setpoint_conflict_{false};
  unsigned int conflict_observations_{0};
  std::string attitude_topic_;

  rclcpp::Publisher<mavros_msgs::msg::AttitudeTarget>::SharedPtr attitude_publisher_;
  rclcpp::Publisher<geographic_msgs::msg::GeoPointStamped>::SharedPtr origin_publisher_;
  rclcpp::Publisher<guided_interfaces::msg::ControlStatus>::SharedPtr status_publisher_;
  rclcpp::Publisher<guided_interfaces::msg::CommandResult>::SharedPtr result_publisher_;

  rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr velocity_subscription_;
  rclcpp::Subscription<guided_interfaces::msg::ControlHeartbeat>::SharedPtr heartbeat_subscription_;
  rclcpp::Subscription<guided_interfaces::msg::MotionIntent>::SharedPtr motion_subscription_;

  rclcpp::Service<AcquireControl>::SharedPtr acquire_service_;
  rclcpp::Service<FlightCommand>::SharedPtr flight_command_service_;
  rclcpp::Service<ExecuteWaypoints>::SharedPtr waypoint_service_;
  rclcpp::Service<SetGpsOrigin>::SharedPtr origin_service_;

  rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_client_;
  rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arming_client_;
  rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedPtr takeoff_client_;
  rclcpp::Client<mavros_msgs::srv::MessageInterval>::SharedPtr message_interval_client_;
  rclcpp::AsyncParametersClient::SharedPtr fcu_parameter_client_;

  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::TimerBase::SharedPtr status_timer_;
  rclcpp::CallbackGroup::SharedPtr control_callback_group_;
};

}  // namespace onboard_control
