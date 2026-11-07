/** @file tracking_diagnostics.cpp
 * @brief Subscriber-activated snapshots; no idle sampling, timer or disk IO.
 */
#include "onboard_control/onboard_control_node.hpp"

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace onboard_control
{

/** Start/stop the independent publisher timer on DDS matching events, not on hover entry. */
void OnboardControlNode::initialize_tracking_diagnostics()
{
  tracking_session_ = static_cast<std::uint64_t>(
    std::chrono::system_clock::now().time_since_epoch().count());
  tracking_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  rclcpp::PublisherOptions options;
  options.callback_group = tracking_callback_group_;
  options.event_callbacks.matched_callback = [this](rclcpp::MatchedInfo & info) {
      std::lock_guard<std::recursive_mutex> lock(mutex_);
      tracking_requested_ = info.current_count > 0;
      if (!tracking_requested_) {
        if (tracking_timer_) {tracking_timer_->cancel();}
        tracking_timer_.reset();
        tracking_sample_.reset();
      } else if (!tracking_timer_) {
        // Poll at twice the control rate only while requested, avoiding same-rate timer aliasing.
        const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::duration<double>(0.5 / control_frequency_hz_));
        tracking_timer_ = create_wall_timer(period, [this]() {
            std::optional<guided_interfaces::msg::TrackingSample> sample;
            {
              std::lock_guard<std::recursive_mutex> state_lock(mutex_);
              sample.swap(tracking_sample_);
            }
            // No execution/state lock, file IO or logging on the flight callback.
            if (sample) {tracking_publisher_->publish(*sample);}
          }, tracking_callback_group_);
      }
    };
  try {
    tracking_publisher_ = create_publisher<guided_interfaces::msg::TrackingSample>(
      interface_prefix_ + "/tracking_sample", rclcpp::QoS(1).best_effort(), options);
  } catch (const rclcpp::UnsupportedEventTypeException & error) {
    // Optional diagnostics must not prevent control on a middleware without matched events.
    RCLCPP_WARN(get_logger(), "Tracking recorder unavailable: %s", error.what());
  }
}

/** Capture actual inputs, active gains and observer state under the existing flight lock. */
void OnboardControlNode::capture_tracking_sample(
  const VehicleKinematics & state, const ControlOutput & output,
  const double dt_seconds, const bool feedforward, const bool output_jerk)
{
  if (!tracking_requested_) {return;}
  guided_interfaces::msg::TrackingSample sample;
  const auto now = SteadyClock::now();
  sample.header.stamp = get_clock()->now();
  sample.header.frame_id = "fcu_local_enu";
  sample.session_id = tracking_session_;
  sample.sequence = ++tracking_sequence_;
  sample.monotonic_s = std::chrono::duration<double>(now.time_since_epoch()).count();
  sample.dt_s = dt_seconds;
  sample.pose_age_s = std::chrono::duration<double>(now - last_pose_time_).count();
  sample.velocity_age_s = std::chrono::duration<double>(now - last_velocity_time_).count();
  sample.control_mode = control_mode_;
  sample.tracking_controller = static_cast<std::uint8_t>(active_tracking_controller_);
  sample.calibration_active = calibration_stage_ != CalibrationStage::kIdle;
  sample.acceleration_feedforward = feedforward;
  sample.output_jerk = output_jerk;
  // Copy ENU vectors without converting through task/map coordinates.
  const auto vector = [](auto & target, const Eigen::Vector3d & source) {
      target.x = source.x(); target.y = source.y(); target.z = source.z();
    };
  vector(sample.position, state.position);
  vector(sample.velocity, state.velocity);
  vector(sample.reference_position, reference_.position);
  vector(sample.reference_velocity, reference_.velocity);
  vector(sample.reference_acceleration, reference_.acceleration);
  vector(sample.disturbance, output.disturbance);
  vector(sample.controller_acceleration, output.acceleration);
  vector(sample.mapped_acceleration, output.mapped_acceleration);
  tf2::Quaternion attitude;
  attitude.setRPY(roll_, pitch_, yaw_);
  sample.attitude = tf2::toMsg(attitude);
  sample.attitude_setpoint.x = output.attitude.x();
  sample.attitude_setpoint.y = output.attitude.y();
  sample.attitude_setpoint.z = output.attitude.z();
  sample.attitude_setpoint.w = output.attitude.w();
  sample.reference_yaw = reference_.yaw;
  sample.thrust = output.thrust;
  sample.hover_throttle = controller_.hover_throttle();
  const auto & profile = active_tracking_controller_ == TrackingControllerType::kTrajectoryPdDob ?
    trajectory_controller_parameters_ : controller_parameters_;
  sample.wn_xy = profile.wn_xy;
  sample.zeta_xy = profile.zeta_xy;
  sample.wn_z = profile.wn_z;
  sample.zeta_z = profile.zeta_z;
  sample.observer_xy = profile.observer_xy;
  sample.observer_z = profile.observer_z;
  sample.max_acceleration_xy = profile.max_acceleration_xy;
  sample.max_acceleration_z = profile.max_acceleration_z;
  sample.max_disturbance = profile.max_disturbance;
  sample.max_tilt_rad = profile.max_tilt_rad;
  sample.gravity = profile.gravity;
  tracking_sample_ = std::move(sample);
}

}  // namespace onboard_control
