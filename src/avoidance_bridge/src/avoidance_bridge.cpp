// Adapt FCU coordinates and goal lifecycle to the native planner; no map or search scheduler.
#include <correction_interfaces/msg/extnav_correction_status.hpp>
#include <guided_interfaces/msg/avoidance_request.hpp>
#include <guided_interfaces/msg/avoidance_result.hpp>
#include <guided_interfaces/msg/control_status.hpp>
#include <path_planning/msg/plan_result.hpp>
#include <path_planning/srv/plan_motion.hpp>
#include <path_planning/srv/set_goal.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <array>
#include <chrono>
#include <cmath>
#include <cstring>
#include <optional>

using Steady = std::chrono::steady_clock;
using Request = guided_interfaces::msg::AvoidanceRequest;
using Result = guided_interfaces::msg::AvoidanceResult;
using NativeResult = path_planning::msg::PlanResult;
using Motion = path_planning::srv::PlanMotion;
using Goal = path_planning::srv::SetGoal;
using Status = correction_interfaces::msg::ExtnavCorrectionStatus;
using Cloud = sensor_msgs::msg::PointCloud2;
using Odom = nav_msgs::msg::Odometry;

// Extnav publishes coordinate identity at 2 Hz; allow scheduling jitter independently of FCU age.
constexpr double kCoordinateStatusTimeoutSeconds = 2.0;

// Return the local reception age independently of device clocks.
static double age(Steady::time_point time) {
  return std::chrono::duration<double>(Steady::now() - time).count();
}

// Reject invalid coordinate values before forwarding them.
static bool finite(const geometry_msgs::msg::Point &p) {
  return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z);
}

class Bridge : public rclcpp::Node {
public:
  // Connect only coordinate, target and output interfaces; the planner owns all map updates.
  Bridge() : Node("avoidance_bridge") {
    mode_ = declare_parameter("coordinate_mode", std::string("extnav"));
    frame_ = declare_parameter("planning_frame", mode_ == "identity" ? "map" : "odom");
    timeout_ = declare_parameter("input_timeout", .5);
    preview_ = declare_parameter("preview_enabled", false);
    // Persisted/CLI backend choices cannot activate an optional module during startup.
    declare_parameter("native_namespace", std::string("/avoidance"),
                      rcl_interfaces::msg::ParameterDescriptor{}, true);
    if ((mode_ != "identity" && mode_ != "extnav") || frame_.empty() ||
        !std::isfinite(timeout_) || timeout_ <= 0.) {
      throw std::invalid_argument("invalid coordinate mode, frame or input timeout");
    }
    session_ = std::to_string(Steady::now().time_since_epoch().count());
    result_pub_ = create_publisher<Result>("/onboard_control/avoidance_result", rclcpp::QoS(10));
    odom_pub_ = create_publisher<Odom>("/avoidance/odom", rclcpp::SensorDataQoS());
    preview_pub_ = create_publisher<Cloud>("/onboard_control/obstacle_preview", rclcpp::SensorDataQoS());
    connect_native("/avoidance");
    parameter_callback_ = add_on_set_parameters_callback([this](const auto &parameters) {
      rcl_interfaces::msg::SetParametersResult result;
      result.successful = true;
      for (const auto &parameter : parameters) {
        if (parameter.get_name() != "native_namespace") { continue; }
        const auto target = parameter.as_string();
        if (target == native_namespace_) { continue; }
        if ((target != "/avoidance" && target != "/fpga_avoidance") ||
            !control_status_ || age(control_time_) > 3.0 || control_status_->armed ||
            !control_status_->on_ground || control_status_->controller_active ||
            control_status_->reboot_in_progress || control_status_->waypoint_count != 0 ||
            control_status_->control_mode != guided_interfaces::msg::ControlStatus::MODE_IDLE) {
          result.successful = false;
          result.reason = "planner selection requires fresh landed, disarmed, idle state";
          return result;
        }
        auto goal = create_client<Goal>(target + "/set_goal");
        auto motion = create_client<Motion>(target + "/plan_motion");
        if (!goal->service_is_ready() || !motion->service_is_ready()) {
          result.successful = false;
          result.reason = "selected planner interfaces unavailable";
          return result;
        }
        cancel_goal();
        ++coordinate_revision_;
        connect_native(target);
      }
      return result;
    });
    request_sub_ = create_subscription<Request>("/onboard_control/avoidance_request", rclcpp::QoS(10),
      [this](Request::SharedPtr message) { request(*message); });
    control_status_sub_ = create_subscription<guided_interfaces::msg::ControlStatus>(
      "/onboard_control/status", rclcpp::SensorDataQoS(),
      [this](guided_interfaces::msg::ControlStatus::SharedPtr message) {
        // Display coordinates remain local; Odin's real map must not share their TF name.
        preview_frame_ = message->navigation_frame == "map" ? "fcu_local" : "map";
        control_status_ = *message;
        control_time_ = Steady::now();
      });
    odom_sub_ = create_subscription<Odom>(declare_parameter("odometry_topic",
        std::string("/mavros/local_position/odom")), rclcpp::SensorDataQoS(),
      [this](Odom::SharedPtr message) {
        state_ = *message;
        state_time_ = Steady::now();
        if (!coordinates()) { return; }
        Odom output = *message;
        output.header.frame_id = frame_;
        const auto t = local_origin_offset();
        output.pose.pose.position.x -= t[0];
        output.pose.pose.position.y -= t[1];
        output.pose.pose.position.z -= t[2];
        // Twist remains in child_frame_id; the native planner applies its own convention parameter.
        odom_pub_->publish(output);
      });
    status_sub_ = create_subscription<Status>("/extnav/correction_status", rclcpp::QoS(10),
      [this](Status::SharedPtr message) {
        if (status_ && (message->odin_session_id != status_->odin_session_id ||
            message->revision != status_->revision || message->reset_counter != status_->reset_counter ||
            message->correction_valid != status_->correction_valid ||
            message->lever_arm_x_m != status_->lever_arm_x_m ||
            message->lever_arm_y_m != status_->lever_arm_y_m ||
            message->lever_arm_z_m != status_->lever_arm_z_m ||
            message->installation_roll_rad != status_->installation_roll_rad ||
            message->installation_pitch_rad != status_->installation_pitch_rad ||
            message->installation_yaw_rad != status_->installation_yaw_rad)) {
          ++coordinate_revision_;
          cancel_goal();
        }
        status_ = *message;
        status_time_ = Steady::now();
      });
    heartbeat_ = create_wall_timer(std::chrono::seconds(1), [this] {
      Result result = identity(Request{});
      result.ready = ready();
      result.status = Result::WAITING_FOR_INPUT;
      result.detail = reason_;
      result.plan_sequence = 0;
      result_pub_->publish(result);
    });
  }

private:
  // Select one native result stream; old callbacks cannot cross a backend generation.
  void connect_native(const std::string &selected) {
    native_namespace_ = selected;
    planner_session_.clear();
    native_time_ = {};
    last_native_sequence_ = 0;
    motion_limits_ = {};
    const auto generation = ++native_generation_;
    goal_client_ = create_client<Goal>(selected + "/set_goal");
    motion_client_ = create_client<Motion>(selected + "/plan_motion");
    native_sub_ = create_subscription<NativeResult>(selected + "/plan_result", rclcpp::QoS(10),
      [this, generation](NativeResult::SharedPtr message) {
        if (generation == native_generation_) { native_result(*message); }
      });
    if (preview_) {
      cloud_sub_ = create_subscription<Cloud>(selected + "/filtered_cloud", rclcpp::SensorDataQoS(),
        [this, generation](Cloud::SharedPtr message) {
          if (generation == native_generation_) { preview(*message); }
        });
    }
  }

  // b=T is a fixed origin offset, not the current rotated FCU->IMU lever arm.
  // Extnav already publishes the actual FCU center: p_LF=p_OI+b-R_OF*T.
  std::array<double, 3> local_origin_offset() const {
    if (mode_ == "identity" || !status_) { return {0., 0., 0.}; }
    return {status_->lever_arm_x_m, status_->lever_arm_y_m, status_->lever_arm_z_m};
  }

  // Check only the coordinate adapter's inputs; native results diagnose map/input validity.
  bool coordinates() {
    if (!state_ || age(state_time_) > timeout_ || !finite(state_->pose.pose.position)) {
      reason_ = "FCU odometry unavailable or stale";
      return false;
    }
    if (mode_ == "identity") {
      if (state_->header.frame_id != frame_) { reason_ = "odometry frame mismatch"; return false; }
      return true;
    }
    if (state_->header.frame_id != "map" || !status_ ||
        age(status_time_) > kCoordinateStatusTimeoutSeconds ||
        !status_->service_available || !status_->odin_available || status_->interface_version != "2.0" ||
        status_->odin_session_id.empty() || status_->correction_valid ||
        !status_->final_sample_available || status_->final_sample_correction_valid ||
        status_->final_sample_revision != status_->revision ||
        status_->final_sample_odin_session_id != status_->odin_session_id ||
        status_->final_sample_reference_mode != "local_identity_origin") {
      reason_ = "extnav local-identity coordinates unavailable (tag correction unsupported)";
      return false;
    }
    // TODO(task6-tag): Tag coordinates use a different origin and remain unsupported.
    for (double value : local_origin_offset()) {
      if (!std::isfinite(value)) { reason_ = "invalid extnav translation"; return false; }
    }
    return true;
  }

  // Readiness describes adapter capability, not a trajectory execution permit.
  bool ready() {
    if (!coordinates()) { return false; }
    if (!goal_client_->service_is_ready() || !motion_client_->service_is_ready()) {
      reason_ = "native planner interfaces unavailable";
      return false;
    }
    // Native identity heartbeats continue near the goal and while idle, without running searches.
    if (age(native_time_) > 3.) {
      reason_ = "native planner identity heartbeat unavailable";
      return false;
    }
    reason_ = "native planner connected; map validity is reported in each planning result";
    return true;
  }

  // Preserve business identity and assign a monotonic stream sequence at this adapter boundary.
  Result identity(const Request &request) {
    Result result;
    result.header.stamp = now();
    result.header.frame_id = "map";
    result.controller_session = request.controller_session;
    result.task_revision = request.task_revision;
    result.waypoint_index = request.waypoint_index;
    result.request_id = request.request_id;
    result.mode = request.mode;
    result.bridge_session = session_;
    result.planner_session = planner_session_;
    result.coordinate_revision = coordinate_revision_;
    result.plan_sequence = ++result_sequence_;
    result.max_velocity_xy = motion_limits_[0];
    result.max_velocity_z = motion_limits_[1];
    result.max_acceleration_xy = motion_limits_[2];
    result.max_acceleration_z = motion_limits_[3];
    return result;
  }

  // Use the core's resolved scalar/directional limits, including native CLI overrides.
  void remember_limits(const NativeResult &message) {
    motion_limits_ = {message.max_velocity_xy, message.max_velocity_z,
      message.max_acceleration_xy, message.max_acceleration_z};
  }

  // Report only a local interface failure; searches remain under the native node's control.
  void fail(const Request &request, uint8_t status, const std::string &detail) {
    Result result = identity(request);
    // Zero identifies a local delivery failure, not a result from an accepted native goal.
    result.plan_sequence = 0;
    result.ready = ready();
    result.status = status;
    result.detail = detail;
    result_pub_->publish(result);
  }

  // Construct a stable target ID from the controller's existing request identity.
  static std::string target_id(const Request &request) {
    return request.controller_session + ":" + std::to_string(request.task_revision) + ":" +
      std::to_string(request.waypoint_index) + ":" + std::to_string(request.request_id);
  }

  // Cancel only our active goal; a later manual or business goal cannot be cancelled by this ID.
  void cancel_goal() {
    if (!active_) { return; }
    auto goal = std::make_shared<Goal::Request>();
    goal->header.stamp = now();
    goal->header.frame_id = frame_;
    goal->target_id = target_id(*active_);
    goal->active = false;
    if (goal_client_->service_is_ready()) { goal_client_->async_send_request(goal); }
    active_.reset();
    goal_confirmed_ = false;
    pending_native_.reset();
    ++goal_generation_;
  }

  // Set one persistent native goal or query a straight line against the native dual-tree map.
  void request(const Request &message) {
    if (message.mode == Request::CANCEL) {
      // A restarted adapter can still cancel the controller's complete old target identity.
      auto goal = std::make_shared<Goal::Request>();
      goal->header.stamp = now(); goal->header.frame_id = frame_;
      goal->target_id = target_id(message); goal->active = false;
      if (goal_client_->service_is_ready()) { goal_client_->async_send_request(goal); }
      if (active_ && target_id(message) == target_id(*active_)) {
        active_.reset(); goal_confirmed_ = false; pending_native_.reset(); ++goal_generation_;
      }
      return;
    }
    if (!message.request_id) { return; }
    if (message.controller_session.empty() || message.waypoint_index == 0 ||
        message.header.frame_id != "map" || !finite(message.start) || !finite(message.goal)) {
      fail(message, Result::INVALID_INPUT, "invalid target identity or coordinates");
      return;
    }
    if (!ready()) { fail(message, Result::WAITING_FOR_INPUT, reason_); return; }
    const auto t = local_origin_offset();
    if (message.mode == Request::PLAN) {
      active_ = message;
      goal_confirmed_ = false;
      pending_native_.reset();
      const uint64_t generation = ++goal_generation_;
      const std::string id = target_id(message);
      auto goal = std::make_shared<Goal::Request>();
      goal->header.stamp = now();
      goal->header.frame_id = frame_;
      goal->target_id = id;
      goal->active = true;
      goal->goal = message.goal;
      goal->goal.x -= t[0]; goal->goal.y -= t[1]; goal->goal.z -= t[2];
      goal_client_->async_send_request(goal, [this, message, id, generation](rclcpp::Client<Goal>::SharedFuture future) {
        if (!active_ || target_id(*active_) != id || generation != goal_generation_) { return; }
        const auto response = future.get();
        planner_session_ = response->planner_session;
        if (!response->accepted) { fail(message, Result::INVALID_INPUT, response->detail); return; }
        active_goal_revision_ = response->goal_revision;
        goal_confirmed_ = true;
        if (pending_native_) {
          const auto result = *pending_native_;
          pending_native_.reset();
          native_result(result);
        }
      });
      return;
    }
    if (message.mode != Request::CHECK_LINE) {
      fail(message, Result::INVALID_INPUT, "only native goals and straight-line checks are supported");
      return;
    }
    auto query = std::make_shared<Motion::Request>();
    query->header.stamp = now(); query->header.frame_id = frame_;
    query->mode = Motion::Request::CHECK_LINE;
    query->start = message.start; query->goal = message.goal;
    query->start.x -= t[0]; query->start.y -= t[1]; query->start.z -= t[2];
    query->goal.x -= t[0]; query->goal.y -= t[1]; query->goal.z -= t[2];
    const uint64_t revision = coordinate_revision_;
    motion_client_->async_send_request(query,
      [this, message, revision](rclcpp::Client<Motion>::SharedFuture future) {
        if (revision != coordinate_revision_) { return; }
        const auto response = future.get();
        planner_session_ = response->planner_session;
        remember_limits(response->result);
        Result result = identity(message);
        result.header.stamp = response->result.header.stamp;
        result.ready = ready();
        result.status = response->result.status;
        result.detail = response->result.detail;
        result_pub_->publish(result);
      });
  }

  // Forward exact native segments and their search-start time, never a stale RViz Path.
  void native_result(const NativeResult &message) {
    if (message.planner_session.empty() || message.header.frame_id != frame_) { return; }
    native_time_ = Steady::now();
    remember_limits(message);
    if (message.planner_session != planner_session_) {
      if (!planner_session_.empty()) {
        goal_confirmed_ = false;
        pending_native_.reset();
        ++goal_generation_;
      }
      planner_session_ = message.planner_session;
      last_native_sequence_ = 0;
    }
    if (!active_) { return; }
    if (goal_confirmed_ && !message.target_id.empty() && message.goal_revision > active_goal_revision_) {
      // A manual replacement must not leave the old business curve executing without native checks.
      Result revoked = identity(*active_);
      revoked.header.stamp = message.header.stamp;
      revoked.ready = ready();
      revoked.status = Result::NO_PATH;
      revoked.detail = "native target replaced; current waypoint reference revoked";
      result_pub_->publish(revoked);
      return;
    }
    if (message.target_id != target_id(*active_)) { return; }
    // The search can publish before SetGoal's ACK; keep only the latest matching outcome until
    // its session/revision is confirmed. Old coordinate generations cannot acquire a new revision.
    if (!goal_confirmed_) { pending_native_ = message; return; }
    if (message.goal_revision != active_goal_revision_ || message.sequence <= last_native_sequence_) { return; }
    last_native_sequence_ = message.sequence;
    Result result = identity(*active_);
    result.header.stamp = message.header.stamp;
    result.ready = ready();
    result.status = message.status;
    result.detail = message.detail;
    if (result.ready && (message.status == NativeResult::REACH_END ||
        message.status == NativeResult::REACH_HORIZON)) {
      const auto t = local_origin_offset();
      for (const auto &segment : message.segments) {
        guided_interfaces::msg::PolynomialSegment output;
        output.duration = segment.duration;
        output.x = segment.x; output.y = segment.y; output.z = segment.z;
        output.x[0] += t[0]; output.y[0] += t[1]; output.z[0] += t[2];
        result.segments.push_back(output);
      }
    }
    result_pub_->publish(result);
  }

  // Bound optional visualization of the planner's own filtered input; do not build another map.
  void preview(const Cloud &message) {
    constexpr size_t max_points = 10000;  // LAN preview cap, unrelated to map/search resolution.
    if (age(preview_time_) < .5 || !coordinates() || message.header.frame_id != frame_ ||
        preview_pub_->get_subscription_count() == 0 || message.is_bigendian || !message.width ||
        !message.height || !message.point_step || message.data.size() < size_t(message.row_step) * message.height) {
      return;
    }
    std::array<uint32_t, 3> offsets{};
    size_t index = 0;
    for (const std::string name : {"x", "y", "z"}) {
      bool found = false;
      for (const auto &field : message.fields) {
        if (field.name == name && field.datatype == sensor_msgs::msg::PointField::FLOAT32 &&
            field.count == 1 && field.offset + sizeof(float) <= message.point_step) {
          offsets[index] = field.offset;
          found = true;
          break;
        }
      }
      if (!found) { return; }
      ++index;
    }
    const size_t total = size_t(message.width) * message.height;
    const size_t count = std::min(total, max_points);
    Cloud output;
    output.header.stamp = now(); output.header.frame_id = preview_frame_;
    output.height = 1; output.width = count; output.point_step = 12; output.row_step = count * 12;
    output.is_dense = true;
    for (size_t axis = 0; axis < 3; ++axis) {
      sensor_msgs::msg::PointField field;
      field.name = axis == 0 ? "x" : axis == 1 ? "y" : "z";
      field.offset = axis * sizeof(float); field.count = 1;
      field.datatype = sensor_msgs::msg::PointField::FLOAT32;
      output.fields.push_back(field);
    }
    output.data.resize(output.row_step);
    const auto t = local_origin_offset();
    for (size_t point = 0; point < count; ++point) {
      const size_t source = point * total / count;
      const size_t base = source / message.width * message.row_step + source % message.width * message.point_step;
      for (size_t axis = 0; axis < 3; ++axis) {
        float value;
        std::memcpy(&value, message.data.data() + base + offsets[axis], sizeof(value));
        value += t[axis];
        std::memcpy(output.data.data() + point * 12 + axis * sizeof(float), &value, sizeof(value));
      }
    }
    preview_pub_->publish(output);
    preview_time_ = Steady::now();
  }

  std::string mode_, frame_, session_, planner_session_, reason_, native_namespace_;
  std::string preview_frame_{"map"};  // Preserve legacy display frame until map is selected.
  rclcpp::Subscription<guided_interfaces::msg::ControlStatus>::SharedPtr control_status_sub_;
  double timeout_{};
  std::array<double, 4> motion_limits_{};  // Native metadata is the only avoidance limit source.
  bool preview_{}, goal_confirmed_{};
  uint64_t coordinate_revision_{1}, result_sequence_{0}, last_native_sequence_{0};
  uint64_t goal_generation_{0}, active_goal_revision_{0};
  std::optional<Request> active_;
  std::optional<NativeResult> pending_native_;
  std::optional<Odom> state_;
  std::optional<Status> status_;
  std::optional<guided_interfaces::msg::ControlStatus> control_status_;
  uint64_t native_generation_{0};  // Invalidates queued callbacks from the previous result source.
  OnSetParametersCallbackHandle::SharedPtr parameter_callback_;
  Steady::time_point state_time_{}, status_time_{}, preview_time_{}, native_time_{}, control_time_{};
  rclcpp::Publisher<Result>::SharedPtr result_pub_;
  rclcpp::Publisher<Odom>::SharedPtr odom_pub_;
  rclcpp::Publisher<Cloud>::SharedPtr preview_pub_;
  rclcpp::Subscription<Request>::SharedPtr request_sub_;
  rclcpp::Subscription<NativeResult>::SharedPtr native_sub_;
  rclcpp::Subscription<Odom>::SharedPtr odom_sub_;
  rclcpp::Subscription<Status>::SharedPtr status_sub_;
  rclcpp::Subscription<Cloud>::SharedPtr cloud_sub_;
  rclcpp::Client<Goal>::SharedPtr goal_client_;
  rclcpp::Client<Motion>::SharedPtr motion_client_;
  rclcpp::TimerBase::SharedPtr heartbeat_;
};

// Start the adapter without creating any FCU command publisher.
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Bridge>());
  rclcpp::shutdown();
  return 0;
}
