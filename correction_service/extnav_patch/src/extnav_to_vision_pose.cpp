// Single-threaded Odin-to-MAVROS bridge preserving correction 2.0 and FCU reference semantics.
#include "extnav_bridge/math.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/create_timer.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#if EXTNAV_HAS_CORRECTION_API
#include <correction_interfaces/msg/extnav_correction_status.hpp>
#include <correction_interfaces/srv/set_correction.hpp>
#endif

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <iomanip>
#include <locale>
#include <limits>
#include <memory>
#include <optional>
#include <random>
#include <sstream>
#include <string>
#include <utility>

namespace extnav_bridge
{
using Odometry = nav_msgs::msg::Odometry;
using PoseStamped = geometry_msgs::msg::PoseStamped;
using TwistStamped = geometry_msgs::msg::TwistStamped;

// Existing public protocol, discovery cadence, status cadence and installation-angle boundary.
constexpr char kInterfaceVersion[] = "2.0";
constexpr double kDiscoverySeconds = 0.1;
constexpr double kStatusSeconds = 0.5;
constexpr double kZeroInstallationAngle = 1e-9;
// Header timestamps are nanoseconds; active status reports degrees while installation stays radians.
constexpr std::int64_t kNanosecondsPerSecond = 1000000000;
constexpr double kRadiansToDegrees = 180.0 / 3.14159265358979323846;
// The legacy local frame and corrected-world horizontal reference identifiers are wire values.
constexpr char kLocalReference[] = "local_identity_origin";
constexpr char kCorrectedReference[] = "tag_world_xy_local_z";

struct Snapshot
{
  Components value;
  bool correction_valid;
  std::uint64_t revision;
  std::string session;
  std::string reference;
};

/** Read the same Linux monotonic time base used for gaps and raw age in Python. */
double monotonic_seconds()
{
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

/** Format the existing human-readable status text with locale-independent decimals. */
std::string decimal(double value, int precision, bool sign = false)
{
  std::ostringstream stream;
  stream.imbue(std::locale::classic());
  stream << std::fixed << std::setprecision(precision);
  if (sign) {stream << std::showpos;}
  stream << value;
  return stream.str();
}

/** Keep signed header arithmetic, including zero/negative timestamps. */
std::int64_t stamp_nanoseconds(const Odometry & message)
{
  return static_cast<std::int64_t>(message.header.stamp.sec) * kNanosecondsPerSecond +
         message.header.stamp.nanosec;
}

/** Read only the geometry used by the bridge; covariance stays in the original message. */
Components components(const Odometry & message)
{
  const auto & p = message.pose.pose.position;
  const auto & q = message.pose.pose.orientation;
  const auto & v = message.twist.twist.linear;
  return {{p.x, p.y, p.z}, {q.x, q.y, q.z, q.w}, {v.x, v.y, v.z}};
}

/** Write final or corrected geometry without modifying a received raw message. */
void write_pose(geometry_msgs::msg::Pose & pose, const Components & value)
{
  pose.position.x = value.position[0];
  pose.position.y = value.position[1];
  pose.position.z = value.position[2];
  pose.orientation.x = value.quaternion[0];
  pose.orientation.y = value.quaternion[1];
  pose.orientation.z = value.quaternion[2];
  pose.orientation.w = value.quaternion[3];
}

class OdometryBridge : public rclcpp::Node
{
public:
  /** Create the same parameters, topics, service and ROS-clock timers as the Python node. */
  explicit OdometryBridge(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("extnav_to_vision_pose", options)
  {
    const auto vision_topic = declare_parameter("vision_pose_topic", "/mavros/vision_pose/pose");
    const auto pose_topic = declare_parameter("pose_fcu_topic", "/extnav/pose_fcu");
    const auto velocity_topic = declare_parameter("vel_fcu_topic", "/extnav/velocity_fcu");
    const double vision_rate = declare_parameter("vision_rate_hz", 30.0);
    const double control_rate = declare_parameter("ctrl_rate_hz", 100.0);
    const auto odometry_topic = declare_parameter("odom_topic", "/odin1/odometry_highfreq");
    const auto corrected_topic = declare_parameter("corrected_odom_topic", "/odin1/odometry_highfreq_corrected");
    const auto status_topic = declare_parameter("correction_status_topic", "/extnav/correction_status");
    const auto service_name = declare_parameter("set_correction_service", "/extnav/set_correction");
    session_gap_ = declare_parameter("odin_session_gap_seconds", 2.0);
    const double regression = declare_parameter("odin_stamp_regression_seconds", 0.25) * kNanosecondsPerSecond;
    // Python int(float) truncates and rejects nonfinite values; do not tighten other limits.
    if (!std::isfinite(regression)) {throw std::invalid_argument("invalid Odin stamp regression");}
    stamp_regression_ns_ = std::trunc(static_cast<long double>(regression));
    max_translation_ = declare_parameter("max_correction_translation_m", 100.0);
    roll_ = declare_parameter("roll_cam", 0.0);
    pitch_ = declare_parameter("pitch_cam", 0.0);
    yaw_ = declare_parameter("yaw_cam", 0.0);
    const double x = declare_parameter("odin_x", 0.0);
    const double y = declare_parameter("odin_y", 0.0);
    const double z = declare_parameter("odin_z", 0.0);
    if (session_gap_ <= 0.0 || max_translation_ <= 0.0) {
      throw std::invalid_argument("invalid correction/session limit");
    }
    mount_ = installation({x, y, z}, roll_, pitch_, yaw_);
    world_center_supported_ = std::abs(roll_) <= kZeroInstallationAngle &&
      std::abs(pitch_) <= kZeroInstallationAngle && std::abs(yaw_) <= kZeroInstallationAngle;
    if (vision_rate <= 0.0 || control_rate <= 0.0) {
      throw std::invalid_argument("invalid publication rate");
    }
    vision_pub_ = create_publisher<PoseStamped>(vision_topic, 10);
    pose_pub_ = create_publisher<PoseStamped>(pose_topic, 10);
    velocity_pub_ = create_publisher<TwistStamped>(velocity_topic, 10);
    corrected_pub_ = create_publisher<Odometry>(corrected_topic, rclcpp::SensorDataQoS());
    raw_sub_ = create_subscription<Odometry>(odometry_topic, rclcpp::SensorDataQoS(),
      [this](Odometry::ConstSharedPtr message) {on_odometry(*message);});
#if EXTNAV_HAS_CORRECTION_API
    status_pub_ = create_publisher<Status>(status_topic, rclcpp::QoS(1).reliable().transient_local());
    correction_service_ = create_service<SetCorrection>(service_name,
      [this](const SetCorrection::Request::SharedPtr request, SetCorrection::Response::SharedPtr response) {
        set_correction(*request, *response);
      });
#else
    (void)status_topic;
    (void)service_name;
    last_error_ = "correction_interfaces 不可用；保持 identity passthrough";
    RCLCPP_ERROR(get_logger(), "%s", last_error_.c_str());
#endif
    discovery_timer_ = ros_timer(kDiscoverySeconds, [this]() {
        corrected_has_readers_ = corrected_pub_->get_subscription_count() > 0;
      });
    vision_timer_ = ros_timer(1.0 / vision_rate, [this]() {publish_vision();});
    control_timer_ = ros_timer(1.0 / control_rate, [this]() {publish_control();});
    status_timer_ = ros_timer(kStatusSeconds, [this]() {status_timer();});
    RCLCPP_INFO(get_logger(), "Bridge started: raw=%s, corrected=%s, vision=%s@%gHz, control@%gHz",
      odometry_topic.c_str(), corrected_topic.c_str(), vision_topic.c_str(), vision_rate, control_rate);
    RCLCPP_INFO(get_logger(), "cam_rpy=(%g,%g,%g), T=(%.2f,%.2f,%.2f)", roll_, pitch_, yaw_, x, y, z);
    RCLCPP_WARN(get_logger(),
      "TODO(task27-reset-counter): 当前 /mavros/vision_pose/pose 是 PoseStamped，"
      "无法携带 MAVLink estimator reset_counter；本节点仅在修正状态中递增计数。");
    if (!world_center_supported_) {
      RCLCPP_WARN(get_logger(), "Odin installation rotation active; Tag SetCorrection remains unsupported");
    }
  }

private:
  /** Use the node ROS clock, including use_sim_time, rather than wall-clock output timers. */
  template<typename Callback>
  rclcpp::TimerBase::SharedPtr ros_timer(double seconds, Callback callback)
  {
    return rclcpp::create_timer(this, get_clock(), rclcpp::Duration::from_seconds(seconds), callback);
  }

  /** Increment only when the active transform changes; the wire reset field wraps at 8 bits. */
  void advance_revision()
  {
    ++revision_;
    reset_counter_ = static_cast<std::uint8_t>(reset_counter_ + 1);
  }

  /** Preserve identity fallback, snapshot clearing, event text and immediate status publication. */
  void invalidate(const std::string & reason)
  {
    if (correction_valid_) {advance_revision();}
    correction_valid_ = false;
    correction_ = correction(0.0, 0.0, 0.0);
    applied_job_.clear();
    latest_.reset();
    last_event_ = reason;
    last_error_ = reason;
    RCLCPP_WARN(get_logger(), "%s", reason.c_str());
    publish_status();
  }

  /** New session identifiers have the legacy timestamp plus an independent eight-hex suffix. */
  void begin_session(const Odometry & message, const std::string & reason)
  {
    if (correction_valid_) {invalidate(reason + "，旧修正立即失效");}
    std::ostringstream identifier;
    identifier << "odin-" << stamp_nanoseconds(message) << '-' << std::hex << std::setfill('0') <<
      std::setw(8) << random_();
    session_ = identifier.str();
    odin_available_ = true;
    last_event_ = reason + "：session=" + session_;
    last_error_.clear();
    RCLCPP_WARN(get_logger(), "%s", last_event_.c_str());
    publish_status();
  }

  /** Process every raw and freeze output/metadata before optional corrected publication. */
  void on_odometry(const Odometry & message)
  {
    const double received = monotonic_seconds();
    const auto stamp = stamp_nanoseconds(message);
    const auto frame = std::make_pair(message.header.frame_id, message.child_frame_id);
    const double gap = last_raw_time_ > 0.0 ? received - last_raw_time_ : 0.0;
    std::string reason;
    if (!odin_available_ || session_.empty()) {
      reason = "首个 Odin 样本";
    } else if (gap > session_gap_) {
      reason = "Odin 断流 " + decimal(gap, 3) + "s 后恢复";
    } else if (last_stamp_ > 0 && stamp > 0 &&
      static_cast<long double>(stamp) < static_cast<long double>(last_stamp_) - stamp_regression_ns_)
    {
      reason = "Odin 时间戳回退";
    } else if (last_frame_ && frame != *last_frame_) {
      reason = "Odin frame_id 改变";
    }
    if (!reason.empty()) {begin_session(message, reason);}
    last_raw_time_ = received;
    last_stamp_ = stamp;
    last_frame_ = frame;
    odin_available_ = true;
    ++raw_messages_;
    bool sample_valid = correction_valid_;
    auto sample_revision = revision_;
    auto sample_session = session_;
    // Retain the pre-invalidation transform for this callback's independent corrected stream.
    const auto frozen_correction = sample_valid ? std::optional<Correction>(correction_) : std::nullopt;
    const auto raw = components(message);
    std::optional<Components> corrected;
    try {
      if (sample_valid) {corrected = planar_components(raw, *frozen_correction);}
    } catch (const std::exception & error) {
      invalidate(std::string("修正计算异常，退回 identity：") + error.what());
      sample_valid = false;
      sample_revision = revision_;
      sample_session = session_;
    }
    std::optional<Snapshot> output;
    try {
      const auto & w = message.twist.twist.angular;
      output = Snapshot{
        fcu_components(raw, corrected ? &*corrected : nullptr, {w.x, w.y, w.z}, mount_, sample_valid),
        sample_valid, sample_revision, sample_session, sample_valid ? kCorrectedReference : kLocalReference};
    } catch (const std::exception & error) {
      invalidate(std::string("飞控中心转换异常，停止重复发布旧样本：") + error.what());
    }
    latest_ = std::move(output);
    if (corrected_has_readers_) {
      if (corrected) {
        Odometry published = message;
        write_pose(published.pose.pose, *corrected);
        published.twist.twist.linear.x = corrected->velocity[0];
        published.twist.twist.linear.y = corrected->velocity[1];
        published.twist.twist.linear.z = corrected->velocity[2];
        published.pose.covariance = rotate_covariance(message.pose.covariance, frozen_correction->pose_transform);
        published.twist.covariance = rotate_covariance(message.twist.covariance, frozen_correction->twist_transform);
        corrected_pub_->publish(published);
      } else {
        // rclcpp serializes a const message; no Python-style deep copy or raw mutation is needed.
        corrected_pub_->publish(message);
      }
      ++corrected_messages_;
    }
  }

#if EXTNAV_HAS_CORRECTION_API
  using Status = correction_interfaces::msg::ExtnavCorrectionStatus;
  using SetCorrection = correction_interfaces::srv::SetCorrection;

  /** Keep the exact CAS/clear/apply validation order, including NumPy's idempotent tolerance. */
  void set_correction(const SetCorrection::Request & request, SetCorrection::Response & response)
  {
    response.accepted = false;
    response.applied = false;
    response.revision = revision_;
    response.odin_session_id = session_;
    response.message = "请求被拒绝";
    if (request.valid && request.job_id == applied_job_ && request.odin_session_id == session_ &&
      correction_valid_ && idempotent_close(request.correction_x_m, correction_.x) &&
      idempotent_close(request.correction_y_m, correction_.y) &&
      idempotent_close(request.correction_yaw_rad, correction_.yaw))
    {
      response.accepted = true;
      response.applied = true;
      response.message = "同一 job 已幂等应用";
      return;
    }
    if (request.expected_revision != revision_) {
      response.message = "revision 冲突：期望 " + std::to_string(request.expected_revision) +
        "，当前 " + std::to_string(revision_);
      return;
    }
    if (!request.valid) {
      if (!request.odin_session_id.empty() && request.odin_session_id != session_) {
        response.message = "清除请求的 Odin session 不匹配";
        return;
      }
      if (correction_valid_) {advance_revision();}
      correction_valid_ = false;
      correction_ = correction(0.0, 0.0, 0.0);
      applied_job_.clear();
      latest_.reset();
      last_event_ = "收到显式清除请求，identity passthrough";
      last_error_.clear();
      response.accepted = true;
      response.revision = revision_;
      response.message = last_event_;
      publish_status();
      return;
    }
    if (!odin_available_ || session_.empty()) {
      response.message = "Odin 当前不可用";
      return;
    }
    if (!world_center_supported_) {
      response.message = "当前 extnav 安装角非零，task29 飞控中心有效分支尚未重新推导";
      return;
    }
    if (request.odin_session_id != session_) {
      response.message = "候选 Odin session 已失效";
      return;
    }
    for (double value : {request.position_std_m, request.yaw_std_rad,
        request.reprojection_error_px, request.odom_match_error_ms})
    {
      if (!std::isfinite(value) || value < 0.0) {
        response.message = "候选质量指标无效";
        return;
      }
    }
    if (request.sample_count == 0) {
      response.message = "候选样本数为零";
      return;
    }
    if (!std::isfinite(request.correction_x_m) || !std::isfinite(request.correction_y_m) ||
      !std::isfinite(request.correction_yaw_rad))
    {
      response.message = "候选修正含非有限值";
      return;
    }
    if (std::hypot(request.correction_x_m, request.correction_y_m) > max_translation_) {
      response.message = "候选水平修正超过 extnav 绝对上限";
      return;
    }
    correction_ = correction(request.correction_x_m, request.correction_y_m,
      std::atan2(std::sin(request.correction_yaw_rad), std::cos(request.correction_yaw_rad)));
    correction_valid_ = true;
    applied_job_ = request.job_id;
    latest_.reset();
    advance_revision();
    last_event_ = "已应用 job=" + applied_job_ + " revision=" + std::to_string(revision_) +
      " x=" + decimal(correction_.x, 4, true) + " y=" + decimal(correction_.y, 4, true) +
      " yaw=" + decimal(correction_.yaw * kRadiansToDegrees, 4, true) + "deg";
    last_error_.clear();
    RCLCPP_WARN(get_logger(), "%s", last_event_.c_str());
    response.accepted = true;
    response.applied = true;
    response.revision = revision_;
    response.odin_session_id = session_;
    response.message = last_event_;
    publish_status();
  }
#endif

  /** Apply the same periodic timeout transition before publishing authoritative status. */
  void status_timer()
  {
    if (last_raw_time_ > 0.0) {
      const double age = monotonic_seconds() - last_raw_time_;
      if (age > session_gap_ && odin_available_) {
        odin_available_ = false;
        session_.clear();
        latest_.reset();
        if (correction_valid_) {
          invalidate("Odin 数据超时 " + decimal(age, 3) + "s，旧修正立即失效");
        } else {
          last_event_ = "Odin 数据超时 " + decimal(age, 3) + "s，identity 等待恢复";
        }
      }
    }
    publish_status();
  }

  /** Preserve active state and independently frozen final-sample identity in correction 2.0. */
  void publish_status()
  {
#if EXTNAV_HAS_CORRECTION_API
    Status message;
    message.header.stamp = now();
    message.interface_version = kInterfaceVersion;
    message.service_available = true;
    message.odin_available = odin_available_;
    message.odin_session_id = session_;
    message.correction_valid = correction_valid_;
    message.revision = revision_;
    message.reset_counter = reset_counter_;
    message.correction_x_m = correction_.x;
    message.correction_y_m = correction_.y;
    message.correction_yaw_deg = correction_.yaw * kRadiansToDegrees;
    message.applied_job_id = applied_job_;
    message.horizontal_reference_mode = correction_valid_ ? kCorrectedReference : kLocalReference;
    message.lever_arm_x_m = mount_.lever[0];
    message.lever_arm_y_m = mount_.lever[1];
    message.lever_arm_z_m = mount_.lever[2];
    message.installation_roll_rad = roll_;
    message.installation_pitch_rad = pitch_;
    message.installation_yaw_rad = yaw_;
    message.final_pose_messages = final_pose_messages_;
    message.final_sample_available = latest_.has_value();
    if (latest_) {
      message.final_sample_correction_valid = latest_->correction_valid;
      message.final_sample_revision = latest_->revision;
      message.final_sample_odin_session_id = latest_->session;
      message.final_sample_reference_mode = latest_->reference;
    }
    message.raw_age_s = last_raw_time_ > 0.0 ?
      std::max(0.0, monotonic_seconds() - last_raw_time_) : std::numeric_limits<double>::infinity();
    message.raw_messages = raw_messages_;
    message.corrected_messages = corrected_messages_;
    message.last_event = last_event_;
    message.last_error = last_error_;
    status_pub_->publish(message);
#endif
  }

  /** Build a final pose with the existing publication-time stamp and odom frame. */
  PoseStamped pose_message(const builtin_interfaces::msg::Time & stamp) const
  {
    PoseStamped message;
    message.header.stamp = stamp;
    message.header.frame_id = "odom";
    write_pose(message.pose, latest_->value);
    return message;
  }

  /** Vision always uses the same frozen FCU geometry as the auxiliary outputs. */
  void publish_vision()
  {
    if (!latest_) {return;}
    vision_pub_->publish(pose_message(now()));
    ++final_pose_messages_;
  }

  /** Keep one shared control stamp and count one dispatch for the two legacy outputs. */
  void publish_control()
  {
    if (!latest_) {return;}
    const bool pose_needed = pose_pub_->get_subscription_count() > 0;
    const bool velocity_needed = velocity_pub_->get_subscription_count() > 0;
    if (pose_needed || velocity_needed) {
      const builtin_interfaces::msg::Time stamp = now();
      if (pose_needed) {pose_pub_->publish(pose_message(stamp));}
      if (velocity_needed) {
        TwistStamped message;
        message.header.stamp = stamp;
        message.header.frame_id = "odom";
        message.twist.linear.x = latest_->value.velocity[0];
        message.twist.linear.y = latest_->value.velocity[1];
        message.twist.linear.z = latest_->value.velocity[2];
        velocity_pub_->publish(message);
      }
    }
    // Preserve the original one-count-per-valid-control-tick status contract.
    ++final_pose_messages_;
  }

  Installation mount_{};
  Correction correction_ = correction(0.0, 0.0, 0.0);
  double roll_ = 0.0, pitch_ = 0.0, yaw_ = 0.0;
  double session_gap_ = 2.0, max_translation_ = 100.0;
  long double stamp_regression_ns_ = 0.0;
  bool world_center_supported_ = true, corrected_has_readers_ = false;
  bool odin_available_ = false, correction_valid_ = false;
  double last_raw_time_ = 0.0;
  std::int64_t last_stamp_ = 0;
  std::optional<std::pair<std::string, std::string>> last_frame_;
  std::string session_, applied_job_;
  std::string last_event_ = "extnav 启动，identity passthrough", last_error_;
  std::optional<Snapshot> latest_;
  std::uint64_t revision_ = 0, raw_messages_ = 0, corrected_messages_ = 0, final_pose_messages_ = 0;
  std::uint8_t reset_counter_ = 0;
  std::mt19937 random_{std::random_device{}()};
  rclcpp::Publisher<PoseStamped>::SharedPtr vision_pub_, pose_pub_;
  rclcpp::Publisher<TwistStamped>::SharedPtr velocity_pub_;
  rclcpp::Publisher<Odometry>::SharedPtr corrected_pub_;
  rclcpp::Subscription<Odometry>::SharedPtr raw_sub_;
  rclcpp::TimerBase::SharedPtr discovery_timer_, vision_timer_, control_timer_, status_timer_;
#if EXTNAV_HAS_CORRECTION_API
  rclcpp::Publisher<Status>::SharedPtr status_pub_;
  rclcpp::Service<SetCorrection>::SharedPtr correction_service_;
#endif
};
}  // namespace extnav_bridge

/** Serial execution preserves the original callback-level transaction and snapshot atomicity. */
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<extnav_bridge::OdometryBridge>();
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    executor.remove_node(node);
    node.reset();
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("extnav_to_vision_pose"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
