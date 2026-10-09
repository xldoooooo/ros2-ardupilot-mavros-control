// Independent cloud evidence and coordinate adapter. No FCU command/setpoint publishers exist here.
#include "avoidance_bridge/observed_map.hpp"
#include "avoidance_bridge/input_contract.hpp"
#include <correction_interfaces/msg/extnav_correction_status.hpp>
#include <guided_interfaces/msg/avoidance_request.hpp>
#include <guided_interfaces/msg/avoidance_result.hpp>
#include <path_planning/srv/plan_motion.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <chrono>
#include <cstring>
#include <deque>
#include <optional>

using Steady=std::chrono::steady_clock;
using Request=guided_interfaces::msg::AvoidanceRequest;
using Result=guided_interfaces::msg::AvoidanceResult;
using Service=path_planning::srv::PlanMotion;
using Status=correction_interfaces::msg::ExtnavCorrectionStatus;
using Cloud=sensor_msgs::msg::PointCloud2;
using Odom=nav_msgs::msg::Odometry;
using avoidance_bridge::Vec;
static double elapsed(Steady::time_point t) { return std::chrono::duration<double>(Steady::now()-t).count(); }
static double stamp(const builtin_interfaces::msg::Time &t) { return t.sec+1e-9*t.nanosec; }
static bool finite(const Vec &p) { return std::isfinite(p[0])&&std::isfinite(p[1])&&std::isfinite(p[2]); }
static Vec vec(const geometry_msgs::msg::Point &p) { return {p.x,p.y,p.z}; }
static size_t map_capacity(rclcpp::Node &node) {
  const int count=node.declare_parameter("max_cells",300000);
  if(count<1||count>1000000) throw std::invalid_argument("max_cells must be in [1,1000000]");
  return size_t(count);
}

class Bridge:public rclcpp::Node {
public:
  Bridge():Node("avoidance_bridge"),map_(declare_parameter("voxel_size",.1),declare_parameter("map_radius",20.),
      map_capacity(*this)) {
    mode_=declare_parameter("coordinate_mode",std::string("extnav"));
    if(mode_!="extnav"&&mode_!="identity") throw std::invalid_argument("coordinate_mode must be extnav or identity");
    frame_=mode_=="extnav"?"odom":"map";
    timeout_=declare_parameter("input_timeout",.5);
    preview_=declare_parameter("preview_enabled",false);
    max_input_=declare_parameter("max_cloud_points",500000);
    if(!std::isfinite(timeout_) || timeout_<=0 || max_input_<=0 || max_input_>1000000)
      throw std::invalid_argument("invalid map/input bounds");
    session_=std::to_string(Steady::now().time_since_epoch().count());
    result_pub_=create_publisher<Result>("/onboard_control/avoidance_result",rclcpp::QoS(10));
    cloud_pub_=create_publisher<Cloud>("/avoidance/map",rclcpp::SensorDataQoS());
    preview_pub_=create_publisher<Cloud>("/onboard_control/obstacle_preview",rclcpp::SensorDataQoS());
    client_=create_client<Service>("/avoidance/plan_motion");
    request_sub_=create_subscription<Request>("/onboard_control/avoidance_request",rclcpp::QoS(10),
      [this](Request::SharedPtr r){
        if(r->request_id==0) {Result reply;identity(reply,*r);reply.ready=ready();reply.status=reply.ready?9:6;
          reply.detail=reason_;result_pub_->publish(reply);return;}
        pending_=*r; dispatch();
      });
    raw_sub_=create_subscription<Odom>(declare_parameter("odometry_topic",std::string("/odin1/odometry_highfreq")),
      rclcpp::SensorDataQoS(),[this](Odom::SharedPtr r){
        if(!avoidance_bridge::valid_stamp(r->header.stamp)||r->header.frame_id!=frame_||!finite(vec(r->pose.pose.position))) {
          raw_valid_=false;map_valid_=false;reason_="invalid raw odometry stamp/frame/position";return;}
        const int64_t source=avoidance_bridge::source_ns(r->header.stamp);
        if(last_raw_source_ && source<=*last_raw_source_) {
          if(source<*last_raw_source_) {last_raw_source_=source;invalidate("odometry clock reset");}
          else {raw_valid_=false;map_valid_=false;reason_="duplicate raw odometry source stamp";}
          return;
        }
        last_raw_source_=source;raw_valid_=true;
        raw_.emplace_back(*r,Steady::now()); if(raw_.size()>100) raw_.pop_front();
      });
    pose_sub_=create_subscription<geometry_msgs::msg::PoseStamped>("/mavros/local_position/pose",rclcpp::SensorDataQoS(),
      [this](geometry_msgs::msg::PoseStamped::SharedPtr p){pose_=*p;pose_time_=Steady::now();});
    status_sub_=create_subscription<Status>("/extnav/correction_status",rclcpp::QoS(10),
      [this](Status::SharedPtr s){
        if(status_ && (s->odin_session_id!=status_->odin_session_id || s->revision!=status_->revision ||
            s->correction_valid!=status_->correction_valid || s->reset_counter!=status_->reset_counter ||
            s->lever_arm_x_m!=status_->lever_arm_x_m || s->lever_arm_y_m!=status_->lever_arm_y_m ||
            s->lever_arm_z_m!=status_->lever_arm_z_m || s->installation_roll_rad!=status_->installation_roll_rad ||
            s->installation_pitch_rad!=status_->installation_pitch_rad || s->installation_yaw_rad!=status_->installation_yaw_rad))
          invalidate("extnav coordinate changed");
        status_=*s;status_time_=Steady::now();
      });
    cloud_sub_=create_subscription<Cloud>(declare_parameter("cloud_topic",std::string("/odin1/cloud_slam")),
      rclcpp::SensorDataQoS(),[this](Cloud::SharedPtr c){cloud(*c);});
    heartbeat_=create_wall_timer(std::chrono::seconds(1),[this]{
      Result r; identity(r,Request{});r.ready=ready();r.status=r.ready?9:6;r.detail=reason_;result_pub_->publish(r);
    });
    watchdog_=create_wall_timer(std::chrono::milliseconds(50),[this]{
      if(active_ && elapsed(active_time_)>.8) {
        if(retry_timer_) retry_timer_->cancel();
        auto request=*active_;client_->remove_pending_request(active_id_);active_.reset();++generation_;
        fail(request,7,"planning service deadline");dispatch();
      }
    });
  }
private:
  // Coordinate state changes invalidate accumulated evidence and every in-flight result.
  void invalidate(const std::string &reason) {
    map_.clear();raw_.clear();raw_valid_=false;map_valid_=false;reason_=reason;++coordinate_revision_;++generation_;
  }
  Vec translation() const {
    if(mode_=="identity" || !status_) return {0,0,0};
    return {status_->lever_arm_x_m,status_->lever_arm_y_m,status_->lever_arm_z_m};
  }
  bool coordinates() {
    if(!raw_valid_||raw_.empty()||elapsed(raw_.back().second)>timeout_) {reason_="raw odometry invalid or stale";return false;}
    if(mode_=="identity") return true;
    if(!status_||elapsed(status_time_)>timeout_||!status_->service_available||!status_->odin_available||
       status_->interface_version!="2.0"||status_->odin_session_id.empty()||status_->correction_valid||
       !status_->final_sample_available||status_->final_sample_correction_valid||
       status_->final_sample_revision!=status_->revision||status_->final_sample_odin_session_id!=status_->odin_session_id||
       status_->final_sample_reference_mode!="local_identity_origin"||!std::isfinite(status_->raw_age_s)||status_->raw_age_s>timeout_) {
      reason_="extnav baseline unavailable (valid tag correction unsupported)";return false;
    }
    if(raw_.empty()||!pose_||elapsed(raw_.back().second)>timeout_||elapsed(pose_time_)>timeout_||
       std::abs(std::chrono::duration<double>(raw_.back().second-pose_time_).count())>.1) {
      reason_="raw/FCU receive pairing stale";return false;
    }
    // Match extnav's q_IO*q_raw*q_IO^-1; never infer translation from one pose difference.
    const auto &q=raw_.back().first.pose.pose.orientation;
    tf2::Quaternion raw(q.x,q.y,q.z,q.w), install, r,p,y;
    if(!std::isfinite(raw.length2())||raw.length2()<1e-12) return false;
    r.setRPY(status_->installation_roll_rad,0,0);p.setRPY(0,status_->installation_pitch_rad,0);
    y.setRPY(0,0,status_->installation_yaw_rad);install=r*p*y;
    raw.normalize();tf2::Matrix3x3 rotation(install*raw*install.inverse());
    const Vec t=translation(); if(!finite(t)) return false;
    tf2::Vector3 arm=rotation*tf2::Vector3(t[0],t[1],t[2]);
    Vec expected=vec(raw_.back().first.pose.pose.position),actual=vec(pose_->pose.position);
    double error=0;
    for(size_t i=0;i<3;++i) { expected[i]+=t[i]-arm[int(i)];error+=std::pow(expected[i]-actual[i],2); }
    if(!finite(expected)||!finite(actual)||std::sqrt(error)>.25) {reason_="FCU center consistency exceeds 0.25 m";return false;}
    return true;
  }
  bool ready() {
    if(!coordinates()) return false;
    if(!map_valid_||elapsed(cloud_time_)>timeout_||map_.saturated()) {reason_="map stale, empty or saturated (unknown)";return false;}
    if(!client_->service_is_ready()) {reason_="planner service unavailable";return false;}
    reason_="ready; discretely observed ray cells only";return true;
  }
  // Source stamps pair cloud and raw in one device clock; reception age is a separate freshness gate.
  void cloud(const Cloud &c) {
    const auto received=Steady::now();
    std::array<uint32_t,3> offsets{};
    if(!avoidance_bridge::valid_stamp(c.header.stamp)||c.header.frame_id!=frame_ ||
       uint64_t(c.width)*c.height>uint64_t(max_input_)||!avoidance_bridge::xyz_schema(c,offsets)) {
      map_valid_=false;reason_="invalid registered cloud schema/frame";return;
    }
    const int64_t source_ns=avoidance_bridge::source_ns(c.header.stamp);
    if(last_cloud_source_ && source_ns<=*last_cloud_source_) {
      if(source_ns<*last_cloud_source_) {last_cloud_source_=source_ns;invalidate("cloud source clock reset");}
      else {map_valid_=false;reason_="duplicate raw cloud source stamp";}
      return;
    }
    last_cloud_source_=source_ns;
    if(c.width==0||c.height==0) {map_valid_=false;reason_="latest raw cloud empty (unknown)";return;}
    if(!coordinates()) {map_valid_=false;return;}
    if(elapsed(cloud_time_)<.5 && map_valid_) return; // Idle map and preview cost are bounded to 2 Hz.
    const double source=stamp(c.header.stamp);
    const Odom *origin=nullptr;double best=.100000001;
    for(const auto &entry:raw_) {
      double d=std::abs(stamp(entry.first.header.stamp)-source);
      if(d<best && elapsed(entry.second)<=timeout_ && entry.first.header.frame_id==frame_) {best=d;origin=&entry.first;}
    }
    if(!origin) {map_valid_=false;reason_="cloud/raw source stamps unmatched";return;}
    std::vector<Vec> points;points.reserve(size_t(c.width)*c.height);
    for(uint32_t row=0;row<c.height;++row) for(uint32_t col=0;col<c.width;++col) {
      Vec point;for(size_t i=0;i<3;++i) {float value;std::memcpy(&value,&c.data[size_t(row)*c.row_step+size_t(col)*c.point_step+offsets[i]],4);point[i]=value;}
      if(finite(point)) points.push_back(point);
    }
    if(points.empty()) {map_valid_=false;reason_="latest scan has no finite endpoints (unknown)";return;}
    map_.update(vec(origin->pose.pose.position),points);cloud_time_=received;cloud_stamp_=c.header.stamp;
    map_valid_=!map_.saturated();
    if(!map_valid_) {reason_="map capacity exhausted (unknown)";return;}
    if(elapsed(received)>timeout_) {map_valid_=false;reason_="scan expired during map construction";return;}
    auto occupied=map_.occupied();auto output=make_cloud(occupied,frame_,cloud_stamp_,{0,0,0},occupied.size());
    cloud_pub_->publish(output);
    // Optional LAN visualization is generated only for actual subscribers, <=10000 XYZ points/2 Hz.
    if(preview_&&preview_pub_->get_subscription_count()+preview_pub_->get_intra_process_subscription_count()>0)
      preview_pub_->publish(make_cloud(occupied,"map",now(),translation(),10000));
  }
  Cloud make_cloud(const std::vector<Vec> &points,const std::string &frame,const builtin_interfaces::msg::Time &time,
                   const Vec &offset,size_t cap) {
    Cloud c;c.header.frame_id=frame;c.header.stamp=time;
    sensor_msgs::PointCloud2Modifier modifier(c);modifier.setPointCloud2Fields(3,"x",1,sensor_msgs::msg::PointField::FLOAT32,
      "y",1,sensor_msgs::msg::PointField::FLOAT32,"z",1,sensor_msgs::msg::PointField::FLOAT32);
    const size_t n=std::min(points.size(),cap);modifier.resize(n);
    sensor_msgs::PointCloud2Iterator<float> x(c,"x"),y(c,"y"),z(c,"z");
    for(size_t i=0;i<n;++i,++x,++y,++z) {
      const auto &p=points[i*points.size()/n];*x=float(p[0]+offset[0]);*y=float(p[1]+offset[1]);*z=float(p[2]+offset[2]);
    }
    return c;
  }
  void identity(Result &r,const Request &q) {
    r.header.stamp=now();r.header.frame_id="map";r.controller_session=q.controller_session;
    r.task_revision=q.task_revision;r.waypoint_index=q.waypoint_index;r.request_id=q.request_id;r.mode=q.mode;
    r.bridge_session=session_;r.planner_session=planner_session_;r.coordinate_revision=coordinate_revision_;
  }
  void fail(const Request &q,uint8_t status,const std::string &detail) {
    Result r;identity(r,q);r.ready=ready();r.status=status;r.detail=detail;result_pub_->publish(r);
  }
  // Discrete centerline observation is required; absence of obstacle points never implies free space.
  bool observed_line(Vec a,Vec b) const {
    if(!finite(a)||!finite(b)) return false;
    double length=0;for(size_t i=0;i<3;++i) length+=std::pow(b[i]-a[i],2);
    if(!std::isfinite(length)||length>25000000) return false;
    const size_t n=size_t(std::ceil(std::sqrt(length)/(map_.voxel()/2)));
    if(n>100000) return false;
    for(size_t j=0;j<=n;++j) {Vec p;for(size_t i=0;i<3;++i) p[i]=a[i]+(b[i]-a[i])*(n?double(j)/n:0);if(!map_.observed(p)) return false;}
    return true;
  }
  template<class Segments> bool observed_curve(const Segments &segments,double skip) const {
    double preceding=0;size_t total=0;
    if(segments.empty()||!std::isfinite(skip)||skip<0) return false;
    Vec last{};bool has_last=false;
    for(const auto &s:segments) {
      if(!std::isfinite(s.duration)||s.duration<=0||s.duration>60) return false;
      const double begin=std::max(0.0,skip-preceding);preceding+=s.duration;
      if(begin>s.duration) continue;
      const size_t n=size_t(std::ceil((s.duration-begin)/.01));total+=n+1;if(total>100000) return false;
      for(size_t j=0;j<=n;++j) {
        const double t=begin+(s.duration-begin)*(n?double(j)/n:0);Vec p{};
        for(size_t i=0;i<3;++i) {const auto &c=i==0?s.x:i==1?s.y:s.z;p[i]=((c[3]*t+c[2])*t+c[1])*t+c[0];}
        if(!finite(p)||!map_.observed(p)||(has_last&&!observed_line(last,p))) return false;
        last=p;has_last=true;
      }
    }
    return has_last;
  }
  void dispatch() {
    if(active_||!pending_) return;
    Request q=*pending_;pending_.reset();
    if(!q.request_id||q.controller_session.empty()||q.waypoint_index==0||q.header.frame_id!="map"||q.mode<1||q.mode>3||
       !finite(vec(q.start))||!finite(vec(q.goal))||!std::isfinite(q.velocity.x)||!std::isfinite(q.velocity.y)||!std::isfinite(q.velocity.z)) {
      fail(q,5,"invalid request identity/frame/mode/state");return;}
    const double age=now().seconds()-stamp(q.header.stamp);
    if(!std::isfinite(age)||age<-.05||age>timeout_) {fail(q,10,"controller execution-state stamp stale");return;}
    if(!ready()) {fail(q,6,reason_);return;}
    auto call=std::make_shared<Service::Request>();call->header.stamp=now();call->header.frame_id=frame_;
    call->mode=q.mode;call->start=q.start;call->velocity=q.velocity;call->goal=q.goal;call->trajectory_elapsed=q.trajectory_elapsed;
    const Vec t=translation();call->start.x-=t[0];call->start.y-=t[1];call->start.z-=t[2];
    call->goal.x-=t[0];call->goal.y-=t[1];call->goal.z-=t[2];
    for(const auto &s:q.segments) {path_planning::msg::PolynomialSegment p;p.duration=s.duration;p.x=s.x;p.y=s.y;p.z=s.z;
      p.x[0]-=t[0];p.y[0]-=t[1];p.z[0]-=t[2];call->segments.push_back(p);}
    if(q.mode==2&&!observed_line(vec(call->start),vec(call->goal))) {fail(q,6,"straight line enters unobserved voxel (unknown)");return;}
    if(q.mode==3&&!observed_curve(call->segments,call->trajectory_elapsed)) {fail(q,6,"remaining curve enters unobserved voxel (unknown)");return;}
    active_=q;active_time_=Steady::now();const uint64_t generation=generation_,serial=++active_serial_;
    send(call,q,t,generation,serial);
  }
  // Reconcile DDS cloud/service ordering within the ORIGINAL request deadline and identity.
  // A PLAN candidate must be validated against the latest map; retries never rerun its clock.
  void send(const Service::Request::SharedPtr &call,const Request &q,const Vec &t,uint64_t generation,uint64_t serial,
            std::shared_ptr<path_planning::msg::PlanResult> candidate=nullptr,std::string candidate_session="") {
    if(!active_||active_serial_!=serial) return;
    if(elapsed(active_time_)>=.8 || now().seconds()-stamp(q.header.stamp)>timeout_) {
      active_.reset();fail(q,7,"original request deadline expired during map reconciliation");dispatch();return;
    }
    if(pending_||generation!=generation_||!ready()) {
      active_.reset();fail(q,10,"request superseded or coordinate/map/service changed during planning");dispatch();return;
    }
    const auto handle=client_->async_send_request(call,[this,call,q,t,generation,serial,candidate,candidate_session](rclcpp::Client<Service>::SharedFuture future) {
      if(!active_||active_serial_!=serial) return;
      if(elapsed(active_time_)>=.8 || now().seconds()-stamp(q.header.stamp)>timeout_) {
        active_.reset();fail(q,7,"original request deadline expired during map reconciliation");dispatch();return;
      }
      if(pending_||generation!=generation_ || !ready()) {
        active_.reset();fail(q,10,"request superseded or coordinate/map/service changed during planning");dispatch();return;}
      auto response=future.get();planner_session_=response->planner_session;Result r;identity(r,q);r.ready=true;
      r.status=response->result.status;r.detail=response->result.detail;
      if(response->result.header.frame_id!=frame_||planner_session_.empty()) {r.status=11;r.detail="planner frame/session invalid";}
      else if(candidate&&!candidate_session.empty()&&candidate_session!=planner_session_) {r.status=10;r.detail="planner restarted during candidate validation";}
      else if(response->result.cloud_stamp!=cloud_stamp_ && r.status!=5&&r.status!=7&&r.status!=8&&r.status!=11) {
        auto next_candidate=candidate;auto next_session=candidate_session;
        if(q.mode==1&&r.status==2&&!candidate) {
          next_candidate=std::make_shared<path_planning::msg::PlanResult>(response->result);next_session=planner_session_;
          call->mode=Service::Request::VALIDATE_TRAJECTORY;call->segments=next_candidate->segments;call->trajectory_elapsed=0;
        }
        // Let the planner's single-thread executor consume queued map samples before retrying.
        retry_timer_=create_wall_timer(std::chrono::milliseconds(50),[this,call,q,t,generation,serial,next_candidate,next_session]{
          if(!active_||active_serial_!=serial) return;
          retry_timer_->cancel();send(call,q,t,generation,serial,next_candidate,next_session);
        });
        return;
      }
      else if(r.status==2&&q.mode==1) {
        const auto &segments=candidate?candidate->segments:response->result.segments;
        if(!observed_curve(segments,0)) {r.status=6;r.detail="planned curve enters unobserved voxel (unknown)";}
        else for(const auto &s:segments) {guided_interfaces::msg::PolynomialSegment p;p.duration=s.duration;p.x=s.x;p.y=s.y;p.z=s.z;
          p.x[0]+=t[0];p.y[0]+=t[1];p.z[0]+=t[2];r.segments.push_back(p);}
      }
      else if(r.status==1) {r.status=3;r.detail="partial horizon trajectory is not an executable waypoint result";}
      active_.reset();
      result_pub_->publish(r);dispatch();
    });active_id_=handle.request_id;
  }
  avoidance_bridge::ObservedMap map_;
  std::string mode_,frame_,session_,planner_session_,reason_="waiting for observations";
  double timeout_;int max_input_;bool preview_,map_valid_=false,raw_valid_=false;
  uint64_t coordinate_revision_=1,generation_=0,active_serial_=0;int64_t active_id_=0;
  std::optional<int64_t> last_raw_source_,last_cloud_source_;
  std::deque<std::pair<Odom,Steady::time_point>> raw_;
  std::optional<Status> status_;std::optional<geometry_msgs::msg::PoseStamped> pose_;
  std::optional<Request> active_,pending_;
  Steady::time_point status_time_{},pose_time_{},cloud_time_{},active_time_{};
  builtin_interfaces::msg::Time cloud_stamp_;
  rclcpp::Publisher<Result>::SharedPtr result_pub_;
  rclcpp::Publisher<Cloud>::SharedPtr cloud_pub_,preview_pub_;
  rclcpp::Subscription<Request>::SharedPtr request_sub_;
  rclcpp::Subscription<Cloud>::SharedPtr cloud_sub_;
  rclcpp::Subscription<Odom>::SharedPtr raw_sub_;
  rclcpp::Subscription<Status>::SharedPtr status_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Client<Service>::SharedPtr client_;
  rclcpp::TimerBase::SharedPtr heartbeat_,watchdog_,retry_timer_;
};
int main(int argc,char **argv) {
  rclcpp::init(argc,argv);rclcpp::spin(std::make_shared<Bridge>());rclcpp::shutdown();return 0;
}
