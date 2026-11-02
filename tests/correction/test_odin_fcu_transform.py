"""Exercise the native mounting helper against physical motion and numerical Jacobians."""

import subprocess
from pathlib import Path


# Standalone message-shaped values keep this numerical test independent of ROS discovery.
CPP = r'''
#include "fcu_odometry.h"
#include <array>
#include <iostream>
#include <string>

struct Vector { double x=0, y=0, z=0; };
struct Quaternion { double x=0, y=0, z=0, w=1; };
struct Pose { Vector position; Quaternion orientation; };
struct Twist { Vector linear, angular; };
struct Message {
    struct { Pose pose; std::array<double,36> covariance{}; } pose;
    struct { Twist twist; std::array<double,36> covariance{}; } twist;
    std::string child_frame_id="imu";
};
using V = Eigen::Vector3d;
using M = Eigen::Matrix3d;
using M6 = Eigen::Matrix<double,6,6>;
using Row6 = Eigen::Matrix<double,6,6,Eigen::RowMajor>;

/** Populate the original SDK convention from independent physical IMU values. */
Message input(const V& p, const M& r, const V& v, const V& w) {
    Message m;
    m.pose.pose.position={p.x(),p.y(),p.z()};
    const Eigen::Quaterniond q(r);
    m.pose.pose.orientation={q.x(),q.y(),q.z(),q.w()};
    m.twist.twist.linear={v.x(),v.y(),v.z()};
    m.twist.twist.angular={w.x(),w.y(),w.z()};
    return m;
}

/** Read the resulting homogeneous body pose. */
Eigen::Isometry3d pose(const Message& m) {
    const auto& p=m.pose.pose.position; const auto& q=m.pose.pose.orientation;
    auto t=Eigen::Isometry3d::Identity();
    t.linear()=Eigen::Quaterniond(q.w,q.x,q.y,q.z).toRotationMatrix();
    t.translation()=V(p.x,p.y,p.z);
    return t;
}

/** Read the standard body twist as a six-vector. */
Eigen::Matrix<double,6,1> twist(const Message& m) {
    const auto& v=m.twist.twist.linear; const auto& w=m.twist.twist.angular;
    Eigen::Matrix<double,6,1> result; result<<v.x,v.y,v.z,w.x,w.y,w.z; return result;
}

/** Check a known FCU trajectory and the full covariance with finite differences. */
int main() {
    const double pitch=-std::acos(-1.0)/6.0;
    const V lever(.100,-.029,.060), center(2.3,-1.1,.9), velocity(.3,-.2,.1);
    const M r_fi=Eigen::AngleAxisd(pitch,V::UnitY()).toRotationMatrix();
    odin_ros_driver::FcuInstallation mount; mount.configure(pitch,lever);
    double pose_error=0, velocity_error=0, covariance_error=0;
    // Pure yaw about FCU must leave the FCU origin fixed and produce no pitch or linear velocity.
    for (int i=0;i<=72;++i) {
        for (bool tilted : {false,true}) {
            const M r=(Eigen::AngleAxisd(i*.1,V::UnitZ()) *
                Eigen::AngleAxisd(tilted?.2:0,V::UnitY()) *
                Eigen::AngleAxisd(tilted?-.15:0,V::UnitX())).toRotationMatrix();
            const V w_f(tilted?.2:0,tilted?-.3:0,.7), v_f=tilted?velocity:V::Zero();
            const M r_i=r*r_fi;
            const V p_i=center+r*lever, v_i=v_f+r*w_f.cross(lever), w_i=r_fi.transpose()*w_f;
            auto raw=input(p_i,r_i,v_i,w_i), out=raw;
            M6 a;
            for (int row=0;row<6;++row) for (int col=0;col<6;++col)
                a(row,col)=std::sin(1+row*6+col)*.01;
            const M6 covariance=a*a.transpose();
            Eigen::Map<Row6>(out.pose.covariance.data())=covariance;
            Eigen::Map<Row6>(out.twist.covariance.data())=covariance;
            mount.apply(out);
            pose_error=std::max(pose_error,(pose(out).translation()-center).norm());
            pose_error=std::max(pose_error,(pose(out).linear()-r).norm());
            velocity_error=std::max(velocity_error,(twist(out).head<3>()-r.transpose()*v_f).norm());
            velocity_error=std::max(velocity_error,(twist(out).tail<3>()-w_f).norm());
            if (out.child_frame_id!="fcu") return 1;
            // The physical device sensor chain must remain exactly where it was in odom.
            pose_error=std::max(pose_error,((pose(out)*mount.fcuFromImu()).matrix()-pose(raw).matrix()).norm());
            M6 jp,jt;
            constexpr double epsilon=1e-6; // Central differences in metres/radians and m/s/rad/s.
            for (int axis=0;axis<6;++axis) {
                V dp=V::Zero(),da=V::Zero(),dv=V::Zero(),dw=V::Zero();
                if(axis<3) {dp[axis]=epsilon;dv[axis]=epsilon;}
                else {da[axis-3]=epsilon;dw[axis-3]=epsilon;}
                const M plus_r=axis<3?r_i:Eigen::AngleAxisd(epsilon,V::Unit(axis-3)).toRotationMatrix()*r_i;
                const M minus_r=axis<3?r_i:Eigen::AngleAxisd(-epsilon,V::Unit(axis-3)).toRotationMatrix()*r_i;
                auto plus=input(p_i+dp,plus_r,v_i,w_i), minus=input(p_i-dp,minus_r,v_i,w_i);
                mount.apply(plus);mount.apply(minus);
                jp.block<3,1>(0,axis)=(pose(plus).translation()-pose(minus).translation())/(2*epsilon);
                const Eigen::AngleAxisd delta(pose(plus).linear()*pose(minus).linear().transpose());
                jp.block<3,1>(3,axis)=delta.axis()*delta.angle()/(2*epsilon);
                plus=input(p_i,r_i,v_i+dv,w_i+dw);minus=input(p_i,r_i,v_i-dv,w_i-dw);
                mount.apply(plus);mount.apply(minus);
                jt.col(axis)=(twist(plus)-twist(minus))/(2*epsilon);
            }
            covariance_error=std::max(covariance_error,(Eigen::Map<Row6>(out.pose.covariance.data())-jp*covariance*jp.transpose()).norm());
            covariance_error=std::max(covariance_error,(Eigen::Map<Row6>(out.twist.covariance.data())-jt*covariance*jt.transpose()).norm());
            raw.pose.covariance[0]=-1;raw.twist.covariance[0]=-1;
            mount.apply(raw);
            if(raw.pose.covariance[0]!=-1 || raw.twist.covariance[0]!=-1) return 2;
        }
    }
    std::cout<<"pose="<<pose_error<<" velocity="<<velocity_error<<" covariance="<<covariance_error<<'\n';
    return pose_error<1e-12 && velocity_error<1e-12 && covariance_error<1e-9 ? 0:3;
}
'''


def test_native_fcu_mounting_and_covariance(tmp_path):
    """Recover a stationary/yawing/tilting FCU and verify full covariance propagation."""
    root = Path(__file__).resolve().parents[2]
    source = tmp_path / "fcu_transform.cpp"
    binary = tmp_path / "fcu_transform"
    source.write_text(CPP)
    subprocess.run([
        "c++", "-std=c++17", "-O2", "-I/usr/include/eigen3",
        f"-I{root / 'drivers/odin/src/odin_ros_driver/include'}", str(source), "-o", str(binary),
    ], check=True, capture_output=True, text=True)
    result = subprocess.run([str(binary)], check=True, capture_output=True, text=True)
    print(result.stdout.strip())
