// Fixed mounting conversion at the host publication boundary; device/world frames stay unchanged.
#pragma once

#include <Eigen/Dense>
#include <cmath>
#include <stdexcept>

namespace odin_ros_driver {

/** Return the cross-product matrix of a three-dimensional vector. */
inline Eigen::Matrix3d skew(const Eigen::Vector3d& value) {
    Eigen::Matrix3d result;
    result << 0.0, -value.z(), value.y(),
              value.z(), 0.0, -value.x(),
              -value.y(), value.x(), 0.0;
    return result;
}

class FcuInstallation {
public:
    /** Cache I<-F: pitch maps IMU axes into FCU FLU, lever is FCU->IMU in FLU metres. */
    void configure(double pitch, const Eigen::Vector3d& lever) {
        if (!std::isfinite(pitch) || !lever.allFinite()) {
            throw std::invalid_argument("Odin installation must be finite");
        }
        fcu_from_imu_ = Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()).toRotationMatrix();
        imu_from_fcu_ = fcu_from_imu_.transpose();
        imu_to_fcu_ = -imu_from_fcu_ * lever;
        lever_ = lever;
    }

    /** Expose the true F<-I sensor transform for the existing IMU/LiDAR TF chain. */
    Eigen::Isometry3d fcuFromImu() const {
        Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
        result.linear() = fcu_from_imu_;
        result.translation() = lever_;
        return result;
    }

    /** Right-multiply pose by I<-F and convert mixed SDK twist to standard FCU body twist. */
    template<class Odometry>
    void apply(Odometry& message) const {
        auto& pose = message.pose.pose;
        Eigen::Quaterniond orientation(pose.orientation.w, pose.orientation.x,
                                       pose.orientation.y, pose.orientation.z);
        if (!orientation.coeffs().allFinite() || orientation.norm() < 1e-9) {
            throw std::invalid_argument("invalid Odin orientation");
        }
        orientation.normalize();
        const Eigen::Matrix3d odom_from_imu = orientation.toRotationMatrix();
        const Eigen::Matrix3d odom_from_fcu = odom_from_imu * imu_from_fcu_;
        const Eigen::Vector3d arm_odom = odom_from_imu * imu_to_fcu_;
        Eigen::Vector3d position(pose.position.x, pose.position.y, pose.position.z);
        position += arm_odom;
        pose.position.x = position.x();
        pose.position.y = position.y();
        pose.position.z = position.z();
        const Eigen::Quaterniond output_orientation(odom_from_fcu);
        pose.orientation.x = output_orientation.x();
        pose.orientation.y = output_orientation.y();
        pose.orientation.z = output_orientation.z();
        pose.orientation.w = output_orientation.w();

        auto& twist = message.twist.twist;
        const Eigen::Vector3d velocity_odom(twist.linear.x, twist.linear.y, twist.linear.z);
        const Eigen::Vector3d omega_imu(twist.angular.x, twist.angular.y, twist.angular.z);
        const Eigen::Vector3d velocity_fcu = odom_from_fcu.transpose() * velocity_odom +
            fcu_from_imu_ * omega_imu.cross(imu_to_fcu_);
        const Eigen::Vector3d omega_fcu = fcu_from_imu_ * omega_imu;
        twist.linear.x = velocity_fcu.x();
        twist.linear.y = velocity_fcu.y();
        twist.linear.z = velocity_fcu.z();
        twist.angular.x = omega_fcu.x();
        twist.angular.y = omega_fcu.y();
        twist.angular.z = omega_fcu.z();

        // Covariance convention: odom translation/world small angle; odom linear/IMU angular.
        // The vendor does not document its perturbation basis; this is the explicit host convention.
        // Propagate the complete cross blocks, conditional on this sample's orientation.
        Matrix6 pose_jacobian = Matrix6::Identity();
        pose_jacobian.template block<3, 3>(0, 3) = -skew(arm_odom);
        Matrix6 twist_jacobian = Matrix6::Zero();
        twist_jacobian.template block<3, 3>(0, 0) = odom_from_fcu.transpose();
        twist_jacobian.template block<3, 3>(0, 3) = -fcu_from_imu_ * skew(imu_to_fcu_);
        twist_jacobian.template block<3, 3>(3, 3) = fcu_from_imu_;
        propagate(message.pose.covariance, pose_jacobian);
        propagate(message.twist.covariance, twist_jacobian);
        message.child_frame_id = "fcu";
    }

private:
    // Fixed 6x6 matrices retain all position/rotation and linear/angular cross covariance.
    using Matrix6 = Eigen::Matrix<double, 6, 6>;
    using RowMatrix6 = Eigen::Matrix<double, 6, 6, Eigen::RowMajor>;

    /** Preserve unknown covariance sentinels and avoid inventing precision for invalid arrays. */
    template<class Covariance>
    static void propagate(Covariance& covariance, const Matrix6& jacobian) {
        Eigen::Map<RowMatrix6> matrix(covariance.data());
        if (covariance[0] < 0.0 || !matrix.allFinite()) return;
        matrix = (jacobian * matrix * jacobian.transpose()).eval();
    }

    // Mounting only; neither an initial orientation nor a world-origin reset is learned here.
    Eigen::Matrix3d fcu_from_imu_ = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d imu_from_fcu_ = Eigen::Matrix3d::Identity();
    Eigen::Vector3d imu_to_fcu_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d lever_ = Eigen::Vector3d::Zero();
};
}  // namespace odin_ros_driver
