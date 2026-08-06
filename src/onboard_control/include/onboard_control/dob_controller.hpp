/**
 * @file dob_controller.hpp
 * @brief 与 ROS/GUI 无关的三轴位置 PD 与一阶扰动观测器接口。
 */
#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace onboard_control
{

/** 控制增益、飞行器推力映射和输出安全限幅。 */
struct ControllerParameters
{
  double wn_xy{2.236};
  double zeta_xy{0.8};
  double wn_z{2.236};
  double zeta_z{0.6};
  double observer_xy{1.5};
  double observer_z{0.6};
  double hover_throttle{0.39};
  double thrust_ratio{2.5};
  double gravity{9.8};
  double max_acceleration_xy{4.0};
  double max_acceleration_z{4.0};
  double max_disturbance{3.0};
  double max_tilt_rad{0.4363323129985824};  // 25 degrees.
  double min_total_acceleration_z{2.0};
};

/** 控制器所需的最新本地 ENU 运动状态。 */
struct VehicleKinematics
{
  Eigen::Vector3d position{Eigen::Vector3d::Zero()};
  Eigen::Vector3d velocity{Eigen::Vector3d::Zero()};
};

/** 由运动意图、悬停或航点执行器生成的统一参考。 */
struct ControlReference
{
  Eigen::Vector3d position{Eigen::Vector3d::Zero()};
  Eigen::Vector3d velocity{Eigen::Vector3d::Zero()};
  double yaw{0.0};
};

/** 一次控制计算产生的姿态、油门与诊断量。 */
struct ControlOutput
{
  Eigen::Quaterniond attitude{Eigen::Quaterniond::Identity()};
  double thrust{0.0};
  Eigen::Vector3d acceleration{Eigen::Vector3d::Zero()};
  Eigen::Vector3d disturbance{Eigen::Vector3d::Zero()};
  bool valid{false};
};

/** 唯一生产 PD+DOB 实现；所有持续控制模式均调用该类。 */
class DobController
{
public:
  explicit DobController(const ControllerParameters & parameters);

  /** 清空观测器状态，必须在控制权或飞行模式边界切换时调用。 */
  void reset();

  /** 用飞控已标定的 MOT_THST_HOVER 更新推力映射；非法值不会生效。 */
  bool set_hover_throttle(double hover_throttle) noexcept;

  /** 返回当前实际用于推力映射的悬停油门。 */
  double hover_throttle() const noexcept {return parameters_.hover_throttle;}

  /** 使用真实控制周期计算一次姿态与归一化推力。 */
  ControlOutput compute(
    const VehicleKinematics & state,
    const ControlReference & reference,
    double dt_seconds);

  const Eigen::Vector3d & disturbance() const noexcept {return disturbance_;}

private:
  ControllerParameters parameters_;
  Eigen::Vector3d observer_state_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d disturbance_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d previous_input_{Eigen::Vector3d::Zero()};
};

}  // namespace onboard_control
