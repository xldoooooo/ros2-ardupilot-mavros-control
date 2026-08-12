/**
 * @file reference_generator.hpp
 * @brief 可插拔航点参考生成器、独立参数组与低成本工厂接口。
 */
#pragma once

#include <cstdint>
#include <memory>

#include <Eigen/Core>

#include "onboard_control/dob_controller.hpp"

namespace onboard_control
{

/** 与 ExecuteWaypoints.srv 数值保持一致的命令生成方法。 */
enum class ReferenceGeneratorType : std::uint8_t
{
  kStepPosition = 0,
  kSecondOrderFilter = 1,
  kTrapezoidalProfile = 2,
  kJerkLimitedSCurve = 3,
};

/** 与 ExecuteWaypoints.srv 数值保持一致的跟踪控制方式。 */
enum class TrackingControllerType : std::uint8_t
{
  kPositionPdDob = 0,
  kTrajectoryPdDob = 1,
};

/** 参考生成器当前阶段，用于状态回读与实验数据解释。 */
enum class ReferencePhase : std::uint8_t
{
  kIdle = 0,
  kStep = 1,
  kFiltering = 2,
  kAccelerating = 3,
  kCruising = 4,
  kDecelerating = 5,
  kComplete = 6,
};

/** 二阶命令滤波器参数；每项均从 control.yaml 独立加载。 */
struct SecondOrderFilterParameters
{
  double wn_xy{0.8};
  double zeta_xy{1.1};
  double wn_z{0.6};
  double zeta_z{1.1};
  double max_velocity_xy{0.30};
  double max_velocity_z{0.15};
  double max_acceleration_xy{0.18};
  double max_acceleration_z{0.10};
  double completion_position_tolerance{0.005};
  double completion_velocity_tolerance{0.01};
};

/** 普通梯形速度参数；加速和减速上限分开，便于公平实验。 */
struct TrapezoidalProfileParameters
{
  double max_velocity_xy{0.30};
  double max_velocity_z{0.15};
  double max_acceleration_xy{0.18};
  double max_acceleration_z{0.10};
  double max_deceleration_xy{0.20};
  double max_deceleration_z{0.12};
};

/** 七阶段限 jerk S 曲线参数；长航段会自动形成恒速平台。 */
struct JerkLimitedSCurveParameters
{
  double max_velocity_xy{0.30};
  double max_velocity_z{0.15};
  double max_acceleration_xy{0.18};
  double max_acceleration_z{0.10};
  double max_deceleration_xy{0.20};
  double max_deceleration_z{0.12};
  double max_jerk_xy{0.15};
  double max_jerk_z{0.08};
};

/** 所有参考方法的配置集合；方法间不共享可调运动学参数。 */
struct ReferenceGeneratorParameters
{
  SecondOrderFilterParameters second_order;
  TrapezoidalProfileParameters trapezoidal;
  JerkLimitedSCurveParameters s_curve;
};

/** 一次 100 Hz 更新产生的完整轨迹参考与阶段状态。 */
struct GeneratedReference
{
  ControlReference control;
  double yaw_rate{0.0};
  ReferencePhase phase{ReferencePhase::kIdle};
  bool finished{false};
};

/** 所有航点命令生成方法共用的最小接口。 */
class ReferenceGenerator
{
public:
  virtual ~ReferenceGenerator() = default;

  /** 从静止参考状态建立一个航段；所有方法沿同一条空间直线同步三轴。 */
  virtual void reset(
    const Eigen::Vector3d & start_position,
    double start_yaw,
    const Eigen::Vector3d & target_position,
    double target_yaw) = 0;

  /** 按实测控制周期推进并返回位置、速度、加速度和偏航参考。 */
  virtual GeneratedReference update(double dt_seconds) = 0;

  /** 返回 reset 后尚未推进时间的初始参考。 */
  virtual GeneratedReference sample() const = 0;
};

/** 校验独立参数组；失败时由调用者拒绝启动节点。 */
bool valid_reference_generator_parameters(
  const ReferenceGeneratorParameters & parameters) noexcept;

/** 按任务选择创建唯一活动生成器；构造只发生在航点任务边界。 */
std::unique_ptr<ReferenceGenerator> make_reference_generator(
  ReferenceGeneratorType type,
  const ReferenceGeneratorParameters & parameters);

}  // namespace onboard_control
