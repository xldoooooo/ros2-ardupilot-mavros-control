/*************************************************************/
/* 致谢: github.com/uzh-rpg/rpg_quadrotor_control */
/*************************************************************/

#ifndef __CONTROLLER_H
#define __CONTROLLER_H

#include "rclcpp/rclcpp.hpp"
#include "mavros_msgs/msg/attitude_target.hpp"
#include "quadrotor_msgs/msg/apctrl_debug.hpp"
#include <queue>
#include <Eigen/Dense>
#include "APCtrlParam.h"
#include "input.h"
#include "utils.h"

// 期望状态结构体，包含位置、速度、加速度等期望值
struct Desired_State_t
{
	Eigen::Vector3d p;        // 期望位置
	Eigen::Vector3d v;        // 期望速度
	Eigen::Vector3d a;        // 期望加速度
	Eigen::Vector3d j;        // 期望加加速度(jerk)
	Eigen::Quaterniond q;     // 期望姿态四元数
	double yaw;               // 期望偏航角
	double yaw_rate;          // 期望偏航角速度

	// 默认构造函数
	Desired_State_t(){};

	// 基于里程计数据的构造函数
	Desired_State_t(Odom_Data_t &odom)
		: p(odom.p),
		  v(Eigen::Vector3d::Zero()),
		  a(Eigen::Vector3d::Zero()),
		  j(Eigen::Vector3d::Zero()),
		  q(odom.q),
		  yaw(uav_utils::get_yaw_from_quaternion(odom.q)),
		  yaw_rate(0){};
};

// 控制器输出结构体
struct Controller_Output_t
{
	// 相对于世界坐标系的机体姿态四元数
	Eigen::Quaterniond q;

	// 归一化的总推力
	double thrust;
};

// 线性控制器类
class LinearControl
{
public:
	LinearControl(Parameter_t &param, const std::shared_ptr<rclcpp::Node>& node);  // 构造函数，接收参数和节点指针

	// 计算控制输出
	quadrotor_msgs::msg::ApctrlDebug calculateControl(const Desired_State_t &des,
		const Odom_Data_t &odom,
		const Imu_Data_t &imu, 
		Controller_Output_t &u);

	// 估计推力模型参数
	bool estimateThrustModel(const Eigen::Vector3d &est_v,
		const Parameter_t &param);

	// 重置推力映射
	void resetThrustMapping(void);

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
	Parameter_t param_;        // 控制器参数
	std::shared_ptr<rclcpp::Node> node_;  // ROS2节点指针
	quadrotor_msgs::msg::ApctrlDebug debug_msg_;  // 调试消息
	std::queue<std::pair<rclcpp::Time, double>> timed_thrust_;  // 推力时间队列
	static constexpr double kMinNormalizedCollectiveThrust_ = 3.0;  // 最小归一化总推力

	// 推力-加速度映射参数
	const double rho2_ = 0.998;  // 遗忘因子，不要修改
	double thr2acc_;             // 推力到加速度的映射系数
	double P_;                   // 协方差矩阵

	// 计算期望的总推力信号
	double computeDesiredCollectiveThrustSignal(const Eigen::Vector3d &des_acc);
	// 从四元数计算偏航角
	double fromQuaternion2yaw(Eigen::Quaterniond q);
};

#endif
