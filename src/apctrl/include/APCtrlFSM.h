#ifndef __APCTRLFSM_H
#define __APCTRLFSM_H

#include "rclcpp/rclcpp.hpp"

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <mavros_msgs/srv/command_long.hpp>
#include <mavros_msgs/srv/command_bool.hpp>

#include "input.h"
#include "controller.h"

// 自动起飞降落相关参数结构体
struct AutoTakeoffLand_t
{
	bool landed{true};  // 是否已着陆
	rclcpp::Time toggle_takeoff_land_time;  // 切换起飞/降落的时间戳
	std::pair<bool, rclcpp::Time> delay_trigger{std::pair<bool, rclcpp::Time>(false, rclcpp::Time(0))};  // ?延迟触发器
	Eigen::Vector4d start_pose;  // 起始姿态 (x,y,z,yaw)
	
	static constexpr double MOTORS_SPEEDUP_TIME = 3.0;  // 起飞前电机预热时间(秒)
	static constexpr double DELAY_TRIGGER_TIME = 2.0;   // 到达目标高度后的延迟触发时间(秒)
};

// AP控制状态机类
class APCtrlFSM
{
public:
	Parameter_t &param;  // 参数引用
	LinearControl &controller;  // 控制器引用
	rclcpp::Node::SharedPtr node_;  // ROS2节点指针

	// 各类数据对象
	RC_Data_t rc_data;                    // 遥控器数据
	State_Data_t state_data;              // 状态数据
	ExtendedState_Data_t extended_state_data;  // 扩展状态数据
	Odom_Data_t odom_data;                // 里程计数据
	Imu_Data_t imu_data;                  // IMU数据
	Command_Data_t cmd_data;              // 指令数据
	Battery_Data_t bat_data;              // 电池数据
	Takeoff_Land_Data_t takeoff_land_data;  // 起降数据

	bool set_GUIDED_flag;  // 是否设置GUIDED模式
	
	// ROS2发布者和服务客户端
	rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr traj_start_trigger_pub;  // 轨迹触发发布者
	rclcpp::Publisher<mavros_msgs::msg::AttitudeTarget>::SharedPtr ctrl_FCU_pub;  // 控制指令发布者
	rclcpp::Publisher<quadrotor_msgs::msg::ApctrlDebug>::SharedPtr debug_pub;  // 调试信息发布者
	
	rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_FCU_mode_client;  // 设置飞控模式服务客户端
	rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arming_client;  // 解锁服务客户端
	rclcpp::Client<mavros_msgs::srv::CommandLong>::SharedPtr reboot_FCU_client;  // 重启飞控服务客户端

	quadrotor_msgs::msg::ApctrlDebug debug_msg;  // 调试消息

	Eigen::Vector4d hover_pose;  // 悬停位姿
	rclcpp::Time last_set_hover_pose_time;  // 最后一次设置悬停位姿的时间

	 
	// 状态机状态枚举
	enum State_t
	{
		MANUAL_CTRL = 1,  // 手动控制模式：apctrl未激活，飞控仅受遥控器控制
		AUTO_HOVER,       // 自动悬停模式：apctrl已激活，保持悬停并等待位置指令
		CMD_CTRL,         // 指令控制模式：apctrl已激活，执行位置指令控制
		AUTO_TAKEOFF,     // 自动起飞模式
		AUTO_LAND         // 自动降落模式
	};
	//构造函数
	APCtrlFSM(Parameter_t &, LinearControl &,rclcpp::Node::SharedPtr node);
	void process();

	// 消息接收检查函数
	bool rc_is_received(const rclcpp::Time &now_time);
	bool cmd_is_received(const rclcpp::Time &now_time);
	bool odom_is_received(const rclcpp::Time &now_time);
	bool imu_is_received(const rclcpp::Time &now_time);
	bool bat_is_received(const rclcpp::Time &now_time);
	bool recv_new_odom();

	// 获取状态相关函数
	State_t get_state() { return state; }
	bool get_landed() { return takeoff_land.landed; }

private:
	State_t state;  // 当前状态
	AutoTakeoffLand_t takeoff_land;  // 自动起降相关数据

	// 控制相关函数
	Desired_State_t get_hover_des();  // 获取悬停期望状态
	Desired_State_t get_cmd_des();    // 获取指令期望状态

	// 自动起降相关函数
	void motors_idling(const Imu_Data_t &imu, Controller_Output_t &u);  // 电机怠速运行
	void land_detector(const State_t state, const Desired_State_t &des, const Odom_Data_t &odom);  // 着陆检测
	void set_start_pose_for_takeoff_land(const Odom_Data_t &odom);  // 设置起降初始位姿
	Desired_State_t get_rotor_speed_up_des(const rclcpp::Time now);  // 获取电机加速期望状态
	Desired_State_t get_takeoff_land_des(const double speed);  // 获取起降期望状态

	// 工具函数
	void set_hov_with_odom();  // 使用里程计数据设置悬停位姿
	void set_hov_with_rc();    // 使用遥控器数据设置悬停位姿

	// 飞控操作函数
	bool toggle_GUIDED_mode(bool on_off);  // 切换GUIDED模式
	bool toggle_arm_disarm(bool arm);        // 切换解锁/上锁状态
	void reboot_FCU();                       // 重启飞控

	// 发布函数
	void publish_bodyrate_ctrl(const Controller_Output_t &u, const rclcpp::Time &stamp);  // 发布体速率控制指令
	void publish_attitude_ctrl(const Controller_Output_t &u, const rclcpp::Time &stamp);  // 发布姿态控制指令
	void publish_trigger(const nav_msgs::msg::Odometry &odom_msg);  // 发布触发消息
};

#endif