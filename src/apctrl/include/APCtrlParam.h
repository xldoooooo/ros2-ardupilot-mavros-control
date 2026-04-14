#ifndef __APCTRLPARAM_H
#define __APCTRLPARAM_H

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/logging.hpp"

// 参数类，用于管理和存储所有控制相关的参数
class Parameter_t
{
public:
	// 控制器增益参数结构体
	struct Gain
	{
		double Kp0, Kp1, Kp2;    // 位置环XYZ轴比例增益
		double Kv0, Kv1, Kv2;    // 速度环XYZ轴比例增益
		double Kvi0, Kvi1, Kvi2; // 速度环XYZ轴积分增益
		double Kvd0, Kvd1, Kvd2; // 速度环XYZ轴微分增益
		double KAngR, KAngP, KAngY; // 角速度环横滚、俯仰、偏航增益
	};

	// 旋翼阻力参数结构体
	struct RotorDrag
	{
		double x, y, z;           // XYZ轴旋翼阻力系数
		double k_thrust_horz;     // 水平推力系数
	};

	// 消息超时参数结构体
	struct MsgTimeout
	{
		double odom;    // 里程计消息超时时间
		double rc;      // 遥控器消息超时时间
		double cmd;     // 控制命令超时时间
		double imu;     // IMU消息超时时间
		double bat;     // 电池消息超时时间
	};

	// 推力映射参数结构体
	struct ThrustMapping
	{
		bool print_val;           // 是否打印推力值
		double K1;                // 推力模型系数K1
		double K2;                // 推力模型系数K2
		double K3;                // 推力模型系数K3
		bool accurate_thrust_model; // 是否使用精确推力模型
		double hover_percentage;    // 悬停油门百分比
	};

	// 遥控器通道反向设置结构体
	struct RCReverse
	{
		bool roll;     // 横滚通道反向
		bool pitch;    // 俯仰通道反向
		bool yaw;      // 偏航通道反向
		bool throttle; // 油门通道反向
	};

	// 自动起降参数结构体
	struct AutoTakeoffLand
	{
		bool enable;           // 启用自动起降
		bool enable_auto_arm;  // 启用自动解锁
		bool no_RC;           // 允许无遥控器操作
		double height;        // 起飞高度
		double speed;         // 起降速度
	};

	// 参数实例
	Gain gain;                // 控制增益参数
	RotorDrag rt_drag;        // 旋翼阻力参数
	MsgTimeout msg_timeout;    // 消息超时参数
	RCReverse rc_reverse;      // 遥控器反向设置
	ThrustMapping thr_map;     // 推力映射参数
	AutoTakeoffLand takeoff_land; // 自动起降参数

	// 基本控制参数
	int pose_solver;          // 姿态解算器选择
	double mass;              // 飞行器质量(kg)
	double gra;               // 重力加速度(m/s^2)
	double max_angle;         // 最大倾角限制(rad)
	double ctrl_freq_max;     // 最大控制频率(Hz)
	double max_manual_vel;    // 手动模式最大速度(m/s)
	double low_voltage;       // 低电压警告阈值(V)

	bool use_bodyrate_ctrl;   // 是否使用机体角速率控制
	// bool print_dbg;        // 是否打印调试信息（已注释）

	// 构造函数和配置方法
	Parameter_t();
	void config_from_ros_handle(const std::shared_ptr<rclcpp::Node>& node);
	// void test_config_from_ros_handle(const std::shared_ptr<rclcpp::Node>& node);

private:
	// 参数读取模板函数
	// 用于从ROS参数服务器读取参数，如果读取失败会终止程序
	template <typename TVal>
	void read_essential_param(const std::shared_ptr<rclcpp::Node>& node,const std::string& name, TVal &val)
	{
		// (void)node; // 防止未使用参数的警告
		node->declare_parameter(name, val);
		if (node->has_parameter(name))
		{
			if (node->get_parameter(name, val))
			{
				// 参数读取成功
				RCLCPP_INFO(node->get_logger(), "Read param %s: %s", name.c_str(), std::to_string(val).c_str());
			}
			else
			{
				RCLCPP_ERROR(node->get_logger(), "Read param: %s failed.", name.c_str());
				throw std::runtime_error("Failed to read essential parameter: " + name);
			}
		}
		else
		{
			RCLCPP_ERROR(node->get_logger(), "Param: %s not found.", name.c_str());
			throw std::runtime_error("Parameter not found: " + name);
		}
	}
};

#endif