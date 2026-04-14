#include "APCtrlParam.h"

/*
 * 参数文件使用方法：
 * .yaml:参数文件-->.py:launch文件引入参数文件--------->.cpp:读取参数
 *               |->CMakeLists.txt:install参数文件   |->declare_parameter->has_parameter->get_parameter
 */


// 参数类的构造函数
Parameter_t::Parameter_t()
{
}

// 从ROS节点句柄读取并配置参数
void Parameter_t::config_from_ros_handle(const std::shared_ptr<rclcpp::Node>& node)
{
    // 位置环和速度环PID控制增益
    read_essential_param(node, "gain.Kp0", gain.Kp0);  // X轴位置比例增益
    read_essential_param(node, "gain.Kp1", gain.Kp1);  // Y轴位置比例增益
    read_essential_param(node, "gain.Kp2", gain.Kp2);  // Z轴位置比例增益
    read_essential_param(node, "gain.Kv0", gain.Kv0);  // X轴速度比例增益
    read_essential_param(node, "gain.Kv1", gain.Kv1);  // Y轴速度比例增益
    read_essential_param(node, "gain.Kv2", gain.Kv2);  // Z轴速度比例增益
    read_essential_param(node, "gain.Kvi0", gain.Kvi0); // X轴速度积分增益
    read_essential_param(node, "gain.Kvi1", gain.Kvi1); // Y轴速度积分增益
    read_essential_param(node, "gain.Kvi2", gain.Kvi2); // Z轴速度积分增益
    read_essential_param(node, "gain.KAngR", gain.KAngR); // 横滚角速度增益
    read_essential_param(node, "gain.KAngP", gain.KAngP); // 俯仰角速度增益
    read_essential_param(node, "gain.KAngY", gain.KAngY); // 偏航角速度增益

    // 旋翼阻力参数
    read_essential_param(node, "rotor_drag.x", rt_drag.x); // X轴旋翼阻力系数
    read_essential_param(node, "rotor_drag.y", rt_drag.y); // Y轴旋翼阻力系数
    read_essential_param(node, "rotor_drag.z", rt_drag.z); // Z轴旋翼阻力系数
    read_essential_param(node, "rotor_drag.k_thrust_horz", rt_drag.k_thrust_horz); // 水平推力系数

    // 消息超时参数设置
    read_essential_param(node, "msg_timeout.odom", msg_timeout.odom); // 里程计消息超时时间
    read_essential_param(node, "msg_timeout.rc", msg_timeout.rc);     // 遥控器消息超时时间
    read_essential_param(node, "msg_timeout.cmd", msg_timeout.cmd);   // 指令消息超时时间
    read_essential_param(node, "msg_timeout.imu", msg_timeout.imu);   // IMU消息超时时间
    read_essential_param(node, "msg_timeout.bat", msg_timeout.bat);   // 电池消息超时时间

    // 基本控制参数
    read_essential_param(node, "pose_solver", pose_solver);           // 位姿解算器类型
    read_essential_param(node, "mass", mass);                         // 飞行器质量
    read_essential_param(node, "gra", gra);                          // 重力加速度
    read_essential_param(node, "ctrl_freq_max", ctrl_freq_max);      // 最大控制频率
    read_essential_param(node, "use_bodyrate_ctrl", use_bodyrate_ctrl); // 是否使用机体角速度控制
    read_essential_param(node, "max_manual_vel", max_manual_vel);    // 最大手动速度
    read_essential_param(node, "max_angle", max_angle);              // 最大倾角
    read_essential_param(node, "low_voltage", low_voltage);          // 低电量警告阈值

    // 遥控器通道反向设置
    read_essential_param(node, "rc_reverse.roll", rc_reverse.roll);      // 横滚通道是否反向
    read_essential_param(node, "rc_reverse.pitch", rc_reverse.pitch);    // 俯仰通道是否反向
    read_essential_param(node, "rc_reverse.yaw", rc_reverse.yaw);        // 偏航通道是否反向
    read_essential_param(node, "rc_reverse.throttle", rc_reverse.throttle); // 油门通道是否反向

    // 自动起降参数
    read_essential_param(node, "auto_takeoff_land.enable", takeoff_land.enable); // 启用自动起降
    read_essential_param(node, "auto_takeoff_land.enable_auto_arm", takeoff_land.enable_auto_arm); // 启用自动解锁
    read_essential_param(node, "auto_takeoff_land.no_RC", takeoff_land.no_RC);   // 无遥控器模式
    read_essential_param(node, "auto_takeoff_land.takeoff_height", takeoff_land.height); // 起飞高度
    read_essential_param(node, "auto_takeoff_land.takeoff_land_speed", takeoff_land.speed); // 起降速度

    // 推力模型参数
    read_essential_param(node, "thrust_model.print_value", thr_map.print_val); // 是否打印推力值
    read_essential_param(node, "thrust_model.K1", thr_map.K1);     // 推力模型系数K1
    read_essential_param(node, "thrust_model.K2", thr_map.K2);     // 推力模型系数K2
    read_essential_param(node, "thrust_model.K3", thr_map.K3);     // 推力模型系数K3
    read_essential_param(node, "thrust_model.accurate_thrust_model", thr_map.accurate_thrust_model); // 是否使用精确推力模型
    read_essential_param(node, "thrust_model.hover_percentage", thr_map.hover_percentage); // 悬停油门百分比

    // 将角度从度转换为弧度
    max_angle /= (180.0 / M_PI);

    // 参数检查和警告
    if ( takeoff_land.enable_auto_arm && !takeoff_land.enable )
    {
        takeoff_land.enable_auto_arm = false;
        RCLCPP_ERROR(node->get_logger(),"\"enable_auto_arm\" is only allowd with \"auto_takeoff_land\" enabled.");
    }
    if ( takeoff_land.no_RC && (!takeoff_land.enable_auto_arm || !takeoff_land.enable) )
    {
        takeoff_land.no_RC = false;
        RCLCPP_ERROR(node->get_logger(), "\"no_RC\" is only allowd with both \"auto_takeoff_land\" and \"enable_auto_arm\" enabled.");
    }

    if ( thr_map.print_val )
    {
        RCLCPP_WARN(node->get_logger(), "You should disable \"print_value\" if you are in regular usage.");
    }
}

// void Parameter_t::test_config_from_ros_handle(const std::shared_ptr<rclcpp::Node>& node)
// {
//     // (void)node;
//     read_essential_param(node, "gain.test_param", test_param);
// }
