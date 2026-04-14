#include "controller.h"

using namespace std;

// 从四元数计算偏航角
double LinearControl::fromQuaternion2yaw(Eigen::Quaterniond q)
{
    // 使用arctan2计算偏航角，范围[-pi,pi]
    double yaw = atan2(2 * (q.x()*q.y() + q.w()*q.z()), q.w()*q.w() + q.x()*q.x() - q.y()*q.y() - q.z()*q.z());
    return yaw;
}

// 构造函数
LinearControl::LinearControl(Parameter_t &param, const std::shared_ptr<rclcpp::Node>& node) 
    : param_(param), node_(node)
{
    resetThrustMapping();  // 初始化推力映射参数
}

/* 
计算控制输出：推力和姿态四元数
输入：
- des: 期望状态（绝对期望）
- odom: 里程计数据
- imu: IMU数据
- u: 控制输出（相对期望）
*/
quadrotor_msgs::msg::ApctrlDebug
LinearControl::calculateControl(const Desired_State_t &des,
    const Odom_Data_t &odom,
    const Imu_Data_t &imu, 
    Controller_Output_t &u)
{
    /* 计算期望加速度 */
    Eigen::Vector3d des_acc(0.0, 0.0, 0.0);
    Eigen::Vector3d Kp,Kv;
    // 获取PID参数
    Kp << param_.gain.Kp0, param_.gain.Kp1, param_.gain.Kp2;
    Kv << param_.gain.Kv0, param_.gain.Kv1, param_.gain.Kv2;
    // 计算期望加速度：前馈项 + 速度反馈 + 位置反馈
    des_acc = des.a + Kv.asDiagonal() * (des.v - odom.v) + Kp.asDiagonal() * (des.p - odom.p);
    des_acc += Eigen::Vector3d(0,0,param_.gra);  // 补偿重力

    // 计算期望推力
    u.thrust = computeDesiredCollectiveThrustSignal(des_acc);//用到z加速度

    // 计算期望姿态
    double roll,pitch;
    double yaw_odom = fromQuaternion2yaw(odom.q);
    double sin = std::sin(yaw_odom);
    double cos = std::cos(yaw_odom);
    // 根据期望加速度计算期望姿态角
    roll = (des_acc(0) * sin - des_acc(1) * cos )/ param_.gra;
    pitch = (des_acc(0) * cos + des_acc(1) * sin )/ param_.gra;
    
    // yaw_imu变量已移除，因为未使用

    // 构建期望姿态四元数
    Eigen::Quaterniond q = Eigen::AngleAxisd(des.yaw,Eigen::Vector3d::UnitZ())
        * Eigen::AngleAxisd(pitch,Eigen::Vector3d::UnitY())
        * Eigen::AngleAxisd(roll,Eigen::Vector3d::UnitX());
    // 计算期望姿态四元数(imu当前姿态*(里程计当前姿态的逆*期望里程计姿态))，绝对姿态掺入相对姿态
    u.q = imu.q * odom.q.inverse() * q;

    // 更新调试信息
    debug_msg_.des_v_x = des.v(0);
    debug_msg_.des_v_y = des.v(1);
    debug_msg_.des_v_z = des.v(2);
    
    debug_msg_.des_a_x = des_acc(0);
    debug_msg_.des_a_y = des_acc(1);
    debug_msg_.des_a_z = des_acc(2);
    
    debug_msg_.des_q_x = u.q.x();
    debug_msg_.des_q_y = u.q.y();
    debug_msg_.des_q_z = u.q.z();
    debug_msg_.des_q_w = u.q.w();
    
    debug_msg_.des_thr = u.thrust;

    // 存储推力数据用于推力-加速度映射估计
    timed_thrust_.push(std::pair<rclcpp::Time, double>(node_->now(), u.thrust));
    while (timed_thrust_.size() > 100)
    {
        timed_thrust_.pop();
    }
    return debug_msg_;
}

/*
计算期望的归一化总推力信号
输入：期望加速度
*/
double 
LinearControl::computeDesiredCollectiveThrustSignal(
    const Eigen::Vector3d &des_acc)
{
    double throttle_percentage(0.0);
    
    // 使用推力到加速度的映射系数计算油门百分比
    throttle_percentage = des_acc(2) / thr2acc_;

    return throttle_percentage;
}

/*
估计推力模型参数
使用递归最小二乘法(RLS)估计推力到加速度的映射关系
*/
bool 
LinearControl::estimateThrustModel(
    const Eigen::Vector3d &est_a,
    const Parameter_t & /*param*/)
{
    rclcpp::Time t_now = node_->now();
    while (timed_thrust_.size() >= 1)
    {
        // 选择35~45ms前的数据
        std::pair<rclcpp::Time, double> t_t = timed_thrust_.front();
        double time_passed = (t_now - t_t.first).seconds();
        if (time_passed > 0.045) // 45ms
        {
            timed_thrust_.pop();
            continue;
        }
        if (time_passed < 0.035) // 35ms
        {
            return false;
        }

        // 递归最小二乘法(RLS)算法
        double thr = t_t.second;
        timed_thrust_.pop();
        
        // 模型：est_a(2) = thr2acc_ * thr
        double gamma = 1 / (rho2_ + thr * P_ * thr);
        double K = gamma * P_ * thr;
        thr2acc_ = thr2acc_ + K * (est_a(2) - thr * thr2acc_);
        P_ = (1 - K * thr) * P_ / rho2_;

        return true;
    }
    return false;
}

// 重置推力映射参数
void 
LinearControl::resetThrustMapping(void)
{
    // 使用悬停推力百分比初始化推力到加速度的映射
    thr2acc_ = param_.gra / param_.thr_map.hover_percentage;
    P_ = 1e6;  // 初始化协方差矩阵
}
