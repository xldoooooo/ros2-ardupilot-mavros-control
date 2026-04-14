#ifndef __TEST_H
#define __TEST_H

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/logger.hpp"
#include "mavros_msgs/msg/state.hpp"
#include "mavros_msgs/msg/extended_state.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "quadrotor_msgs/msg/position_command.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "mavros_msgs/msg/rc_in.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "quadrotor_msgs/msg/takeoff_land.hpp"
#include "std_msgs/msg/bool.hpp"

class Test {
private:
    std::shared_ptr<rclcpp::Node> node_;
    int counter_;  // 用于控制显示频率的计数器

    // 存储最新的消息数据
    mavros_msgs::msg::State::SharedPtr state_msg_;
    mavros_msgs::msg::ExtendedState::SharedPtr extended_state_msg_;
    nav_msgs::msg::Odometry::SharedPtr odom_msg_;
    quadrotor_msgs::msg::PositionCommand::SharedPtr cmd_msg_;
    sensor_msgs::msg::Imu::SharedPtr imu_msg_;
    mavros_msgs::msg::RCIn::SharedPtr rc_msg_;
    sensor_msgs::msg::BatteryState::SharedPtr battery_msg_;
    quadrotor_msgs::msg::TakeoffLand::SharedPtr takeoff_land_msg_;
    // std_msgs::msg::Bool::SharedPtr takeoff_land_msg_;

public:
    Test(const std::shared_ptr<rclcpp::Node>& node);
    
    // 回调函数 - 只保存数据
    void stateCallback(const mavros_msgs::msg::State::SharedPtr msg);
    void extendedStateCallback(const mavros_msgs::msg::ExtendedState::SharedPtr msg);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void cmdCallback(const quadrotor_msgs::msg::PositionCommand::SharedPtr msg);
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
    void rcCallback(const mavros_msgs::msg::RCIn::SharedPtr msg);
    void batteryCallback(const sensor_msgs::msg::BatteryState::SharedPtr msg);
    void takeoffLandCallback(const quadrotor_msgs::msg::TakeoffLand::SharedPtr msg);
    // void takeoffLandCallback(const std_msgs::msg::Bool::SharedPtr msg);

    // 显示函数
    void displayMessages();
};

#endif

