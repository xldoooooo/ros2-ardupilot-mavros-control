#ifndef __INPUT_H
#define __INPUT_H

#include "rclcpp/rclcpp.hpp"
#include <Eigen/Dense>

#include <sensor_msgs/msg/imu.hpp>
#include <quadrotor_msgs/msg/position_command.hpp>
#include <quadrotor_msgs/msg/takeoff_land.hpp>
#include <mavros_msgs/msg/rc_in.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/msg/extended_state.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include "utils.h"
#include "APCtrlParam.h"

/**
 * @brief ң����������
 * ���ڴ����ʹ洢����ң��������������
 */
class RC_Data_t
{
public:
  rclcpp::Node::SharedPtr node_;    // ROS2�ڵ�ָ��
  double mode;                      // ��ǰ����ģʽ
  double gear;                      // ��λ״̬
  double reboot_cmd;               // ��������
  double last_mode;                // ��һ�εķ���ģʽ
  double last_gear;                // ��һ�εĵ�λ״̬
  double last_reboot_cmd;          // ��һ�ε���������
  bool have_init_last_mode{false};        // �Ƿ��ʼ������һ��ģʽ
  bool have_init_last_gear{false};        // �Ƿ��ʼ������һ�ε�λ
  bool have_init_last_reboot_cmd{false};  // �Ƿ��ʼ������һ����������
  double ch[4];                    // ң�����ĸ�ͨ����ֵ

  mavros_msgs::msg::RCIn msg;      // ң����ԭʼ��Ϣ
  rclcpp::Time rcv_stamp;          // ����ʱ���

  bool is_command_mode;            // �Ƿ�������ģʽ
  bool enter_command_mode;         // �Ƿ��������ģʽ
  bool is_hover_mode;              // �Ƿ�����ͣģʽ
  bool enter_hover_mode;           // �Ƿ������ͣģʽ
  bool toggle_reboot;              // ��������״̬

  // ң���������ֵ����
  static constexpr double GEAR_SHIFT_VALUE = 0.75;           // ��λ�л���ֵ
  static constexpr double API_MODE_THRESHOLD_VALUE = 0.75;   // APIģʽ�л���ֵ
  static constexpr double REBOOT_THRESHOLD_VALUE = 0.5;      // ������ֵ
  static constexpr double DEAD_ZONE = 0.25;                  // ����ֵ
  // static constexpr double TAKEOFF_LAND_THRESHOLD_VALUE = 0.75; // ��ɽ�����ֵ

  RC_Data_t(const rclcpp::Node::SharedPtr& node);
  void check_validity();           // ���������Ч��
  bool check_centered();           // ����Ƿ����
  void feed(const mavros_msgs::msg::RCIn::SharedPtr pMsg);  // ��������
  bool is_received(const rclcpp::Time &now_time);           // ����Ƿ���յ�����
};

/**
 * @brief ��̼�������
 * ���ڴ����ʹ洢����λ�á��ٶȺ���̬��Ϣ
 */
class Odom_Data_t
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  rclcpp::Node::SharedPtr node_;
  Eigen::Vector3d p;               // λ��
  Eigen::Vector3d v;               // �ٶ�
  Eigen::Quaterniond q;            // ��̬��Ԫ��
  Eigen::Vector3d w;               // ���ٶ�

  nav_msgs::msg::Odometry msg;     // ��̼�ԭʼ��Ϣ
  rclcpp::Time rcv_stamp;          // ����ʱ���
  bool recv_new_msg;               // �Ƿ���յ�����Ϣ

  Odom_Data_t(const rclcpp::Node::SharedPtr& node);
  void feed(const nav_msgs::msg::Odometry::SharedPtr pMsg);
};


class Imu_Data_t
{
public:
  rclcpp::Node::SharedPtr node_;
  Eigen::Quaterniond q;            // ��̬��Ԫ��
  Eigen::Vector3d w;               // ���ٶ�
  Eigen::Vector3d a;               // ���ٶ�

  sensor_msgs::msg::Imu msg;       // IMUԭʼ��Ϣ
  rclcpp::Time rcv_stamp;          // ����ʱ���

  Imu_Data_t(const rclcpp::Node::SharedPtr& node);
  void feed(const sensor_msgs::msg::Imu::SharedPtr pMsg);
};


class State_Data_t
{
public:
  rclcpp::Node::SharedPtr node_;
  mavros_msgs::msg::State current_state;          
  mavros_msgs::msg::State state_before_GUIDED;   

  State_Data_t(const rclcpp::Node::SharedPtr& node);
  void feed(const mavros_msgs::msg::State::SharedPtr pMsg);
};


class ExtendedState_Data_t
{
public:
  rclcpp::Node::SharedPtr node_;
  mavros_msgs::msg::ExtendedState current_extended_state;  // ��ǰ��չ״̬

  ExtendedState_Data_t(const rclcpp::Node::SharedPtr& node);
  void feed(const mavros_msgs::msg::ExtendedState::SharedPtr pMsg);
};

/**
 * @brief ��������������
 * ���ڴ����ʹ洢λ�ÿ�������
 */
class Command_Data_t
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  rclcpp::Node::SharedPtr node_;
  Eigen::Vector3d p;               // λ������
  Eigen::Vector3d v;               // �ٶ�����
  Eigen::Vector3d a;               // ���ٶ�����
  Eigen::Vector3d j;               // �Ӽ��ٶ�����
  double yaw;                      // ƫ��������
  double yaw_rate;                 // ƫ�����ٶ�����

  quadrotor_msgs::msg::PositionCommand msg;  // λ������ԭʼ��Ϣ
  rclcpp::Time rcv_stamp;                    // ����ʱ���

  Command_Data_t(const rclcpp::Node::SharedPtr& node);
  void feed(const quadrotor_msgs::msg::PositionCommand::SharedPtr pMsg);
};

/**
 * @brief ���������
 * ���ڴ����ʹ洢���״̬��Ϣ
 */
class Battery_Data_t
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  rclcpp::Node::SharedPtr node_;
  double volt{0.0};                // ��ѹ
  double percentage{0.0};          // �����ٷֱ�

  sensor_msgs::msg::BatteryState msg;  // ���״̬ԭʼ��Ϣ
  rclcpp::Time rcv_stamp;              // ����ʱ���

  Battery_Data_t(const rclcpp::Node::SharedPtr& node);
  void feed(const sensor_msgs::msg::BatteryState::SharedPtr pMsg);
};

/**
 * @brief ��ɽ���������
 * ���ڴ����ʹ洢��ɽ�������
 */
class Takeoff_Land_Data_t
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  rclcpp::Node::SharedPtr node_;
  bool triggered{false};                  // �Ƿ񴥷�
  uint8_t takeoff_land_cmd;               // ��ɽ�������

  quadrotor_msgs::msg::TakeoffLand msg;   // ��ɽ���ԭʼ��Ϣ
  rclcpp::Time rcv_stamp;                 // ����ʱ���

  Takeoff_Land_Data_t(const rclcpp::Node::SharedPtr& node);
  void feed(const quadrotor_msgs::msg::TakeoffLand::SharedPtr pMsg);
};

#endif
