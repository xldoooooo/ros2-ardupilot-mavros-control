#include <cstdio>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int64.hpp>
#include <guided_sim/msg/target.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <cstring>
#include <mavros_msgs/srv/set_mav_frame.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <std_msgs/msg/float32.hpp>
#include <termios.h>
#include <thread>
#include <Eigen/Dense>
#include <mavros_msgs/msg/attitude_target.hpp>
#include <mavros_msgs/msg/position_target.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <ctime>
#include <cstring>
#include <tf2_msgs/msg/tf_message.hpp>
#include <vector>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include "PIDControl.h"
#include <iostream>
#include <chrono>
#include <fstream>
#include "ButterWorthFIlter.hpp"
#include <csignal>
#include <cstdlib>
#include <mavros_msgs/srv/command_tol.hpp>

// Global clock for timestamping in callbacks
static rclcpp::Clock g_clock(RCL_ROS_TIME);

double calculate_distance(double x,double y,double z);
double calculate_distance(double x,double y,double z,double x1,double y1,double z1);

//---------------------自定义数据类型区域---------------------
//飞行状态
enum fly_status{
    MODE_POSITION       = 0,    
    MODE_VELOCITY       = 1,
    MODE_AUTO           = 3,
    MODE_AUTO_POSE      = 8,
    MODE_STRAIGHT       = 9,
    TEST_FLOOR_TRAJ     = 12,
    MODE_HOVER_CUSTOM   = 13,  //自定义悬停模式（统一悬停，PD+DOB）
};

//键盘值
enum {
    KEY_0       = 48,
    KEY_1       = 49,
    KEY_2       = 50,
    KEY_3       = 51, 
    KEY_4       = 52, 
    KEY_9       = 57,
    
    KEY_W       = 119,
    KEY_A       = 97,
    KEY_S       = 115,
    KEY_D       = 100,

    KEY_F       = 102,
    KEY_H       = 104,

    KEY_C       = 99,
    KEY_E       = 101,

    KEY_I       = 105,
    KEY_J       = 106,
    KEY_K       = 107,
    KEY_L       = 108,

    KEY_M       = 109,
    KEY_N       = 110,
    KEY_O       = 111,

    KEY_T       = 116,

    KEY_V       = 118,
    KEY_P       = 112,
    KEY_Y       = 121,
    KEY_Z       = 122,

    KEY_ENTER   = 13,
    KEY_SPACE   = 32,
};

typedef struct{
    float x;
    float y;
    float z;
} xyz_float_def;

typedef struct{
    double x;
    double y;
    double z;
} xyz_double_def;

typedef struct{
    double x;
    double y;
    double z;
    double a;
} xyza_double_def;

typedef struct {
    double x;
    double y;
    double z;
    double w;
} quaternion_def;

typedef struct {
    double roll;
    double pitch;
    double yaw;
} euler_def;
//---------------------自定义数据类型区域结束---------------------


//---------------------全局变量定义区域---------------------
#define GRAVITY_ACC 9.8f
double uav_weight = 3.340;              // kg, may be overridden from YAML

pthread_t th;                           //键盘线程
bool th_created = false;                // whether pthread_create succeeded
                                        // (guard against pthread_cancel on invalid handle)

int ctrlMode=0;                         //是否结束线程
int controlMode = 1;                    //控制模式的变换

bool vel_change = false;                //速度模式是否通过按键改变，用于控制是否打印速度控制信息
bool guided_change = false;
bool arm_change = false;
bool mode_int = false;                  //用于记录是否切换模式，进入新模式前，进行初始化

euler_def target_euler;         //发布消息接口中，之后四元素的定义，采用欧拉角的全局变量在定义一次

std::vector<xyz_double_def> fixed_pathpoint;   //固定轨迹的轨迹存储区域
std::vector<xyz_double_def> fill_pathpoint;    //填充轨迹
std::vector<float> fill_yaw;

xyz_double_def avoid_target_position;
quaternion_def avoid_target_pose;
bool isObstacleStop = false;
int turnDirection = 0;

int control_frequency = 100;

// 位姿/速度数据源选择 (通过 YAML 参数 use_odin_pose 控制)
bool g_use_odin_pose = true;
// 全局持有订阅器，防止局部变量析构导致回调失效
rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr  g_pose_sub;
rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr g_vel_sub;
rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr          g_imu_sub;

// ROS2 发布器
rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr refTrajectoryPub;
rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr referencePosePub;
// ROS2 服务
rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_client;
rclcpp::Client<mavros_msgs::srv::SetMavFrame>::SharedPtr set_frame_client;
rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedPtr takeoff_client;

double target_acc_x_pid_last;
double target_acc_y_pid_last;
double target_acc_z_pid_last;

double target_vel_x_pid_last;
double target_vel_y_pid_last;
double target_vel_z_pid_last;
//---------------------全局变量定义区域结束---------------------


//---------------------发送数据消息接口类型区域---------------------
geometry_msgs::msg::Twist        target_twist;       //发布控制无人机速度的消息接口变量
geometry_msgs::msg::PoseStamped  target_pose;        //发布控制无人机位置的消息接口变量
mavros_msgs::msg::AttitudeTarget target_att_thrust;  //发布控制无人机姿态+推力的消息接口类型
std_msgs::msg::Int64 stop_command;                   //发布停止轨迹规划的命令
nav_msgs::msg::Path refTrajectory;                   //发布设定的跟踪轨迹
geometry_msgs::msg::PoseStamped target_position_show;//发布当前的跟踪的目标点
//---------------------发送数据消息接口类型区域结束---------------------


//---------------------回调函数区域---------------------
xyz_double_def localPos;  // 记录当前位置
quaternion_def localAtt;  // 记录当前姿态
void pose_cb(const geometry_msgs::msg::PoseStamped::ConstSharedPtr &msg){
    localPos.x = msg->pose.position.x ;
    localPos.y = msg->pose.position.y ;    
    localPos.z = msg->pose.position.z ;       

    localAtt.x = msg->pose.orientation.x ;
    localAtt.y = msg->pose.orientation.y ; 
    localAtt.z = msg->pose.orientation.z ;
    localAtt.w = msg->pose.orientation.w ;
} //接收当前位姿的回调函数

xyz_double_def localVel;  // 记录当前速度
void vel_cb(const geometry_msgs::msg::TwistStamped::ConstSharedPtr &msg){
    localVel.x = msg->twist.linear.x ;
    localVel.y = msg->twist.linear.y ;  
    localVel.z = msg->twist.linear.z ;    
} //接受当前速度的回调函数

// 接收 extnav_bridge 输出的 FCU 位姿 (Odin SLAM → ENU)
void odin_pose_cb(const geometry_msgs::msg::PoseStamped::ConstSharedPtr &msg) {
    pose_cb(msg);  // 复用 MAVROS 回调的赋值逻辑
}

// 接收 extnav_bridge 输出的 FCU 速度
void odin_vel_cb(const geometry_msgs::msg::TwistStamped::ConstSharedPtr &msg) {
    vel_cb(msg);
}

//接收当前加速度的回调函数
xyz_double_def localAcc;  // 记录当前加速度 
void imudata_cb(const sensor_msgs::msg::Imu::ConstSharedPtr &msg){
    localAcc.x = msg->linear_acceleration.x;
    localAcc.y = msg->linear_acceleration.y;
    localAcc.z = msg->linear_acceleration.z;
} //接受当前加速度的回调函数

mavros_msgs::msg::State current_state; // 记录当前mavros状态
void mavrosState_cb(const mavros_msgs::msg::State::ConstSharedPtr& msg){
    current_state = *msg;
} //接受无人机当前状态的回调函数

std::vector<xyz_double_def> pathpoint_sub;        //用于存储轨迹点信息，采用C++容器的凡是
std::vector<xyza_double_def> pathpoint_sub_test;    
int targetPointNum = 0;                           //当前用于跟踪的轨迹点
int targetPointNum_test = 0;  
void pathsub_cb(const nav_msgs::msg::Path::ConstSharedPtr &navpath){
    rclcpp::Time start_time = g_clock.now();
    if(navpath->poses.size() > 0){
        int check_count=0;
        xyz_double_def old_path_point;
        while((check_count != navpath->poses.size()) && (check_count  < 50)){  
            //如果判断出来轨迹距离一直增加，则判定该点与新轨迹的接续点
            if(pathpoint_sub.empty()){
                pathpoint_sub.clear();
                targetPointNum = 0;
                break;
            }else{
                old_path_point = pathpoint_sub.back();
            }

            double distance_last = calculate_distance((navpath->poses)[0].pose.position.x,(navpath->poses)[0].pose.position.y,(navpath->poses)[0].pose.position.z,old_path_point.x,old_path_point.y,old_path_point.z);
            check_count = 1;
            for(;check_count < (navpath->poses.size());check_count++ ){
                double distance = calculate_distance((navpath->poses)[check_count].pose.position.x,(navpath->poses)[check_count].pose.position.y,(navpath->poses)[check_count].pose.position.z,old_path_point.x,old_path_point.y,old_path_point.z);
                if(distance_last > distance){
                    pathpoint_sub.pop_back();
                    break ; //点在轨迹的前方
                }
                distance_last = distance;    
            }

            if(pathpoint_sub.empty()){
                break;
            }
        }

        //开始压入新轨迹，并且跳出循环
        for (int i=0; i < (navpath->poses.size()); i++)
        {
            pathpoint_sub.push_back({(navpath->poses)[i].pose.position.x ,          //当目标点与轨迹上的计算点大于距离阈值时，保存轨迹点
                                    (navpath->poses)[i].pose.position.y ,
                                    (navpath->poses)[i].pose.position.z });
        }                  
            
    }else{
        RCLCPP_INFO(rclcpp::get_logger("guided_sim"), "Received trajectory is empty");
    }

    rclcpp::Time end_time = g_clock.now();
    rclcpp::Duration elapsed_time = end_time - start_time;
    RCLCPP_INFO(rclcpp::get_logger("guided_sim"), "TIME:%lf ms\r\n",elapsed_time.seconds());
    RCLCPP_INFO(rclcpp::get_logger("guided_sim"), "Received trajectory with a length of: %zu points pathpoint_sub %zu points\r\n",navpath->poses.size(),pathpoint_sub.size());
} //接受轨迹信息的回调函数

double obstacle_distance = 100000.0f;
void nearest_distance_cb(const std_msgs::msg::Float32::ConstSharedPtr &dis){
    obstacle_distance = dis->data;
} // 接收当前最近障碍物距离的回调函数
//---------------------回调函数区域结束---------------------


//---------------launch文件参数读取定义区----------------------------
//PID的控制参数定义区域
typedef struct{
    float p;
    float i;
    float d;
} pid_param_def;

typedef struct{
    pid_param_def x;
    pid_param_def y;
    pid_param_def z;
} xyz_pid_param_def;

xyz_pid_param_def pos_pid;
xyz_pid_param_def vel_pid;
xyz_pid_param_def acc_pid;
xyz_float_def vel_lim;
xyz_float_def att_lim;

//其他参数定义
double ciecle_range;         //进圈判断
double jump_pointnumber;     //跳点的设置
double vel_forward_set;
double target_x;
double target_y;
double hover_wn_xy;          //悬停XY: 固有频率 ωₙ (rad/s)
double hover_zeta_xy;        //悬停XY: 阻尼比 ζ
double hover_wn_z;           //悬停Z:  固有频率 ωₙ (rad/s)
double hover_zeta_z;         //悬停Z:  阻尼比 ζ
double dob_L_xy;             //DOB观测器增益 (XY轴)
double dob_L_z;              //DOB观测器增益 (Z轴)
double hover_throttle;       //悬停油门 (归一化, 典型0.5)
double thrust_ratio;         //最大推重比 (典型2.5)
//---------------launch文件参数读取定义区结束----------------------------

//---------------创建PID控制-------------------------------------------
// 倾角极限 (degrees) 转为加速度极限: G * tan(angle * pi/180)
double degToAccLimit(double angle_deg);
//位置控制器的PID
PID pos_control_x = PID(1.0/control_frequency, 1.0, -1.0, 0,0,0);    //控制器的输出尽可能限制在 -1m/s~1m/s
PID pos_control_y = PID(1.0/control_frequency, 1.0, -1.0, 0,0,0);
PID pos_control_z = PID(1.0/control_frequency, 1.0, -1.0, 0,0,0);
//速度控制器的PID
PID vel_control_x = PID(1.0/control_frequency, 100.0, -100.0, 0,0,0);  // 速度环输出加速度目标，不限幅，由加速度环兜底
PID vel_control_y = PID(1.0/control_frequency, 100.0, -100.0, 0,0,0);
PID vel_control_z = PID(1.0/control_frequency, 100.0, -100.0, 0,0,0);
//加速度控制器的PID
PID acc_control_x = PID(1.0/control_frequency, degToAccLimit(30.0), -degToAccLimit(30.0), 0,0,0);
PID acc_control_y = PID(1.0/control_frequency, degToAccLimit(30.0), -degToAccLimit(30.0), 0,0,0);
PID acc_control_z = PID(1.0/control_frequency, degToAccLimit(30.0), -degToAccLimit(30.0), 0,0,0);

//---------------工具函数区域 ----------------------------
void toEulerAngle(const Eigen::Quaterniond &q, double &roll, double &pitch, double &yaw){
    // roll (x-axis rotation)
    double sinr_cosp = +2.0 * (q.w() * q.x() + q.y() * q.z());
    double cosr_cosp = +1.0 - 2.0 * (q.x() * q.x() + q.y() * q.y());
    roll = atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = +2.0 * (q.w() * q.y() - q.z() * q.x());
    if (fabs(sinp) >= 1)
        pitch = copysign(M_PI / 2, sinp) ; //
    else
        pitch = asin(sinp);

    // yaw (z-axis rotation)
    double siny_cosp = +2.0 * (q.w() * q.z() + q.x() * q.y());
    double cosy_cosp = +1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z());
    yaw = atan2(siny_cosp, cosy_cosp);
    if(yaw < 0)
        yaw += 2.0f * M_PI;
} // 四元数转换成欧拉角的函数

bool clear_scanf_buffer(void){
    char ch;
    int count_time = 0;
    while ((ch = getchar()) != EOF && ch != '\n') {
        count_time++;
        if(count_time >10000){
            return false;
        }
    };
    return true;
} // 清空scanf的缓冲区

double calculate_distance(double x,double y,double z){
    return sqrt(pow((x-localPos.x),2)+pow((y-localPos.y),2)+pow((z-localPos.z),2)); 
} // 计算目标点与当下点的距离

double calculate_distance(double x,double y,double z,double x1,double y1,double z1){
    return sqrt(pow((x-x1),2)+pow((y-y1),2)+pow((z-z1),2)); 
} // 计算目标点与指定点的距离

xyz_double_def calculate_FixedModulusVector(xyz_double_def normalvector,double modulus){
    double norm = sqrt(pow(normalvector.x,2)+pow(normalvector.y,2)+pow(normalvector.z,2));
    double ratio;
    if(norm == 0){
        ratio = 1.0f;
    }else{
        ratio = modulus/norm;
    }
    xyz_double_def returnVector = {
        normalvector.x * ratio ,
        normalvector.y * ratio ,
        normalvector.z * ratio ,
    };
    return returnVector;
} // 计算固定模长的向量

double calculate_AngleDifference(double angle1, double angle2) {
    // 计算两个角度的差值
    double difference = fabs(angle1 - angle2);
    // 如果差值大于pi，说明夹角应该从另一侧计算
    if (difference > M_PI) {
        difference = 2 * M_PI - difference;
    }
    return difference;
} // 计算角度差

void PID_Reset(){
    pos_control_x.reinitiate();    //位置环PID清零
    pos_control_y.reinitiate();
    pos_control_z.reinitiate();

    vel_control_x.reinitiate();    //速度环PID清零
    vel_control_y.reinitiate();
    vel_control_z.reinitiate();

    acc_control_x.reinitiate();    //加速度环PID清零
    acc_control_y.reinitiate();
    acc_control_z.reinitiate();
} // PID重置

double slew_rate_limit(double new_val, double last_val, double limit) {
    double delta = new_val - last_val;
    if (delta > limit)       return last_val + limit;
    else if (delta < -limit) return last_val - limit;
    return new_val;
} // 钳制增长率: clamp (new - last) to ±limit, return last ± limit if exceeded

void fillAttitudeTarget(mavros_msgs::msg::AttitudeTarget& msg,
                        double roll, double pitch, double yaw,
                        double hover_thrust, double height_pid, rclcpp::Node* node) {
    Eigen::AngleAxisd rollAngle_(Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()));
    Eigen::AngleAxisd pitchAngle_(Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()));
    Eigen::AngleAxisd yawAngle_(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
    Eigen::Quaterniond quaternion_ = yawAngle_ * pitchAngle_ * rollAngle_;
    msg.orientation.x = quaternion_.coeffs()[0];
    msg.orientation.y = quaternion_.coeffs()[1];
    msg.orientation.z = quaternion_.coeffs()[2];
    msg.orientation.w = quaternion_.coeffs()[3];
    msg.thrust = hover_thrust + height_pid;
    if (msg.thrust > 1.0)      msg.thrust = 1.0;
    else if (msg.thrust < 0.0) msg.thrust = 0.0;
    msg.type_mask = 1 + 2 + 4;
    msg.header.stamp = node->now();
} // 根据期望推力、推力增量与欧拉角生成attitudeTarget

// 坐标系修正1 x' = -y, y' = x
xyz_double_def pathToLocal(xyz_double_def p) { return {-p.y, p.x, p.z}; }

// 坐标系修正2 x' = y, y' = -x
xyz_double_def cameraToNED(xyz_double_def p) { return {p.y, -p.x, p.z}; }

double degToAccLimit(double angle_deg) {
    return GRAVITY_ACC * tan(angle_deg * M_PI / 180.0);
} // 将倾角极限转为加速度极限

double clamp_symmetric(double val, double limit) {
    if (val > limit)       return limit;
    else if (val < -limit) return -limit;
    return val;
} // 将值钳制在+-limit之间

void read_param(rclcpp::Node *node){
    pos_pid.x.p = node->declare_parameter<float>("position_X_P", 0.0f);
    pos_pid.x.i = node->declare_parameter<float>("position_X_I", 0.0f);
    pos_pid.x.d = node->declare_parameter<float>("position_X_D", 0.0f);
    pos_pid.y.p = node->declare_parameter<float>("position_Y_P", 0.0f);
    pos_pid.y.i = node->declare_parameter<float>("position_Y_I", 0.0f);
    pos_pid.y.d = node->declare_parameter<float>("position_Y_D", 0.0f);
    pos_pid.z.p = node->declare_parameter<float>("position_Z_P", 0.0f);
    pos_pid.z.i = node->declare_parameter<float>("position_Z_I", 0.0f);
    pos_pid.z.d = node->declare_parameter<float>("position_Z_D", 0.0f);
    vel_pid.x.p = node->declare_parameter<float>("vel_X_P", 0.0f);
    vel_pid.x.i = node->declare_parameter<float>("vel_X_I", 0.0f);
    vel_pid.x.d = node->declare_parameter<float>("vel_X_D", 0.0f);
    vel_pid.y.p = node->declare_parameter<float>("vel_Y_P", 0.0f);
    vel_pid.y.i = node->declare_parameter<float>("vel_Y_I", 0.0f);
    vel_pid.y.d = node->declare_parameter<float>("vel_Y_D", 0.0f);
    vel_pid.z.p = node->declare_parameter<float>("vel_Z_P", 0.0f);
    vel_pid.z.i = node->declare_parameter<float>("vel_Z_I", 0.0f);
    vel_pid.z.d = node->declare_parameter<float>("vel_Z_D", 0.0f);
    acc_pid.x.p = node->declare_parameter<float>("acc_X_P", 0.0f);
    acc_pid.x.i = node->declare_parameter<float>("acc_X_I", 0.0f);
    acc_pid.x.d = node->declare_parameter<float>("acc_X_D", 0.0f);
    acc_pid.y.p = node->declare_parameter<float>("acc_Y_P", 0.0f);
    acc_pid.y.i = node->declare_parameter<float>("acc_Y_I", 0.0f);
    acc_pid.y.d = node->declare_parameter<float>("acc_Y_D", 0.0f);
    acc_pid.z.p = node->declare_parameter<float>("acc_Z_P", 0.0f);
    acc_pid.z.i = node->declare_parameter<float>("acc_Z_I", 0.0f);
    acc_pid.z.d = node->declare_parameter<float>("acc_Z_D", 0.0f);
    vel_lim.x = node->declare_parameter<float>("vel_X_limit", 1.0f);
    vel_lim.y = node->declare_parameter<float>("vel_Y_limit", 1.0f);
    vel_lim.z = node->declare_parameter<float>("vel_Z_limit", 1.0f);
    att_lim.x = node->declare_parameter<float>("roll_limit", 30.0f);
    att_lim.y = node->declare_parameter<float>("pitch_limit", 30.0f);
    att_lim.z = node->declare_parameter<float>("high_limit", 30.0f);
    ciecle_range = node->declare_parameter<double>("ciecle_range", 0.5);
    jump_pointnumber = node->declare_parameter<double>("jump_pointnumber", 1.0);
    vel_forward_set = node->declare_parameter<double>("vel_forward_setting", 1.0);
    target_x = node->declare_parameter<double>("target_x", 0.0);
    target_y = node->declare_parameter<double>("target_y", 8.0);
    hover_wn_xy   = node->declare_parameter<double>("hover_wn_xy",   1.0);
    hover_zeta_xy = node->declare_parameter<double>("hover_zeta_xy", 1.0);
    hover_wn_z    = node->declare_parameter<double>("hover_wn_z",    2.0);
    hover_zeta_z  = node->declare_parameter<double>("hover_zeta_z",  1.0);
    dob_L_xy      = node->declare_parameter<double>("dob_L_xy",      1.0);
    dob_L_z       = node->declare_parameter<double>("dob_L_z",       1.0);

    // Physical parameters
    uav_weight        = node->declare_parameter<double>("uav_weight",        3.34);
    control_frequency = node->declare_parameter<int>   ("control_frequency", 100);
    hover_throttle    = node->declare_parameter<double>("hover_throttle",     0.5);
    thrust_ratio      = node->declare_parameter<double>("thrust_ratio",       2.5);
    // 是否使用 extnav_bridge 输出的 FCU 坐标位姿 (Odin SLAM)
    g_use_odin_pose   = node->declare_parameter<bool>("use_odin_pose", true);
} // 读取launch文件中的所有参数

int sh_getch(void) {
    int cr;
    struct termios nts, ots;

    if (tcgetattr(0, &ots) < 0) // 得到当前终端(0表示标准输入)的设置
        return EOF;

    nts = ots;
    cfmakeraw(&nts); // 设置终端为Raw原始模式，该模式下所有的输入数据以字节为单位被处理
    if (tcsetattr(0, TCSANOW, &nts) < 0) // 设置上更改之后的设置
        return EOF;

    cr = getchar();
    if (tcsetattr(0, TCSANOW, &ots) < 0) // 设置还原成老的模式
        return EOF;

    return cr;
} // 读取键盘

void *keyboard_listener(void *arg){
    auto *raw_node = static_cast<rclcpp::Node*>(arg);
    std::shared_ptr<rclcpp::Node> node(raw_node, [](auto*){});
    ctrlMode = 1;
	int key;
    float scale=0.2;

	while(rclcpp::ok())
	{    
        key = sh_getch();     //该函数区分大小写
        if (key == EOF) {
            // No real terminal (e.g. ros2 launch without emulate_tty)
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        }

        // Ctrl+C (0x03) in raw mode — exit cleanly
        if (key == 3) {
            printf("\n[INFO] Ctrl+C pressed, exiting...\n");
            ctrlMode = 0;
            controlMode = MODE_HOVER_CUSTOM;
            break;
        }

	    if(current_state.armed){
            if(current_state.mode == "GUIDED"){
                switch (key){
                    case KEY_W:{   // w->119 向上
                        target_twist.linear.z = target_twist.linear.z+scale;
                        vel_change = true;
                        if(controlMode == MODE_HOVER_CUSTOM){
                            mode_int = false; 
                            controlMode = MODE_VELOCITY;
                        }
                    }break;
                    case KEY_A:{   // a->97 左转
                        target_twist.angular.z = target_twist.angular.z+scale;
                        vel_change = true;
                        if(controlMode == MODE_HOVER_CUSTOM){
                            mode_int = false; 
                            controlMode = MODE_VELOCITY;
                        }
                    }break;
                    case KEY_S:{   //s->115  向下
                        target_twist.linear.z = target_twist.linear.z-scale;
                        vel_change = true;
                        if(controlMode == MODE_HOVER_CUSTOM){
                            mode_int = false; 
                            controlMode = MODE_VELOCITY;
                        }
                    }break;
                    case KEY_D:{   //d->100  右转
                        target_twist.angular.z = target_twist.angular.z-scale;
                        vel_change = true;
                        if(controlMode == MODE_HOVER_CUSTOM){
                            mode_int = false; 
                            controlMode = MODE_VELOCITY;
                        }
                    }break;
                    case KEY_I:{   //i->105  向前
                        target_twist.linear.x = target_twist.linear.x+scale;
                        vel_change = true;
                        if(controlMode == MODE_HOVER_CUSTOM){
                            mode_int = false; 
                            controlMode = MODE_VELOCITY;
                        }
                    }break;
                    case KEY_J:{   //j->106  向左
                        target_twist.linear.y = target_twist.linear.y+scale;
                        vel_change = true;
                        if(controlMode == MODE_HOVER_CUSTOM){
                            mode_int = false; 
                            controlMode = MODE_VELOCITY;
                        }
                    }break;
                    case KEY_K:{   //k->107  向后
                        target_twist.linear.x = target_twist.linear.x-scale;
                        vel_change = true;
                        if(controlMode == MODE_HOVER_CUSTOM){
                            mode_int = false; 
                            controlMode = MODE_VELOCITY;
                        }
                    }break;
                    case KEY_L:{   //l->108  向右
                        target_twist.linear.y = target_twist.linear.y-scale;
                        vel_change = true;
                        if(controlMode == MODE_HOVER_CUSTOM){
                            mode_int = false; 
                            controlMode = MODE_VELOCITY;
                        }
                    }break;
                    case KEY_SPACE:{    //Space->32 悬停
                        mode_int = false; 
                        controlMode = MODE_HOVER_CUSTOM; 
                    }break;
                    case KEY_M:{   //m->109 模式选择
                        mode_int = false;               
                        controlMode = MODE_HOVER_CUSTOM;      //只要进入m的参数输入，等待时候，就进入悬停模式，等待操作完成
                        printf("Please select guided control mode: \r\n \
                        v: velocity control;\r\n \
                        p: position control;\r\n \
                        a: track the trajectory of falio with position;\r\n \
                        s: straightly fly towards a postion;\r\n \
                        f: fixed trajectory \r\n \
                        t: track the trajectory of falio; \r\n \
                        \r\n");

                        key=sh_getch();     //按下m进行功能选择
                        printf("get key value:%d\r\n",key);
                        switch (key){                       
                            case KEY_V:{  //v->118
                                mode_int = false;  //FLAG
                                controlMode = MODE_VELOCITY;
                                printf("Current control mode: \"Veolcity\"\r\n");
                                }break;
                            case KEY_P:{  //p->112  
                                printf("Selsct mode: \"Position\",please input target position in \"x  y  z\" :\n");
                                float xt = 0,yt = 0, zt = 0;
                                scanf("%f %f %f",&xt,&yt,&zt);
                                clear_scanf_buffer();       //清除scanf中的缓冲数据
                                printf("receive position is [%f, %f, %f] :\r\n",xt,yt,zt);
                                printf("Please press enter to confirm, any other key to cancel\r\n");
                                key=sh_getch();     //等待确认按键的输入
                                if(key == KEY_ENTER){
                                    //增量式控制
                                    target_pose.pose.position.x = xt ; //+ localPos.x
                                    target_pose.pose.position.y = yt ; //+ localPos.y
                                    target_pose.pose.position.z = zt ; //+ localPos.z
                                    target_pose.pose.orientation.x = localAtt.x;
                                    target_pose.pose.orientation.y = localAtt.y;
                                    target_pose.pose.orientation.z = localAtt.z;
                                    target_pose.pose.orientation.w = localAtt.w;

                                    xt = target_pose.pose.position.x;
                                    yt = target_pose.pose.position.y;
                                    zt = target_pose.pose.position.z;
                                    
                                    mode_int = false;                       //改变模式
                                    controlMode = MODE_POSITION;                                
                                    printf("Absolute position set [%f, %f, %f] successfully!\r\n",xt,yt,zt);
                                }else{
                                    printf("Position setting canceled\r\n");
                                }
                                }break;
                            case KEY_A:{  //a->97
                                printf("please Set target point position  in \"x  y  z\" :\n");
                                xyz_double_def goalposition_;
                                scanf("%lf %lf %lf",&(goalposition_.x),&(goalposition_.y),&(goalposition_.z));
                                clear_scanf_buffer();       //清除scanf中的缓冲数据

                                printf("receive position is [%f, %f, %f] :\r\n",goalposition_.x,goalposition_.y,goalposition_.z);
                                printf("Please press enter to confirm, any other key to cancel\r\n");
                            
                                key=sh_getch();                 //获取确认按键

                                if(key == KEY_ENTER){
                                    target_pose.pose.position.x = goalposition_.x + localPos.x;
                                    target_pose.pose.position.y = goalposition_.y + localPos.y;
                                    target_pose.pose.position.z = goalposition_.z + localPos.z;

                                    target_pose.pose.orientation.x = localAtt.x;
                                    target_pose.pose.orientation.y = localAtt.y;       //确定航向
                                    target_pose.pose.orientation.z = localAtt.z;
                                    target_pose.pose.orientation.w = localAtt.w;

                                    mode_int = false;
                                    controlMode = MODE_AUTO;
                                    printf("start to auto mode \r\n");
                                }else{
                                    printf("target Postition  setting canceled \r\n");
                                }
                                }break;
                            case KEY_S:{
                                printf("This is straight mode, please input target position in \"x  y  z\" :\r\n");
                                float xt = 0,yt = 0, zt = 0;
                                scanf("%f %f %f",&xt,&yt,&zt);
                                clear_scanf_buffer();       //清除scanf中的缓冲数据

                                printf("receive position is [%f, %f, %f] :\r\n",xt,yt,zt);
                                printf("Please press enter to confirm, any other key to cancel\r\n");

                                key=sh_getch();                 //获取确认按键

                                if(key == KEY_ENTER){
                                    //增量式控制
                                    target_pose.pose.position.x = xt + localPos.x;
                                    target_pose.pose.position.y = yt + localPos.y;
                                    target_pose.pose.position.z = zt + localPos.z;
                                    target_pose.pose.orientation.x = localAtt.x;
                                    target_pose.pose.orientation.y = localAtt.y;
                                    target_pose.pose.orientation.z = localAtt.z;
                                    target_pose.pose.orientation.w = localAtt.w;
                
                                    Eigen::Quaterniond q(target_pose.pose.orientation.w, target_pose.pose.orientation.x, target_pose.pose.orientation.y, target_pose.pose.orientation.z);
                                    toEulerAngle(q,target_euler.roll,target_euler.pitch,target_euler.yaw);
                                    
                                    mode_int = false;
                                    controlMode = MODE_STRAIGHT;

                                    printf("Absolute position set [%f, %f, %f] successfully!\r\n",xt,yt,zt);
                                }else{
                                    printf("Position setting canceled\r\n");
                                }
                                }break;  
                            case KEY_T: {
                                printf("please Set target point position  in \"x  y  z\" :\n");
                                xyz_double_def goalposition_;
                                scanf("%lf %lf %lf",&(goalposition_.x),&(goalposition_.y),&(goalposition_.z));
                                clear_scanf_buffer();       //清除scanf中的缓冲数据

                                printf("receive position is [%f, %f, %f] :\r\n",goalposition_.x,goalposition_.y,goalposition_.z);
                                printf("Please press enter to confirm, any other key to cancel\r\n");
                            
                                key=sh_getch();                 //获取确认按键

                                if(key == KEY_ENTER){
                                    target_pose.pose.position.x = goalposition_.x + localPos.x;        //带坐标转换，从笛卡尔转到东北天
                                    target_pose.pose.position.y = goalposition_.y + localPos.y;
                                    target_pose.pose.position.z = goalposition_.z + localPos.z;

                                    target_pose.pose.orientation.x = localAtt.x;
                                    target_pose.pose.orientation.y = localAtt.y;       //确定航向
                                    target_pose.pose.orientation.z = localAtt.z;
                                    target_pose.pose.orientation.w = localAtt.w;

                                    Eigen::Quaterniond q(target_pose.pose.orientation.w, target_pose.pose.orientation.x, target_pose.pose.orientation.y, target_pose.pose.orientation.z);
                                    toEulerAngle(q,target_euler.roll,target_euler.pitch,target_euler.yaw);    

                                    mode_int = false;
                                    controlMode = MODE_AUTO_POSE;
                                    printf("start to auto pose mode \r\n");
                                }else{
                                    printf("target Postition  setting canceled \r\n");
                                }
                            }break;
                            default:{
                                printf("[Warning!] Invalid selection, please press m and slelect again!: \n");
                            }break;
                        }                     
                        }break;
                    default:{
                        printf("Invalid button!\r\n");
                    }break;
                } 
            }else{
                //先按下s键，后按下w键，最后按下确认enter键切换到guided模式
                //主程序切换模式，获取当前位置，发送当前位置，悬停
                mode_int = false; 
                controlMode = MODE_HOVER_CUSTOM; 

                if (key == KEY_S){
                    printf("Please continue to press the W key to switch to guided mode, and exit with other keys\r\n");
                    key = sh_getch(); 
                    if(key == KEY_W){
                        printf("Press the enter key to switch to guided mode, and use other keys to exit\r\n");

                        key = sh_getch();
                        if(key == KEY_ENTER){

                            auto guided_req_recovery = std::make_shared<mavros_msgs::srv::SetMode::Request>();
                            guided_req_recovery->custom_mode = "GUIDED";

                            auto frame_req_recovery = std::make_shared<mavros_msgs::srv::SetMavFrame::Request>();
                            frame_req_recovery->mav_frame = 1;
                            
                            auto set_mode_future_recovery = set_mode_client->async_send_request(guided_req_recovery);
                            rclcpp::spin_until_future_complete(node, set_mode_future_recovery);
                            if (set_mode_future_recovery.get()->mode_sent)
                            {
                                printf("[INFO] Guided enabled\n");

                                    auto set_frame_future_recovery = set_frame_client->async_send_request(frame_req_recovery);
                                    rclcpp::spin_until_future_complete(node, set_frame_future_recovery);
                                    if (set_frame_future_recovery.get()->success) {
                                        printf("[INFO] Set BODY_FRAME of velocity successfully\n");
                                        mode_int = false; 
                                        controlMode = MODE_HOVER_CUSTOM; 
                                    }
                                    else     {
                                        printf("[INFO] Failed to set BODY_FRAME, please retry!\n");
                                    }
                            }
                            else  {
                                printf("[INFO] Failed to set Guided, please retry! \n");
                            }


                                  
                        }else{
                            printf("current_state_mode2:%s\r\n",current_state.mode.c_str());
                        }
                    }else{
                        printf("current_state_mode1:%s\r\n",current_state.mode.c_str());
                    }
                }   
            }        
        }
        // Disarm is handled by the main loop; keep looping until woken by SIGUSR1.
	}
	return NULL;
} // 子线程监听键盘

int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    signal(SIGUSR1, [](int){});  // needed for clean shutdown: interrupt blocking getchar()
    auto node = std::make_shared<rclcpp::Node>("keyboard_vel_controller");

    ButterWorthFIlter* butterfilter_predicted_x = new ButterWorthFIlter(100, 10);
    ButterWorthFIlter* butterfilter_predicted_y = new ButterWorthFIlter(100, 10);
    ButterWorthFIlter* butterfilter_predicted_z = new ButterWorthFIlter(100, 10);

    ButterWorthFIlter* butterfilter_vel_x = new ButterWorthFIlter(100, 10);
    ButterWorthFIlter* butterfilter_vel_y = new ButterWorthFIlter(100, 10);
    ButterWorthFIlter* butterfilter_vel_z = new ButterWorthFIlter(100, 10);
    
    read_param(node.get());
    const double dt = 1.0 / control_frequency;
    pos_control_x.setDt(dt); pos_control_y.setDt(dt); pos_control_z.setDt(dt);
    vel_control_x.setDt(dt); vel_control_y.setDt(dt); vel_control_z.setDt(dt);
    acc_control_x.setDt(dt); acc_control_y.setDt(dt); acc_control_z.setDt(dt);

    pos_control_x.setLimit(vel_lim.x, -vel_lim.x);
    pos_control_y.setLimit(vel_lim.y, -vel_lim.y);
    pos_control_z.setLimit(vel_lim.z, -vel_lim.z);
    pos_control_x.setPID(pos_pid.x.p,  pos_pid.x.i,  pos_pid.x.d);
    pos_control_y.setPID(pos_pid.y.p,  pos_pid.y.i,  pos_pid.y.d);
    pos_control_z.setPID(pos_pid.z.p,  pos_pid.z.i,  pos_pid.z.d);

    vel_control_x.setPID(vel_pid.x.p,  vel_pid.x.i,  vel_pid.x.d);
    vel_control_y.setPID(vel_pid.y.p,  vel_pid.y.i,  vel_pid.y.d);
    vel_control_z.setPID(vel_pid.z.p,  vel_pid.z.i,  vel_pid.z.d);

    acc_control_x.setLimit(degToAccLimit(att_lim.x), -degToAccLimit(att_lim.x));
    acc_control_y.setLimit(degToAccLimit(att_lim.y), -degToAccLimit(att_lim.y));
    acc_control_z.setLimit(degToAccLimit(att_lim.z), -degToAccLimit(att_lim.z));
    acc_control_x.setPID(acc_pid.x.p,  acc_pid.x.i,  acc_pid.x.d);
    acc_control_y.setPID(acc_pid.y.p,  acc_pid.y.i,  acc_pid.y.d);
    acc_control_z.setPID(acc_pid.z.p,  acc_pid.z.i,  acc_pid.z.d);

    // ROS2 发布器
    auto vel_pub = node->create_publisher<geometry_msgs::msg::Twist>(
        "/mavros/setpoint_velocity/cmd_vel_unstamped", 1);
    auto pos_pub = node->create_publisher<geometry_msgs::msg::PoseStamped>(
        "/mavros/setpoint_position/local", 1);
    auto att_thrust_pub = node->create_publisher<mavros_msgs::msg::AttitudeTarget>(
        "/mavros/setpoint_raw/attitude", 1);
    auto acc_pub = node->create_publisher<mavros_msgs::msg::PositionTarget>(
        "/mavros/setpoint_raw/local", 1);
    auto target_setpositions_pub = node->create_publisher<geometry_msgs::msg::PoseStamped>(
        "/move_base_simple/goal", 1);
    auto command_stop_pub = node->create_publisher<std_msgs::msg::Int64>(
        "/planner_node/stop", 1);

    refTrajectoryPub = node->create_publisher<nav_msgs::msg::Path>(
        "trajectory_publisher/reftrajectory", 1);
    referencePosePub = node->create_publisher<geometry_msgs::msg::PoseStamped>(
        "reference/pose", 1);

    // ROS2 订阅器 — 位姿/速度按 YAML 参数 use_odin_pose 选择数据源
    // 用全局 shared_ptr 持有，防止局部变量析构后回调被切断
    {
        rclcpp::QoS qos_sensor = rclcpp::SensorDataQoS();
        if (g_use_odin_pose) {
            g_pose_sub = node->create_subscription<geometry_msgs::msg::PoseStamped>(
                "/extnav/pose_fcu", qos_sensor, odin_pose_cb);
            g_vel_sub  = node->create_subscription<geometry_msgs::msg::TwistStamped>(
                "/extnav/velocity_fcu", qos_sensor, odin_vel_cb);
            RCLCPP_INFO(node->get_logger(), "Using Odin SLAM pose/vel from extnav_bridge");
        } else {
            g_pose_sub = node->create_subscription<geometry_msgs::msg::PoseStamped>(
                "/mavros/local_position/pose", qos_sensor, pose_cb);
            g_vel_sub  = node->create_subscription<geometry_msgs::msg::TwistStamped>(
                "/mavros/local_position/velocity_local", qos_sensor, vel_cb);
            RCLCPP_INFO(node->get_logger(), "Using MAVROS local_position pose/vel");
        }
    }
    g_imu_sub = node->create_subscription<sensor_msgs::msg::Imu>(
        "/mavros/imu/data", rclcpp::SensorDataQoS(), imudata_cb);
    auto state_sub = node->create_subscription<mavros_msgs::msg::State>(
        "/mavros/state", 1, mavrosState_cb);
    auto pathsub = node->create_subscription<nav_msgs::msg::Path>(
        "/search_node/trajectory_position", 1, pathsub_cb);
    auto nearest_obstacle_distance = node->create_subscription<std_msgs::msg::Float32>(
        "/front_obstacle_distance", 1, nearest_distance_cb);

    // ROS2 服务
    auto arming_client = node->create_client<mavros_msgs::srv::CommandBool>(
        "/mavros/cmd/arming");
    set_mode_client = node->create_client<mavros_msgs::srv::SetMode>(
        "/mavros/set_mode");
    set_frame_client = node->create_client<mavros_msgs::srv::SetMavFrame>(
        "/mavros/setpoint_velocity/mav_frame");
    takeoff_client = node->create_client<mavros_msgs::srv::CommandTOL>(
        "/mavros/cmd/takeoff");

    rclcpp::Rate rate(10);
    rclcpp::Rate rate_control(control_frequency);

    while(rclcpp::ok() && !current_state.connected){
        rclcpp::spin_some(node);
        rate.sleep();
    } // 等待飞控连接

    while ((localAcc.z == 0) && rclcpp::ok()) {
        printf("wait~~~\r\n");
        rclcpp::spin_some(node);
        rate.sleep();
    } // 等待加速度收到

    target_twist.linear.x=0;
    target_twist.linear.y=0;
    target_twist.linear.z=0;
    target_twist.angular.x=0;
    target_twist.angular.y=0;
    target_twist.angular.z=0;
    
    target_pose.pose.position.x = 0;
    target_pose.pose.position.y = 0;
    target_pose.pose.position.z = 0;
    target_pose.pose.orientation.w = 1.0;   // 单位四元数，保持当前航向不变
    target_pose.pose.orientation.x = 0.0;   
    target_pose.pose.orientation.y = 0.0;   
    target_pose.pose.orientation.z = 0.0;  
    
    // 先发几个点
    for(int i = 20; rclcpp::ok() && i > 0; --i){
        rclcpp::spin_some(node);
        rate.sleep();
        pos_pub->publish(target_pose);
    }

    auto guided_req_init = std::make_shared<mavros_msgs::srv::SetMode::Request>();
    guided_req_init->custom_mode = "GUIDED";

    auto arm_req_init = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
    arm_req_init->value = true;

    auto frame_req_init = std::make_shared<mavros_msgs::srv::SetMavFrame::Request>();
    frame_req_init->mav_frame = 1;
    
    // 设置guided模式并解锁
    auto set_mode_future_init = set_mode_client->async_send_request(guided_req_init);
    rclcpp::spin_until_future_complete(node, set_mode_future_init);
    if (set_mode_future_init.get()->mode_sent) {
        printf("[INFO] Guided enabled\n");
        auto arm_future_init = arming_client->async_send_request(arm_req_init);
        rclcpp::spin_until_future_complete(node, arm_future_init);
        if (arm_future_init.get()->success) {
            printf("[INFO] Vehicle armed\n");

            // Set BODY_FRAME - fire-and-forget, non-critical
            auto set_frame_future_init = set_frame_client->async_send_request(frame_req_init);
            // Don't wait — proceed to takeoff immediately

            // Wait for armed state feedback
            while(rclcpp::ok() && !current_state.armed){
                printf("Waiting for unlocking feedback  .....\r\n");
                rclcpp::spin_some(node);
                rate.sleep();
                pos_pub->publish(target_pose);
            }
            // Wait for GUIDED mode feedback
            while(rclcpp::ok() && (current_state.mode != "GUIDED")){
                printf("Waiting for GUIDED mode feedback  .....\r\n");
                rclcpp::spin_some(node);
                rate.sleep();
                pos_pub->publish(target_pose);
            }

            // Create keyboard thread but DON'T block on ctrlMode
            pthread_create(&th, NULL, keyboard_listener, node.get());
            th_created = true;
            ctrlMode = 1;  // proceed to takeoff immediately
        }
        else {
            printf("[INFO] Failed to arm, please retry! \n");
        }
    }else{
        printf("[INFO] Failed to set Guided, please retry! \n");
    }

    // 先发送几个点直到解锁
    while(rclcpp::ok() && !ctrlMode){
        printf("Waiting for pthread_create  .....\r\n");
        rclcpp::spin_some(node);
        rate.sleep();
        pos_pub->publish(target_pose);
    }
    printf("[Ready] Controller Mode On !\n");

    // 尝试起飞（带重试）
    auto takeoff_req = std::make_shared<mavros_msgs::srv::CommandTOL::Request>();
    for (int i = 0; i < 3; ++i) {
        takeoff_req->altitude = 0.3;      // 提高到0.3米，避免高度过小被拒绝
        takeoff_req->min_pitch = 0.0;
        takeoff_req->yaw = 0.0;
        auto takeoff_future = takeoff_client->async_send_request(takeoff_req);
        rclcpp::spin_until_future_complete(node, takeoff_future);
        if (takeoff_future.get()->success) {
            RCLCPP_INFO(rclcpp::get_logger("guided_sim"), "Takeoff succeeded (altitude=1.0m)");
            break;
        } else {
            RCLCPP_WARN(rclcpp::get_logger("guided_sim"), "Takeoff attempt %d failed, retrying...", i+1);
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
    }

    // 等待起飞完成
    printf("Waiting for liftoff...\r\n");
    int liftoff_timeout = 100;  // 10 seconds at 10Hz
    while (rclcpp::ok() && localPos.z < 0.15 && --liftoff_timeout > 0) {
        rclcpp::spin_some(node);
        rate.sleep();
    }
    if (liftoff_timeout <= 0) {
        printf("[WARN] Liftoff timeout — proceeding anyway\n");
    }
    
    while(rclcpp::ok() && ctrlMode){
        if(current_state.mode != "GUIDED"){
            if(guided_change == true){
                guided_change = false;
                mode_int = false; 
                controlMode = MODE_HOVER_CUSTOM; 
                printf("The current mode is %s. Press s, w, and enter in sequence to switch modes\r\n",current_state.mode.c_str());
            }
        }else{
            guided_change = true;
        } // 检测GUIDED模式是否丢失

        if(!current_state.armed){
            if(arm_change == true){
                arm_change = false;
                printf("飞机已经锁定,按任意键退出\r\n");
                ctrlMode = 0;
                break;
            }
        }else{
            arm_change = true;
        } // 检测是否上锁

        if(controlMode == MODE_VELOCITY){
            if(!mode_int){
                target_twist.linear.x=0;
                target_twist.linear.y=0;
                target_twist.linear.z=0;
                // target_twist.angular.x=0;
                // target_twist.angular.y=0;
                target_twist.angular.z=0;
                mode_int = true;
                vel_change = true;
                printf("mode velocity init finish\r\n");
            }else{
                vel_pub->publish(target_twist);
                if(vel_change == true){
                    vel_change = false;
                    printf("mode velocity linear x:%lf,y:%lf,z:%lf,angular:%lf\r\n",target_twist.linear.x,target_twist.linear.y,target_twist.linear.z,target_twist.angular.z);
                }
            }
        }
        else if (controlMode == MODE_POSITION){
            pos_pub->publish(target_pose);
        }
        else if (controlMode == MODE_AUTO){
            if(!mode_int){
                pathpoint_sub.clear();
                targetPointNum = 0;

                // Transform camera (Cartesian) to NED frame
                auto cam = cameraToNED({target_pose.pose.position.x, target_pose.pose.position.y, target_pose.pose.position.z});
                target_pose.pose.position.x = cam.x;
                target_pose.pose.position.y = cam.y;
                
                //发送当前的位置，setTargetPose是通过/goal的topic发送的，以camera_init作为的目标点，向前为x正，向左为y正，向上为z正
                target_setpositions_pub->publish(target_pose);
                
                printf("publish position X:%lf,Y:%lf,Z:%lf \r\n",target_pose.pose.position.x ,target_pose.pose.position.y ,target_pose.pose.position.z );

                //用于没有收到轨迹的时候，发送位置悬停。
                target_pose.pose.position.x = localPos.x;
                target_pose.pose.position.y = localPos.y;
                target_pose.pose.position.z = localPos.z;

                mode_int = true;
            }
            else{
                if(!(pathpoint_sub.empty()) && targetPointNum < pathpoint_sub.size() ){
                    xyz_double_def pathpoint_in_mavros = pathToLocal(pathpoint_sub[targetPointNum]);

                    // 第二步，发送目标点
                    target_pose.pose.position.x = pathpoint_in_mavros.x ;
                    target_pose.pose.position.y = pathpoint_in_mavros.y ;
                    target_pose.pose.position.z = pathpoint_in_mavros.z ;  
                    pos_pub->publish(target_pose);

                        
                        //第三步，计算距离
                        double distance = calculate_distance(pathpoint_in_mavros.x,pathpoint_in_mavros.y,pathpoint_in_mavros.z);

                        printf("set:%d,D %lf,X:%lf,Y:%lf,Z:%lf,mX:%lf,mY:%lf,mZ:%lf\r\n",
                            targetPointNum,distance,
                            pathpoint_in_mavros.x,pathpoint_in_mavros.y,pathpoint_in_mavros.z,
                            localPos.x,localPos.y,localPos.z
                        );                   
                        
                        //第四步，判断距离
                        if(distance < ciecle_range){
                            if(targetPointNum == pathpoint_sub.size() - 1){                             //轨迹上的最后一个点进入圈内
                                if(distance < ciecle_range){
                                    targetPointNum++;                                                   //跟踪到最后一个点，完成本次轨迹跟踪
                                    //悬停时的位置
                                    target_pose.pose.position.x = localPos.x;
                                    target_pose.pose.position.y = localPos.y;
                                    target_pose.pose.position.z = localPos.z;                               
                                }else{
                                    printf("Waiting for the last point to enter the circle!!!!!!!!!!!!\r\n");
                                }
                                
                            }else{
                                targetPointNum = targetPointNum + (int)jump_pointnumber;            //调节跳跃点的数目，用来调节跟踪的速度
                                if(targetPointNum >= pathpoint_sub.size()){
                                    targetPointNum = pathpoint_sub.size() - 1;                          //如果下一个点的轨迹超越了最后一个点，直接设置最后一个点为轨迹跟踪点
                                }
                            }
                        }
                }             
                else{
                    if(targetPointNum == 0){                                //没有收到轨迹
                        pos_pub->publish(target_pose);             //改位置在模式初始化的时候，更新的位置
                        printf("hover no point \r\n");
                    }else if(targetPointNum == pathpoint_sub.size()){       //已经到轨迹终点了
                        pos_pub->publish(target_pose);              //该位置在最后一个点进圈后，更新的位置
                        stop_command.data = 0;
                        command_stop_pub->publish(stop_command);                   
                        printf("finish\r\n");
                    }else{
                        mode_int = false;
                        controlMode = MODE_HOVER_CUSTOM;                           //其他情况，切换到悬停模式
                    }
                }
            }
        }
        else if(controlMode == MODE_AUTO_POSE){
            if(!mode_int){
                pathpoint_sub.clear();
                targetPointNum = 0;
                double xt = target_pose.pose.position.x;
                double yt = target_pose.pose.position.y;
                double zt = target_pose.pose.position.z;

                //开始计算轨迹点
                double lx = localPos.x;
                double ly = localPos.y;
                double lz = localPos.z; 

                double distance = sqrt(pow((xt-lx),2)+pow((yt-ly),2)+pow((zt-lz),2)); 
                int num = (int)(distance/0.02f);

                double x_1 = (xt - lx) / num;           //num=100   xt-lx=2   0.02
                double y_1 = (yt - ly) / num;
                double z_1 = (zt - lz) / num;

                // Transform camera (Cartesian) to NED frame
                auto cam = cameraToNED({target_pose.pose.position.x, target_pose.pose.position.y, target_pose.pose.position.z});
                target_pose.pose.position.x = cam.x;
                target_pose.pose.position.y = cam.y;
                
                //发送当前的位置，setTargetPose是通过/goal的topic发送的，以camera_init作为的目标点，向前为x正，向左为y正，向上为z正
                target_setpositions_pub->publish(target_pose);
                
                printf("publish position X:%lf,Y:%lf,Z:%lf \r\n",target_pose.pose.position.x ,target_pose.pose.position.y ,target_pose.pose.position.z );

                //用于没有收到轨迹的时候，发送位置悬停。
                target_pose.pose.position.x = localPos.x;
                target_pose.pose.position.y = localPos.y;
                target_pose.pose.position.z = localPos.z;

                target_acc_x_pid_last = 0.0f;
                target_acc_y_pid_last = 0.0f;
                target_acc_z_pid_last = 0.0f;

                target_vel_x_pid_last = 0.0f;
                target_vel_y_pid_last = 0.0f;
                target_vel_z_pid_last = 0.0f;

                mode_int = true;
            }else{
                static bool jiansu_flag = false; 
                if(!(pathpoint_sub.empty()) && targetPointNum < pathpoint_sub.size() && (pathpoint_sub.size() > jump_pointnumber + 1)){
                    if(targetPointNum == 0){                                    //收到轨迹的第一个点跟踪，采用飞控的位置控制
                        xyz_double_def first_path = pathpoint_sub[0];
                        target_pose.pose.position.x = -first_path.y;
                        target_pose.pose.position.y = first_path.x;
                        target_pose.pose.position.z = first_path.z;

                        pos_pub->publish(target_pose);
                        double distance = calculate_distance(-first_path.y,first_path.x,first_path.z);
                        if(distance <= 0.1f){
                            targetPointNum = 1;
                            PID_Reset();                  

                            printf("first point tracking finish %lf %lf %lf\r\n",target_pose.pose.position.x,target_pose.pose.position.y,target_pose.pose.position.z);
                        } 
                        jiansu_flag = false;
                    }else{

                    //遍历计算所有点，计算与当前位置最近的点
                        double min_distance = 1000000.0f;
                        double lx = localPos.y;
                        double ly = -localPos.x;
                        double lz = localPos.z;
                        int min_targetPointNum = 0;

                        for(int i=0;i<pathpoint_sub.size();i++){
                            double dis = pow((ly-pathpoint_sub[i].y),2) + pow((lx-pathpoint_sub[i].x),2) + pow((lz-pathpoint_sub[i].z),2);
                            if(dis < min_distance){
                                min_distance = dis;
                                min_targetPointNum = i;
                                targetPointNum = min_targetPointNum;
                                if(targetPointNum == 0){
                                    targetPointNum =1;
                                }
                            }
                        }

                        double distance_finish = sqrt(pow((lx-pathpoint_sub[pathpoint_sub.size()-1].x),2) + pow((ly-pathpoint_sub[pathpoint_sub.size()-1].y),2) + pow((lz-pathpoint_sub[pathpoint_sub.size()-1].z),2)); 


                        //判断最后一个点
                        if(pathpoint_sub.size()-30 > 0){
                            if(min_targetPointNum >= pathpoint_sub.size()-30){
                                targetPointNum = min_targetPointNum = pathpoint_sub.size()-30; //最后一个点，用上一个点作跟踪
                            }
                        }else{
                                targetPointNum = min_targetPointNum = pathpoint_sub.size() - 1 - jump_pointnumber;
                        }

                        
                        //最近点靠后了 前向判断
                        if(min_targetPointNum <= targetPointNum){
                            min_targetPointNum = targetPointNum;
                        }

                        // desired point
                        xyz_double_def pathpoint_in_mavros      = pathToLocal(pathpoint_sub[min_targetPointNum]);
                        xyz_double_def pathpoint_in_mavros_next = pathToLocal(pathpoint_sub[min_targetPointNum + jump_pointnumber]);                   

                    
                        target_position_show.header.stamp = node->now();
                        target_position_show.header.frame_id = "map";
                        target_position_show.pose.position.x = pathpoint_in_mavros.x;
                        target_position_show.pose.position.y = pathpoint_in_mavros.y;
                        target_position_show.pose.position.z = pathpoint_in_mavros.z;
                        target_position_show.pose.orientation.w = localAtt.w;
                        target_position_show.pose.orientation.x = localAtt.x;
                        target_position_show.pose.orientation.y = localAtt.y;
                        target_position_show.pose.orientation.z = localAtt.z;
                        referencePosePub->publish(target_position_show);

                        //获取轨迹上的feedforward速度方向，即做速度跟随的主方向
                        xyz_double_def target_vel_main_direction = {   
                            (pathpoint_in_mavros_next.x - pathpoint_in_mavros.x),
                            (pathpoint_in_mavros_next.y - pathpoint_in_mavros.y),
                            (pathpoint_in_mavros_next.z - pathpoint_in_mavros.z) 
                        };   
                        //计算目标的前向速度大小
                        xyz_double_def target_velocity_main ;
                        // target_velocity_main = calculate_FixedModulusVector(target_vel_main_direction,vel_forward_set);   //目标前向速度

                        {
                            if((distance_finish < 1.0f) || (jiansu_flag)){         //当前位置距离终点只有1m的距离，则开始执行减速操作
                                jiansu_flag = true;
                                //这里需要注意，前向速度必须要大于1.0才行
                                target_velocity_main = calculate_FixedModulusVector(target_vel_main_direction,distance_finish*(1/vel_forward_set));//目标前向速度与距离成正相关函数
                                // target_velocity_main = calculate_FixedModulusVector(target_vel_main_direction,vel_forward_set);
                            } else{
                                if(obstacle_distance > 2.5f){
                                    target_velocity_main = calculate_FixedModulusVector(target_vel_main_direction,vel_forward_set);   //目标前向速度为feedforward速度
                                
                                }else{
                                    // target_velocity_main = calculate_FixedModulusVector(target_vel_main_direction,0.8f);   //目标前向速度为设定速度
                                    target_velocity_main = calculate_FixedModulusVector(target_vel_main_direction,vel_forward_set);   //目标前向速度为设定速度
                                
                                }                            
                                // target_velocity_main = calculate_FixedModulusVector(target_vel_main_direction,vel_forward_set);   //目标前向速度为设定速度                               
                            }
                        }

                        Eigen::Quaterniond eigen_local_q_(localAtt.w, localAtt.x, localAtt.y, localAtt.z);
                        Eigen::Vector3d body_acc_(localAcc.x, localAcc.y, localAcc.z);
                        Eigen::Vector3d local_acc_ = eigen_local_q_*body_acc_;      //////////error

                        double jumptime = (double)jump_pointnumber*(1.0f/(double)control_frequency);

                        xyz_double_def predicted_position = {                //预测飞机下一步的位置
                            localPos.x + localVel.x * jumptime,
                            localPos.y + localVel.y * jumptime,
                            localPos.z + localVel.z * jumptime
                        };

                        xyz_double_def predicted_position_fil = {
                            butterfilter_predicted_x->filter(predicted_position.x),
                            butterfilter_predicted_y->filter(predicted_position.y),
                            butterfilter_predicted_z->filter(predicted_position.z)
                        };        

                        // xld  20260115
                        double target_vel_x_h_pid = 1.2f* pos_control_x.calculate(localPos.x, pathpoint_in_mavros.x);              
                        double target_vel_y_h_pid = 1.2f* pos_control_y.calculate(localPos.y, pathpoint_in_mavros.y);              
                        double target_vel_z_h_pid = 1.2f* pos_control_z.calculate(localPos.z, pathpoint_in_mavros.z); 

                        double target_vel_x_h_pid_LPF =  1.0f * pos_control_x.calculate_LPF(target_vel_x_h_pid, 0.0f); 
                        double target_vel_y_h_pid_LPF =  1.0f * pos_control_y.calculate_LPF(target_vel_y_h_pid, 0.0f);    
                        double target_vel_z_h_pid_LPF = pos_control_z.calculate_LPF(target_vel_z_h_pid, 0.0f);  

                        double target_vel_x_h_pid_BUTTER =  butterfilter_vel_x->filter(target_vel_x_h_pid);
                        double target_vel_y_h_pid_BUTTER =  butterfilter_vel_y->filter(target_vel_y_h_pid);
                        double target_vel_z_h_pid_BUTTER =  butterfilter_vel_z->filter(target_vel_z_h_pid);  

                        double target_vel_x_pid = target_vel_x_h_pid_BUTTER + target_velocity_main.x;
                        double target_vel_y_pid = target_vel_y_h_pid_BUTTER + target_velocity_main.y;
                        double target_vel_z_pid = target_vel_z_h_pid_BUTTER + target_velocity_main.z;         

                        const double DELT_RANGE_VEL = 0.02;
                        target_vel_x_pid = slew_rate_limit(target_vel_x_pid, target_vel_x_pid_last, DELT_RANGE_VEL);
                        target_vel_y_pid = slew_rate_limit(target_vel_y_pid, target_vel_y_pid_last, DELT_RANGE_VEL);
                        target_vel_z_pid = slew_rate_limit(target_vel_z_pid, target_vel_z_pid_last, DELT_RANGE_VEL);

                        target_vel_x_pid_last = target_vel_x_pid;
                        target_vel_y_pid_last = target_vel_y_pid;
                        target_vel_z_pid_last = target_vel_z_pid;

                        double target_acc_x_pid = acc_control_x.calculate(localVel.x,target_vel_x_pid) ;                         //PID的系数设置为0.1~10之间，加速度限制在-5.685~5.685之间，取上下限-6~6之间
                        double target_acc_y_pid = acc_control_y.calculate(localVel.y,target_vel_y_pid) ;                         //预设最大P值10时，速度相差5m/s的时候，最大值输出。
                        double target_acc_z_pid = acc_control_z.calculate(localVel.z,target_vel_z_pid) ;                         //即，10*5 * 0.12 = 6


                        const double DELT_RANGE = degToAccLimit(5.0);
                        target_acc_x_pid = slew_rate_limit(target_acc_x_pid, target_acc_x_pid_last, DELT_RANGE);
                        target_acc_y_pid = slew_rate_limit(target_acc_y_pid, target_acc_y_pid_last, DELT_RANGE);
                        target_acc_z_pid = slew_rate_limit(target_acc_z_pid, target_acc_z_pid_last, DELT_RANGE);

                        target_acc_x_pid_last = target_acc_x_pid;
                        target_acc_y_pid_last = target_acc_y_pid;
                        target_acc_z_pid_last = target_acc_z_pid;

                        //转换到机体坐标系下，将目标加速度转换成飞机的目标姿态
                        Eigen::Quaterniond q(localAtt.w, localAtt.x, localAtt.y, localAtt.z);                                   //当前姿态四元素，转换为欧拉角
                        double yaw,roll,pitch;
                        toEulerAngle(q,roll,pitch,yaw);

                        //转换到机体坐标系下
                        double target_acc_x_pid_in_body = target_acc_x_pid * cos(yaw) + target_acc_y_pid * sin(yaw);
                        double target_acc_y_pid_in_body = -target_acc_x_pid * sin(yaw) + target_acc_y_pid * cos(yaw);

                        //换算成角度
                        double target_pitch_pid =  atan2(target_acc_x_pid_in_body,GRAVITY_ACC);                                         //加速度转换成姿态
                        double target_roll_pid  = -atan2(target_acc_y_pid_in_body,GRAVITY_ACC);                                         //根据右手法则，roll的目标差值需要反向
                        double target_height_pid = target_acc_z_pid * uav_weight / (cos(target_pitch_pid) * cos(target_roll_pid)) *0.2f;                                         //高度不做控制


                        //欧拉角转换成四元素 发送出去，航向角不变，
                        fillAttitudeTarget(target_att_thrust, target_roll_pid, target_pitch_pid,
                                          target_euler.yaw, 0.43, target_height_pid, node.get());

                        // xld 2026.1.14
                        att_thrust_pub->publish(target_att_thrust);

                        double vel_xy = sqrt(pow(localVel.x,2)+pow(localVel.y,2)+pow(localVel.z,2));

                        double vel_xy_main = sqrt(pow(target_velocity_main.x,2)+pow(target_velocity_main.y,2)+pow(target_velocity_main.z,2));
                        double vel_xy_heng = sqrt(pow(target_vel_x_h_pid,2)+pow(target_vel_y_h_pid,2)+pow(target_vel_z_h_pid,2));
                        double distance = calculate_distance(-pathpoint_sub[min_targetPointNum].y,pathpoint_sub[min_targetPointNum].x,pathpoint_sub[min_targetPointNum].z); 

                        printf("traj num:%d,D:%lf  vel_norm:%lf  vel_ff_norm:%lf  x:%lf  y:%lf  z:%lf  vx_ff:%lf  vy_ff:%lf  vz_ff:%lf vx_fb:%lf  vy_fb:%lf  vz_fb:%lf vx:%lf  vy:%lf  vz:%lf  x_des:%lf  y_des:%lf  z_des:%lf  vx_des:%lf  vy_des:%lf  vz_des:%lf\r\n",
                        min_targetPointNum,distance,vel_xy,vel_xy_main,
                        localPos.x,localPos.y,localPos.z,
                        target_velocity_main.x,target_velocity_main.y,target_velocity_main.z,
                        target_vel_x_h_pid_BUTTER, target_vel_y_h_pid_BUTTER,target_vel_z_h_pid_BUTTER,
                        localVel.x,localVel.y,localVel.z,
                        pathpoint_in_mavros_next.x,pathpoint_in_mavros_next.y,pathpoint_in_mavros_next.z,
                        target_vel_x_pid,target_vel_y_pid,target_vel_z_pid
                        ); 



                        if((pathpoint_sub.size() - 30) > 0 ){
                            if(targetPointNum == pathpoint_sub.size() - 30){                             //轨迹上的最后一个点进入圈内
                                targetPointNum = pathpoint_sub.size();     
                                // target_pose.pose.position.x = localPos.x;
                                // target_pose.pose.position.y = localPos.y;
                                // target_pose.pose.position.z = localPos.z;  

                                auto pt = pathToLocal(pathpoint_sub[pathpoint_sub.size() - 1]);
                                target_pose.pose.position.x = pt.x;
                                target_pose.pose.position.y = pt.y;
                                target_pose.pose.position.z = pt.z;    
                            }
                        }else{

                           
                            if(targetPointNum ==  (pathpoint_sub.size() - 1 - jump_pointnumber)){          //轨迹上的最后一个点进入圈内
                                targetPointNum = pathpoint_sub.size();     
                                // target_pose.pose.position.x = localPos.x;
                                // target_pose.pose.position.y = localPos.y;
                                // target_pose.pose.position.z = localPos.z;  

                                auto pt2 = pathToLocal(pathpoint_sub[pathpoint_sub.size() - 1]);
                                target_pose.pose.position.x = pt2.x;
                                target_pose.pose.position.y = pt2.y;
                                target_pose.pose.position.z = pt2.z;    
                            }                            
                        }
                    }
                }             
                else{
                    if(targetPointNum == 0){                                //没有收到轨迹
                        pos_pub->publish(target_pose);             //改位置在模式初始化的时候，更新的位置
                        if(pathpoint_sub.size() == 0){
                            printf("hover no point \r\n");
                        }else{
                            printf("point too small \r\n");
                        }
                        

                    }else if(targetPointNum == pathpoint_sub.size()){       //已经到轨迹终点了
                        pos_pub->publish(target_pose);              //该位置在最后一个点进圈后，更新的位置
                        stop_command.data = 0;
                        command_stop_pub->publish(stop_command);                   
                        printf("finish~~~ tx:%lf ty:%lf tz:%lf lx:%lf ly:%lf lz:%lf \r\n",target_pose.pose.position.x,target_pose.pose.position.y,target_pose.pose.position.z,localPos.x,localPos.y,localPos.z);


                    }else{
                        mode_int = false;
                        controlMode = MODE_HOVER_CUSTOM;                           //其他情况，切换到悬停模式
                    }
                }
            }
        }
        else if (controlMode == MODE_STRAIGHT){
            if(!mode_int){
                PID_Reset();    
                mode_int = true;
            }
            else
            {
                double target_vel_x_pid = pos_control_x.calculate(localPos.x,target_pose.pose.position.x);               //PID的系数设置为0.1~10 之间，速度现在-1~1之间
                double target_vel_y_pid = pos_control_y.calculate(localPos.y,target_pose.pose.position.y);               //预设最大P值10时，距离相差1m的时候，最大值输出。
                double target_vel_z_pid = pos_control_z.calculate(localPos.z,target_pose.pose.position.z);               //即，10*1*0.1 = 1m/s

                double target_acc_x_pid = vel_control_x.calculate(localVel.x,target_vel_x_pid) ;                         //PID的系数设置为0.1~10之间，加速度限制在-5.685~5.685之间，取上下限-6~6之间
                double target_acc_y_pid = vel_control_y.calculate(localVel.y,target_vel_y_pid) ;                         //预设最大P值10时，速度相差5m/s的时候，最大值输出。
                double target_acc_z_pid = vel_control_z.calculate(localVel.z,target_vel_z_pid) ;                         //即，10*5 * 0.12 = 6

 
                double max_acc = 5.0f;
                double max_x_acc = abs(max_acc * (target_acc_x_pid / sqrt(target_acc_x_pid*target_acc_x_pid+target_acc_y_pid*target_acc_y_pid))) ;
                double max_y_acc = abs( max_acc * (target_acc_y_pid / sqrt(target_acc_x_pid*target_acc_x_pid+target_acc_y_pid*target_acc_y_pid)));

                target_acc_x_pid = clamp_symmetric(target_acc_x_pid, degToAccLimit(max_x_acc));
                target_acc_y_pid = clamp_symmetric(target_acc_y_pid, degToAccLimit(max_y_acc));
                target_acc_z_pid = clamp_symmetric(target_acc_z_pid, degToAccLimit(10.0f));   

 
                //以上代码target_vel_x_pid，target_acc_x_pid都为与local的坐标系下，即东北天坐标系

                Eigen::Quaterniond q(localAtt.w, localAtt.x, localAtt.y, localAtt.z);                                   //当前姿态四元素，转换为欧拉角
                double yaw,roll,pitch;
                toEulerAngle(q,roll,pitch,yaw);     

                //转换到机体坐标系下
                double target_acc_x_pid_in_body = target_acc_x_pid * cos(yaw) + target_acc_y_pid * sin(yaw);
                double target_acc_y_pid_in_body = -target_acc_x_pid * sin(yaw) + target_acc_y_pid * cos(yaw);

                //换算成角度
                double target_pitch_pid =  atan2(target_acc_x_pid_in_body,GRAVITY_ACC);                                         //加速度转换成姿态
                double target_roll_pid  = -atan2(target_acc_y_pid_in_body,GRAVITY_ACC);                                         //根据右手法则，roll的目标差值需要反向
                double target_height_pid = target_acc_z_pid * uav_weight / (cos(target_pitch_pid) * cos(target_roll_pid)) *0.2f;                                         //高度不做控制


                //欧拉角转换成四元素 发送出去，航向角不变，
                fillAttitudeTarget(target_att_thrust, target_roll_pid, target_pitch_pid,
                                  target_euler.yaw, 0.43, target_height_pid, node.get());

                att_thrust_pub->publish(target_att_thrust);                                            //发送消息
                
                double distance = calculate_distance(target_pose.pose.position.x,target_pose.pose.position.y,target_pose.pose.position.z);

                printf("target_vel x:%lf,y:%lf,z:%lf\r\n",target_vel_x_pid,target_vel_y_pid,target_vel_z_pid);
                printf("target_acc x:%lf,y:%lf,z:%lf\r\n",target_acc_x_pid,target_acc_y_pid,target_acc_z_pid);
                printf("localVel x:%lf,y:%lf,z:%lf\r\n",localVel.x,localVel.y,localVel.z);
                printf("MODE_STRAIGHT: D:%lf,current_position x:%lf,y:%lf,z:%lf target_ouer r:%lf,p:%lf,y:%f,thu:%f,orientation x:%lf,y:%lf,z:%lf,w:%lf\r\n",
                distance,
                localPos.x,localPos.y,localPos.z,
                target_pitch_pid * 180.0f/M_PI,target_roll_pid * 180.0f/M_PI,target_euler.yaw,
                target_att_thrust.thrust,
                target_att_thrust.orientation.x,target_att_thrust.orientation.y,target_att_thrust.orientation.z,target_att_thrust.orientation.w
                );
            }
        }
        else if (controlMode == TEST_FLOOR_TRAJ){
            if(!mode_int){
                pathpoint_sub_test.clear();
                targetPointNum_test = 0;

                // 总的航迹点
                {
                    double XLOCAL = target_x;
                    double YLOCAL = target_y;
                    // #define ANGLE_DIFF      0.0f
                    double ANGLE_DIFF;
                    double abs_x;
                    
                    if(XLOCAL > 0){
                        ANGLE_DIFF = atan2(XLOCAL,YLOCAL);
                        ANGLE_DIFF = -ANGLE_DIFF;
                    }else{
                        abs_x = -XLOCAL;
                        ANGLE_DIFF = atan2(abs_x,YLOCAL);
                    }                    
                    pathpoint_sub_test.clear();

                    pathpoint_sub_test.push_back({  0 + localPos.x,             //rrrrrrrree
                                                    0 + localPos.y,
                                                    0 + localPos.z,
                                                    M_PI/2 + ANGLE_DIFF
                    });                    

                    pathpoint_sub_test.push_back({  XLOCAL + localPos.x,             //rrrrrrrree
                                                    YLOCAL + localPos.y,
                                                    0 + localPos.z,
                                                    M_PI/2 + ANGLE_DIFF
                    });

                    pathpoint_sub_test.push_back({ XLOCAL + localPos.x,             //rrrrrrrree
                                                    YLOCAL + localPos.y,
                                                    0.5 + localPos.z,
                                                    M_PI*3/2 + ANGLE_DIFF
                    });

                    pathpoint_sub_test.push_back({ 0 + localPos.x,             //rrrrrrrree
                                                    0 + localPos.y,
                                                    0.5 + localPos.z,
                                                    M_PI*3/2 + ANGLE_DIFF
                    });

                    pathpoint_sub_test.push_back({ 0 + localPos.x,             //rrrrrrrree
                                                    0 + localPos.y,
                                                    1.0 + localPos.z,
                                                    M_PI/2 + ANGLE_DIFF
                    });

                    pathpoint_sub_test.push_back({ XLOCAL + localPos.x,             //rrrrrrrree
                                                    YLOCAL + localPos.y,
                                                    1.0 + localPos.z,
                                                    M_PI/2 + ANGLE_DIFF
                    });

                    pathpoint_sub_test.push_back({ XLOCAL + localPos.x,             //rrrrrrrree
                                                    YLOCAL + localPos.y,
                                                    1.5 + localPos.z,
                                                    M_PI*3/2 + ANGLE_DIFF
                    });


                    pathpoint_sub_test.push_back({ 0 + localPos.x,             //rrrrrrrree
                                                    0 + localPos.y,
                                                    1.5 + localPos.z,
                                                    M_PI*3/2 + ANGLE_DIFF
                    });
                }

                target_vel_x_pid_last = 0.0f;
                target_vel_y_pid_last = 0.0f;
                target_vel_z_pid_last = 0.0f;

                target_acc_x_pid_last = 0.0f;
                target_acc_y_pid_last = 0.0f;
                target_acc_z_pid_last = 0.0f;

                mode_int = true;
            }
            else{
                if(!(pathpoint_sub_test.empty()) && targetPointNum_test < pathpoint_sub_test.size() ){

                    //1.1 判断当前跟踪点是前向飞行还是转头的    重要逻辑： 在转向的时候，不设置避障
                    if(targetPointNum_test > 0){
                        if(abs(pathpoint_sub_test[targetPointNum_test].a - pathpoint_sub_test[targetPointNum_test - 1].a) > (90.0*M_PI/180)){
                            turnDirection = 1;  //turnDirection 0: 表示不转向  1: 表示转向
                        }else{
                            turnDirection = 0;
                        }
                    }else{

                        Eigen::Quaterniond q(localAtt.w, localAtt.x, localAtt.y, localAtt.z);                                   //当前姿态四元素，转换为欧拉角
                        double yaw,roll,pitch;
                        toEulerAngle(q,roll,pitch,yaw);  

                        static double init_yaw = yaw;

                        if(abs(pathpoint_sub_test[0].a - init_yaw) > (5.0*M_PI/180)){
                            turnDirection = 1;
                        }else{
                            turnDirection = 0;
                            targetPointNum_test = 1;
                        }
                    }

                    static bool isFillTrajectory = true;
                    //1.2 如果是转头的，则进行转头程序
                    if(turnDirection == 1){
                        if (isFillTrajectory){
                            double xt = pathpoint_sub_test[targetPointNum_test].x;
                            double yt = pathpoint_sub_test[targetPointNum_test].y;
                            double zt = pathpoint_sub_test[targetPointNum_test].z;
                            double yawt = pathpoint_sub_test[targetPointNum_test].a;

                            //开始计算轨迹点
                            double lx, ly, lz, lyaw;
                            if(targetPointNum_test == 0){
                                 lx = localPos.x;    
                                 ly = localPos.y;
                                 lz = localPos.z;                                 
                            }else{
                                 lx = pathpoint_sub_test[targetPointNum_test-1].x;   
                                 ly = pathpoint_sub_test[targetPointNum_test-1].y;
                                 lz = pathpoint_sub_test[targetPointNum_test-1].z;
                            }
                            //转换到机体坐标系下，将目标加速度转换成飞机的目标姿态
                            Eigen::Quaterniond q_1(localAtt.w, localAtt.x, localAtt.y, localAtt.z);                                   //当前姿态四元素，转换为欧拉角
                            double lroll,lpitch;
                            toEulerAngle(q_1,lroll,lpitch,lyaw);

                            double distance = sqrt(pow((xt-lx),2)+pow((yt-ly),2)+pow((zt-lz),2)); 

                            int num ;

                            double x_1 ;          //num=100   xt-lx=2   0.02
                            double y_1 ;
                            double z_1 ; 
                            double yaw_1;  

                            if(distance < 0.001f){
                                x_1 = 0;
                                y_1 = 0;
                                z_1 = 0;
                                yaw_1 = 0;
                                num = 1;
                            }else{
                                num = (int)(distance/0.02f);
                                x_1 = (xt - lx) / num;           //num=100   xt-lx=2   0.02
                                y_1 = (yt - ly) / num;
                                z_1 = (zt - lz) / num; 
                                yaw_1 =  M_PI / num;                            
                            }

                            fill_pathpoint.clear();
                            fill_yaw.clear();
                            for(int i = 0;i<= num;i++){ 
                                fill_pathpoint.push_back({ lx + x_1 * i,
                                                            ly + y_1 * i,
                                                            lz + z_1 * i});
                                fill_yaw.push_back(lyaw + yaw_1 * i);
                            } 
                            isFillTrajectory = false;    
                        }

                        if(!(fill_yaw.empty())){  
                            double min_distance = 1000000.0f;
                            double ly = localPos.y;
                            double lx = localPos.x;
                            double lz = localPos.z;
                            int min_targetPointNum = 0;
                            for(int i=0;i<fill_pathpoint.size();i++){
                                double dis = pow((ly-fill_pathpoint[i].y),2) + pow((lx-fill_pathpoint[i].x),2) + pow((lz-fill_pathpoint[i].z),2);
                                if(dis < min_distance){
                                    min_distance = dis;
                                    min_targetPointNum = i;
                                }
                            }

                            //获取轨迹上的速度方向，即做速度跟随的主方向
                            xyz_double_def target_vel_main_direction;
                            if(min_targetPointNum == (fill_pathpoint.size()-1)){
                                target_vel_main_direction = {   
                                    (fill_pathpoint[fill_pathpoint.size()-1].x - fill_pathpoint[fill_pathpoint.size()-2].x),
                                    (fill_pathpoint[fill_pathpoint.size()-1].y - fill_pathpoint[fill_pathpoint.size()-2].y),
                                    (fill_pathpoint[fill_pathpoint.size()-1].z - fill_pathpoint[fill_pathpoint.size()-2].z) 
                                };
                            }else if((min_targetPointNum + 1) > (fill_pathpoint.size()-1)){
                                target_vel_main_direction = {   
                                    (fill_pathpoint[fill_pathpoint.size()-1].x - fill_pathpoint[min_targetPointNum].x),
                                    (fill_pathpoint[fill_pathpoint.size()-1].y - fill_pathpoint[min_targetPointNum].y),
                                    (fill_pathpoint[fill_pathpoint.size()-1].z - fill_pathpoint[min_targetPointNum].z) 
                                }; 
                            }else{
                                target_vel_main_direction = {   
                                    (fill_pathpoint[min_targetPointNum+1].x - fill_pathpoint[min_targetPointNum].x),
                                    (fill_pathpoint[min_targetPointNum+1].y - fill_pathpoint[min_targetPointNum].y),
                                    (fill_pathpoint[min_targetPointNum+1].z - fill_pathpoint[min_targetPointNum].z) 
                                }; 
                            }

                            //计算目标的前向速度大小
                            xyz_double_def target_velocity_main = calculate_FixedModulusVector(target_vel_main_direction, 0.1);   //目标前向速度

                            Eigen::Quaterniond eigen_local_q_(localAtt.w, localAtt.x, localAtt.y, localAtt.z);
                            Eigen::Vector3d body_acc_(localAcc.x, localAcc.y, localAcc.z);
                            Eigen::Vector3d local_acc_ = eigen_local_q_*body_acc_;      //////////error

                            int forwardPoint = min_targetPointNum;        //预测飞机跟踪轨迹的点位
                            if(forwardPoint + 1 <= fill_pathpoint.size()-1){
                                forwardPoint = forwardPoint + 1;             
                            }else{
                                forwardPoint =  fill_pathpoint.size()-1;
                            }                                

                            double target_vel_x_h_pid = 1.0f* pos_control_x.calculate(localPos.x, fill_pathpoint[forwardPoint].x);              
                            double target_vel_y_h_pid = 1.0f* pos_control_y.calculate(localPos.y, fill_pathpoint[forwardPoint].y);              
                            double target_vel_z_h_pid = 1.0f* pos_control_z.calculate(localPos.z, fill_pathpoint[forwardPoint].z);  

                            double target_vel_x_h_pid_BUTTER =  butterfilter_vel_x->filter(target_vel_x_h_pid);
                            double target_vel_y_h_pid_BUTTER =  butterfilter_vel_y->filter(target_vel_y_h_pid);
                            double target_vel_z_h_pid_BUTTER =  butterfilter_vel_z->filter(target_vel_z_h_pid);

                            //横向控制做预测机结束
                            double target_vel_x_pid = target_vel_x_h_pid_BUTTER + target_velocity_main.x;
                            double target_vel_y_pid = target_vel_y_h_pid_BUTTER + target_velocity_main.y;  
                            double target_vel_z_pid = target_vel_z_h_pid_BUTTER + target_velocity_main.z;                          

                            double target_acc_x_pid = acc_control_x.calculate(localVel.x,target_vel_x_pid);                         //PID的系数设置为0.1~10之间，加速度限制在-5.685~5.685之间，取上下限-6~6之间
                            double target_acc_y_pid = acc_control_y.calculate(localVel.y,target_vel_y_pid);                         //预设最大P值10时，速度相差5m/s的时候，最大值输出。
                            double target_acc_z_pid = acc_control_z.calculate(localVel.z,target_vel_z_pid);  

                            const double DELT_RANGE_2 = degToAccLimit(5.0);
                            target_acc_x_pid = slew_rate_limit(target_acc_x_pid, target_acc_x_pid_last, DELT_RANGE_2);
                            target_acc_y_pid = slew_rate_limit(target_acc_y_pid, target_acc_y_pid_last, DELT_RANGE_2);
                            target_acc_z_pid = slew_rate_limit(target_acc_z_pid, target_acc_z_pid_last, DELT_RANGE_2);

                            target_acc_x_pid_last = target_acc_x_pid;
                            target_acc_y_pid_last = target_acc_y_pid;
                            target_acc_z_pid_last = target_acc_z_pid;

                            //转换到机体坐标系下，将目标加速度转换成飞机的目标姿态
                            Eigen::Quaterniond q(localAtt.w, localAtt.x, localAtt.y, localAtt.z);                                   //当前姿态四元素，转换为欧拉角
                            double yaw,roll,pitch;
                            toEulerAngle(q,roll,pitch,yaw);

                            //转换到机体坐标系下
                            double target_acc_x_pid_in_body = target_acc_x_pid * cos(yaw) + target_acc_y_pid * sin(yaw);
                            double target_acc_y_pid_in_body = -target_acc_x_pid * sin(yaw) + target_acc_y_pid * cos(yaw);

                            //换算成角度
                            double target_pitch_pid =  atan2(target_acc_x_pid_in_body,GRAVITY_ACC);                                         //加速度转换成姿态
                            double target_roll_pid  = -atan2(target_acc_y_pid_in_body,GRAVITY_ACC);                                         //根据右手法则，roll的目标差值需要反向
                            double target_height_pid = target_acc_z_pid * uav_weight / (cos(target_pitch_pid) * cos(target_roll_pid)) *0.2f;       //高度不做控制

                            //欧拉角转换成四元素 发送出去，航向角不变，
                            fillAttitudeTarget(target_att_thrust, target_roll_pid, target_pitch_pid,
                                              fill_yaw[forwardPoint], 0.43, target_height_pid, node.get());

                            att_thrust_pub->publish(target_att_thrust);    //ZZXZZX              
                        
                            //第三步，计算距离,计算航向
                            double distance = calculate_distance(pathpoint_sub_test[targetPointNum_test].x,pathpoint_sub_test[targetPointNum_test].y,pathpoint_sub_test[targetPointNum_test].z);

                            double angulardiff = calculate_AngleDifference(pathpoint_sub_test[targetPointNum_test].a,yaw);

                            printf("pure_yaw_set:%d,D %lf,tA:%lf,X:%lf,Y:%lf,Z:%lf,mX:%lf,mY:%lf,mZ:%lf\r\n",
                                targetPointNum_test,distance,fill_yaw[forwardPoint]*180/M_PI,
                                pathpoint_sub_test[targetPointNum_test].x,pathpoint_sub_test[targetPointNum_test].y,pathpoint_sub_test[targetPointNum_test].z,
                                localPos.x,localPos.y,localPos.z
                            );                   
                        
                            //第四步，判断距离和航向
                            if((distance < ciecle_range )  && (angulardiff < (5.0*M_PI/180))){      
                                targetPointNum_test = targetPointNum_test + 1;            //调节跳跃点的数目，用来调节跟踪的速度
                                isFillTrajectory = true;
                            }                        
                    }
                    }
                    
                    //1.3 如果是直线的，则填充与目标点之间的空闲点,，执行直线飞行程序
                    if(turnDirection == 0){
                        // 1.3.1 填充轨迹点
                        if(isFillTrajectory){

                            double xt = pathpoint_sub_test[targetPointNum_test].x;
                            double yt = pathpoint_sub_test[targetPointNum_test].y;
                            double zt = pathpoint_sub_test[targetPointNum_test].z;

                            //开始计算轨迹点
                            double lx, ly, lz;
                            if(targetPointNum_test == 0){
                                 lx = localPos.x;    
                                 ly = localPos.y;
                                 lz = localPos.z;                                 
                            }else{
                                 lx = pathpoint_sub_test[targetPointNum_test-1].x;   
                                 ly = pathpoint_sub_test[targetPointNum_test-1].y;
                                 lz = pathpoint_sub_test[targetPointNum_test-1].z;
                            }

                            double distance = sqrt(pow((xt-lx),2)+pow((yt-ly),2)+pow((zt-lz),2)); 

                            int num ;

                            double x_1 ;          //num=100   xt-lx=2   0.02
                            double y_1 ;
                            double z_1 ; 

                            if(distance < 0.001f){
                                x_1 = 0;
                                y_1 = 0;
                                z_1 = 0;
                                num = 1;
                            }else{
                                num = (int)(distance/0.02f);

                                x_1 = (xt - lx) / num;           //num=100   xt-lx=2   0.02
                                y_1 = (yt - ly) / num;
                                z_1 = (zt - lz) / num;                                
                            }

                            fill_pathpoint.clear();

                            for(int i = 0;i< num;i++){ 
                                    fill_pathpoint.push_back({ lx + x_1 * i,
                                                               ly + y_1 * i,
                                                               lz + z_1 * i
                                    });
                                } 

                            isFillTrajectory = false;

                            printf("fill_pathpoint.size()=%zu,num=:%d\r\n",fill_pathpoint.size(),num);                                  
                        }

                        printf("obstacle_distance = %lf \r\n",obstacle_distance); 
                        // 1.3.2 判断是否前方有障碍物,进入悬停代码
                        //---------------------------------------------------------------------
                        if(obstacle_distance > 2.0f || isObstacleStop == false){ 
                      
                        //1.3.3  判断前方没有障碍物，则沿轨迹飞行
                            // 1. 遍历计算所有点，计算与当前位置最近的点
                            if(!(fill_pathpoint.empty())  ){  
                                double min_distance = 1000000.0f;
                                double ly = localPos.y;
                                double lx = localPos.x;
                                double lz = localPos.z;
                                int min_targetPointNum = 0;
                                for(int i=0;i<fill_pathpoint.size();i++){
                                    double dis = pow((ly-fill_pathpoint[i].y),2) + pow((lx-fill_pathpoint[i].x),2) + pow((lz-fill_pathpoint[i].z),2);
                                    if(dis < min_distance){
                                        min_distance = dis;
                                        min_targetPointNum = i;
                                    }
                                }

                                //ZZX
                                // int forwardNcount = 6;
                                // if (min_targetPointNum<= fill_pathpoint.size()-forwardNcount-1)
                                // {
                                //     min_targetPointNum = min_targetPointNum+forwardNcount;
                                // }
                                // //ZZX

                                //获取轨迹上的速度方向，即做速度跟随的主方向
                                xyz_double_def target_vel_main_direction;
                                if(min_targetPointNum == (fill_pathpoint.size()-1)){
                                    target_vel_main_direction = {   
                                        (fill_pathpoint[fill_pathpoint.size()-1].x - fill_pathpoint[fill_pathpoint.size()-2].x),
                                        (fill_pathpoint[fill_pathpoint.size()-1].y - fill_pathpoint[fill_pathpoint.size()-2].y),
                                        (fill_pathpoint[fill_pathpoint.size()-1].z - fill_pathpoint[fill_pathpoint.size()-2].z) 
                                    };
                                }else if((min_targetPointNum + jump_pointnumber) > (fill_pathpoint.size()-1)){
                                    target_vel_main_direction = {   
                                        (fill_pathpoint[fill_pathpoint.size()-1].x - fill_pathpoint[min_targetPointNum].x),
                                        (fill_pathpoint[fill_pathpoint.size()-1].y - fill_pathpoint[min_targetPointNum].y),
                                        (fill_pathpoint[fill_pathpoint.size()-1].z - fill_pathpoint[min_targetPointNum].z) 
                                    }; 
                                }else{
                                    target_vel_main_direction = {   
                                        (fill_pathpoint[min_targetPointNum+jump_pointnumber].x - fill_pathpoint[min_targetPointNum].x),
                                        (fill_pathpoint[min_targetPointNum+jump_pointnumber].y - fill_pathpoint[min_targetPointNum].y),
                                        (fill_pathpoint[min_targetPointNum+jump_pointnumber].z - fill_pathpoint[min_targetPointNum].z) 
                                    }; 
                                }
        
                                //计算目标的前向速度大小
                                xyz_double_def target_velocity_main = calculate_FixedModulusVector(target_vel_main_direction,vel_forward_set);   //目标前向速度                        

                                Eigen::Quaterniond eigen_local_q_(localAtt.w, localAtt.x, localAtt.y, localAtt.z);
                                Eigen::Vector3d body_acc_(localAcc.x, localAcc.y, localAcc.z);
                                Eigen::Vector3d local_acc_ = eigen_local_q_*body_acc_;      //////////error
                               
                                // localAcc;

                                double jumptime = (double)jump_pointnumber*(1.0f/(double)control_frequency);

                                int forwardPoint = min_targetPointNum;        //预测飞机跟踪轨迹的点位
                                if(forwardPoint + jump_pointnumber <= fill_pathpoint.size()-1){
                                    forwardPoint = forwardPoint + jump_pointnumber;             
                                }else{
                                    forwardPoint =  fill_pathpoint.size()-1;
                                }                                

                                //横向控制做预测

                                double target_vel_x_h_pid = pos_control_x.calculate(localPos.x, fill_pathpoint[min_targetPointNum].x); 
                                double target_vel_y_h_pid = pos_control_y.calculate(localPos.y, fill_pathpoint[min_targetPointNum].y);    
                                double target_vel_z_h_pid = pos_control_z.calculate(localPos.z, fill_pathpoint[min_targetPointNum].z);  

                                double target_vel_x_h_pid_BUTTER =  butterfilter_vel_x->filter(target_vel_x_h_pid);
                                double target_vel_y_h_pid_BUTTER =  butterfilter_vel_y->filter(target_vel_y_h_pid);
                                double target_vel_z_h_pid_BUTTER =  butterfilter_vel_z->filter(target_vel_z_h_pid);

                                //横向控制做预测机结束

                                double target_vel_x_pid = target_vel_x_h_pid_BUTTER + target_velocity_main.x;
                                double target_vel_y_pid = target_vel_y_h_pid_BUTTER + target_velocity_main.y;  
                                double target_vel_z_pid = target_vel_z_h_pid_BUTTER + target_velocity_main.z;                          

                                const double DELT_RANGE_VEL_2 = 0.02;
                                target_vel_x_pid = slew_rate_limit(target_vel_x_pid, target_vel_x_pid_last, DELT_RANGE_VEL_2);
                                target_vel_y_pid = slew_rate_limit(target_vel_y_pid, target_vel_y_pid_last, DELT_RANGE_VEL_2);
                                target_vel_z_pid = slew_rate_limit(target_vel_z_pid, target_vel_z_pid_last, DELT_RANGE_VEL_2);

                                target_vel_x_pid_last = target_vel_x_pid;
                                target_vel_y_pid_last = target_vel_y_pid;
                                target_vel_z_pid_last = target_vel_z_pid;

                                double target_acc_x_pid = acc_control_x.calculate(localVel.x,target_vel_x_pid);
                                double target_acc_y_pid = acc_control_y.calculate(localVel.y,target_vel_y_pid);
                                double target_acc_z_pid = acc_control_z.calculate(localVel.z,target_vel_z_pid);

                                const double DELT_RANGE_3 = degToAccLimit(5.0);
                                target_acc_x_pid = slew_rate_limit(target_acc_x_pid, target_acc_x_pid_last, DELT_RANGE_3);
                                target_acc_y_pid = slew_rate_limit(target_acc_y_pid, target_acc_y_pid_last, DELT_RANGE_3);
                                target_acc_z_pid = slew_rate_limit(target_acc_z_pid, target_acc_z_pid_last, DELT_RANGE_3);

                                target_acc_x_pid_last = target_acc_x_pid;
                                target_acc_y_pid_last = target_acc_y_pid;
                                target_acc_z_pid_last = target_acc_z_pid;

                                //转换到机体坐标系下，将目标加速度转换成飞机的目标姿态
                                Eigen::Quaterniond q(localAtt.w, localAtt.x, localAtt.y, localAtt.z);                                   //当前姿态四元素，转换为欧拉角
                                double yaw,roll,pitch;
                                toEulerAngle(q,roll,pitch,yaw);     

                                //转换到机体坐标系下
                                double target_acc_x_pid_in_body = target_acc_x_pid * cos(yaw) + target_acc_y_pid * sin(yaw);
                                double target_acc_y_pid_in_body = -target_acc_x_pid * sin(yaw) + target_acc_y_pid * cos(yaw);

                                //换算成角度
                                double target_pitch_pid =  atan2(target_acc_x_pid_in_body,GRAVITY_ACC);                                         //加速度转换成姿态
                                double target_roll_pid  = -atan2(target_acc_y_pid_in_body,GRAVITY_ACC);                                         //根据右手法则，roll的目标差值需要反向
                                double target_height_pid = target_acc_z_pid * uav_weight / (cos(target_pitch_pid) * cos(target_roll_pid)) *0.2f;       //高度不做控制

                                
                                //欧拉角转换成四元素 发送出去，航向角不变，
                                fillAttitudeTarget(target_att_thrust, target_roll_pid, target_pitch_pid,
                                                  pathpoint_sub_test[targetPointNum_test].a,
                                                  0.43, target_height_pid, node.get());

                                att_thrust_pub->publish(target_att_thrust);    //ZZXZZX                   

                                double vel_xy = sqrt(pow(localVel.x,2)+pow(localVel.y,2)+pow(localVel.z,2));

                                double vel_xy_main = sqrt(pow(target_velocity_main.x,2)+pow(target_velocity_main.y,2)+pow(target_velocity_main.z,2));
                                double vel_xy_heng = sqrt(pow(target_vel_x_h_pid,2)+pow(target_vel_y_h_pid,2)+pow(target_vel_z_h_pid,2));

                                double distance = calculate_distance(pathpoint_sub_test[targetPointNum_test].x,pathpoint_sub_test[targetPointNum_test].y,pathpoint_sub_test[targetPointNum_test].z); 

                                printf("line_fly num:%d,D:%lf  vel:%lf main:%lf heng:%lf, lx:%lf ly:%lf lz:%lf,lvx:%lf lvy:%lf lvz:%lf,tlx:%lf tly:%lf tlz:%lf,tvhx:%lf tvhy:%lf tvhz:%lf, tvx:%lf tvy:%lf tvz:%lf,tvx_b:%lf tvy_b:%lf tvz_b:%lf,tax:%lf tay:%lf taz:%lf,tbax:%lf tbay:%lf thu:%lf,tpit:%lf troll:%lf tyaw:%lf,\r\n",
                                min_targetPointNum,distance,vel_xy,vel_xy_main,vel_xy_heng,
                                localPos.x,localPos.y,localPos.z,
                                localVel.x,localVel.y,localVel.z,
                                pathpoint_sub_test[targetPointNum_test].x,pathpoint_sub_test[targetPointNum_test].y,pathpoint_sub_test[targetPointNum_test].z,
                                target_vel_x_h_pid,target_vel_y_h_pid,target_vel_z_h_pid,
                                target_vel_x_pid,target_vel_y_pid,target_vel_z_pid,
                                target_vel_x_h_pid_BUTTER,target_vel_y_h_pid_BUTTER,target_vel_z_h_pid_BUTTER,
                                target_acc_x_pid,target_acc_y_pid,target_acc_z_pid,
                                target_acc_x_pid_in_body,target_acc_y_pid_in_body,0.45f + target_height_pid,
                                target_pitch_pid,target_roll_pid,pathpoint_sub_test[targetPointNum_test].a
                                );


                                //判断是否已经飞到目标点了
                                if(distance < 0.2f){
                                    targetPointNum_test++;
                                    isFillTrajectory = true;
                                }

                                isObstacleStop = false;
                            }else{
                                mode_int = false;
                                controlMode = MODE_HOVER_CUSTOM;
                            }
                        }

                        if(obstacle_distance < 0.5f || isObstacleStop == true){
                            printf("1.5obstacle_distance:%lf",obstacle_distance);
                            // 1. 获取当前悬停点以及航向
                            if(!isObstacleStop){
                                //首先确定悬停的位置,从没有障碍物的状态下，转到有障碍物的情况
                                avoid_target_position.x = localPos.x;          //带坐标转换，从笛卡尔转到东北天
                                avoid_target_position.y = localPos.y;
                                avoid_target_position.z = localPos.z;
                                
                                avoid_target_pose.x = localAtt.x;
                                avoid_target_pose.y = localAtt.y;                  //确定航向
                                avoid_target_pose.z = localAtt.z;
                                avoid_target_pose.w = localAtt.w;                        
                            }

                            // 2. 控制飞行停止
                                double target_vel_x_pid = pos_control_x.calculate(localPos.x,avoid_target_position.x);               //PID的系数设置为0.1~10 之间，速度现在-1~1之间
                                double target_vel_y_pid = pos_control_y.calculate(localPos.y,avoid_target_position.y);               //预设最大P值10时，距离相差1m的时候，最大值输出。
                                double target_vel_z_pid = pos_control_z.calculate(localPos.z,avoid_target_position.z);               //即，10*1*0.1 = 1m/s
                
                                double target_acc_x_pid = vel_control_x.calculate(localVel.x,target_vel_x_pid) ;                         //PID的系数设置为0.1~10之间，加速度限制在-5.685~5.685之间，取上下限-6~6之间
                                double target_acc_y_pid = vel_control_y.calculate(localVel.y,target_vel_y_pid) ;                         //预设最大P值10时，速度相差5m/s的时候，最大值输出。
                                double target_acc_z_pid = vel_control_z.calculate(localVel.z,target_vel_z_pid) ;                         //即，10*5 * 0.12 = 6  
        
        
                                //以上代码target_vel_x_pid，target_acc_x_pid都为与local的坐标系下，即东北天坐标系
                
                                Eigen::Quaterniond q(localAtt.w, localAtt.x, localAtt.y, localAtt.z);                                   //当前姿态四元素，转换为欧拉角
                                double yaw,roll,pitch;
                                toEulerAngle(q,roll,pitch,yaw);     
                
                                //转换到机体坐标系下
                                double target_acc_x_pid_in_body = target_acc_x_pid * cos(yaw) + target_acc_y_pid * sin(yaw);
                                double target_acc_y_pid_in_body = -target_acc_x_pid * sin(yaw) + target_acc_y_pid * cos(yaw);
                
                                //换算成角度
                                double target_pitch_pid =  atan2(target_acc_x_pid_in_body,GRAVITY_ACC);                                         //加速度转换成姿态
                                double target_roll_pid  = -atan2(target_acc_y_pid_in_body,GRAVITY_ACC);                                         //根据右手法则，roll的目标差值需要反向
                                double target_height_pid = target_acc_z_pid * uav_weight / (cos(target_pitch_pid) * cos(target_roll_pid)) *0.2f;                                         //高度不做控制
                
                                euler_def aviod_target_euler;
                                toEulerAngle(Eigen::Quaterniond(avoid_target_pose.w, avoid_target_pose.x,
                                                                avoid_target_pose.y, avoid_target_pose.z),
                                             aviod_target_euler.roll, aviod_target_euler.pitch, aviod_target_euler.yaw);

                                //欧拉角转换成四元素 发送出去，航向角不变，
                                fillAttitudeTarget(target_att_thrust, target_roll_pid, target_pitch_pid,
                                                  aviod_target_euler.yaw, 0.43, target_height_pid, node.get());

                                att_thrust_pub->publish(target_att_thrust);                                            //发送消息


                            // 3 轨迹变量
                            isObstacleStop = true;

                        }
                    }

                           
                }             
                else{
                    if(targetPointNum_test == 0){                                //没有收到轨迹
                        pos_pub->publish(target_pose);             //改位置在模式初始化的时候，更新的位置
                        printf("hover no point \r\n");
                    }else if(targetPointNum_test == pathpoint_sub_test.size()){       //已经到轨迹终点了


                        target_pose.header.stamp = node->now();
                        target_pose.header.frame_id = "map";
                        target_pose.pose.position.x = pathpoint_sub_test[targetPointNum_test-1].x;
                        target_pose.pose.position.y = pathpoint_sub_test[targetPointNum_test-1].y;
                        target_pose.pose.position.z = pathpoint_sub_test[targetPointNum_test-1].z;


                        double target_yaw = pathpoint_sub_test[targetPointNum_test-1].a;

                        //欧拉角转换成四元素 发送出去，航向角不变，
                        Eigen::AngleAxisd rollAngle_(Eigen::AngleAxisd(0,Eigen::Vector3d::UnitX()));      //pid计算的目标航向
                        Eigen::AngleAxisd pitchAngle_(Eigen::AngleAxisd(0,Eigen::Vector3d::UnitY()));
                        Eigen::AngleAxisd yawAngle_(Eigen::AngleAxisd(target_yaw,Eigen::Vector3d::UnitZ()));        //进入模式的时候，设定的目标航向
                        
                        Eigen::Quaterniond quaternion_;                                     
                        quaternion_=yawAngle_*pitchAngle_*rollAngle_;

                        target_pose.pose.orientation.x = quaternion_.coeffs()[0];  
                        target_pose.pose.orientation.y = quaternion_.coeffs()[1]; 
                        target_pose.pose.orientation.z = quaternion_.coeffs()[2];  
                        target_pose.pose.orientation.w = quaternion_.coeffs()[3];  

                        pos_pub->publish(target_pose);              //该位置在最后一个点进圈后，更新的位置                
                        printf("finish\r\n");
                    }else{
                        mode_int = false;
                        controlMode = MODE_HOVER_CUSTOM;                           //其他情况，切换到悬停模式
                    }
                }
            }

        }
        else if (controlMode == MODE_HOVER_CUSTOM){
            // DOB persistent state — kept across mode switches.
            // On very first hover entry, all zero is fine:
            //   dob_u_z = -Kd*vel gives immediate velocity damping.
            // On re-entry, converged state is preserved.
            static double dob_z_x = 0.0, dob_z_y = 0.0, dob_z_z = 0.0;
            static double dob_d_x_hat = 0.0, dob_d_y_hat = 0.0, dob_d_z_hat = 0.0;
            static double dob_last_time = 0.0;
            static double dob_u_x = 0, dob_u_y = 0, dob_u_z = 0;
            if(!mode_int){
                target_pose.header.stamp = node->now();
                target_pose.header.frame_id = "map";
                target_pose.pose.position.x = localPos.x;
                target_pose.pose.position.y = localPos.y;
                target_pose.pose.position.z = localPos.z;

                target_pose.pose.orientation.x = localAtt.x ;
                target_pose.pose.orientation.y = localAtt.y ;
                target_pose.pose.orientation.z = localAtt.z ;
                target_pose.pose.orientation.w = localAtt.w ;

                // 抓拍当前偏航作为整个悬停期间的期望偏航
                target_euler.yaw = atan2(2.0 * (localAtt.w * localAtt.z + localAtt.x * localAtt.y),
                                         1.0 - 2.0 * (localAtt.y * localAtt.y + localAtt.z * localAtt.z));

                // 始终重置计时器，防止进入悬停的首帧 dt 尖峰
                dob_last_time = node->now().seconds();

                printf("hoverMode_CUSTOM x:%f y:%f z:%f\r\n",target_pose.pose.position.x,target_pose.pose.position.y,target_pose.pose.position.z);
                pos_pub->publish(target_pose);
                mode_int = true;
            }else{
                // PD + DOB:   ωₙ = natural frequency,  ζ = damping ratio
                double err_x = target_pose.pose.position.x - localPos.x;
                double err_y = target_pose.pose.position.y - localPos.y;
                double err_z = target_pose.pose.position.z - localPos.z;

                // XY:  acc = ωₙ²·err − 2ζωₙ·vel
                double Kp_xy = hover_wn_xy * hover_wn_xy;
                double Kd_xy = 2.0 * hover_zeta_xy * hover_wn_xy;
                double target_acc_x_pid = Kp_xy * err_x - Kd_xy * localVel.x;
                double target_acc_y_pid = Kp_xy * err_y - Kd_xy * localVel.y;

                // Z:  acc = ωₙ²·err − 2ζωₙ·vel
                double Kp_z_sq = hover_wn_z * hover_wn_z;
                double Kd_z     = 2.0 * hover_zeta_z * hover_wn_z;
                double target_acc_z_pid = Kp_z_sq * err_z - Kd_z * localVel.z;

                // DOB observer gains from YAML
                double L_xy = dob_L_xy;
                double L_z  = dob_L_z;

                double current_time = node->now().seconds();
                double dt =  current_time - dob_last_time;

                if (dt > 0.005) {
                    // Low-pass DOB on all axes: L/(s+L), DC gain = 1
                    dob_z_x = dob_z_x/(L_xy*dt+1) - L_xy*L_xy*dt*localVel.x/(L_xy*dt+1) - L_xy*dob_u_x*dt/(L_xy*dt+1);
                    dob_z_y = dob_z_y/(L_xy*dt+1) - L_xy*L_xy*dt*localVel.y/(L_xy*dt+1) - L_xy*dob_u_y*dt/(L_xy*dt+1);
                    dob_z_z = dob_z_z/(L_z*dt+1)  - L_z*L_z*dt*localVel.z/(L_z*dt+1)   - L_z*dob_u_z*dt/(L_z*dt+1);
                    dob_d_x_hat = dob_z_x + L_xy * localVel.x;
                    dob_d_y_hat = dob_z_y + L_xy * localVel.y;
                    dob_d_z_hat = dob_z_z + L_z  * localVel.z;

                    dob_last_time = current_time;
                }

                dob_u_x = target_acc_x_pid - dob_d_x_hat;
                dob_u_y = target_acc_y_pid - dob_d_y_hat;
                dob_u_z = target_acc_z_pid - dob_d_z_hat;

                // u_x, u_y, u_z 已得到（期望运动加速度，不含重力）

                // 1. 构造总加速度（运动加速度 + 重力补偿）
                Eigen::Vector3d accel_motion(dob_u_x, dob_u_y, dob_u_z);
                Eigen::Vector3d gravity(0.0, 0.0, GRAVITY_ACC);
                Eigen::Vector3d accel_total = accel_motion + gravity;  // 这才是总期望加速度

                // 2. 根据总加速度和期望偏航角生成期望姿态四元数
                double acc_norm = accel_total.norm();
                Eigen::Quaterniond out_quat;
                const double eps = 1e-6;

                // 使用初始化时抓拍的期望偏航（保持不变）
                double yaw_des = target_euler.yaw;

                if (acc_norm < eps) {
                    // 异常：总加速度近乎零（几乎不可能），保持水平，仅按偏航
                    Eigen::AngleAxisd yaw_rot(yaw_des, Eigen::Vector3d::UnitZ());
                    out_quat = Eigen::Quaterniond(yaw_rot);
                } else {
                    // 标准过程：推力方向 = 总加速度方向（机体Z轴）
                    Eigen::Vector3d b_c = accel_total.normalized();
                    Eigen::Vector3d t(std::cos(yaw_des), std::sin(yaw_des), 0.0);
                    double dot = t.dot(b_c);
                    Eigen::Vector3d X_c = t - dot * b_c;
                    if (X_c.norm() < eps) X_c = Eigen::Vector3d(1.0, 0.0, 0.0);
                    X_c.normalize();
                    Eigen::Vector3d Y_c = b_c.cross(X_c);
                    Y_c.normalize();
                    Eigen::Matrix3d R;
                    R.col(0) = X_c;
                    R.col(1) = Y_c;
                    R.col(2) = b_c;
                    out_quat = Eigen::Quaterniond(R);
                }
                out_quat.normalize();

                // 3. 计算总推力（牛顿）并映射到归一化油门
                double thrust_newtons = uav_weight * acc_norm;
                double weight_newtons = uav_weight * GRAVITY_ACC;
                double max_thrust_newtons = weight_newtons * thrust_ratio;

                double throttle;
                if (thrust_newtons <= weight_newtons) {
                    // 悬停以下：线性从 0 到 hover_throttle
                    throttle = hover_throttle * (thrust_newtons / weight_newtons);
                } else {
                    // 悬停以上：线性从 hover_throttle 到 1.0
                    throttle = hover_throttle + (1.0 - hover_throttle) * (thrust_newtons - weight_newtons) / (max_thrust_newtons - weight_newtons);
                }
                throttle = std::clamp(throttle, 0.0, 1.0);

                // 4. 填充 MAVLink 消息
                target_att_thrust.orientation.x = out_quat.x();
                target_att_thrust.orientation.y = out_quat.y();
                target_att_thrust.orientation.z = out_quat.z();
                target_att_thrust.orientation.w = out_quat.w();
                target_att_thrust.thrust = throttle;   // 直接使用映射后的油门
                target_att_thrust.type_mask = 1+2+4;
                target_att_thrust.header.stamp = node->now();
                att_thrust_pub->publish(target_att_thrust);                         
            }
        }
    
        rclcpp::spin_some(node);
        rate_control.sleep();
    }
    // Clean shutdown:
    // 1. Stop ROS2 first (makes rclcpp::ok() return false)
    rclcpp::shutdown();
    // 2. Wake the keyboard thread from blocking getchar() using SIGUSR1.
    //    This avoids pthread_cancel which can leave terminal in raw mode
    //    and/or race with DDS resource destruction.
    if (th_created) {
        // Interrupt getchar() so sh_getch() restores terminal and returns EOF.
        // The keyboard thread's while(rclcpp::ok()) will then fail cleanly.
        // Retry up to 2s (40*50ms) until the thread notices and exits.
        for (int i = 0; i < 40; i++) {
            pthread_kill(th, SIGUSR1);
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
        pthread_join(th, NULL);
    }
    return 0;
}