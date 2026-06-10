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
double calculate_distance2(double x,double y,double z,double x1,double y1,double z1);

std::ofstream fout_current_v, fout_current_p, fout_pid;

// Global node pointer for use in keyboard thread (pthread)
std::shared_ptr<rclcpp::Node> g_node;

//---------------------自定义数据类型区域---------------------
//飞行状态
enum fly_status{
    MODE_POSITION       = 0,    
    MODE_VELOCITY       = 1,
    MODE_HOVER          = 2,
    MODE_AUTO           = 3,
    MODE_AUTO_POSE      = 8,
    MODE_STRAIGHT       = 9,
    TEST_FLOOR_TRAJ     = 12,
    MODE_HOVER_CUSTOM   = 13,  //自定义悬停模式
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
} xyz_f_def;

typedef struct{
    double x;
    double y;
    double z;
} xyz_d_def;

typedef struct{
    double x;
    double y;
    double z;
    double a;
} xyza_d_def;

typedef struct {
    double x;
    double y;
    double z;
    double w;
} QuaternionTypedef;

typedef struct {
    double roll;
    double pitch;
    double yaw;
} EulerTypedef;
//---------------------自定义数据类型区域结束---------------------


//---------------------全局变量定义区域---------------------
#define GRAVITY_ACC 9.8f

pthread_t th;                           //键盘线程
bool      th_created = false;           // whether pthread_create succeeded
                                        // (guard against pthread_cancel on invalid handle)

int ctrlMode=0;                         //是否结束线程
int controlMode = 1;                    //控制模式的变换

bool vel_change = false;     //速度模式是否通过按键改变，用于控制是否打印速度控制信息
bool guided_change = false;
bool arm_change = false;
bool mode_int = false;                  //用于记录是否切换模式，进入新模式前，进行初始化

EulerTypedef target_pose_euler;         //发布消息接口中，之后四元素的定义，采用欧拉角的全局变量在定义一次

std::vector<xyz_d_def> fixed_pathpoint;   //固定轨迹的轨迹存储区域
std::vector<xyz_d_def> fill_pathpoint;    //填充轨迹
std::vector<float> fill_yaw;

xyz_d_def avoid_target_position;
QuaternionTypedef avoid_target_pose;
bool isObstacleStop = false;
int turnDirection = 0;

int control_frequency = 100;

// ROS2 发布器
rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr refTrajectoryPub;
rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr curTrajectoryPub;
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
//发布控制无人机速度的消息接口变量
geometry_msgs::msg::Twist       target_veloity;
//发布控制无人机位置的消息接口变量
geometry_msgs::msg::PoseStamped  target_position;
//发布控制无人机姿态的消息接口类型
mavros_msgs::msg::AttitudeTarget target_pose;

//发布停止轨迹规划的命令
std_msgs::msg::Int64 stop_command;

//发布设定的跟踪轨迹
nav_msgs::msg::Path refTrajectory;

//发布当前的运动轨迹
nav_msgs::msg::Path curTrajectory;
//发布当前的跟踪的目标点
geometry_msgs::msg::PoseStamped target_position_show;
//---------------------发送数据消息接口类型区域结束---------------------

//---------------------回调函数区域---------------------
xyz_d_def localposition;
QuaternionTypedef localpose;
void toEulerAngle(const Eigen::Quaterniond &q, double &roll, double &pitch, double &yaw);
double calculate_VectorAngular_x(xyz_d_def vector_major);

//接收当前位姿的回调函数
void poseCallback(const geometry_msgs::msg::PoseStamped::ConstPtr &msg){
    localposition.x = msg->pose.position.x ;
    localposition.y = msg->pose.position.y ;    
    localposition.z = msg->pose.position.z ;  
    // localposition.y = msg->pose.position.x ;
    // localposition.x = -msg->pose.position.y ;    
    // localposition.z = msg->pose.position.z ;       

    localpose.x = msg->pose.orientation.x ;
    localpose.y = msg->pose.orientation.y ; 
    localpose.z = msg->pose.orientation.z ;
    localpose.w = msg->pose.orientation.w ;

    Eigen::Quaterniond q(localpose.w, localpose.x, localpose.y, localpose.z);
    double p_roll,p_pitch,p_yaw;
    toEulerAngle(q,p_roll,p_pitch,p_yaw);

    fout_current_p<< msg->header.stamp.sec <<","<<localposition.x<<","<<localposition.y<<","<<localposition.z<<std::endl;

    if(curTrajectory.poses.size() < 200){
        curTrajectory.header.stamp = g_clock.now();
        curTrajectory.header.frame_id = "map";
        geometry_msgs::msg::PoseStamped targetPoseStamped;

        targetPoseStamped.pose.position.x = localposition.x;
        targetPoseStamped.pose.position.y = localposition.y;
        targetPoseStamped.pose.position.z = localposition.z;
        curTrajectory.poses.push_back(targetPoseStamped);
        curTrajectoryPub->publish(curTrajectory);  
    }else{
        for(int i=0;(i<(200-1));i++){
            curTrajectory.poses[i] = curTrajectory.poses[i+1];
        }
        curTrajectory.header.stamp = g_clock.now();
        curTrajectory.header.frame_id = "map";

        geometry_msgs::msg::PoseStamped targetPoseStamped;
        targetPoseStamped.pose.position.x = localposition.x;
        targetPoseStamped.pose.position.y = localposition.y;
        targetPoseStamped.pose.position.z = localposition.z;
        curTrajectory.poses[199]=(targetPoseStamped); 
        curTrajectoryPub->publish(curTrajectory);           
    }
}

double obstacle_distance = 100000.0f;
void nearest_distance_cb(const std_msgs::msg::Float32::ConstPtr &dis){
    obstacle_distance = dis->data;
}

xyz_d_def localacceleration;
xyz_d_def localacceleration_offset={0,0,0};
void imudata_cb(const sensor_msgs::msg::Imu::ConstPtr &msg){
    localacceleration.x = msg->linear_acceleration.x;
    localacceleration.y = msg->linear_acceleration.y;
    localacceleration.z = msg->linear_acceleration.z;
}

//接受轨迹信息的回调函数
std::vector<xyz_d_def> pathpoint_sub;                                 //用于存储轨迹点信息，采用C++容器的凡是
std::vector<xyza_d_def> pathpoint_sub_test;    
int targetPointNum = 0;                                                 //当前用于跟踪的轨迹点
int targetPointNum_test = 0;  
void pathsub_cb(const nav_msgs::msg::Path::ConstPtr &navpath){
    rclcpp::Time start_time = g_clock.now();
        if(navpath->poses.size() > 0){      //
            int check_count=0;
            xyz_d_def old_path_point;
            while((check_count != navpath->poses.size()) && (check_count  < 50)){  //如果判断出来轨迹距离一直增加，则判定该点与新轨迹的接续点
                
                if(pathpoint_sub.empty()){
                    pathpoint_sub.clear();
                    targetPointNum = 0;
                    break;
                }else{
                    old_path_point = pathpoint_sub.back();
                }



                double distance_last = calculate_distance2((navpath->poses)[0].pose.position.x,(navpath->poses)[0].pose.position.y,(navpath->poses)[0].pose.position.z,old_path_point.x,old_path_point.y,old_path_point.z);
                check_count = 1;
                for(;check_count < (navpath->poses.size());check_count++ ){
                    double distance = calculate_distance2((navpath->poses)[check_count].pose.position.x,(navpath->poses)[check_count].pose.position.y,(navpath->poses)[check_count].pose.position.z,old_path_point.x,old_path_point.y,old_path_point.z);
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
    // }


    rclcpp::Time end_time = g_clock.now();
    rclcpp::Duration elapsed_time = end_time - start_time;
    RCLCPP_INFO(rclcpp::get_logger("guided_sim"), "TIME:%lf ms\r\n",elapsed_time.seconds());
    RCLCPP_INFO(rclcpp::get_logger("guided_sim"), "Received trajectory with a length of: %d points pathpoint_sub %d points\r\n",navpath->poses.size(),pathpoint_sub.size());



}

//接受当前速度的回调函数
xyz_d_def localvelocity;
void velCallback(const geometry_msgs::msg::TwistStamped::ConstPtr &msg){
    localvelocity.x = msg->twist.linear.x ;
    localvelocity.y = msg->twist.linear.y ;  
    localvelocity.z = msg->twist.linear.z ;   

    // localvelocity.y = msg->twist.linear.x ;
    // localvelocity.x = -msg->twist.linear.y ;  
    // localvelocity.z = msg->twist.linear.z ;   

    fout_current_v<< msg->header.stamp.sec <<","<<localvelocity.x<<","<<localvelocity.y<<","<<localvelocity.z<<std::endl;
}

//接受无人机当前状态的回调函数
mavros_msgs::msg::State current_state;
void state_cb(const mavros_msgs::msg::State::ConstPtr& msg){
    current_state = *msg;
}
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
}pid_vector_param_def;

pid_vector_param_def position_pid;
pid_vector_param_def vel_pid;
pid_vector_param_def vel_l_pid;
pid_vector_param_def vel_a_pid;
xyz_f_def vel_out_limit;
xyz_f_def pose_out_limit;

//其他参数定义
float ciecle_range;         //进圈判断
float jump_pointnumber;     //跳点的设置
float vel_forward_set;
float target_x;
float target_y;
//---------------launch文件参数读取定义区结束----------------------------


//创建PID控制
//位置控制器的PID
PID position_control_x = PID(1.0f/control_frequency,1.0f, -1.0f, 0,0,0);          //控制器的输出尽可能限制在 -1m/s~1m/s
PID position_control_y = PID(1.0f/control_frequency,1.0f, -1.0f, 0,0,0);
PID position_control_z = PID(1.0f/control_frequency,1.0f, -1.0f, 0,0,0);
//速度控制器的PID
PID vel_control_x = PID(1.0f/control_frequency, GRAVITY_ACC * tan(30.0f*M_PI/180.0f), -GRAVITY_ACC * tan(30.0f*M_PI/180.0f), 0,0,0);  //控制器输出的加速度限制尽可能限制在 5.65 m/s^2=GRAVITY_ACC * tan(pose_out_limit.x)
PID vel_control_y = PID(1.0f/control_frequency, GRAVITY_ACC * tan(30.0f*M_PI/180.0f), -GRAVITY_ACC * tan(30.0f*M_PI/180.0f), 0,0,0);  //控制器速度上的y输出
PID vel_control_z = PID(1.0f/control_frequency, GRAVITY_ACC * tan(30.0f*M_PI/180.0f), -GRAVITY_ACC * tan(30.0f*M_PI/180.0f), 0,0,0);
//前向速度上的PID控制器
PID vel_control_l_x = PID(1.0f/control_frequency, GRAVITY_ACC * tan(30.0f*M_PI/180.0f), -GRAVITY_ACC * tan(30.0f*M_PI/180.0f), 0,0,0);
PID vel_control_l_y = PID(1.0f/control_frequency, GRAVITY_ACC * tan(30.0f*M_PI/180.0f), -GRAVITY_ACC * tan(30.0f*M_PI/180.0f), 0,0,0);
PID vel_control_l_z = PID(1.0f/control_frequency, GRAVITY_ACC * tan(30.0f*M_PI/180.0f), -GRAVITY_ACC * tan(30.0f*M_PI/180.0f), 0,0,0);
//这组PID主要作用是横向上的误差控制，可以与位置控制的PID复用，PID的调节的细节，从理论上讲是前向速度和横向速度的比值
PID vel_control_a_x = PID(1.0f/control_frequency, GRAVITY_ACC * tan(30.0f*M_PI/180.0f), -GRAVITY_ACC * tan(30.0f*M_PI/180.0f), 0,0,0);
PID vel_control_a_y = PID(1.0f/control_frequency, GRAVITY_ACC * tan(30.0f*M_PI/180.0f), -GRAVITY_ACC * tan(30.0f*M_PI/180.0f), 0,0,0);
PID vel_control_a_z = PID(1.0f/control_frequency, GRAVITY_ACC * tan(30.0f*M_PI/180.0f), -GRAVITY_ACC * tan(30.0f*M_PI/180.0f), 0,0,0);



//---------------工具函数区域 ----------------------------
//四元素转换成欧拉角的函数
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
}

//清空scanf的缓冲区
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
}

//计算目标点与当下点的距离的平方
double calculate_distance_square(double x,double y,double z){
    return (pow((x-localposition.x),2)+pow((y-localposition.y),2)+pow((z-localposition.z),2)); 
}
//计算目标点与当下点的距离
double calculate_distance(double x,double y,double z){
    return sqrt(pow((x-localposition.x),2)+pow((y-localposition.y),2)+pow((z-localposition.z),2)); 
}
//计算目标点与当下点的距离
double calculate_distance2(double x,double y,double z,double x1,double y1,double z1){
    return sqrt(pow((x-x1),2)+pow((y-y1),2)+pow((z-z1),2)); 
}

//平面采用点法式，计算平面外的一点到平面上的投影点
//具体算法参考网站https://blog.csdn.net/lafengxiaoyu/article/details/78435165                    
xyz_d_def calculate_ProjectionPoint(xyz_d_def normalvector,xyz_d_def includedpoints,xyz_d_def out_point){
    
    // double norm = (pow(normalvector.x,2)+pow(normalvector.y,2)+pow(normalvector.z,2));
    xyz_d_def projectionpoint;
    projectionpoint.x = (normalvector.x*(normalvector.x*includedpoints.x + normalvector.y*includedpoints.y + normalvector.z*includedpoints.z) + \
    (pow(normalvector.y,2)+pow(normalvector.z,2)) * out_point.x \
    -normalvector.x*normalvector.y*out_point.y \
    -normalvector.x*normalvector.z*out_point.z ) \
    /(pow(normalvector.x,2)+pow(normalvector.y,2)+pow(normalvector.z,2));

    if(normalvector.x == 0){
        projectionpoint.y = out_point.y;
        projectionpoint.z = out_point.z;
    }else{
        projectionpoint.y = normalvector.y/normalvector.x*(out_point.x-projectionpoint.x)+out_point.y;
        projectionpoint.z = normalvector.z/normalvector.x*(out_point.x-projectionpoint.x)+out_point.z;
    }
    
    return projectionpoint;
}

xyz_d_def calculate_FixedModulusVector(xyz_d_def normalvector,double modulus){
    double norm = sqrt(pow(normalvector.x,2)+pow(normalvector.y,2)+pow(normalvector.z,2));
    double ratio;
    if(norm == 0){
        ratio = 1.0f;
    }else{
        ratio = modulus/norm;
    }

    xyz_d_def returnVector = {
        normalvector.x * ratio ,
        normalvector.y * ratio ,
        normalvector.z * ratio ,
    };
    return returnVector;
}

xyz_d_def calculate_VectorProjection(xyz_d_def vector_major,xyz_d_def vector_minor){
    double norm_2 = pow(vector_minor.x,2)+pow(vector_minor.y,2)+pow(vector_minor.z,2);
    double vectorProduct =  vector_major.x*vector_minor.x+vector_major.y*vector_minor.y+vector_major.z*vector_minor.z;
    double ratio;
    if(norm_2 == 0.0f){
        ratio = 1.0f;
    }else{
        ratio = vectorProduct / norm_2;
    }

    xyz_d_def returnVector = {
        vector_minor.x * ratio,
        vector_minor.y * ratio,
        vector_minor.z * ratio,
    };
    return returnVector;
}

double calculate_VectorAngular_x(xyz_d_def vector_major){
    double angular =  acos(vector_major.x/sqrt(vector_major.x*vector_major.x+vector_major.y*vector_major.y));

    if(vector_major.y < 0){
        angular = 2*M_PI - angular;
    }

    return angular;
}

// 函数用于计算两个角度之间的夹角
double calculate_AngleDifference(double angle1, double angle2) {
    // 计算两个角度的差值
    double difference = fabs(angle1 - angle2);

    // 如果差值大于pi，说明夹角应该从另一侧计算
    if (difference > M_PI) {
        difference = 2 * M_PI - difference;
    }

    return difference;
}

void PID_Reset(){
    position_control_x.reinitiate();    //位置环PID清零
    position_control_y.reinitiate();
    position_control_z.reinitiate();

    vel_control_x.reinitiate();         //速度环PID清零
    vel_control_y.reinitiate();
    vel_control_z.reinitiate();

    vel_control_l_x.reinitiate();       //前向速度PID清零
    vel_control_l_y.reinitiate();
    vel_control_l_z.reinitiate();

    vel_control_a_x.reinitiate();       //横向速度PID清零
    vel_control_a_y.reinitiate();
    vel_control_a_z.reinitiate();        
}

//读取launch文件中的所有参数
void read_param(rclcpp::Node *node){
    position_pid.x.p = node->declare_parameter<float>("position_X_P", 0.0f);
    position_pid.x.i = node->declare_parameter<float>("position_X_I", 0.0f);
    position_pid.x.d = node->declare_parameter<float>("position_X_D", 0.0f);
    position_pid.y.p = node->declare_parameter<float>("position_Y_P", 0.0f);
    position_pid.y.i = node->declare_parameter<float>("position_Y_I", 0.0f);
    position_pid.y.d = node->declare_parameter<float>("position_Y_D", 0.0f);
    position_pid.z.p = node->declare_parameter<float>("position_Z_P", 0.0f);
    position_pid.z.i = node->declare_parameter<float>("position_Z_I", 0.0f);
    position_pid.z.d = node->declare_parameter<float>("position_Z_D", 0.0f);
    vel_pid.x.p = node->declare_parameter<float>("vel_X_P", 0.0f);
    vel_pid.x.i = node->declare_parameter<float>("vel_X_I", 0.0f);
    vel_pid.x.d = node->declare_parameter<float>("vel_X_D", 0.0f);
    vel_pid.y.p = node->declare_parameter<float>("vel_Y_P", 0.0f);
    vel_pid.y.i = node->declare_parameter<float>("vel_Y_I", 0.0f);
    vel_pid.y.d = node->declare_parameter<float>("vel_Y_D", 0.0f);
    vel_pid.z.p = node->declare_parameter<float>("vel_Z_P", 0.0f);
    vel_pid.z.i = node->declare_parameter<float>("vel_Z_I", 0.0f);
    vel_pid.z.d = node->declare_parameter<float>("vel_Z_D", 0.0f);
    vel_l_pid.x.p = node->declare_parameter<float>("vel_X_F_P", 0.0f);
    vel_l_pid.x.i = node->declare_parameter<float>("vel_X_F_I", 0.0f);
    vel_l_pid.x.d = node->declare_parameter<float>("vel_X_F_D", 0.0f);
    vel_l_pid.y.p = node->declare_parameter<float>("vel_Y_F_P", 0.0f);
    vel_l_pid.y.i = node->declare_parameter<float>("vel_Y_F_I", 0.0f);
    vel_l_pid.y.d = node->declare_parameter<float>("vel_Y_F_D", 0.0f);
    vel_l_pid.z.p = node->declare_parameter<float>("vel_Z_F_P", 0.0f);
    vel_l_pid.z.i = node->declare_parameter<float>("vel_Z_F_I", 0.0f);
    vel_l_pid.z.d = node->declare_parameter<float>("vel_Z_F_D", 0.0f);
    vel_a_pid.x.p = node->declare_parameter<float>("vel_X_T_P", 0.0f);
    vel_a_pid.x.i = node->declare_parameter<float>("vel_X_T_I", 0.0f);
    vel_a_pid.x.d = node->declare_parameter<float>("vel_X_T_D", 0.0f);
    vel_a_pid.y.p = node->declare_parameter<float>("vel_Y_T_P", 0.0f);
    vel_a_pid.y.i = node->declare_parameter<float>("vel_Y_T_I", 0.0f);
    vel_a_pid.y.d = node->declare_parameter<float>("vel_Y_T_D", 0.0f);
    vel_a_pid.z.p = node->declare_parameter<float>("vel_Z_T_P", 0.0f);
    vel_a_pid.z.i = node->declare_parameter<float>("vel_Z_T_I", 0.0f);
    vel_a_pid.z.d = node->declare_parameter<float>("vel_Z_T_D", 0.0f);
    vel_out_limit.x = node->declare_parameter<float>("vel_X_limit", 1.0f);
    vel_out_limit.y = node->declare_parameter<float>("vel_Y_limit", 1.0f);
    vel_out_limit.z = node->declare_parameter<float>("vel_Z_limit", 1.0f);
    pose_out_limit.x = node->declare_parameter<float>("roll_limit", 30.0f);
    pose_out_limit.y = node->declare_parameter<float>("pitch_limit", 30.0f);
    pose_out_limit.z = node->declare_parameter<float>("high_limit", 30.0f);
    ciecle_range = node->declare_parameter<float>("ciecle_range", 0.5f);
    jump_pointnumber = node->declare_parameter<float>("jump_pointnumber", 1.0f);
    vel_forward_set = node->declare_parameter<float>("vel_forward_setting", 1.0f);
    target_x = node->declare_parameter<float>("target_x", 0.0f);
    target_y = node->declare_parameter<float>("target_y", 8.0f);
}

//Keyboard Mode Setting
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
}

//子线程监听键盘
void *keyboard_listener(void *dummy){
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
            controlMode = MODE_HOVER;
            break;
        }

	    if(current_state.armed){
            if(current_state.mode == "GUIDED"){
                switch (key){
                    case KEY_W:{   // w->119 向上
                        target_veloity.linear.z = target_veloity.linear.z+scale;
                        vel_change = true;
                        if(controlMode == MODE_HOVER){
                            mode_int = false; 
                            controlMode = MODE_VELOCITY;
                        }
                        }break;
                    case KEY_A:{   // a->97 左转
                        target_veloity.angular.z = target_veloity.angular.z+scale;
                        vel_change = true;
                        if(controlMode == MODE_HOVER){
                            mode_int = false; 
                            controlMode = MODE_VELOCITY;
                        }
                        }break;
                    case KEY_S:{   //s->115  向下
                        target_veloity.linear.z = target_veloity.linear.z-scale;
                        vel_change = true;
                        if(controlMode == MODE_HOVER){
                            mode_int = false; 
                            controlMode = MODE_VELOCITY;
                        }
                        }break;
                    case KEY_D:{   //d->100  右转
                        target_veloity.angular.z = target_veloity.angular.z-scale;
                        vel_change = true;
                        if(controlMode == MODE_HOVER){
                            mode_int = false; 
                            controlMode = MODE_VELOCITY;
                        }
                        }break;
                    case KEY_I:{   //i->105  向前
                        target_veloity.linear.x = target_veloity.linear.x+scale;
                        vel_change = true;
                        if(controlMode == MODE_HOVER){
                            mode_int = false; 
                            controlMode = MODE_VELOCITY;
                        }
                        }break;
                    case KEY_J:{   //j->106  向左
                        target_veloity.linear.y = target_veloity.linear.y+scale;
                        vel_change = true;
                        if(controlMode == MODE_HOVER){
                            mode_int = false; 
                            controlMode = MODE_VELOCITY;
                        }
                        }break;
                    case KEY_K:{   //k->107  向后
                        target_veloity.linear.x = target_veloity.linear.x-scale;
                        vel_change = true;
                        if(controlMode == MODE_HOVER){
                            mode_int = false; 
                            controlMode = MODE_VELOCITY;
                        }
                        }break;
                    case KEY_L:{   //l->108  向右
                        target_veloity.linear.y = target_veloity.linear.y-scale;
                        vel_change = true;
                        if(controlMode == MODE_HOVER){
                            mode_int = false; 
                            controlMode = MODE_VELOCITY;
                        }
                        }break;
                    case KEY_SPACE:{    //Space->32 悬停
                        mode_int = false; 
                        controlMode = MODE_HOVER; 
                        }break;
                    case KEY_M:{   //m->109 模式选择
                        mode_int = false;               
                        controlMode = MODE_HOVER;      //只要进入m的参数输入，等待时候，就进入悬停模式，等待操作完成
                        printf("Please select guided control mode: \r\n \
                        v: velocity control;\r\n \
                        p: position control;\r\n \
                        a: Tracking the trajectory of falio with position;\r\n \
                        s: Using attitude method to control position;\r\n \
                        f: Fixed trajectory \r\n \
                        t: Tracking the trajectory of falio; \r\n \
                        y: Test for controller performance \r\n \
                        \r\n");

                        key=sh_getch();     //按下m进行功能选择
                        printf("get key value:%d\r\n",key);
                        switch (key){                       
                            case KEY_V:{  //v->118
                                mode_int = false;  //FLAG
                                controlMode = MODE_VELOCITY;
                                printf("Current control mode: \"Veolcity\"\r\n");
                                }break;
                            case KEY_Y:{
                                mode_int = false;
                                controlMode = MODE_HOVER_CUSTOM;
                                printf("Current control mode: \"MODE_HOVER_CUSTOM\"\r\n");
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
                                    target_position.pose.position.x = xt ; //+ localposition.x
                                    target_position.pose.position.y = yt ; //+ localposition.y
                                    target_position.pose.position.z = zt ; //+ localposition.z
                                    target_position.pose.orientation.x = localpose.x;
                                    target_position.pose.orientation.y = localpose.y;
                                    target_position.pose.orientation.z = localpose.z;
                                    target_position.pose.orientation.w = localpose.w;

                                    xt = target_position.pose.position.x;
                                    yt = target_position.pose.position.y;
                                    zt = target_position.pose.position.z;
                                    
                                    mode_int = false;                       //改变模式
                                    controlMode = MODE_POSITION;                                
                                    printf("Absolute position set [%f, %f, %f] successfully!\r\n",xt,yt,zt);
                                }else{
                                    printf("Position setting canceled\r\n");
                                }
                                }break;
                            case KEY_A:{  //a->97
                                printf("please Set target point position  in \"x  y  z\" :\n");
                                xyz_d_def goalposition_;
                                scanf("%lf %lf %lf",&(goalposition_.x),&(goalposition_.y),&(goalposition_.z));
                                clear_scanf_buffer();       //清除scanf中的缓冲数据

                                printf("receive position is [%f, %f, %f] :\r\n",goalposition_.x,goalposition_.y,goalposition_.z);
                                printf("Please press enter to confirm, any other key to cancel\r\n");
                            
                                key=sh_getch();                 //获取确认按键

                                if(key == KEY_ENTER){
                                    target_position.pose.position.x = goalposition_.x + localposition.x;
                                    target_position.pose.position.y = goalposition_.y + localposition.y;
                                    target_position.pose.position.z = goalposition_.z + localposition.z;

                                    target_position.pose.orientation.x = localpose.x;
                                    target_position.pose.orientation.y = localpose.y;       //确定航向
                                    target_position.pose.orientation.z = localpose.z;
                                    target_position.pose.orientation.w = localpose.w;

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
                                    target_position.pose.position.x = xt + localposition.x;
                                    target_position.pose.position.y = yt + localposition.y;
                                    target_position.pose.position.z = zt + localposition.z;
                                    target_position.pose.orientation.x = localpose.x;
                                    target_position.pose.orientation.y = localpose.y;
                                    target_position.pose.orientation.z = localpose.z;
                                    target_position.pose.orientation.w = localpose.w;
                
                                    Eigen::Quaterniond q(target_position.pose.orientation.w, target_position.pose.orientation.x, target_position.pose.orientation.y, target_position.pose.orientation.z);
                                    toEulerAngle(q,target_pose_euler.roll,target_pose_euler.pitch,target_pose_euler.yaw);
                                    
                                    mode_int = false;
                                    controlMode = MODE_STRAIGHT;

                                    printf("Absolute position set [%f, %f, %f] successfully!\r\n",xt,yt,zt);
                                }else{
                                    printf("Position setting canceled\r\n");
                                }
                                }break;  
                            case KEY_T: {
                                printf("please Set target point position  in \"x  y  z\" :\n");
                                xyz_d_def goalposition_;
                                scanf("%lf %lf %lf",&(goalposition_.x),&(goalposition_.y),&(goalposition_.z));
                                clear_scanf_buffer();       //清除scanf中的缓冲数据

                                printf("receive position is [%f, %f, %f] :\r\n",goalposition_.x,goalposition_.y,goalposition_.z);
                                printf("Please press enter to confirm, any other key to cancel\r\n");
                            
                                key=sh_getch();                 //获取确认按键

                                if(key == KEY_ENTER){
                                    target_position.pose.position.x = goalposition_.x + localposition.x;        //带坐标转换，从笛卡尔转到东北天
                                    target_position.pose.position.y = goalposition_.y + localposition.y;
                                    target_position.pose.position.z = goalposition_.z + localposition.z;

                                    target_position.pose.orientation.x = localpose.x;
                                    target_position.pose.orientation.y = localpose.y;       //确定航向
                                    target_position.pose.orientation.z = localpose.z;
                                    target_position.pose.orientation.w = localpose.w;

                                    Eigen::Quaterniond q(target_position.pose.orientation.w, target_position.pose.orientation.x, target_position.pose.orientation.y, target_position.pose.orientation.z);
                                    toEulerAngle(q,target_pose_euler.roll,target_pose_euler.pitch,target_pose_euler.yaw);    

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
                controlMode = MODE_HOVER; 

                if (key == KEY_S){
                    printf("Please continue to press the W key to switch to guided mode, and exit with other keys\r\n");
                    key = sh_getch(); 
                    if(key == KEY_W){
                        printf("Press the enter key to switch to guided mode, and use other keys to exit\r\n");

                        key = sh_getch();
                        if(key == KEY_ENTER){

                            auto guided_req = std::make_shared<mavros_msgs::srv::SetMode::Request>();
                            guided_req->custom_mode = "GUIDED";

                            auto frame_req = std::make_shared<mavros_msgs::srv::SetMavFrame::Request>();
                            frame_req->mav_frame = 1;
                            
                            auto set_mode_future = set_mode_client->async_send_request(guided_req);
                            rclcpp::spin_until_future_complete(g_node, set_mode_future);
                            if (set_mode_future.get()->mode_sent)
                            {
                                printf("[INFO] Guided enabled\n");

                                    auto set_frame_future = set_frame_client->async_send_request(frame_req);
                                    rclcpp::spin_until_future_complete(g_node, set_frame_future);
                                    if (set_frame_future.get()->success) {
                                        printf("[INFO] Set BODY_FRAME of velocity successfully\n");
                                        mode_int = false; 
                                        controlMode = MODE_HOVER; 
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
}

// Empty handler for SIGUSR1 — interrupts blocking getchar() in the keyboard
// thread (returns EINTR) without terminating the process.
static void sigusr1_wake_handler(int) {}

int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    signal(SIGUSR1, sigusr1_wake_handler);
    auto node = std::make_shared<rclcpp::Node>("keyboard_vel_controller");
    g_node = node;

    ButterWorthFIlter* butterfilter_predicted_x = new ButterWorthFIlter(100, 10);
    ButterWorthFIlter* butterfilter_predicted_y = new ButterWorthFIlter(100, 10);
    ButterWorthFIlter* butterfilter_predicted_z = new ButterWorthFIlter(100, 10);

    ButterWorthFIlter* butterfilter_vel_x = new ButterWorthFIlter(100, 10);
    ButterWorthFIlter* butterfilter_vel_y = new ButterWorthFIlter(100, 10);
    ButterWorthFIlter* butterfilter_vel_z = new ButterWorthFIlter(100, 10);
    
    read_param(node.get());

    position_control_x.setLimit(vel_out_limit.x, -vel_out_limit.x);
    position_control_y.setLimit(vel_out_limit.y, -vel_out_limit.y);
    position_control_z.setLimit(vel_out_limit.z, -vel_out_limit.z);
    position_control_x.setPID(position_pid.x.p,  position_pid.x.i,  position_pid.x.d);
    position_control_y.setPID(position_pid.y.p,  position_pid.y.i,  position_pid.y.d);
    position_control_z.setPID(position_pid.z.p,  position_pid.z.i,  position_pid.z.d);

    vel_control_x.setLimit(GRAVITY_ACC * tan(pose_out_limit.x * M_PI/180.0f),
                          -GRAVITY_ACC * tan(pose_out_limit.x * M_PI/180.0f));
    vel_control_y.setLimit(GRAVITY_ACC * tan(pose_out_limit.y * M_PI/180.0f),
                          -GRAVITY_ACC * tan(pose_out_limit.y * M_PI/180.0f));
    vel_control_z.setLimit(GRAVITY_ACC * tan(pose_out_limit.z * M_PI/180.0f),
                          -GRAVITY_ACC * tan(pose_out_limit.z * M_PI/180.0f));

    vel_control_x.setPID(vel_pid.x.p,  vel_pid.x.i,  vel_pid.x.d);
    vel_control_y.setPID(vel_pid.y.p,  vel_pid.y.i,  vel_pid.y.d);
    vel_control_z.setPID(vel_pid.z.p,  vel_pid.z.i,  vel_pid.z.d);

    vel_control_l_x.setLimit(GRAVITY_ACC * tan(pose_out_limit.y * M_PI/180.0f),
                            -GRAVITY_ACC * tan(pose_out_limit.y * M_PI/180.0f));
    vel_control_l_y.setLimit(GRAVITY_ACC * tan(pose_out_limit.y * M_PI/180.0f),
                            -GRAVITY_ACC * tan(pose_out_limit.y * M_PI/180.0f));
    vel_control_l_z.setLimit(GRAVITY_ACC * tan(pose_out_limit.z * M_PI/180.0f),
                            -GRAVITY_ACC * tan(pose_out_limit.z * M_PI/180.0f));
    vel_control_l_x.setPID(vel_l_pid.x.p,  vel_l_pid.x.i,  vel_l_pid.x.d);
    vel_control_l_y.setPID(vel_l_pid.y.p,  vel_l_pid.y.i,  vel_l_pid.y.d);
    vel_control_l_z.setPID(vel_l_pid.z.p,  vel_l_pid.z.i,  vel_l_pid.z.d);

    vel_control_a_x.setLimit(GRAVITY_ACC * tan(pose_out_limit.y * M_PI/180.0f),
                            -GRAVITY_ACC * tan(pose_out_limit.y * M_PI/180.0f));
    vel_control_a_y.setLimit(GRAVITY_ACC * tan(pose_out_limit.y * M_PI/180.0f),
                            -GRAVITY_ACC * tan(pose_out_limit.y * M_PI/180.0f));
    vel_control_a_z.setLimit(GRAVITY_ACC * tan(pose_out_limit.z * M_PI/180.0f),
                            -GRAVITY_ACC * tan(pose_out_limit.z * M_PI/180.0f));
    vel_control_a_x.setPID(vel_a_pid.x.p,  vel_a_pid.x.i,  vel_a_pid.x.d);
    vel_control_a_y.setPID(vel_a_pid.y.p,  vel_a_pid.y.i,  vel_a_pid.y.d);
    vel_control_a_z.setPID(vel_a_pid.z.p,  vel_a_pid.z.i,  vel_a_pid.z.d);

    // ROS2 发布器
    auto vel_pub = node->create_publisher<geometry_msgs::msg::Twist>(
        "/mavros/setpoint_velocity/cmd_vel_unstamped", 1);
    auto position_pub = node->create_publisher<geometry_msgs::msg::PoseStamped>(
        "/mavros/setpoint_position/local", 1);
    auto pose_pub = node->create_publisher<mavros_msgs::msg::AttitudeTarget>(
        "/mavros/setpoint_raw/attitude", 1);
    auto acc_pub = node->create_publisher<mavros_msgs::msg::PositionTarget>(
        "/mavros/setpoint_raw/local", 1);
    auto target_setpositions_pub = node->create_publisher<geometry_msgs::msg::PoseStamped>(
        "/move_base_simple/goal", 1);
    auto command_stop_pub = node->create_publisher<std_msgs::msg::Int64>(
        "/planner_node/stop", 1);

    refTrajectoryPub = node->create_publisher<nav_msgs::msg::Path>(
        "trajectory_publisher/reftrajectory", 1);
    curTrajectoryPub = node->create_publisher<nav_msgs::msg::Path>(
        "trajectory_publisher/curtrajectory", 1);
    referencePosePub = node->create_publisher<geometry_msgs::msg::PoseStamped>(
        "reference/pose", 1);

    // ROS2 subscribers
    auto pos_sub2 = node->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/mavros/local_position/pose", rclcpp::SensorDataQoS(), poseCallback);
    auto odom_sub = node->create_subscription<geometry_msgs::msg::TwistStamped>(
        "/mavros/local_position/velocity_local", rclcpp::SensorDataQoS(), velCallback);
    auto state_sub = node->create_subscription<mavros_msgs::msg::State>(
        "/mavros/state", 1, state_cb);
    auto pathsub = node->create_subscription<nav_msgs::msg::Path>(
        "/search_node/trajectory_position", 1, pathsub_cb);
    auto nearest_obstacle_distance = node->create_subscription<std_msgs::msg::Float32>(
        "/front_obstacle_distance", 1, nearest_distance_cb);
    auto imu_data = node->create_subscription<sensor_msgs::msg::Imu>(
        "/mavros/imu/data", rclcpp::SensorDataQoS(), imudata_cb);

    // ROS2 service clients
    auto arming_client = node->create_client<mavros_msgs::srv::CommandBool>(
        "/mavros/cmd/arming");
    set_mode_client = node->create_client<mavros_msgs::srv::SetMode>(
        "/mavros/set_mode");
    set_frame_client = node->create_client<mavros_msgs::srv::SetMavFrame>(
        "/mavros/setpoint_velocity/mav_frame");
    takeoff_client = node->create_client<mavros_msgs::srv::CommandTOL>(
        "/mavros/cmd/takeoff");
    
    // Wait for takeoff service
    takeoff_client->wait_for_service(std::chrono::seconds(5));

    rclcpp::Rate rate(10);
    rclcpp::Rate rate_main(control_frequency);

    while ((localacceleration.z == 0) && rclcpp::ok()) {
        printf("wait~~~\r\n");
        rclcpp::spin_some(node);
        rate.sleep();
    }

    for (int i = 0; (i < 50 && rclcpp::ok()); i++) {
        Eigen::Quaterniond eigen_local_q_(localpose.w, localpose.x, localpose.y, localpose.z);
        Eigen::Vector3d body_acc_(localacceleration.x, localacceleration.y, localacceleration.z);
        Eigen::Vector3d local_acc_ = eigen_local_q_*body_acc_;      //////////error  

        localacceleration_offset.x += local_acc_(0);
        localacceleration_offset.y += local_acc_(1);
        localacceleration_offset.z += local_acc_(2);
        printf("~~bx:%lf,by:%lf,bz:%lf,lx:%lf,ly:%lf,lz:%lf\r\n",
               body_acc_(0), body_acc_(1), body_acc_(2),
               local_acc_(0), local_acc_(1), local_acc_(2));

        rclcpp::spin_some(node);
        rate.sleep();
    }

    localacceleration_offset.x = localacceleration_offset.x / 50.0f;
    localacceleration_offset.y = localacceleration_offset.y / 50.0f;
    localacceleration_offset.z = localacceleration_offset.z / 50.0f;

    // wait for FCU connection
    while(rclcpp::ok() && !current_state.connected){
        rclcpp::spin_some(node);
        rate.sleep();
    }

    target_veloity.linear.x=0;
    target_veloity.linear.y=0;
    target_veloity.linear.z=0;
    target_veloity.angular.x=0;
    target_veloity.angular.y=0;
    target_veloity.angular.z=0;
    
    target_position.pose.position.x = 0;
    target_position.pose.position.y = 0;
    target_position.pose.position.z = 0;
    target_position.pose.orientation.w = 1.0;   // 单位四元数，保持当前航向不变
    target_position.pose.orientation.x = 0.0;   // 单位四元数，保持当前航向不变
    target_position.pose.orientation.y = 0.0;   // 单位四元数，保持当前航向不变
    target_position.pose.orientation.z = 0.0;   // 单位四元数，保持当前航向不变
    
    // 先发几个点
    for(int i = 20; rclcpp::ok() && i > 0; --i){
        rclcpp::spin_some(node);
        rate.sleep();
        position_pub->publish(target_position);
    }

    auto guided_req2 = std::make_shared<mavros_msgs::srv::SetMode::Request>();
    guided_req2->custom_mode = "GUIDED";

    auto arm_req2 = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
    arm_req2->value = true;

    auto frame_req2 = std::make_shared<mavros_msgs::srv::SetMavFrame::Request>();
    frame_req2->mav_frame = 1;
    
    auto set_mode_future2 = set_mode_client->async_send_request(guided_req2);
    rclcpp::spin_until_future_complete(g_node, set_mode_future2);
    if (set_mode_future2.get()->mode_sent) {
        printf("[INFO] Guided enabled\n");
        auto arm_future2 = arming_client->async_send_request(arm_req2);
        rclcpp::spin_until_future_complete(g_node, arm_future2);
        if (arm_future2.get()->success) {
            printf("[INFO] Vehicle armed\n");

            // Set BODY_FRAME - fire-and-forget, non-critical
            auto set_frame_future2 = set_frame_client->async_send_request(frame_req2);
            // Don't wait — proceed to takeoff immediately

            // Wait for armed state feedback
            while(rclcpp::ok() && !current_state.armed){
                printf("Waiting for unlocking feedback  .....\r\n");
                rclcpp::spin_some(node);
                rate.sleep();
                position_pub->publish(target_position);
            }
            // Wait for GUIDED mode feedback
            while(rclcpp::ok() && (current_state.mode != "GUIDED")){
                printf("Waiting for GUIDED mode feedback  .....\r\n");
                rclcpp::spin_some(node);
                rate.sleep();
                position_pub->publish(target_position);
            }

            // Create keyboard thread but DON'T block on ctrlMode
            pthread_create(&th, NULL, keyboard_listener, 0);
            th_created = true;
            ctrlMode = 1;  // proceed to takeoff immediately

        }
        else {
            printf("[INFO] Failed to arm, please retry! \n");
        }
    }else{
        printf("[INFO] Failed to set Guided, please retry! \n");
    }

    while(rclcpp::ok() && !ctrlMode){
        printf("Waiting for pthread_create  .....\r\n");
        rclcpp::spin_some(node);
        rate.sleep();
        position_pub->publish(target_position);
    }
    printf("[Ready] Controller Mode On !\n");

    // 尝试起飞（带重试）
    auto takeoff_req = std::make_shared<mavros_msgs::srv::CommandTOL::Request>();
    bool takeoff_success = false;
    for (int i = 0; i < 3; ++i) {
        takeoff_req->altitude = 0.3;      // 提高到0.3米，避免高度过小被拒绝
        takeoff_req->min_pitch = 0.0;
        takeoff_req->yaw = 0.0;
        auto takeoff_future = takeoff_client->async_send_request(takeoff_req);
        rclcpp::spin_until_future_complete(g_node, takeoff_future);
        if (takeoff_future.get()->success) {
            RCLCPP_INFO(rclcpp::get_logger("guided_sim"), "Takeoff succeeded (altitude=1.0m)");
            takeoff_success = true;
            break;
        } else {
            RCLCPP_WARN(rclcpp::get_logger("guided_sim"), "Takeoff attempt %d failed, retrying...", i+1);
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
    }

    std::this_thread::sleep_for(std::chrono::seconds(4));
    
    while(rclcpp::ok() && ctrlMode){
        if(current_state.mode != "GUIDED"){
            if(guided_change == true){
                guided_change = false;
                mode_int = false; 
                controlMode = MODE_HOVER; 
                printf("The current mode is %s. Press s, w, and enter in sequence to switch modes\r\n",current_state.mode.c_str());
            }
        }else{
            guided_change = true;
        }

        if(!current_state.armed){
            if(arm_change == true){
                arm_change = false;
                printf("飞机已经锁定,按任意键退出\r\n");
                ctrlMode = 0;
                break;
            }
        }else{
            arm_change = true;
        }

        if(controlMode == MODE_VELOCITY){
            if(!mode_int){
                target_veloity.linear.x=0;
                target_veloity.linear.y=0;
                target_veloity.linear.z=0;
                // target_veloity.angular.x=0;
                // target_veloity.angular.y=0;
                target_veloity.angular.z=0;
                mode_int = true;
                vel_change = true;
                printf("mode velocity init finish\r\n");
            }else{
                vel_pub->publish(target_veloity);
                if(vel_change == true){
                    vel_change = false;
                    printf("mode velocity linear x:%lf,y:%lf,z:%lf,angular:%lf\r\n",target_veloity.linear.x,target_veloity.linear.y,target_veloity.linear.z,target_veloity.angular.z);
                }
            }
        }
        else if (controlMode == MODE_POSITION){
            position_pub->publish(target_position);
        }
        else if (controlMode == MODE_AUTO){
            if(!mode_int){
                pathpoint_sub.clear();
                targetPointNum = 0;

                xyz_d_def coordinate_camera;
                coordinate_camera.x =  target_position.pose.position.y;         //将坐标系从笛卡尔坐标系  转换到东北天坐标系下
                coordinate_camera.y = -target_position.pose.position.x;

                target_position.pose.position.x = coordinate_camera.x;
                target_position.pose.position.y = coordinate_camera.y;
                
                //发送当前的位置，setTargetPose是通过/goal的topic发送的，以camera_init作为的目标点，向前为x正，向左为y正，向上为z正
                target_setpositions_pub->publish(target_position);
                
                printf("publish position X:%lf,Y:%lf,Z:%lf \r\n",target_position.pose.position.x ,target_position.pose.position.y ,target_position.pose.position.z );

                //用于没有收到轨迹的时候，发送位置悬停。
                target_position.pose.position.x = localposition.x;
                target_position.pose.position.y = localposition.y;
                target_position.pose.position.z = localposition.z;

                mode_int = true;
            }
            else{
                if(!(pathpoint_sub.empty()) && targetPointNum < pathpoint_sub.size() ){
                    xyz_d_def pathpoint_in_mavros = {             //将path的坐标转换到local下
                            -(pathpoint_sub[targetPointNum].y),           //x' = -y
                            pathpoint_sub[targetPointNum].x,            //y' =  x
                            pathpoint_sub[targetPointNum].z,            //z' =  z 
                    };

                    // 第二步，发送目标点
                    target_position.pose.position.x = pathpoint_in_mavros.x ;
                    target_position.pose.position.y = pathpoint_in_mavros.y ;
                    target_position.pose.position.z = pathpoint_in_mavros.z ;  
                    position_pub->publish(target_position);

                        
                        //第三步，计算距离
                        double distance = calculate_distance(pathpoint_in_mavros.x,pathpoint_in_mavros.y,pathpoint_in_mavros.z);

                        printf("set:%d,D %lf,X:%lf,Y:%lf,Z:%lf,mX:%lf,mY:%lf,mZ:%lf\r\n",
                            targetPointNum,distance,
                            pathpoint_in_mavros.x,pathpoint_in_mavros.y,pathpoint_in_mavros.z,
                            localposition.x,localposition.y,localposition.z
                        );                   
                        
                        //第四步，判断距离
                        if(distance < ciecle_range){
                            if(targetPointNum == pathpoint_sub.size() - 1){                             //轨迹上的最后一个点进入圈内
                                if(distance < ciecle_range){
                                    targetPointNum++;                                                   //跟踪到最后一个点，完成本次轨迹跟踪
                                    //悬停时的位置
                                    target_position.pose.position.x = localposition.x;
                                    target_position.pose.position.y = localposition.y;
                                    target_position.pose.position.z = localposition.z;                               
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
                        position_pub->publish(target_position);             //改位置在模式初始化的时候，更新的位置
                        printf("hover no point \r\n");
                    }else if(targetPointNum == pathpoint_sub.size()){       //已经到轨迹终点了
                        position_pub->publish(target_position);              //该位置在最后一个点进圈后，更新的位置
                        stop_command.data = 0;
                        command_stop_pub->publish(stop_command);                   
                        printf("finish\r\n");
                    }else{
                        mode_int = false;
                        controlMode = MODE_HOVER;                           //其他情况，切换到悬停模式
                    }
                }
            }
        }
        else if(controlMode == MODE_AUTO_POSE){
            if(!mode_int){
                pathpoint_sub.clear();
                targetPointNum = 0;
                double xt = target_position.pose.position.x;
                double yt = target_position.pose.position.y;
                double zt = target_position.pose.position.z;

                //开始计算轨迹点
                double lx = localposition.x;
                double ly = localposition.y;
                double lz = localposition.z; 

                double distance = sqrt(pow((xt-lx),2)+pow((yt-ly),2)+pow((zt-lz),2)); 
                int num = (int)(distance/0.02f);

                double x_1 = (xt - lx) / num;           //num=100   xt-lx=2   0.02
                double y_1 = (yt - ly) / num;
                double z_1 = (zt - lz) / num;

                xyz_d_def coordinate_camera;
                coordinate_camera.x =  target_position.pose.position.y;
                coordinate_camera.y = -target_position.pose.position.x;

                target_position.pose.position.x = coordinate_camera.x;
                target_position.pose.position.y = coordinate_camera.y;
                
                //发送当前的位置，setTargetPose是通过/goal的topic发送的，以camera_init作为的目标点，向前为x正，向左为y正，向上为z正
                target_setpositions_pub->publish(target_position);
                
                printf("publish position X:%lf,Y:%lf,Z:%lf \r\n",target_position.pose.position.x ,target_position.pose.position.y ,target_position.pose.position.z );

                //用于没有收到轨迹的时候，发送位置悬停。
                target_position.pose.position.x = localposition.x;
                target_position.pose.position.y = localposition.y;
                target_position.pose.position.z = localposition.z;

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
                        xyz_d_def first_path = pathpoint_sub[0];
                        target_position.pose.position.x = -first_path.y;
                        target_position.pose.position.y = first_path.x;
                        target_position.pose.position.z = first_path.z;

                        position_pub->publish(target_position);
                        double distance = calculate_distance(-first_path.y,first_path.x,first_path.z);
                        if(distance <= 0.1f){
                            targetPointNum = 1;
                            PID_Reset();                  

                            printf("first point tracking finish %lf %lf %lf\r\n",target_position.pose.position.x,target_position.pose.position.y,target_position.pose.position.z);
                        } 
                        jiansu_flag = false;
                    }else{

                    //遍历计算所有点，计算与当前位置最近的点
                        double min_distance = 1000000.0f;
                        double lx = localposition.y;
                        double ly = -localposition.x;
                        double lz = localposition.z;
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
                        xyz_d_def pathpoint_in_mavros = {             //将path的坐标转换到local下
                                -(pathpoint_sub[min_targetPointNum].y),           //x' = -y
                                pathpoint_sub[min_targetPointNum].x,            //y' =  x
                                pathpoint_sub[min_targetPointNum].z,            //z' =  z 
                        };
                        // next desired point
                        xyz_d_def pathpoint_in_mavros_next = {             //将path的坐标转换到local下
                                -(pathpoint_sub[min_targetPointNum+jump_pointnumber].y),           //x' = -y
                                pathpoint_sub[min_targetPointNum+jump_pointnumber].x,            //y' =  x
                                pathpoint_sub[min_targetPointNum+jump_pointnumber].z,            //z' =  z 
                        };                   

                    
                        target_position_show.header.stamp = node->now();
                        target_position_show.header.frame_id = "map";
                        target_position_show.pose.position.x = pathpoint_in_mavros.x;
                        target_position_show.pose.position.y = pathpoint_in_mavros.y;
                        target_position_show.pose.position.z = pathpoint_in_mavros.z;
                        target_position_show.pose.orientation.w = localpose.w;
                        target_position_show.pose.orientation.x = localpose.x;
                        target_position_show.pose.orientation.y = localpose.y;
                        target_position_show.pose.orientation.z = localpose.z;
                        referencePosePub->publish(target_position_show);

                        //获取轨迹上的feedforward速度方向，即做速度跟随的主方向
                        xyz_d_def target_vel_main_direction = {   
                            (pathpoint_in_mavros_next.x - pathpoint_in_mavros.x),
                            (pathpoint_in_mavros_next.y - pathpoint_in_mavros.y),
                            (pathpoint_in_mavros_next.z - pathpoint_in_mavros.z) 
                        };   
                        //计算目标的前向速度大小
                        xyz_d_def target_velocity_main ;
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

                        Eigen::Quaterniond eigen_local_q_(localpose.w, localpose.x, localpose.y, localpose.z);
                        Eigen::Vector3d body_acc_(localacceleration.x, localacceleration.y, localacceleration.z);
                        Eigen::Vector3d local_acc_ = eigen_local_q_*body_acc_;      //////////error

                        double jumptime = (double)jump_pointnumber*(1.0f/(double)control_frequency);

                        xyz_d_def predicted_position = {                //预测飞机下一步的位置
                            localposition.x + localvelocity.x * jumptime, //+ 0.5*(local_acc_(0)-localacceleration_offset.x)*(jumptime)*(jumptime),
                            localposition.y + localvelocity.y * jumptime, //+ 0.5*(local_acc_(1)-localacceleration_offset.y)*(jumptime)*(jumptime),
                            localposition.z + localvelocity.z * jumptime  //+ 0.5*(local_acc_(2)-localacceleration_offset.z)*(jumptime)*(jumptime)
                        };

                        xyz_d_def predicted_position_fil = {
                            butterfilter_predicted_x->filter(predicted_position.x),
                            butterfilter_predicted_y->filter(predicted_position.y),
                            butterfilter_predicted_z->filter(predicted_position.z)
                        };        

                        // xld  20260115
                        double target_vel_x_h_pid = 1.2f* position_control_x.calculate(localposition.x, pathpoint_in_mavros.x);              
                        double target_vel_y_h_pid = 1.2f* position_control_y.calculate(localposition.y, pathpoint_in_mavros.y);              
                        double target_vel_z_h_pid = 1.2f* position_control_z.calculate(localposition.z, pathpoint_in_mavros.z); 

                        double target_vel_x_h_pid_LPF =  1.0f * position_control_x.calculate_LPF(target_vel_x_h_pid, 0.0f); 
                        double target_vel_y_h_pid_LPF =  1.0f * position_control_y.calculate_LPF(target_vel_y_h_pid, 0.0f);    
                        double target_vel_z_h_pid_LPF = position_control_z.calculate_LPF(target_vel_z_h_pid, 0.0f);  

                        double target_vel_x_h_pid_BUTTER =  butterfilter_vel_x->filter(target_vel_x_h_pid);
                        double target_vel_y_h_pid_BUTTER =  butterfilter_vel_y->filter(target_vel_y_h_pid);
                        double target_vel_z_h_pid_BUTTER =  butterfilter_vel_z->filter(target_vel_z_h_pid);  

                        double target_vel_x_pid = target_vel_x_h_pid_BUTTER + target_velocity_main.x;
                        double target_vel_y_pid = target_vel_y_h_pid_BUTTER + target_velocity_main.y;
                        double target_vel_z_pid = target_vel_z_h_pid_BUTTER + target_velocity_main.z;         

                        #define DELT_RANGE_VEL (0.02)   //约等于1.728    0.85

                        double vel_delt_x = target_vel_x_pid-target_vel_x_pid_last;
                        if(vel_delt_x > DELT_RANGE_VEL){
                            target_vel_x_pid = target_vel_x_pid_last + DELT_RANGE_VEL;
                        }else if (vel_delt_x < -DELT_RANGE_VEL)
                        {
                            target_vel_x_pid = target_vel_x_pid_last - DELT_RANGE_VEL;
                        }

                        double vel_delt_y = target_vel_y_pid-target_vel_y_pid_last;
                        if(vel_delt_y > DELT_RANGE_VEL){
                            target_vel_y_pid = target_vel_y_pid_last + DELT_RANGE_VEL;
                        }else if (vel_delt_y < -DELT_RANGE_VEL)
                        {
                            target_vel_y_pid = target_vel_y_pid_last - DELT_RANGE_VEL;
                        }

                        double vel_delt_z = target_vel_z_pid-target_vel_z_pid_last;
                        if(vel_delt_z > DELT_RANGE_VEL){
                            target_vel_z_pid = target_vel_z_pid_last + DELT_RANGE_VEL;
                        }else if (vel_delt_z < -DELT_RANGE_VEL)
                        {
                            target_vel_z_pid = target_vel_z_pid_last - DELT_RANGE_VEL;
                        }

                        target_vel_x_pid_last = target_vel_x_pid;
                        target_vel_y_pid_last = target_vel_y_pid;
                        target_vel_z_pid_last = target_vel_z_pid;

                        double target_acc_x_pid = vel_control_a_x.calculate(localvelocity.x,target_vel_x_pid) ;                         //PID的系数设置为0.1~10之间，加速度限制在-5.685~5.685之间，取上下限-6~6之间
                        double target_acc_y_pid = vel_control_a_y.calculate(localvelocity.y,target_vel_y_pid) ;                         //预设最大P值10时，速度相差5m/s的时候，最大值输出。
                        double target_acc_z_pid = vel_control_a_z.calculate(localvelocity.z,target_vel_z_pid) ;                         //即，10*5 * 0.12 = 6


                        #define DELT_RANGE (GRAVITY_ACC * tan(5.0*M_PI/180.0f))   //约等于1.728    0.85

                        double acc_delt_x = target_acc_x_pid-target_acc_x_pid_last;
                        if(acc_delt_x > DELT_RANGE){
                            target_acc_x_pid = target_acc_x_pid_last + DELT_RANGE;
                        }else if (acc_delt_x < -DELT_RANGE)
                        {
                            target_acc_x_pid = target_acc_x_pid_last - DELT_RANGE;
                        }

                        double acc_delt_y = target_acc_y_pid-target_acc_y_pid_last;
                        if(acc_delt_y > DELT_RANGE){
                            target_acc_y_pid = target_acc_y_pid_last + DELT_RANGE;
                        }else if (acc_delt_y < -DELT_RANGE)
                        {
                            target_acc_y_pid = target_acc_y_pid_last - DELT_RANGE;
                        }                                
                        
                        double acc_delt_z = target_acc_z_pid-target_acc_z_pid_last;
                        if(acc_delt_z > DELT_RANGE){
                            target_acc_z_pid = target_acc_z_pid_last + DELT_RANGE;
                        }else if (acc_delt_z < -DELT_RANGE)
                        {
                            target_acc_z_pid = target_acc_z_pid_last - DELT_RANGE;
                        }

                        target_acc_x_pid_last = target_acc_x_pid;
                        target_acc_y_pid_last = target_acc_y_pid;
                        target_acc_z_pid_last = target_acc_z_pid;

                        //转换到机体坐标系下，将目标加速度转换成飞机的目标姿态
                        Eigen::Quaterniond q(localpose.w, localpose.x, localpose.y, localpose.z);                                   //当前姿态四元素，转换为欧拉角
                        double yaw,roll,pitch;
                        toEulerAngle(q,roll,pitch,yaw);

                        //转换到机体坐标系下
                        double target_acc_x_pid_in_body = target_acc_x_pid * cos(yaw) + target_acc_y_pid * sin(yaw);
                        double target_acc_y_pid_in_body = -target_acc_x_pid * sin(yaw) + target_acc_y_pid * cos(yaw);

                        //换算成角度
                        double target_pitch_pid =  atan2(target_acc_x_pid_in_body,GRAVITY_ACC);                                         //加速度转换成姿态
                        double target_roll_pid  = -atan2(target_acc_y_pid_in_body,GRAVITY_ACC);                                         //根据右手法则，roll的目标差值需要反向
                        #define UAV_WEIGHT 3.340f
                        double target_height_pid = target_acc_z_pid * UAV_WEIGHT / (cos(target_pitch_pid) * cos(target_roll_pid)) *0.2f;                                         //高度不做控制


                        //获取当前的目标点需要的航向角
                        double target_yaw = target_pose_euler.yaw;

                        //欧拉角转换成四元素 发送出去，航向角不变，
                        Eigen::AngleAxisd rollAngle_(Eigen::AngleAxisd(target_roll_pid,Eigen::Vector3d::UnitX()));      //pid计算的目标航向
                        Eigen::AngleAxisd pitchAngle_(Eigen::AngleAxisd(target_pitch_pid,Eigen::Vector3d::UnitY()));
                        Eigen::AngleAxisd yawAngle_(Eigen::AngleAxisd(target_pose_euler.yaw,Eigen::Vector3d::UnitZ()));        //进入模式的时候，设定的目标航向
                        
                        Eigen::Quaterniond quaternion_;                                     
                        quaternion_=yawAngle_*pitchAngle_*rollAngle_;

                        target_pose.orientation.x = quaternion_.coeffs()[0];                 //角度赋值
                        target_pose.orientation.y = quaternion_.coeffs()[1];
                        target_pose.orientation.z = quaternion_.coeffs()[2];
                        target_pose.orientation.w = quaternion_.coeffs()[3];        
                        target_pose.thrust = 0.43f + target_height_pid;                     //设置油门
                        if(target_pose.thrust > 1.0f){
                            target_pose.thrust = 1.0f;
                        }else if(target_pose.thrust < 0.0f){
                            target_pose.thrust = 0.0f;
                        }

                        target_pose.type_mask = 1+2+4;
                        target_pose.header.stamp = node->now();

                        // xld 2026.1.14
                        pose_pub->publish(target_pose);

                        double vel_xy = sqrt(pow(localvelocity.x,2)+pow(localvelocity.y,2)+pow(localvelocity.z,2));

                        double vel_xy_main = sqrt(pow(target_velocity_main.x,2)+pow(target_velocity_main.y,2)+pow(target_velocity_main.z,2));
                        double vel_xy_heng = sqrt(pow(target_vel_x_h_pid,2)+pow(target_vel_y_h_pid,2)+pow(target_vel_z_h_pid,2));
                        double distance = calculate_distance(-pathpoint_sub[min_targetPointNum].y,pathpoint_sub[min_targetPointNum].x,pathpoint_sub[min_targetPointNum].z); 

                        printf("traj num:%d,D:%lf  vel_norm:%lf  vel_ff_norm:%lf  x:%lf  y:%lf  z:%lf  vx_ff:%lf  vy_ff:%lf  vz_ff:%lf vx_fb:%lf  vy_fb:%lf  vz_fb:%lf vx:%lf  vy:%lf  vz:%lf  x_des:%lf  y_des:%lf  z_des:%lf  vx_des:%lf  vy_des:%lf  vz_des:%lf\r\n",
                        min_targetPointNum,distance,vel_xy,vel_xy_main,
                        localposition.x,localposition.y,localposition.z,
                        target_velocity_main.x,target_velocity_main.y,target_velocity_main.z,
                        target_vel_x_h_pid_BUTTER, target_vel_y_h_pid_BUTTER,target_vel_z_h_pid_BUTTER,
                        localvelocity.x,localvelocity.y,localvelocity.z,
                        pathpoint_in_mavros_next.x,pathpoint_in_mavros_next.y,pathpoint_in_mavros_next.z,
                        target_vel_x_pid,target_vel_y_pid,target_vel_z_pid
                        ); 



                        if((pathpoint_sub.size() - 30) > 0 ){
                            if(targetPointNum == pathpoint_sub.size() - 30){                             //轨迹上的最后一个点进入圈内
                                targetPointNum = pathpoint_sub.size();     
                                // target_position.pose.position.x = localposition.x;
                                // target_position.pose.position.y = localposition.y;
                                // target_position.pose.position.z = localposition.z;  

                                target_position.pose.position.x = -pathpoint_sub[pathpoint_sub.size() - 1].y;
                                target_position.pose.position.y = pathpoint_sub[pathpoint_sub.size() - 1].x;
                                target_position.pose.position.z = pathpoint_sub[pathpoint_sub.size() - 1].z;    
                            }
                        }else{

                           
                            if(targetPointNum ==  (pathpoint_sub.size() - 1 - jump_pointnumber)){          //轨迹上的最后一个点进入圈内
                                targetPointNum = pathpoint_sub.size();     
                                // target_position.pose.position.x = localposition.x;
                                // target_position.pose.position.y = localposition.y;
                                // target_position.pose.position.z = localposition.z;  

                                target_position.pose.position.x = -pathpoint_sub[pathpoint_sub.size() - 1].y;
                                target_position.pose.position.y = pathpoint_sub[pathpoint_sub.size() - 1].x;
                                target_position.pose.position.z = pathpoint_sub[pathpoint_sub.size() - 1].z;    
                            }                            
                        }
                    }
                }             
                else{
                    if(targetPointNum == 0){                                //没有收到轨迹
                        position_pub->publish(target_position);             //改位置在模式初始化的时候，更新的位置
                        if(pathpoint_sub.size() == 0){
                            printf("hover no point \r\n");
                        }else{
                            printf("point too small \r\n");
                        }
                        

                    }else if(targetPointNum == pathpoint_sub.size()){       //已经到轨迹终点了
                        position_pub->publish(target_position);              //该位置在最后一个点进圈后，更新的位置
                        stop_command.data = 0;
                        command_stop_pub->publish(stop_command);                   
                        printf("finish~~~ tx:%lf ty:%lf tz:%lf lx:%lf ly:%lf lz:%lf \r\n",target_position.pose.position.x,target_position.pose.position.y,target_position.pose.position.z,localposition.x,localposition.y,localposition.z);


                    }else{
                        mode_int = false;
                        controlMode = MODE_HOVER;                           //其他情况，切换到悬停模式
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
                double target_vel_x_pid = position_control_x.calculate(localposition.x,target_position.pose.position.x);               //PID的系数设置为0.1~10 之间，速度现在-1~1之间
                double target_vel_y_pid = position_control_y.calculate(localposition.y,target_position.pose.position.y);               //预设最大P值10时，距离相差1m的时候，最大值输出。
                double target_vel_z_pid = position_control_z.calculate(localposition.z,target_position.pose.position.z);               //即，10*1*0.1 = 1m/s

                double target_acc_x_pid = vel_control_x.calculate(localvelocity.x,target_vel_x_pid) ;                         //PID的系数设置为0.1~10之间，加速度限制在-5.685~5.685之间，取上下限-6~6之间
                double target_acc_y_pid = vel_control_y.calculate(localvelocity.y,target_vel_y_pid) ;                         //预设最大P值10时，速度相差5m/s的时候，最大值输出。
                double target_acc_z_pid = vel_control_z.calculate(localvelocity.z,target_vel_z_pid) ;                         //即，10*5 * 0.12 = 6

 
                        double max_acc = 5.0f;
                        double max_x_acc = abs(max_acc * (target_acc_x_pid / sqrt(target_acc_x_pid*target_acc_x_pid+target_acc_y_pid*target_acc_y_pid))) ;
                        double max_y_acc = abs( max_acc * (target_acc_y_pid / sqrt(target_acc_x_pid*target_acc_x_pid+target_acc_y_pid*target_acc_y_pid)));

                        if(target_acc_x_pid > GRAVITY_ACC * tan(max_x_acc*M_PI/180.0f)) {
                            target_acc_x_pid = GRAVITY_ACC * tan(max_x_acc*M_PI/180.0f);
                        }else if(target_acc_x_pid < -GRAVITY_ACC * tan(max_x_acc*M_PI/180.0f) ){
                            target_acc_x_pid = -GRAVITY_ACC * tan(max_x_acc*M_PI/180.0f);                           
                        }
                    
                        if(target_acc_y_pid > GRAVITY_ACC * tan(max_y_acc*M_PI/ 180.0f)) {
                            target_acc_y_pid = GRAVITY_ACC * tan(max_y_acc*M_PI/180.0f);
                        }else if(target_acc_y_pid < -GRAVITY_ACC * tan(max_y_acc*M_PI/180.0f) ){
                            target_acc_y_pid = -GRAVITY_ACC * tan(max_y_acc*M_PI/180.0f);                           
                        }
                        
                        if(target_acc_z_pid > GRAVITY_ACC * tan(10.0f*M_PI/ 180.0f)) {
                            target_acc_z_pid = GRAVITY_ACC * tan(10.0f*M_PI/180.0f);
                        }else if(target_acc_z_pid < -GRAVITY_ACC * tan(10.0f*M_PI/180.0f) ){
                            target_acc_z_pid = -GRAVITY_ACC * tan(10.0f*M_PI/180.0f);                           
                        }   
 
 
                //以上代码target_vel_x_pid，target_acc_x_pid都为与local的坐标系下，即东北天坐标系

                Eigen::Quaterniond q(localpose.w, localpose.x, localpose.y, localpose.z);                                   //当前姿态四元素，转换为欧拉角
                double yaw,roll,pitch;
                toEulerAngle(q,roll,pitch,yaw);     

                //转换到机体坐标系下
                double target_acc_x_pid_in_body = target_acc_x_pid * cos(yaw) + target_acc_y_pid * sin(yaw);
                double target_acc_y_pid_in_body = -target_acc_x_pid * sin(yaw) + target_acc_y_pid * cos(yaw);

                //换算成角度
                double target_pitch_pid =  atan2(target_acc_x_pid_in_body,GRAVITY_ACC);                                         //加速度转换成姿态
                double target_roll_pid  = -atan2(target_acc_y_pid_in_body,GRAVITY_ACC);                                         //根据右手法则，roll的目标差值需要反向
                #define UAV_WEIGHT 3.340f
                double target_height_pid = target_acc_z_pid * UAV_WEIGHT / (cos(target_pitch_pid) * cos(target_roll_pid)) *0.2f;                                         //高度不做控制


                //欧拉角转换成四元素 发送出去，航向角不变，
                Eigen::AngleAxisd rollAngle_(Eigen::AngleAxisd(target_roll_pid,Eigen::Vector3d::UnitX()));      //pid计算的目标航向
                Eigen::AngleAxisd pitchAngle_(Eigen::AngleAxisd(target_pitch_pid,Eigen::Vector3d::UnitY()));
                Eigen::AngleAxisd yawAngle_(Eigen::AngleAxisd(target_pose_euler.yaw,Eigen::Vector3d::UnitZ()));        //进入模式的时候，设定的目标航向
                
                Eigen::Quaterniond quaternion_;                                     
                quaternion_=yawAngle_*pitchAngle_*rollAngle_;

                target_pose.orientation.x = quaternion_.coeffs()[0];                 //角度赋值
                target_pose.orientation.y = quaternion_.coeffs()[1];
                target_pose.orientation.z = quaternion_.coeffs()[2];
                target_pose.orientation.w = quaternion_.coeffs()[3];        
                target_pose.thrust = 0.43f + target_height_pid;                     //设置油门
                if(target_pose.thrust > 1.0f){
                    target_pose.thrust = 1.0f;
                }else if(target_pose.thrust < 0.0f){
                    target_pose.thrust = 0.0f;
                }

                target_pose.type_mask = 1+2+4;
                target_pose.header.stamp = node->now();

                pose_pub->publish(target_pose);                                            //发送消息
                
                double distance = calculate_distance(target_position.pose.position.x,target_position.pose.position.y,target_position.pose.position.z);

                fout_pid << node->now().seconds() <<","<<localposition.x<<","<<localposition.y<<","<<localposition.z<<","<<
                target_position.pose.position.x<<","<<target_position.pose.position.y<<","<<target_position.pose.position.z<<","<<
                localvelocity.x<<","<<localvelocity.y<<","<<localvelocity.z<<","<<
                target_vel_x_pid<<","<<target_vel_y_pid<<","<<target_vel_z_pid<<","<<
                target_acc_x_pid<<","<<target_acc_y_pid<<","<<target_acc_z_pid<<","<< target_pose.thrust <<
                std::endl;

                printf("target_vel x:%lf,y:%lf,z:%lf\r\n",target_vel_x_pid,target_vel_y_pid,target_vel_z_pid);
                printf("target_acc x:%lf,y:%lf,z:%lf\r\n",target_acc_x_pid,target_acc_y_pid,target_acc_z_pid);
                printf("localvelocity x:%lf,y:%lf,z:%lf\r\n",localvelocity.x,localvelocity.y,localvelocity.z);
                printf("MODE_STRAIGHT: D:%lf,current_position x:%lf,y:%lf,z:%lf target_ouer r:%lf,p:%lf,y:%f,thu:%f,orientation x:%lf,y:%lf,z:%lf,w:%lf\r\n",
                distance,
                localposition.x,localposition.y,localposition.z,
                target_pitch_pid * 180.0f/M_PI,target_roll_pid * 180.0f/M_PI,target_pose_euler.yaw,
                target_pose.thrust,
                target_pose.orientation.x,target_pose.orientation.y,target_pose.orientation.z,target_pose.orientation.w
                );
            }
        }
        else if (controlMode == TEST_FLOOR_TRAJ){
            if(!mode_int){
                pathpoint_sub_test.clear();
                targetPointNum_test = 0;

                // 总的航迹点
                {
                    float XLOCAL = target_x;
                    float YLOCAL = target_y;
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

                    pathpoint_sub_test.push_back({  0 + localposition.x,             //rrrrrrrree
                                                    0 + localposition.y,
                                                    0 + localposition.z,
                                                    M_PI/2 + ANGLE_DIFF
                    });                    

                    pathpoint_sub_test.push_back({  XLOCAL + localposition.x,             //rrrrrrrree
                                                    YLOCAL + localposition.y,
                                                    0 + localposition.z,
                                                    M_PI/2 + ANGLE_DIFF
                    });

                    pathpoint_sub_test.push_back({ XLOCAL + localposition.x,             //rrrrrrrree
                                                    YLOCAL + localposition.y,
                                                    0.5 + localposition.z,
                                                    M_PI*3/2 + ANGLE_DIFF
                    });

                    pathpoint_sub_test.push_back({ 0 + localposition.x,             //rrrrrrrree
                                                    0 + localposition.y,
                                                    0.5 + localposition.z,
                                                    M_PI*3/2 + ANGLE_DIFF
                    });

                    pathpoint_sub_test.push_back({ 0 + localposition.x,             //rrrrrrrree
                                                    0 + localposition.y,
                                                    1.0 + localposition.z,
                                                    M_PI/2 + ANGLE_DIFF
                    });

                    pathpoint_sub_test.push_back({ XLOCAL + localposition.x,             //rrrrrrrree
                                                    YLOCAL + localposition.y,
                                                    1.0 + localposition.z,
                                                    M_PI/2 + ANGLE_DIFF
                    });

                    pathpoint_sub_test.push_back({ XLOCAL + localposition.x,             //rrrrrrrree
                                                    YLOCAL + localposition.y,
                                                    1.5 + localposition.z,
                                                    M_PI*3/2 + ANGLE_DIFF
                    });


                    pathpoint_sub_test.push_back({ 0 + localposition.x,             //rrrrrrrree
                                                    0 + localposition.y,
                                                    1.5 + localposition.z,
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

                        Eigen::Quaterniond q(localpose.w, localpose.x, localpose.y, localpose.z);                                   //当前姿态四元素，转换为欧拉角
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
                                 lx = localposition.x;    
                                 ly = localposition.y;
                                 lz = localposition.z;                                 
                            }else{
                                 lx = pathpoint_sub_test[targetPointNum_test-1].x;   
                                 ly = pathpoint_sub_test[targetPointNum_test-1].y;
                                 lz = pathpoint_sub_test[targetPointNum_test-1].z;
                            }
                            //转换到机体坐标系下，将目标加速度转换成飞机的目标姿态
                            Eigen::Quaterniond q_1(localpose.w, localpose.x, localpose.y, localpose.z);                                   //当前姿态四元素，转换为欧拉角
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
                            double ly = localposition.y;
                            double lx = localposition.x;
                            double lz = localposition.z;
                            int min_targetPointNum = 0;
                            for(int i=0;i<fill_pathpoint.size();i++){
                                double dis = pow((ly-fill_pathpoint[i].y),2) + pow((lx-fill_pathpoint[i].x),2) + pow((lz-fill_pathpoint[i].z),2);
                                if(dis < min_distance){
                                    min_distance = dis;
                                    min_targetPointNum = i;
                                }
                            }

                            //获取轨迹上的速度方向，即做速度跟随的主方向
                            xyz_d_def target_vel_main_direction;
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
                            xyz_d_def target_velocity_main = calculate_FixedModulusVector(target_vel_main_direction, 0.1);   //目标前向速度

                            Eigen::Quaterniond eigen_local_q_(localpose.w, localpose.x, localpose.y, localpose.z);
                            Eigen::Vector3d body_acc_(localacceleration.x, localacceleration.y, localacceleration.z);
                            Eigen::Vector3d local_acc_ = eigen_local_q_*body_acc_;      //////////error

                            int forwardPoint = min_targetPointNum;        //预测飞机跟踪轨迹的点位
                            if(forwardPoint + 1 <= fill_pathpoint.size()-1){
                                forwardPoint = forwardPoint + 1;             
                            }else{
                                forwardPoint =  fill_pathpoint.size()-1;
                            }                                

                            double target_vel_x_h_pid = 1.0f* position_control_x.calculate(localposition.x, fill_pathpoint[forwardPoint].x);              
                            double target_vel_y_h_pid = 1.0f* position_control_y.calculate(localposition.y, fill_pathpoint[forwardPoint].y);              
                            double target_vel_z_h_pid = 1.0f* position_control_z.calculate(localposition.z, fill_pathpoint[forwardPoint].z);  

                            double target_vel_x_h_pid_BUTTER =  butterfilter_vel_x->filter(target_vel_x_h_pid);
                            double target_vel_y_h_pid_BUTTER =  butterfilter_vel_y->filter(target_vel_y_h_pid);
                            double target_vel_z_h_pid_BUTTER =  butterfilter_vel_z->filter(target_vel_z_h_pid);

                            //横向控制做预测机结束
                            double target_vel_x_pid = target_vel_x_h_pid_BUTTER + target_velocity_main.x;
                            double target_vel_y_pid = target_vel_y_h_pid_BUTTER + target_velocity_main.y;  
                            double target_vel_z_pid = target_vel_z_h_pid_BUTTER + target_velocity_main.z;                          

                            double target_acc_x_pid = vel_control_a_x.calculate(localvelocity.x,target_vel_x_pid);                         //PID的系数设置为0.1~10之间，加速度限制在-5.685~5.685之间，取上下限-6~6之间
                            double target_acc_y_pid = vel_control_a_y.calculate(localvelocity.y,target_vel_y_pid);                         //预设最大P值10时，速度相差5m/s的时候，最大值输出。
                            double target_acc_z_pid = vel_control_a_z.calculate(localvelocity.z,target_vel_z_pid);  

                            #define DELT_RANGE (GRAVITY_ACC * tan(5.0*M_PI/180.0f))   //约等于1.728    0.85

                            double acc_delt_x = target_acc_x_pid-target_acc_x_pid_last;
                            if(acc_delt_x > DELT_RANGE){
                                target_acc_x_pid = target_acc_x_pid_last + DELT_RANGE;
                            }else if (acc_delt_x < -DELT_RANGE)
                            {
                                target_acc_x_pid = target_acc_x_pid_last - DELT_RANGE;
                            }

                            double acc_delt_y = target_acc_y_pid-target_acc_y_pid_last;
                            if(acc_delt_y > DELT_RANGE){
                                target_acc_y_pid = target_acc_y_pid_last + DELT_RANGE;
                            }else if (acc_delt_y < -DELT_RANGE)
                            {
                                target_acc_y_pid = target_acc_y_pid_last - DELT_RANGE;
                            }                                
                            
                            double acc_delt_z = target_acc_z_pid-target_acc_z_pid_last;
                            if(acc_delt_z > DELT_RANGE){
                                target_acc_z_pid = target_acc_z_pid_last + DELT_RANGE;
                            }else if (acc_delt_z < -DELT_RANGE)
                            {
                                target_acc_z_pid = target_acc_z_pid_last - DELT_RANGE;
                            }

                            target_acc_x_pid_last = target_acc_x_pid;
                            target_acc_y_pid_last = target_acc_y_pid;
                            target_acc_z_pid_last = target_acc_z_pid;

                            //转换到机体坐标系下，将目标加速度转换成飞机的目标姿态
                            Eigen::Quaterniond q(localpose.w, localpose.x, localpose.y, localpose.z);                                   //当前姿态四元素，转换为欧拉角
                            double yaw,roll,pitch;
                            toEulerAngle(q,roll,pitch,yaw);

                            //转换到机体坐标系下
                            double target_acc_x_pid_in_body = target_acc_x_pid * cos(yaw) + target_acc_y_pid * sin(yaw);
                            double target_acc_y_pid_in_body = -target_acc_x_pid * sin(yaw) + target_acc_y_pid * cos(yaw);

                            //换算成角度
                            double target_pitch_pid =  atan2(target_acc_x_pid_in_body,GRAVITY_ACC);                                         //加速度转换成姿态
                            double target_roll_pid  = -atan2(target_acc_y_pid_in_body,GRAVITY_ACC);                                         //根据右手法则，roll的目标差值需要反向
                            #define UAV_WEIGHT 3.340f
                            double target_height_pid = target_acc_z_pid * UAV_WEIGHT / (cos(target_pitch_pid) * cos(target_roll_pid)) *0.2f;       //高度不做控制

                            //获取当前的目标点需要的航向角
                            double target_yaw = fill_yaw[forwardPoint];

                            //欧拉角转换成四元素 发送出去，航向角不变，
                            Eigen::AngleAxisd rollAngle_(Eigen::AngleAxisd(target_roll_pid,Eigen::Vector3d::UnitX()));      //pid计算的目标航向
                            Eigen::AngleAxisd pitchAngle_(Eigen::AngleAxisd(target_pitch_pid,Eigen::Vector3d::UnitY()));
                            Eigen::AngleAxisd yawAngle_(Eigen::AngleAxisd(target_yaw,Eigen::Vector3d::UnitZ()));        //进入模式的时候，设定的目标航向
                            
                            Eigen::Quaterniond quaternion_;
                            quaternion_=yawAngle_*pitchAngle_*rollAngle_;

                            target_pose.orientation.x = quaternion_.coeffs()[0];                 //角度赋值
                            target_pose.orientation.y = quaternion_.coeffs()[1];
                            target_pose.orientation.z = quaternion_.coeffs()[2];
                            target_pose.orientation.w = quaternion_.coeffs()[3];        
                            target_pose.thrust = 0.43f + target_height_pid;                     //设置油门
                            if(target_pose.thrust > 1.0f){
                                target_pose.thrust = 1.0f;
                            }else if(target_pose.thrust < 0.0f){
                                target_pose.thrust = 0.0f;
                            }

                            target_pose.type_mask = 1+2+4;
                            target_pose.header.stamp = node->now();

                            pose_pub->publish(target_pose);    //ZZXZZX              
                        
                            //第三步，计算距离,计算航向
                            double distance = calculate_distance(pathpoint_sub_test[targetPointNum_test].x,pathpoint_sub_test[targetPointNum_test].y,pathpoint_sub_test[targetPointNum_test].z);

                            double angulardiff = calculate_AngleDifference(pathpoint_sub_test[targetPointNum_test].a,yaw);

                            printf("pure_yaw_set:%d,D %lf,tA:%lf,X:%lf,Y:%lf,Z:%lf,mX:%lf,mY:%lf,mZ:%lf\r\n",
                                targetPointNum_test,distance,target_yaw*180/M_PI,
                                pathpoint_sub_test[targetPointNum_test].x,pathpoint_sub_test[targetPointNum_test].y,pathpoint_sub_test[targetPointNum_test].z,
                                localposition.x,localposition.y,localposition.z
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
                                 lx = localposition.x;    
                                 ly = localposition.y;
                                 lz = localposition.z;                                 
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

                            printf("fill_pathpoint.size()=%d,num=:%d\r\n",fill_pathpoint.size(),num);                                  
                        }

                        printf("obstacle_distance = %lf \r\n",obstacle_distance); 
                        // 1.3.2 判断是否前方有障碍物,进入悬停代码
                        //---------------------------------------------------------------------
                        if(obstacle_distance > 2.0f || isObstacleStop == false){ 
                      
                        //1.3.3  判断前方没有障碍物，则沿轨迹飞行
                            // 1. 遍历计算所有点，计算与当前位置最近的点
                            if(!(fill_pathpoint.empty())  ){  
                                double min_distance = 1000000.0f;
                                double ly = localposition.y;
                                double lx = localposition.x;
                                double lz = localposition.z;
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
                                xyz_d_def target_vel_main_direction;
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
                                xyz_d_def target_velocity_main = calculate_FixedModulusVector(target_vel_main_direction,vel_forward_set);   //目标前向速度                        

                                Eigen::Quaterniond eigen_local_q_(localpose.w, localpose.x, localpose.y, localpose.z);
                                Eigen::Vector3d body_acc_(localacceleration.x, localacceleration.y, localacceleration.z);
                                Eigen::Vector3d local_acc_ = eigen_local_q_*body_acc_;      //////////error
                               
                                // localacceleration;

                                double jumptime = (double)jump_pointnumber*(1.0f/(double)control_frequency);

                                int forwardPoint = min_targetPointNum;        //预测飞机跟踪轨迹的点位
                                if(forwardPoint + jump_pointnumber <= fill_pathpoint.size()-1){
                                    forwardPoint = forwardPoint + jump_pointnumber;             
                                }else{
                                    forwardPoint =  fill_pathpoint.size()-1;
                                }                                

                                //横向控制做预测

                                double target_vel_x_h_pid = position_control_x.calculate(localposition.x, fill_pathpoint[min_targetPointNum].x); 
                                double target_vel_y_h_pid = position_control_y.calculate(localposition.y, fill_pathpoint[min_targetPointNum].y);    
                                double target_vel_z_h_pid = position_control_z.calculate(localposition.z, fill_pathpoint[min_targetPointNum].z);  

                                double target_vel_x_h_pid_BUTTER =  butterfilter_vel_x->filter(target_vel_x_h_pid);
                                double target_vel_y_h_pid_BUTTER =  butterfilter_vel_y->filter(target_vel_y_h_pid);
                                double target_vel_z_h_pid_BUTTER =  butterfilter_vel_z->filter(target_vel_z_h_pid);

                                //横向控制做预测机结束

                                double target_vel_x_pid = target_vel_x_h_pid_BUTTER + target_velocity_main.x;
                                double target_vel_y_pid = target_vel_y_h_pid_BUTTER + target_velocity_main.y;  
                                double target_vel_z_pid = target_vel_z_h_pid_BUTTER + target_velocity_main.z;                          

                                #define DELT_RANGE_VEL (0.02)   //约等于1.728    0.85

                                double vel_delt_x = target_vel_x_pid-target_vel_x_pid_last;
                                if(vel_delt_x > DELT_RANGE_VEL){
                                    target_vel_x_pid = target_vel_x_pid_last + DELT_RANGE_VEL;
                                }else if (vel_delt_x < -DELT_RANGE_VEL)
                                {
                                    target_vel_x_pid = target_vel_x_pid_last - DELT_RANGE_VEL;
                                }

                                double vel_delt_y = target_vel_y_pid-target_vel_y_pid_last;
                                if(vel_delt_y > DELT_RANGE_VEL){
                                    target_vel_y_pid = target_vel_y_pid_last + DELT_RANGE_VEL;
                                }else if (vel_delt_y < -DELT_RANGE_VEL)
                                {
                                    target_vel_y_pid = target_vel_y_pid_last - DELT_RANGE_VEL;
                                }

                                double vel_delt_z = target_vel_z_pid-target_vel_z_pid_last;
                                if(vel_delt_z > DELT_RANGE_VEL){
                                    target_vel_z_pid = target_vel_z_pid_last + DELT_RANGE_VEL;
                                }else if (vel_delt_z < -DELT_RANGE_VEL)
                                {
                                    target_vel_z_pid = target_vel_z_pid_last - DELT_RANGE_VEL;
                                }

                                target_vel_x_pid_last = target_vel_x_pid;
                                target_vel_y_pid_last = target_vel_y_pid;
                                target_vel_z_pid_last = target_vel_z_pid;

                                double target_acc_x_pid = vel_control_a_x.calculate(localvelocity.x,target_vel_x_pid);                         //PID的系数设置为0.1~10之间，加速度限制在-5.685~5.685之间，取上下限-6~6之间
                                double target_acc_y_pid = vel_control_a_y.calculate(localvelocity.y,target_vel_y_pid);                         //预设最大P值10时，速度相差5m/s的时候，最大值输出。
                                double target_acc_z_pid = vel_control_a_z.calculate(localvelocity.z,target_vel_z_pid);  

                                #define DELT_RANGE (GRAVITY_ACC * tan(5.0*M_PI/180.0f))   //约等于1.728    0.85

                                double acc_delt_x = target_acc_x_pid-target_acc_x_pid_last;
                                if(acc_delt_x > DELT_RANGE){
                                    target_acc_x_pid = target_acc_x_pid_last + DELT_RANGE;
                                }else if (acc_delt_x < -DELT_RANGE)
                                {
                                    target_acc_x_pid = target_acc_x_pid_last - DELT_RANGE;
                                }

                                double acc_delt_y = target_acc_y_pid-target_acc_y_pid_last;
                                if(acc_delt_y > DELT_RANGE){
                                    target_acc_y_pid = target_acc_y_pid_last + DELT_RANGE;
                                }else if (acc_delt_y < -DELT_RANGE)
                                {
                                    target_acc_y_pid = target_acc_y_pid_last - DELT_RANGE;
                                }                                
                                
                                double acc_delt_z = target_acc_z_pid-target_acc_z_pid_last;
                                if(acc_delt_z > DELT_RANGE){
                                    target_acc_z_pid = target_acc_z_pid_last + DELT_RANGE;
                                }else if (acc_delt_z < -DELT_RANGE)
                                {
                                    target_acc_z_pid = target_acc_z_pid_last - DELT_RANGE;
                                }

                                target_acc_x_pid_last = target_acc_x_pid;
                                target_acc_y_pid_last = target_acc_y_pid;
                                target_acc_z_pid_last = target_acc_z_pid;

                                //转换到机体坐标系下，将目标加速度转换成飞机的目标姿态
                                Eigen::Quaterniond q(localpose.w, localpose.x, localpose.y, localpose.z);                                   //当前姿态四元素，转换为欧拉角
                                double yaw,roll,pitch;
                                toEulerAngle(q,roll,pitch,yaw);     

                                //转换到机体坐标系下
                                double target_acc_x_pid_in_body = target_acc_x_pid * cos(yaw) + target_acc_y_pid * sin(yaw);
                                double target_acc_y_pid_in_body = -target_acc_x_pid * sin(yaw) + target_acc_y_pid * cos(yaw);

                                //换算成角度
                                double target_pitch_pid =  atan2(target_acc_x_pid_in_body,GRAVITY_ACC);                                         //加速度转换成姿态
                                double target_roll_pid  = -atan2(target_acc_y_pid_in_body,GRAVITY_ACC);                                         //根据右手法则，roll的目标差值需要反向
                                #define UAV_WEIGHT 3.340f
                                double target_height_pid = target_acc_z_pid * UAV_WEIGHT / (cos(target_pitch_pid) * cos(target_roll_pid)) *0.2f;       //高度不做控制

                                
                                //获取当前的目标点需要的航向角
                                double target_yaw = pathpoint_sub_test[targetPointNum_test].a;


                                //欧拉角转换成四元素 发送出去，航向角不变，
                                Eigen::AngleAxisd rollAngle_(Eigen::AngleAxisd(target_roll_pid,Eigen::Vector3d::UnitX()));      //pid计算的目标航向
                                Eigen::AngleAxisd pitchAngle_(Eigen::AngleAxisd(target_pitch_pid,Eigen::Vector3d::UnitY()));
                                Eigen::AngleAxisd yawAngle_(Eigen::AngleAxisd(target_yaw,Eigen::Vector3d::UnitZ()));        //进入模式的时候，设定的目标航向
                                
                                Eigen::Quaterniond quaternion_;
                                quaternion_=yawAngle_*pitchAngle_*rollAngle_;

                                target_pose.orientation.x = quaternion_.coeffs()[0];                 //角度赋值
                                target_pose.orientation.y = quaternion_.coeffs()[1];
                                target_pose.orientation.z = quaternion_.coeffs()[2];
                                target_pose.orientation.w = quaternion_.coeffs()[3];        
                                target_pose.thrust = 0.43f + target_height_pid;                     //设置油门
                                if(target_pose.thrust > 1.0f){
                                    target_pose.thrust = 1.0f;
                                }else if(target_pose.thrust < 0.0f){
                                    target_pose.thrust = 0.0f;
                                }

                                target_pose.type_mask = 1+2+4;
                                target_pose.header.stamp = node->now();

                                pose_pub->publish(target_pose);    //ZZXZZX                   

                                double vel_xy = sqrt(pow(localvelocity.x,2)+pow(localvelocity.y,2)+pow(localvelocity.z,2));

                                double vel_xy_main = sqrt(pow(target_velocity_main.x,2)+pow(target_velocity_main.y,2)+pow(target_velocity_main.z,2));
                                double vel_xy_heng = sqrt(pow(target_vel_x_h_pid,2)+pow(target_vel_y_h_pid,2)+pow(target_vel_z_h_pid,2));

                                double distance = calculate_distance(pathpoint_sub_test[targetPointNum_test].x,pathpoint_sub_test[targetPointNum_test].y,pathpoint_sub_test[targetPointNum_test].z); 

                                printf("line_fly num:%d,D:%lf  vel:%lf main:%lf heng:%lf, lx:%lf ly:%lf lz:%lf,lvx:%lf lvy:%lf lvz:%lf,tlx:%lf tly:%lf tlz:%lf,tvhx:%lf tvhy:%lf tvhz:%lf, tvx:%lf tvy:%lf tvz:%lf,tvx_b:%lf tvy_b:%lf tvz_b:%lf,tax:%lf tay:%lf taz:%lf,tbax:%lf tbay:%lf thu:%lf,tpit:%lf troll:%lf tyaw:%lf,\r\n",
                                min_targetPointNum,distance,vel_xy,vel_xy_main,vel_xy_heng,
                                localposition.x,localposition.y,localposition.z,
                                localvelocity.x,localvelocity.y,localvelocity.z,
                                pathpoint_sub_test[targetPointNum_test].x,pathpoint_sub_test[targetPointNum_test].y,pathpoint_sub_test[targetPointNum_test].z,
                                target_vel_x_h_pid,target_vel_y_h_pid,target_vel_z_h_pid,
                                target_vel_x_pid,target_vel_y_pid,target_vel_z_pid,
                                target_vel_x_h_pid_BUTTER,target_vel_y_h_pid_BUTTER,target_vel_z_h_pid_BUTTER,
                                target_acc_x_pid,target_acc_y_pid,target_acc_z_pid,
                                target_acc_x_pid_in_body,target_acc_y_pid_in_body,0.45f + target_height_pid,
                                target_pitch_pid,target_roll_pid,target_yaw
                                );


                                //判断是否已经飞到目标点了
                                if(distance < 0.2f){
                                    targetPointNum_test++;
                                    isFillTrajectory = true;
                                }

                                isObstacleStop = false;
                            }else{
                                mode_int = false;
                                controlMode = MODE_HOVER;
                            }
                        }

                        if(obstacle_distance < 0.5f || isObstacleStop == true){
                            printf("1.5obstacle_distance:%lf",obstacle_distance);
                            // 1. 获取当前悬停点以及航向
                            if(!isObstacleStop){
                                //首先确定悬停的位置,从没有障碍物的状态下，转到有障碍物的情况
                                avoid_target_position.x = localposition.x;          //带坐标转换，从笛卡尔转到东北天
                                avoid_target_position.y = localposition.y;
                                avoid_target_position.z = localposition.z;
                                
                                avoid_target_pose.x = localpose.x;
                                avoid_target_pose.y = localpose.y;                  //确定航向
                                avoid_target_pose.z = localpose.z;
                                avoid_target_pose.w = localpose.w;                        
                            }

                            // 2. 控制飞行停止
                                double target_vel_x_pid = position_control_x.calculate(localposition.x,avoid_target_position.x);               //PID的系数设置为0.1~10 之间，速度现在-1~1之间
                                double target_vel_y_pid = position_control_y.calculate(localposition.y,avoid_target_position.y);               //预设最大P值10时，距离相差1m的时候，最大值输出。
                                double target_vel_z_pid = position_control_z.calculate(localposition.z,avoid_target_position.z);               //即，10*1*0.1 = 1m/s
                
                                double target_acc_x_pid = vel_control_x.calculate(localvelocity.x,target_vel_x_pid) ;                         //PID的系数设置为0.1~10之间，加速度限制在-5.685~5.685之间，取上下限-6~6之间
                                double target_acc_y_pid = vel_control_y.calculate(localvelocity.y,target_vel_y_pid) ;                         //预设最大P值10时，速度相差5m/s的时候，最大值输出。
                                double target_acc_z_pid = vel_control_z.calculate(localvelocity.z,target_vel_z_pid) ;                         //即，10*5 * 0.12 = 6  
        
        
                                //以上代码target_vel_x_pid，target_acc_x_pid都为与local的坐标系下，即东北天坐标系
                
                                Eigen::Quaterniond q(localpose.w, localpose.x, localpose.y, localpose.z);                                   //当前姿态四元素，转换为欧拉角
                                double yaw,roll,pitch;
                                toEulerAngle(q,roll,pitch,yaw);     
                
                                //转换到机体坐标系下
                                double target_acc_x_pid_in_body = target_acc_x_pid * cos(yaw) + target_acc_y_pid * sin(yaw);
                                double target_acc_y_pid_in_body = -target_acc_x_pid * sin(yaw) + target_acc_y_pid * cos(yaw);
                
                                //换算成角度
                                double target_pitch_pid =  atan2(target_acc_x_pid_in_body,GRAVITY_ACC);                                         //加速度转换成姿态
                                double target_roll_pid  = -atan2(target_acc_y_pid_in_body,GRAVITY_ACC);                                         //根据右手法则，roll的目标差值需要反向
                                #define UAV_WEIGHT 3.340f
                                double target_height_pid = target_acc_z_pid * UAV_WEIGHT / (cos(target_pitch_pid) * cos(target_roll_pid)) *0.2f;                                         //高度不做控制
                
                                Eigen::Quaterniond q_target(avoid_target_pose.w, avoid_target_pose.x, avoid_target_pose.y, avoid_target_pose.z);
                                EulerTypedef aviod_target_euler;
                                toEulerAngle(q_target,aviod_target_euler.roll,aviod_target_euler.pitch,aviod_target_euler.yaw);

                
                                //欧拉角转换成四元素 发送出去，航向角不变，
                                Eigen::AngleAxisd rollAngle_(Eigen::AngleAxisd(target_roll_pid,Eigen::Vector3d::UnitX()));      //pid计算的目标航向
                                Eigen::AngleAxisd pitchAngle_(Eigen::AngleAxisd(target_pitch_pid,Eigen::Vector3d::UnitY()));
                                Eigen::AngleAxisd yawAngle_(Eigen::AngleAxisd(aviod_target_euler.yaw,Eigen::Vector3d::UnitZ()));        //进入模式的时候，设定的目标航向
                                
                                Eigen::Quaterniond quaternion_;                                     
                                quaternion_=yawAngle_*pitchAngle_*rollAngle_;
                
                                target_pose.orientation.x = quaternion_.coeffs()[0];                 //角度赋值
                                target_pose.orientation.y = quaternion_.coeffs()[1];
                                target_pose.orientation.z = quaternion_.coeffs()[2];
                                target_pose.orientation.w = quaternion_.coeffs()[3];        
                                target_pose.thrust = 0.43f + target_height_pid;                     //设置油门
                                if(target_pose.thrust > 1.0f){
                                    target_pose.thrust = 1.0f;
                                }else if(target_pose.thrust < 0.0f){
                                    target_pose.thrust = 0.0f;
                                }
                
                                target_pose.type_mask = 1+2+4;
                                target_pose.header.stamp = node->now();
                
                                pose_pub->publish(target_pose);                                            //发送消息


                            // 3 轨迹变量
                            isObstacleStop = true;

                        }
                    }

                           
                }             
                else{
                    if(targetPointNum_test == 0){                                //没有收到轨迹
                        position_pub->publish(target_position);             //改位置在模式初始化的时候，更新的位置
                        printf("hover no point \r\n");
                    }else if(targetPointNum_test == pathpoint_sub_test.size()){       //已经到轨迹终点了


                        target_position.header.stamp = node->now();
                        target_position.header.frame_id = "map";
                        target_position.pose.position.x = pathpoint_sub_test[targetPointNum_test-1].x;
                        target_position.pose.position.y = pathpoint_sub_test[targetPointNum_test-1].y;
                        target_position.pose.position.z = pathpoint_sub_test[targetPointNum_test-1].z;


                        double target_yaw = pathpoint_sub_test[targetPointNum_test-1].a;

                        //欧拉角转换成四元素 发送出去，航向角不变，
                        Eigen::AngleAxisd rollAngle_(Eigen::AngleAxisd(0,Eigen::Vector3d::UnitX()));      //pid计算的目标航向
                        Eigen::AngleAxisd pitchAngle_(Eigen::AngleAxisd(0,Eigen::Vector3d::UnitY()));
                        Eigen::AngleAxisd yawAngle_(Eigen::AngleAxisd(target_yaw,Eigen::Vector3d::UnitZ()));        //进入模式的时候，设定的目标航向
                        
                        Eigen::Quaterniond quaternion_;                                     
                        quaternion_=yawAngle_*pitchAngle_*rollAngle_;

                        target_position.pose.orientation.x = quaternion_.coeffs()[0];  
                        target_position.pose.orientation.y = quaternion_.coeffs()[1]; 
                        target_position.pose.orientation.z = quaternion_.coeffs()[2];  
                        target_position.pose.orientation.w = quaternion_.coeffs()[3];  

                        position_pub->publish(target_position);              //该位置在最后一个点进圈后，更新的位置                
                        printf("finish\r\n");
                    }else{
                        mode_int = false;
                        controlMode = MODE_HOVER;                           //其他情况，切换到悬停模式
                    }
                }
            }

        }
        else if (controlMode == MODE_HOVER){
            if(!mode_int){

                target_position.header.stamp = node->now();
                target_position.header.frame_id = "map";
                target_position.pose.position.x = localposition.x;
                target_position.pose.position.y = localposition.y;
                target_position.pose.position.z = localposition.z;

                target_position.pose.orientation.x = localpose.x ;
                target_position.pose.orientation.y = localpose.y ;
                target_position.pose.orientation.z = localpose.z ;
                target_position.pose.orientation.w = localpose.w ;

                printf("hoverMode x:%f y:%f z:%f\r\n",target_position.pose.position.x,target_position.pose.position.y,target_position.pose.position.z);
                position_pub->publish(target_position);
                mode_int = true;
            }else{
                target_position.header.stamp = node->now();
                position_pub->publish(target_position);
            }
        }
        else if (controlMode == MODE_HOVER_CUSTOM){
            if(!mode_int){
                target_position.header.stamp = node->now();
                target_position.header.frame_id = "map";
                target_position.pose.position.x = localposition.x;
                target_position.pose.position.y = localposition.y;
                target_position.pose.position.z = localposition.z;

                target_position.pose.orientation.x = localpose.x ;
                target_position.pose.orientation.y = localpose.y ;
                target_position.pose.orientation.z = localpose.z ;
                target_position.pose.orientation.w = localpose.w ;

                target_vel_x_pid_last = 0.0f;  //速度 初始值
                target_vel_y_pid_last = 0.0f;
                target_vel_z_pid_last = 0.0f;

                target_acc_x_pid_last = 0.0f;  //加速度 初始值
                target_acc_y_pid_last = 0.0f;
                target_acc_z_pid_last = 0.0f;

                printf("hoverMode_CUSTOM x:%f y:%f z:%f\r\n",target_position.pose.position.x,target_position.pose.position.y,target_position.pose.position.z);
                position_pub->publish(target_position);
                mode_int = true;
            }else{
                double target_vel_x_pid = target_position.pose.position.x - localposition.x;
                double target_vel_y_pid = target_position.pose.position.y - localposition.y;
                double target_vel_z_pid = target_position.pose.position.z - localposition.z;                       

                double target_acc_x_pid = 4*(target_vel_x_pid - localvelocity.x);                        //PID的系数设置为0.1~10之间，加速度限制在-5.685~5.685之间，取上下限-6~6之间
                double target_acc_y_pid = 4*(target_vel_y_pid - localvelocity.y);                         //预设最大P值10时，速度相差5m/s的时候，最大值输出。
                double target_acc_z_pid = 5*(target_vel_z_pid - localvelocity.z);  

                // DOB 
                double L = 1;
                static double z_x = 0.0, z_y = 0.0, z_z = 0.0;
                static double d_x_hat = 0.0, d_y_hat = 0.0, d_z_hat = 0.0;
                static double last_time = node->now().seconds();
                static double u_x = 0, u_y = 0, u_z = 0;

                u_x = target_acc_x_pid;
                u_y = target_acc_y_pid;
                u_z = target_acc_z_pid;

                double current_time = node->now().seconds();
                double dt =  current_time - last_time;
                if(dt > 0.005) {
                    z_x = z_x/(L*dt+1) - L*L*dt*localvelocity.x/(L*dt+1) - L*target_acc_x_pid*dt/(L*dt+1);
                    z_y = z_y/(L*dt+1) - L*L*dt*localvelocity.y/(L*dt+1) - L*target_acc_y_pid*dt/(L*dt+1);
                    // z_z = z_z/(L*dt+1) - L*L*dt*localvelocity.z/(L*dt+1) - L*target_acc_z_pid*dt/(L*dt+1);

                    d_x_hat = z_x + L * localvelocity.x;
                    d_y_hat = z_y + L * localvelocity.y;
                    // d_z_hat = z_z + L * localvelocity.z;

                    u_x = u_x - d_x_hat;
                    u_y = u_y - d_y_hat;
                    // u_z = u_z - d_z_hat;

                    last_time = current_time;
                }
                #define DELT_RANGE (GRAVITY_ACC * tan(5.0*M_PI/180.0f))   //约等于1.728    0.85

                double acc_delt_x = u_x - target_acc_x_pid_last;
                if(acc_delt_x > DELT_RANGE){
                    u_x = target_acc_x_pid_last + DELT_RANGE;
                }else if (acc_delt_x < -DELT_RANGE)
                {
                    u_x = target_acc_x_pid_last - DELT_RANGE;
                }

                double acc_delt_y = u_y - target_acc_y_pid_last;
                if(acc_delt_y > DELT_RANGE){
                    u_y = target_acc_y_pid_last + DELT_RANGE;
                }else if (acc_delt_y < -DELT_RANGE)
                {
                    u_y = target_acc_y_pid_last - DELT_RANGE;
                }                                
                
                double acc_delt_z = u_z - target_acc_z_pid_last;
                if(acc_delt_z > DELT_RANGE){
                    u_z = target_acc_z_pid_last + DELT_RANGE;
                }else if (acc_delt_z < -DELT_RANGE)
                {
                    u_z = target_acc_z_pid_last - DELT_RANGE;
                }

                target_acc_x_pid_last = u_x;
                target_acc_y_pid_last = u_y;
                target_acc_z_pid_last = u_z;

                const double eps = 1e-6;
                Eigen::Vector3d accel_cmd(u_x, u_y, u_z);
                double acc_norm = accel_cmd.norm();
                #define UAV_WEIGHT 2.0f
                double out_thrust = UAV_WEIGHT * acc_norm;

                //转换到机体坐标系下，将目标加速度转换成飞机的目标姿态
                Eigen::Quaterniond q_des(target_position.pose.orientation.w, target_position.pose.orientation.x, target_position.pose.orientation.y, target_position.pose.orientation.z);                                   //当前姿态四元素，转换为欧拉角
                double yaw_des,roll_des,pitch_des;
                toEulerAngle(q_des,roll_des,pitch_des,yaw_des);

                Eigen::Quaterniond out_quat;
                // 异常处理：期望加速度模长过小 -> 保持水平姿态，仅使用偏航角
                if (acc_norm < eps) {
                    Eigen::AngleAxisd yaw_rot(yaw_des, Eigen::Vector3d::UnitZ());
                    out_quat = Eigen::Quaterniond(yaw_rot);
                    out_quat.normalize();
                } else{
                    // 期望推力方向单位向量 (机体Z轴在世界系中的期望方向)
                    Eigen::Vector3d b_c = accel_cmd.normalized();

                    // 根据期望偏航角构造临时水平向量
                    Eigen::Vector3d t(std::cos(yaw_des), std::sin(yaw_des), 0.0);
                    // 施密特正交化得到 X_c (水平且垂直于 b_c)
                    double dot = t.dot(b_c);
                    Eigen::Vector3d X_c = t - dot * b_c;
                    if (X_c.norm() < eps) {
                        // 退化情况：b_c 平行于 Z 轴，取水平 X 轴
                        X_c = Eigen::Vector3d(1.0, 0.0, 0.0);
                    }
                    X_c.normalize();

                    // Y_c = b_c × X_c
                    Eigen::Vector3d Y_c = b_c.cross(X_c);
                    Y_c.normalize();

                    // 构造旋转矩阵 (三列分别为 X_c, Y_c, b_c)
                    Eigen::Matrix3d R;
                    R.col(0) = X_c;
                    R.col(1) = Y_c;
                    R.col(2) = b_c;

                    // 旋转矩阵转四元数
                    out_quat = Eigen::Quaterniond(R);
                    out_quat.normalize();
                }

                //换算成角度
                Eigen::Quaterniond q(localpose.w, localpose.x, localpose.y, localpose.z);                                   //当前姿态四元素，转换为欧拉角
                double yaw,roll,pitch;
                toEulerAngle(q,roll,pitch,yaw);
                double target_height_pid = u_z * UAV_WEIGHT / (cos(pitch) * cos(roll));       //高度不做控制

                target_pose.orientation.x = out_quat.coeffs()[0];                 //角度赋值
                target_pose.orientation.y = out_quat.coeffs()[1];
                target_pose.orientation.z = out_quat.coeffs()[2];
                target_pose.orientation.w = out_quat.coeffs()[3];        
                target_pose.thrust = 0.24f + target_height_pid;                     //设置油门
                if(target_pose.thrust > 1.0f){
                    target_pose.thrust = 1.0f;
                }else if(target_pose.thrust < 0.0f){
                    target_pose.thrust = 0.0f;
                }
                target_pose.type_mask = 1+2+4;
                target_pose.header.stamp = node->now();
                pose_pub->publish(target_pose);                         
            }
        }
    
        rclcpp::spin_some(node);
        rate_main.sleep();
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
