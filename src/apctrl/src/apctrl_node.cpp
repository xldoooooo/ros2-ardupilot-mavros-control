#include "apctrl_node.h"

#define TEST_OPEN 0 // 0: 不测试，1: 测试

// 信号处理函数，用于优雅地处理Ctrl+C等终止信号
void mySigintHandler(int sig)
{
    RCLCPP_INFO(rclcpp::get_logger("apctrl_node"), "[apctrl_node] exit..."); // 打印退出信息
    rclcpp::shutdown();               // 关闭ROS 2节点
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    /*产生一个apctrl的节点*/
    auto node = std::make_shared<rclcpp::Node>("apctrl_node");
    // 设置日志级别为DEBUG
    rcutils_logging_set_logger_level(node->get_logger().get_name(), RCUTILS_LOG_SEVERITY_DEBUG);
    
    // 注册信号处理函数
    signal(SIGINT, mySigintHandler);
    rclcpp::sleep_for(std::chrono::seconds(1));  // 等待1秒，确保其他节点已经启动

    // 加载参数
    Parameter_t param;
    param.config_from_ros_handle(node);

    // 初始化控制器和状态机(最后再解决)
    LinearControl controller(param, node);  // 创建线性控制器实例
    APCtrlFSM fsm(param, controller, node);  // 创建状态机实例
    #if TEST_OPEN
        // 创建测试类实例
        Test test(node);

        // 1. 订阅飞控状态信息
        auto state_sub = node->create_subscription<mavros_msgs::msg::State>(
            "/mavros/state", 10,
            std::bind(&Test::stateCallback, &test, std::placeholders::_1));

        // 2. 订阅扩展状态信息
        auto extended_state_sub = node->create_subscription<mavros_msgs::msg::ExtendedState>(
            "/mavros/extended_state", 10,
            std::bind(&Test::extendedStateCallback, &test, std::placeholders::_1));

        // 3. 订阅里程计信息
        auto odom_sub = node->create_subscription<nav_msgs::msg::Odometry>(
            "odom", rclcpp::QoS(100).reliable().best_effort(),
            std::bind(&Test::odomCallback, &test, std::placeholders::_1));

        //4. 订阅位置指令
        auto cmd_sub = node->create_subscription<quadrotor_msgs::msg::PositionCommand>(
            "cmd", rclcpp::QoS(100).reliable().best_effort(),
            std::bind(&Test::cmdCallback, &test, std::placeholders::_1));

        // 5. 订阅IMU数据
        auto imu_sub = node->create_subscription<sensor_msgs::msg::Imu>(
            "/mavros/imu/data", rclcpp::QoS(100).reliable().best_effort(),
            std::bind(&Test::imuCallback, &test, std::placeholders::_1));

        // 6. 订阅遥控器数据（如果启用）
        rclcpp::Subscription<mavros_msgs::msg::RCIn>::SharedPtr rc_sub;
        if (!param.takeoff_land.no_RC)
        {
            rc_sub = node->create_subscription<mavros_msgs::msg::RCIn>(
                "/mavros/rc/in", 10,
                std::bind(&Test::rcCallback, &test, std::placeholders::_1));
        }

        // 7. 订阅电池状态
        auto bat_sub = node->create_subscription<sensor_msgs::msg::BatteryState>(
            "/mavros/battery", rclcpp::QoS(100).reliable().best_effort(),
            std::bind(&Test::batteryCallback, &test, std::placeholders::_1));

        // 8. 订阅起飞降落命令
        auto takeoff_land_sub = node->create_subscription<quadrotor_msgs::msg::TakeoffLand>(
            "takeoff_land", rclcpp::QoS(100).reliable().best_effort(),
            std::bind(&Test::takeoffLandCallback, &test, std::placeholders::_1));
    #elif TEST_OPEN == 0
        // 1. 订阅飞控状态信息（仅接受状态）
        auto state_sub = node->create_subscription<mavros_msgs::msg::State>(
            "/mavros/state", 10,
            std::bind(&State_Data_t::feed, &fsm.state_data, std::placeholders::_1));

        // 2. 订阅扩展状态信息（仅接受状态）
        auto extended_state_sub = node->create_subscription<mavros_msgs::msg::ExtendedState>(
            "/mavros/extended_state", 10,
            std::bind(&ExtendedState_Data_t::feed, &fsm.extended_state_data, std::placeholders::_1));

        // 3. 订阅里程计信息（接受里程计数据后提取并检查频率）
        auto odom_sub = node->create_subscription<nav_msgs::msg::Odometry>(
            "odom", rclcpp::QoS(100).reliable().best_effort(),
            std::bind(&Odom_Data_t::feed, &fsm.odom_data, std::placeholders::_1));

        //4. 订阅位置指令
        auto cmd_sub = node->create_subscription<quadrotor_msgs::msg::PositionCommand>(
            "cmd", rclcpp::QoS(100).reliable().best_effort(),
            std::bind(&Command_Data_t::feed, &fsm.cmd_data, std::placeholders::_1));

        // 5. 订阅IMU数据
        auto imu_sub = node->create_subscription<sensor_msgs::msg::Imu>(
            "/mavros/imu/data", rclcpp::QoS(100).reliable().best_effort(),
            std::bind(&Imu_Data_t::feed, &fsm.imu_data, std::placeholders::_1));

        // 6. 订阅遥控器数据（如果启用）
        rclcpp::Subscription<mavros_msgs::msg::RCIn>::SharedPtr rc_sub;
        if (!param.takeoff_land.no_RC)
        {
            rc_sub = node->create_subscription<mavros_msgs::msg::RCIn>(
                "/mavros/rc/in", 10,
                std::bind(&RC_Data_t::feed, &fsm.rc_data, std::placeholders::_1));
        }

        // 7. 订阅电池状态
        auto bat_sub = node->create_subscription<sensor_msgs::msg::BatteryState>(
            "/mavros/battery", rclcpp::QoS(100).reliable().best_effort(),
            std::bind(&Battery_Data_t::feed, &fsm.bat_data, std::placeholders::_1));

        // 8. 订阅起飞降落命令
        auto takeoff_land_sub = node->create_subscription<quadrotor_msgs::msg::TakeoffLand>(
            "takeoff_land", rclcpp::QoS(100).reliable().best_effort(),
            std::bind(&Takeoff_Land_Data_t::feed, &fsm.takeoff_land_data, std::placeholders::_1));
    #endif // TEST_OPEN

    // 创建 best effort QoS profile
    auto best_effort_qos = rclcpp::QoS(10).best_effort().durability_volatile();
    
    // 创建发布器
    fsm.ctrl_FCU_pub = node->create_publisher<mavros_msgs::msg::AttitudeTarget>(
        "/mavros/setpoint_raw/attitude", 
        best_effort_qos);  // 使用 best effort QoS
    fsm.traj_start_trigger_pub = node->create_publisher<geometry_msgs::msg::PoseStamped>(
        "/traj_start_trigger", 10);//发布轨迹启动信号
    fsm.debug_pub = node->create_publisher<quadrotor_msgs::msg::ApctrlDebug>(
        "/debugApctrl", 10);//发布调试信息

    // 创建服务客户端
    fsm.set_FCU_mode_client = node->create_client<mavros_msgs::srv::SetMode>(
        "/mavros/set_mode");//设置飞行模式
    fsm.arming_client = node->create_client<mavros_msgs::srv::CommandBool>(
        "/mavros/cmd/arming");//设置飞机状态
    fsm.reboot_FCU_client = node->create_client<mavros_msgs::srv::CommandLong>(
        "/mavros/cmd/command");//重启飞控

    // 检查遥控器状态
    if (param.takeoff_land.no_RC)
    {
        //遥控器已禁用，请注意安全！
        RCLCPP_WARN(node->get_logger(), "[APCTRL] Remote controller disabled, be careful!");
    }
    else
    {
        RCLCPP_INFO(node->get_logger(), "[APCTRL] Waiting for RC");//等待遥控器连接
        while (rclcpp::ok())
        {
            rclcpp::spin_some(node);
            rclcpp::Time current_time = node->now();
            if (fsm.rc_is_received(current_time))
            {
                RCLCPP_INFO(node->get_logger(), "[APCTRL] RC received.");//连接成功
                break;
            }
            rclcpp::sleep_for(std::chrono::milliseconds(100));//等待100ms
        }
    }

    // 等待连接到AP飞控
    int trials = 0;
    while (rclcpp::ok() && !fsm.state_data.current_state.connected)
    {
        rclcpp::spin_some(node);
        rclcpp::sleep_for(std::chrono::seconds(1));
        if (trials++ > 5)//等待时间不能超过5s
            RCLCPP_ERROR(node->get_logger(), "Unable to connect to AP!!!");
    }


    // 主循环
    //**需要注意的是，如果循环中的处理时间超过了周期时间（1/频率）
    //**系统将无法达到设定的目标频率。因此在设置频率时要考虑系统的实际处理能力。
    rclcpp::Rate rate(param.ctrl_freq_max);  // 设置循环频率
    while (rclcpp::ok())
    {
        rate.sleep();
        rclcpp::spin_some(node);  // 处理回调
        fsm.process();         // 运行状态机

        #if TEST_OPEN
            test.displayMessages();    // 调用显示函数
        #endif // TEST_OPEN
    }

    return 0;
}
