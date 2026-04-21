#include "apctrl_node.h"

// 信号处理函数
void mySigintHandler(int /*sig*/)
{
    RCLCPP_INFO(rclcpp::get_logger("apctrl_node"), "[apctrl_node] exit...");
    rclcpp::shutdown();
}

// 创建自定义订阅函数（选用true为best_effort,false为reliable，即每条消息都传送到位）
template<typename MsgType, typename CallbackType>
auto create_subscription(
    rclcpp::Node::SharedPtr node,
    const std::string& topic,
    int qos_history,
    CallbackType callback,
    bool best_effort = false)
{
    auto qos = rclcpp::QoS(qos_history);
    if (best_effort) {
        qos.best_effort();
    } else {
        qos.reliable();
    }
    return node->create_subscription<MsgType>(topic, qos, callback);
}

// 订阅配置结构体
struct SubscriptionConfig {
    std::string topic;
    int qos_history;
    bool best_effort;
    
    SubscriptionConfig(const std::string& t, int q = 10, bool be = false) 
        : topic(t), qos_history(q), best_effort(be) {}
};

// 订阅初始化函数
void initialize_subscriptions(
    rclcpp::Node::SharedPtr node,
    APCtrlFSM& fsm,
    const Parameter_t& param)
{
    // 创建所有订阅
    create_subscription<mavros_msgs::msg::State>(
        node, "/mavros/state", 10,
        std::bind(&State_Data_t::feed, &fsm.state_data, std::placeholders::_1));
    
    create_subscription<mavros_msgs::msg::ExtendedState>(
        node, "/mavros/extended_state", 10,
        std::bind(&ExtendedState_Data_t::feed, &fsm.extended_state_data, std::placeholders::_1));
    
    create_subscription<nav_msgs::msg::Odometry>(
        node, "odom", 100,
        std::bind(&Odom_Data_t::feed, &fsm.odom_data, std::placeholders::_1), true);
    
    create_subscription<quadrotor_msgs::msg::PositionCommand>(
        node, "cmd", 100,
        std::bind(&Command_Data_t::feed, &fsm.cmd_data, std::placeholders::_1), true);
    
    create_subscription<sensor_msgs::msg::Imu>(
        node, "/mavros/imu/data", 100,
        std::bind(&Imu_Data_t::feed, &fsm.imu_data, std::placeholders::_1), true);
    
    create_subscription<sensor_msgs::msg::BatteryState>(
        node, "/mavros/battery", 100,
        std::bind(&Battery_Data_t::feed, &fsm.bat_data, std::placeholders::_1), true);
    
    create_subscription<quadrotor_msgs::msg::TakeoffLand>(
        node, "takeoff_land", 100,
        std::bind(&Takeoff_Land_Data_t::feed, &fsm.takeoff_land_data, std::placeholders::_1), true);
    
    // RC订阅（条件性）
    if (!param.takeoff_land.no_RC) {
        create_subscription<mavros_msgs::msg::RCIn>(
            node, "/mavros/rc/in", 10,
            std::bind(&RC_Data_t::feed, &fsm.rc_data, std::placeholders::_1));
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("apctrl_node");
    // 设置日志级别并存储返回值以消除警告
    rcutils_ret_t logging_result = rcutils_logging_set_logger_level(node->get_logger().get_name(), RCUTILS_LOG_SEVERITY_DEBUG);
    (void)logging_result; // 显式忽略返回值
    
    signal(SIGINT, mySigintHandler);
    rclcpp::sleep_for(std::chrono::seconds(1));

    // 加载参数
    Parameter_t param;
    param.config_from_ros_handle(node);

    // 初始化控制器和状态机
    LinearControl controller(param, node);
    APCtrlFSM fsm(param, controller, node);
    
    // 初始化订阅
    initialize_subscriptions(node, fsm, param);

    // 创建发布器
    auto best_effort_qos = rclcpp::QoS(10).best_effort().durability_volatile();
    fsm.ctrl_FCU_pub = node->create_publisher<mavros_msgs::msg::AttitudeTarget>(
        "/mavros/setpoint_raw/attitude", best_effort_qos);
    fsm.traj_start_trigger_pub = node->create_publisher<geometry_msgs::msg::PoseStamped>(
        "/traj_start_trigger", 10);
    fsm.debug_pub = node->create_publisher<quadrotor_msgs::msg::ApctrlDebug>(
        "/debugApctrl", 10);

    // 创建服务客户端
    fsm.set_FCU_mode_client = node->create_client<mavros_msgs::srv::SetMode>("/mavros/set_mode");
    fsm.arming_client = node->create_client<mavros_msgs::srv::CommandBool>("/mavros/cmd/arming");
    fsm.reboot_FCU_client = node->create_client<mavros_msgs::srv::CommandLong>("/mavros/cmd/command");

    // 等待遥控器连接
    if (!param.takeoff_land.no_RC) {
        RCLCPP_INFO(node->get_logger(), "[APCTRL] Waiting for RC");
        while (rclcpp::ok()) {
            rclcpp::spin_some(node);
            if (fsm.rc_is_received(node->now())) {
                RCLCPP_INFO(node->get_logger(), "[APCTRL] RC received.");
                break;
            }
            rclcpp::sleep_for(std::chrono::milliseconds(100));
        }
    } else {
        RCLCPP_WARN(node->get_logger(), "[APCTRL] Remote controller disabled, be careful!");
    }

    // 等待连接到飞控
    int trials = 0;
    while (rclcpp::ok() && !fsm.state_data.current_state.connected) {
        rclcpp::spin_some(node);
        rclcpp::sleep_for(std::chrono::seconds(1));
        if (trials++ > 5)
            RCLCPP_ERROR(node->get_logger(), "Unable to connect to AP!!!");
    }

    // 主循环
    rclcpp::Rate rate(param.ctrl_freq_max);
    while (rclcpp::ok()) {
        rate.sleep();
        rclcpp::spin_some(node);
        fsm.process();
    }

    return 0;
}
