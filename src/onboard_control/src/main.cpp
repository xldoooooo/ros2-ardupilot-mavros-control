/**
 * @file main.cpp
 * @brief 启动多线程机载控制执行器，使服务响应不会阻塞固定周期控制。
 */
#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "onboard_control/onboard_control_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<onboard_control::OnboardControlNode>();
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
  executor.add_node(node);
  executor.spin();
  executor.remove_node(node);
  rclcpp::shutdown();
  return 0;
}
