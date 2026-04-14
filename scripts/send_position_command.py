#!/usr/bin/env python3
"""
发送位置命令到无人机控制器的脚本
这个脚本允许用户输入目标位置点，让无人机移动到指定位置
"""

import rclpy
from rclpy.node import Node
import time
import sys

from quadrotor_msgs.msg import PositionCommand
from geometry_msgs.msg import Vector3
from std_msgs.msg import Header


class PositionCommandSender(Node):
    def __init__(self):
        super().__init__('position_command_sender')
        
        # 创建发布器
        self.publisher = self.create_publisher(
            PositionCommand,
            '/position_cmd',
            10
        )
        
        self.get_logger().info('位置命令发送器已启动')
        self.get_logger().info('发布到主题: /position_cmd')
        
    def send_position_command(self, x, y, z, yaw=0.0):
        """发送位置命令
        
        参数:
            x: X轴位置 (米)
            y: Y轴位置 (米)
            z: Z轴位置 (高度，米)
            yaw: 偏航角 (弧度，默认0)
        """
        msg = PositionCommand()
        
        # 设置header
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        
        # 设置位置
        msg.pos = Vector3()
        msg.pos.x = float(x)
        msg.pos.y = float(y)
        msg.pos.z = float(z)
        
        # 设置速度（设为0，让控制器计算）
        msg.vel = Vector3()
        msg.vel.x = 0.0
        msg.vel.y = 0.0
        msg.vel.z = 0.0
        
        # 设置加速度（设为0）
        msg.acc = Vector3()
        msg.acc.x = 0.0
        msg.acc.y = 0.0
        msg.acc.z = 0.0
        
        # 设置加加速度（设为0）
        msg.jerk = Vector3()
        msg.jerk.x = 0.0
        msg.jerk.y = 0.0
        msg.jerk.z = 0.0
        
        # 设置偏航角
        msg.yaw = float(yaw)
        msg.yaw_dot = 0.0  # 偏航角速度
        
        # 设置控制器增益（使用默认值）
        msg.kx = [1.5, 1.5, 1.5]  # 位置增益
        msg.kv = [1.5, 1.5, 1.5]  # 速度增益
        
        # 设置轨迹ID和标志
        msg.trajectory_id = 1
        msg.trajectory_flag = 1  # TRAJECTORY_STATUS_READY
        
        # 发布消息
        self.publisher.publish(msg)
        self.get_logger().info(f'发送位置命令: x={x:.2f}m, y={y:.2f}m, z={z:.2f}m, yaw={yaw:.2f}rad')


def interactive_mode(node):
    """交互式模式：让用户输入位置"""
    print("\n=== 无人机位置命令发送器 ===")
    print("输入目标位置（输入 'q' 退出）")
    print("格式: x y z [yaw]")
    print("示例: 5.0 3.0 2.0 0.0")
    print("===========================\n")
    
    while rclpy.ok():
        try:
            # 获取用户输入
            user_input = input("请输入位置 (x y z [yaw]): ").strip()
            
            if user_input.lower() == 'q':
                print("退出程序")
                break
            
            if not user_input:
                continue
                
            # 解析输入
            parts = user_input.split()
            if len(parts) < 3:
                print("错误: 需要至少3个参数 (x, y, z)")
                continue
                
            x = float(parts[0])
            y = float(parts[1])
            z = float(parts[2])
            
            # 可选参数：偏航角
            yaw = 0.0
            if len(parts) >= 4:
                yaw = float(parts[3])
            
            # 发送命令
            node.send_position_command(x, y, z, yaw)
            
            # 等待一下
            time.sleep(0.1)
            
        except ValueError:
            print("错误: 请输入有效的数字")
        except KeyboardInterrupt:
            print("\n程序被用户中断")
            break


def predefined_mission(node):
    """预定义任务：执行一系列位置点"""
    print("\n=== 执行预定义任务 ===")
    
    # 定义一系列位置点 (x, y, z, yaw)
    waypoints = [
        (0.0, 0.0, 2.0, 0.0),      # 起飞到2米高度
        (5.0, 0.0, 2.0, 0.0),      # 向前移动5米
        (5.0, 5.0, 2.0, 1.57),     # 向右移动5米，偏航90度
        (0.0, 5.0, 2.0, 3.14),     # 向后移动5米，偏航180度
        (0.0, 0.0, 2.0, 0.0),      # 返回起点
        (0.0, 0.0, 0.5, 0.0),      # 降落
    ]
    
    for i, (x, y, z, yaw) in enumerate(waypoints):
        print(f"\n航点 {i+1}/{len(waypoints)}: ({x}, {y}, {z}), yaw={yaw}")
        node.send_position_command(x, y, z, yaw)
        
        # 等待无人机到达位置（简单等待）
        if i < len(waypoints) - 1:  # 不是最后一个航点
            wait_time = 5.0  # 等待5秒
            print(f"等待 {wait_time} 秒...")
            time.sleep(wait_time)
    
    print("\n任务完成!")


def main():
    rclpy.init()
    
    # 创建节点
    node = PositionCommandSender()
    
    try:
        # 询问用户选择模式
        print("选择模式:")
        print("1. 交互式模式 (手动输入位置)")
        print("2. 预定义任务 (执行预设航点)")
        print("3. 发送单个测试命令")
        
        choice = input("请输入选择 (1/2/3): ").strip()
        
        if choice == '1':
            interactive_mode(node)
        elif choice == '2':
            predefined_mission(node)
        elif choice == '3':
            # 发送测试命令
            print("\n发送测试命令: x=2.0, y=1.0, z=1.5, yaw=0.0")
            node.send_position_command(2.0, 1.0, 1.5, 0.0)
            print("测试命令已发送")
        else:
            print("无效选择，使用交互式模式")
            interactive_mode(node)
            
    except KeyboardInterrupt:
        print("\n程序被用户中断")
    finally:
        # 清理
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()