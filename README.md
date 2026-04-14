# ROS2-ArduPilot SITL Hardware

一个用于无人机仿真的 ROS2 项目，集成了 ArduPilot SITL（Software-In-The-Loop）仿真和硬件控制功能。

## 项目概述

这个项目提供了一个完整的 ROS2 生态系统，用于：
- 通过 ArduPilot SITL 进行无人机仿真
- 使用 MAVROS 与仿真无人机通信
- 实现高级飞行控制器（APCtrl）
- 提供自定义的无人机消息类型
- 支持任务规划和执行

## 项目结构

```
ros2-ardupilot-sitl-hardware/
├── launch/                    # 启动脚本
│   ├── start_sitl.sh         # 启动 ArduPilot SITL 仿真
│   ├── start_mavros.sh       # 启动 MAVROS 节点
│   ├── start_mavros_real.sh  # 启动真实硬件 MAVROS
│   └── stop_*.sh             # 停止脚本
├── scripts/                  # Python 脚本
│   └── missions/             # 任务脚本
│       ├── mission_simple.py # 简单任务示例
│       └── README.md         # ROS2 命令参考
├── src/                      # ROS2 包源代码
│   ├── apctrl/               # APM 控制器包
│   │   ├── include/          # C++ 头文件
│   │   ├── src/              # C++ 源文件
│   │   ├── config/           # 配置文件
│   │   ├── launch/           # ROS2 启动文件
│   │   ├── CMakeLists.txt    # CMake 配置
│   │   └── package.xml       # ROS2 包定义
│   └── quadrotor_msgs/       # 自定义消息包
│       ├── msg/              # ROS2 消息定义
│       ├── CMakeLists.txt    # CMake 配置
│       └── package.xml       # ROS2 包定义
└── .gitignore               # Git 忽略文件
```

## 主要功能

### 1. ArduPilot SITL 仿真
- 使用 Gazebo 进行物理仿真
- 支持 ArduCopter 模型
- MAVLink 通信接口（端口 14550, 14551）

### 2. MAVROS 集成
- ROS2 与 ArduPilot 之间的桥梁
- 提供 ROS2 话题和服务接口
- 支持飞行控制、状态监控等

### 3. APCtrl 控制器
- 高级飞行状态机（FSM）
- 位置控制、姿态控制
- 输入处理和几何计算工具

### 4. 自定义消息
- 四旋翼专用消息类型
- 包括状态估计、控制命令、调试信息等
- 支持复杂的飞行控制算法

### 5. 任务执行
- 预定义的任务脚本
- 支持自主起飞、航点飞行、返航等
- 可扩展的任务框架

## 快速开始

### 前提条件

1. **ROS2 Humble** 或更高版本
2. **ArduPilot** 安装（建议使用最新版本）
3. **MAVROS** ROS2 包
4. **Gazebo** 仿真环境

### 安装步骤

1. 克隆仓库：
   ```bash
   git clone https://github.com/xldoooooo/ros2-ardupilot-sitl-hardware.git
   cd ros2-ardupilot-sitl-hardware
   ```

2. 构建 ROS2 工作空间：
   ```bash
   colcon build
   source install/setup.bash
   ```

3. 确保 ArduPilot 已正确安装并配置环境变量。

### 启动仿真

1. **启动 SITL 仿真**：
   ```bash
   ./launch/start_sitl.sh
   ```
   这将启动 Gazebo 和 ArduPilot SITL。

2. **启动 MAVROS**（在新终端中）：
   ```bash
   ./launch/start_mavros.sh
   ```

3. **运行示例任务**（在新终端中）：
   ```bash
   cd scripts/missions
   python3 mission_simple.py
   ```

## 使用示例

### 监控无人机状态

```bash
# 检查连接状态
ros2 topic echo /mavros/state

# 监控本地位置
ros2 topic echo /mavros/local_position/pose

# 检查 GPS 状态
ros2 topic echo /mavros/global_position/global
```

### 手动控制

```bash
# 设置 GUIDED 模式
ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode "{custom_mode: 'GUIDED'}"

# 解锁电机
ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool "{value: true}"

# 起飞到 10 米高度
ros2 service call /mavros/cmd/takeoff mavros_msgs/srv/CommandTOL "{altitude: 10.0}"

# 返航
ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode "{custom_mode: 'RTL'}"
```

### 运行自定义任务

查看 `scripts/missions/mission_simple.py` 作为基础模板，创建自己的任务脚本。

## 开发指南

### 添加新的消息类型

1. 在 `src/quadrotor_msgs/msg/` 目录中创建新的 `.msg` 文件
2. 更新 `CMakeLists.txt` 和 `package.xml`
3. 重新构建包：
   ```bash
   colcon build --packages-select quadrotor_msgs
   ```

### 扩展控制器功能

1. 修改 `src/apctrl/include/` 中的头文件
2. 更新 `src/apctrl/src/` 中的实现
3. 重新构建包：
   ```bash
   colcon build --packages-select apctrl
   ```

### 创建新的启动配置

1. 在 `launch/` 目录中创建新的 shell 脚本
2. 确保正确处理依赖关系和端口配置
3. 测试启动脚本的各个组件

## 故障排除

### 常见问题

1. **SITL 无法启动**：
   - 检查 ArduPilot 安装路径
   - 确保 Gazebo 正确安装
   - 验证端口 14550 和 14551 未被占用

2. **MAVROS 连接失败**：
   - 确认 SITL 正在运行
   - 检查 `fcu_url` 配置
   - 验证网络连接

3. **ROS2 话题不可见**：
   - 确保正确 source 了工作空间
   - 检查节点是否正在运行
   - 使用 `ros2 node list` 和 `ros2 topic list` 验证

### 调试工具

```bash
# 列出所有节点
ros2 node list

# 列出所有话题
ros2 topic list

# 查看节点信息
ros2 node info /mavros

# 监控服务质量
ros2 topic hz /mavros/local_position/pose
```

## 贡献指南

1. Fork 本仓库
2. 创建功能分支 (`git checkout -b feature/AmazingFeature`)
3. 提交更改 (`git commit -m 'Add some AmazingFeature'`)
4. 推送到分支 (`git push origin feature/AmazingFeature`)
5. 打开 Pull Request

## 许可证

本项目采用 TODO 许可证。详见 `src/apctrl/package.xml` 和 `src/quadrotor_msgs/package.xml`。

## 致谢

- **ArduPilot** 团队提供的优秀开源自动驾驶仪软件
- **ROS2** 社区提供的机器人操作系统框架
- **MAVROS** 开发者提供的 MAVLink-ROS 桥接
- 所有为本项目做出贡献的开发者

## 联系方式

如有问题或建议，请通过以下方式联系：
- GitHub Issues: [项目 Issues 页面](https://github.com/xldoooooo/ros2-ardupilot-sitl-hardware/issues)
- 维护者邮箱: your_email@example.com

---

**注意**: 本项目仍在积极开发中，API 和功能可能会发生变化。建议定期更新到最新版本。