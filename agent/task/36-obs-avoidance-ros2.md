# 避障算法ROS2移植

## 核心要求

https://github.com/hku-mars/dyn_small_obs_avoidance



将此避障方法移植到ROS2, 重点是需要适配ros2-jazzy



用当前git的LostPatrol账号形成独立仓库dyn_small_obs_avoidance-ros2，本地目录放在/home/nvidia/scq/projects 目录下



只需要移植其path  search 和 path plan



## 其他参考

task35 任务文件

以及task35执行报告两份



## 注意事项

飞机new当前在线，可按需连接，调试，启动和采集点云等；但不要解锁或起飞。点云由Odin给出，odin配置与本地不一致是因为开启了点云相关配置。

可以尝试使用其提供的数据包进行验证。

先实现独立目录，先不进行与本项目相关的集成工作；不要为了适配本项目而改动或破坏原ros1版本正常的算法和逻辑。

原项目如果存在明显缺陷或严重漏洞，视情况进行修复。

需要形成必要配套文档