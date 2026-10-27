<!-- Navigation coordinate conventions and the small set of implementation entry points. -->
# odom / map 任务坐标

飞行接口 3.5 支持起飞前选择 odom 或 Odin 重定位 map。地图使用 Odin SLAM 保存的 bin；
不在飞行中切换工作模式、坐标系或地图。地面站、CSV 和 WebSocket 的航点数值不随模式切换改写，
发送时按飞机报告的当前任务坐标解释。起飞高度、手动速度和返航点继续使用 FCU 本地语义。

## 坐标关系

Odin 原始高频里程计与实时 SLAM 点云在 `odom`，原生 TF 是 `odom ← map`。
记 `T_OM=(R_OM,t_OM)`，FCU 本地原点偏移为既有 extnav 的常量 `b=T_FI`：

```text
p_local = R_OM * p_map + t_OM + b
p_map   = inverse(R_OM) * (p_local - t_OM - b)
```

航点与采点均以 **FCU 中心** 为参考。`b` 是历史本地原点偏移，实际物理杆臂在 extnav 内计算：
`R_OF=R_OI R_IF`，`p_local=p_OI+b-R_OF T_FI`，速度同时减去旋转杆臂速度。
安装仰角只改变传感器到机体的姿态转换，不再次倾转 Odin 已调平的世界点云。

MAVROS 消息原有的 `map` 名称仍表示 FCU 本地值；不能用于推断 Odin 地图坐标。
map 模式下只读预览使用 `fcu_local`，由机载发布静态 `odom ← fcu_local`、平移 `-b`，
避免与 Odin 的真实 map TF 同名。odom 模式保留原显示行为。
航点与轨迹显示使用零时间戳查询最新 TF；MAP 机体位姿先按最新 `map ← fcu_local` 转入显示系，
模型 TF 保留状态源时刻，兼容 Odin 设备启动秒与控制状态墙钟不同的配置，不改写原生 TF 或控制输入。

## 配置与切换

- 安装外参：`config/aircraft/<AIRCRAFT_ID>/odin_extrinsics.yaml`，由本机 `config/aircraft.local.env` 选择飞机。
  `roll_cam/pitch_cam/yaw_cam` 定义 `R_FI=Rz Ry Rx`，单位弧度；机头向上安装为负 pitch。
  `odin_x/y/z` 是 FCU 到 Odin IMU 的向量，以 FCU FLU 表达，单位米。
  手工回退角按飞机设置，上仰22°对应约`-0.3839724354`。
  `auto_installation: true`在未解锁静止约3秒后，用Odin/飞控重力方向自动学习安装俯仰并锁定，
  roll/yaw及杆臂保持配置，不要求机身摆平；重启extnav可重新学习。项目默认YAML已启用，
  旧逐机YAML需添加开关，详情见[extnav说明](../correction_service/extnav_patch/README.md)。
- 地图：`config/aircraft/<AIRCRAFT_ID>/odin/reloc.yaml` 的 `relocalization_map_abs_path`。
  必须先提供存在的 bin；地图文件放 `drivers/odin/maps/`，不提交 Git。
- 默认选择：`config/aircraft/<AIRCRAFT_ID>/navigation.env` 的 `NAVIGATION_MODE=odom|map`。
  不传 `-c` 时完整、分步启动均读取该选择；显式 `-c odom|reloc` 可覆盖本次启动。
- 地面站开关调用 `/onboard_control/set_navigation_mode`，需持权、未解锁、已落地且无任务。
  二次确认后脚本退出原飞行栈及原运行的避障，再按原运行情况重启；视频、修正服务独立。
  systemd 启动需允许服务用户无交互执行相关 `sudo` 命令；失败在状态和持久日志中报告。

驱动 `/odin1/navigation_status` 提供设备 mode 设置 ACK、map 路径、连接 session、首次定位证据和 TF 年龄。
机载以实际运行状态识别模式，不能仅凭地面开关或配置文件宣称定位成功。

## 维护入口

| 功能 | 源码 |
| --- | --- |
| 坐标数学、小修正的连续偏移 | `src/onboard_control/include/onboard_control/navigation_frame.hpp` |
| 定位健康、目标转换、模式切换服务 | `src/onboard_control/src/navigation_frame.cpp` |
| 航点提交、起飞前校验、状态输出 | `src/onboard_control/src/onboard_control_node.cpp` |
| 规划目标撤销重发、遇障悬停通道重查 | `src/onboard_control/src/avoidance_execution.cpp` |
| 姿态、杆臂及自动安装俯仰 | `correction_service/extnav_patch/src/extnav_to_vision_pose.cpp`、`include/extnav_bridge/math.hpp` |
| 地面重启事务 | `scripts/onboard/switch_navigation_mode.sh` |
| GUI / 上位机坐标适配 | `ground_station_core/ros_controller.py`、`qt_ui/`、`upstream/` |
| RViz 机体时间与轨迹显示 | `src/guided_sim/scripts/pose_to_tf.py`、`avoidance_preview.py` |

修改 RViz 桥脚本后，地面站需重新构建 `guided_sim` 并重启预览进程；运行依赖由包内 `tf2_geometry_msgs`、`tf2_ros` 声明。
飞机掉线恢复并完成连接维护后，地面站只重开已打开的硬件 RViz 进程组，清除 Odin 设备秒归零前的显示缓存；
控制租约、ROS 会话及其他本地进程保持，未使用预览时不会自动打开窗口。

地图原航点始终保留，机载转换后检查本地 Z `[0.1,50]m`。正常小修正累计到配置精度后，
直线叠加连续位置/速度/加速度偏移，自主避障撤销旧目标再规划，遇障悬停先重新检查新通道再应用偏移。
大跳变、定位超时或飞行中 session/模式变化取消原任务，本地制动悬停，默认 15 秒后 LAND；
不会自动切回 odom 或自动恢复旧任务。阈值在 `control.yaml` 的 `navigation_*`，不与 FCU 遥测时效混用。

**TODO：map 与 Odin-Tag 的组合尚未适配。** map 要求 extnav 无 active Tag 修正，GUI 禁用 Tag 入口；
非零安装角的 Tag apply 仍明确拒绝。切换和构建均不等于完成实飞或绝对定位精度验收。
