<!-- 航点行为、自动 Tag 校准状态机及两端升级入口。 -->
# 航点 Tag 校准工作流

## 使用与范围

航点编辑表的巡检、校准标志互相独立：

- 普通点：移动、到点；不抓拍、不上报巡检到达消息。
- 巡检点：到点即抓拍并上报；不等待同点的校准完成。
- 校准点：到点后暂停推进索引，开启已有下视校准服务并自动应用。
- 两种标志可同时启用；新增点及旧 CSV 导入点均默认普通点。

校准点只支持 **odom 绝对坐标＋直线/遇障悬停**；map、自主避障、`local_coordinates` 暂不支持。无校准点的原有策略不受此限制。旧上位机 `taskPoints` 仍显式视为巡检点，无需改变 V2.2 协议。

CSV 表头如下，`yaw` 单位为度，程序/ROS 中为弧度：

```csv
index,x,y,z,yaw,is_inspection,is_calibration,calibration_tag_id,continue_window
1,0,0,1.2,0,true,true,0,false
2,1,0,1.2,0,false,false,0,false
3,2,0,1.2,0,false,true,1,true
```

布尔字段接受 `true/false` 或 `1/0`；Tag ID 为非负 int32。旧五列表头仍可导入。非法字段在导入/提交时报错，未知 Tag、重复窗口 Tag、服务不可用和不兼容组合由提交时的机载权威检查拒绝，GUI 显示具体原因。

## 窗口和控制衔接

本任务首个校准点总是清空残留滑窗并执行 FIRST，忽略它的沿窗选项；**清窗不清除 extnav 已生效修正**。后续点沿窗时执行 NEXT，不沿窗则清窗开启新 FIRST 段。窗口长度沿用校准服务配置；同一 Tag 尚在窗口内时不能再次沿窗加入，退出窗口或新开段后可重用同一真实 Tag 参数。

沿窗依赖本任务记录的服务实例、Odin session、窗口 revision 与有效历史；重启、过期或手动修改历史不会静默改成 FIRST。到点之前及校准结束后仍允许手动面板操作，但修改历史后不能继续依赖原窗口。

状态机非阻塞运行：申请独占 → 必要清窗 → 启动 FIRST/NEXT → 核对本次 job 的应用状态及 extnav revision/最终样本 → 等待实际 MAVROS EKF 与修正后辅助 pose 的 XY/yaw 一致、低速且持续稳定 → 推进下一点。ACK 或固定延时不能单独推进任务。

应用期间悬停参考随当前融合位置衔接，不追赶旧坐标参考；PD+DOB 的水平速度暂用既有 `/extnav/velocity_fcu` 物理速度，避免把 EKF 修正暂态速度当作机体运动。该速度失效时 LAND，融合完成还须核对 EKF 与桥速度一致；Z 与控制器算法/扰动历史不变。余下航点始终是校正世界里的原始绝对目标，不再次变换。末点同样等待校准与融合完成后才报告任务成功。

既有采样/求解/应用分阶段超时保持不变；机载 `calibration_timeout_seconds` 默认 **120 秒**，包含申请、应用及飞控融合。异常结束任务并调用既有原地 LAND；用户悬停、取消、降落或失联接管则异步停止校准，迟到结果不能恢复旧任务。校准独占期间面板允许看状态，不允许并发启动、清窗、应用或保存 Tag 配置；释放资源后恢复按钮。

## 构建与维护

飞行接口为 **3.6**，视频仍为 3.2，校准/extnav仍为 2.0。部署须更新两端源码，先停止旧进程，再原生重建消息、控制/修正服务及消费 `ControlStatus` 的 `avoidance_bridge`，不能混用旧可执行程序与新消息动态库：

```bash
source /opt/ros/jazzy/setup.bash
# 先加载本机配置的独立规划器 overlay（以下为相邻目录默认路径）。
source ../dyn_small_obs_avoidance-ros2/install/setup.bash
colcon build --base-paths src correction_service \
  --packages-select correction_interfaces guided_interfaces onboard_control guided_sim correction_service avoidance_bridge \
  --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

- `src/guided_interfaces/msg/Waypoint.msg`、`ground_station_core/waypoint_io.py`：标志与 CSV。
- `ground_station_core/qt_ui/waypoint_panel.py`、`ros_controller.py`：编辑、提交和状态展示。
- `src/onboard_control/src/waypoint_calibration.cpp`：到点异步状态机、窗口约束和融合衔接。
- `correction_service/correction_service/node.py`：复用校准算法及跨操作独占；面板逻辑在 `correction_panel.py`、`ros_client.py`。
- `ground_station_core/upstream/status_projector.py`：只统计巡检点，并处理抓拍/校准并发。

自动校准不替代原有算法质量检查。SITL 注入候选可以检验真实桥、EKF 和控制衔接，但不等于相机检测或实机验收；实机解锁、起飞仍只能由用户手动完成。
