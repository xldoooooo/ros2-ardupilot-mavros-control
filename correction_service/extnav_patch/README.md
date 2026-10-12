# extnav correction 2.0 生产补丁

这里保存飞机 `/home/nvidia/vrpn_mavros/src/extnav_bridge/` 的可审计源码模板。部署脚本只在
用户确认的未解锁维护窗口覆盖对应源码并原生重建；它会拒绝运行中的飞控链路，不会自行启动、
停止、重启、解锁或起飞飞机。

`extnav_to_vision_pose.py` 始终直接订阅 raw Odin。接口包或 correction_service 缺失、修正
无效或 Odin session 失效时，corrected 话题保持 identity，最终 FCU 输出保持既有局部零点
约定。修正有效时，corrected 仍表示 Odin IMU 中心；`/extnav/pose_fcu` 与
`/mavros/vision_pose/pose` 共同使用去除额外 `+T_xy` 的世界水平飞控中心结果。

当前简式只对已核实的零安装角和杆臂 `T=(0.06,-0.03,0.05)m` 作保证。补丁通过 2.0 状态
回报杆臂、安装角、水平参考模式、session/revision 和最终发布计数；非零安装角时拒绝设置有效
correction。raw 回调同时冻结最终 pose/velocity 与相关元数据，状态改变后必须等
下一条 raw，避免定时器把新 valid/revision 套到旧 pose。

raw 始终全频处理。corrected 仅在有匹配订阅者时按 raw 频率发布；DDS 订阅者数量每
0.1 秒查询一次，发现匹配后在下一轮查询及后续 raw 恢复输出（另受 DDS 发现和调度延迟影响）。
无订阅者时只计算最终中心快照，不构造 corrected 消息或旋转 covariance。identity 只读 raw，
发布时才复制独立消息；有效修正也不修改 raw。未使用的 `latest_msg` 已移除，定时器只读
独立数值快照。状态 `raw_messages` 统计处理输入，`corrected_messages` 只统计实际发布，
两者不再要求相等。vision/control 的配置频率及 QoS 保持原样。

世界高度仍未标定，z 保留旧局部数值约定。`PoseStamped` 仍无法携带 estimator reset
counter；该风险只在状态/日志中提示，不能把 correction ACK 解释为 EKF 稳定或可实飞。
