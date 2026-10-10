<!-- refresh 下视相机新外参部署与只读运行核验记录。 -->
# refresh UQ212 外参同步

日期：2026-09-17。用户明确要求同步 refresh。
目标：drone-refresh，/home/nvidia/ros2-ardupilot-mavros-control。

## 执行与结果

- 机载 main 从 57a15c1 快进至功能提交 97179d5，保留 start_drone/image/ 未跟踪产物。
- 原活动/安装外参分别备份在机载 agent/codex/uq212-extrinsics-sync-20260917/。
- colcon build --packages-select correction_service 成功（1 package）。
- 活动及 UQ212 档案的源码/install 文件逐字节一致。
- Wasintek 源码/install 备份 SHA-256 均保持
  `07b59f92545ad0ccb3d7af18b9e140ab2178190743eebd39d6bfd74fa3fd4b0e`。
- 仅重启 odin-correction.service，未重启 Odin、extnav、视频或飞控服务。
- 重启后服务 active，节点接口 2.0，state=idle、active=false、window_count=0，
  resources_released=true，相机与任务 Odin 订阅关闭，无 last_error。
- 运行 config_fingerprint 与开发机配置一致：
  `34f1f5313652e114afe4ea8507d3b717e8e91e6ed6eb8dcd112213abe5dae106`。

## 保留状态与未覆盖内容

重启前修正任务 inactive，窗口为空，未保存候选。extnav 旧修正保持 revision=1、
correction_valid=true、job=d6a27a45d988-apply，数值
x=-0.18162441531401205m、y=-0.060927308993003575m、yaw=-18.76852244965993°；
重启前后相同。本次没有调用应用/清除修正服务，也没有运行新标定。
这些旧 active 数值不是新外参的验证结果，需用户后续重新采样并应用。

/mavros/state 未发现发布，无法据此确认 armed 状态；本次未发送任何飞行命令、解锁或起飞。
新外参仍是机械安装近似，未做实物精度验收。new 飞机仍待同步，MEMORY 已维护。
本次另行提交部署记录并同步 main，不包含开发机原有其他未提交修改。
