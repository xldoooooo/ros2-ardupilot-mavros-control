# refresh 飞控重启后 MAVROS 参数刷屏修复

## 问题与根因

- 用户日志为 `PR: got an unsolicited param value idx=..., not resetting retries count 3`。
- refresh 保留日志确认此前重启请求已经成功，启动时钟回退被检测到，控制链路随后恢复；
  刷屏来自之后的 MAVROS 参数整表补拉，并非重启命令再次失败。
- refresh 的系统 MAVROS 为 2.15.1。参数插件在超时补拉阶段收到非当前等待参数时，
  仍立即重新请求当前缺失参数，并重置计时器，导致重复回包放大请求、干扰重试上限。
- 同一源码的定时拉取回调在 busy 分支重新排期后没有 return，会重启正在进行的拉取。

## 改动

- 固定 MAVROS 2.15.1 上游源码与 SHA-256，维护小范围补丁：只有当前等待参数到达时，
  才重置超时并立即请求下一个；其他参数仍正常进入缓存和事件通知。无关回包日志改为 DEBUG，
  实际超时和重试耗尽仍保留 WARN/ERROR。定时拉取 busy 分支立即返回。
- 新增独立 overlay 构建入口 `src/onboard_control/deploy/install_mavros_param_fix.sh`，
  保留系统安装，源码与产物位于飞机 `~/mavros_param_fix_ws`。构建不自动重启服务。
- 完整及独立 MAVROS 入口读取 `MAVROS_OVERLAY_SETUP`；完整入口最后加载 MAVROS overlay，
  避免 Odin/extnav 工作区覆盖它。部署说明包含依赖、启用与回退步骤。
- 保留启动时钟验证及飞控重启后的新参数验证，本任务没有优化或缩短启动等待。

## 验证记录

- 在 refresh 上使用独立 domain 229、localhost UDP 虚拟飞控复现，不接串口：
  原版收到30个重复回包后，单个缺失参数产生31次请求；补齐该参数后整表接收完成。
- 本地启动/部署测试：23 passed；shell 语法检查和 diff 空白检查通过。
- 补丁后同样30个重复回包只产生1次请求，补齐参数后整表正常完成。连续重复回包7秒的
  另一用例只产生3次重试，正常报出缺失参数；没有被重复回包拖延，也没有 unsolicited 刷屏。
- refresh 原生构建成功；通过 `/proc/<pid>/maps` 确认真实串口 MAVROS 加载独立 overlay 的
  `libmavros_plugins.so`。飞机 connected=true、armed=false，实际 TIMESYNC 校验通过。
- 两轮真实强制拉取均完成：第一轮1235项，伴有索引变化及缺11项警告，不能作为完整整表
  成功；第二轮收到全部1246项并明确输出 `PR: parameters list received`。两轮均没有
  unsolicited 刷屏。`STAT_RUNTIME` 的异步更新索引65535仍可能产生单条索引变化警告，
  本补丁没有掩盖此类诊断。
- 已推送 main 并同步 refresh，配置已启用 overlay；原配置备份为
  `/etc/ros2-ardupilot/onboard.env.before-param-fix-20260918`。
- 校验后停止本次启动的 MAVROS，保持此前手动运行的 Odin/extnav 不变；飞控 systemd unit
  原先 inactive，未改为自启。下次通过项目完整/独立入口启动时自动使用补丁。

## 边界与回退

- 未发送解锁、起飞或飞控重启命令。真实重启闭环需由用户手动触发。
- 保留 refresh 上手动运行的 Odin/extnav；不修改现场相机数据或其他任务文件。
- 移除 onboard.env 的 `MAVROS_OVERLAY_SETUP` 并重新启动 MAVROS 即回退系统版本。
- 该 overlay 针对 2.15.1 构建；以后升级系统 MAVROS 必须重新核对源码补丁和 ABI。
- new 飞机本轮未连接、未部署，下次同步时一并评估版本并安装。
