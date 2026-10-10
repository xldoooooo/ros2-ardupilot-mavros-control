# refresh MAVROS 时间同步修复

本报告记录飞控重启被拒绝的原因、启动修复与验证边界。

- 现场错误：10:33:24 地面站 reboot_fcu 被拒绝，提示 MAVROS 命令服务或飞控启动时钟不可用。命令服务存在，但 MAVROS 2.15.1 时间插件实际 timesync_rate=0.0，配置文件为10.0，未产生有效时钟消息。
- `runtime_ensure_mavros_timesync` 检查实际模式与频率，仅在MAVLINK模式且频率为0时恢复10Hz；保留已有正频率。必须参数读回成功且收到remote_timestamp_ns>0，才报告成功。
- 完整机载入口与分组件MAVROS入口共用检查。检查失败清理本次启动的进程；未修改飞控重启安全门、控制算法或飞控参数，不自动执行重启。
- 本地6项模拟异常/正常分支、17项部署回归及取消清理测试通过（合计24项）；原有启动集成测试首次1项参数拉取等待超时，单独复跑4项全部通过，未为通过测试降低断言。
- refresh真实MAVROS 2.15.1在隔离domain228、localhost UDP虚拟飞控下，完成0→10Hz设置、读回及有效TIMESYNC接收。早期隔离测试因测试进程时间窗口结束及DDS发现等待不足失败，随后统一编排启动/清理并增加有界发现等待后通过。
- 首次现场热应用期间，MAVROS与onboard进程退出，参数设置曾返回成功，但无法完成后续读回；此轮不计为现场验收成功。
- 过程脚本/日志位于开发机agent/codex及refresh的agent/codex/timesync-fix。未操作new；其Shell迁移和本修复仍待后续同步。

## 最终部署与现场验收

- 修复提交f3186bd已推送main，并通过增量Git bundle快进同步refresh；无需更改系统MAVROS安装包或编译控制器。
- 确认所有旧飞控栈进程均已退出后，于10:52:27启动ros2-ardupilot-onboard.service（未enable）。
- 10:52:40新入口自动检测0Hz并恢复10Hz；10:52:47参数设置成功；10:52:55读回10.0Hz并验证真实FCU TIMESYNC；10:52:57完整启动输出READY。
- 真实时钟样本remote_timestamp_ns=2040329897000；状态fcu_connected=true、armed=false、on_ground=true、local_position_valid=true、controller_active=false，参数和消息频率均就绪。
- 服务最终active/running，开机自启仍disabled；未启动视频/修正服务。无显示环境的既有RViz退出不影响Odin数据与READY，本轮未修改厂商launch。
- 已通知用户可在地面站手动重试。代理未发送飞控重启、解锁或起飞命令，因此实际热重启闭环仍待用户操作验收。
