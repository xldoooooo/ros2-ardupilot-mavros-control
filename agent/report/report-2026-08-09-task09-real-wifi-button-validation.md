# 任务 09：独立 Wi-Fi 通讯检测按钮真机验证简报

日期：2026-08-09
地面站：Ubuntu 24.04 / ROS 2 Jazzy
真机伴随计算机：Ubuntu 22.04 / ROS 2 Humble / `192.168.112.186`

## 结论

调整后的独立 Wi-Fi 通讯检测按钮已在真机伴随计算机上完成实际点击验证，功能正常。

两次检测均通过生产 Qt 按钮信号、生产 `GroundStationRosController` 和生产 `EnvironmentInitializer` 路径执行，没有直接调用内部工作流来绕过 GUI。全程没有点击正式“连接实机”，没有申请控制租约，没有发送维护、模式、解锁、起飞、运动或航点指令。

## 安全准备

测试前只读检查确认：

```text
真机 onboard_control/MAVROS/Odin/extnav：无进程
onboard-control/mavros/odin systemd：      inactive
/dev/ttyTHS1：                            无占用
新仓库：                                  c8abad9，工作树干净
旧仓库：                                  dad9067，工作树干净
```

为只验证“地面站 ↔ 真机机载服务”的按钮职责，本轮仅在真机临时前台启动新工作区的 `onboard_control_node`，使用 domain 42。没有启动 MAVROS、Odin 或 extnav，也没有访问飞控串口，因此机载状态按预期报告 FCU 未连接、未武装。

按钮本身没有启动或停止真机服务；机载节点由测试人员在按钮外部临时启动，完成后通过 SIGINT 正常停止。

## 第一次真实按钮检测

真实 Qt Wi-Fi 图标按钮点击结果：

```text
结果：                       PASS
ControlStatus：              30 条 / 9.98 Hz
最大接收间隔：               102.2 ms
interface：                  2.0
FCU：                        未连接
armed：                      false
environment/mode：           false / none
control_enabled/authority：  false / false
lease_active/owner：         false / 空
command_result：             0
飞行按钮：                   全部禁用
```

启动日志未落入第一次按钮建立的 rosout 检测窗，因此该次报告远端日志 0 条；`ControlStatus.status_message`“机载控制服务已启动”仍由生产状态日志路径进入 GUI。没有把“稳定节点恰好没有新日志”误报为通信失败。

## 真机 rosout 复验

为验证 GUI 远端日志路径，在真机 Humble 端另启一个不提供参数服务、不发布控制消息的 ROS CLI 发布器，仅向 `/rosout` 发送 8 条固定测试文本：

```text
source：  task09_real_wifi_probe
message： task09-wifi-button-real-log
level：   INFO
```

随后第二次点击同一个真实 Wi-Fi 按钮：

```text
结果：                       PASS
ControlStatus：              30 条 / 9.98 Hz
最大接收间隔：               101.0 ms
远端 rosout：                8 条，GUI 原文接收
armed：                      false
environment/mode：           false / none
control_enabled/authority：  false / false
command_result：             0
飞行按钮：                   全部禁用
```

该测试日志发布器只验证日志话题，不调用任何机载服务或飞控接口。

## 清理核查

测试后主动停止临时机载节点并再次只读检查：

```text
onboard_control/MAVROS/测试日志/Odin/extnav：无残留
onboard-control/mavros/odin systemd：          inactive
/dev/ttyTHS1：                                无占用
新、旧真机仓库：                              工作树干净
SSH ControlMaster：                           已退出，控制 socket 已删除
```

未产生真机仓库修改，未安装或启用服务，未运行旧 `odin.sh`。

## 限制与说明

- 本次验证证明新 Wi-Fi 按钮能够通过真实无线局域网连接 Humble 机载服务、量化状态流并把远端日志反馈到 GUI。
- 本轮刻意没有启动 MAVROS，所以不验证本次按钮运行时的物理 FCU 串口链；此前任务 09 已单独完成 FCU 连接且始终 `armed=false` 的只读链路验证。
- 正式“连接实机”按钮仍未在本轮执行，因为它会按产品设计申请租约、配置消息频率并写入 GPS 原点。
- Humble/Jazzy Fast DDS 仍出现已知 `sequence size exceeds remaining buffer` 告警。已测话题稳定不代表完整跨发行版 ROS 图受到官方支持，也不构成实飞授权。
