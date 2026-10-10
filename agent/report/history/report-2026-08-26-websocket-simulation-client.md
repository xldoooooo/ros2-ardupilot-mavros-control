# 独立地面站 WebSocket 模拟客户端交付报告

- 执行日期：2026-08-26
- 交付目录：`dev/integration/gcs-websocket-client/`
- 总体结果：已完成，单文件客户端、依赖清单、安装运行调试文档和端到端验证均通过
- 安全边界：全程只运行本机 WebSocket 消息模拟；未连接 ROS、MAVROS、仿真飞控或实机，未执行
  任何实机解锁或起飞操作

## 1. 任务结论

已将当前地面站与上位机之间的 WebSocket 收发行为实现为一个可单独交付和运行的模拟客户端。
生产逻辑集中在 `gcs-websocket-client.py` 单个 Python 文件中，不导入项目地面站、ROS、MAVROS、
飞控或视频服务代码。

客户端启动后自动连接可配置 URL，完成随附 JAR 使用的
`SYSTEM → SUBSCRIBE → SUB_ACK` 主题握手，订阅
`drone/{clientNo}/command`，并向 `drone/{clientNo}/status` 实际发布协议回复。每次连接成功先
发送待机 `01`，此后每 1 秒发送电量 `0A` 和位置 `0B`。这些状态不是地面站日志文本，而是实际
WebSocket `PUBLISH` 消息；终端仅保留独立程序自身的运维调试输出。

## 2. 协议依据与实现取舍

### 2.1 权威资料核对

- 完整读取并渲染检查 `dev/integration/无人机与上位机通信协议V2.0.docx` 全部 4 页。
- DOCX 表 2 含 `0C`“巡检电量不足，暂停巡检，返航充电”；同目录复制版 TXT 漏掉该行，故实现
  以 DOCX 为准保留 `0C`。
- DOCX 定义业务主题和业务 JSON，但没有写出主题服务外层信封；外层信封按当前生产地面站与随附
  `websocket-server-1.0.0.jar` 已验证实现统一为 `SYSTEM/SUBSCRIBE/SUB_ACK` 和
  `PUBLISH/BROADCAST`。
- 协议原始命令为 02/03/05/07；当前地面站已经扩展 01 起飞和 06 降落，因此模拟客户端同时支持
  01/02/03/05/06/07。
- 协议没有定义失败确认格式。合法命令先回复 `{clientNo, commandNo}`；无效消息只拒绝并记录
  独立程序诊断，不私自添加负 ACK 字段。

### 2.2 模拟时序

| 命令 | 模拟行为与 WebSocket 回复 |
| --- | --- |
| 01 起飞 | 立即回复 ACK；默认 5 秒后进入空中状态，后续 0B 反映高度 |
| 02 航线下发 | 校验并保存 1～256 个航点，回复 ACK 和状态 02 |
| 03 执行巡检 | 回复 ACK；未起飞时模拟起飞 5 秒；发送 03；每 5 秒完成一航点并发送 09；发送 07 并模拟降落 5 秒；最后发送 08 和 01 |
| 05 一键返航 | 回复 ACK，抢占旧动作，发送 05；默认 5 秒返回原点，再发送 07、模拟降落并发送 01；不发送 08/09 |
| 06 降落 | 回复 ACK，抢占旧动作，发送 07；默认 5 秒后发送 01 |
| 07 紧急停机 | 回复 ACK，抢占旧动作，按原地降落模拟，默认 5 秒后发送 01 |

03 必须已有一条通过校验的 02 航线。没有航线时仍按协议确认收到 03，但不伪造巡检开始或完成。
巡检点的 `pointNo` 使用原始 `taskPoints.index`；`cameraAngle` 当前只校验不执行云台动作，
`photoNo` 原样保留供后续媒体扩展。

默认媒体占位值为：

- `08.data.videoPath=/home/share/test.mp4`
- `08.data.JPGPath=/home/share/jpg`
- `09.data.pointPic=/home/share/jpg/test.jpg`

电量低于 20% 时发送一次 0C；若模拟飞机已在空中，还会按当前仿真地面站行为抢占任务并模拟
返航。电量恢复后可再次触发新低电量边沿。

### 2.3 连接与配置

- 默认 URL 为 `ws://127.0.0.1:8581/ws`，默认编号为 `UAV01001`。
- URL、编号、电量、初始 XYZ、起飞/航点/返航/降落时长、媒体路径、遥测周期、重连周期和日志
  等级均可用命令行参数修改；URL 和编号默认值也可直接在文件顶部修改。
- 连接、SYSTEM 首帧和 SUB_ACK 使用同一握手超时；首次 15 秒，连续失败按 5 秒递增并在 30 秒
  封顶，成功连接后恢复 15 秒。
- 断线后默认每 3 秒重连；断线期间不生成或积压旧状态。新会话重新发送 01、0A、0B。
- WebSocket ping/pong、发送串行锁、2 MiB 接收上限和 UTF-8 JSON 对象校验均已配置。

## 3. 文件变更清单

| 文件 | 类型 | 内容与目的 |
| --- | --- | --- |
| `dev/integration/gcs-websocket-client/gcs-websocket-client.py` | 新增 | 唯一生产实现；协议校验、主题握手、自动重连、命令确认、任务时序、遥测和 CLI 参数 |
| `dev/integration/gcs-websocket-client/requirements.txt` | 新增 | 固定唯一依赖 `websockets==15.0.1`，供一条 pip 命令安装 |
| `dev/integration/gcs-websocket-client/README.md` | 新增 | Windows/Ubuntu 安装 Python、虚拟环境、依赖安装、运行参数、Postman 消息、时序和排障说明 |
| `MEMORY.md` | 修改 | 增加独立模拟客户端当前基线与真实 JAR 验证事实，并把当前 Python 测试基线更新为 182 项 |
| `agent/report/report-2026-08-26-websocket-simulation-client.md` | 新增 | 本次实现、验证、限制与工作树保护记录 |

没有修改生产地面站、ROS 接口、机载控制、飞控算法或 `video_service/`。工作树中原有的根
`README.md`、`TODO.md`、Windows 摄像头教程 Markdown/PDF 改动均属于用户已有内容，本任务没有
触碰或覆盖。

## 4. 验证结果

### 4.1 静态与协议边界

- Python `py_compile`：通过。
- Ruff：`All checks passed!`。
- `git diff --check`：通过。
- 文件可执行权限：通过。
- `requirements.txt` dry-run：`websockets==15.0.1` 可满足。
- `pip check`：`No broken requirements found.`。
- 纯协议检查：主题、状态构造、合法 02，以及错误 clientNo、缺字段、重复 index、非法 Z 拒绝均
  通过。
- 新增目录禁用称呼扫描：无匹配。

### 4.2 随附真实 JAR 端到端

使用 Java 8 在本机隔离测试端口 18951 启动随附
`dev/integration/websocket-server-1.0.0.jar`，客户端和上位机测试端均经过真实主题服务通讯。

两点完整巡检的非遥测业务顺序精确为：

```text
01 → ACK02 → 02 → ACK03 → 03 → 09 → 09 → 07 → 08 → 01
```

进一步核对：

- 09 点号：`1, 2`。
- 两条 09 图片路径：均为 `/home/share/jpg/test.jpg`。
- 08 数据：`{"videoPath":"/home/share/test.mp4","JPGPath":"/home/share/jpg"}`。
- 默认 1 Hz 遥测：0A 两轮间隔 `1.001 s, 1.001 s`；0B 两轮间隔
  `1.001 s, 1.001 s`。
- 01 起飞：`ACK01` 与后续空中 0B 通过。
- 06 普通降落：`ACK06 → 07 → 01` 通过。
- 07 紧急降落：`ACK07 → 07 → 01` 通过。
- 05 抢占正在执行的 03：`ACK05 → 05 → 07 → 01` 通过，未出现迟到 09/08。
- 低电量：0C 只发送一次；空中自动返航状态为 `05 → 07 → 01`。

测试结束后只停止本次启动的 JAR；端口 18951 已确认无监听。

### 4.3 自动重连

独立本机 WebSocket 探针主动关闭首次会话后，客户端自动建立第二次会话；两个会话都重新完成
订阅并各发送一次待机 01。结果：`connections=2`、`standby_sessions=[1,2]`。

### 4.4 项目回归

第一次直接运行项目测试时未加载 ROS 2 Jazzy 与项目 `install/` overlay，得到
`175 passed / 7 failed`；7 项均为同一个环境错误 `No module named 'guided_interfaces'`。
随后按项目基线正确加载：

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
QT_QPA_PLATFORM=offscreen .venv/bin/python -m pytest -q tests
```

最终结果为 `182 passed in 43.03s`。这次环境性首轮失败和正确复跑结果均在此如实保留。

## 5. 已知边界

- 只验证了随附本地 JAR 和独立 WebSocket 探针，尚未连接上位机生产服务器；生产 URL、网络、
  鉴权、TLS 证书和服务端实现仍需现场联调。
- FTP、RTSP、真实图片、真实录像和云台控制不在本模拟模块范围内；媒体字段仅返回固定占位路径。
- 本模块只模拟通讯和时间推进，不模拟飞行动力学、避障、飞控故障或真实媒体生成。
- 每个进程只模拟一个 `clientNo`；需要同时模拟多架时，应为每个编号启动一个独立进程。
- 命令 03 无已保存航线时不执行；协议没有负 ACK，所以只能在独立程序日志中看到拒绝原因。
- 本次没有创建、停止、重启或部署任何实机服务，也没有实机飞行验证结果。
