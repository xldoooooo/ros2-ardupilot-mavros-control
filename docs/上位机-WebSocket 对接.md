# 上位机 WebSocket 对接

> 协议版本 V2.0。
>
> JAR 是 WebSocket 服务端；本项目地面站和 Reqable 都是客户端，二者不直接连接。



## 1. 

```text
Reqable：发 command，收 status
地面站：收 command，发 status
JAR：只按 topic 路由消息
机载服务：接收地面站 ROS 2 高层命令，真正控制飞行器
```

没有 JAR 或现场 WebSocket 服务端时，Reqable 和地面站两个客户端都会连接失败，它们不能直接互相通信。

## 2. 

| 组件 | 网络角色 | 订阅 | 发布 | 是否理解飞行动作 |
|---|---|---|---|---|
| JAR | WebSocket 服务端、主题路由器 | 不适用 | 将 `PUBLISH` 路由成 `BROADCAST` | 否 |
| Reqable | WebSocket 客户端，模拟上位机 | `drone/UAV01001/status` | `drone/UAV01001/command` | 只负责发送测试 JSON |
| 本项目地面站 | WebSocket 客户端 | `drone/UAV01001/command` | `drone/UAV01001/status` | 是，负责解析、映射和门控 |
| 机载服务 | ROS 2 服务端/状态发布端 | 地面站高层动作 | `ControlStatus`、`CommandResult` | 是，执行既有控制逻辑 |
| MAVROS / ArduPilot | MAVLink/飞控链路 | 机载控制请求 | 飞控状态和遥测 | 执行最终飞控动作 |

## 3. 完整通讯链路

![上位机、JAR、地面站与无人机通讯链路](<./上位机-地面站-无人机 通讯链路图.png>)

### 控制命令方向

```text
Reqable
  → PUBLISH 控制主题
  → JAR
  → BROADCAST 控制主题
  → 本项目地面站
  → JSON 校验、动作映射、执行条件检查
  → ROS 2 高层命令
  → 机载服务
  → MAVROS / MAVLink
  → ArduPilot SITL 或实机飞控
```

### 状态返回方向

```text
ArduPilot / MAVROS
  → 机载服务 ControlStatus / CommandResult
  → 本项目地面站
  → PUBLISH 状态主题
  → JAR
  → BROADCAST 状态主题
  → Reqable
```

## 4. 启动顺序

### 4.1 启动本地测试 JAR


java8:
```bash
sudo apt install -y openjdk-8-jre-headless
```

在项目根目录执行：

```bash
/usr/lib/jvm/java-8-openjdk-amd64/jre/bin/java \
  -jar dev/integration/websocket-server-1.0.0.jar
```

默认测试地址：

```text
ws://127.0.0.1:8581/ws
```

停止时在终端按 `Ctrl+C`。

### 4.2 启动地面站和仿真

```bash
bash start_ground_all.sh
```

如果希望通讯模块启动但不自动连接：

```bash
UPSTREAM_WS_AUTO_CONNECT=0 bash start_ground_all.sh
```

然后执行：

1. 点击“上位机通讯面板”。
2. 填写 WebSocket URL 和无人机编号。
3. 点击“连接”或“重启连接”。
4. 确认状态为“已连接”，控制主题为 `drone/UAV01001/command`。
5. 点击“启动本地仿真”，等待飞行控制链路就绪。

### 4.3 启动 Reqable

1. 新建 WebSocket 请求。
2. 输入 `ws://127.0.0.1:8581/ws`。
3. 点击连接。
4. 等待服务端发送 `SYSTEM`。
5. 订阅状态主题。
6. 发布控制命令。

使用真实服务时，替换为真实 `ws://` 或 `wss://` URL，包括端口和路径。

## 5. WebSocket 握手

Reqable 和地面站分别建立自己的 WebSocket 连接。每个客户端连接后都应先收到：

```json
{
  "type": "SYSTEM",
  "msg": "Connected successfully"
}
```

地面站随后自动订阅控制主题：

```json
{
  "type": "SUBSCRIBE",
  "topic": "drone/UAV01001/command"
}
```

Reqable需要手动订阅状态主题：

```json
{
  "type": "SUBSCRIBE",
  "topic": "drone/UAV01001/status"
}
```

正确订阅后服务端返回：

```json
{
  "type": "SUB_ACK",
  "topic": "drone/UAV01001/status"
}
```

## 6. Reqable发送控制命令

### 6.1 通用格式

业务命令不能直接作为 WebSocket 顶层 JSON，必须放在 JAR 的 `PUBLISH.data` 中：

```json
{
  "type": "PUBLISH",
  "topic": "drone/UAV01001/command",
  "data": {
    "clientNo": "UAV01001",
    "commandNo": "05"
  }
}
```

JAR 转发给地面站时变为：

```json
{
  "type": "BROADCAST",
  "topic": "drone/UAV01001/command",
  "sender": "服务端分配的会话ID",
  "data": {
    "clientNo": "UAV01001",
    "commandNo": "05"
  }
}
```

### 6.2 命令表

| `commandNo` | 含义 | 地面站动作 | 必要条件 |
|---|---|---|---|
| `01` | 起飞 | 复用地面站起飞入口 | 仿真或实机会话就绪、当前未起飞 |
| `02` | 巡检任务下发 | 原子替换 GUI 航点列表 | 活动环境允许编辑航点 |
| `03` | 执行巡检任务 | 发送并执行当前 GUI 航点 | 已起飞、航点非空、无运行中任务 |
| `05` | 一键返航 | 覆盖当前任务并飞往 `(0,0,起飞高度)` | 已起飞、控制链路就绪 |
| `06` | 降落 | 复用普通 LAND | 已起飞且未处于 LAND |
| `07` | 紧急停机 | 映射为原地 LAND | 已起飞且未处于 LAND |

除命令 `02` 外，其他命令当前都不带 `taskPoints`。

### 6.3 命令 02 完整示例

```json
{
  "type": "PUBLISH",
  "topic": "drone/UAV01001/command",
  "data": {
    "clientNo": "UAV01001",
    "commandNo": "02",
    "taskPoints": [
      {
        "index": 1,
        "x": 1.0,
        "y": 2.0,
        "z": 1.2,
        "forwardAngle": 90,
        "cameraAngle": 1495,
        "photoNo": 1
      },
      {
        "index": 2,
        "x": 2.0,
        "y": 0.0,
        "z": 1.2,
        "forwardAngle": 180,
        "cameraAngle": 1495,
        "photoNo": 2
      }
    ]
  }
}
```

字段要求：

- `clientNo` 必须与地面站通讯面板中的无人机编号完全一致。
- `commandNo` 当前必须为两位字符串，例如 `"02"`，不是数字 `2`。
- `taskPoints` 必须是非空数组。
- `index` 必须是不重复的正整数。
- `x/y/z/forwardAngle/cameraAngle` 必须是 JSON 数值。
- `photoNo` 必须是整数。
- `z` 必须位于 `[0.1, 50] m`。
- `forwardAngle` 单位为度，地面站会转换为弧度偏航角。
- `cameraAngle` 和 `photoNo` 当前校验后忽略。

## 7. Reqable接收确认和状态

Reqable必须已经订阅：

```text
drone/UAV01001/status
```

### 7.1 命令接收确认

地面站发布：

```json
{
  "type": "PUBLISH",
  "topic": "drone/UAV01001/status",
  "data": {
    "clientNo": "UAV01001",
    "commandNo": "02"
  }
}
```

Reqable实际收到 JAR 转发的：

```json
{
  "type": "BROADCAST",
  "topic": "drone/UAV01001/status",
  "sender": "地面站对应的会话ID",
  "data": {
    "clientNo": "UAV01001",
    "commandNo": "02"
  }
}
```

命令确认只表示地面站已经收到并解析了合法业务命令，不等于飞行动作已经执行。若不满足本地执行条件，地面站会在 GUI 和人类日志中明确拒绝。

### 7.2 状态示例

```json
{
  "type": "BROADCAST",
  "topic": "drone/UAV01001/status",
  "sender": "地面站对应的会话ID",
  "data": {
    "clientNo": "UAV01001",
    "uavStatus": "03"
  }
}
```

| `uavStatus` | 当前含义 |
|---|---|
| `01` | 当前阶段不发送 |
| `02` | GUI 已成功接收并替换航点 |
| `03` | 巡检任务开始 |
| `05` | 返航开始 |
| `07` | 降落开始 |
| `08` | 航点任务可靠完成 |
| `09` | 到达一个航点 |
| `0A` | 1 秒周期电量；仿真为百分比，实机为电压 |
| `0B` | 1 秒周期 XYZ 位置 |
| `0C` | 低电量告警；当前只上报，不自动返航 |

## 8. 测试顺序

只在本地仿真中执行：

1. 连接 JAR、地面站和 Reqable。
2. Reqable订阅状态主题。
3. 地面站启动本地仿真并等待控制链路就绪。
4. Reqable发送 `02`，检查 GUI 航点列表和返回状态 02。
5. Reqable发送 `01`，检查仿真起飞。
6. Reqable发送 `03`，检查状态 03、09、08 和 GUI 进度。
7. Reqable发送 `05`，检查返航航点、状态 05 和 08。
8. Reqable发送 `07`，检查状态 07、LAND 和最终解除武装。
9. 关闭地面站仿真。
10. 在 JAR 终端按 `Ctrl+C`。

命令 06 与 07 共用同一 LAND 执行链，可在另一轮仿真中单独验证，不需要在同一次降落后重复发送。

## 9. 故障判断

现场同时打开：

- 地面站“上位机通讯面板 → 原始报文”；
- 地面站主日志；
- Reqable WebSocket 消息列表；
- JAR 启动终端。

| 现象 | 判断 | 优先检查 |
|---|---|---|
| `Connection refused` | IP/端口没有服务 | JAR 是否启动、IP、端口、防火墙 |
| HTTP 403/404 | 路径或鉴权错误 | 完整 URL、Token、反向代理路径 |
| WebSocket 已连接但没有 `SYSTEM` | 服务端握手与当前 JAR 协议不同 | 第一条原始帧、现场协议说明 |
| 有 `SYSTEM`，没有 `SUB_ACK` | 订阅信封或 topic 不一致 | `type/topic/clientNo` |
| Reqable发出命令，地面站无 RX | JAR 未路由到控制主题 | Reqable发送 topic、地面站订阅 topic |
| 地面站有 `BROADCAST`，但提示拒绝无效报文 | `data` 业务字段不兼容 | 字段名、大小写、字符串/数字类型、数组层级 |
| 有命令确认，但 GUI 不动作 | 地面站业务门控拒绝 | 是否启动仿真、是否起飞、控制权、航点和任务状态 |
| GUI 已动作，但 Reqable无状态 | Reqable没有订阅正确状态主题 | `drone/UAV01001/status` 和 `SUB_ACK` |
| 02 成功但 03 被拒绝 | 无人机尚未起飞或任务条件不满足 | 先完成 01、检查主日志 |
| 只有原始帧、主日志没有完整 JSON | 正常设计 | 精确 JSON 只在通讯面板显示 |

## 10. 格式变化

| 变化 | 修改位置 |
|---|---|
| 只改 URL、端口、路径、无人机编号 | 不改代码，在通讯面板修改后点击“重启连接” |
| 控制/状态 topic 改名 | [`upstream/protocol.py`](../ground_station_core/upstream/protocol.py) |
| `SYSTEM/SUBSCRIBE/SUB_ACK` 握手变化 | [`upstream/service.py`](../ground_station_core/upstream/service.py) 的 `_run_session()` |
| `PUBLISH/BROADCAST/data` 信封变化 | [`upstream/service.py`](../ground_station_core/upstream/service.py) 的 `_reader()` |
| `taskPoints` 或航点字段变化 | [`upstream/mapping.py`](../ground_station_core/upstream/mapping.py) 的 `parse_command()`、`_parse_task_points()` |
| 命令编号改变 | [`upstream/mapping.py`](../ground_station_core/upstream/mapping.py) 的 `COMMAND_MAPPINGS` |
| 增加新动作语义 | [`upstream/models.py`](../ground_station_core/upstream/models.py) 和 [`qt_ui/main_window.py`](../ground_station_core/qt_ui/main_window.py) |
| ACK、状态 JSON 改变 | [`upstream/protocol.py`](../ground_station_core/upstream/protocol.py) |
| 状态触发规则改变 | [`upstream/status_projector.py`](../ground_station_core/upstream/status_projector.py) |
| 新增鉴权 Header、Token 或 TLS 特殊配置 | [`upstream/service.py`](../ground_station_core/upstream/service.py) 的 `websockets.connect()` |

如果只是字段别名变化，应优先在 `mapping.py` 做兼容归一化，不要把协议字段散落到地面站主体

## 11. 修改格式后的最小回归

先把现场收到的完整原始帧加入测试，再运行：

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
.venv/bin/python -m pytest -q \
  tests/test_upstream_communication.py \
  tests/test_qt_gui.py
```

重点确认：

- 原始 JSON 能解析为正确的 `UpstreamCommand`；
- 无效字段仍会在进入 ROS 前被拒绝；
- 02 会同步 GUI 航点；
- 03/05 在未起飞时仍拒绝；
- 状态 topic 和返回字段没有被兼容修改破坏。

