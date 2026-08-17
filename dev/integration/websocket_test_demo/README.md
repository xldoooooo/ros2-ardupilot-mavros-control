# WebSocket 双端通讯模拟 Demo

本目录是任务 19 的完全独立实现。它模拟“甲方地面站 → WebSocket 通道 → 我方地面站
client”的通讯，只收发 JSON，不导入现有地面站、ROS、MAVROS 或飞控代码，也不会执行真实
解锁、起飞、返航、降落或停机。

## 给甲方现场演示（推荐）

不要用下面的快速验收入口作为主要演示。现场建议使用逐帧入口：

```bash
bash integration/websocket_test_demo/run_showcase.sh --step
```

`--step` 会在每个业务步骤后等待 Enter，便于边讲边演示。终端会持续显示：

- 谁发送给谁；
- `SUBSCRIBE/PUBLISH/BROADCAST` 动作；
- 实际主题；
- WebSocket 中发送或收到的完整 JSON（默认单行，减少滚屏）；
- JAR 分配的真实 `sender` 会话 ID；
- 1 秒遥测实测间隔和两架飞机隔离结果。

如果希望全自动播放：

```bash
bash integration/websocket_test_demo/run_showcase.sh
```

默认每个业务步骤停顿 0.8 秒；可用 `--delay 1.5` 放慢。完整讲解顺序见
[SHOWCASE.md](SHOWCASE.md)。

如需将每条 JSON 展开为多行缩进格式，追加 `--pretty-json`。

## 一键快速验收

在仓库根目录执行：

```bash
bash integration/websocket_test_demo/run_demo.sh
```

入口会在本目录创建 `.venv`，安装 `websockets`/`pytest`，依次运行协议单测并在随机本机
端口启动甲方原始 `websocket-server-1.0.0.jar`。结束时会逐项显示 PASS/FAIL，并只清理本次
启动的 JAR 进程组。

如需同时保存 JSON 结果：

```bash
bash integration/websocket_test_demo/run_demo.sh \
  --json-report integration/websocket_test_demo/results/latest.json
```

JAR 实际依赖 Java 8 的 `javax.xml.bind`。Ubuntu 24.04 首次运行前如未安装 Java 8：

```bash
sudo apt-get install openjdk-8-jre-headless
```

验收入口会自动寻找 Java 8，不更改系统默认 `java`。也可以显式指定：

```bash
bash integration/websocket_test_demo/run_demo.sh \
  --java /usr/lib/jvm/java-8-openjdk-amd64/jre/bin/java
```

## 已覆盖内容

- 甲方 JAR `/ws` 端点的 `SYSTEM` 首帧。
- `SUBSCRIBE`、`SUB_ACK`、`PUBLISH`、`BROADCAST`、`HEARTBEAT`、
  `HEARTBEAT_ACK` 和无效 JSON 的 `ERROR`。
- 命令 `02` 巡检任务下发、`03` 执行巡检、`05` 一键返航、`07` 紧急停机。
- 表 1 规定的 `{clientNo, commandNo}` 命令确认。
- 状态 `01/02/03/05/07/08/09/0A/0B/0C` 及全部 `data` 字段。
- 电量 `0A` 与位置 `0B` 的真实 1 秒周期上报。
- 精确主题、`drone/+/status` 单层通配、发布者不自收、两个 `clientNo` 互不串线。

`0C`（巡检电量不足，暂停巡检，返航充电）存在于权威 DOCX 表 2，但在随附 TXT 中遗漏；
本实现以 DOCX 为准并覆盖它。

## 协议与 JAR 的两层 JSON

甲方 JAR 是主题路由通道，业务消息不能直接作为 WebSocket 顶层帧发送。订阅命令主题：

```json
{"type":"SUBSCRIBE","topic":"drone/UAV01001/command"}
```

发布业务状态：

```json
{
  "type": "PUBLISH",
  "topic": "drone/UAV01001/status",
  "data": {"clientNo": "UAV01001", "uavStatus": "01"}
}
```

订阅方收到的 JAR 信封为：

```json
{
  "type": "BROADCAST",
  "topic": "drone/UAV01001/status",
  "sender": "JAR分配的会话ID",
  "data": {"clientNo": "UAV01001", "uavStatus": "01"}
}
```

## 单独运行我方 client

先启动甲方 JAR：

```bash
/usr/lib/jvm/java-8-openjdk-amd64/jre/bin/java \
  -jar integration/websocket-server-1.0.0.jar
```

再启动我方单飞机 client：

```bash
integration/websocket_test_demo/.venv/bin/python \
  integration/websocket_test_demo/run_client.py \
  --url ws://127.0.0.1:8581/ws \
  --client-no UAV01001
```

client 会订阅 `drone/UAV01001/command`，收到合法命令后向
`drone/UAV01001/status` 依次发布表 1 确认和对应状态，并每秒发布一次 `0A` 与 `0B`。
“紧急停机”等只会形成模拟状态日志，不会连接真实飞机。

协议文档中的生产示例 URL 是
`ws://192.168.80.xx:8501/s12-websocket/ws`，甲方测试 JAR 的实际 URL 是
`ws://<host>:8581/ws`；使用 `--url` 或验收入口的 `--server-url` 切换，不在代码中猜测生产地址。

## 目录说明

- `ws_demo/protocol.py`：协议主题、四种命令、十种状态与最小字段校验。
- `ws_demo/transport.py`：甲方 JAR 的主题信封适配。
- `ws_demo/client.py`：我方单飞机地面站 client。
- `run_client.py`：长期运行的独立 client 入口。
- `run_showcase.py` / `run_showcase.sh`：逐帧现场演示入口。
- `run_acceptance.py`：甲方侧模拟、JAR 生命周期和 25 项端到端验收。
- `tests/test_protocol.py`：23 项协议单元测试。
- `report-2026-08-12-task19-websockettest.md`：本次执行证据和限制。

## 范围与限制

本任务只验证 WebSocket 通讯。文档中的 FTP 下载和 RTSP 拉流使用占位地址，且不属于本次
WebSocket 双端链路，因此没有伪造可用性结果。按照任务假设，本实现不增加协议未定义的
序号、过期时间字段、重试、去重、鉴权或额外 ACK。
