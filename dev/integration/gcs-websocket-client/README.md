# 地面站 WebSocket 模拟客户端

本目录提供一个完全独立的 WebSocket 客户端，用于模拟当前地面站向上位机回复协议消息。
它只收发 JSON，不导入地面站、ROS、MAVROS 或飞控代码，不会连接、解锁或起飞真实飞机。

模块遵循 `../无人机与上位机通信协议V2.0.docx`，并使用当前地面站与联调 JAR 的主题信封：

- 连接成功后订阅 `drone/{clientNo}/command`；
- 向 `drone/{clientNo}/status` 发布命令确认和状态；
- 完成 `SYSTEM → SUBSCRIBE → SUB_ACK` 后先发送待机 `01`；
- 在线期间每 1 秒发送电量 `0A` 和位置 `0B`；
- 收到巡检航线 `02` 后保存航点并回复确认、状态 `02`；
- 收到执行巡检 `03` 后模拟起飞、逐点巡检和降落，逐点发送 `09`，最后发送 `08`；
- 支持当前地面站使用的 `01` 起飞、`05` 返航、`06` 降落、`07` 紧急停机；
- 断线后自动重连，不积压断线期间的旧状态。

所有业务回复都实际发送到 WebSocket 状态主题。终端内容只是本程序自身的调试信息，不会写入
项目地面站日志。

## 1. 安装 Python

要求 Python 3.10 或更高版本。

### Windows 10/11

1. 打开 <https://www.python.org/downloads/windows/>，下载 Python 3 的 64 位安装程序。
2. 运行安装程序，先勾选 **Add python.exe to PATH**，再选择 **Install Now**。
3. 打开新的 PowerShell，确认安装：

   ```powershell
   py --version
   ```

### Ubuntu 22.04/24.04

```bash
sudo apt update
sudo apt install -y python3 python3-venv python3-pip
python3 --version
```

## 2. 一键安装依赖

先进入本目录，然后创建独立环境并安装 `requirements.txt`。

Windows PowerShell：

```powershell
cd gcs-websocket-client
py -m venv .venv
.\.venv\Scripts\Activate.ps1
python -m pip install -r requirements.txt
```

如果 PowerShell 阻止激活，可不激活环境，直接执行：

```powershell
.\.venv\Scripts\python.exe -m pip install -r requirements.txt
```

Ubuntu：

```bash
cd gcs-websocket-client
python3 -m venv .venv
./.venv/bin/python -m pip install -r requirements.txt
```

## 3. 运行

联调 JAR 的常用地址是 `ws://<服务器IP>:8581/ws`。生产地址以现场提供值为准，不要照抄协议
文档中的 `xx` 占位符。

Windows PowerShell：

```powershell
.\.venv\Scripts\python.exe .\gcs-websocket-client.py `
  --url ws://192.168.80.10:8581/ws `
  --client-no UAV01001
```

Ubuntu：

```bash
./.venv/bin/python ./gcs-websocket-client.py \
  --url ws://192.168.80.10:8581/ws \
  --client-no UAV01001
```

不传参数时，程序自动连接 `ws://127.0.0.1:8581/ws`，编号为 `UAV01001`。URL 和编号既可以
用上述参数修改，也可以直接修改 Python 文件顶部的 `DEFAULT_WEBSOCKET_URL` 和
`DEFAULT_CLIENT_NO`。按 `Ctrl+C` 停止。

## 4. 默认模拟时序

| 上位机命令 | 模拟回复与行为 |
| --- | --- |
| `01` 起飞 | 立即回复 `{clientNo, commandNo}`；5 秒后进入 1.5 m 模拟高度，后续周期 `0B` 反映新高度 |
| `02` 航线下发 | 立即回复确认，保存航点，再发送状态 `02` |
| `03` 执行巡检 | 回复确认；未起飞时等待 5 秒；发送 `03`；每 5 秒完成一航点并发送 `09`；发送 `07` 并等待 5 秒降落；发送 `08` 和 `01` |
| `05` 一键返航 | 回复确认并中断旧动作；发送 `05`；5 秒后到原点；发送 `07`；5 秒后发送 `01` |
| `06` 降落 | 回复确认并中断旧动作；发送 `07`；5 秒后发送 `01` |
| `07` 紧急停机 | 回复确认并中断旧动作；发送 `07`；5 秒后发送 `01` |

`03` 必须先收到一条有效 `02` 航线；如果没有航线，程序仍按协议回复命令接收确认，但不会伪造
巡检完成。命令 `05/06/07` 会中断正在进行的巡检，且返航不会发送巡检专用的 `08/09`。

默认媒体占位值：

- `08.data.videoPath`：`/home/share/test.mp4`
- `08.data.JPGPath`：`/home/share/jpg`
- `09.data.pointPic`：`/home/share/jpg/test.jpg`

电量低于 20% 时发送一次 `0C`；若模拟飞机已经在空中，还会中断当前任务并模拟返航。时长、
初始位置、电量和媒体路径都可修改，例如快速联调：

```bash
./.venv/bin/python ./gcs-websocket-client.py \
  --url ws://127.0.0.1:8581/ws \
  --takeoff-seconds 0.2 \
  --waypoint-seconds 0.2 \
  --landing-seconds 0.2 \
  --return-seconds 0.2 \
  --power 55.6
```

查看全部参数：

```bash
./.venv/bin/python ./gcs-websocket-client.py --help
```

## 5. 使用上位机或 Postman 调试

上位机测试端也应连接同一个 WebSocket URL。收到服务端 `SYSTEM` 后，先订阅模拟客户端的状态
主题：

```json
{"type":"SUBSCRIBE","topic":"drone/UAV01001/status"}
```

服务端返回 `SUB_ACK` 后，下发两点巡检航线：

```json
{"type":"PUBLISH","topic":"drone/UAV01001/command","data":{"clientNo":"UAV01001","commandNo":"02","taskPoints":[{"index":1,"x":1.0,"y":0.0,"z":1.5,"forwardAngle":0,"cameraAngle":1495,"photoNo":1},{"index":2,"x":2.0,"y":0.0,"z":1.5,"forwardAngle":90,"cameraAngle":1495,"photoNo":2}]}}
```

再下发执行命令：

```json
{"type":"PUBLISH","topic":"drone/UAV01001/command","data":{"clientNo":"UAV01001","commandNo":"03"}}
```

状态订阅端会收到 `BROADCAST` 信封。正常巡检的关键业务顺序是：

```text
02确认 → 02 → 03确认 → 03 → 09(点1) → 09(点2) → 07 → 08 → 01
```

`0A/0B` 会在上述过程间每秒持续出现。若要查看每个实际收发帧，追加：

```bash
--log-level DEBUG
```

常见问题：

- 一直重连：检查服务器 IP、端口、防火墙，以及 URL 路径是否确实为 `/ws`。
- 已连接但收不到命令：确认上位机发布到了同一 `clientNo` 的 `/command` 主题。
- 上位机收不到状态：确认上位机已订阅同一 `clientNo` 的 `/status` 主题。
- `03` 只有确认：先发送字段完整且合法的 `02`，再发送 `03`。
- 想缩短测试时间：使用 `--takeoff-seconds`、`--waypoint-seconds`、
  `--landing-seconds` 和 `--return-seconds`。
