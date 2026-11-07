<!-- Current V2.2 WebSocket routing, command metadata and media delivery contract. -->
# 上位机 WebSocket 对接（V2.2）

JAR 是 WebSocket 服务端；上位机/Reqable 与本项目地面站分别作为客户端连接它。
地面站收到命令后通过 ROS 2 高层接口调用机载服务，再由 MAVROS/ArduPilot 执行。
仿真使用方式及样例媒体见[仿真工作流](../simulation-workflow.md)。

## 1. 角色与主题

| 组件 | 订阅 | 发布 |
| --- | --- | --- |
| 上位机 / Reqable | `drone/UAV01001/status` | `drone/UAV01001/command` |
| 本项目地面站 | `drone/UAV01001/command` | `drone/UAV01001/status` |
| JAR | 按 topic 路由 | 将 `PUBLISH` 转成 `BROADCAST` |

编号以地面站“上位机通讯面板”设置为准，示例 `UAV01001` 不等于本机飞机档案 `uav002`。

## 2. 完整通讯链路

![上位机、JAR、地面站与无人机通讯链路](<./上位机-地面站-无人机 通讯链路图.png>)

控制方向为 `上位机 PUBLISH → JAR BROADCAST → 地面站校验/门控 → ROS 2 → 机载服务`；
状态方向为 `机载权威状态/任务结果 → 地面站 PUBLISH → JAR BROADCAST → 上位机`。
两个客户端不能直接互连；需要 JAR 或兼容现场服务端。

## 3. 启动顺序

本地测试先准备兼容 Java 运行时，在项目根目录执行：

```bash
java -jar dev/integration/websocket-server-1.0.0.jar
bash scripts/ground/start_ground_all.sh
```

测试 URL 为 `ws://127.0.0.1:8581/ws`；生产环境在通讯面板填写实际 `ws://`/`wss://` 地址
及无人机编号。`UPSTREAM_WS_AUTO_CONNECT=0` 可关闭启动自动连接。
连接成功后，手动初始化仿真或连接对应飞机；协议命令不自动创建/切换飞行环境。
Reqable 单独连接同一服务端并订阅状态主题。停止测试时关闭仿真并在 JAR 终端按 Ctrl+C。

## 4. 握手与信封

连接后先接收 `SYSTEM`，地面站自动发 `SUBSCRIBE` 订阅 command 主题；Reqable 手动订阅：

```json
{"type":"SUBSCRIBE","topic":"drone/UAV01001/status"}
```

等待对应 `SUB_ACK` 后再发送业务报文。命令必须放入 `PUBLISH.data`，JAR 转发时仅把外层
`type` 变为 `BROADCAST`（可附 `sender`），内层业务对象保留。外层 `type` 是字符串，
内层 `type` 是实机/仿真数值，不能混用。

## 5. 命令与元数据

```json
{
  "type": "PUBLISH",
  "topic": "drone/UAV01001/command",
  "data": {
    "id": "550e8400-e29b-41d4-a716-446655440000",
    "timestamp": 1793350657000,
    "clientNo": "UAV01001",
    "commandNo": "03",
    "type": 1
  }
}
```

- 每条新命令使用独立标准 UUID `id`，`timestamp` 为非负整数 Unix 毫秒。
- `clientNo` 与通讯面板一致，`commandNo` 为两位字符串。
- 内层 `type` 为整数0（实机）或1（仿真），布尔值无效；未连接飞行环境或模式不匹配时忽略命令，不发接收确认。
- 兼容旧工具省略 `id/timestamp`；省略 `type` 按实机0，不可控制仿真。显式字段会严格校验，但不按时间新鲜度拒绝。

| `commandNo` | 动作及条件 |
| --- | --- |
| `01` | 单独起飞，沿用当前环境就绪和起飞门控。 |
| `02` | 原子替换 GUI 航点表，要求当前允许编辑航点。 |
| `03` | 从未解锁地面状态自动起飞、巡检、末点降落；需航点非空、链路就绪且无运行中组合。 |
| `05` | 当前复用 `07` 的紧急原地 LAND，不执行返航路径。 |
| `06` | 普通 LAND。 |
| `07` | 紧急原地 LAND；沿用已起飞且未处于 LAND 的门控。 |

完整巡检为 `02 → 03 → 08`。实机保留现有三秒确认倒计时及取消入口，实际解锁、起飞由用户
手动操作。落地后的巡检延时默认60秒；仿真到期且起飞可用即可下一轮，实机待机01另需机库位置门控。

## 6. 命令02航点

```json
{
  "type": "PUBLISH",
  "topic": "drone/UAV01001/command",
  "data": {
    "id": "6ba7b810-9dad-41d1-80b4-00c04fd430c8",
    "timestamp": 1793350656000,
    "clientNo": "UAV01001",
    "commandNo": "02",
    "type": 1,
    "taskPoints": [
      {"index":1,"x":1.0,"y":2.0,"z":1.2,"forwardAngle":90,"cameraAngle":1495,"photoNo":1},
      {"index":2,"x":2.0,"y":0.0,"z":1.2,"forwardAngle":180,"cameraAngle":1495,"photoNo":2}
    ]
  }
}
```

`taskPoints` 为非空数组，`index` 为不重复正整数；`x/y/z/forwardAngle/cameraAngle` 为有限
数值，航向单位为度、内部转弧度。`cameraAngle` 当前忽略，`photoNo` 是不限制类型/递增的文件标识，
与航点索引分开保存。除02外当前命令不使用 `taskPoints`。

坐标按飞机报告的 odom/map 任务系解释，不自动改写航点值；odom Z 限制为 `[0.1,50]m`，
map 允许负高度并由机载转换后检查本地高度，详见[坐标说明](../odom-map.md)。

## 7. 接收确认与状态

合法且模式匹配的命令先返回接收确认，其 `commandNo/id/type` 对应原命令，`timestamp` 为
实际发送时刻；确认只说明解析成功，不等于动作执行成功。动作拒绝、取消及阶段失败保存在 GUI
和 `log/ground_station.log`，不新增未约定的协议消息。

```json
{
  "type":"BROADCAST",
  "topic":"drone/UAV01001/status",
  "data":{
    "clientNo":"UAV01001","commandNo":"02",
    "id":"6ba7b810-9dad-41d1-80b4-00c04fd430c8",
    "timestamp":1793350656100,"type":1
  }
}
```

状态业务对象使用 `uavStatus` 代替 `commandNo`，同样带 `timestamp/type`。
02/03及相关动作状态关联对应命令id；08/09始终关联本轮03的id，即使媒体延迟或切换环境。
01/0A/0B/0C主动状态不带命令id。旧命令省略id时，相关结果也不凭空生成命令id。

| `uavStatus` | 含义 |
| --- | --- |
| `01` | 可起飞待机；实机要求机库阈值内，仿真不要求。 |
| `02` | GUI 已成功替换航点。 |
| `03` | 巡检开始。 |
| `05` | 返航阶段状态；当前05命令按原地降落映射，不承诺返回机库。 |
| `07` | 已开始降落。 |
| `08` | 本轮媒体交付终态，成功或空路径均冻结一份结果。 |
| `09` | 巡检点照片结果。 |
| `0A` | 约1秒周期电量，`data.uavPower`：实机电压V，仿真百分比。 |
| `0B` | 约1秒周期任务坐标，`data.X/Y/Z`。 |
| `0C` | 低电量告警；实机仅提示，不自动返航或降落。 |

## 8. 串行媒体交付

08/09内层业务示例（生产发送时仍包在状态主题信封）：

```json
{"clientNo":"UAV01001","uavStatus":"08","id":"550e8400-e29b-41d4-a716-446655440000","timestamp":1793350757000,"type":1,"data":{"videoPath":"/uav002/session/video.mp4","JPGPath":"/uav002/session/"}}
```

```json
{"clientNo":"UAV01001","uavStatus":"09","id":"550e8400-e29b-41d4-a716-446655440000","timestamp":1793350707000,"type":1,"data":{"pointNo":"1","pointName":"巡检点位 1","pointPic":"/uav002/session/photo.jpg"}}
```

路径相对 FTP 虚拟根，使用现场 FTP 主机及端口下载，不发本机家目录或 HTTP URL。
地面媒体配置启用 `remote_ftp` 后，实机文件上传及路径校验成功才开放08/09；
仿真样例需预先同步到相同FTP根。公网配置及恢复方法见[视频服务](../../video_service/README.md#公网直播与-ftp)。
实机优先交付地面实时去畸变录像，可播放但缺段时交付并告警；没有可播放地面录像才请求
机载原片重建。明确失败、需人工处理或落地后交付超时（`media.json` 默认600秒）发送
`videoPath/JPGPath` 均为空字符串的08，终止本轮自动处理且不补发。正常08不等待迟到照片，
09按本轮点序独立报告；失败终止轮不补09。仿真引用公开样例，不冒充实机生成媒体。

07/08/09待发事件在同一服务对象、相同URL/编号的重连中保留；进程退出不自动恢复。
没有业务接收ACK，断线边界可能重复，不保证网络恰好一次。01依据恢复后的当前状态重新判定。

## 9. 本地仿真检查

手动初始化仿真后，发送 `type:1` 的02、03，核对航点、确认、自动任务及08/09的03 UUID归属，
通过FTP下载结果。待落地/延时结束再执行第二轮；01和05/06/07另轮检查。无环境或发 `type:0`
时应忽略且无确认。实机解锁和起飞不能由自动代理执行。

## 10. 故障与维护入口

| 现象 | 检查 |
| --- | --- |
| 连接拒绝或403/404 | 服务端是否启动，完整URL、端口、路径、鉴权及防火墙。 |
| 已连接但无SYSTEM/SUB_ACK | 握手协议和订阅topic；查看通讯面板原始帧与服务端日志。 |
| 无命令确认 | command主题、编号、内外type、活动环境及业务字段。 |
| 有确认无动作 | 主GUI日志中的就绪、权限、航点或任务门控。 |
| GUI动作但无状态 | 上位机订阅的status主题及SUB_ACK。 |

| 修改内容 | 入口（相对项目根） |
| --- | --- |
| topic、信封、确认与状态字段 | `ground_station_core/upstream/protocol.py` |
| 握手、收发、模式过滤、鉴权 | `ground_station_core/upstream/service.py` |
| 命令编号、航点校验与映射 | `ground_station_core/upstream/mapping.py` |
| 状态触发、命令归属与媒体交付 | `ground_station_core/upstream/status_projector.py` |
| GUI动作与巡检编排 | `ground_station_core/qt_ui/main_window.py` |

仅更改URL/编号时在面板重连即可；字段兼容统一放在映射层，避免散落到GUI。

## 11. 协议改动验证

将现场完整原始帧作为用例输入，运行 `bash tests/run.sh --group upstream video`。
重点检查元数据、模式过滤、航点坐标、命令归属、拒绝门控及媒体终态；本说明维护不运行飞行测试。
