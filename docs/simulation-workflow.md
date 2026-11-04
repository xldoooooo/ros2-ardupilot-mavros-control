<!-- V2.2 上位机接入与地面站独立 SITL 媒体模拟的使用、维护入口。 -->
# 上位机仿真巡检（V2.2）

地面站复用实机的起飞、航点、降落和任务结果流程；仅在 SITL 会话中使用样例媒体。无需启动飞机或修改机载服务。

## 使用

1. 启动地面媒体服务 `ground-media.service` 和地面站，配置上位机 WebSocket 地址。
2. **手动点击“初始化仿真”**。起飞按钮可用后，上报 `01` 待机；未启动仿真且未连接飞机时，忽略控制报文，不发送接收确认。
3. 上位机先发送 `02` 航点，再发送 `03`，两条命令的业务对象均带 `type: 1`。模式不匹配的命令也会被忽略，不会启动或切换飞行环境。
4. 自动执行起飞、直线巡检、末点降落。仿真忽略避障策略选择，不触发低电量返航或迫降。
5. 降落交付 `08`，原有巡检待机延时（默认60秒）结束且起飞按钮重新可用后，上报 `01`，可继续下一轮，无需重启 SITL。仿真不要求落点位于机库，实机机库门控保持不变。

`05` 和 `07` 均沿用原地降落，不实现返航；实机仍使用原有确认/倒计时和飞行门控。实机 `0A.uavPower` 保留电压（V），仿真使用百分比。

## 消息格式

外层主题信封保持 `SUBSCRIBE` / `PUBLISH` / `BROADCAST`；数值 `type` 位于内层业务对象，不替换外层字符串 `type`。

```json
{
  "type": "BROADCAST",
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

- 所有确认和状态业务对象包含数值 `type`（实机0、仿真1）及标准 Unix 毫秒 `timestamp`。
- 接收确认保留 `commandNo`，状态使用 `uavStatus`，二者回传对应命令的 `id`。`08/09` 始终关联本轮 `03` UUID，延迟发送或切换模式不改变归属。
- `01/0A/0B/0C` 主动状态不带命令 `id`。旧工具可省略 `id/timestamp`；省略 `type` 按实机0，不可控制仿真。
- `01` 单独起飞、`06` 降落扩展命令保留。`02.taskPoints` 字段与单位不变，`cameraAngle` 仍忽略；完整示例见地面站“上位机通讯面板”。

## 媒体与配置

默认从地面媒体配置 `config/ground/media.json` 的 `root/public` 读取资源：

- 视频：`~/Media/ardupilot/public/examples/videos`
- 图片：`~/Media/ardupilot/public/examples/pictures`

起飞后由独立后台线程随机洗牌循环播放视频，通过 FFmpeg 发布到**现有** MediaMTX 的 `rtsp://127.0.0.1:8554/uav002/rectified`，不去畸变。H.264直接复制，其他编码转为H.264；片段切换会重新建立发布，拉流端应允许短暂断流并重连。落地或关闭仿真后仅停止本模块自己的发布进程。

mini-ground 的外部入口与 uav002 实机一致：`rtsp://192.168.1.111:8554/uav002/rectified`；测试媒体通过既有匿名只读 FTP `192.168.1.111:2121` 下载。实际主机地址以现场配置为准。不要让另一地面站或真实飞机同时占用同一路径，现有服务会拒绝覆盖正在发布的流。

真实到点后，`09.pointPic` 随机引用 `/examples/pictures/` 下的一张现有图片。可靠落地后，`08.videoPath` 随机引用 `/examples/videos/` 下的一个文件，`JPGPath` 为图片目录。路径均相对于 FTP 虚拟根，不发送本机家目录。缺失资源会记录告警并返回空路径，不冒充已生成媒体。

可复制 `config/ground/simulation_media.example.conf` 为 `config/ground/simulation_media.json`，按需修改 `rtsp_url`、`videos_dir`、`pictures_dir` 或 `root` / `public_root`。默认不用复制模板；本机 JSON 不入 Git，重启地面站加载。依赖复用现有 FFmpeg/ffprobe、地面媒体服务，无新增安装步骤。

## 维护入口

- 协议校验与示例：`ground_station_core/upstream/mapping.py`、`protocol.py`。
- 接收模式过滤及媒体启停：`upstream/service.py`；Qt 二次模式核对和巡检序列：`qt_ui/main_window.py`。
- 命令身份、航点与落地状态：`upstream/status_projector.py`。
- 仿真样例媒体：`upstream/simulation_media.py`，不调用机载视频或去畸变服务。
- SITL电池保护参数仅由 `ground_station_core/environment.py` 的本地启动命令设置，不修改实机飞控参数或控制包。
