# 任务 19 WebSocket 双端通讯测试简报

日期：2026-08-12

范围：仅 `integration/websocket_test_demo/`

结论：协议单元测试 23/23、甲方原始 JAR 端到端验收 25/25，全部通过。

## 实现结果

完成了独立 Python 我方地面站 client。每个实例固定一个 `clientNo`，订阅自己的
`drone/{clientNo}/command`，向自己的 `drone/{clientNo}/status` 发布命令确认和状态。
模块不引用现有 Qt 地面站、ROS、MAVROS 或飞控代码。

甲方 JAR 经字节码与实测确认采用以下通道协议：

- WebSocket 路径 `/ws`，JAR 默认端口 `8581`。
- 首帧 `SYSTEM`，订阅 `SUBSCRIBE/SUB_ACK`，发布 `PUBLISH/BROADCAST`。
- `HEARTBEAT/HEARTBEAT_ACK`。
- 单层主题通配符 `+`。
- 发布者不会收到自己发布的广播。
- 无效 JSON 返回 `{"type":"ERROR","msg":"Invalid JSON format"}`。

业务层实现并测试了命令 `02/03/05/07`、表 1 确认、状态
`01/02/03/05/07/08/09/0A/0B/0C`，以及航点和状态 `data` 的全部文档字段。

## 权威材料核验

- `websocket-server-1.0.0.jar` SHA-256：
  `9b8125d0ce112d616a5b0d3d040442218964f4f1d35e45c5e4e4cddcba98af26`
- `无人机与上位机通信协议V2.0.docx` SHA-256：
  `f59f45bdf69d5b9b6e28b7c2837c6a37771d8769c3e4fe000b95b72c6e95b7c2`
- DOCX 已完整渲染为 4 页并逐页核对；TXT 漏掉了表 2 的 `0C`，实现按 DOCX 补入测试。
- DOCX 中个别 JSON 展示存在排版笔误（例如巡检点 `uavStatus` 引号），实现采用可解析且与
  字段语义一致的标准 JSON，不添加新字段。

## 实测证据

复现命令：

```bash
bash integration/websocket_test_demo/run_demo.sh \
  --json-report integration/websocket_test_demo/results/latest.json
```

本轮环境：Ubuntu 24.04、Python 3.12、OpenJDK 8u492。系统原有 Java 21 无法直接运行该
Java 8 时代 JAR，因为内部 Forest 依赖使用已移除的 `javax.xml.bind.JAXBException`；已安装
`openjdk-8-jre-headless`，验收程序会自动选择 Java 8，未更改系统默认 Java。

最终结果：

- 协议单元测试：`23 passed in 0.05s`。
- JAR 端到端：`25/25`。
- `0A` 实测周期：`1.002 s`。
- `0B` 实测周期：`1.002 s`。
- `UAV01001` 与 `UAV01002` 精确主题隔离：通过。
- 精确状态订阅与 `drone/+/status` 通配订阅内容一致：通过。
- JAR 在随机本机端口启动，测试结束后进程和监听端口均由精确 PID/PGID 清理。

## 与任务要求的对应关系

1. 所有代码、说明、测试和结果入口均在 `integration/websocket_test_demo/`；未修改现有地面站。
2. JAR 真实行为与原始 DOCX 为权威，TXT 只用于交叉核对。
3. 没有擅自加入协议未定义的序号、重试、去重、鉴权、过期时间字段或额外 ACK。
4. client、甲方侧模拟器、测试驱动均由 Python 实现；JAR 作为被测权威服务器原样运行。
5. `run_demo.sh` 提供一键、逐项 PASS/FAIL 的直观验收。
6. 没有触碰 `integration/` 外既有用户改动。

## 如实说明的未覆盖项

- 未连接甲方公司内网或生产 WebSocket 地址，只验证甲方提供的本地 JAR。
- 文档中的 FTP 与 RTSP 地址是占位值；它们属于 WebSocket 之外的文件/视频链路，本任务未测。
- 按任务给定的理想局域网假设，没有补做丢包、大延迟、断线重试和消息堆积压力测试。
- 全部飞行动作均为 JSON 通讯模拟；未连接实机、未申请控制权、未解锁、未起飞。
