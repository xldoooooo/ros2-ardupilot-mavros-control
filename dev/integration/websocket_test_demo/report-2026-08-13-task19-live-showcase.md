# 任务 19 现场演示入口追加简报

日期：2026-08-13

范围：仅 `integration/websocket_test_demo/`

## 用户反馈

原 `run_demo.sh` 偏向自动回归，主要在结束时集中显示 PASS/FAIL，不利于向甲方解释消息的
实际流转过程。

## 本次改动

- 新增 `run_showcase.sh` 和 `run_showcase.py`，逐帧显示甲方、权威 JAR、我方 client 三方通讯。
- 每帧包含方向、动作、主题、完整 WebSocket JSON 和相对时间；默认单行防止滚屏过快，
  `--pretty-json` 可展开。
- 新增 `--step` 人工讲解模式和 `--delay` 自动播放速度。
- 真实展示 `SYSTEM/SUBSCRIBE/SUB_ACK/PUBLISH/BROADCAST`，包括 JAR 返回的会话 `sender`。
- 五幕覆盖四类命令、十类状态、实际 1 秒周期遥测和 `UAV01001/UAV01002` 隔离。
- 我方 client 新增原始命令 `BROADCAST` 事件，仅用于可视化原始 JAR 信封；业务校验逻辑不变。
- 新增 `SHOWCASE.md`，提供现场话术、按幕讲解重点、外部服务器用法和日志保存命令。
- 保留原 `run_demo.sh` 作为快速自动回归，避免牺牲原有 CI 风格验收。

## 验证

执行：

```bash
bash integration/websocket_test_demo/run_showcase.sh --delay 0 --no-color
```

实测输出 5 幕、完整 4 类命令、全部 10 类状态、两轮周期遥测和双飞机隔离，正常结束。
输出包含真实 JAR 分配的 `sender` UUID；并非只打印预设结论。

追加执行快速回归、Python 编译、Shell 语法和致命级 flake8；均通过。退出后没有
`websocket-server-1.0.0.jar` 进程或 8581 监听残留。

## 安全与限制

本次仍为纯通讯模拟，未连接 ROS、MAVROS、飞控或实机，未解锁、未起飞。现场显示的返航、
降落和紧急停机只代表协议消息被解析及确认，不代表真实飞机执行。
