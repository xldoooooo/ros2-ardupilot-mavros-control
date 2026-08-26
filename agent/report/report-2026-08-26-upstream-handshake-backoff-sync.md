# 上位机握手递增超时与双机同步报告

本文记录 2026-08-26 在 `scq@192.168.112.101` 整理未提交修改、实现上位机握手超时退避并同步
开发机与 GitHub `main` 的结果。

## 修改结论

- 保留上位机握手超时由原 5 秒提高到 15 秒的意图，并改为连续失败递增：
  `15 → 20 → 25 → 30 → 30` 秒；
- 超时覆盖 WebSocket 连接、等待 SYSTEM 首帧和等待 SUB_ACK 三个握手阶段；
- 成功订阅、用户主动断开或更换连接后，下一次握手恢复从 15 秒开始；
- `DEFAULT_JPG_PATH` 按用户要求改为 `/home/share/jpg`；
- 上位机状态 08 的视频占位路径仍为 `/home/share/test.mp4`。

代码提交：`d543e76 fix: back off upstream handshake timeout`。

## 101 工作区清理

检查时发现三个修改和一个未跟踪日志。按用户指定范围处理：

- 保留并完善 `ground_station_core/upstream/service.py`；
- 保留 `ground_station_core/upstream/status_projector.py` 的 JPG 路径修改；
- 恢复 `dev/integration/web_demo_2/websocket_client2.py`，取消其 `open_timeout=10` 修改；
- 删除 `dev/integration/debug/20260825log.txt`，并移除空的 debug 目录。

日志删除优先使用目标机的 `gio trash`，没有对其他调试文件或目录执行递归删除。

## 验证

- `tests/test_upstream_communication.py`：15 passed；
- 新测试通过真实监督循环记录连续尝试值，确认 `15/20/25/30/30`，并模拟成功握手后下一次恢复
  `15` 秒；
- 本机和 101 变基后的同一提交均完成专项测试；
- 本次 Python 范围 Ruff 检查通过（忽略两个文件既有的 import 排序问题）；
- `git diff --check` 通过。

## 主线整合与保护

处理期间 GitHub `main` 被另一项 Windows 摄像头文档工作推进到 `a66425f`。没有覆盖或回退该提交；
101 的握手修改先变基到最新主线，再以快进方式推送为 `d543e76`。

本机原有未提交的 TODO、Windows 摄像头教程和 `docs/assets/` 修改均保持未提交状态，没有被本任务
暂存或覆盖。101 上此前保存的 TODO stash 和备份分支也继续保留。

## 安全边界

本任务只修改开发机的上位机 WebSocket 客户端代码和仓库状态，没有连接飞机、解锁、起飞或发送
任何飞行命令。
