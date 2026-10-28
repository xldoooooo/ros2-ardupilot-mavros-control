<!-- 低电压起飞门控的部署范围、验证和现场状态。 -->
# mini-ground 低电压起飞门控同步

日期：2026-10-28；功能提交：`f321f43`。

- mini-ground（`intel@192.168.1.111`）的main已从8253ed7快进至功能提交，4个修改源码文件的SHA-256与开发机一致；未引入依赖或构建步骤。
- 原地面站在同步期间已由操作者安全关闭。新版由 `ground-station-voltage-20261028.service` 用户临时服务重新打开，GUI PID为127934，启动日志确认Jazzy及ROS domain 0，服务active。自动连接飞机保持false，GPS原点配置保持原值。
- mini本机执行 `.venv/bin/python -m pytest -q tests/flight/test_takeoff_voltage.py`：9项通过，1.16秒；测试使用Qt/ROS替身，没有发送真实起飞指令。开发机此前57项相关回归通过。
- mini原MEMORY、暂存差异和工作区差异已备份至 `/home/intel/backups/low-voltage-takeoff-20261028-123043/`。快进前后核对原暂存差异和其他工作区差异完全一致，本机配置和已有修改保留。
- refresh（`nvidia@192.168.1.186`）仅被动订阅机载状态，确认未解锁、落地、控制器未活动、电压约23.852 V、无有效控制租约。其稀疏检出不运行地面站，本次没有机载代码改动，故无需同步功能文件、重建或重启；其main仍为8253ed7。机载飞控和避障服务保持active。

未连接mini的实机控制会话、未解锁或起飞飞机；未在真实低电压电池上触发门控，现场低电压行为尚未实测。hp/new尚未同步本项地面源码，已更新MEMORY中的部署差异。
