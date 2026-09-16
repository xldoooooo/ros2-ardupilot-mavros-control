# correction_service 独立启停入口实施简报

## 目标与结果

为独立 `odin-correction.service` 增加与现有飞控、视频入口一致的根目录脚本，并同步本地与
新飞机。已完成：

- `start_onboard_correction.sh`：前台启动默认 idle 的 `correction_node`，读取现场环境文件，
  核对 ROS/工作区/相机 overlay/配置，并拒绝重复节点。
- `stop_onboard_correction.sh`：停止 `odin-correction.service`，并用 `SIGINT -> SIGTERM ->
  SIGKILL` 分级清理仅属于修正服务的手工启动进程及子进程。
- systemd unit 改为直接调用根目录启动脚本，systemd 与人工启动不再维护两套命令。
- 机载 sparse checkout、布署文档、修正服务 README 和回归测试已同步更新。

两个脚本均不启停 Odin、extnav、MAVROS、`onboard_control` 或视频服务，不包含解锁、起飞、
模式或轨迹命令。停止 correction_service 不会清除 extnav 已应用的 active correction，脚本
终态会明确提示这一点。

## 文件与开机自启位置

- 根目录入口：`start_onboard_correction.sh`、`stop_onboard_correction.sh`。
- unit 模板：`correction_service/deploy/correction-service.service.example`。
- 新飞机实际 unit：`/etc/systemd/system/odin-correction.service`。
- 安装器：`correction_service/deploy/install_correction_service.sh`。

## 本地验证

- `bash -n` 通过：新增启停脚本、修正安装器、机载工作区脚本。
- 修正/部署定向测试：`24 passed in 0.54s`。
- 完整测试：`249 passed in 92.44s`，使用 domain 231、localhost-only 与 offscreen Qt 隔离。
- 本机未安装 `shellcheck`；未伪造该项通过，由 `bash -n`、静态边界断言和运行验证覆盖。
- 本任务文件 `git diff --check` 通过；仓库中用户先有 `README.md` 修改存在一处行尾空白，
  本任务未修改或代为清理该文件。

## 新飞机同步与验收

平台为 `drone-new`，工作区为 `/home/nvidia/ros2-ardupilot-mavros-control`。同步前确认：

- `ros2-ardupilot-onboard.service` 仍为用户手动停止后的 failed/inactive 状态，本任务未启动它。
- 修正服务为 idle、无活动 job、窗口为空、`resources_released=true`。
- 远端保留历史选择性部署工作树，本次未使用 `git pull/reset/clean`，而是精确备份和逐文件同步。

覆盖前备份：

```text
/home/nvidia/backups/correction-entrypoints-20260916-lcuVvK/files-before.tar.gz
SHA-256: 4212307f473ae079a6fd7941c1e092a6eb068a7f93dfbba8d886b8452e03870b
/home/nvidia/backups/correction-entrypoints-20260916-lcuVvK/odin-correction.service.before
SHA-256: 46e23cbd99d570075f44aaa5d763837707a3073947bb7cdf3b2da8ba8a33a989
```

远端验收结果：

- 9 个同步文件的 SHA-256 与本地逐项一致。
- `systemd-analyze verify` 通过新 unit；另外报告两个 NVIDIA 系统 unit 使用旧
  `StandardOutput=syslog`，与本任务无关。
- `odin-correction.service` 为 `enabled + active`，`ExecStart` 已指向
  `/home/nvidia/ros2-ardupilot-mavros-control/start_onboard_correction.sh`。
- 日志确认修正节点 2.0 启动后保持 idle、相机关闭；`/dev/video2` 无占用者。
- 运行中再调用启动脚本会以 `already running` 拒绝，原 unit 保持 active。
- 新停止脚本已实际停止旧 unit 并确认零残留；随后只重启独立修正 unit。

部署时首次用随机后缀临时文件执行 `systemd-analyze verify` 被 systemd 以非标准 `.service`
文件名拒绝；当时新 unit 尚未覆盖。改用临时目录中标准名称后验证和安装均成功。

## 终态与未改变边界

- 新飞机 correction_service 开机自启已启用，当前 active/idle。
- 飞控总服务仍未启动；本次未启停 Odin、extnav、MAVROS、onboard_control 或视频服务。
- 未发送飞行、模式、解锁或起飞命令，未修改飞控参数。
- 本次只改启停/部署入口，没有改校准数学、配置、接口版本或运行包代码。
