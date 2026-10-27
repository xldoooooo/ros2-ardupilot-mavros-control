<!-- 无人值守启动、独立服务与有界事件日志的部署和维护入口。 -->
# 启动与日志

## 地面站

顶栏“开机自启”安装当前用户的 `~/.config/autostart/ros2-ardupilot-ground-station.desktop`。
它在 Ubuntu 桌面登录后运行项目启动脚本；无人值守开机需系统配置桌面自动登录。
应用菜单入口由 `scripts/ground/setup_ground_station.sh` 安装，也可单独运行
`.venv/bin/python -m ground_station_core.startup` 更新，不会启动飞机。

“自动连接飞机”及已确认的 GPS 原点保存到本机 `config/ground/startup.json`。
程序启动、首次连接失败和飞机重新上线时均自动尝试正式实机连接；不会自动解锁、起飞或恢复旧航点任务。
手动断开会关闭此选项；本地仿真运行时不切到实机。飞机重启后仅在未解锁时补齐原点和遥测配置。
已打开的实机 RViz 会在掉线恢复后重开，清除飞机重新上电造成的设备时间缓存；不会主动打开原本未用的窗口。
配置恢复失败会显示警告，手动功能仍可用。

上位机通讯面板保存 URL 和编号到 `~/.config/ros2-ardupilot-ground-station/upstream.json`，
既有功能默认启动连接、断线重试。交付时应将 URL 设置为实际上位机地址。

## 飞机

保留飞控、避障、视频、Tag 四个独立服务。构建本机原生产物后，安装避障服务：

```bash
bash scripts/onboard/stop_avoidance.sh
./src/onboard_control/deploy/install_avoidance_service.sh
sudo systemctl enable ros2-ardupilot-onboard.service video-service.service
sudo systemctl disable --now odin-correction.service
```

安装器使用已构建的主项目和独立规划器；`--install-only` 仅安装，不启用或启动。
避障任一组件退出会结束该组，systemd 十秒后重启；主动停止服务不会自动重启。
`stop_avoidance.sh` 同时停止对应 unit，避免清理进程后被监督器再次拉起。
各服务故障互不绑定，不更改飞控解锁或起飞逻辑。

默认重定位在 `config/aircraft/<编号>/navigation.env` 设置 `NAVIGATION_MODE=map`，
地图路径在同档案 `odin/reloc.yaml`；不写入系统环境。更新后重启飞控服务生效。
地图应为本机已保存的有效 `.bin`，其他飞机不得直接套用路径。

## 故障复盘

地面 `log/ground_station.log` 记录启动配置、连接/租约、上位机命令和任务结果；
达到 20 MiB 自动轮转，保留五个历史文件。预览重复刷新不再逐帧写相同日志。
机载只在 FCU 连接/解锁/模式变化、离散命令结果和航点抓拍时补充身份信息；不逐控制周期落盘。
媒体记录会话起止、抓拍、上传确认和照片处理结果，可按 source、mission、waypoint 关联。

飞机若仅有易失 journal，可安装项目提供的有界持久化设置：

```bash
sudo install -D -m 0644 system/journald/ros2-ardupilot.conf /etc/systemd/journald.conf.d/ros2-ardupilot.conf
sudo systemctl restart systemd-journald
sudo journalctl --flush
```

该设置将系统 journal 保留在磁盘，预算 256 MiB、最长十四天；不下载或改变飞控自身日志。
排查可用 `journalctl -u ros2-ardupilot-onboard -u ros2-ardupilot-avoidance -u video-service --since today`，
地面媒体使用 `journalctl -u ground-media`。
