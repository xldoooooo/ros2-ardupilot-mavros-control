<!-- new 机载公共代码、逐机配置及 systemd 入口同步结果；含原生验证和未覆盖项。 -->
# new 飞机机载代码与配置同步

## 检查结论

- 目标别名 `drone-new`，本次实际地址 `192.168.112.169`，Ubuntu 24.04/Jazzy/ARM64。
- 确认漏同步：Git HEAD 仍为 `6a40713`，叠加现场未提交/未跟踪部署文件；三个 unit 仍调用根目录旧入口。
  缺少后续 Shell/构建入口迁移、时间同步检查、UVC 相机采集与 AprilTag 检测修复等公共更新。
- 开发机与 origin/main 操作前均为 `7f91a1b`。refresh (`192.168.112.186`) SSH 返回 No route to host，
  本次未操作 refresh；以当前 main、new 实际文件及既有 refresh 部署报告核对差异。
- 两机硬件配置不同：new 为 Wasintek/Tag0 0.170 m/MAVROS 2.14.0；仓库默认 UQ212/Tag0 0.099 m
  来自 refresh，不能将默认配置原样套到 new。

## 已完成同步

- 使用本地 main Git bundle 建立新的干净 non-cone sparse checkout，工作区绝对路径保持
  `/home/nvidia/ros2-ardupilot-mavros-control`，origin 保持项目 GitHub 地址，仍使用 main。
- 检出公共飞行接口、控制器、修正接口/服务、视频服务及 `scripts/onboard`、`scripts/lib`，
  当前实际检出134个受控文件。旧工作树和历史整体保留，不执行 reset/clean、不删除现场标定。
- 保留机载原生 `.venv/build/install/log`，在原路径重新编译，未复制开发机安装产物。
- 三个 systemd unit 仅迁移 ExecStart 到 `scripts/onboard/`，保留原环境、逐机 overlay、设备及自启设置，
  完成 systemd-analyze verify 和 daemon-reload。无关 NVIDIA unit 的既有 syslog 弃用提示未修改。
- new 活动修正配置保存于 `/etc/ros2-ardupilot/correction-config`，由 correction.env 显式选择。
  原内参、Task32 外参、镜头参数和 Tag 世界配置逐字节保留；仅采集入口升级为本包 UVC 驱动，
  日志目录调整为原项目 correction_service/log 的绝对路径。相机仍为1920×1080@30。
- `/etc/ros2-ardupilot/onboard.env`、视频 camera.conf/lens.conf 原样保留。
- MAVROS 时间检查和补丁安装器已同步；new 系统为2.14.0，未启用仅支持2.15.1的参数补拉补丁，
  未升级系统包。此差异是版本适配要求，不是漏同步。
- extnav 源码与实际 overlay 导入文件均与受控补丁一致，SHA-256：
  `b15b05a3ba61d23e34a9075c3147f6ef2e33afb87081d0fa013c610d230753d1`，无需重复覆盖/重建。
- 独立避障工作区未改动；其 new 已同步、refresh 待同步状态仍按原 MEMORY 记录。

## 备份与验证

- 机载备份根目录：`/home/nvidia/ros2-ardupilot-maintenance/new-sync-20261008-1920/`。
  `workspace-before/` 保存原源码/Git/现场材料；原生环境和构建目录已迁回运行路径。
  完整操作前工作区另有 project-before.tar.gz，配置和三个 unit 有 etc-before.tar.gz。
- 两个归档 SHA-256 校验通过：
  - project-before.tar.gz：`0d3b2f8f45b5362575dc594b8de1a4d9e57fda9a7ac3b51fc32ead28a37bdb55`。
  - etc-before.tar.gz：`b2eb7de83bc43fb15af4454415a3011c663dba573cea0c5be2c3e1048092ecf1`。
- 四包 ARM64 原生 Release 构建通过；source/install 飞行版本3.3.0、修正版本2.0.0一致。
- 机载 colcon 测试24项：23通过、1失败、0 error、0 skipped。失败为既有包配置断言：
  仓库默认 Tag0 尺寸0.099 m，test_aircraft_configuration_loads_for_expected_tag仍期待0.170 m。
  未为通过测试修改实际配置或断言。new 活动逐机配置另行验证为0.170 m，四项标定/Tag文件与部署前一致。
- `--verify` 因上述测试失败退出，随后独立执行 localhost/domain231 smoke，通过：
  interface=3.3、fcu_connected=false、armed=false、setpoint_messages=0；隔离测试进程已退出。
- 地面项目 `.venv` 部署/时间同步/修正部署/Tag方向专项测试38项通过。
- 修正服务恢复 enabled/active，真实状态消息回报interface2.0、idle、window_count=0、
  resources_released=true、last_error为空，未启动相机或发起采样/应用。
- 机载构建/测试与smoke日志保存在备份目录 verify.log、smoke.log；本地过程材料在
  `agent/codex/new-sync-20261008/`。源码工作树检查干净，报告/记忆随后随 main 同步。

## 未覆盖项与最终状态

- 当前无 `/dev/v4l/by-id`、`/dev/v4l/by-path`，未完成真实摄像头取帧/新UVC驱动台架验收。
- 未启动真实飞控栈，未验收new上的真实FCU时间检查、热重启或完整硬件READY。
- 飞控服务保持操作前 failed/无进程状态：日志表明其18:04被停止，旧入口退出130，
  不是本次同步引起的新故障；视频保持inactive，修正服务短暂停止后恢复。三个服务的enabled设置未改。
- 全程未发送飞行、解锁、起飞、飞控重启或修正应用命令；不能将本次隔离结果当作实飞验收。
