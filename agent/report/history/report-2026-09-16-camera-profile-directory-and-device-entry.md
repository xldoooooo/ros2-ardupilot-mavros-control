# 相机档案目录纠正与设备配置入口说明

## 结果

按用户纠正，相机档案已移动到各服务的 `config/` 下，并为当前两种相机同时建立命名目录：

- `correction_service/config/Wasintek/`
- `correction_service/config/UQ212/`
- `video_service/config/Wasintek/`
- `video_service/config/UQ212/`

根 `config/` 下的文件仍是当前活动配置，命名子目录是人工核验、恢复和换机时使用的完整档案，
不会在运行过程中自动切换。旧的服务顶层 `Wasintek/` 目录已移除。

## 档案内容

修正服务每种相机保存四份硬件相关配置：

- `camera.conf`：稳定设备路径、分辨率、帧率、格式和采集驱动；
- `intrinsics.yaml`：内参与畸变；
- `extrinsics.yaml`：相机到 Odin IMU 的外参；
- `lens.conf`：V4L2 镜头控制及写入顺序。

视频服务每种相机保存三份配置：

- `camera.conf`：设备、编码、视频模式、RTSP、存储和运行参数；
- `intrinsics.yaml`：内参与畸变档案；
- `lens.conf`：V4L2 镜头控制。

UQ212 视频档案使用已核实的 MJPEG 1920×1080@120 fps、稳定 by-id、默认镜头控制和临时理论
内参；Wasintek 档案保留更换前参数。视频服务根活动配置仍未切换，保持原配置不动。

## 当前设备配置入口

修正服务已有明确入口：

`correction_service/config/camera.conf` 的 `[camera] device`。

生产环境应优先填写 `/dev/v4l/by-id/...-video-index0`，不要依赖可能随插拔变化的
`/dev/videoN`。地面站标定面板不直接枚举飞机 USB 设备；它连接飞机上的
`correction_service`。服务收到 `first/next` 后才读取活动配置中的路径并打开设备。

换机规则：

1. 同一设备换 USB 口、by-id 不变：无需修改；
2. 同型号设备仅序列号/by-id 改变：修改根 `camera.conf` 的 `device`，重启独立修正服务；
3. 更换型号或镜头：先在 `config/<新相机名>/` 保存四份完整配置，再将核验后的四份内容复制成
   根活动配置并重启独立修正服务；
4. 单纯配置变更无需重新编译，但配置指纹会改变，旧窗口和候选不得继续使用。

没有新增自动 profile 选择或地面站远程写配置功能。原因是相机路径、内参、外参和镜头控制必须
作为同一套几何配置一起切换；只选择一个 USB 设备但静默沿用另一颗相机的几何参数会产生错误
修正。README 已写明 `v4l2-ctl` 探测与换机步骤。

## 安装与回归

`correction_service/setup.py` 已从只安装根 `config/*` 改为递归安装活动配置和所有命名相机档案，
后续新增第三个相机目录无需再修改安装清单。

- 命名档案与活动 UQ212 配置一致性测试：通过；
- 视频 UQ212 档案由现有加载器读取：通过；
- 定向测试：55 passed；
- 全部 Python 回归：258 passed；
- `colcon build --packages-select correction_service`：通过；
- 安装树确认包含根配置及两个命名档案的全部文件；
- `colcon test --packages-select correction_service`：5 passed；
- `colcon test-result --verbose`：24 tests，0 errors，0 failures，0 skipped。

## refresh 飞机同步

- `main` 功能提交 `49f2d6b` 已同步到 refresh 飞机并完成 correction 包重建安装；
- 更新前只读确认旧实例为 `idle`、窗口为空、资源已释放，随后仅停止并恢复独立修正服务；
- 安装树已确认同时包含根活动配置和 Wasintek/UQ212 两套命名档案；
- 新实例仍为 `idle`、窗口为空、`last_error` 为空，相机设备无人占用；
- correction unit 保持 active/disabled，没有改成开机自启；onboard 与 video unit 保持 inactive；
- 飞机原有未跟踪文件 `start_drone/image/cam_in_ex.txt` 未改动。

本次没有解锁或起飞飞机，没有发送飞行命令。
