# UQ212 相机配置迁移与 Wasintek 备份简报

## 结果

refresh 飞机的下视相机已确认更换为像视界/英龙芯 UQ212（USB `1bcf:28c4`）。本次完成：

- 在 `video_service/Wasintek/` 保存原视频服务的 `camera.conf`、`lens.conf` 和
  `intrinsics.yaml`；
- 在 `correction_service/Wasintek/` 保存原修正服务的 `camera.conf`、`lens.conf`、
  `intrinsics.yaml` 和 `extrinsics.yaml`；
- 7 份备份均与修改前 Git 版本做 INI/YAML 结构化比对，参数完全一致；运行时仍只读取各服务
  原有 `config/`，不会自动加载备份目录；
- `video_service/config/` 未改动，继续保留原活动配置；
- `correction_service/config/` 已切换为 UQ212 临时活动配置。

## UQ212 活动配置

### 外参

按任务要求继续使用上一颗 Wasintek 相机的
`success01-run_20260827_233838` 矩阵，矩阵数值未改。配置元数据已标记
`inherited_from_previous_camera_unverified`，不得解释成 UQ212 已完成外参标定。

### 理论内参

UQ212 同一型号可搭配多种镜头，USB 描述符不包含镜头视场版本。本次按所购 89°无畸变镜头的
标称角为对角视场，临时采用：

- 分辨率：1920×1080；
- 方形像素、中心主点：`cx=960`、`cy=540`；
- `fx=fy=sqrt(1920²+1080²)/(2*tan(89°/2))=1120.847311153525 px`；
- 5 个 `plumb_bob` 畸变系数全部为 0；
- `rms_reprojection_error=null`，避免为理论值伪造标定重投影误差。

配置加载器相应允许理论内参不提供 RMS；真实标定内参仍要求 RMS 为正数。

### 镜头控制

在 refresh 飞机对 UQ212 执行 `v4l2-ctl --list-ctrls-menus`，并在 1080p120 开流后逐项写入、读回
确认以下设备默认值：

`auto_exposure=3`、`white_balance_automatic=1`、`focus_automatic_continuous=0`、
`brightness=0`、`contrast=34`、`saturation=60`、`hue=0`、`gamma=120`、
`sharpness=2`、`backlight_compensation=1`、`power_line_frequency=1`、`zoom_absolute=0`。

UQ212 不提供 Wasintek 的 `gain` 控制，因此活动配置不再写入 `gain`。配置白名单已加入 UQ212
实际支持且本次使用的四个控制名。

### 视频模式与设备路径

- 稳定路径：`/dev/v4l/by-id/usb-YLX-WYZ-260812_UQ212_UQ212-video-index0`；
- MJPEG 1920×1080@120 fps；
- 设备在 1080p 下只枚举 120 fps。实测请求 30 fps 会被 V4L2恢复为 120 fps，现有 GStreamer
  直接写 30 fps caps 会 `not-negotiated`；经用户确认后直接采用原生 120 fps，未增加抽帧或驱动
  兼容层；
- 真实 UQ212 GStreamer 1080p120 管线已短时启动并正常结束。

## 文档与代码同步

- 更新两个服务 README，说明 Wasintek 备份目录不参与运行时加载；
- 更新 `MEMORY.md` 当前基线，移除 refresh 飞机仍使用旧 USB 1.2 Wasintek 的过时结论；
- 更新配置回归断言，覆盖 UQ212 路径、120 fps、零畸变、理论 RMS 和默认镜头控制；
- `camera_process.py` 仅把曝光稳定注释改为相机无关表述，没有引入额外兼容逻辑。

## 验证

- 旧配置结构化一致性：7/7 完全一致；
- 定向配置与官方 Tag 回归：23 passed；
- 全部 Python 回归（正确 build/source 自定义接口后）：254 passed；
- `colcon build --packages-select guided_interfaces correction_interfaces correction_service`：通过；
- `colcon test --packages-select correction_service`：4 passed；
- `colcon test-result --verbose`：23 tests，0 errors，0 failures，0 skipped；
- Git 差异空白检查：本任务文件无错误。

第一次未 source 当前 `install/` 的全量 pytest 出现 13 个 correction 接口导入错误和 7 个 guided
接口相关失败；完成接口构建并 source 后全量 254 项通过。另一个极端 Tag 场景曾揭示把 89°误当
水平视场会超过原 8 mm 门限；改按对角视场重算后在不放宽门限的情况下通过。

## 尚未解决的真实风险

- 89°按对角视场解释仍是理论假设，USB 信息无法证明实物镜头版本；
- 新相机内参、畸变和相对 Odin 外参均未实测标定；
- 未进行 UQ212 AprilTag 识别收敛、修正应用、移动检查点或飞行验证；
- 上述临时配置不得作为世界坐标精度或可实飞结论。

## refresh 飞机同步

- `main` 提交 `f0b706f` 已推送 GitHub；
- 飞机仓库原先停在 `b7d7560`，7 个已修改文件和 2 个未跟踪启动脚本经逐文件 SHA-256 核对，
  均与远端历史提交 `4eef06a` 完全一致；据此只校正 Git HEAD/index 后无损快进到 `f0b706f`；
- 飞机原有未跟踪文件 `start_drone/image/cam_in_ex.txt` 全程保留；
- 使用 `install_correction_service.sh --install-only` 构建并安装修正接口/服务，未自动启用或启动 unit；
- 发现此前人工启动的旧修正节点仍加载旧 Wasintek 路径。只读状态确认其 `active=false`、窗口为空、
  `resources_released=true` 后，使用项目停止脚本结束该旧节点，再对独立 unit 执行一次 `start`；
- 新实例配置指纹为 `5c3f19a...342c338c`，状态为 `idle`、窗口为空、`last_error` 为空，UQ212
  设备无人占用；unit 当前 active 但仍 disabled；
- 飞机安装态配置解析确认：UQ212 by-id、1920×1080@120、`fx=fy=1120.847311153525`、
  `rms=None`、零畸变和 12 项默认镜头控制均与仓库一致。

本次没有解锁或起飞飞机，没有发送飞行命令，也没有停止或重启 MAVROS、Odin、extnav、
onboard_control 或 video_service。

## 型号资料

- 厂商 UQ212 页面列出 1920×1080@120 fps，并说明该型号可搭配多种视场镜头：
  <https://www.seecapx.com/products/1080P-120fps-camera.html>
- UQ212 商品页列出 15°、45°、60°、89°、90°、100°、120°等不同镜头版本：
  <https://ic-item.jd.com/10226101039874.html>
