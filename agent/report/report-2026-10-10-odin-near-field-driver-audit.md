<!-- Odin 近身点云干扰检查：记录实际驱动链路、公开接口边界和未验证事项；本次没有修改运行代码或设备配置。 -->
# Odin 近身点云干扰与驱动过滤可行性检查

日期：2026-10-10（Asia/Shanghai）。对象：翻新飞机 `ssh drone-refresh`，实际连接地址
`192.168.112.186`，Odin 工作区 `/home/nvidia/catkin_ws`。

## 结论

**现有公开驱动无法通过增加一个距离过滤参数，保证机体附近的点在 Odin 内部处理前被忽略，
从而消除 `/odin1/cloud_slam` 的拖点。** SLAM 位姿和 SLAM 点云已在 Odin 设备内部生成，
机载驱动接收到的是输出。修改 ROS 发布回调、过滤 `cloud_raw`，或者调整避障侧
`blind_radius`，均不会把过滤结果送回 Odin 内部算法。

驱动已提供设备端 `image_mask` 上传接口，是值得进一步确认的候选；但目前只证实 PNG
文件会上传到设备，**未证实它会排除雷达点、作用于哪一阶段，或能按三维距离屏蔽**。
不能把这个接口当成已验收方案。

厂商 FAQ 已提到 0.4 m 内明显拖点，建议近处感知/避障使用 `cloud_raw`，建图定位使用
`cloud_slam`，同时避让机器近距结构遮挡。因此优先核查安装遮挡；若必须保留安装位置，
需要厂商提供当前固件支持的算法输入过滤或雷达掩膜语义。当前用户要求的“完全忽略近身范围”
尚未实现，本次没有用输出删点替代这一要求。

## 实际环境与核验

- 实机驱动目录 `/home/nvidia/catkin_ws/src/odin_ros_driver` 没有 `.git`，不能按 Git HEAD 确认版本。
- 当前运行进程是安装目录的 `host_sdk_sample`，启动参数文件指向
  `/home/nvidia/catkin_ws/install/odin_ros_driver/share/odin_ros_driver/config/control_command.yaml`。
- 源码与安装目录的 `control_command.yaml` SHA256 一致：
  `58aaa49413dba5024734a8a0f5013d7d79d8323c6745a27ba9b35d82b00f0a3e`。
  源码与安装目录的 `include/host_sdk_sample.h` SHA256 也一致。
- 运行日志 `/home/nvidia/.ros/log/host_sdk_sample_9129_*.log` 报告：
  `ros_driver_version:0.14.0`、`soc_version:V0.14.2`、`slam_version:V0.13.1`。
  此处以实际运行日志为准，不以公开仓库最新 README 的版本代替现场版本。
- `custom_map_mode: 0`，仍输出 `cloud_slam`；模式 0 是里程计模式，不能理解成关闭内部算法。
- 当前配置：`sendcloudslam=1`、`senddtof=0`、`sendodom=1`、`sendrgb=1`、
  `cloud_raw_confidence_threshold=35`、`sendimagemask=0`、`image_mask_abs_path=""`。
- `ros2-ardupilot-onboard.service` 当时为 inactive，但用户手动运行的 Odin、RViz 和规划器进程存在。
  本次没有启停它们，没有读写 FCU、没有解锁或起飞，也没有把服务 inactive 当作已确认未解锁。

## 源码证据

以下路径均相对于实机 `/home/nvidia/catkin_ws/src/odin_ros_driver`，行号来自本次现场副本。

| 位置 | 实际行为 | 对要求的含义 |
| --- | --- | --- |
| `src/host_sdk_sample.cpp:1271` | SDK 选择 `LIDAR_MODE_SLAM` | 算法在设备侧运行 |
| `src/host_sdk_sample.cpp:1030` | 收到 `LIDAR_DT_SLAM_CLOUD` 后调用 `publishPC2XYZRGBA` | 输入已经是 SLAM 点云 |
| `include/host_sdk_sample.h:909` | 将设备整数 XYZ 换算为米，包装 RGB 并发布，坐标系为 `odom` | 这里删点只能改变输出 |
| `include/host_sdk_sample.h:693` | 用置信度阈值将主机 raw 云低置信度点的 XYZ 等字段置零 | 不改变设备内部 SLAM 输入；也不是距离阈值 |
| `include/lidar_api_type.h:333` | `lidar_depth_para_t` 只有 `odr` 帧率字段 | 这个 depth 接口不能设最小距离或 FOV |
| `src/yaml_parser.cpp:68`、`:217` | `custom_` 参数去掉前缀后经 SDK 下发设备 | 通用下发不代表固件支持任意参数名称 |
| `src/host_sdk_sample.cpp:1776` | 开流前调用 `lidar_set_image_mask` 上传掩膜 | 存在设备端入口，内部消费语义未知 |
| `src/host_sdk_sample.cpp:1906` | `senddtof` 只控制 raw 数据流发送开关 | 关闭 raw 发布不代表设备不使用雷达 |

数据链路：

```text
Odin 内部雷达/相机/IMU → 内部处理及 SLAM → USB 输出 SLAM 云和位姿
                                        → host_sdk_sample → /odin1/cloud_slam
```

公开驱动、SDK 头文件和官方 Wiki 未查到算法输入 `min_range`、`blind_radius`、
近身三维排除区域或雷达 FOV 裁剪参数。不能因此断言厂商私有固件绝不存在这类参数，
但目前没有可正确调用、可验证的名称、类型、单位和生效阶段。
`set_param.sh` 只是向 `/tmp/odin_command.txt` 写命令，现场运行驱动的命令解析使用整数值；
不应凭名称尝试 `blind` 或 `min_range`，更不能把 ACK 当成实际过滤验证。

## `image_mask` 能做什么、还缺什么

现有 YAML 入口（仅展示，**未应用**）：

```yaml
# 向设备上传掩膜；以下不是已验证的点云距离过滤配置。
register_keys:
  sendimagemask: 1
  image_mask_abs_path: "/absolute/path/to/mask.png"
```

现场配置注释示例要求 1600×1296 图像。SDK 头文件 `include/lidar_api.h:470` 起只描述
“读取并发送掩膜文件”，没有规定黑白/通道含义、图像是否为原始畸变坐标、作用于视觉特征
还是雷达点、持久化和清除方式。已从现场可执行文件中核对出 `sendimagemask`、
`image_mask_abs_path` 和上传成功/失败日志文本，确认安装程序也包含这一入口。

即使固件确实用图像掩膜剔除投影雷达点，固定二维掩膜也会遮掉同一方向的远处环境，
不能自然等同于“只忽略半径 R 内的点”。若掩膜仅用于视觉特征，则无法保证改善雷达拖点。
需要向厂商确认以下具体事项，再决定是否做现场 A/B：

1. SoC 0.14.2 / SLAM 0.13.1 是否消费该掩膜，以及格式、黑白定义、原始/去畸变坐标约定。
2. 是否同时作用于视觉跟踪、雷达预处理、匹配、地图更新和输出云，分别在哪个阶段生效。
3. 是否支持按雷达原点距离或机体区域过滤，参数名称、数据类型、单位和适用版本是什么。
4. 如何让修改立即生效，是否保留旧地图/状态，如何恢复并清除已上传掩膜。

## 视场和干扰原因

官方深度 FOV 约为水平 120°、垂直 90°；这是光学规格，当前接口没有可直接调窄它的字段。
`showcamerapose` 属于显示开关；修改相机标定、`maxIncidentAngle` 或 RViz 裁剪也不能
等同于控制设备内的雷达视场，误改标定还会影响投影几何。

截图与厂商描述的近距离拖点相符，但截图不足以确定远处点变形是在测距、内部点云处理、
位姿估计还是显示时形成。本次没有同步采集 raw/slam 云作对照，不能确认因果机制。
近处结构可能带来测距层面的混合回波等干扰；这是待验证的技术推断。
若外部障碍的原始测距已经被污染，算法前距离过滤也不保证恢复这些障碍的真实位置。

官方安装指南要求 0.2 m 内 FOV 无遮挡，并建议安装支架避免设备下方近距结构侵入视野。
对于已出现拖点的机体，建议让机臂、脚架、防撞圈等退出 TX/RX 的有效视野，再比较同场景数据。
不要直接在光学窗口前加挡板缩小视场；新增近距遮挡本身可能引入干扰。

## 可行路径及验证边界

| 路径 | 能否满足内部算法前排除 | 当前状态 |
| --- | --- | --- |
| 在避障端或驱动发布回调里删近点 | 否 | 用户明确排除，本次未实施 |
| 调高 `cloud_raw_confidence_threshold` | 否，仅改主机 raw 输出 | 当前 35，未改 |
| 设备端 `image_mask` | 有候选入口，实际作用未明 | 需厂商确认，未启用 |
| 厂商提供内部最小距离/雷达 ROI 参数或固件 | 取决于生效阶段，可针对本要求 | 目前缺接口定义及验证 |
| 调整安装使机体退出视野 | 可减少干扰源，仍需测量验证 | 优先建议；本次未改硬件 |
| raw 云过滤后运行外部 SLAM | 可控制外部算法输入 | 属于更换定位链路；仍不能撤销 raw 测距层干扰，本次未实施 |

如后续只将避障感知改用 `cloud_raw`，仍须处理每点 `offset_time`、雷达外参及实际时间对应的位姿，
不能只替换话题名称。此方案不改变 Odin 内部 SLAM，不能作为本任务目标已实现。

后续地面对照应在相同环境记录同步的 raw 云、slam 云和位姿，使用固定障碍的独立距离真值，
比较机体遮挡存在/移除、掩膜关闭/开启时障碍的距离偏差、离散度和拖点分布；
仅观察近身点消失或上传返回成功不足以通过验收。解锁和起飞只能由用户手动操作。

## 本次交付与未完成指标

- 完成：默认目标连接、现场源码与安装配置核对、运行版本日志核对、数据链路与参数作用域审查、
  官方驱动/SDK/Wiki 与公开问题记录检索。
- 未完成：内部近身过滤效果实现及 A/B 实测、`image_mask` 雷达作用确认、拖点的具体成因判定。
- 无代码、固件、启动参数、标定或硬件变更；没有执行构建、重启、参数下发或试飞。
  本次验证为源码与运行日志审查，不属于功能测试通过。
- 仅增加本检查报告并维护项目记忆；过程证据位于本地忽略目录
  `agent/codex/odin-self-cloud-20261010/`。new 未连接，两个飞机均无此次代码/配置待同步项。

## 官方资料

- [Odin1 FAQ，Q5.1–Q5.3：近距拖点及 raw/slam 区别](https://github.com/ManifoldTechLtd/wiki/blob/master/docs/odin_series/odin1/15.%20FAQ.md)
- [Odin1 安装指南：近距遮挡和 TX/RX FOV](https://github.com/ManifoldTechLtd/wiki/blob/master/docs/odin_series/odin1/3.%20Installation%20Guide_.md)
- [官方驱动 SDK：掩膜上传和设备参数接口](https://github.com/manifoldsdk/odin_ros_driver/blob/main/include/lidar_api.h)
- [官方驱动配置](https://github.com/manifoldsdk/odin_ros_driver/blob/main/config/control_command.yaml)

上述线上资料为 2026-10-10 检索到的公开版本，不能代替现场固件支持范围确认。
