<!-- 本目录及history下的任务报告检索入口；摘要保留当时结果，新增或移动报告时同步维护。 -->
# 任务报告索引

更新日期：2026-10-13。共 **206 份报告**，按月份分组、日期倒序排列；每份摘要不超过100字（含标点和英文字符）。

按日期、任务编号或关键词查找，再点击报告标题阅读相关全文。报告反映当时的版本与实验条件，当前基线请结合根目录 [MEMORY.md](../../MEMORY.md) 和实际源码核对；不必在每次任务开始时通读索引或全部报告。

范围包括本目录及 `history/` 下的执行报告、研究报告；`AGENTS.md`、索引本身及 `assets/` 附件不作为报告条目。

## 2026-10（24份）

| 日期 | 报告 | 摘要 |
| --- | --- | --- |
| 2026-10-13 | [Task2：保留原始云注释代码](report-2026-10-13-task2-retain-raw-comments.md) | 恢复9行原始云代码并注释，便于重新启用；有效源码不变，两机源码同步，无需重建或重启。 |
| 2026-10-13 | [Task2：仅发布down云](report-2026-10-13-task2-down-only.md) | 删除原始ROS云发布，几何不改；两机原生安装、refresh20帧验证通过，new现有完整链运行保持，新版待重启生效。 |
| 2026-10-13 | [Task3：extnav性能优化](report-2026-10-13-extnav-performance-opt.md) | Python热路径按需发布，两机原生部署及数值/ROS通过；同负载CPU仅降1.2%/3.3%，30%目标和全面尾部不退化未达，C++待决定。 |
| 2026-10-13 | [Task2：降采样云去除RGB](report-2026-10-13-task2-xyz-only.md) | 仅header改为XYZ/12字节输出；两机ARM与refresh100帧参考对照、new普通重连格式验证通过，几何逻辑保持。 |
| 2026-10-13 | [Task2补充：校验与auto球心修复](report-2026-10-13-task2-cloud-validation-auto-fix.md) | 严格配置与XYZ非有限校验，删除auto原点回退；两机ARM通过、refresh100帧对照及new普通连接通过，私有仓库已推送。 |
| 2026-10-13 | [Task2：近点剔除与降采样SDK集成](report-2026-10-13-task2-down-samp-exec-inte.md) | 三文件集成并同步两机；ARM边界及200帧逐字节对照通过，new冷启动恢复云；负零和重连风险保留，私有仓库授权待完成。 |
| 2026-10-12 | [恢复Task38两种策略，保持169现场避障算法](history/report-2026-10-12-task38-restore-169-planner.md) | 恢复3.4及两种策略，保留169现场搜索核心并同步两机；SITL通过，旧净空失败保留，重启后Odin混流且降采样缺失，实机联调停止。 |
| 2026-10-12 | [飞机186同步169版本交接验收](history/report-2026-10-12-飞机186同步169版本交接验收.md) | 交接186同步169旧版3.3的构建、配置保留和规划器回退；Tag尺寸及净空各有失败，未验收实机闭环，随后Task38恢复部署替代该基线。 |
| 2026-10-12 | [agent/codex 清理执行简报](report-2026-10-12-agent-codex-cleanup.md) | 清理开发机过程文件，释放854.4MiB；保留原始数据、近期证据和交付材料，3152份保留文件哈希一致，部分历史调研及构建路径已失效。 |
| 2026-10-10 | [MEMORY 精炼执行简报](history/report-2026-10-10-memory-refinement.md) | 将MEMORY由868行精炼至161行，保留当前基线、逐机差异和风险，移除流水账及软著专利记忆；仅文档核对，无新的硬件验收。 |
| 2026-10-10 | [new-1-path-delay：规划延迟的控制层适配](history/report-2026-10-10-new-1-path-delay.md) | 实现连续曲线最近点接入和前缀裁剪，修复独立Odin盲区参数优先级；DDS、SITL及ARM验证通过，正式实机桥仍无路，参数修复后来回退。 |
| 2026-10-10 | [Odin 近身点云干扰与驱动过滤可行性检查](history/report-2026-10-10-odin-near-field-driver-audit.md) | 审查Odin设备内部SLAM与主机输出链，未找到可验证的算法前近身过滤参数；掩膜接口作用待厂商确认，未改配置或实现拖点消除。 |
| 2026-10-10 | [Task38：避障启停脚本执行权限修复](history/report-2026-10-10-task38-avoidance-script-permissions.md) | 修复避障启停脚本缺少执行权限的问题，改为100755并补齐186权限；语法及只检查入口通过，未启动组件，无需重建。 |
| 2026-10-10 | [Task38：GUI本地仿真自动启动避障](history/report-2026-10-10-task38-gui-sitl-avoidance.md) | GUI本地仿真自动启动扫描、地图桥与规划服务，两种策略实际SITL通过；补齐演示和回收规则，仅开发机验证，未同步其他设备。 |
| 2026-10-09 | [extnav 与飞控最终俯仰角不同的原因](history/report-2026-10-09-extnav-ekf-pitch-difference.md) | 结合只读参数与标准EKF3源码解释外部俯仰约负19度、最终俯仰近零：外部姿态仅直接融合yaw，杆臂转换错误仍可能影响位置。 |
| 2026-10-09 | [Odin 仰角 20 度安装对坐标变换的影响评估](history/report-2026-10-09-odin-20deg-coordinate-assessment.md) | 评估Odin仰角20度的安装旋转、杆臂和速度关系；Tag完整链可继续使用，extnav零安装配置需适配，静止一致性不等于动态或绝对精度验收。 |
| 2026-10-09 | [Task38：机载避障策略集成执行报告](history/report-2026-10-09-task38-dyn-integration.md) | 接入两种避障策略、观测地图和3.4接口，六场SITL通过并部署186台架；实机目标仍无路，负载及实飞未验收，后续恢复版替换了规划核心。 |
| 2026-10-08 | [new 飞机机载代码与配置同步](history/report-2026-10-08-new-onboard-sync.md) | 同步169公共机载代码、UVC修正及启动入口，保留逐机标定；原生构建和隔离smoke通过，Tag尺寸断言仍失败，真实硬件闭环未验收。 |
| 2026-10-08 | [机载避障策略集成可行性与工作量评估](history/report-2026-10-08-obstacle-avoidance-integration-assessment.md) | 评估独立规划库接入两种避障策略，建议复用PD+DOB，估算18至30人日；坐标、时序、观测覆盖和连续净空仍待解决，未实施。 |
| 2026-10-08 | [Task37：目标投递与0.45 m策略现场输出验证](history/report-2026-10-08-task37-goal-delivery-verification.md) | 确认新规划进程未收到目标；补发同一目标后取得163次成功及162条非空Path，解释DDS投递和旧预览语义，未验收连续净空或飞行。 |
| 2026-10-08 | [Task37：现场无Path与成功预览停留诊断](history/report-2026-10-08-task37-live-no-path-diagnosis.md) | 诊断现场无Path及RViz旧曲线停留；捕获帧中原目标被占用，抬高目标的离线重放可成功，未改现场目标，不能推定此前全部失败原因。 |
| 2026-10-08 | [Task37：独立避障 ROS2 组件修正](history/report-2026-10-08-task37-refine-dyn-ros2.md) | 修复独立ROS2规划器输入校验、输出时效和有界构造，两平台回归通过；真实云抬高目标成功、同高目标失败，全链P99与飞行闭环未达成。 |
| 2026-10-08 | [Task37追加：按ROS1原项目对齐碰撞距离](history/report-2026-10-08-task37-ros1-clearance-parameter-alignment.md) | 按ROS1恢复0.45米质心严格距离判定，移除体素及采样补偿；开发机与169构建验证完成，33例中1例连续净空失败原样保留。 |
| 2026-10-08 | [Task38：机载避障集成可行性详细分析报告](history/report-2026-10-08-task38-feasibility-assessment.md) | 详细评估Task38参考替换、两种策略、恢复降落及RViz预览，估算20至35人日；专项复核重现净空失败，未实施集成或操作飞机。 |

## 2026-09（73份）

| 日期 | 报告 | 摘要 |
| --- | --- | --- |
| 2026-09-29 | [Task36 路径预览闪断与输入匹配修复](history/report-2026-09-29-task36-path-preview-sync.md) | 新增单帧等待迟到里程计、失败不清历史Path预览，new 20秒197次成功且零空Path；仍4次过期/匹配失败，执行结果照实失效，未消除现场延迟。 |
| 2026-09-24 | [Task36 可配置盲区过滤](history/report-2026-09-24-task36-blind-filter.md) | 新增可配置源时钟匹配三维球形盲区过滤并在new验证生效，默认关闭、Odin入口0.5米；测试通过但目标仍NO_PATH，球内真实障碍同样会被删除。 |
| 2026-09-24 | [Task36 搜索预算调整至300ms](history/report-2026-09-24-task36-budget-300ms.md) | 按要求将独立规划ROS配置搜索预算改300毫秒并同步new，既有成功目标返回REACH_END；原失败目标未重试，困难搜索不能保证实际10Hz，refresh未同步。 |
| 2026-09-24 | [Odin 实时规划启动配置修正](history/report-2026-09-24-task36-odin-launch.md) | 新增Odin实时launch统一odom/receive输入并放宽搜索z下界至-2米，new构建启动通过；目标仍因端点近障NO_PATH，实景成功轨迹未验收。 |
| 2026-09-23 | [Task35：避障算法 ROS2 移植与机载集成评估](history/report-2026-09-23-task35-obstacle-avoidance-assessment.md) | 评估独立ROS2规划移植及两避障策略集成有条件可行，审查上游缺陷、Odin输入与控制边界；估算完整31～49人日，仅评估未移植或测性能。 |
| 2026-09-23 | [Task35：ROS2 独立避障规划器与本项目集成实施计划](history/report-2026-09-23-task35-obstacle-avoidance-implementation-plan.md) | 制定两包独立规划器、主项目适配与HOLD/恢复/LAND分阶段实施计划和验收矩阵；时延及工作量均为建议目标，本轮未创建仓库、改代码或部署。 |
| 2026-09-23 | [Task36：配置参数说明与 magic number 整理](history/report-2026-09-23-task36-config-constants.md) | 为独立规划库24项参数补充单位和调参说明，提取具名实现常量但保留原数值；构建与核心/DDS回归通过，未部署两机或重做性能与实飞测试。 |
| 2026-09-23 | [Task36：面向库使用者补充注释](history/report-2026-09-23-task36-library-comments.md) | 补充独立规划库公开接口、调用顺序、坐标/线程/状态语义与关键算法注释；剥除注释后五文件及消息结构不变，仅文档维护，未做新增运行验收。 |
| 2026-09-23 | [Task36：避障算法 ROS2 Jazzy 独立移植](history/report-2026-09-23-task36-obstacle-avoidance-ros2-port.md) | 完成独立Jazzy规划库移植、缺陷修复和amd64/ARM64测试；平均约10Hz但全链P99未达100ms，真实Odin回放仅验证NO_PATH，未集成飞控。 |
| 2026-09-21 | [软著仓库 Odin-Tag 面板补图简报](history/report-2026-09-21-软著仓库Odin-Tag面板补图.md) | 为独立软著仓库补拍真实Qt AprilTag-Odin面板并更新README，1180×1050视觉检查通过；离线状态如实不可用，不表示校准或飞行成功。 |
| 2026-09-21 | [软著仓库界面截图更新简报](history/report-2026-09-21-软著仓库界面截图更新.md) | 重拍独立软著仓库当前1600×920地面主界面，展示热重启、打点与导出入口；其他无变化图片保留，离屏检查通过，未启动飞行会话。 |
| 2026-09-21 | [软著代码仓库简单同步执行简报](history/report-2026-09-21-软著代码仓库简单同步.md) | 同步97个生产文件至独立软著仓库，保留V1.0与许可证归属，五包构建和隔离smoke通过；该发行不带单测，历史手册与截图未同步。 |
| 2026-09-20 | [Task34：AprilTag–Odin 滑窗标定检查与现场失败分析](history/report-2026-09-20-task34-odin-tag-full-check.md) | 只读重放确认Tag3运行坐标差37厘米导致三次失败，小Tag受倾斜与首次航向门阻塞；说明固定SE(2)不能消非线性漂移，提出窗口改进但未实施。 |
| 2026-09-18 | [refresh 飞控重启后 MAVROS 参数刷屏修复](history/report-2026-09-18-refresh-mavros-param-retry-fix.md) | 为refresh MAVROS2.15.1独立overlay修复参数重复回包放大重试及busy重排期，隔离复现与真实整表通过；new未部署，升级须核对补丁ABI。 |
| 2026-09-18 | [refresh MAVROS 时间同步修复](history/report-2026-09-18-refresh-mavros-timesync-fix.md) | 定位refresh实际timesync频率为零导致重启门拒绝，启动入口恢复10Hz并要求读回和真实时钟；现场READY通过，代理未执行FCU重启，闭环待用户验收。 |
| 2026-09-18 | [refresh 机载代码同步与 sparse checkout 收紧](history/report-2026-09-18-refresh-sparse-sync.md) | 同步refresh Shell迁移并收紧机载sparse，归档现场资料、更新三个unit路径；四包原生构建、24项测试和3.3 smoke通过，独立Odin未受干扰。 |
| 2026-09-18 | [scq 地面机 Tag-Odin 面板启动修复](history/report-2026-09-18-scq-odin-tag-startup.md) | 修复HP地面机旧install缺ApplySavedCorrection，重建修正接口后真实Qt客户端订阅正常；包测试仍有Tag尺寸固定断言失败，未验证完整飞控链。 |
| 2026-09-18 | [小尺寸 Tag 亚像素角点精度复核](history/report-2026-09-18-small-tag-subpixel-accuracy.md) | 修复小旋转Tag固定精修窗口跨编码问题，自适应窗口使63帧角点/PnP显著稳定；回归和源码部署通过，服务加载及约5厘米绝对偏差仍待复测。 |
| 2026-09-18 | [静止 Tag1 间歇漏检排查与修复](history/report-2026-09-18-tag-outer-border-detection.md) | 复现外部黑框合并真实Tag候选造成漏检，调整候选距离后同图62/62与独立63/63成功；refresh重建重启保留active，绝对精度及动态条件未验收。 |
| 2026-09-17 | [右上角动态提示动画改进](history/report-2026-09-17-activity-banner-animation.md) | 改进右上提示蓝色扫光、单次绿光与持续红光，58项相关GUI回归及离屏视觉检查通过；未做实机/SITL与真实桌面主观体验验收。 |
| 2026-09-17 | [动态提示底色与动画状态统一](history/report-2026-09-17-activity-banner-background.md) | 将动态提示背景统一按进行中、成功、失败和空闲状态配色，16项回归与四状态离屏检查通过；仅地面GUI变更，飞机无需部署。 |
| 2026-09-17 | [correction_service 可配置的解码/发布限频](history/report-2026-09-17-correction-configurable-publish-fps.md) | 新增解码前可配置发布限频，refresh默认30Hz，采集进程CPU降约40.8%；两轮标定和停止通过，非整机降幅，动态精度及完整负载未验收。 |
| 2026-09-17 | [当前位置追加航点执行报告](history/report-2026-09-17-current-pose-waypoint.md) | 增加当前位置打点按钮，直接将显示同帧原始位姿追加航点并沿用有效性门控；68项回归与离屏检查通过，未自动上传或验证真实遥测。 |
| 2026-09-17 | [MAVROS 插件裁剪评估（仅方案，未实施）](history/report-2026-09-17-mavros-plugin-trimming-plan.md) | 审查new MAVROS插件及SITL配置差异，提出分批裁剪与参数优先读取评估；本轮仅方案，无插件配置改动或裁剪性能数据。 |
| 2026-09-17 | [机载构建入口目录调整](history/report-2026-09-17-move-onboard-build-entry.md) | 将机载构建脚本移至onboard_control/deploy并更新调用、sparse和文档；17项回归及路径替身验证通过，未完整重建或同步两机。 |
| 2026-09-17 | [new 飞机机载启动耗时实测](history/report-2026-09-17-new-onboard-startup-measurement.md) | 实测new完整READY约54秒，主要等待参数表和固定40秒校验；发现开机校时使120秒等待提前超时，未改生产代码，非冷启动耗时统计。 |
| 2026-09-17 | [new 飞机重启客户端迁移同步简报](history/report-2026-09-17-new-reboot-client-sync.md) | 同步new飞控重启客户端路径迁移及自检版本修正，源码哈希与隔离smoke通过；三服务PID保持，未执行真实FCU重启，refresh已下线。 |
| 2026-09-17 | [通过必要参数事件进一步缩短机载启动](history/report-2026-09-17-onboard-parameter-event-startup.md) | 接收必要参数事件并保留缓存回退、断线清空及原值校验，new READY降至32～34秒；构建回归通过，整表速度未改善，冷启动及实飞未验收。 |
| 2026-09-17 | [机载启动校时缺陷与等待时间修复](history/report-2026-09-17-onboard-startup-clock-and-latency-fix.md) | 改READY为相对超时、并行启动与持续订阅，提前只读拉参核验；new两轮READY约39秒，隔离校时回归通过，真实断电/NTP冷启动未复验。 |
| 2026-09-17 | [Shell 操作入口目录整理](history/report-2026-09-17-organize-shell-entrypoints.md) | 将14个Shell文件归入scripts分层目录并更新调用、unit与sparse，102项回归及带空格路径验证通过；本轮未重建或部署两机，现场数据须保留。 |
| 2026-09-17 | [清理重复 ROS 包查询与优先参数读取验证](history/report-2026-09-17-prefix-cleanup-and-priority-param-validation.md) | 部署ament索引查询替代重复ROS CLI，new预检减少约4.5秒；独立只读优先参数实验使READY约7.5秒，尚未接入生产，未裁剪插件。 |
| 2026-09-17 | [正式接入关键参数优先读取与 Odin 辅助节点耗时估算](history/report-2026-09-17-priority-parameters-production-and-odin-timing.md) | 正式接入有界关键参数优先读取并保留校验/回退，new两轮READY约7秒；估算Odin辅助节点耗时未裁剪，真实冷启动与实飞仍未验收。 |
| 2026-09-17 | [删除 tools 与重启客户端归位](history/report-2026-09-17-reboot-client-relocation-remove-tools.md) | 原样迁移FCU重启客户端并按用户要求删除项目标定tools及11项专测；282项回归通过，refresh同步完成，项目外标定结果保留，new待同步。 |
| 2026-09-17 | [将 new 的启动优化同步至 refresh](history/report-2026-09-17-refresh-startup-sync.md) | 同步new启动优化至refresh并原生构建，19项C++测试通过；无飞控台架仅验证Odin输出和零优先请求，未达到READY，终态无残留。 |
| 2026-09-17 | [refresh UQ212 外参同步](history/report-2026-09-17-refresh-uq212-extrinsics-sync.md) | 同步refresh机械近似UQ212外参并重建、重启独立修正服务，配置指纹一致；旧extnav active保留，本轮未重新标定或证明新外参精度。 |
| 2026-09-17 | [全套机载服务启动耗时排查](history/report-2026-09-17-startup-latency-investigation.md) | 两轮生产SITL初始化约44秒，主要等待EKF导航而非进程启动；只读审查实机固定等待与提示粒度，未实施优化或测Jetson完整冷启动。 |
| 2026-09-17 | [Task33：飞控热重启与热重联](history/report-2026-09-17-task33-fcu-hot-reboot.md) | 实现GUI/机载共用FCU热重启及协议3.3，new三入口真实未武装恢复约10～12秒；首轮缺原点如实超时后修复，未验收实飞及USB重枚举。 |
| 2026-09-17 | [Task33 变动统计与 refresh 机载代码同步](history/report-2026-09-17-task33-refresh-sync.md) | 统计Task33变更并同步refresh协议3.3，修正自检版本，四包ARM64构建及24项测试、零setpoint smoke通过；无飞控，真实热重启未验收。 |
| 2026-09-17 | [通讯面板打开状态提示统一](history/report-2026-09-17-upstream-panel-activity.md) | 补齐通讯面板打开及再次唤起的成功提示、浅绿背景和单次动画；19项GUI回归与实际点击验证通过，窗口装饰延迟回调问题未扩展修复。 |
| 2026-09-17 | [UQ212 标定空 ROI 崩溃修复](history/report-2026-09-17-uq212-calibration-empty-roi-fix.md) | 复现边缘负索引造成空ROI的Laplacian崩溃，加入角点、单应矩阵及非空检查；两端11项测试通过并部署普通文件，真实持板移动待用户复测。 |
| 2026-09-17 | [UQ212 同板内参标定程序](history/report-2026-09-17-uq212-intrinsic-calibrator.md) | 适配UQ212同AprilGrid板标定工具，保留原采样流程并输出原图K/D双格式；refresh真实读帧与窗口smoke通过，用户多姿态采集及独立精度未完成。 |
| 2026-09-17 | [UQ212 实测内参投入生产](history/report-2026-09-17-uq212-measured-intrinsics-production.md) | 按授权采用全部30张UQ212实测内参、RMS约0.886px并同步开发机/refresh；构建配置核对通过，异常视图及边角覆盖风险保留，独立精度未验收。 |
| 2026-09-17 | [UQ212 下视相机机械近似外参更新](history/report-2026-09-17-uq212-mechanical-extrinsics.md) | 依据用户尺寸和Odin图纸更新UQ212机械近似外参，30项回归及旋转/轴向核对通过；仅本地main更新，平移与安装角误差未实测，两机待同步。 |
| 2026-09-17 | [UQ212 原算法一致性审计](history/report-2026-09-17-uq212-original-algorithm-audit.md) | 审计UQ212工具与旧Wasintek算法，撤回检测间隔和rational开关的非必要改动，保留已证实错误修复；11项测试通过并部署，未启动采集。 |
| 2026-09-17 | [UQ212 单 Python 文件入口](history/report-2026-09-17-uq212-single-python-entry.md) | 将UQ212工具合为单Python文件，内置采集与自动进入项目环境，refresh软链接改普通文件；30帧及12项控制读回通过，真实持板标定未完成。 |
| 2026-09-16 | [相机档案目录纠正与设备配置入口说明](history/report-2026-09-16-camera-profile-directory-and-device-entry.md) | 将Wasintek/UQ212完整档案归入各服务config，修复递归安装并同步refresh；258项回归通过，活动配置仍需人工成套切换，未新增远程选相机。 |
| 2026-09-16 | [correction_service 自有 UVC 采集与 refresh 静态标定验证](history/report-2026-09-16-correction-owned-uvc-capture.md) | 实现本包V4L2 UVC采集与可信缓冲时间戳，解除外部相机依赖；refresh六轮24/0收敛并干净退出，但偏航门拒绝应用，真实几何精度未验收。 |
| 2026-09-16 | [correction_service 独立启停入口实施简报](history/report-2026-09-16-correction-service-entrypoints.md) | 新增修正服务独立启停脚本并统一systemd入口，249项回归及new启停、防重实测通过；仅管理修正进程，停止服务保留extnav active。 |
| 2026-09-16 | [地面站到 refresh 的 UQ212 标定流程实机测试](history/report-2026-09-16-ground-refresh-uq212-calibration-flow-test.md) | 真实地面Qt面板到refresh请求成功，但首帧超时；定位原生120协商与实际60帧导致PTS落后丢帧，未改代码或应用修正，完整标定被阻断。 |
| 2026-09-16 | [refresh 飞机 Odin/extnav 独立启动入口同步简报](history/report-2026-09-16-refresh-aircraft-split-launchers.md) | 同步Odin/extnav独立入口至refresh，修复未读现场overlay导致找不到包；新SSH终端启动与清理成功，无头RViz及extnav退出异常仍保留。 |
| 2026-09-16 | [refresh 飞机 correction_service 独立启停入口同步简报](history/report-2026-09-16-refresh-correction-entrypoints.md) | 同步refresh修正服务独立启停脚本及unit，真实启动idle与停止无残留通过；保持disabled/inactive策略，未启动其他链路或应用修正。 |
| 2026-09-16 | [Odin 与 extnav 独立一键启动修复简报](history/report-2026-09-16-split-odin-extnav-one-command-launchers.md) | 独立Odin/extnav入口自动读取现场环境及项目接口，new实测约399/100Hz并清理；未带起其他服务，无头RViz与extnav停止traceback未解决。 |
| 2026-09-16 | [UQ212 相机配置迁移与 Wasintek 备份简报](history/report-2026-09-16-uq212-camera-profile-and-correction-config.md) | 备份Wasintek配置并将refresh修正服务切至UQ212原生1080p120、默认镜头控制与理论内参；254项回归通过，新相机内外参和识别精度未验收。 |
| 2026-09-16 | [独立 USB Jetson Ubuntu 24.04 环境配置](history/report-2026-09-16-usb-jetson-ubuntu2404-bootstrap.md) | 为独立USB Orin NX配置24.04/Jazzy、WiFi、软件、输入法及RDP，原生构建smoke通过；无外设闭环未测，同版本ARM64 NoMachine安装未完成。 |
| 2026-09-16 | [地面站航点列表 CSV 导出](history/report-2026-09-16-waypoint-list-csv-export.md) | 地面站新增当前航点列表CSV导出，保持index/x/y/z/角度yaw与导入一致；248项回归通过，按钮只读快照不上传，不涉及实机验证。 |
| 2026-09-15 | [Task29 下视相机镜头预设与 Tag 计数说明简报](history/report-2026-09-15-task29-camera-lens-controls-and-tag-counter.md) | 澄清tag=0表示零次解码，持久化镜头预设并在首帧后写入、回读和清队列；169 dry-run 24/0收敛，历史间歇零识别根因仍未定位。 |
| 2026-09-15 | [Task29 旧 T 公式对照与响应式修正面板简报](history/report-2026-09-15-task29-legacy-t-comparison-responsive-panel.md) | 增加同快照旧+T公式只读对照及560像素窄窗布局，242项回归通过；不改生产定位链，Tag中心偏差根因及实际桌面体验仍未验收。 |
| 2026-09-15 | [Task29 下视相机 video_service 同款参数与曝光 50 实机验证简报](history/report-2026-09-15-task29-lens50-aircraft-validation.md) | 采用视频同款镜头预设并改曝光50，在169三次独立开流均零拒绝、显著消除过曝；仅保存dry-run候选，未应用，旧零识别原因仍不确定。 |
| 2026-09-15 | [Task29 恢复版首次标定实机部署与验证简报](history/report-2026-09-15-task29-restored-first-calibration-aircraft-validation.md) | 将恢复版首次算法部署169，第二轮采样、应用与旧T对照成功，FCU/EKF跟随；首轮零识别未归因，物理约20厘米偏差仍待独立核查。 |
| 2026-09-15 | [Task32：Tag 坐标约定导致的反向位置修复](history/report-2026-09-15-task32-fix29-tag-coordinate-convention.md) | 修复官方Tag与OpenCV角点180°基定义错误并撤销错误相机补偿，静态位置方向恢复且EKF最终跟随；未做人工搬回中心和独立精度验收。 |
| 2026-09-14 | [Task31 专利图2含义明确化简报](history/report-2026-09-14-task31-patent-figure2-clarification.md) | 明确专利图2的O/W方向、航向旋转及加权质心图例，调整点对箭头；26页DOCX逐页与结构复核通过，未改变技术内容或仿真数据。 |
| 2026-09-14 | [Task31 专利图2箭头端点与图2—图4黑体修正简报](history/report-2026-09-14-task31-patent-figure234-heiti-and-arrow-endpoints.md) | 将专利图2～4文字统一黑体并消除图2箭头端点间隙；附图和26页DOCX视觉审计通过，无缺字或分页漂移，旧窗口需重新加载文件。 |
| 2026-09-11 | [Task31 专利版式兼容性微调简报](history/report-2026-09-11-task31-patent-layout-compatibility-refinement.md) | 解释LibreOffice逻辑页位与实际纸张差异，调整图1/2及附图间距，使文档稳定为26页；结构和逐页检查通过，分节重编号仍保留。 |
| 2026-09-10 | [Task 29 恢复 Task27 首次标定算法简报](history/report-2026-09-10-task29-restore-task27-first-calibration.md) | 撤销错误首次P-RQ重锚，恢复Task27完整SE(3)首次候选及筛选，保留Task29有效分支去T；238项回归通过，本轮未部署飞机。 |
| 2026-09-10 | [Task 29 首次粗标定水平锚点 Bug 修复与真机复验简报](history/report-2026-09-10-task29-single-tag-planar-anchor-bug.md) | 历史错误记录：误判非零倾斜投影差并引入首次P-RQ重锚，虽台架应用成功却使用户误差恶化；报告已声明结论作废，后续恢复Task27算法。 |
| 2026-09-10 | [Task31 专利内容精简执行简报](history/report-2026-09-10-task31-patent-content-refinement.md) | 精简航向校准专利为26页、两实施例和四幅附图，修复拒绝分支连线；逐页与结构审计通过，保留现场单次试验和合成数据证据边界。 |
| 2026-09-10 | [Task31 无人机跨时刻视觉标记航向校准发明专利执行简报](history/report-2026-09-10-无人机跨时刻视觉标记航向校准发明专利.md) | 完成跨时刻视觉标记基线航向校准专利DOCX，30页含12项权利要求与六幅附图；渲染审计通过，现场单次记录与仿真分述，未改生产代码。 |
| 2026-09-09 | [双摄像头 testcam 修复与下视安装方向检查简报](history/report-2026-09-09-dual-camera-testcam-and-orientation.md) | 修复Linux双摄像头预览，采用by-path区分同序列号设备并并发采集1080p MJPEG；确认下视相机约偏转90°，机械调整待用户完成。 |
| 2026-09-09 | [多 Tag 滑窗标定数学分析与三维动画简报](history/report-2026-09-09-multi-tag-sliding-window-calibration-analysis.md) | 推导跨位置Tag中心配准、加权SE(2)与基线精度关系，完成离线数值自检及三维动画；未改生产服务，0.1～0.2°示例不代表实机验收。 |
| 2026-09-09 | [任务 29 真机编译部署与下视相机台架简报](history/report-2026-09-09-task29-aircraft-deployment-camera-bench.md) | 在169原生部署修正接口2.0，修复终态耗时累加并确认下视相机路径；两轮真实候选因偏航跳变被拒绝，未应用或验证世界精度。 |
| 2026-09-09 | [Task 29 多 Tag 滑窗标定与飞控参考中心修正执行报告](history/report-2026-09-09-task29-multi-tag-window-fcu-center.md) | 实现多Tag FIFO滑窗、加权SE(2)、事务应用与服务端权威状态，删除有效FCU分支多余T；本地回归构建通过，真实多Tag及绝对精度未验收。 |
| 2026-09-09 | [任务 29 放宽实验门限与真机修正效果测试简报](history/report-2026-09-09-task29-relaxed-gates-apply-bench.md) | 按授权放宽地面倾斜与偏航跳变门限，真实候选应用后extnav/EKF跟随；清回identity时实测航向瞬态，外参与多Tag绝对精度仍未验证。 |
| 2026-09-09 | [视频服务内参归档与直播去畸变算力分析简报](history/report-2026-09-09-video-service-intrinsics-and-undistortion-analysis.md) | 归档视频相机内参并测量Jetson去畸变算力，CPU完整链约需3.8核；直播去畸变未实现，硬件编码与端到端集成仍需原型验证。 |

## 2026-08（109份）

| 日期 | 报告 | 摘要 |
| --- | --- | --- |
| 2026-08-31 | [map 系无人机位姿终端解算脚本执行简报](history/report-2026-08-31-map-frame-odometry-terminal-solver.md) | 新增只读map系IMU位姿终端工具，用逆TF复合Odin里程计，并配置drone-new密钥；合成与真机输出通过，采用最近TF且未转换为机体中心。 |
| 2026-08-31 | [Task 27 下视相机外参 180°修正简报](history/report-2026-08-31-task27-camera-extrinsic-180deg-fix.md) | 当时将相机外参右乘180度并部署，dry-run收敛但未应用；绝对精度未验收，该补偿后来由Task32撤销。 |
| 2026-08-29 | [任务 27：AprilTag-Odin 定位修正（refined）执行报告](history/report-2026-08-29-27-tag-odin-fix-refined.md) | 实现并部署独立Tag-Odin修正、历史时间匹配、CAS应用和故障保底；169未武装台架及198项回归通过，世界精度、实飞与MAVLink重置通知未验收。 |
| 2026-08-29 | [Task 27（refined）工作量与难度评估](history/report-2026-08-29-task27-refined-workload-assessment.md) | 按Task27实际系统交付评估资深单人工期25至40人日，区分一两天算法演示与完整工程；实飞及30米精度验收另需估算，不计入已完成台架。 |
| 2026-08-26 | [Windows 摄像头 RTSP 使用教程任务简报](history/report-2026-08-26-Windows摄像头RTSP使用教程.md) | 编写Windows无代码摄像头调参与RTSP教程，使用OBS、MediaMTX和VLC，核对型号与模式；仅文档核查，未做Windows实际驱动及GUI验收。 |
| 2026-08-26 | [Windows 摄像头 RTSP 教程精简任务简报](history/report-2026-08-26-Windows摄像头RTSP教程精简.md) | 按要求将Windows摄像头RTSP教程由约500行缩至200行，保留安装、调参、推拉流与停止步骤，删除原理和排障；Markdown检查通过。 |
| 2026-08-26 | [机载视频冷启动网络竞态修复简报](history/report-2026-08-26-onboard-video-cold-boot-network-race-fix.md) | 两次授权重启复现并修复视频DDS早于Wi-Fi地址创建的竞态；启动前等待有效局域网路由，第二次冷启动跨机发现通过，初次awk语法故障已修复。 |
| 2026-08-26 | [上位机握手递增超时与双机同步报告](history/report-2026-08-26-upstream-handshake-backoff-sync.md) | 上位机三阶段握手超时改为15至30秒递增，成功后复位，调整JPG路径并同步双机main；15项专项通过，保留并发文档提交及用户修改。 |
| 2026-08-26 | [视频服务离线启动与面板首次发现竞态修复简报](history/report-2026-08-26-video-service-offline-start-and-first-discovery-fix.md) | 视频unit移除外网校时等待，面板首次命令增加2秒服务发现；182项测试及跨机首次请求通过，当时Wi-Fi冷启动竞态未处理，外部停止后的飞控未重启。 |
| 2026-08-26 | [独立地面站 WebSocket 模拟客户端交付报告](history/report-2026-08-26-websocket-simulation-client.md) | 交付独立单文件WebSocket模拟客户端，支持握手重连、巡检返航时序与遥测；真实JAR流程及182项回归通过，媒体为占位，不模拟飞行动力学。 |
| 2026-08-26 | [Windows 摄像头教程图片 Git 忽略简报](history/report-2026-08-26-windows-camera-tutorial-assets-gitignore.md) | 将docs/assets加入Git忽略，三张Windows教程截图保留本机并确认规则命中；未修改教程Markdown或处理生成PDF。 |
| 2026-08-25 | [双开发机代码提交与同步报告](history/report-2026-08-25-dual-workstation-code-sync.md) | 整合两开发仓库，保留Java8提交、媒体占位常量与本机文档和云台示例，统一远端main；专项验证通过，101旧TODO留stash，未实现云台。 |
| 2026-08-25 | [无外网地面站本地时间校验修复报告](history/report-2026-08-25-offline-ground-clock-command-validation.md) | 命令TTL改为租约及心跳建立的相对时间基准，移除飞控外网校时依赖；数十天偏差注入和ARM64台架通过，真实断WAN冷启动待验。 |
| 2026-08-24 | [机载服务同步、systemd 重装与未解锁验证报告](history/report-2026-08-24-onboard-service-sync-restart-verification.md) | 选择同步61个机载文件并原生重建、重装飞控和视频unit；未武装READY及1080p60录像通过，MAVROS时间跳变归因与Odin高负载抖动仍待耐久验证。 |
| 2026-08-24 | [任务 24：多个 Bug 修复执行简报](history/report-2026-08-24-task24-multiple-bug-fixes.md) | 修复Wayland装饰、LAND忙碌门控、无帧纯黑预览及最终航点进度；176项测试和真实SITL通过，装饰仅证实留边，未取得目标桌面截图或实飞验证。 |
| 2026-08-24 | [任务 24 复验：Wayland 子面板自绘阴影修复简报](history/report-2026-08-24-task24-wayland-subpanel-shadow-followup.md) | 复验发现adwaita仍无所需阴影，两个子面板改用透明外沿与自绘窗口；101真实Wayland阴影像素和177项回归通过，已保留现场改动同步。 |
| 2026-08-24 | [上位机自动返航连接门控修改简报](history/report-2026-08-24-upstream-auto-return-connection-gating.md) | 低电量及异常自动返航限定WebSocket在线仿真，断线不触发、在线实机仅提示；180项测试通过，实机返航留TODO，已下发任务断线不中止。 |
| 2026-08-24 | [上位机通讯面板窗口行为修复简报](history/report-2026-08-24-upstream-panel-window-fix.md) | 上位机面板改为无父级普通顶层窗口，修复压住主窗口和最小化失效，入口支持恢复；167项回归通过，桌面标题栏外观仅离屏验证。 |
| 2026-08-24 | [航点 GUI 默认控制组合修改简报](history/report-2026-08-24-waypoint-gui-default-control-combination.md) | GUI启动默认选择改为普通梯形速度与轨迹PD+DOB，既有发送链读取实际枚举；167项回归通过，接口回退值与机载控制逻辑未改。 |
| 2026-08-21 | [AprilTag 相机外参标定工具执行简报](history/report-2026-08-21-apriltag-camera-extrinsic-calibrator.md) | 部署独立AprilTag与Odin二维外参标定工具，含静止质量门及鲁棒求解；两端合成自测通过，Odin无数据故未联合实测，内参与Tag配置待确认。 |
| 2026-08-20 | [双机载安装器与系统 MediaMTX 简报（2026-08-20）](history/report-2026-08-20-dual-onboard-installers-system-mediamtx.md) | 整理飞控和视频独立安装器，四组件unit统一入口并移除仓库MediaMTX二进制；164项测试通过，依赖系统安装，未运行真实安装器，历史大对象仍在。 |
| 2026-08-20 | [入口脚本重命名与真机同步简报（2026-08-20）](history/report-2026-08-20-entry-script-renames-and-aircraft-sync.md) | 重命名飞控启停及地面安装入口，更新部署、测试和飞机unit路径；164项回归及未武装核验通过，飞控PID保持，未重启服务。 |
| 2026-08-20 | [Git 历史大对象清理简报（2026-08-20）](history/report-2026-08-20-git-history-large-object-cleanup.md) | 完整备份后改写97个提交，删除三类大对象及相关快照引用，媒体留本机并停止跟踪；Git完整性通过，仓库缩至41MiB，远端main已强制更新。 |
| 2026-08-20 | [地面站本机摄像头 MediaMTX 故障修复简报](history/report-2026-08-20-ground-camera-mediamtx-repair.md) | 定位本机视频失败为旧后台缓存已删除二进制路径及系统依赖缺失；退出旧后台并安装MediaMTX后，真实RTSP、截图、录像与资源释放通过。 |
| 2026-08-20 | [真机视频独立启停与手工启动配置一致性修复简报](history/report-2026-08-20-independent-onboard-video-control-fix.md) | 视频启停改由面板直连独立视频节点，手工启动统一系统配置；飞控离线时真实720p120启停截图录像通过，首次同步漏模块已补齐，飞控未启动。 |
| 2026-08-20 | [机载视频一键安装简报（2026-08-20）](history/report-2026-08-20-onboard-video-one-command-install.md) | 新增机载视频一键安装器，保留现场配置并独立注册systemd，开机仅待命；19项部署视频测试通过，限Jazzy/ARM64，未在飞机重复安装。 |
| 2026-08-20 | [机载视频根目录启停与彻底残留清理报告](history/report-2026-08-20-onboard-video-root-launch-stop.md) | 视频启停入口迁至根目录，停止器清理节点、RTSP端口和相机占用，支持独立重启；真机孤立FFmpeg故障注入清理通过，飞控保持未武装运行。 |
| 2026-08-20 | [摄像头开启后应用镜头参数修复简报](history/report-2026-08-20-post-open-camera-lens-controls.md) | 镜头参数改为RTSP就绪后等待1秒逐项写入并读回，解决打开设备重置曝光；真实1080p60读回25/200通过，录像首秒可能保留默认曝光。 |
| 2026-08-20 | [Task 22.5 视频链路边界审计与 1080p60 真机复验简报](history/report-2026-08-20-task22-5-video-route-audit-1080p60.md) | 移除飞控手动视频代理，明确手动直连视频、飞行事件由机载发布；169真实1080p60启停截图录像通过，自动起降与航点抓拍仅隔离验证。 |
| 2026-08-20 | [视频部署文档与 720p120 默认值调整报告](history/report-2026-08-20-video-deployment-docs-and-720p120-default.md) | 完善Jetson视频部署文档并统一H.264720p120默认值，保留地面用户配置；真机跨机解码与28秒录像通过，未做运动清晰度A/B定量比较。 |
| 2026-08-19 | [项目 MEMORY 清理与当前基线整理简报](history/report-2026-08-19-memory-cleanup.md) | 将MEMORY由808行流水账整理为245行当时基线，纠正协议、默认坐标及飞机部署表述；仅文档修改，旧部署文档部分3.0文字仍待维护。 |
| 2026-08-19 | [任务 22.5：机载视频、地面面板与上位机媒体回传执行报告](history/report-2026-08-19-task22-5-onboard-video.md) | 实现独立机载视频、起降开关、航点抓拍、真实媒体回传及三来源面板，协议升3.2；本机摄像头与隔离ROS通过，飞机ARM64部署和实飞待补。 |
| 2026-08-19 | [任务 22.5：新机载计算机未武装真机补测报告](history/report-2026-08-19-task22-5-real-aircraft-bench.md) | 在169完成视频ARM64部署、跨机RTSP、截图与故障隔离，修复245消息请求及幂等关闭；未武装台架通过，真实起降和巡检媒体回传未验。 |
| 2026-08-19 | [平滑航点启动速度重试修复简报](history/report-2026-08-19-waypoint-start-speed-retry-fix.md) | 平滑航点启动余速过高改为悬停重试，连续10次才锁存异常；两轮SITL命中首次失败后恢复，持续失败由单测覆盖，未实机验证。 |
| 2026-08-18 | [摄像头面板双设备切换修复简报](history/report-2026-08-18-camera-dual-device-selection-fix.md) | 修复设备探测后旧配置覆盖新选择，避免HP路径配Wasintek模式；双摄像头切换录像与136项测试通过，101历史分叉未擅自改写。 |
| 2026-08-18 | [摄像头服务 Ubuntu 24.04 部署文档补充简报](history/report-2026-08-18-camera-service-deployment-readme.md) | 补充Ubuntu24.04摄像头后台、独立面板与完整地面站部署说明；入口及MediaMTX版本核查通过，未做新机器安装或ARM64运行验证。 |
| 2026-08-18 | [HP MJPEG DRI RTSP 纯绿色修复简报](history/report-2026-08-18-hp-mjpeg-dri-rtsp-green-fix.md) | 定位HP绿屏为JPEG重启标记经RTP丢失，新增仅RTSP分支兼容转码；HP彩色播放和Wasintek原码回归通过，录像不转码但兼容分支有CPU成本。 |
| 2026-08-18 | [HP 高分辨率 MJPEG RTSP 绿屏修复简报](history/report-2026-08-18-hp-mjpeg-rtsp-limit-fix.md) | 确认2560宽MJPEG超出RTP/JPEG2040上限导致宽度回绕；隐藏并拒绝超限模式、增大发送队列，合法1080p播放录像通过，协议上限仍在。 |
| 2026-08-18 | [任务 21：上位机 WebSocket 通讯集成实施报告](history/report-2026-08-18-task21-websocket-implement.md) | 实现独立WebSocket插件与Qt薄适配，命令映射、原始日志和状态上报经真实JAR及SITL通过；媒体、云台、低电量自动返航与实飞尚未实现或验收。 |
| 2026-08-18 | [任务 22：上位机交互时序实现与验证简报](history/report-2026-08-18-task22-timeseq.md) | 实现巡检起飞至降落时序、返航入库、01/08门控及入点异常恢复，协议升3.1；两轮JAR+SITL通过，机库仅坐标阈值，实机未验。 |
| 2026-08-17 | [摄像头面板启停按钮灰色禁用态修正报告](history/report-2026-08-17-camera-panel-disabled-button-style.md) | 修复彩色角色样式覆盖摄像头启停按钮禁用态的问题，统一灰色禁用外观；渲染目视与121项回归通过，未改变后台逻辑。 |
| 2026-08-17 | [摄像头面板 Dock 名称乱码修复报告](history/report-2026-08-17-camera-panel-dock-name-fix.md) | 将摄像头面板桌面应用名及窗口名统一为ASCII，解决Dock乱码；Qt字段和122项回归通过，已运行旧面板需重新打开生效。 |
| 2026-08-17 | [摄像头面板二次启动黑屏与界面精简修复报告](history/report-2026-08-17-camera-panel-restart-and-layout-fix.md) | 通过停止后重建QMediaPlayer修复摄像头二次启动黑屏，并精简布局与按钮状态；两轮真实开关预览及录像通过，后台生命周期独立。 |
| 2026-08-17 | [USB 摄像头 RTSP、录像、截图与地面站集成执行报告](history/report-2026-08-17-camera-video-service-ground-station-integration.md) | 完成独立USB视频服务、RTSP/录像/截图和Qt面板集成，单次采集双路输出；真实多模式及119项测试通过，最长仅75秒，未做半小时耐久。 |
| 2026-08-14 | [真机摄像头 MJPEG 120 FPS AVI 录像脚本简报](history/report-2026-08-14-demo2-mjpeg-120fps-avi.md) | 新增MJPEG原码AVI录像与独立预览脚本；真实601帧完整保存解码，预览未限速录像，但码率高、异常断电可能缺索引，窗口绘制未测。 |
| 2026-08-14 | [真机摄像头 120 FPS 录像加速问题修复简报](history/report-2026-08-14-demo2-native-h264-120fps-fix.md) | 用摄像头原生H.264直封装及硬件预览修复120FPS录像加速；真机约118至119FPS且时长正确，依赖指定硬件，窗口绘制与断电封装未验。 |
| 2026-08-14 | [悬停油门回退值 0.22 同步简报](history/report-2026-08-14-hover-throttle-022-sync.md) | 将悬停油门回退值0.22发布并同步两端源码和安装配置，构建通过；未重启旧进程，其参数仍0.39，实际推力标定继续取飞控值。 |
| 2026-08-14 | [真机摄像头录像改为 MP4 执行简报](history/report-2026-08-14-real-camera-mp4-output.md) | 把真机录像脚本输出由MJPG/AVI改为mp4v/MP4，保留采集与预览行为；合成三帧写入回读通过，未实际打开摄像头录制。 |
| 2026-08-14 | [真机摄像头本地录像脚本执行简报](history/report-2026-08-14-real-camera-video-save.md) | 基于真机demo2新增带时间戳的本地AVI录像脚本，保留预览和退出方式；合成帧封装回读通过，为避免占用未测试真实采集。 |
| 2026-08-13 | [control.yaml 中文参数注释补全简报](history/report-2026-08-13-control-yaml-chinese-comments.md) | 为control.yaml全部67项参数补充中文用途、单位与语义注释；键值完全不变，解析和隔离节点启动通过。 |
| 2026-08-13 | [Git 大型 MP4 历史清理执行简报](history/report-2026-08-13-git-large-mp4-cleanup.md) | 修订未推送提交以移除127MiB视频对象，新增全部MP4忽略规则；可达对象核查通过，推送结果未在报告中具体列明。 |
| 2026-08-13 | [真机通讯检测与完整连接接口错配修复简报](history/report-2026-08-13-hardware-communication-interface-fix.md) | 修复飞机源码3.0与安装产物2.2错配，补版本门、READY探针和关闭竞态；原生重建后检测与连接通过，跨ROS版本及实飞仍未验收。 |
| 2026-08-13 | [机载服务源码同步与本地构建测试简报](history/report-2026-08-13-onboard-source-sync-local-build-test.md) | 审查并同步机载接口3.0、平滑航点与控制配置源码；本地构建测试通过，飞机未重建重启，运行仍为2.2，新速度参数未实飞。 |
| 2026-08-13 | [真机机载服务彻底停止脚本修改简报](history/report-2026-08-13-onboard-stop-script-complete.md) | 完善机载停止脚本，清理systemd外手工启动组件及后代；真实清理、幂等停止和启动前检查通过，入口已发布同步，未正式启动飞行栈。 |
| 2026-08-13 | [地面站/飞机通用机载构建入口简报](history/report-2026-08-13-portable-onboard-build-entry.md) | 新增通用机载构建快捷入口，自动选择ROS发行版并支持完整验证；本地及飞机原生构建通过，运行服务未重启，状态查询超时。 |
| 2026-08-13 | [192.168.112.101 地面站仿真启动故障排查与修复报告](history/report-2026-08-13-remote-simulation-start-fix.md) | 修复101地面站MAVProxy入口及future依赖缺失，增加启动前探针；真实未武装SITL初始化与清理通过，历史demo收集错误保留。 |
| 2026-08-13 | [三轮 5 米往返爬升航点任务简报](history/report-2026-08-13-three-round-waypoints.md) | 生成三轮5米往返、每次爬升0.5米并交替航向的13点CSV，终点高度3.8米；导入和静态语义验证通过，未执行仿真或实飞。 |
| 2026-08-12 | [通讯频率状态块宽度防抖修复报告](history/report-2026-08-12-communication-rate-chip-width-stability.md) | 通讯频率状态块按正常位数设置最小宽度，消除9.99/10.00Hz切换导致邻块抖动，保留长文扩展；76项回归通过，仅Qt显示改动。 |
| 2026-08-12 | [当前改动审查、清理与推送简报](history/report-2026-08-12-current-changes-review-and-push.md) | 审查工作树，保留协议原件、任务资料与正式报告，回收代理过程文件及重复/空白素材，补齐忽略规则；文件完整性和测试通过，未连接实机。 |
| 2026-08-12 | [地面站异常退出无残留修复简报](history/report-2026-08-12-ground-station-exit-cleanup.md) | 统一GUI信号、事件循环退出及父启动壳兜底清理，覆盖GUI被SIGKILL后的本地残留；100项回归及进程替身验证通过，整机/父壳强杀不覆盖。 |
| 2026-08-12 | [任务 15：RViz 启动位姿与空闲抖动日志修复简报](history/report-2026-08-12-task15-rviz-startup-and-idle-jitter-fix.md) | 仿真RViz改直接取隔离MAVROS位姿以免依赖预览点击，降低显示负载并停止未武装空闲WARN；96项回归及真实TF通过，底层抖动仍存在。 |
| 2026-08-12 | [任务 15：RViz 航点预览实现简报](history/report-2026-08-12-task15-rviz-waypoint-preview.md) | 实现航点Marker、名义直线Path与聚合位姿预览，仿真复用/实机独立地面RViz；95项回归及隔离真实渲染通过，未做障碍物规划或实机验收。 |
| 2026-08-12 | [Task 16 beautify-3 执行报告](history/report-2026-08-12-task16-beautify-3.md) | 重排遥测摘要和阈值状态块，新增真实roll并升接口2.2；76项回归及真机未武装状态验证通过，但推力参数未验证、未达到飞行READY。 |
| 2026-08-12 | [任务 17：CSV 批量导入航点执行简报](history/report-2026-08-12-task17-csv.md) | 实现CSV按钮/拖放导入、256点完整校验及确认后原子替换，yaw度转弧度；88项回归通过，Excel未实现，未发送示例航点或飞行验收。 |
| 2026-08-12 | [航点平稳飞行方法、GUI 切换与 SITL 调参简报](history/report-2026-08-12-waypoint-smooth-reference-methods.md) | 新增二阶滤波、梯形和S曲线参考与轨迹PD+DOB，接口3.0及武装周期锁定；103项回归与SITL对比通过，推荐S曲线，实机及自由组合未验收。 |
| 2026-08-11 | [当前改动发布与机载启动文件更新范围简报](history/report-2026-08-11-current-changes-publish-and-onboard-launcher-scope.md) | 扩展机载稀疏更新范围至启动目录与一键脚本，排除地面专用入口，并维护文档/权限；62项回归通过，记录统一发布工作，未连接实机。 |
| 2026-08-11 | [详细状态与真实结果确认改造简报](history/report-2026-08-11-detailed-status-and-truthful-results.md) | 协议2.1加入姿态、电池与状态频率；LAND等解除武装、原点等回读、频率等实测后才成功；74项回归及未武装SITL通过，正式飞机未同步。 |
| 2026-08-11 | [GPS 原点重复连接与失败彻底断开修复简报](history/report-2026-08-11-gps-origin-reconnect-and-failure-teardown.md) | 同FCU会话原点匹配缓存改幂等成功，所有完整连接失败销毁ROS context；75项回归及真机连续重连通过，部署后服务运行，DDS告警保留。 |
| 2026-08-11 | [真机交互启动配置修复执行简报](history/report-2026-08-11-manual-onboard-launch-env-fix.md) | 交互机载启动读取统一onboard.env，解决systemd串口和overlay配置未进shell的问题；74项回归及真机无启动检查通过，未启动飞行栈。 |
| 2026-08-11 | [真机机载自启动首次校时修复简报](history/report-2026-08-11-onboard-autostart-time-sync-fix.md) | 定位首次NTP墙钟跳变造成开机MAVROS退化，服务改等待校时target；75项回归和实际启动READY通过，未再次冷重启，后续版本已替代此方案。 |
| 2026-08-11 | [真机机载启动脚本开机自启动配置简报](history/report-2026-08-11-onboard-boot-autostart.md) | 安装启用四组件systemd开机服务，最终inactive且零残留；旧脚本忽略--check曾短暂启动后清理，未解锁起飞，真实冷启动尚未验证。 |
| 2026-08-11 | [真机机载服务同步与未武装测试简报](history/report-2026-08-11-real-onboard-sync-and-test.md) | 旧Humble飞机同步接口2.1并原生构建，5项测试和隔离烟雾通过；真实状态约10Hz、零setpoint且无IMU订阅，最终停服，频率仅ACK确认。 |
| 2026-08-11 | [移除 100 Hz 实测监听简报](history/report-2026-08-11-remove-100hz-rate-listeners.md) | 按要求撤销两路IMU订阅和100Hz实测状态机，频率配置恢复服务ACK含义；保留详细遥测、LAND/原点回读，74项回归通过，未同步实机。 |
| 2026-08-11 | [Task 13 refine-3 执行与调查报告](history/report-2026-08-11-task13-refine-3.md) | 完成手动提示和详情布局，仿真就绪由52.9降至43.5秒；调查MAVROS无害告警、日志和握手真实性，65项回归通过，后续路线仅建议。 |
| 2026-08-11 | [任务 14：甲方地面站对接架构调研报告](history/report-2026-08-11-task14-integration-research.md) | 调研甲方V2.0对接，建议独立协议适配器共享单控制核心、事件源和任务仓库；列出坐标、安全命令及媒体歧义，未实施或验证控制功能。 |
| 2026-08-11 | [任务 15：航点与障碍物可视化方案调研](history/report-2026-08-11-task15-visualization-research.md) | 建议地面独立RViz显示航点/轨迹，机载处理Odin地图与规划；发现Path订阅Pose类型错误，Qt5/6嵌入边界明确；仅调研，实机点云/TF未核实。 |
| 2026-08-11 | [Ubuntu 22.04 / Humble 免硬编码部署适配简报](history/report-2026-08-11-ubuntu2204-humble-portable-deployment.md) | 新增按系统自动发现ROS、Python、overlay和串口的部署入口，Ubuntu22.04优先Humble；73项回归及本地部署通过，目标地面22.04未实装。 |
| 2026-08-10 | [2026-08-10 目录、ROS Domain、参数同步与 DDS 异常诊断简报](history/report-2026-08-10-domain-parameter-dds-cleanup.md) | 清理169MiB旧产物并诊断domain、参数同步与DDS错误；本地实现GUI按需ROS和机载重连配置频率，50项回归通过，实机未部署，跨版本未根治。 |
| 2026-08-10 | [地面站仿真/实机隔离与退出死锁修复报告](history/report-2026-08-10-ground-simulation-domain-isolation-cleanup-fix.md) | 仿真固定domain231本地发现，实机domain0，支持销毁重建context；修复日志管道清理死锁，60项回归及未武装SITL通过，DDS根治仍待。 |
| 2026-08-10 | [不配置 GPS 原点改造清单](history/report-2026-08-10-originless-refactor-checklist.md) | 提出删除GPS原点接口、协议3.0和本地PD+DOB相对起飞改造清单，建议仅改ArduPilot参数；仅调研未实施，真机参数与定位链未核实。 |
| 2026-08-10 | [真机同步与四组件一键启动实测简报（2026-08-10）](history/report-2026-08-10-real-deployment-integrated-start.md) | 同步机载自动频率/参数修正并新增四组件一键启动，两轮旧Humble真机READY、位姿约94Hz且零setpoint；清理通过，跨发行版DDS告警仍在。 |
| 2026-08-10 | [启动脚本结构同步与地面站入口完成简报（2026-08-10）](history/report-2026-08-10-start-layout-ground-launcher-complete.md) | 恢复网络后原样同步五个机载启动文件并核对哈希，完成地面专用入口及旧入口回收；52项回归通过，分步extnav续行偏移参数隐患保留。 |
| 2026-08-10 | [启动脚本结构同步与地面站入口简报（2026-08-10）](history/report-2026-08-10-start-layout-ground-launcher.md) | 确认机载启动目录结构并新增地面入口，隔离检查及5项专项测试通过；飞机随后网络不可达，五个机载文件尚未拉取，旧入口暂保留。 |
| 2026-08-10 | [任务 12.5：仿真/实机切换 bug 修复简报](history/report-2026-08-10-task12.5-simulation-hardware-switch-bugfix.md) | 修复断开未销毁DDS及跨context租约序号重置，仿真抓包零出站、真机同身份重连通过；62项回归通过，切换RMW未能根治跨版本告警。 |
| 2026-08-09 | [GUI 窗口性能全面优化执行报告](history/report-2026-08-09-gui-window-performance-optimization.md) | 以增量状态和幂等显示更新降低Qt绘制开销，稳定刷新无重绘，resize耗时降23.7%；48项回归、像素一致性验证通过，保留高DPI阴影成本。 |
| 2026-08-09 | [任务 09：独立 Wi-Fi 通讯检测按钮真机验证简报](history/report-2026-08-09-task09-real-wifi-button-validation.md) | 真实Qt Wi-Fi按钮两次连接Humble机载服务，状态约9.98Hz并接收8条测试日志；零租约/命令且清理完成，未启动MAVROS，DDS告警仍在。 |
| 2026-08-09 | [任务 10-refine-2 追加修复简报](history/report-2026-08-09-task10-refine-2-followup.md) | 补修数值箭头/单位、完整三项策略菜单、白色加号和输入门控灰态；44项回归及构建通过，速度仍仅UI预设，未连接或运行飞行链。 |
| 2026-08-09 | [任务 10-refine-2 执行简报：地面站 UI 细节完善](history/report-2026-08-09-task10-refine-2.md) | 落实18项UI细节，含Wi-Fi取消、帮助图标、日志和进度、防误触确认及紧凑航点；44项回归通过，起降速度未下传，未执行飞行验证。 |
| 2026-08-09 | [任务 11 执行情况简报：手动操作模式 UI 界面修改](history/report-2026-08-09-task11-mode.md) | 三栏改为操作/航点两栏，连接和飞行动作上排、美国手双摇杆下排；保持八向命令映射，45项回归通过，最小窗口需滚动，未运行飞行链。 |
| 2026-08-09 | [任务 12 执行情况简报：手动操作模式 UI 美化完善](history/report-2026-08-09-task12-beautify-2.md) | 增加双摇杆底盘、按压反馈、独立灵敏度及机体/ENU输入转换，遥测改摘要与折叠详情；46项回归通过，未改协议或机载控制，未实飞。 |
| 2026-08-09 | [任务 12 追加修正简报：坐标系日志与完整下拉菜单](history/report-2026-08-09-task12-coordinate-dropdown-fix.md) | 坐标系切换写入语义日志，坐标和灵敏度选择器改完整向下菜单以消除裁切；46项回归通过，未改变输入转换或连接实机。 |
| 2026-08-09 | [任务 12 追加调整：手动选择器门控与摇杆响应式间距](history/report-2026-08-09-task12-manual-control-gating-responsive-spacing.md) | 坐标系与两灵敏度选择器统一手动运动门控，调整摇杆响应式留白及固定中缝；46项回归和尺寸截图核对通过，仅本地Qt验证。 |
| 2026-08-09 | [航点执行与进度区压缩执行报告](history/report-2026-08-09-waypoint-execution-panel-compression.md) | 移除航点执行区灰色说明布局，保留隐藏状态接收器；三尺寸表格均增高30px，48项回归通过，未改变进度和飞行逻辑。 |
| 2026-08-08 | [Grok 昨日工作审计报告](history/report-2026-08-08-grok-work-audit.md) | 审计Grok历史工作并补做SITL，确认核心闭环可用；发现协议未升级、生产日志未降级、原点错误文案和策略回退问题，未证实改写控制算法。 |
| 2026-08-08 | [协议版本、MAVROS 日志与原点文案修复报告](history/report-2026-08-08-interface-log-origin-fixes.md) | 协议统一升至2.0，修复mavros_sim生产日志分级和仅实机写原点文案；27项回归、构建及ROS测试通过，未实现避障策略行为仍保留。 |
| 2026-08-08 | [任务 08：机载服务安全部署报告](history/report-2026-08-08-task08-onboard-deployment.md) | 在旧Humble/aarch64飞机最小检出并原生构建机载包，修复Clock API差异；5项测试与隔离零setpoint烟雾通过，未连接真实飞控链。 |
| 2026-08-08 | [任务 09：地面站—真机通信测试报告](history/report-2026-08-08-task09-communication.md) | 完成Humble飞机至Jazzy地面站只读状态、飞控和日志链测试，两窗各601条状态；曾将实机连接改为只读，跨版本DDS告警仍未解决。 |
| 2026-08-08 | [任务 09 调整简报：独立 Wi-Fi 通讯检测与完整实机连接恢复](history/report-2026-08-08-task09-wifi-communication-adjustment.md) | 纠正只读连接产品误读，恢复正式连接租约/频率/原点功能，新增独立零命令Wi-Fi检测；37项回归及隔离Qt→ROS通过，正式实机连接未复验。 |
| 2026-08-08 | [航点飞行策略下拉与接口预留](history/report-2026-08-08-waypoint-flight-strategy.md) | 新增直线、自动避障和遇障悬停下拉及ROS策略字段；当时只有直线实现，其他选项告警后仍直线执行，报告仅列验证命令。 |
| 2026-08-07 | [环境入口与航点组件门控](history/report-2026-08-07-env-waypoint-gates.md) | 收紧环境和航点门控：已有会话禁止重复连接，无会话禁用全部航点组件；保留原发送安全条件，并补充回归断言。 |
| 2026-08-07 | [原点齿轮会话锁定与绿色按钮 Hover](history/report-2026-08-07-origin-gear-success-hover.md) | 原点齿轮仅空闲时可改缓存，运行或连接期间禁用；补齐绿色按钮悬停变深样式及对应Qt回归，未列出测试通过数量。 |
| 2026-08-07 | [位姿发散 / RViz 三轴回归修复报告](history/report-2026-08-07-pose-rviz-regression-fix.md) | 撤销SITL写入实机缓存原点并清理假位姿发布者，修复百万米位姿和RViz模型消失；24项回归通过，未完整在线复飞，部分文案未改。 |
| 2026-08-07 | [Task 07 部分撤销：集成终端回退](history/report-2026-08-07-task07-terminal-revert.md) | 因用户反馈撤销PTY集成终端，恢复外部终端入口；保留启动日志降级和关闭自动滚动修复，原实现报告仅作历史记录。 |
| 2026-08-07 | [Task 07：终端集成与日志修改执行报告](history/report-2026-08-07-task07-terminal.md) | 实现日志/PTY终端页签、启动噪声降级和自动滚动修复，22项回归通过；终端非完整VT模拟器，未复验完整SITL，随后另报撤销终端。 |
| 2026-08-07 | [Qt 主窗口与子窗口外框阴影修复报告](history/report-2026-08-07-window-shadow.md) | 统一主窗口与消息框四边轮廓、透明留边和阴影，补齐拖动缩放及最大化行为；20项Python、5项ROS测试通过，仅改外观，未复飞。 |
| 2026-08-06 | [Task 04：PySide6/Qt 地面站 GUI 重构执行报告](history/report-2026-08-06-task04-qt.md) | 将Tk地面站替换为模块化PySide6，加入统一门控、确认和实时日志；Qt→ROS→SITL全流程及14项Python回归通过，实机未验收。 |
| 2026-08-06 | [Task 05：Qt 界面美化与修复执行报告](history/report-2026-08-06-task05-beautify.md) | 完成三栏布局、紧凑状态带、组合日志筛选、输入箭头和航点清空进度修复；18项回归及SITL起降通过，未验证实机，存在调度峰值。 |
| 2026-08-06 | [task/01-init.md 执行情况详细报告](history/report-2026-08-06.md) | 汇总任务01模块化与任务03机载C++控制重构，统一PD+DOB、租约和失联LAND并修复直接启动环境；SITL通过，实机与硬实时未验收。 |
| 2026-08-06 | [机载端—地面站解耦与 GUI 技术选型调研报告](history/research-architecture-gui-2026-08-06.md) | 调研机载与地面站解耦及Qt/Web选型，指出当时Python控制与双实现问题；建议闭环留机载、首版PySide6，属方案分析而非实施验收。 |
