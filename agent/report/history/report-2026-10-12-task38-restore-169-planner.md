<!-- 恢复Task38集成、169现场算法/参数保持、两机部署与本次失败记录。 -->
# 恢复Task38两种策略，保持169现场避障算法

日期：2026-10-12。用户要求恢复地面站“自主避障”“遇到障碍悬停”，排除算法重写，以169当前实际使用源码与用户调参为准；两架飞机均在线。

## 结果

已恢复Task38完整接口链并同步两机原生构建：避障执行、同航点等待/恢复、15s超时LAND、策略锁、航点入点/拍照与RViz路径共用原实现。开发机及两机共享飞行接口3.4，独立规划器保留169搜索和碰撞核心，只新增兼容服务。直线模式不依赖规划组件。两个仓库在main提交/推送，不创建分支。

**没有发送实机解锁、起飞或飞行命令，没有申请实机控制租约。** 飞机检查仅源码/配置/参数读取、原生构建以及隔离localhost域的合成DDS/无FCU smoke，未把它称为真实传感器闭环或实飞验收。代理未重启原169 Odin、降采样和目标规划进程，也未启动正式飞行/视频/修正unit；用户在收尾时关机重启169，三个unit自动启动，运行状态以下文末尾核对为准。

## 169基线与保留内容

开始时抓取169源码、源/安装配置及正在运行节点的参数；不使用旧MEMORY中的20/0.2数值代替现场事实。

| 参数 | 169当前实际基线 |
| --- | --- |
| search.safe_distance | 0.25m，体素质心严格小于拒绝 |
| search.horizon / resolution / voxel_size | 100m / 0.1m / 0.1m |
| search.max_vel / max_acc / max_tau | 2m/s / 2m/s²轴向 / 0.6s |
| search.collision_step / search_budget | 0.05m / 0.3s |
| input_timeout / frame / stamp_clock | 0.5s / odom / receive |
| 运行bounds | [-50,-50,-2]～[50,50,10] |
| 盲区 | 源YAML0.3m，旧Odin launch默认覆盖为0m |
| 独立输入 | /odin1/cloud_slam_down |

`path_searching`整个目录与169任务开始时快照diff为空；所有旧搜索/碰撞文件和旧核心测试保持，原语最多ceil(0.6×sqrt(3)×2/0.05)=42次检查。wrapper的缺省bounds恢复169源码±50/上界5m；实际启动上界仍由YAML10m决定。

独立点云发布者是两机现有 `/home/nvidia/down_samp_exe.py`，169读参为0.1m降采样、0.4m球形剔除、auto球心。两机脚本仅文件末尾换行有差异，本次没有修改或自动管理它。真实近身障碍可能被上游剔除；桥关闭的是二次盲区，不能宣称整条链无删点。

两机全部原控制参数逐值核对无变化，只增加Task38九项参数；逐机Tag配置、所有/etc文件均与备份一致，机载相机现场改动保留。没有实现TODO，也没有提交用户的任务归档、TODO或独立库.gitignore本地改动。

## 必要兼容适配

1. 独立库恢复PlanMotion服务，PLAN使用控制权威给出的实际p/v及下一个业务航点，原样调用旧search/getSegments。CHECK_LINE和剩余曲线检查用单独MotionAdapter离散调用旧isSafe，不把连续距离求根、新版搜索限值或核心改写合入旧算法。
2. 服务模式接收桥完整观测快照，wrapper每次clearMap后setKdtree；独立Odin入口保留双树累计和定时goal行为。桥仍将unknown视为不可执行，障碍退役仍依赖新扫描射线。
3. 正式launch读取169 YAML，避免原Task38强行覆盖0.45m净空、max_tau1s、低加速度等搜索参数。只保留169当前frame、receive、下界和关闭二次盲区的入口设置。配置另可用config:=选择；namespace下实测全部search参数继承。
4. 原Task38执行包线小于旧规划器2/2轴向搜索参数。控制层统一放慢完整曲线时间：duration乘s，k次系数除s^k，空间路径不变。已符合包线的曲线不变；仍完整检查C1、末点、60s时长、水平1/垂直0.2m/s、加速度0.35/0.15m/s²和实际最近点p/v接入。不能接入时继续原制动等待，不放宽控制门槛，不改PD+DOB。
5. 0.5s输入新鲜度下地图最多4Hz，避免2Hz边界过期；可选LAN预览仍最多2Hz、10000点。构建入口增加cmake-force-configure，避免回退后缓存漏掉新接口；启动--check额外验证PlanMotion确实已安装。

## 本次验证

| 验证 | 实际结果 |
| --- | --- |
| 开发机Release构建 | 独立库与主工程通过 |
| 主工程C++ | 4个CTest套件/25个gtest通过；桥2例通过 |
| 独立库适配与旧输出 | MotionAdapter4例、输出契约6例通过 |
| 独立库Python | 服务DDS1例、原ROS输入/benchmark8例通过 |
| GUI、协议、启动/部署相关Python | 50例通过 |
| 控制器真实CPP/DDS | 仿真全部回收后独占4例通过；身份、延迟接入、取消、LAND重试/解除武装等门槛保持 |
| 桥DDS | mock3例、mock extnav3例通过；真实规划器2通过、1个仅mock延迟用例跳过 |
| 启动参数 | 开发机2例通过（多组Odin覆盖与Task38默认/自定义继承）；两机各Task38参数继承用例通过 |
| ARM64两机 | 原生构建、控制4套件、桥2例、新适配/旧输出2套件通过；真实桥DDS各2通过1skip；正式启动--check通过 |
| 无FCU smoke | 开发机/169/186均interface=3.4、connected=false、armed=false、setpoint_messages=0 |
| **旧核心失败** | **19例18通过，CollisionBetweenPrimitiveEndpoints仍失败：0.099990361693828175m<0.1m** |

旧核心失败是保留的离散采样间隙，169原二进制在前次回退已复现。没有重新修改算法、阈值、删除测试或把结果美化为全量通过。本次新增服务校验也不构成连续净空证明。

真实ArduPilot SITL均domain231/LOCALHOST，用有限FOV几何射线，保持恢复版169参数：

| 场景 | 本次结果 |
| --- | --- |
| straight，规划组件不启动 | 航点完成，照片1次，LAND确认解除武装 |
| hover_recover | 悬停等待、移走障碍后恢复同航点，照片1次，LAND |
| avoid | 完整绕行，照片1次，LAND |
| avoid_recover | 目标受阻、移走障碍后恢复，照片1次，LAND |
| timeout | 连续等待15s任务失败并LAND，确认实际解除武装，无照片 |
| fault_cancel | 等待中用户悬停，撤销任务；移走障碍不恢复，正常LAND |

遇障悬停恢复时间0.805s，自主避障恢复0.904s；
等待漂移均约0.055～0.057m。绕行最小实际圆柱表面间距0.3715m，
横向偏移0.7678m，最高参考/实际水平速度
0.8529/0.8861m/s。
直线/悬停恢复实际约1.064m/s，原PD+DOB超调仍存在，没有实现或声称严格实测速率≤1m/s。

## 失败与纠正

- 186首次构建缺guided_interfaces新头，回退后的CMake缓存未按旧文件时间戳重新配置。新增force-configure后原生重建、接口和smoke通过。
- 桥DDS初次固定等待2s尚未发现请求订阅者，首个volatile请求丢失。测试改为等待实际发现后发送，开发机和两机复测通过，不改变生产响应标准。
- 远端初次直接CTest未source ROS，ament模块缺失；加载正确overlay后重跑通过，不将环境失败当作算法通过。
- 控制器测试每25ms发送心跳且紧跟到期结果，KEEP_LAST(1)覆盖回复；测试改成生产桥一致1Hz，先心跳后到期结果。并行SITL清理按进程basename回收，曾误停另一个隔离域测试节点；最后全部仿真回收后独占4例通过。不能把不同DDS域当作可与全局清理并行的保证。
- fault_cancel首次没有进入航点任务，未保存及时服务拒绝诊断，不能据此判为规划算法失败或成功。保留首次失败JSON；脚本在插入障碍后等待当前scan ready，并增加事件日志，重试后等待、取消、不恢复和LAND全部通过。

## 部署、备份和未完成验收

两机备份 `/home/nvidia/ros2-ardupilot-maintenance/task38-restore-20261012/` 包含同步前源码、逐机配置、Git差异、/etc归档及构建/验证日志；开发机完整169快照、参数、SITL JSON/samples/logs位于 `agent/codex/task38-restore-20261012/`。源码在两个仓库main维护，旧算法核心和新服务适配可分别审阅。过程文件不提交。

正式避障需启动独立组件 `scripts/onboard/start_avoidance.sh`，并先确保既有cloud_slam_down发布者运行；`--check`仅校验安装，不启动服务。主飞行/视频/修正unit本次未启动，169现有旧goal规划进程未动；已安装源码需要按正常工作流重新启动相应组件后生效。两机同时在线并不代表应在相同domain同时启用同名飞控端点。

实飞、真实Odin/FCU闭环、近身覆盖、约20°安装外参、有效Tag分支、长时热稳态、全链P99及视频并行负载仍未验收。桥unknown规则、20m原点地图、外部0.4m剔除和旧离散净空风险均保留。hp-desktop未在本次同步，MEMORY保留其待升级标记。

## 收尾Git核对与用户重启后的真实状态

功能提交主工程4e5d969、独立库621a2dc已推送main，两机Git均已快进同步；原机载相机/Tag修改保留，原src等payload已存入逐机Git stash，没有reset/clean或强制推送。主工程73个功能文件、独立库23个功能文件分别与两机逐个SHA256一致，独立核心与任务开始169快照相同。186首次stash遇到新包在稀疏检出规则外，先扩大稀疏规则再stash/快进成功，未丢现场文件。

169首次最后同步时不可达；用户明确说明自行关机并重新启动。重新连接后完成Git同步和安装检查。被动8s读取76条ControlStatus和8条MAVROS State：interface=3.4，armed始终false，control_mode=IDLE，avoidance_ready=false；不是飞行验收。169飞控/视频/修正unit自动active；186三个unitinactive，但独立Odin正在运行。

**实际阻塞：domain0/SUBNET内点云及高频Odin里程计各2个同名发布者，cloud_slam_down为0个发布者。** 169有唯一MAVROS/控制状态，但其外部定位输入可能混入另一机Odin数据。按AGENTS第12条停止进一步实机联调，没有自行停止用户Odin进程、修改ROS域或启动正式避障/降采样链。软件恢复与隔离验收已完成；现场使用前必须先按用户指定目标隔离定位数据源，再启动该机独立降采样和避障组件。此次未完成真实传感器联调与正式避障ready，不把它描述为可直接实飞。
