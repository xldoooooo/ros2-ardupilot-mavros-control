# AprilTag-Odin 定位修正



以下不是任务正文，是可能有用的背景资料补充，按需参考即可

> # AprilTag-Odin-Correction
>
> 我现在完成了几个相互独立的工作，
>
> 1. 摄像头内参已经标定（不保证精度但比较准确，去畸变效果比较好）
> 2. 摄像头外参(摄像头距Odin的IMU的外参）已经标定（不保证精度但比较准确）
> 3. 实现了一个用cv实时识别AprilTag，并据此计算出tag与镜头中心相对位姿的python程序；此程序还会接受一个IMU估计出来的自身位姿，然后计算IMU的估计位姿的偏差，以及相应的将其修正到真实位姿所需的补偿量（偏航角旋转与x y 方向的平移）。（假设AprilTag的标签的坐标系是世界坐标系，tag的坐标是固定且已知的绝对坐标）
>
> 现在要将他们串联并集成到飞机上, 具体想做的事情是：
>
> 
>
> ## 已知问题
>
> Odin在每次启动时，根据自己的imu和视觉等算法，会自己建立一个坐标系，认为自己启动时所在的位置是（0,0,0），偏航角0。然而，每次启动所建立的坐标系之间会存在误差，并不是每次建立的坐标系都完全相同。
>
> 例如，飞机从完全相同的位置连续启动两次Odin, 虽然每次自己的初始位置都是原点，但同样的世界坐标系（30, 0, 0）（假设沿世界坐标系x方向往前30米）处，odin的读数会不一样，比如说第一次启动后，到（30,0,0）处的读数是（29.8, 0.24，0.001）；第二次则会变成（29.78, -0.15, 0.0006）。
>
> 这是因为，两次启动Odin所建立的坐标系Todin与世界坐标系Tworld之间 相差的平移矩阵T与旋转矩阵R会有变化。尤其关键的问题在于横向偏移，也就是偏航角的误差。我们希望30米误差不超过0.1米，则偏航角精度需要在0.1度的数量级，以目前的odin的精度来说是达不到的。
>
> 已知事实：Odin建立的坐标系的原点就是Odin的IMU的原点，IMU是Odin融合建系的中心点。
>
> 我希望解决这个问题
>
> 
>
> ## 设想的解决方法
>
> 
>
> 1. 事先约定并设置好每个ID的Tag在绝对世界坐标系中绝对准确的坐标。例如id为0的tag, 真实坐标是（0,0），id为1的tag, 真实坐标是（30, 0） （单位是米，表示tag0是真实原点，tag1是原点正前方30米处的点）
> 2. 飞机在（0,0）附近起飞（坐标不固定，只知道接近0,0，能拍摄到tag0），起飞高度，起飞后自己的偏航角每次都不固定，可能有小偏差
> 3. 飞机启动下视摄像头，拍摄到（0,0,0）处的tag0
> 4. 通过AprilTag识别，此时应获取到下视摄像头中心距离tag0的相对位姿 cam_relative
> 5. 以tag0为绝对原点，则可以通过p_cam_relative得到下视摄像头中心在世界坐标系中的真实位姿cam_real
> 6. 通用摄像头的外参，可以得知摄像头中心与Odin（的IMU的）中心的相对位姿 cam_odin
> 7. 通过此相对位姿 cam_odin，结合下视摄像头的真实位姿cam_real，计算Odin的真实位姿 odin_real
> 8. 将odin_real 与Odin自己每次启动后自己建立的坐标系 odin_self 对比，可以计算得到两个坐标系的水平偏移量 T_xy 以及 （偏航角）旋转偏移量 R_yaw. 保存T和R, 关停摄像头。
> 9. 使用T和R，修正odin根据odin_self输出的融合位姿估计（Odin odometry-highfreq），得到（理论上来说）真实无偏的位姿估计 state_real
> 10. 对odin-extnav-飞控这条信息链路进行修改，原本是odin的highfreq会直接经由extnav发给飞控，现在改成不再发送原始highfreq, 而是发送修正后的 state_real 作为代替，例如：odin-fix-extnav-飞控
> 11. 上述流程3～10中，可以是利用不同的tag进行修正。比如飞到了（10,0）附近，启动摄像头，通过贴在绝对坐标（10,0,0）处的tag1 再次对odin_self进行校准，得到一组新的T和R ( 重新走一遍修正流程，不复用上次的修正量，上一轮的T和R直接丢弃)
> 12. 暂时假定摄像头视野内至多同时出现1个Tag。（后续可能会同时拍摄多个tag以提高精度，暂时不实现，在源码中标记 TODO 并预留一定的可扩展性即可）
> 13. tag来自36h11 family
>
> 
>
> ## 其他信息
>
> 1. 飞机上有两个摄像头：云台相机，用于video service，现在是断开状态；下视相机，用于本次任务，已连接。两个是同型号的。不要搞混。
>
> 2. 本次增加的功能，应当是一个尽量独立的服务，它的崩溃/失败等 不能影响到机载的飞行控制以及摄像头服务等其他功能；可以参考video_service服务那种
>
> 3. 不考虑对Odin的z轴（垂直方向）进行修正，假设它z轴是准确的，只进行 x y 水平方向平移以及偏航角旋转 的修正
>
>    下视相继的z轴可能是歪的（下视相机安装在机体上，飞机水平悬停的时候，下视相机可能不是严格竖直沿重力方向向下的，有小角度安装偏差）







## 目标

利用世界坐标已知的 AprilTag，在飞机悬停于 Tag 上方时估计 Odin 自建坐标系相对世界坐标系的水平平移与偏航修正量，使后续 Odin 位姿能够对齐固定的车间世界坐标。

本次只修正 `x、y、yaw`，不修正 `z`。相机内参、相机到 Odin IMU 的外参和现有 AprilTag 识别程序均作为已知输入。

## 总体架构

原始 Odin 数据必须始终保留，新增修正功能不能成为原链路的额外单点故障：

```text
/odin1/odometry_highfreq ───────────────> extnav ──> ArduPilot EKF
                                              ▲
correction_service ──候选 T/R──> SetCorrection
                                              │
                                              └──> /odin1/odometry_highfreq_corrected
```

extnav 始终订阅原始 Odin：

- 有有效修正量时，对原始位姿应用修正后发送给飞控；
- 没有有效修正量时，继续发送原始位姿；
- 将实际发送给飞控的位姿同步发布到 `/odin1/odometry_highfreq_corrected`，供地面站调试和录包；
- 维护当前生效的修正量、版本号和有效状态。

修正关系必须按 SE(2) 变换计算，不能简单分别给 `x、y、yaw` 加常数：

```text
C_world_odin = T_world_imu(tag) × inverse(T_odin_imu)
T_world_imu_corrected(t) = C_world_odin × T_odin_imu(t)
```

## correction_service

在项目根目录建立独立的 `correction_service/`，负责按需打开下视相机、识别 Tag、计算候选修正量并评价其质量。

主要要求：

- 默认空闲并关闭摄像头，只有收到明确命令才开始采样；
- 根据图像时间戳匹配同一时刻的 `/odin1/odometry_highfreq`，不能直接使用识别完成时的最新位姿；
- 使用相机内外参和 Tag 世界位姿，先完成完整空间变换，再提取水平 `x、y、yaw` 修正量；
- 持续采样直至结果收敛，采样期间只维护候选值，不得逐帧改变当前生效修正；
- 只有样本数、残差、离散度和预期 Tag ID 均满足要求后，才向 extnav 提交候选修正；
- extnav 明确确认应用并返回 revision 后，本次任务才算成功；
- 识别失败、结果发散、超时或摄像头被占用时，明确报告失败，不修改当前修正量，也不终止其他进程；
- 服务崩溃不得影响 extnav 继续使用最后一组有效修正或原始 Odin；
- 在模块自己的 `log/` 下记录每次任务、Tag ID、修正量、质量指标和错误信息。

## 配置

`correction_service/config/` 至少包含：

- `intrinsics.yaml`：来自飞机 `/home/nvidia/camera_calib`；
- `extrinsics.yaml`：来自飞机 `/home/nvidia/vins_odin_calib/output/success01-run_20260827_233838`；
- `tag_pose.csv`：人工维护 Tag 的世界位姿，格式为：

  ```csv
  tag_id,x,y,z,yaw_deg,size_m
  ```

- `general_settings.yaml`：Tag family、话题名、采样时间、超时、收敛与发散阈值等；
- `camera.conf`、`lens.conf`：参考 `video_service`，只保留采集和镜头所需配置。

Tag family 使用 `36h11`。首版假定画面中至多使用一个 Tag；多 Tag 联合估计暂不实现，只预留清晰扩展点。

## 指令与状态

correction_service 保持独立 ROS 2 接口：

- 地面站可以直接 start、stop、查看 status/result，不经过 `onboard_control`；
- start 支持 `expected_tag_id` 和 `apply`；
- `apply=false` 只计算和展示候选 T/R，用于独立调试；
- `apply=true` 在结果合格后提交给 extnav；
- 地面站模式可以持续运行到收到 stop；未来 onboard 模式必须同时由双方维护独立超时；
- 同一时刻只允许一个校准任务运行。

本次实现地面站直接控制。`onboard_control` 自动触发和飞机端 bash 启停暂不实现，但接口需要能够支持后续接入。

地面站上的相关控制ui组件，通过一个子面板实现，其入口按钮放在地面站GUI右上角“摄像头配置面板“右侧。要展示一些必要信息以方便调试，因为我们目前并不确定这一套校准方法是否行之有效。顺便，删除“在此处打开终端“按钮及其功能。

## extnav 修正与保底

extnav 负责维护和应用 active T/R，而不是 `onboard_control`：

- 从未校准或修正无效时，identity passthrough，继续传递原始 Odin；
- correction_service 退出时，若 Odin 未重启，继续使用最后一组有效 T/R；
- Odin 重新启动或重新建系时，旧 T/R 必须立即失效，不得跨 Odin session 复用；
- 修正量更新或退回原始坐标系属于外部估计器重置，必须按当前 extnav 实际使用的 MAVLink 消息正确通知 ArduPilot，不能伪装成普通连续测量；(需要确认当前是否能做到这一点。如果不能做到，或者需要大规模改动， 则暂时按照普通连续测量量更新 不做特殊处理，源码标记 TODO 并在报告中记录)
- 退回原始 Odin 只保证定位数据不断流，不代表世界坐标航点仍然有效，必须同时发布 `correction_valid=false` 状态；
- 

## 后续航点集成

后续可为航点增加：

```text
correction: true
tag_id: 0
```

到达此类航点后，由 `onboard_control` 负责悬停、协调摄像头占用、异步发起校准、维护 watchdog，并在收到 extnav 已应用新 revision 且定位重新稳定后继续后续航点。

`onboard_control` 只负责任务编排，不维护 T/R，也不发布 corrected odometry。地面站仍可脱离 `onboard_control` 独立调试修正功能。

## 验收原则

包括但不限于：

- 原始 Odin 话题和原始 extnav 能力始终保留；
- correction_service 空闲、失败或退出时，不切断 Odin 到 extnav 的数据；
- `apply=false` 可独立完成识别、计算、日志和地面显示，不改变飞控输入；
- `apply=true` 只有在结果合格并被 extnav 确认后才更新 active T/R；
- 原始、修正后、最终 MAVROS 位姿和修正状态均可录包量化比较；
- 实机验证不得由程序自行解锁或起飞，移动与飞行操作只能由用户完成。

在 `correction_service/README.md` 中说明配置、接口、运行方法、状态含义和已知限制等。



## 其他

extnav应该是在飞机的/home/nvidia/vrpn_mavros

odin应该是在飞机的/home/nvidia/catkin_ws



本机/home/nvidia/scq/projects/AprilTag下有apriltag仓库源码，apriltag-ros仓库源码以及识别apriltag并计算imu修正量的demo（AprilTag_IMU_Correction.py）。本次暂时先用python实现试一下，顺便估计其性能开销。若你认为后续有必要换成ros或c++或其他优化后的方法，在任务报告里说明。



注意，在改动飞机原odin与extnav等旧的部署的生产环境之前，务必进行备份，不能毁坏现有的能跑通的这套生产环境。



本次更新工作量大，难度高，任务复杂。务必全力以赴，认真反复调试，严格遵守规范，完成每一条任务要求，不得遗漏或偷工减料，不同任务要求实现时不得相互引入bug，不得使本项目出现本任务之前不存在的bug。注意仍遵守最小化实现原则。



完成后，生成详尽的任务报告。

