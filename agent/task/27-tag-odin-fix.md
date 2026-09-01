



## 你需要做的

1. 

2. 在项目根目录创建一个correction_service目录，用于集中实现上述功能

   + config子目录里面存放一个tag_pose.csv文件，其格式是：

     第一行：tagid, x, y, z

     每一行依次存放：tag的id, 这个tag贴在世界坐标系的绝对坐标（x,y,z)

     此文件由开发人员手动填写并维护。

   + config子目录里面存放内外参配置文件，我已为你创建好。内参在飞机的`/home/nvidia/camera_calib`, 外参在飞机的`/home/nvidia/vins_odin_calib/output/success01-run_20260827_233838`

   + config子目录下再新建一个general_settings配置文件，里面包含各必要配置项，包括但不限于所使用的AprilTag family, 每次启动服务时的持续采样时间 最大超时等待时间 等等

   + config下再新建一个camera.conf 和 一个 lens.conf，用于配置摄像头输出以及镜头参数等（参考video_service的那两个配置文件，没用的（RTSP等）配置项删掉即可

   + 他应当是一个尽量独立的服务，它的崩溃/失败等 不能影响到机载的飞行控制以及摄像头服务等其他功能；可以参考video_service服务

   + 它自己应当有日志记录功能，记录在/log目录下，主要保存每次上报的修正量，以及重要信息/报错异常等

   + 它应当只有接收到明确的指令时才会开启和关闭，在默认状态下为关闭

   + 它通过ros2 node 进行通讯；它的指令有3条来源道路：

     1. 机载飞控服务 onboard_control_node 发来的启停命令（本次先不实现）
     2. 地面站GUI中直接发来的启停命令（本次需要实现）
     3. 在飞机上直接执行bash启动/终止脚本（本次先不实现）

     可以类比video_service的实现方法

   + 收到开启命令后，开启下视摄像头，识别tag->得到tagid以及摄像头相对tag的位姿->查表得到tag的绝对位置->得到摄像头中心的绝对位姿->得到以Odin中心为原点的真实位姿->比较Odin自己建系输出的位姿与真实位姿，得到修正量（T,R）

     在摄像头开启过程中，持续计算，使修正量收敛，稳定可靠

     如果是从onboard_control受到的开启命令，则correction_service开始计算并计时，并采取如下安全策略（仅供参考，若有不合理之处可自行修正）：（A是onboard_control，B是correction_service）

     ```
     2. A 向 B 发 start
     3. A 不阻塞等待 B
     4. B 开始执行
     5. B 自己维护执行超时
     6. B 完成后主动 report
     7. B 尝试确认 A 已收到
     8. 无论 A 是否收到，B 到达生命周期上限后都自行结束
     9. A 同时维护一个 watchdog
     10. 若超过 A 自己的 deadline 仍未收到结果：
            A 将本次 job 标记为 timeout/failure
            可选发送 cancel
            但不依赖 cancel 保证 B 最终退出
     ```

     + 这样做的原因是：这个服务会占用很高的计算资源，因此一定要确保B不能因为某种阻塞导致一直持续运行而挤占资源，产生危险）

     

     如果是从地面站/bash脚本启动，则不设任何超时时限，开启后持续运行，直到再次收到地面站/bash脚本的终止命令才汇报T R;

     不论哪种启动方式，若计算过程中修正量明显发散（发散判断阈值参数需便于修改,放入general_settings配置文件里），则直接报告异常并终止

   + T和R在机载飞控内部进行维护完成一次修正量计算后，onboard_control接受到T和R, 

   + 在目录下写一个readme文档进行必要的说明



## 位姿信息传递结构

现有的链路是：

```
Odin内部融合里程计
        ↓
	extnav
        ↓
ArduPilot EKF
```



加入修正后的链路应该是：

- extnav始终直接接收原始Odin，原链路永远存在。
- correction_service只负责计算候选T、R。
- extnav维护当前生效的T、R。
- extnav将应用后的完整Odometry发布到corrected话题，便于地面站录包、观察和比较。
- 发给ArduPilot的数据和corrected话题使用同一份计算结果。

```
Odin内部融合里程计
        ↓
	extnav 进行判断：correction有效：C × raw│ correction无效：raw
        ↓
ArduPilot EKF
```



具体话题：

```
输入： /odin1/odometry_highfreq
输出： /odin1/odometry_highfreq_corrected
```

原话题始终保留。验证通过后，只需把extnav参数从：

```
odom_topic:=/odin1/odometry_highfreq
```

改为：

```
odom_topic:=/odin1/odometry_highfreq_corrected
```











## 注意事项

+ 不考虑对Odin的z轴（垂直方向）进行修正，假设它z轴是准确的，只进行 x y 水平方向平移以及偏航角旋转 的修正

  下视相继的z轴可能是歪的（下视相机安装在机体上，飞机水平悬停的时候，下视相机可能不是严格竖直沿重力方向向下的，有小角度安装偏差）

