# 静止 Tag1 间歇漏检排查与修复

## 结论

已在 drone-refresh（192.168.112.186）复现并修复。不是增加 Tag1 坐标配置导致，
而是纸张额外黑色外框参与候选筛选：OpenCV4.6 默认 minMarkerDistanceRate=0.05
可能把内部真实 Tag 与较大外框合并，优先留下无法解码的外框，导致“未检测到 Tag”。
显式设为0.02保留内部候选后，原失败帧均能解码。字典、纠错、黑边校验、PnP、
质量门、同帧多Tag拒绝规则、首次与滑窗公式、相机内外参均未更改。

OpenCV官方参数说明：
https://docs.opencv.org/4.7.0/d5/dae/tutorial_aruco_detection.html
新版OpenCV筛选实现及默认值已有变化，本次实机版本为4.6.0；未声称已在4.14上验证。

## 现场与证据

- 用户确认位置、光照从故障到排查均未变化。当前Tag1清晰可见，外围另有黑色方框。
- 用户CSV：Tag0=(0,0,0)，Tag1=(0,1.8,0)，yaw均0，size_m均0.099。
  文件由用户维护，本次未覆盖、未提交；size_m应测编码最外黑边，不含装饰外框。
- 日志中16:10 Tag1首次超时；16:14 Tag0成功但33 accepted/190 rejected；
  16:16 Tag1 next成功但25 accepted/247 rejected。因此并非只影响Tag1。
- 先直接UVC采样39/39成功，完整写入生产镜头参数后39/39也成功。没有据此宣布问题消失。
- 随后用生产CameraProcess、UVC节点、ROS Image、原检测器，在隔离domain232/localhost
  保存62帧，复现26帧检测成功、36帧无ID。同一帧仅配置Tag1和配置Tag0+Tag1结果完全一致。
- 对完全相同62帧离线对照：

| image_scale | minMarkerDistanceRate | 阈值窗口max | 识别 |
|---|---|---|---|
| 0.5 | 0.05 | 23 | 26/62 |
| 1.0 | 0.05 | 23 | 0/62 |
| 0.5 | 0.01、0.02、0.03 | 23 | 各62/62 |
| 0.5 | 0.05 | 53 | 26/62 |

- 采用0.02后，独立实机同链采样63/63成功；生产安装产物更新后再次63/63成功。
  两轮均检查单/双Tag配置等价。历史失败任务未保存图像，不能逐帧反推其全部拒绝原因；
  本次原因通过现场新采样和同图对照确认。
- 流程证据及原始PNG保存在开发机/refresh的 `agent/codex/tag1-diagnosis-20260918/`。

## 实现与测试

- detector.py仅增加候选合并参数及说明。
- 新增Tag0/1各三种白边间隔加装饰黑框的合成回归，验证唯一ID、正确内部角点及PnP残差；
  原0.05下6项全部失败，修复后6项通过，另有纯背景阴性测试。
- 旧方向真值测试隐式依赖现场17cm纸张；用户换为9.9cm后5个精度断言在修复前后均失败。
  固定历史合成场景为17cm，保留原8mm/0.3°精度断言，不放宽标准。
- 配置加载测试原来锁死17cm，改为逐项比对用户CSV的ID、位置、尺寸和角度，验证加载精确性。
- 开发机 `tests/correction`：93 passed。refresh原生新回归：7 passed。
- 未运行全仓飞行测试：生产只改图像候选筛选，不涉及飞行控制。

## 部署与终态

- 备份：`/home/nvidia/backups/tag1-detection-20260918/`，含原detector.py和现场tag_pose.csv。
- 定点同步detector.py，原生 `colcon build --packages-select correction_service` 成功。
  源码/install SHA256一致：`6db8da1183206c64be1796cad0ce888107db5270998d1006b4f762a3a310df17`。
- 用户明确回复“重启即可”后，停止旧人工校准进程并启动独立odin-correction.service。
  unit active，NRestarts=0；新instance为99f683f256fe400e86e70c6fb0edf545，窗口N=0。
- extnav重启前后均valid=true/revision=4，同session `odin-389069420985-7ec680f9`，
  修正保持(-0.15615950684864952,-0.6581351347389787,15.318981569446459°)，
  applied_job_id仍为8e712e3ff704-apply。未调用apply/clear校准服务。
- 原extnav PID7292、Odin PID8885均保留。没有运行MAVROS/onboard飞控进程，未收到FCU State；
  因此不虚称通过armed=false遥测验证。本次只使用相机、只读状态和独立校准服务启停，
  没有解锁、起飞、模式切换、位姿控制、参数或原点写入。
- 诊断采集全部结束，设备无占用；用户需要重新首次/下一次采样。未代用户重新应用校准。
- new及独立USB Jetson未部署本次补丁，下次同步需补齐。

## 边界

本次验证静止场景识别稳定性和内部角点语义，不代表任意高度、光照、运动条件100%识别，
也不代表两点世界坐标精度已验收。真实校准整条采样收敛需用户复测；本次没有覆盖现有active。
