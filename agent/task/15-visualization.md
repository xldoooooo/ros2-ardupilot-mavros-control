# 航点可视化预览调研

![98b6f35f-3e4d-4a04-b2ec-de80e3353549](/home/nvidia/scq/projects/ros2-ardupilot-sitl-hardware/agent/task/assets/98b6f35f-3e4d-4a04-b2ec-de80e3353549.png)



这个视频在assets下，6db32d5410f641a3a68f4dcfbbd03838_raw.mp4

我现在有个疑问，虽然这个功能可以自行实现，但我记得Rviz里面有相关的功能。

你调研一下，这个功能有没有必要自行实现？能利用一下rviz吗？地面站也有Rviz，机载计算机也有Rviz, 能用他们吗，能的话，用哪个？ 跟自行实现相比，代价如何，工作量，难度如何，风险-回报如何？



考虑到后续这个预览里面会渲染障碍物，然后轨迹规划可能是一条避障算法规划出来的曲线。
障碍物获取应该是用当前的奥丁之眼 进行3d建图

你先调研，不修改代码。形成简要分析报告放入report
