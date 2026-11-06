<!-- 只读 Odin 位姿和点云坐标诊断工具的运行入口。 -->
# odom/map 诊断工具

这些脚本仅订阅数据，不启动飞控或发布控制指令。先在仓库根目录加载环境：

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
.venv/bin/python dev/odom2map/odom_pose_in_map.py --help
```

- `odom_pose_in_map.py`：订阅高频里程计与 TF，输出 map 下的机体位姿；默认 child 为 `fcu`，无输入不输出数值。
- `test_cloud_point_pose_odom.py`：抽样打印点云原坐标。
- `test_could_point_pose_odom2map.py`：通过 TF 转换到 map 后抽样打印。

运行时使用目标会话的 ROS domain；实机通常为0，SITL为231。脚本中的历史 Humble/示例文件名注释不作为当前启动命令；使用本目录实际文件名与项目 `.venv`。坐标约定见[导航说明](../../docs/odom-map.md)。
