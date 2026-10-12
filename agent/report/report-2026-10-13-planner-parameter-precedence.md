<!-- Parameter source unification, verified deployment and remaining scope. -->
# 规划器参数读取优先级统一

日期：2026-10-13。

## 结果

三个launch（主工程avoidance、独立odin/planner）统一采用显式命令行 → 同一planner.yaml节点配置 → launch/节点默认值。
移除启动文件对有效YAML盲区、搜索下界、frame及时钟的固定覆盖；CLI支持完整ROS参数名及blind_radius/map_mode/search_lower短名。
同一YAML公共path_planning段保存调参，/avoidance/path_planning专属段选service_only=true/latest_observation，
avoidance_bridge段保存桥参数，odin_launch/planner_launch段保存独立话题。桥专属CLI参数使用bridge.<参数>。
文件不可读、YAML语法/节点段结构错误告警回退；非法参数值继续由节点校验，不静默掩盖。
显式coordinate_mode=identity选择map，显式planning_frame仍有最高优先级；YAML identity须配套map frame。

latest_observation替换桥提供的完整持久地图，桥用观测射线维护障碍及unknown；不是丢弃历史扫描。
dual_tree在规划器中交替累计KD树，按tree_period（当前20有效帧）轮换；正常累计历史约20–40次更新，非墙钟TTL。
service_only关闭独立goal/定时规划，保留点云更新及PlanMotion服务；latest_observation要求service_only=true。
内部/avoidance/map与/avoidance/plan_motion连接保持。没有修改搜索算法、控制器或硬件驱动。

## 验证

- 开发机path_planning及avoidance_bridge构建通过；仅已有FLANN/CMake开发者警告。
- 真实ROS参数验证：独立两个入口各9组、桥5组，共23组全部通过。
  包含YAML/嵌套参数、CLI非零与零覆盖、整数形式double、frame/时钟/下界、模式、桥参数及错误文件回退。
- 开发机真实规划器＋桥合成点云回归：2项通过、1项按真实规划器模式跳过（非失败）。
- refresh在domain149/LOCALHOST执行相同23组实参验证通过；未连接FCU、未启动Odin/飞控，未解锁或起飞。
- refresh源码/安装YAML哈希一致：55fabfa53c84d3f41f6079a492fb5a54636427655eec2bffc12d6fd39169aca7。
- 只检查本次修改文件的diff格式；已有TODO及报告规则文件的格式问题未改。

## 部署与限制

refresh(.186)两个仓库的launch/helper/config源码与安装资产已同步；均为Python/YAML资产，未替换或重建ARM二进制。
备份：/home/nvidia/ros2-ardupilot-maintenance/planner-config-precedence-20261013-O0zVU6/before.tar.gz。
开发机日志：agent/codex/planner-config-precedence-20261013/；参数细项日志仍在测试定义的agent/codex子目录。
现有生产进程未重启，配置下次启动生效；源码YAML与安装YAML为普通文件，修改源码须构建更新install或显式config指向源码。
new(.169)尚未同步本次配置入口修改，已在两个MEMORY标记；真实传感器闭环、飞行、长时负载不属于本次验收。
