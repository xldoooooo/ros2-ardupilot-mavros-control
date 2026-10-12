<!-- Task3 Python optimization, CPU/timing evidence and two-aircraft deployment. -->
# Task3：extnav 性能优化

日期：2026-10-13。完成 Python 第一阶段，并在用户确认的未解锁维护窗口同步测试、原生部署 refresh/new。**整节点 CPU 相对下降30%的目标未达；时序尾部全面不退化也未验收。** C++ 阶段仅形成建议，按任务要求等待用户决定，未实现或切换入口。

## 修改范围

- 仅修改 canonical extnav、其 README 和现有 extnav 集成测试；另维护报告、索引和 MEMORY。TODO、控制器、接口、Odin配置、相机/Tag/安装参数均未改。
- identity 数值转换只读 raw；无 corrected 订阅者时不构造或复制 Odometry。有效修正仅计算必要的 pose/linear 数值，独立 corrected 消息及 covariance 在需要发布时才生成；raw 不修改。
- 删除无调用方的 latest_msg；定时器只读独立数值快照。apply/clear 等待下一条 raw，CAS、幂等、session/revision/reset、z及水平原点语义保持。
- 预计算固定 `R_IO.T @ T`，调整速度矩阵结合顺序，用标量四元数归一化/叉乘减少小数组。保留非有限/零范数检查、安装旋转和位置/角速度杆臂补偿，兼容转换入口同步修复。
- DDS匹配订阅者每0.1秒查询；匹配后下一轮查询及新raw恢复发布，另受发现/调度影响。查询平均约1.1µs（开发机）、1.72/1.76µs（refresh/new，各10,000次）。raw_messages统计输入回调，corrected_messages仅统计实际publish。有订阅者时每条已处理raw发布一次；不按armed停算，不改QoS或40/100Hz定时器。

## 数值与 ROS 验证

- 开发机相关回归 **36通过**；每机ARM extnav数学/隔离ROS回归 **2通过、15未选中**。只扩展既有集成测试，没有新增测试文件、生产门限或夹具。部分本地退出出现rclpy Destroyable清理警告，生产采样last_error为空，未改无关executor实现。
- 开发机：1,000生成样本+两机各2,000实际Odin样本，共5,000样本/20,000组新旧比较；每机ARM另做3,000样本/12,000组比较。覆盖零/非零固定安装旋转、identity/有效修正、位置/姿态/速度/covariance和冻结元数据。
- float64对照使用rtol=atol=1e-12；位置/速度最大绝对差 **4.44e-16**，姿态最大 **3.33e-16**，两种covariance一致。负unknown哨兵、NaN/Inf covariance透传保持，raw未修改；没有放宽飞行或修正质量门限。
- 现有ROS测试覆盖订阅加入/退出、速度/covariance、apply/clear后无raw不输出旧快照、幂等/CAS冲突、时间回退、frame变化、断流失效/恢复、零/NaN/Inf四元数、非有限位置和兼容入口。断流使用隔离DDS输入，没有设备断电/重置。
- 生产标准入口晚加入corrected的首消息延迟 **227/173ms**（含DDS发现）；退出后corrected计数停止，raw处理和最终快照继续。10次maintenance clear响应最大 **4.51/3.42ms**。

## 同负载整节点 CPU

停止原有项目进程、new视频/修正服务和独立Odin，并关闭发现的new更新管理器；保留桌面/远程连接。测试维持实际Odin和MAVROS，refresh domain232、new domain229，均LOCALHOST，避免两机混流。最初new domain233被DDS拒绝，未进入采样，改为229。开始、采样和结束均确认armed=false，无控制租约或解锁/起飞操作。

A/B由项目.venv执行前/后canonical节点，ROS/rclpy来自Jazzy；参数、传感器、QoS及raw/vision/pose/velocity/status observer负载相同。生产ros2 run入口另做运行核对，其smoke不混入A/B CPU。CPU由/proc进程用户+系统时间除以单调墙钟，100%表示一核，不包含observer CPU。原schedutil未改变；本次额外观测负载下的72–75%基线不能直接与历史65%截图拼接。

首轮每场景25秒；关闭更新管理器后，identity无订阅者补做20秒ABBA、有效无订阅者补做20秒成对比较。首轮refresh最大间隔变大，因此identity有订阅者再做20秒成对比较。下表采用最终可比数据，首轮原始结果保留。

| 飞机 | 场景 | 前CPU | 后CPU | 相对下降 |
| --- | --- | --- | --- | --- |
| refresh | identity无订阅（ABBA均值） | 71.65% | 70.81% | 1.16% |
| new | identity无订阅（ABBA均值） | 75.27% | 72.77% | 3.32% |
| refresh | identity有订阅 | 71.64% | 71.55% | 0.13% |
| new | identity有订阅 | 71.39% | 70.85% | 0.76% |
| refresh | 有效修正无订阅 | 72.45% | 70.60% | 2.55% |
| new | 有效修正无订阅 | 75.45% | 73.65% | 2.38% |
| refresh | 有效修正有订阅 | 72.84% | 72.12% | 0.99% |
| new | 有效修正有订阅 | 74.96% | 72.67% | 3.06% |

开发机简单生成样本、无ROS publish的identity回调为221.9→34.2µs/条（10,000次）。**离线加速不能代替上表；所有场景均未达到30%整节点目标。** 无订阅者corrected实际publish为0，raw全频订阅/处理保留，无主动丢弃或降采样。

## 输出时序与未达指标

vision约40Hz、pose/velocity约100Hz。下表为接收间隔 `P99/最大`（ms），前→后；ABBA多窗口取各指标最大值。

| 飞机/场景 | vision | pose | velocity |
| --- | --- | --- | --- |
| refresh identity无订阅 | 27.51/28.22→27.27/28.18 | 12.62/14.08→12.26/13.46 | 12.13/13.55→11.98/14.11 |
| new identity无订阅 | 26.55/27.08→26.22/26.80 | 11.84/12.84→11.74/12.47 | 11.69/12.59→11.46/12.48 |
| refresh identity有订阅 | 27.31/27.80→27.37/28.08 | 12.30/13.69→12.12/14.09 | 12.05/13.27→11.90/13.69 |
| new identity有订阅 | 26.30/26.85→26.13/26.52 | 11.48/12.08→11.30/12.13 | 11.41/12.11→11.21/11.79 |
| refresh有效无订阅 | 27.75/28.18→27.28/27.83 | 12.77/14.03→12.40/13.37 | 12.30/13.44→11.98/12.97 |
| new有效无订阅 | 26.54/27.09→26.41/27.46 | 11.79/12.82→11.74/16.07 | 11.54/12.38→11.49/13.82 |
| refresh有效有订阅 | 27.40/28.10→27.28/28.05 | 12.20/13.15→12.37/13.17 | 11.93/13.08→11.99/12.76 |
| new有效有订阅 | 26.41/27.47→26.63/27.09 | 11.62/12.38→11.45/12.44 | 11.46/12.48→11.40/12.15 |

首轮refresh identity有订阅新版vision最大36.09ms、pose最大19.93ms；复测28.08/14.09ms。new有效无订阅新版pose最大16.07ms、旧版12.82ms。**平均频率保持不能证明P99/最大间隔全面不退化，本项严格指标未验收，异常尾部因果归属未确定。** 没有新增门限来让结果过关。

服务单次采样refresh约2.08–6.31ms、new约1.71–4.90ms，部分新版更慢（refresh有效无订阅4.93→6.23ms）。单次请求及10次生产clear均不足以证明服务尾部不退化；无超时/状态异常。

raw observer约402–403Hz（refresh）/398–399Hz（new）。refresh raw约23ms P99、约29ms最大批量到达，corrected约384–393Hz，新版约386–393Hz；new corrected约398–399Hz。旧版也有最佳努力DDS接收不足理想400Hz的情况，不能宣称零丢失/严格400Hz；每条已处理raw仍对应一次corrected publish。status计数与observer窗口不同步，数量差不能当精确丢包数。

## 部署及当前状态

- 功能提交 `2c3960e` 同步两机main。Git HTTPS多次TLS失败，最终GitHub API上传原Git对象、校验同一SHA并非强制快进main；无新分支，用户已有工作区改动保留。
- 两机现有安装器原生重建correction_interfaces/extnav_bridge。canonical、生产源和实际运行导入的build源SHA-256均为 `afb0bc794e74e3041d25edc012462142cfa677ee94924145d4db1e943fe6c73d`；symlink安装按实际导入路径核对。
- 原配置/相机/Tag/启动参数哈希保持。备份refresh `/home/nvidia/backups/extnav-task29-20261013-040123`，new对应 `040124`，名称沿用安装器。为运行测试扩展了tests/correction稀疏检出，现场标定修改保留。
- 结束前两机connected=true、armed=false；所有测试Odin/extnav/MAVROS已停止。refresh三个unit inactive；new飞行unit仍为原failed、视频/修正inactive。整套机载服务未恢复，本次不是完整飞行链路验收。
- 开发机和各机过程证据均在 `agent/codex/extnav-performance-opt/`，未提交过程文件。first/repeat/reader-results.json保存完整CPU、频率、P99/最大间隔及响应，另保存实际样本、ARM对照、状态和配置核验。

## 第二阶段建议（待用户决定）

纯计算大幅加速而整节点仅少量下降，结合任务文件已有rclpy采样，继续压缩Python数学的收益有限。**最小rclcpp候选隔离评估有价值，收益尚未证明：**

1. 先做隔离候选，保留生产Python入口和同话题/参数/QoS/40–100Hz节奏，不添加另一控制输出源。
2. 完整迁移correction2.0状态/服务、CAS/幂等、session/revision/reset、apply/clear冻结及现有covariance/z/杆臂/安装语义，不顺带适配20°安装变化。
3. 两机实际Odin逐样本对照和同负载整节点CPU、输出P99/最大间隔、服务尾部比较；不靠减订阅、降频或未测量的executor/busy polling制造收益。
4. 证明收益与契约后再单独决定是否迁移现有包构建、安装器和唯一生产入口；未证明收益不切换。

本次未实施C++。长时热稳态、视频并行、实飞及完整生产unit CPU/时序仍未验收。
