# Task29 旧 T 公式对照与响应式修正面板简报

## 任务与边界

- 用户最新复测：仅用 Tag 0 首次校准，把飞控中心放到 Tag 0 中心后，地面站仍显示约
  `x=0.24 m, y=0.15 m`。
- 本轮按要求不改已有首次标定、滑窗配准、extnav 修正、FCU/MAVROS 发布或门限，只增加旧
  Odin 杆臂 `T` 处理公式的并行显示，用于同一时刻直接对照。
- 响应式 GUI 由独立子代理实现，主代理复核、视觉检查并统一测试。
- 本轮只在本地开发机修改和验证；未连接、部署、重启、解锁或起飞实机。

## 旧 T 公式对照

Task29 已证明修正有效时，当前与旧公式为：

```text
current: p_F.xy = (p_corrected - R_corrected*T).xy
old:     p_old.xy = (p_corrected + T - R_corrected*T).xy

p_old.xy - p_F.xy = T.xy
```

因此不必复制 extnav 计算链或增加第二个 ROS 发布器。地面面板客户端从实际收到的同一条
`/extnav/pose_fcu` 快照只读派生：

```text
x_old = x_current + T_x
y_old = y_current + T_y
z_old = z_current
yaw_old = yaw_current
```

当前生产杆臂为 `T=(+0.06,-0.03,+0.05)m`。若同一快照的当前输出是用户本次报告的
`(0.24,0.15)m`，旧公式对照应显示 `(0.30,0.12)m`。这只是公式差值，不是对尚未同步采集的
历史两次实测进行推算，也不说明哪套公式更接近物理真值。

对照值只有在以下条件全部成立时才显示：

- extnav 状态和 `pose_fcu` 均新鲜；
- active correction 有效；
- final sample 可用且标记为 corrected；
- final sample 的 revision/session 与当前 active revision/session 一致；
- `pose_fcu` 的本机接收时刻不早于当前 extnav 状态，避开切换瞬间的跨话题旧样本；
- 位姿和杆臂均为有限值。

修正未生效时，identity 分支本来就沿用旧局部原点公式，Task29 前后没有这项差异；面板会明确
显示原因而不是伪造第二组数值。revision/session 尚未对齐时同样等待新鲜样本。

该诊断：

- 不发布 ROS 话题；
- 不写 extnav；
- 不改变 correction candidate；
- 不改变 `/extnav/pose_fcu` 或 `/mavros/vision_pose/pose`；
- 不改变 MAVROS EKF 输入或输出。

## 响应式布局

原窗口最小尺寸为 `940×700`，顶部命令区固定四列，长状态文本的 size hint 还会继续撑宽滚动
内容。修改后：

- 最小尺寸降为 `560×520`；
- 以 820 px 为响应式断点；
- 窄窗中两个输入项分成两行；
- 首次/下一次、停止/清空分别按两列排列，“应用当前滑窗结果”占整行；
- 候选质量和 extnav 状态从左右双栏改为上下排列；
- 长数值标签允许换行，不再把滚动区撑出水平滚动条；
- 宽窗仍恢复原四列命令区和 3:2 状态双栏。

Qt offscreen 在精确 `560×520` 下实测：viewport 为 `516×453`，body 宽度为 516，水平滚动最大值
为 0，五个顶部操作按钮横向均完整可见。窄窗增加的垂直高度由既有纵向滚动区承载。
另对 560、620、819、820、821、900、1180 px 宽度逐点跨越断点检查，全部保持水平滚动最大值为
0 且五个按钮横向完整可见。

## 修改文件

- `correction_service/correction_service/ros_client.py`
  - 只读派生旧 `+T_xy` 对照；增加新鲜度、valid、revision/session 和有限值门控。
- `correction_service/correction_service/correction_panel.py`
  - 增加旧 T 对照行；实现 820 px 断点下的响应式命令区和状态区。
- `tests/test_correction_panel.py`
  - 覆盖公式差值、不修改输入、identity/revision 拒绝、GUI 展示和窄窗重排。
- `correction_service/README.md`
  - 记录诊断语义、非发布边界、响应式布局和最新实测风险。
- `MEMORY.md`
  - 更新当前真实基线和仍未解决的 Tag0 中心偏差。

没有修改以下生产算法文件：

- `correction_service/correction_service/geometry.py`
- `correction_service/correction_service/estimator.py`
- `correction_service/correction_service/node.py`
- `correction_service/extnav_patch/extnav_to_vision_pose.py`
- `src/correction_interfaces/` 下的消息与服务

没有新增依赖或改变接口版本。

## 本地验证

```text
Ruff format --check                         passed
Ruff check                                  passed
git diff --check                            passed
tests/test_correction_panel.py               10 passed
tests/test_qt_gui.py                         51 passed
tests + correction_service/test              242 passed
colcon build correction_interfaces + correction_service   2 packages passed
colcon test-result                           23 tests, 0 failures
```

另在 560×520 窄窗中将页面滚动到对照区做了截图复核：旧公式的 x/y/z/yaw、固定 `Δxy` 和 revision
均完整可见，水平滚动最大值仍为 0。过程截图保存在忽略提交的 `agent/codex/` 下。

## 结论与后续实测

本轮交付的是严格隔离的 A/B 观察能力，不能仅凭它宣称 `(0.24,0.15)m` 根因已确定。下一轮部署
地面面板后，应在同一次校准、同一静止摆放、同一 active revision/session 下同时记录：

1. 当前 `/extnav/pose_fcu`；
2. 面板旧 `+T_xy` 对照；
3. corrected Odin；
4. MAVROS EKF final；
5. 对应 correction result、raw/PnP/C_full 日志。

由于旧/当前两行来自同一条 FCU 输入快照，它可以准确回答固定 `+T_xy` 对这一次结果贡献了多少；
其余偏差仍需再按 Tag 中心摆放、检测/PnP、相机外参、时间匹配、首次 `C_full` 和 EKF 融合逐层
定位。实际 Wayland/高 DPI 桌面拖拽观感尚未人工验收，但 Qt 几何和离屏视觉验证均已通过。
