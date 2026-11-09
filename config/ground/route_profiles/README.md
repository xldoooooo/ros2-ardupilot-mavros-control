<!-- 上位机巡检航线适配配置及其维护入口。 -->
# 航线适配配置

上位机 `02` 接收时生成不可变执行副本，`03` 起飞前显示并执行该副本。手动上传不应用适配。
修改 YAML 对下一次 `02` 生效。适配失败或随后编辑了 GUI 航点，须重新下发 `02` 才能执行 `03`；
手动编辑、上传入口仍按原有方式使用。

`default.yaml` 是没有 `routId` 或未匹配专用配置时的默认规则。新建同目录的 `.yaml` 文件，
用 `route_ids: ["A", "B"]` 声明专用航线；一个 ID 只能命中一份配置。兼容旧拼写 `routeId`，
不使用单次命令 UUID `id` 选择航线。

- `point_overrides`：`point_index` 引用上位机原始 `index`；`first` 表示接收列表首点。
  `is_inspection` 控制拍照；`calibration: {tag_id: 0, continue_window: false}` 增加校准，
  `calibration: false` 取消校准。未写字段保持原值。
- `insert_points`：用 `after_point` 或 `before_point` 指定原始点号，也支持 `first`。
  填写 `position: [x,y,z]` 和 `yaw_deg`；同一锚点按文件顺序插入，不因先前插点改变引用。
  新点默认普通点，不继承原始点的巡检设置。插入巡检点还需填写双方约定的 `point_index` 和
  `photo_no`。`local_id` 可作为文件中的维护标签。
- `defaults` 可设置原始点的默认行为，`point_overrides` 随后显式覆盖。当前
  `arrival_mode` 只接受 `stop`，尚不支持连续通过。

坐标为飞机任务系绝对 ENU，距离单位米，航向为度；插入点坐标不是相对首点的偏移。
现有校准只支持 odom 绝对坐标＋直线/遇障悬停。默认首点保留原巡检和照片身份并校准 Tag7，
其后依次插入 `(5.84,0,0.8)` Tag8、`(20.03,0,0.8)` Tag1，均朝向 0°，为纯沿窗校准点；
原第二点及后续点保持顺序。只有一个原始点时，03 在最后一个插入点完成校准后降落。

普通点只执行既有入点判定；纯校准点入点后校准；巡检兼校准点入点后先触发拍照再校准。
插入的纯校准点不生成巡检照片或 `09`，其余业务点号和照片编号保持原映射。
Tag 实际位置、朝向和尺寸仍由所选飞机的 `correction/tag_pose.csv` 管理。

地面站仿真启动时为机载进程设置 `simulation_calibration:=true`，每次校准保持目标并等待
5 秒，不需要相机、Tag 服务或 extnav。该参数默认 false，实机使用原有校准和融合流程。
适配实现在 `ground_station_core/upstream/route_adapter.py`，03 编排在 `qt_ui/main_window.py`。
