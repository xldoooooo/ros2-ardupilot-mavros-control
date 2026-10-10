<!-- UQ212 机械近似外参更新：尺寸推导、验证范围与部署状态。 -->
# UQ212 下视相机机械近似外参更新

日期：2026-09-17。

## 用户确认与推导

用户确认相机基座在 Odin 模块前方 41 mm、底面下方 46 mm，横向对齐模块宽度中点；
镜头中心在基座下方约 12 mm，安装按横平竖直、无偏航角近似。
参考点是模块前表面下边缘的宽度中点。官方图纸：
https://manifoldtechltd.github.io/wiki/odin_series/odin1/5.%20Data%20output_.html

- structure1.png：IMU 在前表面后方 5.57±0.5 mm。
- structure2.png：模块宽 100 mm，IMU 正视图距左边缘 21.03 mm、距底面 9.26 mm。
- coordinate.png：IMU +X 朝模块前方、+Y 朝正视图右侧、+Z 朝上。

相机光学系到 Odin IMU：`p_imu = R * p_camera + t`。

```text
t_x = (41 + 5.57)/1000 = 0.04657 m
t_y = (100/2 - 21.03)/1000 = 0.02897 m
t_z = -(46 + 12 + 9.26)/1000 = -0.06726 m

T_imu_camera =
[ 0 -1  0  0.04657 ]
[-1  0  0  0.02897 ]
[ 0  0 -1 -0.06726 ]
[ 0  0  0  1       ]
```

画面右方为 IMU -Y，画面上方为 +X，光轴为 -Z；无额外偏航或倾斜。
这是固定安装外参，不是机体倾斜时仍保持地垂线的云台模型。
平移模长约 86.787 mm，水平杆臂约 54.845 mm；参考中心是 Odin IMU，不是 FCU。

## 修改范围

更新活动 `correction_service/config/extrinsics.yaml` 及 UQ212 同名档案；
来源改为本报告，移除不再适用于新机械近似的旧标定收敛指标。
保持 `quality.passed=false`，状态为 `mechanical_estimate_axis_aligned_unverified`。
更新说明、MEMORY 与相关配置/方向测试。未修改内参、Tag 基变换、FCU 杆臂或飞控逻辑。

Wasintek 外参未修改，修改前后 SHA-256 均为：
`07b59f92545ad0ccb3d7af18b9e140ab2178190743eebd39d6bfd74fa3fd4b0e`。

## 验证

项目 `.venv/bin/python -m pytest -q tests/test_correction_service.py tests/test_correction_tag_orientation.py correction_service/test/test_correction_package.py`：30 passed。
包括配置解析、UQ212 活动/档案一致、完整 SE(3) 链，以及官方 Tag 像素下多个位置/航向的
新 UQ212 机械模型和旧 Wasintek 标定模型回归。保留独立冻结的旧矩阵，避免丢失历史方向覆盖。
旋转正交且行列式 +1；光轴和画面上方向符合用户最终确认。

上述是软件与合成图像回归，不代表实物绝对精度验收；镜头中心暂近似光心，
12 mm 为近似尺寸，实物角度和制造/测量误差尚未验证。

## 部署状态与边界

本次更新开发机仓库并同步 main；未连接、部署或重启任何飞机。
refresh/new 飞机均待同步本次改动，部署标记已写入 MEMORY。
运行中服务需重新加载配置并重新采样，不复用旧窗口/旧修正；服务重启不会自动清除
extnav active。本次未改变实机 active、未解锁、未起飞。
工作区原有根 README、TODO、部署文档迁移和其他未跟踪任务/报告保留，不纳入本次提交。
