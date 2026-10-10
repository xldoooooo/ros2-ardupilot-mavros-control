<!-- 用户授权原始30张UQ212实测内参替换理论配置及双端部署核验。 -->
# UQ212 实测内参投入生产

日期：2026-09-17。用户要求以本次标定替换临时理论内参并同步双端。
双端为开发机与refresh；new飞机未连接，MEMORY记录待同步。

## 采用的数据

原始 `/home/nvidia/camera_int_calib/uq212_runs/20260917-011117-295510/intrinsics.yaml`，
全部30张，未采用排除20/21后的28张诊断结果，未改动标定算法或原始数据。
活动config/intrinsics.yaml与config/UQ212/intrinsics.yaml均逐字节复制原YAML。
SHA-256：1de1fc452348666ddc75590739cf7a2ed5631c5476e13f7e3575de4ebd386caa。

K：fx942.1240709375937、fy942.3489849358813、cx954.1999309924863、cy538.6404506117269。
D：[0.03626268524324754,-0.056048601794863075,0.0010844756398792788,
-0.0010052529149732062,0.00958122664915688]。RMS0.8863085437393902px。
保留 calibrated_pending_independent_validation，不把用户授权部署等同于独立精度验收。
已知20/21样本异常与四角约束不足仍存在，前轮审查结论不改变。

## 部署与验证

- 两端均 colcon build --packages-select correction_service 成功。
- 两端活动与UQ212档案的源码/install配置逐字节一致。
- refresh 独立修正服务重启，运行config_fingerprint与开发机一致：
  34489d9b900618fcb85f20c27e3474d7af16a71595374c2847e80556d72eac55。
- 重启后接口2.0、idle、active=false、window_count=0、resources_released=true、无last_error。
- 开发机41项相关测试通过，refresh配置/包级21项测试通过。
- 首次测试暴露Wasintek方向回归错误借用了活动UQ212的K；改为读取Wasintek自身档案K，
  保持原8mm门限及所有算法不变，随后通过。此修改仅限测试输入，不改生产求解。
- Wasintek内参备份双端哈希保持
  9da07fcff90f576edc60dc23aeb0dbb093e3e2d7e24dd99d2a2029a98ebc8749。
- 开发机理论配置备份于agent/codex/uq212-production-intrinsics-20260917/theoretical-before.yaml；
  refresh源码/install旧配置备份于agent/codex/intrinsics-before-measured-<时间戳>/。

## 运行边界

重启前修正任务inactive，旧窗口stale且有一次成功采样/旧应用记录；重启后重新建立空窗口。
本次未调用extnav应用/清除服务。extnav状态读取超时，因此不能声称已核对其实际当前值；
需用户用新内参重新采样并应用。没有重启其他服务，没有解锁或起飞，没有实际新定位精度验收。
主项目原有README/TODO/部署文档等未提交改动未纳入本次提交。
