# extnav 生产补丁

这里保存飞机 `/home/nvidia/vrpn_mavros/src/extnav_bridge/` 的可审计源文件模板。
部署脚本只在飞控主服务停止的维护窗口覆盖对应源码并原生重建；不会启动、停止、解锁或起飞飞机。

`extnav_to_vision_pose.py` 始终订阅原始 Odin。没有有效修正时，corrected 话题和 MAVROS 输出都走
identity；接口包或 correction_service 缺失也不会切断原链路。
