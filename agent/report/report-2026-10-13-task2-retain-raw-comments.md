<!-- Restore disabled raw-cloud code as comments for easy re-enablement. -->
# Odin 原始云代码保留为注释

日期：2026-10-13。

按用户要求，将上次删除的9行ROS1/ROS2发布调用、初始化及成员声明恢复到原位置并全部注释。附一行英文恢复说明，README注明取消整组注释后重新构建、重启即可恢复原始cloud_slam。SDK main提交 `3ce35d8`。

验证：删除本次新增的注释文本后，header与修改前逐字相同；diff检查通过。此次仅新增注释，不改变编译行为，无需重复硬件测试或重新安装二进制。源码已同步两机；原始云继续关闭，new之前安装的仅down版本仍待旧进程重启生效，未中断当前机载链路。
