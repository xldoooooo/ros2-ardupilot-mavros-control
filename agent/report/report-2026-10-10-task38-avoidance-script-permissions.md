<!-- 避障启停脚本执行权限修复与refresh验证；未启动任何飞行进程。 -->
# Task38：避障启停脚本执行权限修复

日期：2026-10-10。

用户在refresh执行`./start_avoidance.sh`收到`Permission denied`。原因是两个新脚本
在Git中都是100644，部署时也没有执行位；此前`bash scripts/onboard/start_avoidance.sh`
验证绕过了这个问题，遗漏了直接执行入口。

将`start_avoidance.sh`和`stop_avoidance.sh`修正为Git模式100755，同时在refresh原文件上
补充执行权限。没有覆盖飞机脚本内容，没有重启或启动组件，也没有申请控制租约、解锁或起飞。

验证：

- 本地两个脚本`bash -n`通过，Git diff只包含100644→100755的模式变化。
- refresh两个脚本均具备执行权限，`./start_avoidance.sh --check`成功，输出安装/源码清单
  校验通过且未启动进程。
- MEMORY已维护脚本权限基线；new尚未同步，需要下次补齐Task38代码与此权限修改。

用户可在原终端直接重新执行`./start_avoidance.sh`。无需重新构建。
