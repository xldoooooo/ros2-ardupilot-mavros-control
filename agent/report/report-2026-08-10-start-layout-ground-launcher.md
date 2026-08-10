# 启动脚本结构同步与地面站入口简报（2026-08-10）

## 当前结果

- 已从真机只读确认用户调整后的结构为根目录 `start_drone_all.sh`，以及
  `start_drone/` 下的 `start_link.sh`、`start_mavros.sh`、`start_odin.sh`、
  `start_extnav.sh`；`check.sh` 已删除。
- 真机 `start_drone_all.sh` SHA-256 为
  `4b8a3307a71fecb84904df5374b90d1d54309b18181d5a5f1cdece4b5c624452`，与本地旧名
  `start_all.sh` 完全一致，确认本次变化仅为文件结构调整。
- 已在本地新增可执行的 `start_ground_all.sh`，只加载 Jazzy、本地 install overlay 和项目
  Python 后启动 `ground_station.py`，不会启动或写入无人机。
- 已更新本地部署说明和 `MEMORY.md` 中的当前机载入口名称。

## 地面站验证

执行：

```bash
ROS_DOMAIN_ID=231 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
  bash start_ground_all.sh --check-environment
```

结果为 `guided_interfaces + rclpy available`。`tests/test_onboard_deploy.py` 定向回归为
5 passed；`bash -n` 与 ShellCheck 通过。验证使用隔离 domain，不创建地面站 GUI 控制会话，
不连接或操作真机。

`start_ground_all.sh` SHA-256 为
`5a6d5bb3e7015ae257a13bd314edb06105735b464c033fb1c5cf8d41c8254f03`，权限为 `775`。

## 未完成项与原因

首次 SSH 目录读取和哈希校验成功后，真机 `192.168.112.186` 变为网络不可达。开发机仍连接
`ATFCDSW-5g`，地址为 `192.168.112.176/24`，但目标机 ARP 状态失败，SSH/rsync 返回
`No route to host`。因此没有把真机五个启动文件伪造或手工重建为“已同步”；本地尚未生成
`start_drone/` 与 `start_drone_all.sh`，旧文件也暂未删除。

待真机重新上线后，应从真机原样 rsync 两个目标，逐文件比对 SHA-256，再以可恢复方式移除
本地完全同内容的旧名 `start_all.sh`，更新测试常量并完成全量回归。本轮未向真机写入文件，
未启动任何机载进程，也未执行解锁、起飞、模式或飞行控制操作。
