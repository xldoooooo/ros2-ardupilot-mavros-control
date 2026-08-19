# 地面站

## 新机部署完整地面站

1. apt update 
2. 安装 ROS、mavros, ArduPilot 源码（本项目仓库），参考金山文档
3. 运行安装脚本 `setup_project.sh`


```bash
ROS_DOMAIN_ID=0
ROS_LOCALHOST_ONLY=0
```

# 机载服务

## 新机载计算机部署机载服务

1. 安装ROS, mavros dataset
2. build odin驱动
3. build extnav：暂时叫vrpn （删除动捕可以不安装vrpn）
4. 按照src/onboard_control/ONBOARD_DEPLOYMENT.md的方法拉取仓库
5. build 机载服务


## 重新构建机载控制

地面开发机和飞机都在仓库根目录执行同一个命令：

```bash
./build_onboard_control.sh
```

如需同时检查依赖、运行单元测试和隔离 smoke：

```bash
./build_onboard_control.sh --verify
```

脚本会自动选择地面端 Jazzy 或飞机 Humble。它不会启动、停止或重启机载服务，也不会发送飞行
命令；构建完成后如需让运行中的飞机加载新产物，应另行选择安全窗口重启服务。

## 开机自启与自动重启

配置：

```text
/etc/systemd/system/ros2-ardupilot-onboard.service
```
新机需要手动创建，需要修改绝对路径

然后启动此服务
```bash
sudo systemctl enable ros2-ardupilot-onboard.service
```


机载计算机开机后，会等待网络就绪和系统首次校时完成，然后运行：

```text
/home/onboard/ros2-ardupilot-mavros-control/start_drone_all.sh
```

若机载服务在系统首次 NTP 校时之前启动，墙钟跳变可能使已创建的 MAVROS/ROS 定时链异常，
导致地面站无法连接。

该服务会自动重启：`start_drone_all.sh` 启动的四个组件中，任一主要进程异常退出后，脚本会清理
其余组件并以失败状态退出，systemd 在 10 秒后重启整组服务。

通过 `systemctl stop` 主动停止服务不会触发自动重启。

仓库中的通用服务示例位于
[`src/onboard_control/deploy/onboard-control.service.example`](src/onboard_control/deploy/onboard-control.service.example)。


## 配置

`start_drone_all.sh` 会自动读取以下配置文件：

```text
/etc/ros2-ardupilot/onboard.env
```
新机需要手动创建，需要修改绝对路径


```bash
ROS_DOMAIN_ID=0
ROS_LOCALHOST_ONLY=0
MAVROS_FCU_DEVICE=/dev/ttyTHS1
MAVROS_FCU_BAUD=460800
```

当前实机 FCU 串口配置为 `/dev/ttyTHS1:460800`。

使能端口：

永久提权：
```bash
sudo usermod -aG dialout $USER
```
一次性：
```
sudo chmod 777 /dev/ttyTHS1
```

## 一键启动

```bash
cd /home/onboard/ros2-ardupilot-mavros-control
./start_drone_all.sh
```

## 结束机载服务

```bash
cd /home/onboard/ros2-ardupilot-mavros-control
./stop_onboard_service.sh
```

该入口先停止 systemd 服务，再清理从其他终端手工启动的 MAVROS、Odin、extnav、
onboard_control 和相关 RViz；只有服务 inactive 且目标进程全部退出时才返回成功。

或者在启动机载服务的脚本的终端按一次 Ctrl+C, 出现
```
[startup] stopping all four components...
```
后等待，直到出现成功提示。连续Ctrl+C会导致残留


# 远程桌面

Ubuntu2404 remmina远程桌面

1. 终端输入`seahorse`，左上角加号添加“密码密钥环”，名称为“Login”，用户名 密码 均留空，直接点continue；
2. 右键Login密钥环，设为默认
3. 设置-系统-远程桌面：打开桌面共享，关闭远程登录；
4. 确认 设置-系统-用户 开启了自动登录

 