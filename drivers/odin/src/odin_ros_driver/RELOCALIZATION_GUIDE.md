<!-- Current project mapping/relocalization workflow and supported configuration. -->
# Odin 建图与重定位

本指南对应项目启动入口。飞机解锁与起飞只能由用户手动执行；建图、模式切换及台架检查均在
未解锁状态进行。导航坐标和飞行限制见 [odom/map 任务坐标](../../../../docs/odom-map.md)。

## 保存地图

1. 确认本机 `config/aircraft.local.env` 的 `AIRCRAFT_ID`，停止已有 Odin/飞控进程。
2. 在项目根目录启动建图模式：

   ```bash
   ./scripts/onboard/components/start_odin.sh -c slam
   ```

3. 完成环境采集后，在同机另一个终端执行：

   ```bash
   bash drivers/odin/src/odin_ros_driver/set_param.sh save_map 1
   ```

地图保存位置由所选 `odin/slam.yaml` 中 `register_keys.mapping_result_dest_dir` 和
`mapping_result_file_name` 控制；未指定时默认在 `drivers/odin/maps/` 生成带时间的 `.bin`。
保存命令通过同机 `/tmp/odin_command.txt` 投递，须等驱动日志确认下载完成并检查文件，
脚本打印“Command sent”仅表示已写入命令文件。不要并行启动多个驱动共享该命令文件。
地图和运行标定不进入 Git。

## 配置重定位

修改本机 `config/aircraft/<AIRCRAFT_ID>/odin/reloc.yaml`：

```yaml
register_keys:
  custom_map_mode: 2
  relocalization_map_abs_path: /absolute/path/to/map.bin
```

必须使用目标设备支持的地图格式，不能靠修改后缀转换；先确认路径存在、可读且非空。
保留该飞机的标定和安装外参，不把另一架飞机整份配置直接覆盖过来。

独立台架启动：

```bash
./scripts/onboard/components/start_odin.sh -c reloc
```

完整服务的持久模式使用 `navigation.env` 的 `NAVIGATION_MODE=map`。切换正在运行的完整
机载链使用地面站导航开关或以下脚本，由脚本检查已连接、未解锁、已落地且无任务后重启
原运行组件，不直接向运行中飞行栈覆盖配置：

```bash
./scripts/onboard/switch_navigation_mode.sh map
./scripts/onboard/switch_navigation_mode.sh odom
```

## 判断是否定位成功

```bash
ros2 topic echo --once /odin1/navigation_status
```

检查实际 `mode`、`connected`、`localized`、`map_path`、`session_id` 和 `tf_age_seconds`。
地图上传成功和设备接受 map 模式不等于已定位；定位成功后需持续收到原生 `odom←map` TF。
点云与 FCU 里程计仍在 Odin odom，消费端按 TF 转入地图；不得直接把 frame 名改成 map。
机载控制器还会核对定位质量及 TF 年龄，不能仅以曾收到一次 TF 判定可飞。

起始位置应接近原建图轨迹，并有足够稳定、可辨识的环境特征；实际可定位范围须现场验证。
失败时先查地图路径/格式、设备上传日志、USB 链路、实际模式及 TF，不以已有配置替代运行证据。

## 初始位姿与源码

需要向设备提供地图初值时，厂商约定为 `custom_init_pos: [x,y,z,qx,qy,qz,qw]`，
位置以地图米为单位、姿态为归一化四元数。当前 YAML 解析器把所有 `custom_` 键去掉前缀，
再通过 `lidar_set_custom_parameter()` 下发，因此此字段会作为 `init_pos` 发送给设备。
需检查日志中的参数 ACK 及目标固件支持；Host 不验证该初值语义，也不保证它改善收敛。
地图上传使用 `lidar_set_relocalization_map()`；不要把参数发送成功当成实际定位成功。

- 地图保存、上传及连接状态：`src/host_sdk_sample.cpp`。
- 自定义模式配置与设备 ACK：`src/yaml_parser.cpp`。
- 原生 `odom←map` 发布：`include/host_sdk_sample.h`。
- 任务坐标、健康检查和模式切换：[机载部署](../../../../src/onboard_control/deploy/ONBOARD_DEPLOYMENT.md)。
