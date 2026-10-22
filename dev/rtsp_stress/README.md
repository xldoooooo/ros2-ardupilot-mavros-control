<!-- 独立 RTSP 路由器压测工具的部署、场景和指标说明。 -->
# RTSP 路由器压测

通过真实 H.264 RTSP 推流/拉流，逐级增加流数、每路订阅数及码率，评估当前网络能否承担视频业务。
适用于更换 TP-Link 等路由器前后对比；不依赖路由器管理接口，不修改路由器、摄像头或项目服务。

## 环境与拓扑

两台 Linux 电脑位于被测网络，安装 Python 3、FFmpeg/ffprobe；服务端另需 **MediaMTX 1.20+**。
Ubuntu 的 FFmpeg 可以用 `sudo apt install ffmpeg` 安装；MediaMTX 按项目
[视频部署说明](../../video_service/README.md)安装对应架构版本。Python 只使用标准库，不需要 pip 依赖。
以下命令从项目根目录执行，使用本项目 `.venv/bin/python`。复制到其他机器时也可使用该机项目虚拟环境。

推荐第一轮把服务端接路由器 LAN 口，负载端接 Wi-Fi；再测试实际部署的双无线场景：

```text
负载端 A（多个 FFmpeg 发布者和读者）──被测路由器/AP──服务端 B（MediaMTX）
                     推流 A → B；拉流 B → A
```

`both` 模式两端均经过网络，不需要真实摄像头。A 对自己的本机地址推拉只用于功能验证。
同一子网一般测的是 AP/交换转发能力；测 WAN/NAT 时应把两端放在路由器两侧并配置连通性。
两个无线端经同一 AP 时，一份端到端单向数据需要两段无线传输；不能把此结果当成有线口或 WAN 的极限。
多个读者都是独立 RTSP 会话，但不等同于多个实际 Wi-Fi 终端争用空口。

## 快速使用

在 B 上启动独立临时服务，结束后 Ctrl+C：

```bash
.venv/bin/python dev/rtsp_stress/rtsp_stress.py serve \
  --interface enp1s0 --label 'TP-Link型号_固件_LAN服务端'
```

接口名用 `ip -br address` 查看。默认端口为 TCP 18554、UDP 18000/18001；
不会重启现有 MediaMTX，端口占用会报错，可用 `--port`、`--rtp-port` 指定其他端口。
临时服务允许网络内匿名推拉，仅在测试期间运行；UDP 测试还需允许两端协商的 UDP 端口。
服务端可用 `--duration 900` 自动结束，并通过 `--output` 指定结果位置。

在 A 上测试，替换 B 的 IP：

```bash
.venv/bin/python dev/rtsp_stress/rtsp_stress.py run \
  --url rtsp://192.168.112.111:18554/bench \
  --label 'TP-Link型号_固件_5GHz_A无线B有线' \
  --streams 1,2,4,8 --readers 1 --bitrates 8 \
  --transports tcp,udp --duration 60 --warmup 5 \
  --ping 192.168.112.1
```

默认根据到服务端的路由选择监测网卡，也可用 `--interface wlp3s0` 指定。
每个阶段都会建立新会话，先预热再测量，阶段之间默认休息 2 秒。
推流地址自动使用本轮唯一前缀，不覆盖相机路径；实际地址保存在每级 `result.json`。
`--stop-on-failure` 可在第一级不健康时结束；默认完成全部阶段以保留过载后的变化。

常见场景：

```bash
# 一路视频分发给多个订阅者：1/2/4/8 路独立拉流连接。
.venv/bin/python dev/rtsp_stress/rtsp_stress.py run \
  --url rtsp://192.168.112.111:18554/bench --streams 1 --readers 1,2,4,8 \
  --bitrates 8 --duration 120 --ping 192.168.112.1

# 比较每路 4/8/16 Mbps，多路推拉；仅测试项目正在使用的 TCP。
.venv/bin/python dev/rtsp_stress/rtsp_stress.py run \
  --url rtsp://192.168.112.111:18554/bench --streams 1,2,4 \
  --bitrates 4,8,16 --transports tcp --duration 120

# 仅压推流上行；不会启动读者。
.venv/bin/python dev/rtsp_stress/rtsp_stress.py run \
  --url rtsp://192.168.112.111:18554/bench --mode publish --streams 1,2,4

# 拉取已有直播：每级为同一地址建立 1/4/8 个连接，不启动发布者。
# --fps/--bitrates 是已有源的预期值，应按实际直播填写，不会改变源画质。
.venv/bin/python dev/rtsp_stress/rtsp_stress.py run \
  --url rtsp://192.168.112.186:8554/camera --mode read \
  --streams 1 --readers 1,4,8 --fps 30 --bitrates 8

# 已有多个源时，地址中 {stream} 展开为 1、2……；模板需要引号。
.venv/bin/python dev/rtsp_stress/rtsp_stress.py run \
  --url 'rtsp://192.168.112.111:18554/camera{stream}' --mode read \
  --streams 1,2,4 --readers 1 --fps 30 --bitrates 8
```

生成源默认 1920×1080、30 fps，可用 `--size 1280x720 --fps 60` 改变。
预先编码 8 秒 H.264 CBR，实际测试循环读取并按实时速率转发，拉流默认直接收包而不解码。
`--video /path/recording.mp4 --clip-seconds 30` 可改用真实视频的前 30 秒，按所选分辨率、帧率和码率预编码，
原录像不会修改；不加 `--video` 时使用合成测试图。
`--decode` 加入接收端软件解码，适合检查画面解码日志，但也会增加主机负载。
本工具不采集相机或启动飞控；读取已有直播要求该直播已经可用。

## mini-ground 拉流后再分发

本机 A 提供录像直播，mini-ground B 拉取后原码转发，C 可以是本机、其他电脑或多个 FFmpeg 会话。
这里固定测试路径，方便多端协作；`--fixed-paths` 只用于专用压测地址，不能填生产相机路径。
示例按 A=`192.168.112.176`、B=`192.168.112.111`，对应终端保持运行：

```bash
# A 的终端 1：提供源 RTSP 服务。
.venv/bin/python dev/rtsp_stress/rtsp_stress.py serve \
  --port 18557 --rtp-port 18006 --interface wlp3s0

# A 的终端 2：真实录像预编码完成后会打印 Publishing，再启动 B 的转发。
.venv/bin/python dev/rtsp_stress/rtsp_stress.py run \
  --url rtsp://127.0.0.1:18557/router-bench-source --fixed-paths --mode publish \
  --streams 1 --bitrates 20 --fps 60 --clip-seconds 30 --duration 1800 \
  --video '/home/nvidia/Videos/recording-202608+09/recording-20260901-170805-187709.mp4'

# B 的终端 1：独立分发服务器。
.venv/bin/python dev/rtsp_stress/rtsp_stress.py serve --interface wlo1

# B 的终端 2：主动拉 A，再推给本机服务端；只转发，不解码/编码。
.venv/bin/python dev/rtsp_stress/rtsp_stress.py run \
  --source-url rtsp://192.168.112.176:18557/router-bench-source \
  --url rtsp://127.0.0.1:18554/router-bench-relay --fixed-paths --mode publish \
  --streams 1 --bitrates 20 --fps 60 --duration 1800 --interface wlo1

# C：逐级增加从 B 拉流的订阅数。可在多台 C 上同时执行，订阅数需相加。
.venv/bin/python dev/rtsp_stress/rtsp_stress.py run \
  --url rtsp://192.168.112.111:18554/router-bench-relay --mode read \
  --streams 1 --readers 1,2,4,8 --bitrates 20 --fps 60 \
  --duration 60 --warmup 8 --ping 192.168.112.1
```

`--source-transport tcp|udp` 指定 B 从 A 拉流的传输方式，默认 TCP；`--transports` 指定输出/读者的传输方式。
中继和读取已有源时，`--bitrates` 只填一个预期码率，不对输入限速或转码。
多个不同输入可用 `--source-url 'rtsp://A:18557/source{stream}'` 加固定输出模板。
先启动源与中继、确认无错误后再启动读者；脚本保留失败结果，不会自动重连掩盖故障。

直接执行激进测试时，在开发机和 hp-desktop 各执行下面一组固定配置（各 8 路，共 16 路）：

```bash
.venv/bin/python dev/rtsp_stress/rtsp_stress.py run \
  --url rtsp://192.168.112.111:18554/router-bench-relay --mode read \
  --streams 1 --readers 8 --bitrates 20 --fps 60 \
  --duration 180 --warmup 10 --cooldown 0 --timeout 20 \
  --ping 192.168.112.1 --label '路由器型号_16订阅_双客户端主机'
```

目标为 mini-ground 入站一路 20 Mbps、出站十六路共 320 Mbps 的视频载荷。
这是过载观察场景，不意味着路由器应当达到这个指标；换路由器可原参数复测。

## 结果与判断

默认保存在工具目录 `results/`，每轮单独建目录且已忽略 Git：

| 文件 | 内容 |
| --- | --- |
| `environment.json` | 场景标签、参数、地址、路由、Wi-Fi 信息、FFmpeg 版本 |
| `summary.md / .csv / .json` | 各级汇总，JSON 包含逐路指标 |
| `source-*.mp4` | 复用的合成源，实测视频包平均码率写入结果 |
| `阶段/samples.jsonl` | 每秒帧进度、主机/FFmpeg CPU、内存、网卡收发计数 |
| `阶段/plan.json / result.json` | 初始地址/负载计划；各路实测 fps/视频净码率、帧比例、启动耗时、最长进度停顿、异常退出、ping 结果 |
| `阶段/publish-*.log / read-*.log / ping.log` | 原始诊断日志 |
| 服务端 `server-*/` | 服务端环境、MediaMTX 日志、每秒 CPU/内存/网卡计数 |

`healthy=true` 默认要求所有推/拉进程未提前退出、测量窗口帧数达到预期的 95%、
采样所见无帧进度最长停顿不超过 3 秒。可通过 `--min-frame-ratio` 和 `--max-stall` 调整。
直接转发时通过 FFmpeg 的 `framecrc` 统计实际视频包，推流额外输出同一份压缩包供计数，
不做二次编码；`--decode` 时使用 FFmpeg 的解码帧进度。未解码不保证每帧可正确解码。
停顿按秒采样，是近似观测；短于 60 秒的阶段更易受窗口边界影响，只适合功能验证。
UDP 的 `rtp_missed_packets` 是 FFmpeg 日志中的局部漏包计数，不能当作完整丢包率。
`warning_lines` 包含普通探测警告，需要结合日志判断。

N 路、每路 B Mbps、每路 R 个读者时，服务端视频载荷入站约 `N×B`，出站约 `N×R×B` Mbps；
协议开销及其他业务会增加实际流量。网卡吞吐记录的是全网卡数据，不能直接当作本次视频净码率。
未解码的会话另提供 `video_mbps`，由实际转发的压缩视频包字节数计算，不含网络协议开销。
CPU 的 `*_one_core_percent` 以单核 100% 计，主机 CPU 以整机 100% 计；同时查看服务端采样，避免把主机瓶颈归因于路由器。

换路由器时，保持两端硬件、频段/带宽、位置、码率、帧率、流数和时长一致，修改 `--label` 标明型号/固件。
先扫阶找到拐点，再对目标负载持续测试 10–30 分钟并重复几轮，结合网关 RTT、停顿、CPU 和实际画面判断容量。
工具不直接测量玻璃到玻璃延迟，也不获取路由器 CPU/内存；单轮通过不构成最大稳定容量结论。

退出码：0=全部阶段健康，1=存在不健康阶段，2=参数/依赖/执行错误，130=中断。
Ctrl+C/SIGTERM 会停止本轮子进程并保存已采集的部分结果；已有项目服务不受影响。
