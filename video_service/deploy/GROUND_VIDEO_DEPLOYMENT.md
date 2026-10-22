<!-- Ground relay dependencies, GPU driver requirements and deployment verification. -->
# 地面视频处理部署（Ubuntu 24.04 / amd64）

本文对应 `video_service/ground_relay.py` 的 Intel、NVIDIA、AMD、CPU 后端。
操作在项目根目录执行；使用说明与参数见[视频 README](../README.md#地面去畸变中继)。
机载 Jetson 的部署使用[机载视频说明](../README.md#新飞机快速部署ubuntu-2404--jazzy--arm64)，
不能把桌面 NVIDIA 后端的验证直接等同于 Jetson 兼容性。

## 通用依赖

```bash
sudo apt update
sudo apt install -y ffmpeg python3-venv python3-opencv python3-numpy python3-yaml ocl-icd-libopencl1 clinfo

# 已有项目环境时保留；新环境需要访问上述系统 Python 包。
test -x .venv/bin/python || python3 -m venv --system-site-packages .venv
./.venv/bin/python -m pip install -r requirements-gui.txt
./.venv/bin/python -c 'import av, cv2, numpy, yaml; print("CPU relay dependencies OK")'
```

完整地面站入口 `scripts/ground/setup_ground_station.sh` 会创建项目环境并安装
`requirements-gui.txt`（含 PyAV），检查 FFmpeg/FFprobe/MediaMTX，并构建 ROS 包。
**该入口不安装或升级 NVIDIA 驱动。** GPU 驱动按下面对应硬件步骤准备；
CPU 后端无需显卡驱动。RTSP 发布另需按视频 README 安装并配置 MediaMTX，
文件输出自检则不需要启动 MediaMTX。

## Intel 核显

新机器优先安装系统包：

```bash
sudo apt install -y intel-media-va-driver intel-opencl-icd
ls -l /dev/dri/renderD*
clinfo --list
```

启动用户需要对应 render 设备的读写权限；调整用户组后重新登录。
中继默认使用 `/dev/dri/renderD128`，多显卡机器可通过 `--device` 指定。
硬件、驱动和 FFmpeg 必须支持 VAAPI/OpenCL 表面共享，程序会实际探测整条链路。

没有系统 ICD 时，也可部署完整的用户级运行时：

```text
~/.local/share/ros2-ardupilot-video/intel-opencl/
└── usr/lib/<架构>/
    ├── intel-opencl/libigdrcl.so
    └── 驱动依赖库及其链接
```

运行时应包含适配当前系统/架构的 Intel OpenCL 驱动及依赖，不能只复制一个 `.so`。
mini-ground 已部署此目录，启动时自动加载，无需手工设置环境变量。
若设置了 `XDG_DATA_HOME`，默认目录跟随该变量；自定义位置使用
`--intel-opencl-runtime /path/to/runtime`。

加载顺序是系统/调用者环境优先，失败后再试用户目录。自动设置的库路径和 ICD 仅传给
Intel 子进程，不影响 NVIDIA/CPU；普通终端的 `clinfo` 不一定能列出这份用户级驱动，
应通过中继实际测试。程序不会自行下载驱动，用户级目录不随 Git 分发。

## NVIDIA 显卡

没有 RTX 3080 型号限制，但显卡必须支持输入格式的 **NVDEC** 解码和 **H.264 NVENC** 编码，
同时提供可用的 NVIDIA OpenCL GPU 设备。核对具体型号的
[官方编解码支持表](https://developer.nvidia.com/video-encode-decode-support-matrix)。

需要完整的 NVIDIA 驱动及对应用户态库：CUDA Driver API、NVDEC、NVENC、OpenCL ICD。
Ubuntu 包通常由同一驱动分支的 `nvidia-driver-*` 及 `libnvidia-compute-*`、
`libnvidia-decode-*`、`libnvidia-encode-*` 提供；内核模块和这些库应保持匹配。
仅安装 OpenCL 通用加载器 `ocl-icd-libopencl1` 不会提供 NVIDIA GPU 驱动。

新机器按 Ubuntu 推荐分支安装：

```bash
sudo apt install -y ubuntu-drivers-common
ubuntu-drivers devices
sudo ubuntu-drivers install
```

驱动安装后按系统提示完成必要的重启，再运行 `nvidia-smi`。现有驱动已经通过下面检查时，
无需为了本功能重装。安装方法见 [Ubuntu 官方指南](https://ubuntu.com/server/docs/nvidia-drivers-installation/)。

**驱动最低版本由所用 FFmpeg 编译时的 NVIDIA Codec API 决定，不固定为某个版本号。**
若提示 `Driver does not support the required nvenc API version`，应匹配驱动与 FFmpeg；
不能仅依据 GPU 型号或 `nvidia-smi` 正常就判定可用。
当前已验证组合为 RTX 3080、驱动 580.178.04、Ubuntu FFmpeg 6.1.1；这是验证基线，
不是要求所有机器使用 3080 或 580。版本关系见
[NVIDIA FFmpeg 部署说明](https://docs.nvidia.com/video-technologies/video-codec-sdk/13.1/ffmpeg-with-nvidia-gpu/index.html)。

本路线运行时**不需要另装 CUDA Toolkit、CuPy 或 PyNvVideoCodec**。
它使用 NVDEC → OpenCL 去畸变 → NVENC，中间包含主存搬运。
系统 FFmpeg 需要 CUDA/NVDEC、`h264_nvenc` 和 `program_opencl` 支持：

```bash
nvidia-smi
clinfo --list
ffmpeg -hide_banner -hwaccels
ffmpeg -hide_banner -encoders 2>/dev/null | grep h264_nvenc
ffmpeg -hide_banner -filters 2>/dev/null | grep program_opencl
```

应能看到 NVIDIA OpenCL 设备、CUDA 硬件加速和上述编码器/滤镜。能力列表只说明接口存在，
最终以实际视频处理成功为准。多显卡机器可配置 `--cuda-device` 和 `--opencl-device`，
分别选择匹配的 CUDA GPU 与 OpenCL 平台/设备。

## AMD 核显

使用 Mesa VAAPI 解码/编码和 Vulkan/libplacebo 去畸变；无需 ROCm、AMD 专有驱动或 OpenCL。
Ubuntu 24.04 上需要可用的 `amdgpu` 内核驱动与以下系统包：

```bash
sudo apt install -y mesa-va-drivers mesa-vulkan-drivers ffmpeg
ls -l /dev/dri/renderD*
ffmpeg -hide_banner -filters 2>/dev/null | grep libplacebo
ffmpeg -hide_banner -encoders 2>/dev/null | grep h264_vaapi
```

启动用户需有 render 设备读写权限，多显卡使用 `--device /dev/dri/renderDxxx` 选择 AMD 设备。
FFmpeg 必须支持从 VAAPI 派生 Vulkan 设备、硬件帧映射及 `libplacebo` 的 `custom_shader_path`。
GPU 本身需支持输入格式的 VAAPI 解码和 H.264 编码；不同代际/发行版仍须实际处理验收。

已验证 laptop / hp-desktop 的 Ryzen 7 8845HS / Radeon 780M，Mesa 25.2.8、FFmpeg 6.1.1、
libplacebo 6.338。该机器复用现有组件，未安装新驱动。
当前输出通过主存搬运后交给 VAAPI 编码，避免直接把不可渲染的 AMD 编码表面作为 Vulkan 输出。
这不是零拷贝路线；不为旧驱动强行增加专用后端，不兼容时沿用 CPU 回退及面板提醒。
着色器位于 `video_service/shaders/undistort.hook`，与其他后端共用同一份镜头标定。

## 实际处理验收与回退

使用与所选标定尺寸一致的8位 H.264 视频测试；下面默认 Wasintek 1920×1080 标定：

```bash
video_check_dir="$(mktemp -d)"
./.venv/bin/python video_service/ground_relay.py \
  --input /path/to/1920x1080-h264.mp4 \
  --output "${video_check_dir}/corrected.mp4" \
  --backend auto --frames 120
ffmpeg -v error -xerror -i "${video_check_dir}/corrected.mp4" -f null -
```

默认自动选择顺序为 Intel → NVIDIA → AMD → CPU。
可指定 `--backend intel`、`--backend nvidia` 或 `--backend amd` 检查目标 GPU。
**指定 GPU 时依然允许 CPU 回退，因此退出码为0不等于GPU验收通过：须确认日志明确显示
“使用 Intel”、“使用 NVIDIA”或“使用 AMD”。** 用户级Intel运行时被选中时还会显示“已自动加载”。

无法使用GPU时，日志及本机摄像头面板会提示CPU兼容模式；CPU缺少PyAV时会明确给出安装指引。
运行中切换后端会中断流，失败录像保留为 `*.gpu-failed-*`；CPU可能低于源帧率并增加延迟。
保留时间戳和成功解码不代表源录像完整。自检不通过时，按提示检查驱动、设备权限、
输入与标定尺寸、FFmpeg能力；不要把CPU回退的成功当成GPU部署成功。
