# Windows USB 摄像头调参与 RTSP 推流

## 第一步：安装软件

在连接摄像头的 Windows 电脑上安装：

1. [OBS Studio](https://obsproject.com/download)：用于打开摄像头、调整参数和推流；
2. [MediaMTX](https://github.com/bluenviron/mediamtx/releases/latest)：下载文件名包含 `windows_amd64` 的压缩包；(https://github.com/bluenviron/mediamtx/releases/latest）
3. [VLC](https://www.videolan.org/vlc/download-windows.html)：用于测试拉流。

将 MediaMTX 解压, 应能看到：

```text
mediamtx.exe
mediamtx.yml
```

## 第二步：在 OBS 中打开摄像头

1. 将摄像头用USB数据线连接到电脑；
2. 打开 OBS Studio；在左下角“来源”区域点击 `+`；选择“视频采集设备”；选择“新建”；
6. 在“设备”中选择 `Wasintek camera`（或`USB Camera` 或 `USB Video Device`）；

## 第三步：配置摄像头画质与镜头参数

点击“视频采集设备”的齿轮图标，打开设置界面

| 配置项 | 建议值 |
|---|---|
| 分辨率/FPS 类型 | 自定义 |
| 分辨率 | `1920x1080` |
| FPS | `60` |
| 视频格式 | 优先 `H264`；没有时选择 `MJPEG` 或 `MJPG` |
| 色彩空间 | 默认 |
| 色彩范围 | 默认 |
| 缓冲 | 自动检测 |

镜头参数：
- 曝光 `Exposure`；
- 增益 `Gain`；
- 亮度 `Brightness`；
- 对比度 `Contrast`；
- 饱和度 `Saturation`；
- 锐度 `Sharpness`；
- 白平衡 `White Balance`；

**注意，镜头参数配置在摄像头断电或重启后会恢复为默认值。**

设置界面示意图：

![image-20260826205229260](/home/nvidia/scq/projects/ros2-ardupilot-sitl-hardware/docs/assets/image-20260826205229260.png)

![image-20260826205252297](/home/nvidia/scq/projects/ros2-ardupilot-sitl-hardware/docs/assets/image-20260826205252297.png)





## 第五步：配置 OBS 输出

### 5.1 配置分辨率和帧率

打开“OBS → 设置 → 视频”，填写：

| 配置项 | 设置值 |
|---|---|
| 基础（画布）分辨率 | `1920x1080` |
| 输出（缩放）分辨率 | `1920x1080` |
| 常用 FPS 值 | `60` |

### 5.2 配置编码格式

打开“OBS → 设置 → 输出”，填写：

| 配置项 | 设置值 |
|---|---|
| 输出模式 | 简单 |
| 视频编码器 | 选择名称中包含 `H.264` 的编码器 |
| 视频码率 | 自行设置即可 |

## 第六步：配置MediaMTX

1. 打开`mediamtx.yml`

2. 搜索 `rtspTransports:`，修改为：

   ```yaml
   rtspTransports: [tcp]
   ```

3. 搜索 `rtspAddress:`，修改为希望使用的端口，例如：

   ```yaml
   rtspAddress: :8556
   ```

4. 保存，然后运行`mediamtx.exe`并保持 MediaMTX 一直打开。

## 第七步：推流

1. 打开“OBS → 设置 → 推流”；
2. 填写：

| 配置项 | 设置值 |
|---|---|
| 服务 | 自定义 |
| 服务器 | `rtmp://127.0.0.1/camera` |
| 串流密钥/推流码 | 留空 |
| 使用身份验证 | 不勾选 |

3. 点击右下角“开始推流” 并保持 OBS 和 MediaMTX 同时运行。

然后查看推流设备的ip地址并得到拉流地址，例如推流电脑的 IPv4 地址是 `192.168.112.101`，那么按上述示例填写后，得到的最终拉流地址就是：

```text
rtsp://192.168.112.101:8556/camera
```

## 第九步：在另一台设备上拉流

使用 VLC 测试：

1. 保持推流电脑上的 OBS 和 MediaMTX 正在运行；

2. 在另一台设备上打开 VLC；点击“媒体 → 打开网络串流”；输入拉流地址后点击播放

   ```text
   rtsp://192.168.112.101:8556/camera
   ```
