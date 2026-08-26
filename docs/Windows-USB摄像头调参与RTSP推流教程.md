# Windows USB 摄像头调参与 RTSP 推流教程

本教程使用以下示例地址：

```text
rtsp://192.168.112.101:8556/camera
```

其中：

- `192.168.112.101`：运行摄像头的 Windows 电脑的实际局域网 IP；
- `8556`：RTSP 端口，可自行修改；
- `camera`：视频路径，可自行修改。

## 第一步：安装软件

在连接摄像头的 Windows 电脑上安装：

1. [OBS Studio](https://obsproject.com/download)：用于打开摄像头、调整参数和推流；
2. [MediaMTX](https://github.com/bluenviron/mediamtx/releases/latest)：下载文件名包含 `windows_amd64` 的压缩包；
3. [VLC](https://www.videolan.org/vlc/download-windows.html)：用于测试拉流。

将 MediaMTX 解压到：

```text
C:\CameraRTSP
```

解压后应能看到：

```text
C:\CameraRTSP\mediamtx.exe
C:\CameraRTSP\mediamtx.yml
```

## 第二步：在 OBS 中打开摄像头

1. 将摄像头插入 Windows 电脑；
2. 打开 OBS Studio；
3. 在左下角“来源”区域点击 `+`；
4. 选择“视频采集设备”；
5. 选择“新建”，点击“确定”；
6. 在“设备”中选择 `Wasintek camera`、`USB Camera` 或 `USB Video Device`；
7. 确认 OBS 中已经显示摄像头画面。

## 第三步：配置摄像头画质

在 OBS 的“视频采集设备属性”中设置：

| 配置项 | 建议值 |
|---|---|
| 分辨率/FPS 类型 | 自定义 |
| 分辨率 | `1920x1080` |
| FPS | `30` |
| 视频格式 | 优先 `H264`；没有时选择 `MJPEG` 或 `MJPG` |
| 色彩空间 | 默认 |
| 色彩范围 | 默认 |
| 缓冲 | 自动检测 |

设置完成后点击“确定”。

## 第四步：配置曝光、增益等参数

1. 在 OBS 的“来源”中双击摄像头；
2. 点击“配置视频”（英文为 `Configure Video`）；
3. 在弹出的摄像头参数窗口中设置：
   - 曝光 `Exposure`；
   - 增益 `Gain`；
   - 亮度 `Brightness`；
   - 对比度 `Contrast`；
   - 饱和度 `Saturation`；
   - 锐度 `Sharpness`；
   - 白平衡 `White Balance`；
   - 电源频率选择 `50 Hz`；
4. 如需手动曝光，取消曝光旁边的“自动”勾选；
5. 调整时直接观察 OBS 画面；
6. 设置完成后点击“应用”，再点击“确定”。

## 第五步：配置 OBS 输出

### 5.1 配置分辨率和帧率

打开“OBS → 设置 → 视频”，填写：

| 配置项 | 设置值 |
|---|---|
| 基础（画布）分辨率 | `1920x1080` |
| 输出（缩放）分辨率 | `1920x1080` |
| 常用 FPS 值 | `30` |

### 5.2 配置编码格式

打开“OBS → 设置 → 输出”，填写：

| 配置项 | 设置值 |
|---|---|
| 输出模式 | 简单 |
| 视频编码器 | 选择名称中包含 `H.264` 的编码器 |
| 视频码率 | `6000 Kbps` |

设置完成后点击“应用”，再点击“确定”。

## 第六步：配置并打开 MediaMTX

1. 使用记事本打开：

   ```text
   C:\CameraRTSP\mediamtx.yml
   ```

2. 搜索 `rtspTransports:`，修改为：

   ```yaml
   rtspTransports: [tcp]
   ```

3. 搜索 `rtspAddress:`，修改为：

   ```yaml
   rtspAddress: :8556
   ```

4. 保存文件；
5. 双击运行：

   ```text
   C:\CameraRTSP\mediamtx.exe
   ```

6. 如果 Windows 弹出防火墙提示，勾选“专用网络”，然后点击“允许访问”；
7. 保持 MediaMTX 黑色窗口一直打开。

如果要使用其他 RTSP 端口，将 `8556` 改成需要的端口即可。

## 第七步：让 OBS 开始推流

1. 打开“OBS → 设置 → 推流”；
2. 填写：

| 配置项 | 设置值 |
|---|---|
| 服务 | 自定义 |
| 服务器 | `rtmp://127.0.0.1/camera` |
| 串流密钥/推流码 | 留空 |
| 使用身份验证 | 不勾选 |

3. 点击“应用”，再点击“确定”；
4. 回到 OBS 主界面；
5. 点击右下角“开始推流”；
6. 保持 OBS 和 MediaMTX 同时运行。

如果要修改视频路径，例如改成 `test01`，OBS 服务器地址应改为：

```text
rtmp://127.0.0.1/test01
```

## 第八步：确认 Windows 电脑的 IP

1. 打开“Windows 设置 → 网络和 Internet”；
2. 点击当前正在使用的“以太网”或“Wi-Fi”；
3. 找到“IPv4 地址”；
4. 将该地址作为 RTSP 地址中的 IP。

例如电脑的 IPv4 地址是 `192.168.112.101`，最终拉流地址就是：

```text
rtsp://192.168.112.101:8556/camera
```

## 第九步：在另一台设备上拉流

### 使用 VLC 测试

1. 保持推流电脑上的 OBS 和 MediaMTX 正在运行；
2. 在另一台设备上打开 VLC；
3. 点击“媒体 → 打开网络串流”；
4. 输入：

   ```text
   rtsp://192.168.112.101:8556/camera
   ```

5. 点击“播放”；
6. 能看到摄像头画面即表示推流和拉流正常。

### 使用自己的系统测试

将同一个地址填写到系统的 RTSP 地址输入框：

```text
rtsp://192.168.112.101:8556/camera
```

如果系统可以选择 RTSP 传输方式，选择 `TCP`。

## 第十步：停止推流

1. 在 OBS 中点击“停止推流”；
2. 关闭 OBS；
3. 关闭 MediaMTX 黑色窗口。
