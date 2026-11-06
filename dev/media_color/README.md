<!-- 历史样例媒体的独立离线校色工具及预览、处理入口。 -->
# 样例媒体离线校色

为 `examples/videos` 和 `examples/pictures` 生成独立校色副本，不修改原片、播放配置或服务。
仅依赖项目 `.venv` 的 Python 标准库及系统 FFmpeg/ffprobe，无新增 pip 依赖。

在仓库根目录运行：

```bash
# 每段视频中间取 6 秒，另抽 8 张图片；先比较两档效果。
.venv/bin/python dev/media_color/correct_media.py preview \
  --input ~/Media/ardupilot/public/examples \
  --output ~/Media/ardupilot/color-preview/natural --profile natural

.venv/bin/python dev/media_color/correct_media.py preview \
  --input ~/Media/ardupilot/public/examples \
  --output ~/Media/ardupilot/color-preview/enhanced --profile enhanced

# 按逐文件确认的轻重程度预览：严重的加强，其他保留 enhanced 参数。
.venv/bin/python dev/media_color/correct_media.py preview \
  --input ~/Media/ardupilot/public/examples \
  --output ~/Media/ardupilot/color-preview/selective --profile selective

# 效果确认后，在 mini 本机处理完整素材；输出仍在 FTP public 根目录之外。
.venv/bin/python dev/media_color/correct_media.py batch \
  --input ~/Media/ardupilot/public/examples \
  --output ~/Media/ardupilot/color-staging/selective --profile selective
```

输出目录必须全新或为空，且不能与输入目录相互包含。默认使用全部逻辑 CPU；
单个文件失败会记入 `manifest.json` 并继续处理其他文件，最终以非零状态退出。
每个结果完整解码后才改为正式文件名，并核对尺寸、视频帧率与时长，记录原片和结果 SHA-256。
中断后的目录保留已完成文件及记录，重试需另选空目录。

`profiles.json` 定义固定曲线和饱和度：`natural` 较轻，`enhanced` 更明显。
`selective` 默认沿用原版 `enhanced`，只把规则中明确列出的 2026-08-25
11:24 视频改用大幅降低中间调及白点的 `strong_exposure`，
11:50 视频及 16 张较亮图片保留 `severe` 中高光压缩曲线；
其他 8 段视频、144 张图片保持原版参数，较暗的同组照片也不跟随视频加强。
选择来自逐文件画面审阅，不根据日期自动推断相机是否调过参数。
新增规则只需维护 `selective.rules` 的文件名匹配及档位；每个结果在清单中记录实际档位和参数。
所有档位均从原片单次处理，不在已校色文件上叠加校色。
固定参数不逐帧自适应，不额外引入亮度闪烁；原片已有的曝光变化会保留。
`natural/enhanced/severe` 保留黑白端点；`strong_exposure` 将白点降低至 0.82，
用于降低严重过曝片源的刺眼程度。降低白点不会补回已经丢失的高光纹理。
视频按输入标记正确转换到 BT.709 有限范围，再统一编码为 H.264 High、CRF18；
保留分辨率和帧率，现有 1080p60 样例统一 120 帧 GOP、90k 时间基，适配连续 concat 播放。
JPEG 按全范围及 JPEG 色彩矩阵输出；PNG 保持 RGB 输出。

迁移到 mini 仅需复制本目录，使用 mini 项目的 `.venv/bin/python` 执行。
先检查清单无失败、完整视频循环及图片显示，再由人工安排切换并保留原素材用于回退。
脚本不包含部署、覆盖、重启或发布入口。
