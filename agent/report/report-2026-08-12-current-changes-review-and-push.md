# 当前改动审查、清理与推送简报

日期：2026-08-12

## 结果

逐项检查了当前工作树中的已修改、已删除和未跟踪文件。保留并纳入本次提交的内容包括：

- 根目录 `README.md`，并修正路径、主动停止与自动重启语义等明显文档问题；
- `integration/` 下甲方协议原件和目录说明；
- 任务 14～16 的任务说明、被任务实际引用的截图/视频，以及任务 14、15 和机载开机自启动报告；
- `agent/ref/` 下 Jetson/AP 飞控重启调查的 DOCX 与 PDF 交付件；
- 用户维护的 `TODO.md`、任务 13 说明、RViz 视角配置和已有 `MEMORY.md` 记录；
- `.gitignore` 中新增的 `agent/codex/`、`agent/grok/` 忽略规则。

## 已清理内容

以下内容不纳入版本库，并已移入桌面环境回收站：

- `agent/codex/` 的全部内容：过程截图、视频抽帧、Python 字节码和真机专用 systemd 过程副本；
- `agent/grok/` 的全部内容；清理前该目录没有文件；
- `agent/ref/6db32d5410f641a3a68f4dcfbbd03838_raw.mp4`：与任务 15 素材逐字节相同的重复副本；
- `agent/task/assets/image-20260811223337752.png`、`image-20260811223340865.png`、
  `image-20260811223348358.png`：均为 8×1 像素、90 字节且没有被任务或报告引用的空白残片。

`agent/codex/ros2-ardupilot-onboard.service` 虽然记录过真机配置，但它位于明确要求清空的过程目录，
且仓库已经有正式的 `src/onboard_control/deploy/onboard-control.service.example`、部署文档和回归测试，
因此不再保留重复的机器专用副本。

## 文件完整性与验证

- `integration/无人机与上位机通信协议V2.0.docx` 和
  `agent/ref/Jetson_AP_Flight_Controller_Reboot_Investigation_Report_CN.docx` 均通过 ZIP 容器完整性检查；
- AP 重启调查 PDF 可正常识别，任务 15 视频可识别为 MP4，保留的任务截图均为有效 PNG；
- `.gitignore` 匹配测试确认 `agent/codex/`、`agent/grok/` 后续内容不会再次进入 Git；
- RViz 配置完成 YAML 解析检查；部署专项测试和完整 Python 测试通过；
- `git diff --check` 通过。

本次只整理本地仓库并执行测试，没有连接、解锁或起飞实机，也没有发送任何飞行命令。
