<!-- 本项目维护清理的稳定边界；当前值和部署事实仍读取项目文件。 -->
# ROS2–ArduPilot 项目约定

本技能属于含 `src/onboard_control/`、`drivers/odin/` 的ROS2–ArduPilot仓库。`agent/task/14-trim.md` 等本机资料存在时定向查阅，其八项维护范围为：MEMORY、报告、项目文档、agent临时目录、Git跟踪、备份、已有未提交改动、后续可清清单。

- 纯维护不修改程序和运行参数，不修改或实施 TODO.md；已有代码/配置/TODO改动需要整理提交时保留其原内容。实机解锁、起飞只允许用户手动执行，清理无需连接飞机或启停机载服务。
- Python 使用项目 `.venv/bin/python`；当前主工程、Odin和外部定位的有效 build/install、逐机配置和本机依赖环境按实际入口核对，不因为“都能编译”就全部删掉。
- 正式维护源码包含 `drivers/odin/`、`correction_service/extnav_patch/`、`fpga_avoidance/`；旧实验源码副本可能不同，先比差异。设备SDK静态库和 `intrinsics.yaml.old` 等镜头存档可能被构建或测试依赖。
- `agent/codex/`、`agent/grok/` 是过程目录。重点找重复源码/构建树、从bag导出的点云与replay-input、DataFlash解析CSV、视频抽帧/校色预览、合成RTSP录像和SITL输出；保留原始MCAP/DB3/实飞BIN、标定/参数、唯一补丁和紧凑结果。
- 当前正式媒体目录和MediaMTX/Intel运行时可能在仓库外，由配置和进程决定；不能把mini公共成品当原片再处理，不能清正在抓取的FTP控制证据或当前预览所需目录。
- 本机MEMORY必须保留，Git使用根规则 `/MEMORY.md`；`agent/`已整体忽略。分别用 `git check-ignore -v MEMORY.md` 与 `git ls-files MEMORY.md` 验证。报告和本地规则不因Git忽略而免于维护。
- `agent/report/AGENTS.md`、INDEX、MEMORY中的当前报告上限统一为**50份**。历史报告记载的30份是当时结果，不全局替换历史正文。报告根/history保留平面Markdown总索引，topics提供专题入口；专题、交付包内副本和附件不重复计入。新增报告和专题维护遵守[报告工作流](report-workflow.md)。
- 纯维护新简报使用 `agent/report/report-<日期>-<任务标题>.md`，同日重复任务另取唯一标题，不覆盖旧报告。清理清单放本次 `agent/codex/` 小目录，避免创建大尺寸“清理前全量备份”。
- 保留用户旧README原文到 `docs/`，并在当前README留入口；docs中已有的历史README/手工记录按实际文件名识别，不作为当前启动指南，也不继续精简或改写。
- 本技能本体由Git跟踪，MEMORY与报告仍留在本机。按项目AGENTS默认同步main，使用git/GitHub API，不依赖gh、不自行新建分支；只提交有授权且核对过的内容。仅更新忽略资料、没有仓库改动时不制造空提交。

这些规则不冻结设备地址、PID、参数数值或故障结论；接续时以当前AGENTS、配置、MEMORY和实际进程为准。
