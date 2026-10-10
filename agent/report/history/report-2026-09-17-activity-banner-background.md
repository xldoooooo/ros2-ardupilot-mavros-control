# 动态提示底色与动画状态统一

本报告记录右上角动态框的底色适配和验证。

- 底色改为按 activityState 选择：进行中浅蓝、完成浅绿、失败/异常浅红、空闲中性。
- 保留 tone 日志等级，不再用日志等级控制背景，避免 WARN 进度显示黄色或成功仍显示蓝色。
- 光晕和背景共享同一状态，绿光消退后仍保留浅绿成功底色。
- UI 和环境 GUI 回归：16 passed；git diff --check 通过。
- 已在 offscreen Qt 中渲染检查四种状态，并刻意使用同一 WARN 等级验证底色不再依赖等级。
  截图：agent/codex/activity-banner/background-preview.png（本地过程产物）。
- 未连接实机或启动飞行；仅地面 GUI 修改，两台飞机无需部署。
- 未修改或提交工作区原有 README.md、TODO.md 改动。MEMORY.md 已更新当前基线。
