<!-- Remove project-local calibration tooling and relocate the unchanged FCU reboot client. -->
# 删除 tools 与重启客户端归位

日期：2026-09-17。用户要求删除项目 tools 目录和相机标定文件夹，迁移并明确命名重启客户端。

## 改动

- `tools/reboot_fcu.py` 原样迁移到 `src/onboard_control/scripts/reboot_fcu_client.py`；
  与原版本逐字比较一致，仍只负责租约、提交请求、续租和输出结果。
  机载 C++ `fcu_reboot.cpp` 及飞行协议3.3未改动。
- 根目录 `reboot_fcu.sh` 调整目标路径，仍使用项目 `.venv/bin/python`。
- 删除 `tools/camera_int_calib/` 的程序、README、Python缓存，以及仅针对该工具的
  `tests/test_uq212_intrinsic_calibration.py`（11项测试）；项目内不再保留 tools 目录。
- 稀疏检出不再单列旧客户端路径，新路径已由 `/src/onboard_control/` 覆盖。
  更新部署文档和 MEMORY，并移除构建示例里误混入的两个重启相关行。
- 历史任务报告保留当时路径，不改写历史记录。相机生产配置和已采用的内参未修改。
  refresh 项目外 `/home/nvidia/camera_int_calib/uq212_cam_calib.py` 及标定结果未删除。

## 验证与部署

- Bash语法、迁移后Python语法检查通过；客户端与旧文件字节一致。
- 开发机入口按预期拒绝直接执行；refresh无运行中的生产机载服务时同样拒绝执行。
- 开发机完整Python测试：282 passed。数量从293减少11，全部对应已删除工具的专用测试；
  没有通过跳过或放宽其他测试规避失败。
- refresh 已应用相同迁移，入口和包内文件一致，安装目录中的部署脚本同步更新。
  该机当前没有飞控，三项生产服务始终 disabled/inactive；未启动机载链，未重启FCU、解锁或起飞。
- 本次仅为路径整理，无C++修改，不重复构建飞行二进制。调用现有安装步骤更新部署文档/脚本。
- new 本轮未连接；MEMORY标记其后续须同步根入口、新客户端、tools删除及先前自检版本修正。
- 过程记录在 `agent/codex/reboot-client-relocation/`；refresh同目录保留迁移前定点备份。
  已有README、TODO、部署文档迁移等用户自有工作树改动未纳入本次提交。
