<!-- 本地过程文件清理记录；包含删除依据、保留范围及核验结果。 -->
# agent/codex 清理执行简报

日期：2026-10-12。范围仅为本开发机 `agent/codex/`。

- 清理前：1,571,147,776 字节（1.463 GiB）。
- 清理后：675,266,560 字节（644.0 MiB）。
- 释放：895,881,216 字节（854.4 MiB），按 `du -s -B1` 实际占用统计。
- 删除 2579 个文件/链接，重复点云目录替换为指向原件的相对链接。

## 保留与核验

- 保留原始飞行、相机、标定、点云、现场配置和部署备份；保留近期 Task38 恢复、两机同步、Odin 排障、路径延迟与 CPU 分析证据。
- 保留报告证据日志、软著最终版及旧版 PDF/修改前备份、专利最新渲染及源脚本/附图/仿真数据。
- 六个删除的调研仓库均无工作区改动，也无本地独有提交；被删除的 bundle 全部引用均是当前 HEAD 的祖先。
- 两份点云及 metadata 删除前逐文件 SHA-256 一致；原件保留在 `task36/real-data/odin-20260923/`，旧 `new-evidence/odin-20260923/` 路径仍可读取。
- 3152 个保留文件清理前后 SHA-256 全部一致；清理目标未发现进程 cwd/exe/文件描述符占用。
- 清理完成后、写本报告前 Git 状态与清理前完全一致；用户原有 TODO/任务历史变动未修改。
- 未改业务代码，未连接或操作实机。本次不属于代码修改任务，项目运行基线未变化，故不向 MEMORY.md 追加清理流水账。
- 历史报告中的调研检出、旧构建和旧渲染路径属于当时过程记录，清理后不再存在；关键测试日志及现场证据仍保留。

## 删除清单

路径相对于 `agent/codex/`。体积为文件逻辑大小，释放总量以上述实际占用为准。

| 路径 | 文件数 | MiB | 删除依据 |
| --- | ---: | ---: | --- |
| `__pycache__` | 8 | 0.119 | Python 字节码缓存 |
| `apriltag_extrinsic_calibrator/__pycache__` | 1 | 0.040 | Python 字节码缓存 |
| `check-shell-relocation.py` | 1 | 0.002 | 已完成任务的下载包、阅读渲染、迁移中间文件或空目录 |
| `correction-rate-20260917/rate-sync.bundle` | 1 | 0.010 | 同步包全部引用已包含在当前 Git 历史中 |
| `correction-uvc-20260916/memory-sync.bundle` | 1 | 0.001 | 同步包全部引用已包含在当前 Git 历史中 |
| `extnav-cpu-audit/py_spy-0.4.2-py2.py3-none-manylinux_2_17_aarch64.manylinux2014_aarch64.whl` | 1 | 2.697 | 已完成任务的下载包、阅读渲染、迁移中间文件或空目录 |
| `jetson-bootstrap/final-commit.bundle` | 1 | 0.026 | 同步包全部引用已包含在当前 Git 历史中 |
| `jetson-bootstrap/freerdp3-x11_3.31.0+dfsg-0ubuntu0.24.04.1_amd64.deb` | 1 | 0.118 | 已完成任务的下载包、阅读渲染、迁移中间文件或空目录 |
| `jetson-bootstrap/geographiclib-datasets.tar.gz` | 1 | 18.854 | 已完成任务的下载包、阅读渲染、迁移中间文件或空目录 |
| `jetson-bootstrap/project-main.bundle` | 1 | 45.415 | 同步包全部引用已包含在当前 Git 历史中 |
| `jetson-bootstrap/ros.key` | 1 | 0.001 | 已完成任务的下载包、阅读渲染、迁移中间文件或空目录 |
| `mavros-param-fix/fix.bundle` | 1 | 0.006 | 同步包全部引用已包含在当前 Git 历史中 |
| `mavros-param-fix/record.bundle` | 1 | 0.003 | 同步包全部引用已包含在当前 Git 历史中 |
| `move-build-entry.patch` | 1 | 0.010 | 已完成任务的下载包、阅读渲染、迁移中间文件或空目录 |
| `new-sync-20261008/main.bundle` | 1 | 46.006 | 同步包全部引用已包含在当前 Git 历史中 |
| `new-sync-20261008/record.bundle` | 1 | 0.005 | 同步包全部引用已包含在当前 Git 历史中 |
| `odin_driver_aircraft/launch_ROS2/__pycache__` | 1 | 0.005 | Python 字节码缓存 |
| `prepare-shell-move.py` | 1 | 0.004 | 已完成任务的下载包、阅读渲染、迁移中间文件或空目录 |
| `priority-production-20260917/new-before/install/onboard_control/share/onboard_control/launch/__pycache__` | 1 | 0.001 | Python 字节码缓存 |
| `protocol-read-bMpCwt` | 5 | 1.131 | 已完成任务的下载包、阅读渲染、迁移中间文件或空目录 |
| `pycache` | 41 | 0.920 | 可重新生成的构建、测试临时产物；测试日志保留 |
| `readme-build-entry-index.patch` | 1 | 0.001 | 已完成任务的下载包、阅读渲染、迁移中间文件或空目录 |
| `reboot-client-relocation/update.bundle` | 1 | 0.005 | 同步包全部引用已包含在当前 Git 历史中 |
| `reboot-eval-pdf` | 6 | 1.122 | 已完成任务的下载包、阅读渲染、迁移中间文件或空目录 |
| `refresh-sparse-final.bundle` | 1 | 0.004 | 同步包全部引用已包含在当前 Git 历史中 |
| `refresh-startup-sync-20260917/report.bundle` | 1 | 0.002 | 同步包全部引用已包含在当前 Git 历史中 |
| `refresh-startup-sync-20260917/startup.bundle` | 1 | 0.040 | 同步包全部引用已包含在当前 Git 历史中 |
| `refresh-timesync-record.bundle` | 1 | 0.002 | 同步包全部引用已包含在当前 Git 历史中 |
| `refresh-timesync.bundle` | 1 | 0.006 | 同步包全部引用已包含在当前 Git 历史中 |
| `ros-key-validation.gpg` | 1 | 0.001 | 已完成任务的下载包、阅读渲染、迁移中间文件或空目录 |
| `shell-changed.json` | 1 | 0.001 | 已完成任务的下载包、阅读渲染、迁移中间文件或空目录 |
| `shell-move.patch` | 1 | 0.055 | 已完成任务的下载包、阅读渲染、迁移中间文件或空目录 |
| `shell-moves.json` | 1 | 0.001 | 已完成任务的下载包、阅读渲染、迁移中间文件或空目录 |
| `sync-169-to-186-20261012/main.bundle` | 1 | 0.289 | 同步包全部引用已包含在当前 Git 历史中 |
| `t28v` | 0 | 0.000 | 已完成任务的下载包、阅读渲染、迁移中间文件或空目录 |
| `tag-xy-audit-20260918/__pycache__` | 1 | 0.008 | Python 字节码缓存 |
| `task28-software/__pycache__` | 1 | 0.031 | Python 字节码缓存 |
| `task28-software/code-render/page-*.png` | 60 | 15.567 | 软著材料旧版逐页渲染；PDF、最终版和修改前备份保留 |
| `task28-software/code-v2/page-*.png` | 60 | 16.610 | 软著材料旧版逐页渲染；PDF、最终版和修改前备份保留 |
| `task28-software/form-v1/page-*.png` | 4 | 0.724 | 软著材料旧版逐页渲染；PDF、最终版和修改前备份保留 |
| `task28-software/form-v2/page-*.png` | 4 | 0.717 | 软著材料旧版逐页渲染；PDF、最终版和修改前备份保留 |
| `task28-software/manual-v1/page-*.png` | 19 | 8.494 | 软著材料旧版逐页渲染；PDF、最终版和修改前备份保留 |
| `task28-software/manual-v2/page-*.png` | 24 | 11.143 | 软著材料旧版逐页渲染；PDF、最终版和修改前备份保留 |
| `task28-software/revision1/code/page-*.png` | 60 | 16.607 | 软著材料旧版逐页渲染；PDF、最终版和修改前备份保留 |
| `task28-software/revision1/form/page-*.png` | 6 | 0.806 | 软著材料旧版逐页渲染；PDF、最终版和修改前备份保留 |
| `task28-software/revision1/manual/page-*.png` | 24 | 11.177 | 软著材料旧版逐页渲染；PDF、最终版和修改前备份保留 |
| `task28-software/revision2/code/page-*.png` | 60 | 14.647 | 软著材料旧版逐页渲染；PDF、最终版和修改前备份保留 |
| `task28-software/revision2/manual/page-*.png` | 24 | 11.186 | 软著材料旧版逐页渲染；PDF、最终版和修改前备份保留 |
| `task31/__pycache__` | 1 | 0.057 | Python 字节码缓存 |
| `task31/render-fig2-clarified-20260914-v1` | 27 | 7.720 | 专利旧版渲染；最新渲染、源脚本、附图和数据保留 |
| `task31/render-layout-refine-v1` | 27 | 7.704 | 专利旧版渲染；最新渲染、源脚本、附图和数据保留 |
| `task31/render-refine-v1` | 27 | 7.716 | 专利旧版渲染；最新渲染、源脚本、附图和数据保留 |
| `task31/render-refine-v2` | 27 | 7.717 | 专利旧版渲染；最新渲染、源脚本、附图和数据保留 |
| `task31/render-user-discrepancy-20260911` | 27 | 7.717 | 专利旧版渲染；最新渲染、源脚本、附图和数据保留 |
| `task31/render-v1` | 31 | 8.939 | 专利旧版渲染；最新渲染、源脚本、附图和数据保留 |
| `task31/render-v2` | 32 | 8.935 | 专利旧版渲染；最新渲染、源脚本、附图和数据保留 |
| `task31/render-v3` | 32 | 8.898 | 专利旧版渲染；最新渲染、源脚本、附图和数据保留 |
| `task31/render-v4` | 31 | 8.853 | 专利旧版渲染；最新渲染、源脚本、附图和数据保留 |
| `task32-fix29/__pycache__` | 2 | 0.024 | Python 字节码缓存 |
| `task33-refresh/main-update.bundle` | 1 | 0.003 | 同步包全部引用已包含在当前 Git 历史中 |
| `task35-ego` | 108 | 1.817 | 已完成调研的干净上游检出，无本地独有提交 |
| `task35-mighty` | 149 | 6.060 | 已完成调研的干净上游检出，无本地独有提交 |
| `task35-mrs` | 55 | 0.221 | 已完成调研的干净上游检出，无本地独有提交 |
| `task35-super` | 307 | 5.250 | 已完成调研的干净上游检出，无本地独有提交 |
| `task35-upstream` | 184 | 56.079 | 已完成调研的干净上游检出，无本地独有提交 |
| `task36/upstream-review` | 184 | 56.079 | 已完成调研的干净上游检出，无本地独有提交 |
| `task37/__pycache__` | 4 | 0.081 | Python 字节码缓存 |
| `task37/build-sanitized` | 635 | 265.486 | 可重新生成的构建、测试临时产物；测试日志保留 |
| `task37/install-sanitized` | 188 | 77.037 | 可重新生成的构建、测试临时产物；测试日志保留 |
| `task37/pytest-cache-live-probe` | 5 | 0.001 | 旧 pytest 缓存及临时目录 |
| `task37/pytest-cache-release` | 5 | 0.001 | 旧 pytest 缓存及临时目录 |
| `task37/pytest-cache-sanitized` | 5 | 0.001 | 旧 pytest 缓存及临时目录 |
| `task37/pytest-release` | 8 | 0.002 | 旧 pytest 缓存及临时目录 |
| `task37/pytest-release-final` | 8 | 0.001 | 旧 pytest 缓存及临时目录 |
| `task37/pytest-release-final.log` | 1 | 0.000 | 旧 pytest 缓存及临时目录 |
| `task37/pytest-release-final.xml` | 1 | 0.001 | 旧 pytest 缓存及临时目录 |
| `task37/pytest-release.log` | 1 | 0.060 | 旧 pytest 缓存及临时目录 |
| `task37/pytest-release.xml` | 1 | 0.063 | 旧 pytest 缓存及临时目录 |
| `task37/pytest-sanitized` | 8 | 0.001 | 旧 pytest 缓存及临时目录 |
| `task37/pytest-sanitized.log` | 1 | 0.000 | 旧 pytest 缓存及临时目录 |
| `task37/pytest-sanitized.xml` | 1 | 0.001 | 旧 pytest 缓存及临时目录 |
| `task38-executor-lock-review` | 6 | 0.006 | 可重新生成的构建、测试临时产物；测试日志保留 |
| `task38-executor-review` | 4 | 0.004 | 可重新生成的构建、测试临时产物；测试日志保留 |
| `task38-restore-20261012/169/path_planning/test/__pycache__` | 3 | 0.083 | Python 字节码缓存 |
| `task38-restore-20261012/main-docs.bundle` | 1 | 0.003 | 同步包全部引用已包含在当前 Git 历史中 |
| `task38-restore-20261012/main.bundle` | 1 | 0.251 | 同步包全部引用已包含在当前 Git 历史中 |
| `task38_math_review_fixture` | 1 | 0.102 | 可重新生成的构建、测试临时产物；测试日志保留 |
| `win-camera-diagnostics` | 0 | 0.000 | 已完成任务的下载包、阅读渲染、迁移中间文件或空目录 |
| `task36/new-evidence/odin-20260923/`（改为链接） | 2 | 78.304 | 与保留原件逐文件哈希一致 |
