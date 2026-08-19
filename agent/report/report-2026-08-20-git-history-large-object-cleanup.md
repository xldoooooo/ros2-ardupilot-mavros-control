# Git 历史大对象清理简报（2026-08-20）

## 目标

- 将本地当前完整功能状态提交到 `main`。
- 从全部历史中删除 127 MiB MP4、53 MiB MediaMTX 和 25 MiB rtsp-simple-server blob。
- 忽略并停止跟踪 `agent/task/assets/` 下的图片与视频，同时保留本机文件。
- 判断并处理不可达 Git 对象，完成远端 `main` 历史更新。

## 不可达对象审计

- 改写前 `git fsck --unreachable --no-reflogs` 检出 4 个 commit、620 个 tree、361 个 blob。
- 4 个提交包含历史重构、路径生成、WebSocket协议和推力参数等非空改动，无法证明完全无用，不能
  直接当垃圾永久删除。
- 因历史过滤工具会清理活动对象库，执行前将整个 `.git` 复制到项目外备份；这同时保存无引用的
  独立 blob/tree，比只给4个提交建分支更完整。活动仓库可以清理，恢复证据仍然保留。

## 执行结果

- 将本地全部待处理功能与文档状态提交到 `main`，任务媒体和架构相关 demo 可执行文件未进入新树。
- 完整改写前备份：
  `/home/nvidia/backups/ros2-ardupilot-git-pre-history-rewrite-20260820.tar.gz`，276 MiB，SHA-256：
  `77ff2b0b6a82dfbf922c3f3b41effbffae20cdbbeea62aa1f832dd96c9625215`；已验证其中包含 `.git/HEAD`、
  index 和最大不可达 blob。
- 使用 `git-filter-repo 2.47.0` 按三个精确 blob ID 重写97个提交。Codex桌面的16个本地 tree 快照
  仍引用目标 blob，已在备份存在的前提下精确删除这些私有 refs，然后执行 reflog expire 与 GC。
- 三个目标 OID 均已通过 `git cat-file -e` 反向验证为不存在；`git fsck --full --no-dangling` 通过，
  loose object、garbage 均为0。
- `.git` 从约290 MiB降至41 MiB，整个工作目录从约1.5 GiB降至约1.2 GiB；剩余主体仍是903 MiB
  `.venv`，不属于Git历史。
- `.gitignore` 覆盖 `agent/task/assets` 常见图片/视频扩展名，并精确忽略移动到 `dev/demo` 的旧
  rtsp-simple-server；13个原已跟踪任务媒体已从索引删除但本机文件保持存在。
- 历史改写前项目全量测试为 `164 passed`；过滤只改变提交图和指定大对象，不改变已测试工作树。
- GitHub `main` 已通过强制更新从旧历史 `c3a362d` 切换到过滤后的当前状态；包含本报告最终补记的
  amend 提交随后再次强推并以 `ls-remote` 核对。
