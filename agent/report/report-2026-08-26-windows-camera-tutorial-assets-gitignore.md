# Windows 摄像头教程图片 Git 忽略简报

## 修改

- 在根 `.gitignore` 中加入 `/docs/assets/`；
- 当前教程的 3 张 PNG 截图继续保留在本机，不删除、不加入 Git；
- 未修改用户正在编辑的教程 Markdown，也未处理本机生成的 PDF。

## 验证

- `git check-ignore -v docs/assets/*.png`：3 张图片均由新增规则命中；
- `git status --short -- docs/assets`：无输出；
- 提交范围只包含 `.gitignore`、本报告和 `MEMORY.md`，不包含教程图片。
