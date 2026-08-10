# 原点齿轮会话锁定与绿色按钮 Hover

## 结论

1. **原点齿轮**在仿真运行或实机已连接（及环境工作流进行中）时禁用，仅空闲可改本地缓存。  
2. **绿色 `success` 按钮**补齐 hover 变深样式，与 primary/danger/中性按钮一致。

## 改动

| 文件 | 内容 |
| --- | --- |
| `qt_ui/state.py` | `UiAvailability.origin_settings`：`not closing and not busy and not environment_active` |
| `qt_ui/operations_panel.py` | 按 `state.origin_settings` 启用齿轮；禁用时 tooltip 说明原因 |
| `qt_ui/theme.py` | `success_hover`；`[role="success"]:hover`；primary/danger hover 同步边框色 |
| `tests/test_qt_gui.py` | 会话后齿轮禁用；success hover 样式断言 |
| `MEMORY.md` | 同步齿轮门控与按钮 hover 约定 |

## 验证

```text
pytest -q tests/test_qt_gui.py
# 含 origin_settings / environment_session / success_buttons 相关用例
```
