# 通讯频率状态块宽度防抖修复报告

日期：2026-08-12

范围：地面站手动操纵区“通讯频率”状态块的宽度稳定性与响应式布局回归。

安全边界：本次仅修改 Qt 显示与测试，没有连接或操作真机，没有申请控制权，也没有发送任何飞行指令。

## 一、问题与结论

状态块原先使用 `QSizePolicy.Minimum`，其 `sizeHint()` 会随文字变化。频率从 `10.00 Hz` 波动到 `9.99 Hz` 时少一个字符，通讯频率块会收窄，右侧“最近指令”随之左右移动，形成持续抖动。

现已修复：通讯频率块按配置中的正常频率范围计算所需整数位数，并用同位数宽字符样本确定常态最小宽度，因此 9.xx/10.xx 波动不再改变组件几何位置；没有设置固定宽度或最大宽度，更长内容仍能扩展，原有窄窗口换行逻辑保持不变。

## 二、实现方式

- 使用 `STATUS_RATE_TARGET_HZ + STATUS_RATE_TOLERANCE_HZ` 计算正常范围所需整数位数，不重复硬编码频率位数。
- 在控件完成 Qt style polish 后读取真实 `sizeHint().width()`，将其设置为 `minimumWidth`。
- 随后恢复初始显示 `通讯频率 · -- Hz`。
- 继续保留 `QSizePolicy.Minimum / Fixed`：宽度下限稳定，但 `100.00 Hz` 等更长内容仍可扩大。
- 没有修改状态频率计算、阈值颜色、两行响应布局或其他状态块。

## 三、回归验证

新增几何回归依次刷新 `10.00 → 9.99 → 10.00 → 9.98 Hz`，确认：

- 通讯频率块四次宽度完全一致；
- 右侧“最近指令”块的 X 坐标完全一致；
- `100.00 Hz` 时组件宽度大于常态宽度且不小于当前 `sizeHint()`，证明响应式扩展未被固定宽度破坏；
- 1180×700 下既有换行、无交叠和无裁字断言继续通过。

最终验证结果：

- 定向 Qt 回归：1 passed；
- 全量 pytest：76 passed（24.61 s）；
- `compileall`、flake8 致命规则、修改文件 whitespace 检查：通过；
- `ground_station.py --check-environment`：workspace environment OK。

## 四、改动文件

- `ground_station_core/qt_ui/operations_panel.py`
- `tests/test_qt_gui.py`
- 本报告与 `MEMORY.md`
