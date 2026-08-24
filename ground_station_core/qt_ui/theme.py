"""地面站浅色工程主题与统一视觉常量。"""

from __future__ import annotations

from pathlib import Path

from PySide6.QtGui import QColor, QPalette
from PySide6.QtWidgets import QApplication


# 低饱和工程配色；危险色只用于真实警告和高风险动作。
COLORS = {
    "background": "#eef1f4",
    "surface": "#ffffff",
    "surface_alt": "#f7f8fa",
    "border": "#cfd6de",
    "border_strong": "#aeb8c4",
    "text": "#182433",
    "muted": "#5f6f80",
    "accent": "#245f87",
    "accent_hover": "#194d70",
    "success": "#247457",
    "success_hover": "#1b5c44",
    "success_soft": "#e7f2ed",
    "warning": "#946200",
    "warning_soft": "#fff4d6",
    "danger": "#a7352a",
    "danger_hover": "#87271f",
    "danger_soft": "#fbe9e7",
    "disabled": "#98a4b1",
}

# 使用模块旁的 SVG，避免桌面主题差异导致步进箭头缺失或显示成残缺线段。
_ASSET_DIR = Path(__file__).with_name("assets")
_SPIN_UP_ICON = (_ASSET_DIR / "chevron-up.svg").as_posix()
_SPIN_DOWN_ICON = (_ASSET_DIR / "chevron-down.svg").as_posix()


STYLE_SHEET = f"""
QWidget {{
    color: {COLORS['text']};
    font-family: "Noto Sans CJK SC", "Noto Sans", "DejaVu Sans", sans-serif;
    font-size: 10pt;
}}
QMainWindow {{
    background: transparent;
}}
QWidget#centralRoot {{
    background: {COLORS['background']};
}}
QFrame#outerWindowFrame {{
    background: {COLORS['surface_alt']};
    border: 1px solid #8595a5;
    border-radius: 8px;
}}
QFrame#subpanelWindowFrame {{
    background: {COLORS['surface_alt']};
    border: 1px solid #8595a5;
    border-radius: 8px;
}}
QFrame#outerWindowFrame[windowMaximized="true"],
QFrame#subpanelWindowFrame[windowMaximized="true"] {{
    border-radius: 0;
}}
QFrame#subpanelTitleBar {{
    background: {COLORS['surface']};
    border: none;
    border-bottom: 1px solid {COLORS['border']};
}}
QLabel#subpanelWindowTitle {{
    color: {COLORS['text']};
    font-size: 11pt;
    font-weight: 700;
}}
QFrame#windowSurface {{
    background: {COLORS['surface_alt']};
    border: none;
    border-radius: 0;
}}
QMessageBox#shadowMessageBox {{
    background: transparent;
}}
QFrame#dialogSurface {{
    background: {COLORS['surface']};
    border: 1px solid #8595a5;
    border-radius: 8px;
}}
QFrame#dialogTitleBar {{
    background: transparent;
    border: none;
    border-bottom: 1px solid {COLORS['border']};
}}
QLabel#dialogTitle {{
    font-size: 11pt;
    font-weight: 700;
    color: {COLORS['text']};
}}
QFrame#card, QFrame#statusBadge, QFrame#activityBanner {{
    background: {COLORS['surface']};
    border: 1px solid {COLORS['border']};
    border-radius: 4px;
}}
QLabel#windowTitle {{
    font-size: 18pt;
    font-weight: 700;
    color: {COLORS['text']};
}}
QLabel#statusValue {{
    font-family: "DejaVu Sans Mono", monospace;
    font-size: 9pt;
    font-weight: 700;
}}
QLabel#environmentChip {{
    min-height: 30px;
    padding: 3px 10px;
    color: {COLORS['accent']};
    background: #e8f1f7;
    border: 1px solid #b9cfdf;
    border-radius: 4px;
    font-weight: 700;
}}
QLabel#windowSubtitle, QLabel#mutedLabel, QLabel#cardSubtitle {{
    color: {COLORS['muted']};
}}
QLabel#cardTitle {{
    font-size: 11pt;
    font-weight: 700;
    color: {COLORS['text']};
}}
QLabel#waypointMethodLabel {{
    color: {COLORS['muted']};
    font-size: 8.5pt;
    font-weight: 700;
}}
QLabel#cardHelpIcon {{
    color: {COLORS['accent']};
    background: #e8f1f7;
    border: 1px solid #9dbbd0;
    border-radius: 9px;
    font-size: 9pt;
    font-weight: 700;
}}
QLabel#metricValue {{
    font-family: "DejaVu Sans Mono", monospace;
    font-size: 10pt;
    font-weight: 600;
}}
QLabel#manualStatusChip {{
    min-height: 25px;
    padding: 2px;
    background: {COLORS['surface_alt']};
    border: 1px solid {COLORS['border']};
    border-radius: 3px;
    font-size: 8.5pt;
    font-weight: 700;
}}
QLabel#manualStatusChip[tone="good"] {{
    color: {COLORS['success']};
    background: {COLORS['success_soft']};
    border-color: #a8cbbb;
}}
QLabel#manualStatusChip[tone="warning"] {{
    color: {COLORS['warning']};
    background: {COLORS['warning_soft']};
    border-color: #dec789;
}}
QLabel#manualStatusChip[tone="bad"] {{
    color: {COLORS['danger']};
    background: {COLORS['danger_soft']};
    border-color: #dda69f;
}}
QFrame[joystickDeck="true"] {{
    background: #f8fafb;
    border: 1px solid {COLORS['border_strong']};
    border-radius: 16px;
}}
QWidget#joystickCenterControls {{
    background: transparent;
}}
QPushButton[manualActive="true"] {{
    color: white;
    background: {COLORS['accent']};
    border-color: {COLORS['accent_hover']};
}}
QFrame#manualSummaryMetric {{
    background: #f5f8fa;
    border: 1px solid {COLORS['border']};
    border-radius: 4px;
}}
QLabel#manualSummaryTitle, QLabel#manualSummaryUnit {{
    color: {COLORS['muted']};
    font-size: 8.5pt;
}}
QLabel#manualSummaryValue {{
    color: {COLORS['accent']};
    font-family: "DejaVu Sans Mono", monospace;
    font-size: 16pt;
    font-weight: 700;
}}
QToolButton#engineeringTelemetryToggle {{
    min-height: 24px;
    padding: 2px 7px;
    color: {COLORS['muted']};
    background: transparent;
    border: 1px solid transparent;
    border-radius: 3px;
    font-weight: 600;
}}
QToolButton#engineeringTelemetryToggle:hover {{
    color: {COLORS['accent']};
    background: #e8f1f7;
    border-color: #b9cfdf;
}}
QLabel#shortcutHint {{
    color: {COLORS['muted']};
    font-size: 9pt;
}}
QFrame#statusBadge[tone="good"] {{ border-left: 4px solid {COLORS['success']}; }}
QFrame#statusBadge[tone="warn"] {{ border-left: 4px solid {COLORS['warning']}; }}
QFrame#statusBadge[tone="bad"] {{ border-left: 4px solid {COLORS['danger']}; }}
QFrame#statusBadge[tone="accent"] {{ border-left: 4px solid {COLORS['accent']}; }}
QFrame#statusBadge[tone="neutral"] {{
    border-left: 4px solid {COLORS['border_strong']};
}}
QFrame#activityBanner[tone="debug"] {{ background: {COLORS['surface_alt']}; }}
QFrame#activityBanner[tone="info"] {{ background: #eaf2f8; border-color: #b9cfdf; }}
QFrame#activityBanner[tone="warn"] {{
    background: {COLORS['warning_soft']}; border-color: #e2c675;
}}
QFrame#activityBanner[tone="error"] {{
    background: {COLORS['danger_soft']}; border-color: #dda69f;
}}
QMenuBar {{
    background: {COLORS['surface']};
    border-bottom: 1px solid {COLORS['border']};
    border-top-left-radius: 7px;
    border-top-right-radius: 7px;
    padding: 2px 6px;
}}
QMenuBar::item {{ padding: 5px 10px; border-radius: 3px; }}
QMenuBar::item:selected {{ background: #e2ebf2; }}
QMenu {{
    background: {COLORS['surface']};
    border: 1px solid {COLORS['border_strong']};
    padding: 5px;
}}
QMenu::item {{ padding: 6px 28px 6px 24px; border-radius: 3px; }}
QMenu::item:selected {{ background: #dcebf5; }}
QMenu::separator {{ height: 1px; background: {COLORS['border']}; margin: 4px 7px; }}
QPushButton {{
    min-height: 34px;
    padding: 4px 12px;
    background: {COLORS['surface_alt']};
    border: 1px solid {COLORS['border_strong']};
    border-radius: 3px;
    font-weight: 600;
}}
QPushButton:hover {{ background: #e8edf1; border-color: #82909e; }}
QPushButton:pressed {{ background: #dde4e9; }}
QPushButton:disabled {{
    color: {COLORS['disabled']};
    background: #edf0f2;
    border-color: #dbe0e5;
}}
QPushButton[role="primary"] {{
    color: white;
    background: {COLORS['accent']};
    border-color: {COLORS['accent']};
}}
QPushButton[role="success"] {{
    color: white;
    background: {COLORS['success']};
    border-color: {COLORS['success']};
}}
QPushButton[role="success"]:hover {{
    background: {COLORS['success_hover']};
    border-color: {COLORS['success_hover']};
}}
QPushButton[role="danger"] {{
    color: white;
    background: {COLORS['danger']};
    border-color: {COLORS['danger']};
}}
QPushButton[role="danger"]:hover {{
    background: {COLORS['danger_hover']};
    border-color: {COLORS['danger_hover']};
}}
QPushButton[role="primary"]:hover {{
    background: {COLORS['accent_hover']};
    border-color: {COLORS['accent_hover']};
}}
QPushButton[role="primary"]:disabled,
QPushButton[role="success"]:disabled,
QPushButton[role="danger"]:disabled {{
    color: {COLORS['disabled']};
    background: #edf0f2;
    border-color: #dbe0e5;
}}
QPushButton[compact="true"] {{ min-height: 28px; padding: 2px 9px; }}
QPushButton#simulationButton,
QPushButton#hardwareButton {{
    padding: 4px;
}}
QPushButton#originSettingsButton,
QPushButton#communicationTestButton {{
    padding: 0;
}}
QPushButton#moveWaypointUpButton,
QPushButton#moveWaypointDownButton,
QPushButton#removeWaypointButton {{
    min-height: 26px;
    max-height: 26px;
    padding: 0;
}}
QPushButton#addWaypointButton {{
    min-height: 26px;
    max-height: 26px;
    min-width: 26px;
    max-width: 26px;
    padding: 0;
    background: white;
    font-size: 14pt;
    font-weight: 700;
}}
QPushButton#addWaypointButton:hover {{ background: #edf3f7; }}
QPushButton#previewWaypointButton,
QPushButton#importWaypointButton {{ background: white; }}
QPushButton#previewWaypointButton:hover,
QPushButton#importWaypointButton:hover {{ background: #edf3f7; }}
QPushButton#addWaypointButton:disabled {{
    color: {COLORS['disabled']};
    background: #edf0f2;
    border-color: #dbe0e5;
}}
QPushButton#previewWaypointButton:disabled,
QPushButton#importWaypointButton:disabled {{
    color: {COLORS['disabled']};
    background: #edf0f2;
    border-color: #dbe0e5;
}}
QPushButton#originSettingsButton {{
    font-size: 14pt;
}}
QPushButton[windowControl="true"] {{
    min-height: 25px;
    max-height: 25px;
    min-width: 28px;
    padding: 0;
    background: transparent;
    border: 1px solid transparent;
    border-radius: 3px;
    font-size: 12pt;
}}
QPushButton[windowControl="true"]:hover {{
    background: #dfe7ed;
    border-color: {COLORS['border']};
}}
QPushButton[closeControl="true"]:hover {{
    color: white;
    background: {COLORS['danger']};
    border-color: {COLORS['danger']};
}}
QLineEdit, QComboBox {{
    min-height: 30px;
    padding: 2px 7px;
    background: white;
    border: 1px solid {COLORS['border_strong']};
    border-radius: 3px;
    selection-background-color: {COLORS['accent']};
}}
QComboBox#manualCoordinateMode {{
    min-height: 26px;
    max-height: 26px;
    min-width: 96px;
}}
QComboBox[sensitivityControl="true"] {{
    min-height: 24px;
    max-height: 24px;
    padding: 1px 5px;
    font-size: 8.5pt;
}}
QLineEdit#logSearchInput {{ min-height: 28px; max-height: 28px; }}
QDoubleSpinBox {{
    min-height: 30px;
    padding: 2px 25px 2px 7px;
    background: white;
    border: 1px solid {COLORS['border_strong']};
    border-radius: 3px;
    selection-background-color: {COLORS['accent']};
}}
QDoubleSpinBox[waypointCoordinate="true"] {{
    min-height: 22px;
    max-height: 22px;
}}
QLineEdit:disabled, QComboBox:disabled, QDoubleSpinBox:disabled {{
    color: {COLORS['disabled']};
    background: #edf0f2;
    border-color: #dbe0e5;
}}
QLineEdit:focus, QComboBox:focus {{
    border: 2px solid {COLORS['accent']};
    padding: 1px 6px;
}}
QDoubleSpinBox:focus {{
    border: 2px solid {COLORS['accent']};
    padding: 1px 24px 1px 6px;
}}
QDoubleSpinBox::up-button, QDoubleSpinBox::down-button {{
    subcontrol-origin: border;
    width: 20px;
    background: #e8edf2;
    border-left: 1px solid {COLORS['border_strong']};
}}
QDoubleSpinBox::up-button {{
    subcontrol-position: top right;
    border-bottom: 1px solid {COLORS['border']};
    border-top-right-radius: 3px;
}}
QDoubleSpinBox::down-button {{
    subcontrol-position: bottom right;
    border-bottom-right-radius: 3px;
}}
QDoubleSpinBox::up-button:hover, QDoubleSpinBox::down-button:hover {{
    background: #d6e2eb;
}}
QDoubleSpinBox::up-button:pressed, QDoubleSpinBox::down-button:pressed {{
    background: #c8d7e2;
}}
QDoubleSpinBox::up-arrow {{
    image: url("{_SPIN_UP_ICON}");
    width: 12px;
    height: 8px;
}}
QDoubleSpinBox::down-arrow {{
    image: url("{_SPIN_DOWN_ICON}");
    width: 12px;
    height: 8px;
}}
QDoubleSpinBox[compactValueInput="true"] {{
    padding: 2px 3px;
    font-size: 9pt;
}}
QDoubleSpinBox[compactValueInput="true"]:focus {{
    padding: 1px 2px;
}}
QDoubleSpinBox[compactValueInput="true"]::up-button,
QDoubleSpinBox[compactValueInput="true"]::down-button {{ width: 13px; }}
QDoubleSpinBox[compactValueInput="true"]::up-arrow,
QDoubleSpinBox[compactValueInput="true"]::down-arrow {{
    width: 9px;
    height: 6px;
}}
QDoubleSpinBox:disabled::up-button,
QDoubleSpinBox:disabled::down-button {{
    background: #e3e7ea;
    border-color: #dbe0e5;
}}
QCheckBox {{ spacing: 5px; }}
QCheckBox#logLevelDebug, QCheckBox#logLevelInfo,
QCheckBox#logLevelWarn, QCheckBox#logLevelError {{
    padding: 3px 6px;
    background: {COLORS['surface_alt']};
    border: 1px solid {COLORS['border']};
    border-radius: 3px;
    font-family: "DejaVu Sans Mono", monospace;
    font-weight: 700;
}}
QCheckBox#logLevelDebug {{ color: {COLORS['muted']}; }}
QCheckBox#logLevelInfo {{ color: {COLORS['text']}; }}
QCheckBox#logLevelWarn {{ color: {COLORS['warning']}; }}
QCheckBox#logLevelError {{ color: {COLORS['danger']}; }}
QProgressBar {{
    min-height: 23px;
    background: #e7ebef;
    border: 1px solid {COLORS['border_strong']};
    border-radius: 3px;
    text-align: center;
}}
QProgressBar::chunk {{ background: {COLORS['accent']}; border-radius: 2px; }}
QProgressBar#waypointProgress::chunk {{
    background: {COLORS['success']};
    border-radius: 2px;
}}
QTableWidget, QTextEdit {{
    background: white;
    alternate-background-color: {COLORS['surface_alt']};
    border: 1px solid {COLORS['border']};
    border-radius: 2px;
    gridline-color: #e2e6ea;
    selection-background-color: #dcebf5;
    selection-color: {COLORS['text']};
}}
QHeaderView::section {{
    background: #e7ebef;
    border: none;
    border-right: 1px solid {COLORS['border']};
    border-bottom: 1px solid {COLORS['border_strong']};
    padding: 7px;
    font-weight: 700;
}}
QTableWidget#waypointTable QHeaderView::section {{ padding: 2px 5px; }}
QMenu#downwardComboPopup {{
    background: white;
    border: 1px solid {COLORS['border_strong']};
    padding: 2px;
}}
QMenu#downwardComboPopup::item {{
    min-height: 24px;
    padding: 5px 22px 5px 9px;
}}
QMenu#downwardComboPopup::item:selected {{
    background: #dcebf5;
    color: {COLORS['text']};
}}
QScrollArea {{ border: none; background: transparent; }}
QTabWidget::pane {{
    border: none;
    top: -1px;
}}
QTabBar::tab {{
    min-height: 31px;
    padding: 3px 15px;
    margin-right: 3px;
    background: #dde3e8;
    border: 1px solid {COLORS['border']};
    border-bottom: 2px solid {COLORS['border_strong']};
}}
QTabBar::tab:selected {{
    background: {COLORS['surface']};
    border-bottom: 2px solid {COLORS['accent']};
    font-weight: 700;
}}
QSplitter::handle {{ background: {COLORS['border']}; }}
QSplitter::handle:horizontal {{ width: 5px; }}
QSplitter::handle:vertical {{ height: 5px; }}
QScrollBar:vertical {{ background: #edf0f2; width: 11px; margin: 0; }}
QScrollBar::handle:vertical {{ background: #aeb8c4; min-height: 28px; }}
QScrollBar:horizontal {{ background: #edf0f2; height: 11px; margin: 0; }}
QScrollBar::handle:horizontal {{ background: #aeb8c4; min-width: 28px; }}
QStatusBar {{
    background: #e1e6ea;
    color: {COLORS['muted']};
    border-bottom-left-radius: 7px;
    border-bottom-right-radius: 7px;
}}
QToolTip {{ background: #243342; color: white; border: none; padding: 5px; }}
"""


def apply_theme(application: QApplication) -> None:
    """应用 Fusion 风格、浅色调色板和工程软件样式表。"""
    application.setStyle("Fusion")
    palette = QPalette()
    palette.setColor(QPalette.ColorRole.Window, QColor(COLORS["background"]))
    palette.setColor(QPalette.ColorRole.Base, QColor(COLORS["surface"]))
    palette.setColor(QPalette.ColorRole.Text, QColor(COLORS["text"]))
    palette.setColor(QPalette.ColorRole.WindowText, QColor(COLORS["text"]))
    palette.setColor(QPalette.ColorRole.Button, QColor(COLORS["surface_alt"]))
    palette.setColor(QPalette.ColorRole.ButtonText, QColor(COLORS["text"]))
    palette.setColor(QPalette.ColorRole.Highlight, QColor(COLORS["accent"]))
    palette.setColor(QPalette.ColorRole.HighlightedText, QColor("#ffffff"))
    application.setPalette(palette)
    application.setStyleSheet(STYLE_SHEET)
