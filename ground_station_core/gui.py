"""Tkinter 地面站界面：环境编排、状态展示与三种飞行模式入口。"""

from __future__ import annotations

import math
import queue
import threading
import tkinter as tk
from tkinter import messagebox

from .config import (
    DEFAULT_GPS_ORIGIN,
    PUBLISH_RATE_HZ,
    PUBLISH_TOPIC,
    TAKEOFF_ALTITUDE,
    VELOCITY_SCALE,
)
from .environment import EnvironmentInitializer
from .models import FlightMode
from .process_manager import CleanupReport
from .ros_controller import GroundStationRosController


class GroundStationApp:
    """地面站主窗口；业务逻辑均委托给 ROS、飞行模式及环境模块。"""

    _BACKGROUND = "#1c1c1c"

    def __init__(self, root: tk.Tk) -> None:
        """创建常驻 ROS 控制器、环境编排器和所有 Tk 控件。"""
        self.root = root
        self.root.title("地面站控制台 — Ground Station")
        self.root.geometry("1040x780")
        self.root.minsize(960, 700)
        self.root.configure(bg=self._BACKGROUND)
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

        self._ros = GroundStationRosController()
        self._ros.start()
        self._environment = EnvironmentInitializer(self._ros)
        self._ui_events: queue.Queue[tuple] = queue.Queue()
        self._cleanup_thread: threading.Thread | None = None
        self._closing = False
        self._last_result_sequence = 0
        self._waypoints: list[tuple[float, float, float, float]] = []

        self._build_ui()
        self._setup_keyboard()
        self._periodic_update()

    # ---- 整体布局 ----

    def _build_ui(self) -> None:
        """构建左右双栏 GUI。"""
        container = tk.Frame(self.root, bg=self._BACKGROUND)
        container.pack(fill="both", expand=True)
        self._left = tk.Frame(container, bg=self._BACKGROUND)
        self._left.pack(side="left", fill="both", expand=True)
        tk.Frame(container, width=2, bg="#444444").pack(
            side="left", fill="y", padx=(0, 4)
        )
        right = tk.Frame(container, bg=self._BACKGROUND)
        right.pack(side="left", fill="both", expand=True)

        self._build_control_panel(self._left)
        self._build_waypoint_panel(right)

    def _build_control_panel(self, parent: tk.Frame) -> None:
        """构建环境、GPS、起降、键盘和进程控制区。"""
        tk.Label(
            parent,
            text="地面站控制台",
            font=("Helvetica", 16, "bold"),
            fg="#ffffff",
            bg=self._BACKGROUND,
        ).pack(pady=(10, 0))

        status_row = tk.Frame(parent, bg=self._BACKGROUND)
        status_row.pack(pady=(0, 5))
        self._ros_status = self._status_label(status_row, "ROS2: 连接中...", "#ccaa00")
        self._ros_status.pack(side="left", padx=6)
        self._fc_status = self._status_label(status_row, "飞控: --", "#ccaa00")
        self._fc_status.pack(side="left", padx=6)
        self._mode_status = self._status_label(status_row, "模式: 待机", "#888888")
        self._mode_status.pack(side="left", padx=6)

        self._section_title(parent, "── 环境初始化 / Environment ──")
        button_style = {
            "font": ("Helvetica", 11, "bold"),
            "relief": "flat",
            "cursor": "hand2",
            "padx": 10,
            "pady": 6,
            "fg": "#ffffff",
        }
        self._simulation_button = tk.Button(
            parent,
            text="初始化仿真环境",
            bg="#2d5a27",
            activebackground="#3d7a33",
            command=self._initialize_simulation,
            **button_style,
        )
        self._simulation_button.pack(fill="x", padx=20, pady=(0, 4))
        self._hardware_button = tk.Button(
            parent,
            text="初始化实机环境",
            bg="#1a5a8a",
            activebackground="#2070a0",
            command=self._initialize_hardware,
            **button_style,
        )
        self._hardware_button.pack(fill="x", padx=20, pady=(0, 4))

        self._separator(parent)
        self._build_origin_panel(parent)
        self._separator(parent)
        self._build_takeoff_panel(parent)
        self._separator(parent)
        self._build_keyboard_panel(parent)
        self._separator(parent)
        self._build_process_panel(parent)

    def _build_origin_panel(self, parent: tk.Frame) -> None:
        """构建 GPS 原点输入及手动发送按钮。"""
        self._section_title(parent, "── GPS 原点 / Origin ──")
        row = tk.Frame(parent, bg=self._BACKGROUND)
        row.pack(pady=(0, 4))
        values = (
            ("Lat:", f"{DEFAULT_GPS_ORIGIN[0]:.7f}", 12),
            ("Lon:", f"{DEFAULT_GPS_ORIGIN[1]:.7f}", 12),
            ("Alt:", f"{DEFAULT_GPS_ORIGIN[2]:.1f}", 7),
        )
        self._origin_entries: dict[str, tk.Entry] = {}
        for index, (label, default, width) in enumerate(values):
            tk.Label(
                row,
                text=label,
                font=("Helvetica", 9),
                fg="#aaaaaa",
                bg=self._BACKGROUND,
            ).pack(side="left", padx=(6 if index == 0 else 2, 1))
            entry = tk.Entry(
                row,
                width=width,
                font=("Helvetica", 9),
                bg="#2a2a2a",
                fg="#ffffff",
                insertbackground="#ffffff",
                relief="flat",
            )
            entry.insert(0, default)
            entry.pack(side="left", padx=(0, 4))
            self._origin_entries[label] = entry
        tk.Button(
            row,
            text="设置原点",
            font=("Helvetica", 9),
            bg="#1a5a8a",
            fg="#ffffff",
            activebackground="#2070a0",
            relief="flat",
            padx=8,
            pady=3,
            cursor="hand2",
            command=self._set_origin,
        ).pack(side="left", padx=(4, 0))

    def _build_takeoff_panel(self, parent: tk.Frame) -> None:
        """构建起飞/降落模式按钮。"""
        self._section_title(parent, "── 起飞/降落模式 / Flight ──")
        row = tk.Frame(parent, bg=self._BACKGROUND)
        row.pack()
        style = {
            "font": ("Helvetica", 12, "bold"),
            "width": 12,
            "height": 1,
            "relief": "flat",
            "cursor": "hand2",
            "fg": "#ffffff",
        }
        tk.Button(
            row,
            text=f"起飞 {TAKEOFF_ALTITUDE:.1f}m",
            bg="#1a6a3a",
            activebackground="#208a40",
            command=self._takeoff,
            **style,
        ).pack(side="left", padx=6)
        tk.Button(
            row,
            text="降落",
            bg="#8b3a1a",
            activebackground="#a04020",
            command=self._land,
            **style,
        ).pack(side="left", padx=6)

    def _build_keyboard_panel(self, parent: tk.Frame) -> None:
        """构建键盘控制模式的方向及悬停按钮。"""
        self._section_title(parent, "── 键盘控制模式 / Velocity ──")
        tk.Label(
            parent,
            text=(
                f"Topic: {PUBLISH_TOPIC}  |  步长: {VELOCITY_SCALE}  |  "
                f"{PUBLISH_RATE_HZ:.0f} Hz"
            ),
            font=("Helvetica", 8),
            fg="#666666",
            bg=self._BACKGROUND,
        ).pack(pady=(0, 4))

        direction = tk.Frame(parent, bg=self._BACKGROUND)
        direction.pack()
        self._direction_button(direction, "↑ 上 W", 0, 1, 0, 0, VELOCITY_SCALE, 0)
        self._direction_button(direction, "← 左 J", 1, 0, 0, VELOCITY_SCALE, 0, 0)
        tk.Button(
            direction,
            text="■\n悬停",
            font=("Helvetica", 12, "bold"),
            width=6,
            height=1,
            relief="flat",
            cursor="hand2",
            bg="#8b6f1a",
            fg="#ffffff",
            activebackground="#a08020",
            command=self._hover,
        ).grid(row=1, column=1, padx=3, pady=2)
        self._direction_button(direction, "→ 右 L", 1, 2, 0, -VELOCITY_SCALE, 0, 0)
        self._direction_button(direction, "↓ 下 S", 2, 1, 0, 0, -VELOCITY_SCALE, 0)

        forward = tk.Frame(parent, bg=self._BACKGROUND)
        forward.pack(pady=(4, 0))
        self._direction_button(
            forward, "▲ 前 I", 0, 0, VELOCITY_SCALE, 0, 0, 0, color="#1a5a8a"
        )
        self._direction_button(
            forward, "▼ 后 K", 0, 1, -VELOCITY_SCALE, 0, 0, 0, color="#1a5a8a"
        )
        yaw = tk.Frame(parent, bg=self._BACKGROUND)
        yaw.pack(pady=(4, 0))
        self._direction_button(
            yaw, "↺ 左转 A", 0, 0, 0, 0, 0, VELOCITY_SCALE, color="#5a275a"
        )
        self._direction_button(
            yaw, "↻ 右转 D", 0, 1, 0, 0, 0, -VELOCITY_SCALE, color="#5a275a"
        )

        self._velocity_status = tk.Label(
            parent,
            text="Vx: +0.00  Vy: +0.00  Vz: +0.00  Yaw: +0.00",
            font=("Courier", 11, "bold"),
            fg="#00cc66",
            bg=self._BACKGROUND,
        )
        self._velocity_status.pack(pady=(7, 1))
        self._position_status = tk.Label(
            parent,
            text="X: --.--  Y: --.--  Z: --.--  Yaw: --.-°",
            font=("Courier", 11, "bold"),
            fg="#00aacc",
            bg=self._BACKGROUND,
        )
        self._position_status.pack(pady=(0, 3))
        tk.Label(
            parent,
            text="Space 悬停 | W/S 上下 | I/K 前后 | J/L 左右 | A/D 偏航",
            font=("Helvetica", 8),
            fg="#555555",
            bg=self._BACKGROUND,
        ).pack()

    def _build_process_panel(self, parent: tk.Frame) -> None:
        """构建彻底清理与退出按钮及主状态文本。"""
        row = tk.Frame(parent, bg=self._BACKGROUND)
        row.pack(pady=(0, 4))
        self._cleanup_button = tk.Button(
            row,
            text="关闭所有进程",
            font=("Helvetica", 10),
            bg="#8b3a1a",
            fg="#ffffff",
            activebackground="#a04020",
            relief="flat",
            padx=8,
            pady=4,
            cursor="hand2",
            command=self._cleanup_all,
        )
        self._cleanup_button.pack(side="left", padx=4)
        tk.Button(
            row,
            text="退出界面",
            font=("Helvetica", 10),
            bg="#333333",
            fg="#aaaaaa",
            activebackground="#444444",
            relief="flat",
            padx=8,
            pady=4,
            cursor="hand2",
            command=self._on_close,
        ).pack(side="left", padx=4)
        self._main_status = tk.Label(
            parent,
            text="就绪",
            wraplength=470,
            font=("Helvetica", 9),
            fg="#777777",
            bg=self._BACKGROUND,
        )
        self._main_status.pack(pady=(4, 5))

    # ---- 航点面板 ----

    def _build_waypoint_panel(self, parent: tk.Frame) -> None:
        """构建航点飞行模式的输入、排序和发送面板。"""
        tk.Label(
            parent,
            text="航点飞行模式 / Waypoints",
            font=("Helvetica", 13, "bold"),
            fg="#ffffff",
            bg=self._BACKGROUND,
        ).pack(pady=(10, 6))
        tk.Label(
            parent,
            text="本地 ENU 坐标 · 直接飞行 · PD+DOB 跟踪",
            font=("Helvetica", 9),
            fg="#777777",
            bg=self._BACKGROUND,
        ).pack(pady=(0, 5))

        entry_row = tk.Frame(parent, bg=self._BACKGROUND)
        entry_row.pack(pady=(0, 4))
        self._waypoint_entries: dict[str, tk.Entry] = {}
        defaults = (("X", "0.0"), ("Y", "0.0"), ("Z", "1.0"), ("Yaw°", "0.0"))
        for column, (label, default) in enumerate(defaults):
            tk.Label(
                entry_row,
                text=f"{label}:",
                font=("Helvetica", 9),
                fg="#aaaaaa",
                bg=self._BACKGROUND,
            ).grid(row=0, column=column * 2, padx=(3, 1))
            entry = tk.Entry(
                entry_row,
                width=7,
                font=("Helvetica", 10),
                bg="#2a2a2a",
                fg="#ffffff",
                insertbackground="#ffffff",
                relief="flat",
            )
            entry.insert(0, default)
            entry.grid(row=0, column=column * 2 + 1, padx=(0, 4))
            self._waypoint_entries[label] = entry

        tk.Button(
            parent,
            text="＋ 添加航点",
            font=("Helvetica", 10, "bold"),
            bg="#1a5a8a",
            fg="#ffffff",
            activebackground="#2070a0",
            relief="flat",
            padx=12,
            pady=4,
            cursor="hand2",
            command=self._add_waypoint,
        ).pack(pady=(0, 5))

        list_frame = tk.Frame(parent, bg=self._BACKGROUND)
        list_frame.pack(fill="both", expand=True, padx=10)
        scrollbar = tk.Scrollbar(list_frame)
        scrollbar.pack(side="right", fill="y")
        self._waypoint_list = tk.Listbox(
            list_frame,
            font=("Courier", 10),
            bg="#2a2a2a",
            fg="#00cc66",
            selectbackground="#1a5a8a",
            selectforeground="#ffffff",
            relief="flat",
            height=10,
            yscrollcommand=scrollbar.set,
        )
        self._waypoint_list.pack(side="left", fill="both", expand=True)
        scrollbar.config(command=self._waypoint_list.yview)

        controls = tk.Frame(parent, bg=self._BACKGROUND)
        controls.pack(pady=(5, 0))
        for text, command, color in (
            ("删除选中", self._remove_waypoint, "#5a2727"),
            ("上移", self._move_waypoint_up, "#3a3a3a"),
            ("下移", self._move_waypoint_down, "#3a3a3a"),
            ("清空全部", self._clear_waypoints, "#5a2727"),
        ):
            tk.Button(
                controls,
                text=text,
                font=("Helvetica", 9),
                bg=color,
                fg="#ffffff",
                activebackground="#4a4a4a",
                relief="flat",
                cursor="hand2",
                padx=8,
                pady=4,
                command=command,
            ).pack(side="left", padx=3)

        tk.Frame(parent, height=1, bg="#444444").pack(
            fill="x", padx=10, pady=(8, 7)
        )
        self._waypoint_send_button = tk.Button(
            parent,
            text="▶ 发送航点",
            font=("Helvetica", 12, "bold"),
            bg="#1a6a3a",
            fg="#ffffff",
            activebackground="#208a40",
            relief="flat",
            padx=16,
            pady=6,
            cursor="hand2",
            command=self._send_waypoints,
        )
        self._waypoint_send_button.pack()
        self._waypoint_status = tk.Label(
            parent,
            text="就绪 — 请添加航点",
            wraplength=470,
            font=("Helvetica", 9),
            fg="#666666",
            bg=self._BACKGROUND,
        )
        self._waypoint_status.pack(pady=(7, 8))

    # ---- 环境按钮 ----

    def _initialize_simulation(self) -> None:
        """从 GUI 启动完整仿真初始化工作流。"""
        self._set_environment_buttons(False)
        started = self._environment.initialize_simulation(
            self._queue_environment_status, self._queue_environment_done
        )
        if not started:
            self._set_environment_buttons(True)

    def _initialize_hardware(self) -> None:
        """读取 GPS 原点并启动完整实机初始化工作流。"""
        origin = self._read_origin()
        if origin is None:
            return
        self._set_environment_buttons(False)
        started = self._environment.initialize_hardware(
            origin, self._queue_environment_status, self._queue_environment_done
        )
        if not started:
            self._set_environment_buttons(True)

    def _queue_environment_status(self, message: str) -> None:
        """工作线程只写队列，避免跨线程调用 Tk。"""
        self._ui_events.put(("status", message))

    def _queue_environment_done(self, success: bool, message: str) -> None:
        """排队初始化完成事件。"""
        self._ui_events.put(("done", success, message))

    def _cleanup_all(self) -> None:
        """后台执行彻底清理，保持 GUI 可响应并允许后续重新初始化。"""
        if self._cleanup_thread is not None and self._cleanup_thread.is_alive():
            return
        self._set_environment_buttons(False)
        self._cleanup_button.config(state="disabled")
        self._main_status.config(text="正在终止并校验所有 ROS/SITL 进程...", fg="#ccaa00")

        def worker() -> None:
            report = self._environment.cleanup()
            self._ui_events.put(("cleanup", report))

        self._cleanup_thread = threading.Thread(
            target=worker, name="ground-station-cleanup", daemon=True
        )
        self._cleanup_thread.start()

    # ---- 飞行按钮 ----

    def _takeoff(self) -> None:
        """按下起飞键，选择性覆盖当前飞行模式。"""
        self._main_status.config(text="正在执行起飞流程...", fg="#ccaa00")
        self._ros.request_takeoff(TAKEOFF_ALTITUDE)

    def _land(self) -> None:
        """按下降落键，选择性覆盖当前飞行模式。"""
        self._main_status.config(text="正在发送降落指令...", fg="#ccaa00")
        self._ros.request_land()

    def _on_direction(self, vx: float, vy: float, vz: float, yaw_rate: float) -> None:
        """按下方向键，选择键盘模式并累加一次速度。"""
        self._ros.adjust_velocity(vx, vy, vz, yaw_rate)
        self._main_status.config(text="键盘控制模式已接管", fg="#00cc66")

    def _hover(self) -> None:
        """按下悬停键，选择键盘模式的 PD+DOB 分支。"""
        self._ros.request_hover()
        self._main_status.config(text="键盘控制模式 — PD+DOB 悬停", fg="#00cc66")

    def _set_origin(self) -> None:
        """手动发布当前输入的 GPS 原点。"""
        origin = self._read_origin()
        if origin is None:
            return
        self._main_status.config(text="正在设置 GPS 原点...", fg="#ccaa00")
        self._ros.request_set_gp_origin(*origin)

    # ---- 航点编辑 ----

    def _add_waypoint(self) -> None:
        """校验输入并在本地航点列表末尾追加一项。"""
        try:
            waypoint = (
                float(self._waypoint_entries["X"].get()),
                float(self._waypoint_entries["Y"].get()),
                float(self._waypoint_entries["Z"].get()),
                math.radians(float(self._waypoint_entries["Yaw°"].get())),
            )
        except ValueError:
            messagebox.showwarning("输入错误", "X、Y、Z、Yaw 必须是有效数字。")
            return
        self._waypoints.append(waypoint)
        self._refresh_waypoint_list()
        self._waypoint_status.config(
            text=f"已添加 {len(self._waypoints)} 个航点", fg="#00cc66"
        )

    def _remove_waypoint(self) -> None:
        """删除当前选中航点。"""
        selection = self._waypoint_list.curselection()
        if selection:
            self._waypoints.pop(selection[0])
            self._refresh_waypoint_list()

    def _clear_waypoints(self) -> None:
        """清空待发送航点列表。"""
        self._waypoints.clear()
        self._refresh_waypoint_list()
        self._waypoint_status.config(text="已清空 — 请添加航点", fg="#666666")

    def _move_waypoint_up(self) -> None:
        """将选中航点上移一位。"""
        selection = self._waypoint_list.curselection()
        if not selection or selection[0] == 0:
            return
        index = selection[0]
        self._waypoints[index - 1], self._waypoints[index] = (
            self._waypoints[index],
            self._waypoints[index - 1],
        )
        self._refresh_waypoint_list(index - 1)

    def _move_waypoint_down(self) -> None:
        """将选中航点下移一位。"""
        selection = self._waypoint_list.curselection()
        if not selection or selection[0] >= len(self._waypoints) - 1:
            return
        index = selection[0]
        self._waypoints[index + 1], self._waypoints[index] = (
            self._waypoints[index],
            self._waypoints[index + 1],
        )
        self._refresh_waypoint_list(index + 1)

    def _refresh_waypoint_list(self, selection: int | None = None) -> None:
        """根据内存数据重建带编号的航点显示。"""
        self._waypoint_list.delete(0, tk.END)
        for index, waypoint in enumerate(self._waypoints, start=1):
            self._waypoint_list.insert(
                tk.END,
                f"#{index}: X={waypoint[0]:+.1f}  Y={waypoint[1]:+.1f}  "
                f"Z={waypoint[2]:+.1f}  Yaw={math.degrees(waypoint[3]):+.0f}°",
            )
        if selection is not None:
            self._waypoint_list.selection_set(selection)

    def _send_waypoints(self) -> None:
        """按下发送键，选择航点模式并开始执行列表副本。"""
        if not self._waypoints:
            messagebox.showwarning("无航点", "请先添加至少一个航点。")
            return
        self._waypoint_send_button.config(state="disabled", text="任务执行中...")
        self._waypoint_status.config(text="正在启动航点任务...", fg="#ccaa00")
        self._ros.request_waypoints(tuple(self._waypoints))

    # ---- 周期更新与键盘 ----

    def _periodic_update(self) -> None:
        """在 Tk 主线程统一消费 ROS/环境结果并刷新状态。"""
        if self._closing:
            return
        snapshot = self._ros.snapshot()
        if self._ros.ready:
            self._ros_status.config(text="ROS2: 已连接", fg="#00cc66")
        elif self._ros.error:
            self._ros_status.config(text=f"ROS2: 错误 {self._ros.error[:45]}", fg="#cc3333")
        else:
            self._ros_status.config(text="ROS2: 连接中...", fg="#ccaa00")
        self._fc_status.config(
            text=f"飞控: {'已连接' if snapshot.connected else '未连接'}",
            fg="#00cc66" if snapshot.connected else "#ccaa00",
        )
        mode = self._ros.active_mode
        self._mode_status.config(
            text=f"模式: {mode.value}", fg="#00cc66" if mode is not FlightMode.IDLE else "#888888"
        )
        vx, vy, vz, yaw_rate = self._ros.velocity
        self._velocity_status.config(
            text=f"Vx: {vx:+.2f}  Vy: {vy:+.2f}  Vz: {vz:+.2f}  Yaw: {yaw_rate:+.2f}"
        )
        self._position_status.config(
            text=f"X: {snapshot.x:+.2f}  Y: {snapshot.y:+.2f}  Z: {snapshot.z:+.2f}  "
            f"Yaw: {math.degrees(snapshot.yaw):+.1f}°"
        )
        self._consume_ros_results()
        self._consume_ui_events()
        self.root.after(200, self._periodic_update)

    def _consume_ros_results(self) -> None:
        """增量消费 ROS 命令结果，避免旧消息反复覆盖 GUI。"""
        for result in self._ros.results_after(self._last_result_sequence):
            self._last_result_sequence = max(self._last_result_sequence, result.sequence)
            color = "#00cc66" if result.success else "#cc3333"
            self._main_status.config(text=result.message, fg=color)
            if result.command == "waypoints":
                self._waypoint_status.config(text=result.message, fg=color)
                if result.final:
                    self._waypoint_send_button.config(
                        state="normal", text="▶ 发送航点"
                    )

    def _consume_ui_events(self) -> None:
        """消费环境线程传来的纯数据事件并更新 Tk 控件。"""
        while True:
            try:
                event = self._ui_events.get_nowait()
            except queue.Empty:
                return
            if event[0] == "status":
                self._main_status.config(text=event[1], fg="#ccaa00")
            elif event[0] == "done":
                _, success, message = event
                self._main_status.config(
                    text=message, fg="#00cc66" if success else "#cc3333"
                )
                if self._cleanup_thread is None or not self._cleanup_thread.is_alive():
                    self._set_environment_buttons(True)
            elif event[0] == "cleanup":
                report: CleanupReport = event[1]
                if report.success:
                    message = (
                        f"清理完成：终止 {report.managed_stopped} 个受管进程、"
                        f"{len(report.stale_stopped)} 个历史残留；已验证无 ROS/SITL 残余"
                    )
                    color = "#00cc66"
                else:
                    message = f"清理未完成：残留={report.remaining}，错误={report.errors}"
                    color = "#cc3333"
                self._main_status.config(text=message, fg=color)
                self._cleanup_button.config(state="normal")
                self._set_environment_buttons(True)

    def _setup_keyboard(self) -> None:
        """绑定控制键；输入框获得焦点时不触发飞行命令。"""
        mapping = {
            "w": (0.0, 0.0, VELOCITY_SCALE, 0.0),
            "s": (0.0, 0.0, -VELOCITY_SCALE, 0.0),
            "i": (VELOCITY_SCALE, 0.0, 0.0, 0.0),
            "k": (-VELOCITY_SCALE, 0.0, 0.0, 0.0),
            "j": (0.0, VELOCITY_SCALE, 0.0, 0.0),
            "l": (0.0, -VELOCITY_SCALE, 0.0, 0.0),
            "a": (0.0, 0.0, 0.0, VELOCITY_SCALE),
            "d": (0.0, 0.0, 0.0, -VELOCITY_SCALE),
        }

        def keypress(event) -> None:
            if isinstance(event.widget, tk.Entry):
                return
            key = event.keysym.lower()
            if key == "space":
                self._hover()
            elif key in mapping:
                self._on_direction(*mapping[key])

        self.root.bind("<KeyPress>", keypress)

    # ---- 小型 UI 工具 ----

    def _read_origin(self) -> tuple[float, float, float] | None:
        """读取并校验 GPS 原点输入。"""
        try:
            return (
                float(self._origin_entries["Lat:"].get()),
                float(self._origin_entries["Lon:"].get()),
                float(self._origin_entries["Alt:"].get()),
            )
        except ValueError:
            messagebox.showwarning("输入错误", "GPS 纬度、经度和高度必须是有效数字。")
            return None

    def _direction_button(
        self,
        parent: tk.Frame,
        text: str,
        row: int,
        column: int,
        vx: float,
        vy: float,
        vz: float,
        yaw_rate: float,
        color: str = "#2d5a27",
    ) -> None:
        """创建一个累加速度方向按钮。"""
        tk.Button(
            parent,
            text=text,
            font=("Helvetica", 11, "bold"),
            width=6,
            height=1,
            relief="flat",
            cursor="hand2",
            bg=color,
            fg="#ffffff",
            activebackground="#3d7a33",
            command=lambda: self._on_direction(vx, vy, vz, yaw_rate),
        ).grid(row=row, column=column, padx=2, pady=1)

    def _set_environment_buttons(self, enabled: bool) -> None:
        """统一设置两个初始化按钮状态。"""
        state = "normal" if enabled else "disabled"
        self._simulation_button.config(state=state)
        self._hardware_button.config(state=state)

    @classmethod
    def _status_label(cls, parent: tk.Frame, text: str, color: str) -> tk.Label:
        """创建顶部紧凑状态标签。"""
        return tk.Label(
            parent,
            text=text,
            font=("Helvetica", 9),
            fg=color,
            bg=cls._BACKGROUND,
        )

    @classmethod
    def _section_title(cls, parent: tk.Frame, text: str) -> None:
        """创建分区标题。"""
        tk.Label(
            parent,
            text=text,
            font=("Helvetica", 9, "bold"),
            fg="#888888",
            bg=cls._BACKGROUND,
        ).pack(pady=(0, 4))

    @classmethod
    def _separator(cls, parent: tk.Frame) -> None:
        """创建横向分隔线。"""
        tk.Frame(parent, height=1, bg="#444444").pack(
            fill="x", padx=20, pady=(5, 6)
        )

    def _on_close(self) -> None:
        """退出前同步清理外部进程与内嵌 ROS 节点。"""
        if self._closing:
            return
        self._closing = True
        self._main_status.config(text="正在安全退出并清理进程...", fg="#ccaa00")
        self.root.update_idletasks()
        if self._cleanup_thread is not None and self._cleanup_thread.is_alive():
            self._cleanup_thread.join(timeout=8.0)
        report = self._environment.cleanup()
        if not report.success:
            print(
                f"[GS] 退出清理仍有残留: {report.remaining}, errors={report.errors}",
                flush=True,
            )
        self._ros.stop()
        self.root.destroy()
