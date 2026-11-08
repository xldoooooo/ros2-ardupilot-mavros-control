"""功能闭环、界面专项与源码影响映射；数据调整只验证选择器及受影响功能。"""

from pathlib import Path

# 源码与构建产物统一相对仓库根目录解析。
ROOT = Path(__file__).resolve().parents[1]
# 功能分组与地面公共依赖的消费组。
GROUPS = ("runtime", "flight", "waypoints", "upstream", "video", "correction", "ui")
GROUND = ("runtime", "flight", "waypoints", "upstream", "ui")
# 面板和主窗口行为只验证实际界面调用链，不附带原生控制/媒体传输套件。
WAYPOINT_GUI = (
    "tests/waypoints/test_waypoint_panel.py",
    "tests/waypoints/test_waypoints_gui.py",
    "tests/waypoints/test_avoidance_gui.py",
    "tests/waypoints/test_avoidance_backend.py",
)
# 主窗口的连接、起飞门控、上位机编排与独立面板启动回归。
GROUND_GUI = (
    "tests/ui",
    "tests/runtime/test_runtime_gui.py",
    "tests/runtime/test_ground_startup.py",
    "tests/flight/test_flight_gui.py",
    "tests/flight/test_navigation_ground.py",
    "tests/flight/test_takeoff_voltage.py",
    "tests/upstream/test_upstream_gui.py",
    "tests/upstream/test_upstream_countdown.py",
    "tests/video/test_video_gui.py",
    "tests/correction/test_correction_gui.py",
    *WAYPOINT_GUI,
)
# 摄像头文件混有采集/IPC 测试，专项只选择面板行为；文件管理保留本机 HTTP 交互。
CAMERA_GUI = tuple(
    f"tests/video/test_camera_service.py::{name}" for name in (
        "test_panel_desktop_name_is_ascii",
        "test_panel_draws_own_window_shadow_and_preserves_window_controls",
        "test_panel_close_does_not_send_camera_stop",
        "test_panel_probe_keeps_requested_device_and_matching_modes",
        "test_preview_recreates_player_before_reusing_same_rtsp_url",
        "test_unavailable_preview_is_opaque_black_until_first_valid_frame",
        "test_preview_frame_watchdog_blacks_out_disconnect_but_preserves_pause",
        "test_panel_modes_gate_commands_but_keep_manual_viewer_available",
        "test_onboard_command_failure_is_shown_explicitly",
        "test_typed_rtsp_url_survives_config_poll_stop_and_backend_failure",
        "test_preview_icons_copy_validation_and_accessibility",
        "test_playback_metrics_freeze_on_pause_and_ignore_service_metrics",
        "test_every_panel_combo_and_spinbox_ignores_wheel",
        "test_small_preview_reserves_controls_with_video_and_media_details",
    )
)
# 专项与业务组可取并集；业务目录已经覆盖的专项用例只运行一次。
CHECKS = {
    "waypoint-gui": WAYPOINT_GUI,
    "ground-gui": GROUND_GUI,
    "gui": (*GROUND_GUI, *CAMERA_GUI, "tests/video/test_file_manager.py",
            "tests/correction/test_correction_panel.py"),
    "configuration": ("tests/runtime/test_project_configuration.py",),
}
# 执行器/映射的契约测试单独运行，不用产品全量测试替代选择器验证。
SELF_TESTS = ("tests/test_selection.py", "tests/test_build_native.py")
# 同一个文件可命中多条规则，取并集；跨模块调用点必须包含消费方闭环。
RULES = {
    "ground_station.py": ("runtime", "ground-gui"),
    "ground_station_core/bootstrap.py": ("runtime",),
    "ground_station_core/startup.py": ("runtime",),
    "ground_station_core/ground_media_reader.py": ("upstream",),
    "ground_station_core/environment.py": (
        "runtime",
        "flight",
        "waypoints",
        "upstream",
    ),
    "ground_station_core/process_manager.py": ("runtime",),
    "ground_station_core/event_log.py": GROUND,
    "ground_station_core/config.py": GROUPS,
    "project_configuration.py": GROUPS,
    "config/aircraft.local.env*": ("configuration",),
    "config/host.local.env*": ("runtime", "video", "upstream"),
    "config/aircraft/*/aircraft.yaml": ("configuration",),
    "config/aircraft/*/control.yaml": ("flight", "waypoints"),
    "config/aircraft/*/navigation.env": ("runtime", "flight", "waypoints"),
    "config/aircraft/*/avoidance/*": ("runtime", "flight", "waypoints"),
    "config/aircraft/*/onboard.env": ("runtime", "flight", "video", "correction"),
    "config/aircraft/*/odin/*": ("runtime", "flight", "correction"),
    "config/aircraft/*/odin_extrinsics.yaml": ("runtime", "flight", "correction"),
    "config/aircraft/*/mavros/*": ("runtime", "flight"),
    "config/aircraft/*/correction.env": ("runtime", "correction"),
    "config/aircraft/*/correction/*": ("correction", "flight", "waypoints"),
    "config/aircraft/*/video/*": ("video",),
    "config/ground/*": ("video", "upstream"),
    "config/mavros/*": ("runtime", "flight"),
    "config/video/*": ("video", "correction"),
    "system/*": ("runtime", "video", "correction"),
    "ground_station_core/models.py": GROUND,
    "ground_station_core/__init__.py": GROUND,
    "ground_station_core/ros_controller.py": (
        "flight",
        "waypoints",
        "runtime",
        "upstream",
    ),
    "ground_station_core/waypoint_io.py": ("waypoints", "upstream"),
    "ground_station_core/upstream/*": ("upstream",),
    "ground_station_core/upstream/status_projector.py": ("upstream", "video"),
    "ground_station_core/upstream/mapping.py": ("upstream", "flight"),
    "ground_station_core/qt_ui/main_window.py": ("ground-gui",),
    "ground_station_core/qt_ui/operations_panel.py": ("ground-gui",),
    "ground_station_core/qt_ui/state.py": ("ground-gui",),
    "ground_station_core/qt_ui/waypoint_panel.py": ("waypoint-gui",),
    "ground_station_core/qt_ui/upstream_panel.py": ("ground-gui",),
    "ground_station_core/qt_ui/log_panel.py": ("ui",),
    "ground_station_core/qt_ui/log_history.py": ("ui",),
    "ground_station_core/qt_ui/cached_shadow.py": ("gui",),
    "ground_station_core/qt_ui/theme.py": ("gui",),
    "ground_station_core/qt_ui/widgets.py": ("gui",),
    "ground_station_core/qt_ui/window_chrome.py": ("gui",),
    "ground_station_core/qt_ui/__init__.py": ("gui",),
    "video_service/*": ("video",),
    "video_service/camera_panel.py": ("video", "ui"),
    "*onboard_video.sh": ("video",),
    "correction_service/*": ("correction",),
    "src/correction_interfaces/*": ("correction", "flight", "waypoints"),
    "src/correction_native/*": ("correction",),
    "*onboard_correction.sh": ("correction",),
    "src/guided_sim/*": ("waypoints", "runtime"),
    "src/avoidance_bridge/*": ("runtime", "flight", "waypoints"),
    "drivers/odin/*": ("runtime", "flight", "correction"),
    "scripts/onboard/start_avoidance.sh": ("runtime", "flight", "waypoints"),
    "scripts/onboard/stop_avoidance.sh": ("runtime", "flight", "waypoints"),
    "scripts/onboard/switch_navigation_mode.sh": ("runtime", "flight", "waypoints"),
    # The removable module has an explicit independent test entry; default groups never import it.
    "fpga_avoidance/*": (),
    "src/onboard_control/src/main.cpp": ("flight", "waypoints", "video"),
    "src/onboard_control/src/onboard_control_node.cpp": (
        "flight",
        "waypoints",
        "video",
    ),
    "src/onboard_control/include/onboard_control/onboard_control_node.hpp": (
        "flight",
        "waypoints",
        "video",
    ),
    "src/onboard_control/src/fcu_reboot.cpp": ("flight",),
    "src/onboard_control/src/control_diagnostics.cpp": ("flight",),
    "src/onboard_control/src/tracking_diagnostics.cpp": ("flight",),
    "scripts/diagnostics/*": ("flight",),
    "src/onboard_control/src/navigation_frame.cpp": ("flight", "waypoints"),
    "src/onboard_control/include/onboard_control/navigation_frame.hpp": ("flight", "waypoints"),
    "src/onboard_control/src/waypoint_calibration.cpp": ("flight", "waypoints"),
    "src/onboard_control/test/test_navigation_frame.cpp": ("waypoints",),
    "src/onboard_control/src/avoidance_execution.cpp": ("flight", "waypoints"),
    "src/onboard_control/include/onboard_control/avoidance_trajectory.hpp": ("flight", "waypoints"),
    "src/onboard_control/test/test_avoidance_trajectory.cpp": ("flight", "waypoints"),
    "src/onboard_control/src/dob_controller.cpp": ("flight", "waypoints"),
    "src/onboard_control/include/onboard_control/dob_controller.hpp": ("flight", "waypoints"),
    "src/onboard_control/src/reference_generator.cpp": ("waypoints",),
    "src/onboard_control/include/onboard_control/reference_generator.hpp": (
        "waypoints",
    ),
    "src/onboard_control/include/onboard_control/waypoint_arrival_tracker.hpp": (
        "waypoints",
    ),
    "src/onboard_control/config/*": ("flight", "waypoints"),
    "src/onboard_control/launch/*": ("runtime", "flight"),
    "src/onboard_control/deploy/*": ("runtime", "correction"),
    "src/onboard_control/scripts/*": ("flight",),
    "src/onboard_control/test/test_dob_controller.cpp": ("flight",),
    "src/onboard_control/test/test_reference_generator.cpp": ("waypoints",),
    "src/onboard_control/test/test_waypoint_arrival_tracker.cpp": ("waypoints",),
    "src/onboard_control/CMakeLists.txt": ("runtime", "flight", "waypoints", "video"),
    "src/onboard_control/package.xml": ("runtime", "flight", "video"),
    "src/guided_interfaces/*": ("flight", "waypoints", "upstream", "video", "runtime"),
    "scripts/lib/*": ("runtime", "flight", "video", "correction"),
    "scripts/onboard/components/*": ("runtime", "flight", "video", "correction"),
    "*onboard_control.sh": ("runtime", "flight", "video", "correction"),
    "scripts/onboard/reboot_fcu.sh": ("flight",),
    "scripts/ground/start_ground_all.sh": ("runtime",),
    "scripts/ground/setup_ground_station.sh": ("runtime", "video", "correction"),
    "tests/support/__init__.py": GROUPS,
    "tests/support/qt.py": ("gui",),
    "tests/support/avoidance_node.py": ("flight",),
    "tests/support/calibration_node.py": ("flight",),
    "tests/support/navigation_node.py": ("flight",),
    "tests/support/extnav.py": ("correction",),
    "tests/support/mavros_param_retry_fixture.py": ("flight",),
    "tests/support/mavros_telemetry_fixture.py": ("flight",),
    "tests/conftest.py": GROUPS,
    "tests/__init__.py": GROUPS,
    "tests/run.py": GROUPS,
    "tests/impact.py": (),
    "tests/run.sh": GROUPS,
    "pytest.ini": GROUPS,
    "tests/build_native.sh": (),
    "tests/test_selection.py": (),
    "tests/test_build_native.py": (),
    "tests/test_qt_gui.py": GROUPS,
    "tests/test_bootstrap.py": ("runtime", "flight", "video"),
    "tests/test_onboard_deploy.py": ("runtime", "video"),
    "tests/test_ros_controller.py": ("flight", "waypoints"),
}
# 允许 --changed 的基线早于目录重构：旧路径删除仍归入迁移后的闭环。
for _group in GROUPS:
    for _test in (ROOT / "tests" / _group).glob("test_*.py"):
        RULES.setdefault(f"tests/{_test.name}", (_group,))
# 纯文档/实验记录不触发产品回归；未知代码报错，避免漏测或擅自全量。
IGNORED = ("*.md", "docs/*", "agent/*", ".gitignore")
# CTest 套件清单；与 onboard_control/CMakeLists.txt 的注册对应。
CPP_TESTS = {
    "flight": ("test_dob_controller",),
    "waypoints": ("test_navigation_frame", "test_reference_generator", "test_waypoint_arrival_tracker", "test_avoidance_trajectory"),
}
