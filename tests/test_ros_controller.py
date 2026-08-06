"""ROS 后台控制器的启动、失败反馈与干净停止测试。"""

from ground_station_core.models import FlightMode
from ground_station_core.ros_controller import GroundStationRosController


def test_controller_rejects_takeoff_without_fcu_and_stops() -> None:
    """未连接飞控时起飞应快速失败，ROS 线程仍能正常关闭。"""
    controller = GroundStationRosController()
    controller.start()
    try:
        assert controller.ready
        ticket = controller.request_takeoff(0.3)
        result = controller.wait_for_result(ticket, timeout=3.0)
        assert result is not None
        assert not result.success
        assert "飞控未连接" in result.message
        assert controller.active_mode is FlightMode.TAKEOFF_LAND
    finally:
        controller.stop()

    assert not controller.ready
    assert controller.active_mode is FlightMode.IDLE


def test_queued_old_command_cannot_override_new_keyboard_input() -> None:
    """较早排队的起飞命令不能在稍后执行时反向覆盖新方向键。"""
    controller = GroundStationRosController()
    controller.request_takeoff(0.3)
    command = controller._command_queue.get_nowait()
    controller.adjust_velocity(0.2, 0.0, 0.0, 0.0)

    activated = controller._claim_flight_action(command, lambda: None)
    assert not activated
    assert controller.active_mode is FlightMode.KEYBOARD
