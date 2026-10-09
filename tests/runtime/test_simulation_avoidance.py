"""GUI simulation composition, strict hardware isolation and first-surface geometry regressions."""
import importlib.util
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from ground_station_core.environment import EnvironmentInitializer
from ground_station_core.event_log import EventLog
from src.guided_sim.scripts.simulation_obstacles import raycast_scene, main as scene_main


class DemoSupervisor:
    def __init__(self):
        self.calls = []

    def start(self, name, command, **kwargs):
        self.calls.append((name, command, kwargs))
        return SimpleNamespace(running=True, log_path=Path('/tmp/simulation-avoidance.log'))


def initializer(domain=231, discovery='LOCALHOST'):
    events = EventLog()
    ros = SimpleNamespace(domain_id=domain, discovery_range=discovery, event_log=events,
                          snapshot=lambda: SimpleNamespace(avoidance_ready=True))
    supervisor = DemoSupervisor()
    return EnvironmentInitializer(ros, supervisor, events), supervisor


def test_default_gui_simulation_includes_demo_and_tests_can_opt_out(monkeypatch):
    env, _ = initializer()
    choices = []
    monkeypatch.setattr(env, '_simulation_workflow', lambda status, *, avoidance_demo=True: choices.append(avoidance_demo))
    monkeypatch.setattr(env, '_start_workflow', lambda name, work, *args: work())
    env.initialize_simulation(lambda *_: None, lambda *_: None)
    env.initialize_simulation(lambda *_: None, lambda *_: None, avoidance_demo=False)
    assert choices == [True, False]


def test_gui_demo_starts_real_planner_overlay_and_waits_for_authoritative_readiness(tmp_path, monkeypatch):
    setup = tmp_path / 'install/setup.bash'
    setup.parent.mkdir()
    setup.touch()
    monkeypatch.setenv('AVOIDANCE_WORKSPACE', str(tmp_path))
    env, supervisor = initializer()
    checked = []
    monkeypatch.setattr(env, '_verify_ros_packages_parallel', lambda packages, setups: checked.append(packages))
    messages = []
    assert env._start_simulation_avoidance_demo(lambda level, text: messages.append(text))
    assert checked == [('path_planning', 'avoidance_bridge')]
    name, command, kwargs = supervisor.calls[0]
    assert name == 'simulation_avoidance'
    assert command[-1] == 'avoidance_demo.launch.py'
    assert setup in kwargs['setup_files']
    assert kwargs['extra_environment']['ROS_DOMAIN_ID'] == '231'
    assert kwargs['extra_environment']['ROS_AUTOMATIC_DISCOVERY_RANGE'] == 'LOCALHOST'
    assert any('已就绪' in m for m in messages)


def test_missing_planner_keeps_original_straight_simulation_available(tmp_path, monkeypatch):
    monkeypatch.setenv('AVOIDANCE_WORKSPACE', str(tmp_path))
    env, supervisor = initializer()
    messages = []
    assert not env._start_simulation_avoidance_demo(lambda level, text: messages.append(text))
    assert not supervisor.calls
    assert any('直线仿真可继续' in m and '尚未安装' in m for m in messages)


@pytest.mark.parametrize('domain,discovery', [(0, 'SUBNET'), (231, 'SUBNET'), (0, 'LOCALHOST')])
def test_hardware_and_nonlocal_sessions_cannot_start_demo(domain, discovery):
    env, supervisor = initializer(domain, discovery)
    with pytest.raises(RuntimeError, match='禁止'):
        env._start_simulation_avoidance_demo(lambda *_: None)
    assert not supervisor.calls


@pytest.mark.parametrize('domain,discovery', [('0', 'SUBNET'), ('231', 'SUBNET')])
def test_scene_and_launch_refuse_real_aircraft_environment(monkeypatch, domain, discovery):
    monkeypatch.setenv('ROS_DOMAIN_ID', domain)
    monkeypatch.setenv('ROS_AUTOMATIC_DISCOVERY_RANGE', discovery)
    with pytest.raises(RuntimeError, match='requires'):
        scene_main()
    path = Path(__file__).resolve().parents[2] / 'src/guided_sim/launch/avoidance_demo.launch.py'
    spec = importlib.util.spec_from_file_location('avoidance_demo_launch', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    with pytest.raises(RuntimeError, match='only available'):
        module.generate_launch_description()


def test_cylinder_occludes_back_wall_and_removal_reveals_it():
    origin, anchor = np.array([0., 0., 1.5]), np.zeros(3)
    directions = np.array([[1., 0., 0.]])
    center = np.array([1.5, 0.])
    occupied = raycast_scene(origin, directions, anchor, center, True)
    clear = raycast_scene(origin, directions, anchor, center, False)
    assert occupied.shape == (1, 3) and clear.shape == (1, 3)
    np.testing.assert_allclose(occupied[0], [1.32, 0., 1.5], atol=1e-6)
    np.testing.assert_allclose(clear[0], [6., 0., 1.5], atol=1e-6)


def test_outside_demo_room_is_unknown_instead_of_fake_free_space():
    assert len(raycast_scene(np.array([7., 0., 1.5]), np.array([[1., 0., 0.]]),
                             np.zeros(3), np.array([1.5, 0.]), False)) == 0
