"""原生采集时钟、调度器、入口与配置验证；不连接飞控。"""

import configparser
import shutil
from pathlib import Path

import pytest

import _correction_native as native
from correction_service.config import load_config
# Read the source profile directly; test modules are independently collectable.
CONFIG_DIR = Path(__file__).resolve().parents[2] / "config/aircraft/simulation/correction"


def _copy_configuration(root):
    """Copy the correction profile together with its shared Odin geometry."""
    config = root / "correction"
    shutil.copytree(CONFIG_DIR, config)
    shutil.copy2(CONFIG_DIR.parent / "odin_extrinsics.yaml", root / "odin_extrinsics.yaml")
    return config


def test_capture_clock_preserves_backlog_and_rejects_invalid_timestamp():
    """真正积压的帧保持积压年龄，不重新标为现在；拒绝缺失/未来采集时间。"""
    assert native.capture_stamp(0x2000, 100, 0, 100_500_000_000, 2_000_000_000)[1] == 500_000_000
    with pytest.raises(RuntimeError, match="CLOCK_MONOTONIC"):
        native.capture_stamp(0, 100, 0, 100_000_000_000, 100_000_000_000)
    with pytest.raises(RuntimeError, match="future"):
        native.capture_stamp(0x2000, 100, 0, 99_999_999_999, 100_000_000_000)


def test_old_external_driver_is_rejected(tmp_path):
    """现场遗留配置不能重新引入独立联合标定工作区。"""
    config_dir = _copy_configuration(tmp_path)
    path = config_dir / "camera.conf"
    path.write_text(path.read_text().replace("package = correction_service", "package = wasintek_gst_camera"))
    with pytest.raises(ValueError, match="本包"):
        load_config(config_dir)


def test_launch_and_install_have_no_external_camera_overlay():
    """入口链不 source 外部工作区，也不间接加载历史 underlay。"""
    from pathlib import Path
    root = Path(__file__).resolve().parents[2]
    launcher = (root / "scripts/onboard/start_onboard_correction.sh").read_text()
    installer = (root / "correction_service/deploy/install_correction_service.sh").read_text()
    for text in (launcher, installer):
        assert "vins_odin_calib" not in text
        assert "CAMERA_OVERLAY" not in text
        assert "install/local_setup.bash" in text


@pytest.mark.parametrize("source_fps", [29.97, 59.94, 60.1, 120.0])
def test_rate_limiter_keeps_average_rate_without_frame_rounding_loss(source_fps):
    """59.94→30 不能因每次重置截止时间退化为约 20 Hz；低帧率不补帧。"""
    limiter = native.CaptureRateLimiter(30.0)
    stamps = [1_000_000_000 + round(i * 1e9 / source_fps) for i in range(round(source_fps * 20))]
    kept = [stamp for stamp in stamps if limiter.accept(stamp)]
    assert abs(len(kept) - min(len(stamps), 600)) <= 1
    assert set(kept).issubset(stamps)


def test_rate_limiter_skips_stalls_and_zero_disables_limit():
    """断流跳过遗漏节拍，只处理恢复后的新帧；0 保留全部原始帧。"""
    limiter = native.CaptureRateLimiter(30.0)
    assert limiter.accept(1_000_000_000)
    assert not limiter.accept(1_010_000_000)
    assert limiter.accept(9_010_000_000)
    assert not limiter.accept(9_011_000_000)
    assert limiter.accept(9_040_000_000)
    unlimited = native.CaptureRateLimiter(0.0)
    assert all(unlimited.accept(stamp) for stamp in range(1000, 1100))


@pytest.mark.parametrize("value", [-1.0, float("nan"), float("inf")])
def test_rate_limiter_and_camera_config_reject_invalid_rates(tmp_path, value):
    """入口与文件配置均拒绝负数及非有限限频值，避免启动后失效。"""
    with pytest.raises(ValueError, match="publish_fps"):
        native.CaptureRateLimiter(value)
    config_dir = _copy_configuration(tmp_path)
    path = config_dir / "camera.conf"
    parser = configparser.ConfigParser()
    parser.read(path)
    parser["camera"]["publish_fps"] = str(value)
    with path.open("w") as stream:
        parser.write(stream)
    with pytest.raises(ValueError, match="publish_fps"):
        load_config(config_dir)


def test_camera_rate_default_and_unlimited_preserve_native_mode(tmp_path):
    """缺省使用 30；0 关闭软件限频，二者均保留显式原生采集帧率。"""
    config_dir = _copy_configuration(tmp_path)
    path = config_dir / "camera.conf"
    parser = configparser.ConfigParser()
    parser.read(path)
    parser["camera"].pop("publish_fps", None)
    parser["camera"]["fps"] = "55"
    with path.open("w") as stream:
        parser.write(stream)
    default = load_config(config_dir).camera
    assert default.publish_fps == 30.0
    assert default.fps == 55
    parser["camera"]["publish_fps"] = "0"
    with path.open("w") as stream:
        parser.write(stream)
    camera = load_config(config_dir).camera
    assert camera.publish_fps == 0.0
    assert camera.fps == 55


def test_camera_entry_executes_native_node_with_unchanged_ros_arguments(monkeypatch):
    """原入口exec替换进程，保留配置参数、进程组及原生退出码。"""
    from correction_service import uvc_camera_node as camera
    calls = []
    monkeypatch.setattr(camera, "get_package_prefix", lambda package: "/native" if package == "correction_native" else None)
    monkeypatch.setattr(camera.os, "execv", lambda executable, args: calls.append((executable, args)))
    arguments = ["--ros-args", "-p", "framerate:=60", "-p", "publish_fps:=10.0"]
    camera.main(arguments)
    assert calls == [("/native/lib/correction_native/uvc_camera_node",
                      ["/native/lib/correction_native/uvc_camera_node", *arguments])]
