"""correction_service 2.0 的目标采样、窗口 CAS、clear、session 与应用对账测试。"""

from __future__ import annotations

import math
import shutil
import time
from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace
from typing import Any

import numpy as np
import pytest
import yaml
from rclpy.serialization import serialize_message
from correction_service.estimator import QualitySnapshot
from correction_service.journal import JobJournal
from correction_service.window import (
    CalibrationKeyframe,
    RegistrationResult,
    WindowCandidate,
)

# These cases create real ROS contexts and subscriptions.
pytestmark = pytest.mark.integration

# 当前源码根目录和本机仿真档案；测试不使用飞机配置。
PROJECT_ROOT = Path(__file__).resolve().parents[2]
CONFIG_DIR = PROJECT_ROOT / "config" / "aircraft" / "simulation" / "correction"


def _keyframe(job_id: str = "sample-job") -> CalibrationKeyframe:
    """构造一个已通过停留段质量门的首个 keyframe。"""
    return CalibrationKeyframe(
        tag_id=0,
        world_x_m=0.0,
        world_y_m=0.0,
        odin_x_m=-0.1,
        odin_y_m=0.2,
        capture_start_ns=1_000_000_000,
        capture_end_ns=3_500_000_000,
        created_monotonic=time.monotonic(),
        created_ros_ns=3_500_000_000,
        samples_accepted=30,
        samples_rejected=2,
        independent_blocks=10,
        position_std_m=0.004,
        effective_sigma_m=0.012,
        reprojection_error_px=0.4,
        odom_match_error_ms=3.0,
        max_sync_motion_error_m=0.001,
        correction_tilt_rad=math.radians(1.0),
        single_tag_yaw_rad=math.radians(0.2),
        odom_time_source="header",
        odin_session_id="odin-a",
        config_fingerprint="placeholder",
        source_job_id=job_id,
    )


def _candidate(node: Any, *, saved: bool = False) -> WindowCandidate:
    """构造与节点当前实例/session/revision 相符的冻结粗候选。"""
    keyframe = replace(_keyframe(), config_fingerprint=node.config.config_fingerprint)
    registration = RegistrationResult(
        valid=True,
        reason="coarse",
        x_m=0.1,
        y_m=-0.2,
        yaw_rad=math.radians(0.2),
        rms_m=0.0,
        max_residual_m=0.0,
        model_yaw_std_rad=math.radians(0.1),
        residuals_m=(0.0,),
    )
    return WindowCandidate(
        keyframes=(keyframe,),
        x_m=0.1,
        y_m=-0.2,
        yaw_rad=math.radians(0.2),
        stage="rough_single_tag",
        odin_session_id="odin-a",
        config_fingerprint=node.config.config_fingerprint,
        source_job_id="sample-job",
        application_job_id="sample-job-apply",
        base_window_revision=node._window_revision,
        expected_extnav_revision=node._extnav.revision,
        created_monotonic=time.monotonic(),
        created_ros_ns=3_500_000_000,
        valid=True,
        quality_reason="coarse",
        position_std_m=0.012,
        yaw_std_rad=math.radians(0.1),
        correction_tilt_rad=math.radians(1.0),
        reprojection_error_px=0.4,
        odom_match_error_ms=3.0,
        odom_time_source="header",
        registration=registration,
        can_apply=True,
        apply_reason="test",
        saved=saved,
        saved_window_revision=node._window_revision if saved else 0,
        application_state="not_requested" if saved else "pending",
    )


@pytest.fixture
def correction_node(monkeypatch, tmp_path):
    """在 localhost-only 独立 domain 创建不启动相机的真实节点实例。"""
    rclpy = pytest.importorskip("rclpy")
    from correction_service.node import CorrectionServiceNode, _ExtnavSnapshot
    from rclpy.context import Context
    from rclpy.parameter import Parameter

    monkeypatch.setenv("ROS_AUTOMATIC_DISCOVERY_RANGE", "LOCALHOST")
    context = Context()
    rclpy.init(args=[], context=context, domain_id=226)
    config_dir = tmp_path / "node-profile" / "correction"
    shutil.copytree(CONFIG_DIR, config_dir)
    shutil.copy2(CONFIG_DIR.parent / "odin_extrinsics.yaml", config_dir.parent / "odin_extrinsics.yaml")
    settings_path = config_dir / "general_settings.yaml"
    settings = yaml.safe_load(settings_path.read_text())
    settings["logging"]["directory"] = str(tmp_path / "logs")
    settings_path.write_text(yaml.safe_dump(settings))
    node = CorrectionServiceNode(
        context=context,
        parameter_overrides=[Parameter("config_dir", value=str(config_dir))],
    )
    node._extnav = _ExtnavSnapshot(
        received_monotonic=time.monotonic(),
        interface_version="2.0",
        service_available=True,
        odin_available=True,
        odin_session_id="odin-a",
        correction_valid=False,
        revision=0,
        lever_arm_m=node.config.fcu_reference.lever_arm_m.copy(),
        installation_rpy=np.zeros(3),
    )
    yield node
    node.close()
    node.destroy_node()
    context.shutdown()


def test_idle_capture_subscriptions_are_created_and_released_on_demand(correction_node):
    """默认 idle 不订相机或 raw；任务订阅在采集结束后实际销毁。"""
    node = correction_node
    capture_topics = {node.config.topics["raw_odometry"], node.config.topics["camera_image"]}
    assert capture_topics.isdisjoint(subscription.topic_name for subscription in node.subscriptions)
    node._start_odometry_capture()
    node._start_image_capture()
    assert node._synchronizer.running
    assert node.config.topics["camera_image"] in {subscription.topic_name for subscription in node.subscriptions}
    assert node._stop_image_capture()
    assert node._stop_odometry_capture()
    assert not node._synchronizer.running
    assert capture_topics.isdisjoint(subscription.topic_name for subscription in node.subscriptions)


def _job(node: Any, tmp_path: Path, candidate: WindowCandidate):
    """构造无需启动相机即可测试提交路径的内部 job。"""
    from correction_service.node import _Job

    from correction_interfaces.msg import CorrectionStatus

    return _Job(
        job_id="sample-job",
        source_id="test",
        operation=CorrectionStatus.OPERATION_FIRST,
        expected_tag_id=0,
        window_size=5,
        apply_requested=False,
        odin_session_id="odin-a",
        base_revision=node._extnav.revision,
        base_window_revision=node._window_revision,
        started_monotonic=time.monotonic(),
        started_ros_ns=1,
        estimator=None,
        journal=JobJournal(tmp_path, "node-state"),
        candidate=candidate,
        temporary_window=candidate.keyframes,
        resources_released=True,
    )


def test_raw_capture_requires_fcu_frame_and_nonzero_installation_can_apply(
    correction_node,
) -> None:
    """旧 IMU 流不得进入标定；固定安装角已由驱动转换，不再阻止应用。"""
    from nav_msgs.msg import Odometry

    node = correction_node
    node._start_odometry_capture()
    raw = Odometry()
    raw.header.frame_id = "odom"
    raw.child_frame_id = "odin_imu"
    raw.pose.pose.orientation.w = 1.0
    node._synchronizer._capture.add(
        serialize_message(raw), node.get_clock().now().nanoseconds, time.monotonic()
    )
    node._on_odometry_fault()
    with node._lock:
        node._refresh_odometry_locked()
    assert node._latest_raw_transform is None

    raw.child_frame_id = "fcu"
    node._synchronizer._capture.add(
        serialize_message(raw), node.get_clock().now().nanoseconds, time.monotonic()
    )
    node._on_odometry_fault()
    with node._lock:
        node._refresh_odometry_locked()
    assert np.allclose(node._latest_raw_transform, np.eye(4))
    node._extnav.installation_rpy = np.array((0.0, -math.pi / 6.0, 0.0))
    candidate = _candidate(node)
    with node._lock:
        evaluated = node._evaluate_candidate_apply_locked(candidate)
    assert evaluated.can_apply, evaluated.apply_reason
    expected = np.linalg.norm(
        np.array((candidate.x_m, candidate.y_m))
        - node.config.fcu_reference.lever_arm_m[:2]
    )
    assert math.isclose(evaluated.expected_position_jump_m, expected, abs_tol=1e-12)


def test_native_cleanup_retains_faults_before_final_commit(correction_node, tmp_path):
    """原生线程退出前的异常仍须令任务失败，不能被close清理吞掉。"""
    from nav_msgs.msg import Odometry
    node = correction_node
    node._job = _job(node, tmp_path, _candidate(node))
    node._start_odometry_capture()
    raw = Odometry()
    raw.header.frame_id, raw.child_frame_id = "odom", "odin_imu"
    node._synchronizer._capture.add(
        serialize_message(raw), node.get_clock().now().nanoseconds, time.monotonic()
    )
    assert node._stop_odometry_capture()
    assert not node._synchronizer.running
    with pytest.raises(RuntimeError, match="odom/fcu"):
        node._raise_if_stopping(node._job)


def test_window_commit_and_clear_increment_revision_without_touching_extnav(
    correction_node, tmp_path: Path
) -> None:
    """成功才增加 N；clear 归零 N 但 revision 不归零且 active 不受影响。"""
    node = correction_node
    candidate = _candidate(node)
    job = _job(node, tmp_path, candidate)

    with node._lock:
        saved = node._commit_new_window_locked(
            job, candidate, application_state="not_requested"
        )
    assert saved.saved
    assert node._successful_calibrations == 1
    assert node._window_revision == 1
    assert len(node._window) == 1
    assert node._odom_subscription is None
    assert node._image_subscription is None

    node._extnav.correction_valid = True
    node._extnav.correction_x_m = 7.0
    request = SimpleNamespace(
        source_id="panel",
        sequence=1,
        expected_service_instance_id=node._service_instance_id,
        expected_window_revision=1,
    )
    response = node._on_clear_window(request, SimpleNamespace())

    assert response.accepted
    assert response.window_revision == 2
    assert node._window == ()
    assert node._successful_calibrations == 0
    assert node._extnav.correction_valid
    assert node._extnav.correction_x_m == 7.0


def test_first_frozen_candidate_preserves_task27_aggregated_c_full_projection(
    correction_node, tmp_path: Path
) -> None:
    """冻结首标必须原样使用 Task27 x/y/yaw，Q 只保存供后续配准。"""
    node = correction_node
    job = _job(node, tmp_path, _candidate(node))
    job.samples_accepted = 24
    job.samples_rejected = 1
    job.snapshot = QualitySnapshot(
        window_samples=24,
        inlier_samples=23,
        span_seconds=2.8,
        # 故意让 Q/yaw 的单点二维锚定值不同于 Task27 的 C_full 平移。
        x_m=0.40,
        y_m=-0.30,
        yaw_rad=math.radians(-15.0),
        tilt_rad=math.radians(7.0),
        position_std_m=0.004,
        yaw_std_rad=math.radians(0.03),
        reprojection_error_px=0.3,
        odom_match_error_ms=1.0,
        odom_time_source="arrival_history",
        odin_tag_x_m=-0.05,
        odin_tag_y_m=-0.13,
        odin_tag_position_std_m=0.001,
        effective_position_sigma_m=0.012,
        independent_blocks=12,
        max_sync_motion_error_m=0.001,
        capture_start_ns=1_000_000_000,
        capture_end_ns=3_800_000_000,
        converged=True,
        reason="候选修正已收敛",
    )

    with node._lock:
        node._freeze_candidate_locked(job)

    assert job.candidate is not None
    candidate = job.candidate
    assert candidate.x_m == job.snapshot.x_m
    assert candidate.y_m == job.snapshot.y_m
    assert candidate.yaw_rad == job.snapshot.yaw_rad
    assert candidate.registration.x_m == job.snapshot.x_m
    assert candidate.registration.y_m == job.snapshot.y_m
    assert candidate.keyframes[0].odin_x_m == job.snapshot.odin_tag_x_m
    assert candidate.keyframes[0].odin_y_m == job.snapshot.odin_tag_y_m
    assert candidate.registration.residuals_m == (0.0,)


def test_stale_service_instance_and_window_revision_are_rejected(
    correction_node,
) -> None:
    """旧面板请求不能跨服务重启或窗口提交继续写入权威状态。"""
    node = correction_node
    with node._lock:
        accepted, reason = node._validate_common_request_locked(
            "panel", 1, "stale-instance", node._window_revision
        )
        assert not accepted
        assert "service instance" in reason

        accepted, reason = node._validate_common_request_locked(
            "panel", 2, node._service_instance_id, node._window_revision + 1
        )
        assert not accepted
        assert "window revision" in reason

        accepted, reason = node._validate_common_request_locked(
            "panel", 3, node._service_instance_id, node._window_revision
        )
        assert accepted
        assert reason == ""


@pytest.mark.parametrize("has_window", [True, False])
def test_session_change_automatically_invalidates_saved_window(
    correction_node, tmp_path: Path, has_window
) -> None:
    """session 改变清空 Q；空窗也递增 CAS/释放 owner，不能跨建系迟到 FIRST。"""
    from correction_interfaces.msg import ExtnavCorrectionStatus

    node = correction_node
    candidate = _candidate(node)
    job = _job(node, tmp_path, candidate)
    if has_window:
        with node._lock:
            node._commit_new_window_locked(
                job, candidate, application_state="not_requested"
            )
    else:
        assert _owner_request(node).accepted
    before_revision = node._window_revision
    status = ExtnavCorrectionStatus()
    status.interface_version = "2.0"
    status.service_available = True
    status.odin_available = True
    status.odin_session_id = "odin-b"
    status.lever_arm_x_m, status.lever_arm_y_m, status.lever_arm_z_m = (
        node.config.fcu_reference.lever_arm_m
    )

    node._on_extnav_status(status)

    assert node._window == ()
    assert node._saved_candidate is None
    assert node._successful_calibrations == 0
    assert node._window_revision == before_revision + 1
    assert not node._automatic_owner
    if not has_window:
        assert not _owner_request(node).accepted


class _ImmediateFuture:
    """同步完成的最小 rclpy Future 替身。"""

    def __init__(self, response: Any) -> None:
        self._response = response

    def add_done_callback(self, callback) -> None:
        callback(self)

    def result(self) -> Any:
        return self._response


class _NeverFuture:
    """模拟请求已发出但 ACK 永久丢失。"""

    def add_done_callback(self, callback) -> None:
        self._callback = callback


class _FakeSetClient:
    """可注入 ACK 或 ACK 丢失后的 extnav 状态变化。"""

    def __init__(self, future: Any, on_call=None) -> None:
        self.future = future
        self.on_call = on_call
        self.requests: list[Any] = []

    def wait_for_service(self, timeout_sec: float) -> bool:
        return True

    def call_async(self, request: Any) -> Any:
        self.requests.append(request)
        if self.on_call is not None:
            self.on_call(request)
        return self.future


def test_apply_ack_and_lost_ack_status_reconciliation_are_distinct(
    correction_node, tmp_path: Path
) -> None:
    """服务 ACK 与 applied_job_id 状态对账都可确认，但结果必须标明证据来源。"""
    node = correction_node
    node.config = replace(
        node.config,
        timeouts=replace(
            node.config.timeouts,
            extnav_apply_seconds=0.01,
            extnav_reconcile_seconds=0.05,
        ),
    )
    candidate = _candidate(node)
    job = _job(node, tmp_path, candidate)
    response = SimpleNamespace(
        accepted=True,
        applied=True,
        odin_session_id="odin-a",
        revision=1,
        message="applied",
    )
    node._set_client = _FakeSetClient(_ImmediateFuture(response))

    ack = node._apply_to_extnav(job, candidate)

    assert ack.state == "applied_ack"
    assert not ack.confirmed_by_status

    node._extnav.revision = 0

    def expose_authoritative_status(request: Any) -> None:
        node._extnav.revision = 1
        node._extnav.correction_valid = True
        node._extnav.applied_job_id = request.job_id
        node._extnav.correction_x_m = request.correction_x_m
        node._extnav.correction_y_m = request.correction_y_m
        node._extnav.correction_yaw_rad = request.correction_yaw_rad
        node._extnav.received_monotonic = time.monotonic()

    node._set_client = _FakeSetClient(_NeverFuture(), expose_authoritative_status)
    status_confirmed = node._apply_to_extnav(job, candidate)

    assert status_confirmed.state == "applied_status"
    assert status_confirmed.confirmed_by_status


def test_ack_timeout_without_matching_status_locks_unknown(
    correction_node, tmp_path: Path
) -> None:
    """ACK 超时不能谎报未应用；没有权威证据时必须锁存 unknown。"""
    node = correction_node
    node.config = replace(
        node.config,
        timeouts=replace(
            node.config.timeouts,
            extnav_apply_seconds=0.01,
            extnav_reconcile_seconds=0.01,
        ),
    )
    candidate = _candidate(node)
    job = _job(node, tmp_path, candidate)
    node._set_client = _FakeSetClient(_NeverFuture())

    outcome = node._apply_to_extnav(job, candidate)

    assert outcome.state == "unknown"
    assert "无定论" in outcome.message


def test_unknown_retry_refuses_to_refresh_cas_after_third_party_override(
    correction_node,
) -> None:
    """unknown 后若 active 被其他 job 覆盖，只能保留诊断，不能自动强行写回。"""
    node = correction_node
    candidate = replace(
        _candidate(node), application_state="unknown", expected_extnav_revision=0
    )
    node._pending_candidate = candidate
    node._extnav.revision = 2
    node._extnav.correction_valid = True
    node._extnav.applied_job_id = "third-party"
    request = SimpleNamespace(
        source_id="panel",
        sequence=1,
        expected_service_instance_id=node._service_instance_id,
        expected_window_revision=0,
        expected_extnav_revision=2,
    )

    response = node._on_apply_saved(request, SimpleNamespace())

    assert not response.accepted
    assert "不能刷新 CAS" in response.message
    assert node._pending_candidate is candidate
    assert node._odom_subscription is None


def test_saved_candidate_delta_refreshes_when_active_revision_changes(
    correction_node,
) -> None:
    """同 session 的 active 更新不清 Q，但状态 delta 必须按当前 active 重新展示。"""
    from correction_interfaces.msg import CorrectionStatus

    node = correction_node
    candidate = _candidate(node, saved=True)
    node._saved_candidate = candidate
    node._extnav.correction_valid = True
    node._extnav.correction_x_m = 0.04
    node._extnav.correction_y_m = -0.05
    node._extnav.correction_yaw_rad = math.radians(-0.3)
    node._extnav.revision = 3
    node._extnav.received_monotonic = time.monotonic()
    message = CorrectionStatus()

    node._fill_candidate_status(message, candidate)

    assert math.isclose(message.delta_x_m, 0.06, abs_tol=1e-12)
    assert math.isclose(message.delta_y_m, -0.15, abs_tol=1e-12)
    assert math.isclose(message.delta_yaw_deg, 0.5, abs_tol=1e-12)
    assert not message.can_apply
    assert "delta 已按当前 active 刷新" in message.apply_reason


def test_confirmed_apply_with_window_commit_failure_is_not_unknown(
    correction_node, tmp_path: Path, monkeypatch
) -> None:
    """extnav 已确认而本地提交失败时保留旧窗，并准确区分 applied 与 unknown。"""
    from correction_service.node import _ApplicationOutcome

    from correction_interfaces.msg import CorrectionStatus

    node = correction_node
    candidate = _candidate(node)
    job = _job(node, tmp_path, candidate)
    job.apply_requested = True
    job.resources_released = True
    old_window = node._window
    monkeypatch.setattr(
        node,
        "_apply_to_extnav",
        lambda _job, _candidate: _ApplicationOutcome(
            "applied_ack", 1, "ACK applied", False
        ),
    )

    def fail_commit(*_args, **_kwargs):
        raise RuntimeError("synthetic commit conflict")

    monkeypatch.setattr(node, "_commit_new_window_locked", fail_commit)

    node._apply_and_finalize(job)

    assert job.application_state == "applied_ack"
    assert job.outcome == "applied_but_window_commit_failed"
    assert not job.window_saved
    assert node._pending_candidate is None
    assert node._window == old_window
    assert job.state == CorrectionStatus.STATE_FAILED
    assert not job.active


def test_stop_before_and_after_apply_request_preserves_authoritative_window(
    correction_node, tmp_path: Path
) -> None:
    """提交前 stop 置取消事件；请求已发出后只标记并继续对账，均不改正式窗口。"""
    node = correction_node
    candidate = _candidate(node)
    job = _job(node, tmp_path, candidate)
    node._job = job
    old_window = node._window
    request = SimpleNamespace(
        source_id="panel",
        sequence=1,
        expected_service_instance_id=node._service_instance_id,
        job_id=job.job_id,
    )

    response = node._on_stop(request, SimpleNamespace())

    assert response.accepted
    assert job.stop_event.is_set()
    assert node._window == old_window

    job.stop_event.clear()
    job.user_stop = False
    job.apply_request_sent = True
    request.sequence = 2
    response = node._on_stop(request, SimpleNamespace())

    assert response.accepted
    assert not job.stop_event.is_set()
    assert "继续等待 ACK" in response.message
    assert node._window == old_window


def test_apply_saved_confirmation_does_not_increment_n_or_open_camera(
    correction_node, tmp_path: Path, monkeypatch
) -> None:
    """保存候选的应用只更新 application 事实，不重复保存 keyframe 或增加 N。"""
    from correction_service.node import _ApplicationOutcome

    from correction_interfaces.msg import CorrectionStatus

    node = correction_node
    candidate = _candidate(node)
    first_job = _job(node, tmp_path, candidate)
    with node._lock:
        saved = node._commit_new_window_locked(
            first_job, candidate, application_state="not_requested"
        )
    count_before = node._successful_calibrations
    revision_before = node._window_revision
    apply_job = _job(node, tmp_path, saved)
    apply_job.operation = CorrectionStatus.OPERATION_APPLY_SAVED
    apply_job.apply_requested = True
    apply_job.increment_window_on_apply = False
    monkeypatch.setattr(
        node,
        "_apply_to_extnav",
        lambda _job, _candidate: _ApplicationOutcome(
            "applied_ack", 1, "ACK applied", False
        ),
    )

    node._apply_and_finalize(apply_job)

    assert apply_job.application_state == "applied_ack"
    assert node._successful_calibrations == count_before
    assert node._window_revision == revision_before
    assert node._image_subscription is None
    assert node._odom_subscription is None


def test_subscription_cleanup_failure_latches_resource_fault(
    correction_node, monkeypatch
) -> None:
    """无法销毁高负载订阅时必须锁存故障并阻止继续伪装 idle 可用。"""
    node = correction_node
    sentinel = object()
    node._image_subscription = sentinel
    # 仅在被测资源释放调用期间注入故障，避免污染 fixture 的节点销毁阶段。
    with monkeypatch.context() as scoped_patch:
        scoped_patch.setattr(node, "destroy_subscription", lambda subscription: False)
        released = node._stop_image_capture()

    assert not released
    assert node._image_subscription is None
    assert "相机图像订阅" in node._resource_fault
    accepted, reason = node._validate_common_request_locked(
        "panel", 1, node._service_instance_id, node._window_revision
    )
    assert not accepted
    assert "资源故障" in reason


def test_finished_job_elapsed_time_remains_frozen(
    correction_node, tmp_path: Path, monkeypatch
) -> None:
    """终态 status 与 result 的耗时必须冻结，不能随心跳继续增长。"""
    node = correction_node
    job = _job(node, tmp_path, _candidate(node))
    job.started_monotonic = time.monotonic() - 0.05
    job.outcome = "dry_run_saved"
    node._job = job
    results: list[Any] = []
    statuses: list[Any] = []
    monkeypatch.setattr(node._result_pub, "publish", results.append)
    monkeypatch.setattr(node._status_pub, "publish", statuses.append)

    node._finish_job(job, success=True, message="test complete")
    finished_elapsed = statuses[-1].elapsed_s
    time.sleep(0.02)
    node._publish_status()

    assert job.finished_monotonic > job.started_monotonic
    assert results[-1].duration_s == pytest.approx(finished_elapsed, abs=1e-6)
    assert statuses[-1].elapsed_s == pytest.approx(finished_elapsed, abs=1e-6)


def test_tag_config_save_is_atomic_and_invalidates_old_geometry_only(
    correction_node, tmp_path: Path
) -> None:
    """真实服务回调保存到临时档案，失败保留文件，成功换几何且不动 active。"""
    import shutil
    from correction_interfaces.srv import ConfigureTags
    node = correction_node
    root = tmp_path / "correction"
    shutil.copytree(CONFIG_DIR, root)
    shutil.copy2(CONFIG_DIR.parent / "odin_extrinsics.yaml", tmp_path / "odin_extrinsics.yaml")
    node.config = replace(node.config, config_dir=root)
    path = root / "tag_pose.csv"
    original = path.read_bytes()
    candidate = _candidate(node, saved=True)
    node._window = candidate.keyframes
    node._saved_candidate = candidate
    active_revision = node._extnav.revision
    request = ConfigureTags.Request()
    request.save = True
    request.source_id = "tag-editor-test"
    request.sequence = 1
    request.expected_service_instance_id = node._service_instance_id
    request.expected_window_revision = node._window_revision
    request.csv = "tag_id,x,y,z,yaw_deg,size_m\n2,1,2,3,90,0.2\n2,0,0,0,0,0.1\n"
    response = node._on_configure_tags(request, ConfigureTags.Response())
    assert not response.accepted
    assert path.read_bytes() == original
    assert node._saved_candidate is candidate
    request.sequence += 1
    request.csv = "tag_id,x,y,z,yaw_deg,size_m\n10,1,2,3,90,0.2\n2,-1,0,0,0,0.1\n"
    response = node._on_configure_tags(request, ConfigureTags.Response())
    assert response.accepted
    assert list(node.config.tags) == [2, 10]
    assert path.read_text() == response.csv
    assert node.config.tags[10].yaw_rad == pytest.approx(math.pi / 2)
    assert not node._window and node._saved_candidate is None
    assert node._extnav.revision == active_revision
    assert node._window_revision == request.expected_window_revision + 1
    saved_bytes = path.read_bytes()
    request.sequence += 1
    request.expected_window_revision = node._window_revision
    node._pending_candidate = candidate
    response = node._on_configure_tags(request, ConfigureTags.Response())
    assert not response.accepted
    assert path.read_bytes() == saved_bytes


def _owner_request(node: Any, *, source: str = "onboard-calibration-test",
                   acquire: bool = True, instance: str | None = None):
    """通过真实 ROS 请求类型调用 owner 入口，不启动相机或飞行控制。"""
    from correction_interfaces.srv import SetCalibrationOwner

    return node._on_set_calibration_owner(
        SetCalibrationOwner.Request(
            source_id=source, acquire=acquire,
            expected_service_instance_id=node._service_instance_id if instance is None else instance,
        ),
        SetCalibrationOwner.Response(),
    )


def test_owner_claim_renew_clear_release_keeps_extnav_and_window_rules(correction_node):
    """同源续租不改 revision，owner 清窗只丢历史，释放后恢复手动清窗权限。"""
    from correction_service.ros_client import CorrectionPanelClient

    from correction_interfaces.srv import ClearWindow

    node = correction_node
    assert not _owner_request(node, instance="old-instance").accepted
    assert not _owner_request(node, source="").accepted
    assert _owner_request(node).accepted
    original_deadline = node._owner_deadline
    assert _owner_request(node).accepted
    assert node._owner_deadline >= original_deadline
    assert node._window_revision == 0
    assert not _owner_request(node, source="other").accepted
    assert not _owner_request(node, source="other", acquire=False).accepted
    statuses = []
    node._status_pub = SimpleNamespace(publish=statuses.append)
    node._publish_status()
    projected = CorrectionPanelClient._correction_status(statuses[-1])
    assert projected["automatic_owner"] == "onboard-calibration-test"
    assert projected["configured_tag_ids"] == sorted(node.config.tags)
    assert projected["service_instance_id"] == node._service_instance_id
    assert projected["window_revision"] == 0
    assert projected["service_session"] == "odin-a"
    assert projected["window_session"] == ""
    node._window = (_candidate(node).keyframes[0],)
    active_extnav = node._extnav
    response = node._on_clear_window(ClearWindow.Request(
        source_id="onboard-calibration-test", sequence=1,
        expected_service_instance_id=node._service_instance_id,
        expected_window_revision=0,
    ), ClearWindow.Response())
    assert response.accepted and node._window_revision == 1
    assert not node._window and node._extnav is active_extnav
    assert _owner_request(node, acquire=False).accepted
    assert _owner_request(node, acquire=False).accepted
    response = node._on_clear_window(ClearWindow.Request(
        source_id="manual", sequence=1, expected_service_instance_id=node._service_instance_id,
        expected_window_revision=1,
    ), ClearWindow.Response())
    assert response.accepted and not node._automatic_owner


@pytest.mark.parametrize("service_name,handler", [
    ("StartCorrection", "_on_start"), ("StopCorrection", "_on_stop"),
    ("ClearWindow", "_on_clear_window"), ("ApplySavedCorrection", "_on_apply_saved"),
    ("ConfigureTags", "_on_configure_tags"),
])
def test_automatic_owner_rejects_every_nonowner_write(correction_node, service_name, handler):
    """所有现有写入口拒绝非 owner，且不消耗手动 sequence。"""
    from correction_interfaces import srv

    node = correction_node
    assert _owner_request(node).accepted
    service = getattr(srv, service_name)
    request = service.Request(
        source_id="manual", sequence=1, expected_service_instance_id=node._service_instance_id,
    )
    if service_name == "ConfigureTags":
        request.save = True
    response = getattr(node, handler)(request, service.Response())
    assert not response.accepted and "独占" in response.message
    assert "manual" not in node._last_sequences
    assert node._window_revision == 0


def test_owner_allows_read_and_rejects_claim_over_existing_job_or_unknown(correction_node, tmp_path):
    """自动期间配置读取仍可用；claim 不劫持手动 job 或 unknown 事务。"""
    from correction_interfaces.srv import ConfigureTags

    node = correction_node
    candidate = _candidate(node)
    job = _job(node, tmp_path, candidate)
    node._job = job
    assert not _owner_request(node).accepted
    job.active = False
    node._pending_candidate = candidate
    assert not _owner_request(node).accepted
    node._pending_candidate = None
    assert _owner_request(node).accepted
    response = node._on_configure_tags(ConfigureTags.Request(save=False), ConfigureTags.Response())
    assert response.accepted and "tag_id" in response.csv


@pytest.mark.parametrize("expire,already_sent", [(False, False), (True, False), (False, True)])
def test_release_or_lease_expiry_waits_for_full_job_thread(
    correction_node, tmp_path, expire, already_sent,
):
    """释放不等待；终态线程未退时仍独占，已发送的 apply 保持原对账语义。"""
    import threading

    node = correction_node
    assert _owner_request(node).accepted
    candidate = _candidate(node)
    job = _job(node, tmp_path, candidate)
    job.source_id = node._automatic_owner
    job.apply_request_sent = already_sent
    node._job = job
    done = threading.Event()
    job.thread = threading.Thread(target=lambda: done.wait(timeout=2.0))
    job.thread.start()
    try:
        node._window = candidate.keyframes
        node._saved_candidate = candidate
        if expire:
            node._owner_deadline = time.monotonic() - 1.0
            node._publish_status()
        else:
            assert _owner_request(node, acquire=False).accepted
        assert node._automatic_owner and node._owner_releasing
        assert job.user_stop and job.stop_event.is_set() != already_sent
        assert node._window == candidate.keyframes and node._saved_candidate is candidate
        assert not _owner_request(node).accepted
        assert not node._validate_owner_locked("manual")[0]
        job.active = False
        node._publish_status()
        assert node._automatic_owner  # 终态发布不等于线程退出。
    finally:
        job.active = False
        done.set()
        job.thread.join(timeout=2.0)
    node._publish_status()
    assert not node._automatic_owner
    assert node._validate_owner_locked("manual")[0]
    assert not node._validate_owner_locked(job.source_id)[0]
    assert not _owner_request(node).accepted


def test_release_blocks_apply_that_arrives_after_service_discovery(correction_node, tmp_path):
    """Stop 可在 wait_for_service 期间到达；发现服务后不能发送迟到的修正。"""
    node = correction_node
    assert _owner_request(node).accepted
    candidate = _candidate(node)
    job = _job(node, tmp_path, candidate)
    job.source_id = node._automatic_owner
    node._job = job
    client = _FakeSetClient(_NeverFuture())

    def release_during_discovery(timeout_sec):
        """模拟服务发现返回前自动任务取消。"""
        assert _owner_request(node, acquire=False).accepted
        return True

    client.wait_for_service = release_during_discovery
    node._set_client = client
    outcome = node._apply_to_extnav(job, candidate)
    assert outcome.state == "rejected" and not client.requests
    assert not job.apply_request_sent and not node._window
    node._save_dry_run_and_finalize(job)
    assert not job.window_saved and not node._window
    assert node._successful_calibrations == 0 and job.outcome == "stopped_before_commit"


def test_automatic_ack_requires_matching_authoritative_status(correction_node, tmp_path):
    """自动 owner 使用既有对账确认，错 job 的状态不能升级 ACK。"""
    node = correction_node
    node.config = replace(node.config, timeouts=replace(
        node.config.timeouts, extnav_apply_seconds=0.01, extnav_reconcile_seconds=0.02,
    ))
    assert _owner_request(node).accepted
    candidate = _candidate(node)
    job = _job(node, tmp_path, candidate)
    job.source_id = node._automatic_owner
    response = SimpleNamespace(accepted=True, applied=True, odin_session_id="odin-a", revision=1, message="ACK")

    def expose_status(request):
        """复制实际提交值到权威 extnav 快照。"""
        node._extnav.revision = 1
        node._extnav.correction_valid = True
        node._extnav.applied_job_id = request.job_id
        node._extnav.correction_x_m = request.correction_x_m
        node._extnav.correction_y_m = request.correction_y_m
        node._extnav.correction_yaw_rad = request.correction_yaw_rad
        node._extnav.received_monotonic = time.monotonic()

    node._set_client = _FakeSetClient(_ImmediateFuture(response), expose_status)
    outcome = node._apply_to_extnav(job, candidate)
    assert outcome.state == "applied_status" and outcome.confirmed_by_status
    node._extnav.revision = 0
    node._extnav.applied_job_id = "different-job"
    node._set_client = _FakeSetClient(_ImmediateFuture(response))
    outcome = node._apply_to_extnav(job, candidate)
    assert outcome.state == "applied_ack" and not outcome.confirmed_by_status

    # ACK 对账窗口后才到达的状态也应升级，但必须精确匹配 job 和 ACK revision。
    node._job = job
    job.active = False
    job.application_state = "applied_ack"
    job.extnav_revision = 1
    job.candidate = replace(
        candidate, application_state="applied_ack", saved=True,
        expected_extnav_revision=1, saved_window_revision=node._window_revision,
    )
    node._saved_candidate = job.candidate
    statuses = []
    node._status_pub = SimpleNamespace(publish=statuses.append)
    node._publish_status()
    assert job.application_state == "applied_ack"
    node._extnav.revision = 2
    node._extnav.applied_job_id = candidate.application_job_id
    node._publish_status()
    assert job.application_state == "applied_ack"  # 状态 revision 必须与 ACK 精确一致。
    node._extnav.revision = 1
    for flag in ("service_available", "odin_available"):
        setattr(node._extnav, flag, False)
        node._publish_status()
        assert job.application_state == "applied_ack"
        setattr(node._extnav, flag, True)
    node._publish_status()
    assert job.application_state == "applied_status" and job.application_confirmed_by_status
    assert statuses[-1].application_state == statuses[-1].APPLICATION_APPLIED_STATUS
    assert statuses[-1].application_confirmed_by_status


def test_owner_start_keeps_first_next_tag_validation_and_default_window(correction_node, monkeypatch):
    """owner 只管并发；真实 Start 入口继续使用既有窗口/Tag 检查和 apply 字段。"""
    from correction_interfaces.msg import CorrectionStatus
    from correction_interfaces.srv import ClearWindow, StartCorrection

    node = correction_node
    assert _owner_request(node).accepted
    assert ("/correction_service/set_calibration_owner", [
        "correction_interfaces/srv/SetCalibrationOwner"
    ]) in node.get_service_names_and_types()
    source = node._automatic_owner
    sequence = 0

    def start(operation, tag_id):
        """携带当前实例与 CAS revision 调用 Start，不跳过服务校验。"""
        nonlocal sequence
        sequence += 1
        return node._on_start(StartCorrection.Request(
            source_id=source, sequence=sequence, operation=operation,
            expected_tag_id=tag_id, window_size=node._window_size, apply=True,
            expected_service_instance_id=node._service_instance_id,
            expected_window_revision=node._window_revision,
            expected_extnav_revision=node._extnav.revision,
        ), StartCorrection.Response())

    assert not start(CorrectionStatus.OPERATION_NEXT, 0).accepted
    assert not start(CorrectionStatus.OPERATION_FIRST, max(node.config.tags) + 1).accepted
    node._window = _candidate(node).keyframes
    assert not start(CorrectionStatus.OPERATION_FIRST, 0).accepted
    assert "相同 Tag ID" in start(CorrectionStatus.OPERATION_NEXT, 0).message
    assert "未配置 Tag" in start(CorrectionStatus.OPERATION_NEXT, max(node.config.tags) + 1).message
    saved_window = node._window
    node._window = (replace(saved_window[0], created_monotonic=(
        time.monotonic() - node.config.window.maximum_keyframe_gap_seconds - 1.0
    )),)
    assert "过期" in start(CorrectionStatus.OPERATION_NEXT, 1).message
    node._window = (replace(saved_window[0], odin_session_id="old-session"),)
    assert "session 已失效" in start(CorrectionStatus.OPERATION_NEXT, 1).message
    node._window = saved_window
    sequence += 1
    response = node._on_clear_window(ClearWindow.Request(
        source_id=source, sequence=sequence,
        expected_service_instance_id=node._service_instance_id,
        expected_window_revision=node._window_revision,
    ), ClearWindow.Response())
    assert response.accepted

    def no_camera_capture():
        """测试只覆盖请求准入，不订阅传感器或启动相机。"""
        return None

    def finish_admission_test(job):
        """不执行估计或应用，也不将请求准入伪装成成功校准。"""
        job.active = False
        job.resources_released = True

    monkeypatch.setattr(node, "_start_image_capture", no_camera_capture)
    monkeypatch.setattr(node, "_start_odometry_capture", no_camera_capture)
    monkeypatch.setattr(node, "_run_sampling_job", finish_admission_test)
    response = start(CorrectionStatus.OPERATION_FIRST, 0)
    assert response.accepted
    node._job.thread.join(timeout=2.0)
    assert node._job.apply_requested and node._job.source_id == source
    assert node._job.window_size == node.config.window.default_size
    assert node._automatic_owner == source and not node._window

    def fail_input_creation():
        """模拟订阅创建失败，不启动摄像头。"""
        raise RuntimeError("synthetic input creation failure")

    monkeypatch.setattr(node, "_start_image_capture", fail_input_creation)
    response = start(CorrectionStatus.OPERATION_FIRST, 0)
    assert not response.accepted and "synthetic input creation failure" in response.message
    assert node._job.state == CorrectionStatus.STATE_FAILED
    assert node._job.resources_released and not node._job.active
    assert node._job.application_state == "rejected"
    assert _owner_request(node, acquire=False).accepted


@pytest.mark.parametrize("cancel", [
    "release", "expiry", "release_before_acquire", "release_while_other_owner",
])
def test_retired_owner_rejects_late_start_stop_and_renewal(correction_node, tmp_path, cancel):
    """owner 已完全消失仍不把迟到写当手动；取消可先于 acquire 到达。"""
    from correction_interfaces.srv import StartCorrection, StopCorrection

    node = correction_node
    source = "cancelled-owner"
    if cancel == "release_while_other_owner":
        assert _owner_request(node, source="blocking-owner").accepted
    elif cancel != "release_before_acquire":
        assert _owner_request(node, source=source).accepted
    if cancel == "expiry":
        node._owner_deadline = time.monotonic() - 1.0
        node._publish_status()
    elif cancel == "release_while_other_owner":
        assert not _owner_request(node, source=source, acquire=False).accepted
        assert node._automatic_owner == "blocking-owner"
        assert _owner_request(node, source="blocking-owner", acquire=False).accepted
    else:
        assert _owner_request(node, source=source, acquire=False).accepted
    assert not node._automatic_owner
    assert not _owner_request(node, source=source).accepted
    start = StartCorrection.Request(
        source_id=source, sequence=100, operation=StartCorrection.Request.OPERATION_FIRST,
        expected_tag_id=0, window_size=node.config.window.default_size, apply=True,
        expected_service_instance_id=node._service_instance_id,
        expected_window_revision=node._window_revision,
        expected_extnav_revision=node._extnav.revision,
    )
    assert not node._on_start(start, StartCorrection.Response()).accepted
    assert node._job is None and not node._window
    # 未取得下一轮 owner 的手动 job 也不会被旧来源的空 job_id Stop 误停。
    job = _job(node, tmp_path, _candidate(node))
    node._job = job
    stop = StopCorrection.Request(
        source_id=source, sequence=101, expected_service_instance_id=node._service_instance_id,
    )
    assert not node._on_stop(stop, StopCorrection.Response()).accepted
    assert job.active and not job.user_stop and not job.stop_event.is_set()
    job.active = False
    assert _owner_request(node, source="new-owner").accepted
    assert not _owner_request(node, source=source, acquire=False).accepted
    assert node._automatic_owner == "new-owner"


def test_stop_ordering_and_job_identity_do_not_cancel_new_transaction(correction_node, tmp_path):
    """Stop 先到时消费序号拦旧 Start；后续旧 job Stop 不影响新 job。"""
    from correction_interfaces.srv import StartCorrection, StopCorrection

    node = correction_node
    assert _owner_request(node).accepted
    source = node._automatic_owner
    stop = StopCorrection.Request(
        source_id=source, sequence=2, expected_service_instance_id=node._service_instance_id,
    )
    assert not node._on_stop(stop, StopCorrection.Response()).accepted
    response = node._on_start(StartCorrection.Request(
        source_id=source, sequence=1, operation=StartCorrection.Request.OPERATION_FIRST,
        expected_tag_id=0, window_size=node.config.window.default_size, apply=True,
        expected_service_instance_id=node._service_instance_id,
    ), StartCorrection.Response())
    assert not response.accepted and "乱序" in response.message
    job = _job(node, tmp_path, _candidate(node))
    job.source_id = source
    node._job = job
    stop.sequence = 3
    stop.job_id = "previous-job"
    assert not node._on_stop(stop, StopCorrection.Response()).accepted
    assert not job.user_stop and not job.stop_event.is_set()
    job.active = False


def test_status_separates_latest_job_from_service_session_and_candidate(correction_node, tmp_path):
    """旧窗口的应用状态不能投影为新 job 完成，当前 session 不由最近 job 替代。"""
    from correction_service.ros_client import CorrectionPanelClient

    node = correction_node
    previous = replace(_candidate(node, saved=True), application_state="applied_status",
                       application_confirmed_by_status=True)
    node._window = previous.keyframes
    node._saved_candidate = previous
    node._job = _job(node, tmp_path, _candidate(node))
    node._job.base_window_revision = 7
    node._job.application_state = "rejected"
    node._job.active = False
    node._extnav.odin_session_id = "new-session"
    statuses = []
    node._status_pub = SimpleNamespace(publish=statuses.append)
    node._publish_status()
    projected = CorrectionPanelClient._correction_status(statuses[-1])
    assert projected["service_session"] == "new-session"
    assert projected["window_session"] == projected["session"] == "odin-a"
    assert projected["job_source_id"] == node._job.source_id
    assert projected["base_window_revision"] == 7
    assert projected["window_revision"] == node._window_revision
    assert projected["application_state"] == "rejected"
    assert not projected["application_confirmed_by_status"] and not projected["extnav_applied"]


def test_automatic_ack_alone_fails_without_undoing_applied_correction(correction_node, tmp_path):
    """自动完成必须见权威状态；后到事实可更新，但不能恢复已失败任务。"""
    from correction_service.node import _ApplicationOutcome
    from correction_interfaces.msg import CorrectionStatus

    node = correction_node
    assert _owner_request(node).accepted
    job = _job(node, tmp_path, _candidate(node))
    job.source_id = node._automatic_owner
    job.apply_requested = True
    job.apply_request_sent = True
    node._job = job
    node._apply_to_extnav = lambda *_args: _ApplicationOutcome("applied_ack", 1, "ACK")
    results = []
    node._result_pub = SimpleNamespace(publish=results.append)
    node._apply_and_finalize(job)
    assert job.state == CorrectionStatus.STATE_FAILED
    assert job.application_state == "applied_ack" and not job.application_confirmed_by_status
    assert job.window_saved and not results[-1].success
    assert _owner_request(node, acquire=False).accepted
    node._extnav.correction_valid = True
    node._extnav.revision = 1
    node._extnav.applied_job_id = job.candidate.application_job_id
    node._extnav.correction_x_m = job.candidate.x_m
    node._extnav.correction_y_m = job.candidate.y_m
    node._extnav.correction_yaw_rad = job.candidate.yaw_rad
    node._publish_status()
    assert job.application_confirmed_by_status
    assert job.state == CorrectionStatus.STATE_FAILED and not results[-1].success
    assert node._extnav.correction_valid and node._extnav.revision == 1


@pytest.mark.parametrize("failure", ["status_unavailable", "unavailable", "discovery_error", "send_error"])
@pytest.mark.parametrize("retrying_unknown", [False, True])
def test_application_service_failure_finishes_job_and_preserves_unknown(
    correction_node, tmp_path, failure, retrying_unknown,
):
    """服务故障必须结束 job；发送异常或旧 unknown 不能伪装成确定未应用。"""
    from correction_interfaces.msg import CorrectionStatus

    node = correction_node
    assert _owner_request(node).accepted
    candidate = _candidate(node)
    job = _job(node, tmp_path, candidate)
    job.source_id = node._automatic_owner
    job.apply_requested = True
    job.retrying_unknown = retrying_unknown
    node._job = job
    node._window = candidate.keyframes
    node._saved_candidate = replace(candidate, saved=True)
    if retrying_unknown:
        node._pending_candidate = replace(candidate, application_state="unknown")
    old_window = node._window
    old_saved = node._saved_candidate
    active_extnav = node._extnav
    if failure == "status_unavailable":
        node._extnav.service_available = False
    results = []
    node._result_pub = SimpleNamespace(publish=results.append)
    client = _FakeSetClient(_NeverFuture())

    def discover(timeout_sec):
        """模拟发现失败或抛异常，不连接外部服务。"""
        if failure == "discovery_error":
            raise RuntimeError("synthetic discovery failure")
        return failure != "unavailable"

    def send(request):
        """模拟发送异常；不能保证远端从未收到请求。"""
        client.requests.append(request)
        raise RuntimeError("synthetic send failure")

    client.wait_for_service = discover
    client.call_async = send
    node._set_client = client
    node._apply_and_finalize(job)

    expected_state = "unknown" if retrying_unknown or failure == "send_error" else "rejected"
    assert not job.active and job.state == CorrectionStatus.STATE_FAILED
    assert job.application_state == expected_state
    assert not results[-1].success and job.resources_released
    assert not job.window_saved and node._window is old_window
    assert node._saved_candidate is old_saved and node._extnav is active_extnav
    assert node._successful_calibrations == 0 and node._window_revision == 0
    assert (node._pending_candidate is not None) == (expected_state == "unknown")
    assert bool(client.requests) == (failure == "send_error")
    assert _owner_request(node, acquire=False).accepted
    assert not node._automatic_owner


@pytest.mark.parametrize(
    "target_id,marker_ids,pose_state,accepted,reason",
    [
        (0, (0, 1, 9), "valid", True, ""),
        (1, (9, 0, 1), "valid", True, ""),
        (0, (1, 9), "missing", False, "但预期 Tag 0"),
        (0, (0, 9), "missing", False, "PnP 求解失败"),
        (0, (0, 9), "poor", False, "重投影 RMS"),
    ],
)
def test_sampling_uses_only_requested_tag(
    correction_node, tmp_path, monkeypatch,
    target_id, marker_ids, pose_state, accepted, reason,
) -> None:
    """其他已配置/未知ID不阻塞目标；目标缺失、PnP失败和质量差仍拒绝。"""
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import Image
    from correction_service.detector import DetectionBatch, PoseDetection
    from correction_service.estimator import CorrectionEstimator
    from correction_service.geometry import configured_tag_from_standard, world_tag_transform

    node = correction_node
    tag = node.config.tags[target_id]
    job = _job(node, tmp_path, _candidate(node))
    job.expected_tag_id = target_id
    job.estimator = CorrectionEstimator(node.config.quality, node.config.keyframe)
    odometry = Odometry()
    odometry.header.frame_id, odometry.child_frame_id = "odom", "fcu"
    odometry.header.stamp.sec = 1
    odometry.pose.pose.position.z = 1.0
    odometry.pose.pose.orientation.w = 1.0
    stamp_ns = 1_000_000_000
    node._synchronizer._capture.add(serialize_message(odometry), stamp_ns, time.monotonic())
    odin_from_fcu = np.eye(4)
    odin_from_fcu[2, 3] = 1.0
    camera_from_tag = (
        np.linalg.inv(odin_from_fcu @ node.config.t_fcu_camera)
        @ world_tag_transform(tag) @ np.linalg.inv(configured_tag_from_standard())
    )
    detection = PoseDetection(
        target_id, camera_from_tag,
        node.config.detection.max_reprojection_error_px + 1.0 if pose_state == "poor" else 0.1,
        np.zeros((4, 2)),
    )
    seen_tags = {}

    def detect(gray, tags):
        """仅替代像素解码；后续真实同步、几何与样本入库路径照常执行。"""
        seen_tags.update(tags)
        return DetectionBatch(marker_ids, () if pose_state == "missing" else (detection,), 1.0)

    monkeypatch.setattr(node, "_detector", SimpleNamespace(detect=detect))
    width = round(node.config.camera.width * node.config.detection.image_scale)
    height = round(node.config.camera.height * node.config.detection.image_scale)
    image = Image(
        height=height, width=width, encoding="mono8", step=width,
        data=bytes(width * height),
    )
    node._process_image(job, image, stamp_ns)
    assert job.samples_accepted == int(accepted)
    assert job.samples_rejected == int(not accepted)
    assert seen_tags == {target_id: tag}
    if accepted:
        assert job.detected_tag_id == target_id
    else:
        assert reason in job.message
