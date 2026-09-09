"""correction_service 2.0 的窗口 CAS、clear、session 与应用对账事务测试。"""

from __future__ import annotations

import math
import time
from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace
from typing import Any

import numpy as np
import pytest
from correction_service.journal import JobJournal
from correction_service.window import (
    CalibrationKeyframe,
    RegistrationResult,
    WindowCandidate,
)

PROJECT_ROOT = Path(__file__).resolve().parent.parent
CONFIG_DIR = PROJECT_ROOT / "correction_service" / "config"


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
def correction_node(monkeypatch):
    """在 localhost-only 独立 domain 创建不启动相机的真实节点实例。"""
    rclpy = pytest.importorskip("rclpy")
    from correction_service.node import CorrectionServiceNode, _ExtnavSnapshot
    from rclpy.context import Context
    from rclpy.parameter import Parameter

    monkeypatch.setenv("ROS_AUTOMATIC_DISCOVERY_RANGE", "LOCALHOST")
    context = Context()
    rclpy.init(args=[], context=context, domain_id=226)
    node = CorrectionServiceNode(
        context=context,
        parameter_overrides=[Parameter("config_dir", value=str(CONFIG_DIR))],
    )
    node._extnav = _ExtnavSnapshot(
        received_monotonic=time.monotonic(),
        interface_version="2.0",
        service_available=True,
        odin_available=True,
        odin_session_id="odin-a",
        correction_valid=False,
        revision=0,
        lever_arm_m=np.array((0.06, -0.03, 0.05)),
        installation_rpy=np.zeros(3),
    )
    yield node
    node.close()
    node.destroy_node()
    context.shutdown()


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


def test_session_change_automatically_invalidates_saved_window(
    correction_node, tmp_path: Path
) -> None:
    """extnav 权威 session 改变会清空原始 Q，不能跨建系拼接。"""
    from correction_interfaces.msg import ExtnavCorrectionStatus

    node = correction_node
    candidate = _candidate(node)
    job = _job(node, tmp_path, candidate)
    with node._lock:
        node._commit_new_window_locked(
            job, candidate, application_state="not_requested"
        )
    before_revision = node._window_revision
    status = ExtnavCorrectionStatus()
    status.interface_version = "2.0"
    status.service_available = True
    status.odin_available = True
    status.odin_session_id = "odin-b"
    status.lever_arm_x_m = 0.06
    status.lever_arm_y_m = -0.03
    status.lever_arm_z_m = 0.05

    node._on_extnav_status(status)

    assert node._window == ()
    assert node._saved_candidate is None
    assert node._successful_calibrations == 0
    assert node._window_revision == before_revision + 1


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
