"""隔离摄像头面板的后台HTTP轮询，真实本地HTTP测试显式启用。"""

import pytest


@pytest.fixture(autouse=True)
def isolate_camera_media_polling(request, monkeypatch):
    """普通面板测试只验证UI；本地HTTP用例保留实际后台请求。"""
    if request.node.get_closest_marker("real_media_http"):
        return
    from video_service.camera_app.media_status import GroundMediaStatus

    monkeypatch.setattr(GroundMediaStatus, "refresh", lambda self: None)
