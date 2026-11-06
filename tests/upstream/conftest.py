"""隔离上位机会话附带的媒体后台，避免读取或修改现场通知队列。"""

import pytest

from ground_station_core.ground_media_reader import GroundMediaReader
from ground_station_core.upstream import service


@pytest.fixture(autouse=True)
def isolate_upstream_media_reader(tmp_path, monkeypatch):
    """会话测试不启动媒体轮询；独立reader测试保留真实线程实现。"""
    config = tmp_path / "isolated-media.json"
    config.write_text('{"api_url":"http://media.invalid"}', encoding="utf-8")
    monkeypatch.setenv("GROUND_MEDIA_CONFIG", str(config))

    def session_reader(*args, **kwargs):
        """仅替换会话持有实例的启动入口，保留失败通知内存排队。"""
        reader = GroundMediaReader(*args, **kwargs)
        monkeypatch.setattr(reader, "start", lambda: None)
        return reader

    monkeypatch.setattr(service, "GroundMediaReader", session_reader)
