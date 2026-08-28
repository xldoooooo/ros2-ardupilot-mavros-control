"""correction_service 轮转服务日志和逐任务 JSONL 审计记录。"""

from __future__ import annotations

import json
import logging
import math
import threading
from datetime import datetime, timezone
from logging.handlers import RotatingFileHandler
from pathlib import Path
from typing import Any

from .config import LoggingSettings


def _json_safe(value: Any) -> Any:
    """递归把非有限浮点投影为 null，保证每行都是严格 JSON。"""
    if isinstance(value, float):
        return value if math.isfinite(value) else None
    if isinstance(value, dict):
        return {str(key): _json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_safe(item) for item in value]
    return value


def configure_service_logger(settings: LoggingSettings) -> logging.Logger:
    """建立模块私有轮转日志，重复构造节点时不叠加 handler。"""
    settings.directory.mkdir(parents=True, exist_ok=True)
    logger = logging.getLogger("correction_service")
    logger.setLevel(logging.INFO)
    logger.propagate = False
    for handler in tuple(logger.handlers):
        handler.close()
        logger.removeHandler(handler)
    handler = RotatingFileHandler(
        settings.directory / "correction_service.log",
        maxBytes=settings.service_log_max_bytes,
        backupCount=settings.service_log_backups,
        encoding="utf-8",
    )
    handler.setFormatter(
        logging.Formatter("%(asctime)s %(levelname)s %(threadName)s %(message)s")
    )
    logger.addHandler(handler)
    return logger


class JobJournal:
    """每行一个结构化事件，保留 Tag、候选、质量、ACK 和错误。"""

    def __init__(self, directory: Path, job_id: str) -> None:
        timestamp = datetime.now().astimezone().strftime("%Y%m%d-%H%M%S")
        self.path = directory / f"job-{timestamp}-{job_id}.jsonl"
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._lock = threading.Lock()

    def write(self, event: str, **fields: Any) -> None:
        """原子追加一个 UTF-8 JSON 对象并立即刷盘。"""
        record = {
            "timestamp": datetime.now(timezone.utc).astimezone().isoformat(),
            "event": event,
            **fields,
        }
        encoded = json.dumps(
            _json_safe(record),
            ensure_ascii=False,
            sort_keys=True,
            allow_nan=False,
        )
        with self._lock, self.path.open("a", encoding="utf-8") as stream:
            stream.write(encoded + "\n")
            stream.flush()
