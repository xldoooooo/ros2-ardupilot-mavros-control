"""甲方 WebSocket JAR 对接演示的独立 Python 包。"""

from .client import OurGroundStationClient
from .protocol import COMMAND_LABELS, STATUS_LABELS, ProtocolError

__all__ = [
    "COMMAND_LABELS",
    "OurGroundStationClient",
    "ProtocolError",
    "STATUS_LABELS",
]

