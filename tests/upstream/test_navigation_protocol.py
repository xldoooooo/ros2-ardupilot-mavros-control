"""Map-coordinate commands and status projection use the upstream wire contract."""
import math

import pytest

from ground_station_core.event_log import EventLog
from ground_station_core.models import VehicleSnapshot
from ground_station_core.upstream.mapping import UpstreamProtocolError, parse_command
from ground_station_core.upstream.status_projector import UpstreamStatusProjector
from ground_station_core.waypoint_io import Waypoint


def test_map_websocket_accepts_negative_z_and_reports_task_position() -> None:
    """Map Z may be negative, while hangar membership remains FCU-local."""
    payload = {
        "clientNo": "test", "commandNo": "02", "taskPoints": [{
            "index": 1, "x": 3, "y": 4, "z": -2,
            "forwardAngle": 90, "cameraAngle": 0, "photoNo": "one",
        }],
    }
    point = parse_command(payload, "test", "map").waypoints[0]
    assert tuple(point) == pytest.approx(tuple(Waypoint(3, 4, -2, math.pi/2, is_inspection=True)))
    with pytest.raises(UpstreamProtocolError, match="z"):
        parse_command(payload, "test", "odom")
    sent = []
    projector = UpstreamStatusProjector(lambda: "test", lambda message: sent.append(message) or True, EventLog())
    snapshot = VehicleSnapshot(
        local_position_valid=True, navigation_frame="map", task_position=(3, 4, -2),
    )
    assert projector.is_in_hangar(snapshot)
    projector.observe_vehicle(snapshot, "hardware", now=5)
    assert any(item.get("data") == {"X": 3, "Y": 4, "Z": -2} for item in sent)
