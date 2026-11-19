import json
from unittest.mock import patch

import numpy as np
import pytest

from robot.drive_server import parse_command
from vision.geometry import CameraModel, FloorPose, Geometry
from vision.joint_pose import JointFeedback
from vision.markers import MarkerDetection
from vision.robots import estimate_robots, robot_specs_from_config, wrap_deg


@pytest.fixture
def articulated():
    return robot_specs_from_config({"robots": [{
        "id": 1, "faces": {1: 90, 2: 180, 3: 270, 0: 0},
        "head_pose": {
            "neutral_units": [500, 500], "degrees_per_unit": [1, -1],
            "pan_pivot_m": [0.1, 0, 0.3], "tilt_pivot_m": [0.1, 0, 0.4],
            "markers": {
                1: {"mount": "pan", "center_m": [0.1, 0.15, 0.3]},
                2: {"mount": "tilt", "center_m": [-0.1, 0, 0.4]},
                3: {"mount": "pan", "center_m": [0.1, -0.15, 0.3]},
            },
        },
    }]})


@pytest.mark.parametrize("marker_id, corners", [
    (1, [[-.05, .05, .35], [-.05, -.05, .35], [-.05, -.05, .25], [-.05, .05, .25]]),
    (3, [[.25, -.05, .35], [.25, .05, .35], [.25, .05, .25], [.25, -.05, .25]]),
    (2, [[.05, .05, .60], [.15, .05, .60], [.15, -.05, .60], [.05, -.05, .60]]),
])
def test_recovers_chassis_from_each_articulated_marker(articulated, marker_id, corners):
    # Independently calculated corners for +90° pan and +90° tilt.
    geometry = Geometry(CameraModel.from_fov(1920, 1080, 66), FloorPose.look_at([0, 0, 2], [0, 4, .3]))
    angle = np.radians(35)
    rotation = np.array([[np.cos(angle), np.sin(angle)], [-np.sin(angle), np.cos(angle)]])
    corners = np.array(corners)
    corners[:, :2] = corners[:, :2] @ rotation + [.3, 4]
    detect = MarkerDetection(marker_id, geometry.world_to_pixels(corners))
    assert estimate_robots([detect], articulated, geometry) == []
    obs, = estimate_robots([detect], articulated, geometry, {1: np.array([590, 410])})
    assert (obs.x, obs.y) == pytest.approx((.3, 4), abs=1e-5)
    assert abs(wrap_deg(obs.heading_deg - 35)) < 1e-4


def test_feedback_requires_fresh_samples_from_configured_address():
    with patch("vision.joint_pose.socket.socket") as socket, patch("vision.joint_pose.time.time", return_value=100):
        def packet(t, source):
            return json.dumps({"robot_id": 1, "t": t, "units": [450, 520]}).encode(), (source, 10000)

        socket.return_value.recvfrom.side_effect = [
            packet(99.97, "192.168.50.12"), packet(99.5, "192.168.50.11"),
            packet(99.94, "192.168.50.11"), BlockingIOError, BlockingIOError,
        ]
        feedback = JointFeedback(5006, {1: "192.168.50.11"})
        assert list(feedback.positions_at(100)[1]) == [450, 520]
        assert feedback.positions_at(100.2) == {}
        feedback.close()


def test_compensated_drive_requires_marker_pose_and_accepts_stop():
    drive = {"type": "drive", "robot_id": 1, "vx": .1, "vy": 0, "wz": 0}
    with pytest.raises(ValueError, match="compensated pose"):
        parse_command(json.dumps(drive), 1, require_compensated_pose=True)
    drive["pose_compensated"] = True
    assert parse_command(json.dumps(drive), 1, require_compensated_pose=True)[0] == "drive"
    assert parse_command('{"type":"stop","robot_id":1}', 1, require_compensated_pose=True)[0] == "stop"
