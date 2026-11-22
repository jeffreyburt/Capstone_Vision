"""Approximate positions must be labeled and must never produce wheel commands."""

import math

import numpy as np
import pytest

from control.controller import RobotController
from control.main import compute_commands
from control.planner import FormationPlanner
from vision.geometry import CameraModel, FloorPose, Geometry, load_geometry
from vision.markers import MarkerDetection
from vision.robots import estimate_marker_positions, robot_specs_from_config
from vision.pipeline import VisionPipeline
from vision.people import PersonDetection


def test_approximate_geometry_uses_owner_height_without_claiming_calibration(tmp_path):
    cfg = {"camera": {"hfov_deg": 102}, "calibration": {"floor_pose": str(tmp_path / "pending.yaml"),
           "approximate": {"enabled": True, "height_m": .395, "tilt_deg": 10}}}
    geometry = load_geometry(cfg, (1600, 900))
    assert geometry.camera_position == pytest.approx([0, 0, .395])
    assert geometry.approximation["driving_allowed"] is False
    point = [.4, 3, 0]
    pixel = geometry.world_to_pixels([point])[0]
    assert geometry.pixel_to_floor(*pixel) == pytest.approx(point[:2])
    pose = FloorPose.look_at([0, 0, 1.2], [0, 3, 0])
    pose.save(tmp_path / "pending.yaml")
    assert load_geometry(cfg, (1600, 900)).approximation is None  # measured pose wins


@pytest.mark.parametrize("tilt", [0, 35, 70])
def test_marker_position_does_not_require_an_upright_speaker(tilt):
    a = math.radians(10)
    geometry = Geometry(CameraModel.from_fov(1600, 900, 102),
                        FloorPose.look_at([0, 0, .395], [0, math.cos(a), .395 - math.sin(a)]))
    geometry.approximation = {"height_m": .395}
    s = .05
    square = np.array([[-s, s, 0], [s, s, 0], [s, -s, 0], [-s, -s, 0]])
    a = math.radians(tilt)
    rotation = np.array([[1, 0, 0], [0, math.cos(a), -math.sin(a)], [0, math.sin(a), math.cos(a)]])
    upright = np.array([[1, 0, 0], [0, 0, -1], [0, 1, 0]])
    center = np.array([.5, 2.5, .4])
    corners = geometry.world_to_pixels(square @ upright.T @ rotation.T + center)
    marker = MarkerDetection(6, corners)
    specs = robot_specs_from_config({"markers": {"size_m": .1}, "robots": [{"id": 2, "faces": {6: 180}}]})
    result = estimate_marker_positions([marker], specs, geometry)
    assert len(result) == 1
    assert [result[0].x, result[0].y] == pytest.approx(center[:2], abs=.01)
    assert result[0].heading_deg is None


def test_approximate_positions_cannot_drive_even_with_fresh_targets():
    msg = {"position_quality": "approximate", "driving_allowed": False,
           "robots": [{"id": 2, "x": 0, "y": 1, "heading": 90, "age": 0}],
           "user": {"x": 0, "y": 4, "age": 0}}
    targets, commands = compute_commands(msg, 0, FormationPlanner({2: "right"}, distance_m=2),
                                         {2: RobotController()})
    assert targets == {}
    assert commands[2][0] == (0, 0, 0)
    assert "driving disabled" in commands[2][1]


@pytest.mark.parametrize("feet_visible", [True, False])
def test_pipeline_reports_person_only_when_feet_can_be_projected(feet_visible):
    a = math.radians(10)
    geometry = Geometry(CameraModel.from_fov(1600, 900, 102),
                        FloorPose.look_at([0, 0, .395], [0, math.cos(a), .395 - math.sin(a)]))
    geometry.approximation = {"height_m": .395, "tilt_deg": 10, "driving_allowed": False}
    head, foot = geometry.world_to_pixels([[.2, 3, 1.7], [.2, 3, 0]])

    class Detector:
        def detect(self, frame):
            return [PersonDetection((foot[0] - 25, head[1], foot[0] + 25,
                                     foot[1] if feet_visible else 900), .9)]

    pipeline = VisionPipeline({"people": {"async": False}}, geometry=geometry, person_detector=Detector())
    try:
        pipeline.process(np.zeros((900, 1600, 3), np.uint8), 100)  # a new user is confirmed on the 2nd detection
        result = pipeline.process(np.zeros((900, 1600, 3), np.uint8), 100.1)
        assert result.message["position_quality"] == "approximate"
        assert result.message["driving_allowed"] is False
        if feet_visible:
            assert [result.message["user"]["x"], result.message["user"]["y"]] == pytest.approx([.2, 3], abs=.01)
        else:
            assert result.message["user"] is None
    finally:
        pipeline.close()
