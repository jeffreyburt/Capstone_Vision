"""tools/floor_from_markers.py: recover a known camera pose from markers lying flat on the floor."""

import importlib.util
from pathlib import Path

import numpy as np

from vision.geometry import CameraModel, FloorPose, Geometry

_spec = importlib.util.spec_from_file_location(
    "floor_from_markers", Path(__file__).resolve().parent.parent / "tools" / "floor_from_markers.py")
ffm = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(ffm)


def marker_corners_px(geom, center, yaw_deg, size):
    """Pixel corners (ArUco order) of a flat marker on the floor at center (x, y), rotated by yaw."""
    s, a = size / 2, np.radians(yaw_deg)
    local = np.array([[-s, s], [s, s], [s, -s], [-s, -s]])  # marker frame: x right, y "up" the marker
    R = np.array([[np.cos(a), -np.sin(a)], [np.sin(a), np.cos(a)]])
    world = (local @ R.T) + center
    return geom.world_to_pixels(np.column_stack([world, np.zeros(4)]))


def test_recovers_height_tilt_and_marker_positions():
    camera = CameraModel.from_fov(4608, 2592, 100)
    tilt = np.radians(8.4)
    true_pose = FloorPose.look_at([0, 0, 1.02], [0, np.cos(tilt), 1.02 - np.sin(tilt)])
    geom = Geometry(camera, true_pose)
    corners = {0: marker_corners_px(geom, (0.0, 2.5), 10, 0.15), 1: marker_corners_px(geom, (-0.92, 2.32), -25, 0.15)}

    pose, rep = ffm.floor_pose_from_markers(camera, corners, 0.15, forward_id=0)
    assert abs(rep["height_m"] - 1.02) < 0.005
    assert abs(rep["pitch_deg"] - 8.4) < 0.1
    assert abs(rep["markers"][0]["x"]) < 1e-6 and abs(rep["markers"][0]["y"] - 2.5) < 0.01
    assert abs(rep["markers"][1]["x"] + 0.92) < 0.01 and abs(rep["markers"][1]["y"] - 2.32) < 0.01
    np.testing.assert_allclose(pose.camera_position, [0, 0, 1.02], atol=0.005)
    # The recovered pose maps pixels back to the right floor points.
    est = Geometry(camera, pose)
    px = geom.world_to_pixels([[0.5, 3.5, 0.0]])[0]
    np.testing.assert_allclose(est.pixel_to_floor(*px), [0.5, 3.5], atol=0.02)
