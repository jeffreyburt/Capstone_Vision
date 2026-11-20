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


def test_collect_rejects_duplicate_ids_instead_of_averaging_two_physical_tags():
    import pytest
    from vision.markers import MarkerDetection
    class Source:
        def read(self):
            return np.zeros((100, 100, 3), dtype=np.uint8), 0
    class Detector:
        def detect(self, frame):
            return [MarkerDetection(0, np.array([[1,1],[5,1],[5,5],[1,5]],float)),
                    MarkerDetection(0, np.array([[20,60],[30,60],[30,70],[20,70]],float))]
    with pytest.raises(SystemExit, match='duplicate marker ID 0'):
        ffm.collect_corners(Source(), Detector(), [0], 1)
    corners, size = ffm.collect_corners(Source(), Detector(), [0], 1, min_y_fraction=.5)
    assert size == (100,100)
    np.testing.assert_allclose(corners[0], [[20,60],[30,60],[30,70],[20,70]])


def test_floor_reference_ids_stay_visible_but_never_become_robot_observations(monkeypatch):
    from vision.pipeline import VisionPipeline
    from vision.markers import MarkerDetection
    import vision.pipeline as module
    cfg = {'robots':[{'id':1, 'faces':{0:0, 1:90, 3:270}}],
           'calibration':{'floor_marker_ids':[0,1]}}
    camera = CameraModel.from_fov(640,480,100)
    geometry = Geometry(camera,FloorPose.look_at([0,0,.395],[0,2,0]))
    markers = [MarkerDetection(i,np.zeros((4,2))) for i in (0,0,1,3)]
    seen = []
    def estimate(markers, *args):
        seen.extend(m.id for m in markers)
        return []
    monkeypatch.setattr(module,'estimate_robots',estimate)
    pipeline = VisionPipeline(cfg,geometry=geometry)
    pipeline.detector.detect = lambda frame: markers
    try:
        result = pipeline.process(np.zeros((480,640,3),np.uint8),100)
        assert seen == [3]
        assert len(result.markers) == 4  # floor markers remain visible in camera/debug overlays
        assert result.message['robots'] == []
    finally:
        pipeline.close()


def test_measured_floor_does_not_claim_a_chassis_heading_for_uncalibrated_moving_tags(monkeypatch):
    from vision.pipeline import VisionPipeline
    from vision.robots import RobotObservation
    import vision.pipeline as module
    camera = CameraModel.from_fov(640,480,100)
    geometry = Geometry(camera,FloorPose.look_at([0,0,.395],[0,2,0]))
    monkeypatch.setattr(module,'estimate_robots',lambda *args: [RobotObservation(1,0,2,90,[3])])
    pipeline = VisionPipeline({'robots':[{'id':1,'faces':{3:270},'moving_head_markers':True}]},geometry=geometry)
    try:
        result=pipeline.process(np.zeros((480,640,3),np.uint8),100)
        robot=result.message['robots'][0]
        assert robot['heading'] is None
        assert robot['pose_compensated'] is False
        assert robot['reference']=='uncalibrated_head_markers'
    finally:
        pipeline.close()
