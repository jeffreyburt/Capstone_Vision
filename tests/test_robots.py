import numpy as np
import pytest

from vision.config import load_config
from vision.geometry import CameraModel, FloorPose, Geometry
from vision.markers import MarkerDetection, MarkerDetector
from vision.pipeline import VisionPipeline
from vision.robots import estimate_robots, robot_specs_from_config, wrap_deg
from vision.sim import face_corners_world, render_scene


@pytest.fixture
def cfg():
    return load_config()


@pytest.fixture
def specs(cfg):
    return robot_specs_from_config(cfg)


@pytest.fixture
def geom():
    return Geometry(CameraModel.from_fov(1920, 1080, 66), FloorPose.look_at([0, 0, 2.0], [0, 4.0, 0.3]))


def fake_detections(geom, specs, robots):
    """Exact marker corner pixels for the camera-facing faces (no image rendering)."""
    out = []
    by_id = {s.id: s for s in specs}
    for rid, x, y, hd in robots:
        s = by_id[rid]
        for mid, fdeg in s.faces.items():
            corners, fc, n = face_corners_world(x, y, hd, fdeg, s.center_offset_m, s.marker_size_m, s.marker_center_height_m)
            if np.dot(n, geom.camera_position[:2] - fc) > 0.2 * np.linalg.norm(geom.camera_position[:2] - fc):
                out.append(MarkerDetection(mid, geom.world_to_pixels(corners)))
    return out


def test_wrap_deg():
    assert wrap_deg(190) == -170
    assert wrap_deg(-180) == 180
    assert wrap_deg(360) == 0


@pytest.mark.parametrize("heading", [-150, -90, -30, 0, 45, 90, 135, 180])
def test_exact_pose_from_corners(geom, specs, heading):
    robots = [(1, -1.0, 4.0, heading), (2, 1.2, 6.0, heading + 20)]
    obs = {o.robot_id: o for o in estimate_robots(fake_detections(geom, specs, robots), specs, geom)}
    for rid, x, y, hd in robots:
        assert obs[rid].x == pytest.approx(x, abs=1e-3)
        assert obs[rid].y == pytest.approx(y, abs=1e-3)
        assert abs(wrap_deg(obs[rid].heading_deg - hd)) < 0.1


def test_false_size_marker_rejected(geom, specs):
    # A marker-like blob 3x too big for its supposed size is rejected.
    det = fake_detections(geom, specs, [(1, 0.0, 4.0, -90)])[0]
    c = det.center
    det.corners = (det.corners - c) * 3 + c
    assert estimate_robots([det], specs, geom) == []


@pytest.mark.parametrize("dist", [2.5, 4.0, 6.1])
def test_rendered_images_end_to_end(cfg, specs, geom, dist):
    """Render robots at up to 20 ft (6.1 m), detect markers in the image, and check the positions."""
    robots = [(1, -0.8, dist, -60.0), (2, 0.9, dist, -120.0)]
    img = render_scene(geom, robots, specs, (1920, 1080), blur=0.7, noise=3)
    pipe = VisionPipeline(cfg, geometry=geom)
    res = pipe.process(img, 0.0)
    obs = {o.robot_id: o for o in res.observations}
    assert set(obs) == {1, 2}
    for rid, x, y, hd in robots:
        assert np.hypot(obs[rid].x - x, obs[rid].y - y) < 0.10  # within 10 cm
        assert abs(wrap_deg(obs[rid].heading_deg - hd)) < 15


def test_unknown_ids_ignored(geom, specs):
    img = render_scene(geom, [(1, 0, 3, -90)], specs, (1280, 720))
    assert MarkerDetector("DICT_4X4_50", allowed_ids=[40]).detect(img) == []
