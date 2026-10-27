import numpy as np
import pytest

from vision.geometry import CameraModel, FloorPose, Geometry, solve_floor_pose


@pytest.fixture
def geom():
    cam = CameraModel.from_fov(1920, 1080, 66)
    pose = FloorPose.look_at([0.0, 0.0, 2.0], [0.0, 4.0, 0.3])  # 2 m high, looking slightly down
    return Geometry(cam, pose)


def test_camera_position(geom):
    assert np.allclose(geom.camera_position, [0, 0, 2.0])


def test_pixel_floor_round_trip(geom):
    world = np.array([[x, y, z] for x in (-2, 0, 1.5) for y in (2, 4, 6.1) for z in (0.0, 0.3)])
    px = geom.world_to_pixels(world)
    back = geom.pixels_to_floor(px, world[:, 2])
    assert np.allclose(back, world[:, :2], atol=1e-6)


def test_pixel_above_horizon_is_nan(geom):
    assert geom.pixel_to_floor(960, -400, 0.0) is None  # a ray pointing above the horizon


def test_solve_floor_pose_recovers_camera(geom):
    floor = np.array([[-1, 2], [1, 2], [1.5, 5], [-1.5, 5], [0, 3.5], [0.5, 6]])
    px = geom.world_to_pixels(np.column_stack([floor, np.zeros(len(floor))]))
    px += np.random.default_rng(1).normal(0, 0.5, px.shape)  # half-pixel click error
    pose, rms = solve_floor_pose(geom.camera, px, floor)
    assert rms < 1.5
    assert np.allclose(pose.camera_position, [0, 0, 2.0], atol=0.05)


def test_scaled_intrinsics():
    cam = CameraModel.from_fov(1920, 1080, 66)
    small = cam.scaled_to(1280, 720)
    assert np.allclose(small.K, CameraModel.from_fov(1280, 720, 66).K)
