import time

import numpy as np
import pytest

from vision.config import load_config
from vision.geometry import CameraModel, FloorPose, Geometry
from vision.people import AsyncDetector, PersonDetection, locate_person
from vision.pipeline import VisionPipeline
from vision.tracking import UserTracker

W, H = 1920, 1080


@pytest.fixture
def geom():
    return Geometry(CameraModel.from_fov(W, H, 66), FloorPose.look_at([0, 0, 2.0], [0, 4.5, 0.3]))


def person_box(geom, x, y, height=1.75, half_width=0.25):
    """Bounding box a standing person at floor (x, y) would have (feet at z=0, head at z=height)."""
    pts = geom.world_to_pixels([[x - half_width, y, 0], [x + half_width, y, 0], [x, y, height]])
    return (float(pts[0, 0]), float(pts[2, 1]), float(pts[1, 0]), float(pts[0, 1]))


def test_height_above(geom):
    u, v = geom.world_to_pixels([[0.7, 5.0, 1.75]])[0]
    assert geom.height_above(u, v, 0.7, 5.0) == pytest.approx(1.75, abs=1e-6)


@pytest.mark.parametrize("x,y", [(0.0, 3.0), (-1.5, 4.5), (1.2, 6.1)])
def test_locate_person_from_feet(geom, x, y):
    obs = locate_person(PersonDetection(person_box(geom, x, y), 0.9), geom, H)
    assert obs.from_feet
    assert (obs.x, obs.y) == pytest.approx((x, y), abs=0.02)
    assert obs.height == pytest.approx(1.75, abs=0.03)


def test_locate_person_feet_cut_off_uses_head(geom):
    x1, y1, x2, _ = person_box(geom, 0.3, 4.0, height=1.70)
    obs = locate_person(PersonDetection((x1, y1, x2, H - 1), 0.9), geom, H, head_height_m=1.70)
    assert not obs.from_feet
    assert (obs.x, obs.y) == pytest.approx((0.3, 4.0), abs=0.05)


class Obs:
    def __init__(self, x, y, c=0.9, height=1.7):
        self.x, self.y, self.confidence, self.height = x, y, c, height


AREA = {"x": [-2.0, 2.0], "y": [0.3, 7.0]}


def tracker():
    return UserTracker({"lost_after_s": 1.0, "max_jump_m": 1.0, "forget_after_s": 3.0, "confirm_frames": 2}, AREA)


def test_user_tracker_follows_same_person():
    ut = tracker()
    ut.update(0.0, [Obs(0.0, 4.0, 0.6), Obs(1.5, 5.0, 0.95)])
    assert ut.state(0.0) is None                      # a new user must be confirmed
    ut.update(0.1, [Obs(0.0, 4.0, 0.6), Obs(1.5, 5.0, 0.95)])
    assert ut.state(0.1)["x"] == pytest.approx(0.0)   # nearest to the camera, not the most confident
    for i in range(2, 10):  # a second person appears more confidently; keep following the first
        ut.update(i * 0.1, [Obs(0.01 * i, 4.0, 0.7), Obs(-1.0, 3.0, 0.99)])
    assert ut.state(0.9)["x"] == pytest.approx(0.09, abs=0.05)
    assert ut.state(10.0) is None  # forgotten


def test_background_people_never_become_the_user():
    ut = tracker()
    for i in range(10):  # the 2026-11-22 cases: someone 9 m back, someone off to the side of the hallway
        ut.update(i * 0.2, [Obs(1.2, 9.0, 0.95), Obs(-4.5, 4.3, 0.9)])
    assert ut.state(2.0) is None


def test_lost_user_is_reacquired_near_their_last_position_not_by_someone_else():
    ut = tracker()
    for i in range(5):
        ut.update(i * 0.1, [Obs(0.3, 2.4)])
    # The user is hidden for 2 s while a bystander stands 1.8 m away: not taken as the user.
    for i in range(1, 11):
        ut.update(0.4 + 0.2 * i, [Obs(-1.5, 2.6)])
    assert ut.state(2.4)["x"] == pytest.approx(0.3, abs=0.05)
    ut.update(2.6, [Obs(-1.5, 2.6), Obs(0.4, 2.5)])  # the user reappears near where they were
    assert ut.state(2.6)["x"] == pytest.approx(0.4, abs=0.05)


def test_height_learned_only_when_the_head_is_in_view():
    ut = tracker()
    for i in range(6):
        ut.update(i * 0.1, [Obs(0.0, 2.4, height=None if i % 2 else 1.72)])
    assert ut.head_height(1.70) == pytest.approx(1.72)


def test_feet_behind_a_robot_use_the_head_at_the_user_height():
    geom = Geometry(CameraModel.from_fov(1600, 900, 102), FloorPose.look_at([0, 0, .33], [0, 4, .33]))
    x1, y1, x2, y2 = person_box(geom, 0.2, 2.5, height=1.72)
    # Legs hidden below a robot's top: the box ends at the robot, which on the floor would read as ~1 m farther.
    robot = [{"x": 0.2, "y": 1.0}]
    from vision.people import robot_image_boxes
    boxes = robot_image_boxes(geom, robot)
    cut = PersonDetection((x1, y1, x2, float(boxes[0][:, 1].mean())), 0.9)  # feet hidden: box ends inside the chassis
    naive = locate_person(cut, geom, 900)
    assert naive.from_feet and abs(naive.y - 2.5) > 0.5  # read as standing at the robot
    obs = locate_person(cut, geom, 900, head_height_m=1.72, occluders=boxes)
    assert not obs.from_feet and (obs.x, obs.y) == pytest.approx((0.2, 2.5), abs=0.03)


class FakeDetector:
    def __init__(self, box):
        self.box, self.calls = box, 0

    def detect(self, frame):
        self.calls += 1
        return [PersonDetection(self.box, 0.88)]


@pytest.mark.parametrize("use_async", [False, True])
def test_pipeline_reports_user(geom, use_async):
    cfg = load_config()
    cfg["people"] = dict(cfg["people"], enabled=True, **{"async": use_async})
    det = FakeDetector(person_box(geom, 0.5, 4.2))
    pipe = VisionPipeline(cfg, geometry=geom, person_detector=det)
    frame = np.zeros((H, W, 3), np.uint8)
    msg = None
    for i in range(20):
        msg = pipe.process(frame, i * 0.05).message
        if use_async:
            time.sleep(0.01)
    pipe.close()
    u = msg["user"]
    assert u is not None
    assert (u["x"], u["y"]) == pytest.approx((0.5, 4.2), abs=0.03)
    assert u["height"] == pytest.approx(1.75, abs=0.05)
    assert u["confidence"] == pytest.approx(0.88)


def test_async_detector_skips_while_busy():
    def slow(frame):
        time.sleep(0.2)
        return [frame]

    a = AsyncDetector(slow)
    assert a.submit("f1", 1.0)
    time.sleep(0.05)
    assert not a.submit("f2", 2.0)  # busy -> skipped
    time.sleep(0.3)
    assert a.poll() == (1.0, ["f1"])
    assert a.poll() is None
    a.close()


def test_real_yolo_on_sample_image():
    """End-to-end check of the real detector (skipped if ultralytics/model aren't available)."""
    ultralytics = pytest.importorskip("ultralytics")
    import cv2
    from pathlib import Path

    from vision.people import PersonDetector

    img = cv2.imread(str(Path(ultralytics.__file__).parent / "assets" / "bus.jpg"))
    if img is None:
        pytest.skip("sample image not found")
    try:
        det = PersonDetector("yolo11n.pt")
    except Exception as e:  # noqa: BLE001 - e.g. offline, can't download weights
        pytest.skip(f"model unavailable: {e}")
    people = det.detect(img)
    assert len(people) >= 3  # bus.jpg shows 4 people
