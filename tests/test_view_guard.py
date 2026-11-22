"""Camera-view soft stop and return-to-view (control/view_guard.py) and the view polygon (vision)."""

import math

import numpy as np
import pytest

from control.controller import Limits, RobotController
from control.main import DeadReckoner, fresh_poses, vision_usable
from control.view_guard import STOP, CameraView, ViewGuard, soft_limit
from vision.geometry import CameraModel, FloorPose, Geometry, lens_fold_radius, visible_floor_polygon

# A wedge like the real main camera: apex at the camera, edges at +-45 deg from +y, clipped at 8 m.
WEDGE = [[0.0, 0.0], [5.657, 5.657], [0.0, 8.0], [-5.657, 5.657]]
SETTINGS = {"stop_m": 0.2, "slow_m": 0.5, "return_speed_m_s": 0.1, "return_inset_m": 0.15,
            "return_timeout_s": 4.0, "return_max_m": 0.6}


def edge_distance(x, y):
    """Distance inside the left edge of WEDGE (the line y = -x)."""
    return (x + y) / math.sqrt(2)


def test_lens_fold_radius():
    assert lens_fold_radius(np.zeros(5)) == math.inf
    # The main camera's calibrated distortion peaks at ~1.26 (51.5 deg off axis, the Wide lens' half-FOV).
    real = [-0.07277557714677091, 0.14969925483848512, 0.0014, 0.00006, -0.0914401289250165]
    assert lens_fold_radius(real) == pytest.approx(1.257, abs=0.005)


def test_view_polygon_matches_direct_projection():
    g = Geometry(CameraModel.from_fov(1920, 1080, 66), FloorPose.look_at([0, 0, 2.0], [0, 4.0, 0.3]))
    view = CameraView(visible_floor_polygon(g, (0.25, 0.35), margin_px=20))
    rng = np.random.default_rng(1)
    pts = rng.uniform([-5, -1], [5, 9], size=(3000, 2))
    disagree = 0
    for x, y in pts:
        px = g.world_to_pixels([[x, y, 0.25], [x, y, 0.35]])
        truth = bool(np.all((px >= 20) & (px <= [1919 - 20, 1079 - 20])))
        d, _ = view.boundary(x, y)
        if abs(d) > 0.12 and view.contains(x, y) != truth and math.hypot(x, y) < 7.9:
            disagree += 1
    assert disagree == 0


def test_camera_view_from_vision_message():
    from vision.pipeline import VisionPipeline
    g = Geometry(CameraModel.from_fov(1920, 1080, 66), FloorPose.look_at([0, 0, 2.0], [0, 4.0, 0.3]))
    pipe = VisionPipeline({"robots": [{"id": 1, "faces": {0: 0}}], "control": {"view_guard": {"edge_margin_frac": 0.01}}},
                          geometry=g)
    msg = pipe.process(np.zeros((1080, 1920, 3), np.uint8), 1.0).message
    assert msg["view"]["margin_px"] == pytest.approx(19.2) and msg["view"]["heights_m"] == [0.25, 0.35]
    view = CameraView.from_message(msg)
    assert view.contains(0.0, 3.0) and not view.contains(5.0, 3.0)
    off = VisionPipeline({"control": {"view_guard": {"enabled": False}}}, geometry=g)
    assert "view" not in off.process(np.zeros((1080, 1920, 3), np.uint8), 1.0).message


def test_signed_distance_and_inward_point():
    view = CameraView(WEDGE)
    d, n = view.boundary(-1.0, 1.5)
    assert d == pytest.approx(edge_distance(-1.0, 1.5)) and n == pytest.approx((-1 / math.sqrt(2), -1 / math.sqrt(2)))
    d, n = view.boundary(-2.0, 1.0)  # outside: negative, normal still points outward
    assert d < 0 and d == pytest.approx(edge_distance(-2.0, 1.0)) and n == pytest.approx((-1 / math.sqrt(2), -1 / math.sqrt(2)))
    x, y = view.inward_point(-2.0, 1.0, 0.35)
    assert view.boundary(x, y)[0] >= 0.35 - 1e-3


def test_soft_limit_scales_only_the_outward_part():
    n = (-1.0, 0.0)  # edge to the -x side
    assert soft_limit(-0.3, 0.0, 0.6, n, 0.2, 0.5) == (-0.3, 0.0, 1.0)         # far from the edge
    vx, vy, f = soft_limit(-0.3, 0.1, 0.3, n, 0.2, 0.5)
    assert f == pytest.approx(1 / 3) and vx == pytest.approx(-0.1) and vy == pytest.approx(0.1)  # slides along
    assert soft_limit(-0.3, 0.1, 0.1, n, 0.2, 0.5)[:2] == pytest.approx((0.0, 0.1))  # inside stop_m: none outward
    assert soft_limit(0.3, 0.0, -0.1, n, 0.2, 0.5) == (0.3, 0.0, 1.0)           # inward always allowed


def _msg(robots, user=None):
    return {"t": 0.0, "robots": robots, "user": user, "view": {"polygon": WEDGE}}


def _robot(x, y, heading=90.0):
    return {"id": 1, "x": x, "y": y, "heading": heading, "age": 0.0}


def test_seen_robot_is_soft_stopped_at_the_edge():
    guard, ctl = ViewGuard(SETTINGS), {1: RobotController(Limits(allow_strafe=True))}
    # 0.1 m inside the left edge, heading +y, commanded straight left (body +vy = world -x): blocked outward.
    x, y = -1.0 + 0.1 * math.sqrt(2) / 2, 1.0 + 0.1 * math.sqrt(2) / 2
    robot = _robot(x, y)
    d = CameraView(WEDGE).boundary(robot["x"], robot["y"])[0]
    assert d < 0.2
    out = guard.apply(_msg([robot]), {1: ((0.0, 0.2, 0.0), "moving")}, ctl, {1: DeadReckoner()},
                      fresh={1: robot}, vision_ok=True, allow_return=True, now=10.0)
    cmd, reason = out[1]
    wx, wy = -cmd[1], cmd[0]  # heading 90: body (fwd, left) -> world (x, y) = (-left, fwd)
    assert -(wx + wy) / math.sqrt(2) == pytest.approx(0, abs=1e-9)  # nothing toward the outside (normal (-1,-1)/sqrt2)
    assert (wx, wy) == pytest.approx((-0.1, 0.1))  # the along-edge part is kept: it slides along the edge
    assert "camera view edge" in reason
    # Moving inward is unchanged.
    out = guard.apply(_msg([robot]), {1: ((0.0, -0.2, 0.0), "moving")}, ctl, {1: DeadReckoner()},
                      fresh={1: robot}, vision_ok=True, allow_return=True, now=10.0)
    assert out[1] == ((0.0, -0.2, 0.0), "moving")


def test_no_strafe_robot_slows_its_forward_speed():
    guard, ctl = ViewGuard(SETTINGS), {1: RobotController(Limits(allow_strafe=False))}
    robot = _robot(-1.0, 1.0 + 0.3 * math.sqrt(2), heading=180.0)  # facing -x, 0.3 m inside the left edge
    d = CameraView(WEDGE).boundary(robot["x"], robot["y"])[0]
    out = guard.apply(_msg([robot]), {1: ((0.2, 0.0, 0.0), "moving")}, ctl, {1: DeadReckoner()},
                      fresh={1: robot}, vision_ok=True, allow_return=True, now=1.0)
    assert out[1][0][1] == 0.0 and 0 <= out[1][0][0] < 0.2
    assert out[1][0][0] == pytest.approx(0.2 * min(1, max(0, (d - 0.2) / 0.3)), abs=1e-6)


def test_lost_robot_returns_from_its_estimate_then_watchdog_stops_it():
    guard, ctl = ViewGuard(SETTINGS), {1: RobotController(Limits(allow_strafe=True))}
    reck = {1: DeadReckoner(max_s=1.5)}
    robot = _robot(-0.9, 1.0)  # ~0.07 m inside the left edge
    out = guard.apply(_msg([robot]), {1: ((0.0, 0.3, 0.0), "moving")}, ctl, reck,
                      fresh={1: robot}, vision_ok=True, allow_return=True, now=0.0)
    reck[1].sent((0.0, 0.3, 0.0), 0.0)  # pretend the limited command was not honoured (slip, latency)
    # Unseen 0.5 s later: estimated 0.15 m further left, past the edge -> drives back in, heading held.
    out = guard.apply(_msg([]), {1: (STOP, "robot not seen")}, ctl, reck,
                      fresh={}, vision_ok=True, allow_return=True, now=0.5)
    cmd, reason = out[1]
    assert reason.startswith("returning to camera view") and cmd[2] == 0.0
    wx, wy = -cmd[1], cmd[0]
    assert (wx, wy) == pytest.approx((0.1 / math.sqrt(2), 0.1 / math.sqrt(2)))  # straight back in, at return speed
    reck[1].sent(cmd, 0.5)
    # Not allowed to return (keyboard / disarmed / paused): leave the stop alone.
    assert guard.apply(_msg([]), {1: (STOP, "x")}, ctl, reck, fresh={}, vision_ok=True, allow_return=False,
                       now=0.6)[1] == (STOP, "x")
    # No usable vision: nothing is guarded or returned.
    assert guard.apply(None, {1: (STOP, "no vision")}, ctl, reck, fresh={}, vision_ok=False, allow_return=True,
                       now=0.7)[1] == (STOP, "no vision")
    # Keep returning until the estimate is back inside, then wait for the camera.
    t, reason = 0.7, ""
    while t < 3.9 and not reason.startswith("back inside"):
        cmd, reason = guard.apply(_msg([]), {1: (STOP, "robot not seen")}, ctl, reck,
                                  fresh={}, vision_ok=True, allow_return=True, now=t)[1]
        reck[1].sent(cmd, t)
        t += 0.1
    assert reason.startswith("back inside the camera view") and cmd == STOP
    # Watchdog: estimate too old -> stays stopped with a hand-recovery reason until it is seen again.
    cmd, reason = guard.apply(_msg([]), {1: (STOP, "robot not seen")}, ctl, reck,
                              fresh={}, vision_ok=True, allow_return=True, now=4.5)[1]
    assert cmd == STOP and reason.startswith("lost outside the camera view")
    seen = _robot(-0.5, 1.5)
    out = guard.apply(_msg([seen]), {1: ((0.1, 0.0, 0.0), "moving")}, ctl, reck,
                      fresh={1: seen}, vision_ok=True, allow_return=True, now=5.0)
    assert out[1] == ((0.1, 0.0, 0.0), "moving") and 1 not in guard.state


def test_return_distance_watchdog():
    guard, ctl = ViewGuard({**SETTINGS, "return_max_m": 0.05}), {1: RobotController(Limits(allow_strafe=True))}
    reck = {1: DeadReckoner()}
    robot = _robot(-0.95, 1.0)
    guard.apply(_msg([robot]), {1: (STOP, "")}, ctl, reck, fresh={1: robot}, vision_ok=True, allow_return=True, now=0.0)
    reck[1].sent((0.0, 0.2, 0.0), 0.0)
    cmd, reason = guard.apply(_msg([]), {1: (STOP, "robot not seen")}, ctl, reck,
                              fresh={}, vision_ok=True, allow_return=True, now=0.2)[1]
    assert reason.startswith("returning")
    reck[1].sent(cmd, 0.2)
    cmd, reason = guard.apply(_msg([]), {1: (STOP, "robot not seen")}, ctl, reck,
                              fresh={}, vision_ok=True, allow_return=True, now=1.0)[1]
    assert cmd == STOP and reason.startswith("lost outside")  # moved > 0.05 m on the estimate: give up


def test_unseen_robot_in_the_middle_is_not_returned():
    guard, ctl = ViewGuard(SETTINGS), {1: RobotController(Limits(allow_strafe=True))}
    reck = {1: DeadReckoner()}
    robot = _robot(0.0, 2.0)
    guard.apply(_msg([robot]), {1: (STOP, "")}, ctl, reck, fresh={1: robot}, vision_ok=True, allow_return=True, now=0.0)
    assert guard.apply(_msg([]), {1: (STOP, "robot not seen")}, ctl, reck, fresh={}, vision_ok=True,
                       allow_return=True, now=0.5)[1] == (STOP, "robot not seen")


def test_fresh_poses_and_vision_usable():
    msg = {"t": 0.0, "robots": [_robot(0, 2), {"id": 2, "x": 1, "y": 2, "heading": None, "age": 0.0},
                                {**_robot(0, 2), "id": 3, "age": 2.0}]}
    assert set(fresh_poses(msg, 0.1, {1, 2, 3})) == {1}
    assert fresh_poses(msg, 0.1, {1}, compensated_ids={1}) == {}  # moving head: needs a compensated pose
    assert vision_usable(msg, 0.1) and not vision_usable(msg, 5.0) and not vision_usable(None, 0.1)
    assert not vision_usable({**msg, "position_quality": "approximate"}, 0.1)
