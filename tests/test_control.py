"""Laptop-side control: planner geometry, controller behaviour, a delayed closed-loop simulation, UDP."""

import math
import time

import pytest

from control.controller import Limits, RobotController, world_to_body
from control.link import VisionListener
from control.main import STOP, compute_commands
from control.planner import FormationPlanner, Target
from vision.output import UdpPublisher, make_message


def planner(**kw):
    args = dict(sides={1: "left", 2: "right"}, distance_m=1.5, spread_deg=60, facing_deg=-90, replan_m=0.3)
    args.update(kw)
    return FormationPlanner(**args)


def test_listening_triangle_geometry():
    t = planner().update((0.0, 4.0))
    left, right = t[1], t[2]
    for r in (left, right):
        assert math.hypot(r.x - 0.0, r.y - 4.0) == pytest.approx(1.5)
        assert r.y < 4.0  # facing the camera (-y): speakers are between the user and the camera
        # heading points the robot's front at the user
        assert r.heading_deg == pytest.approx(math.degrees(math.atan2(4.0 - r.y, 0.0 - r.x)))
    assert math.hypot(left.x - right.x, left.y - right.y) == pytest.approx(1.5)  # equilateral
    assert left.x > 0 > right.x  # facing -y, the user's left is +x


def test_targets_only_move_after_replan_distance():
    p = planner()
    first = p.update((0.0, 4.0))
    assert p.update((0.2, 4.0)) == first  # small sway: keep targets
    moved = p.update((0.5, 4.0))
    assert moved[1].x == pytest.approx(first[1].x + 0.5)
    assert p.update(None) == {}  # no user: nothing to plan


def test_targets_stay_inside_area():
    p = planner(area={"x": [-5, 5], "y": [-1, 10]})
    t = p.update((0.0, 0.0))  # user so close to the camera that the speakers would be behind it
    assert all(r.y >= -1 + 0.3 for r in t.values())


def test_world_to_body():
    # Robot facing +y: world +y is forward, world -x is to its left
    assert world_to_body(0.0, 1.0, 90.0) == pytest.approx((1.0, 0.0))
    assert world_to_body(-1.0, 0.0, 90.0) == pytest.approx((0.0, 1.0))


def test_controller_saturates_and_settles_with_hysteresis():
    lim = Limits()
    c = RobotController(lim)
    pose = {"x": 0.0, "y": 0.0, "heading": 0.0}
    vx, vy, wz = c.command(pose, Target(2.0, 0.0, 0.0))
    assert (vx, vy, wz) == pytest.approx((lim.max_speed_m_s, 0.0, 0.0))
    assert c.command(pose, Target(0.03, 0.0, 0.0)) == STOP  # inside tolerance: stop
    assert c.command(pose, Target(0.08, 0.0, 0.0)) == STOP  # 1.6x tolerance: still settled (hysteresis)
    assert c.command(pose, Target(0.12, 0.0, 0.0))[0] > 0  # beyond 2x tolerance: moves again
    vx, vy, wz = c.command(pose, Target(0.0, 0.0, 90.0))
    assert wz == pytest.approx(lim.max_yaw_rad_s)  # turn counter-clockwise toward +90


def test_keep_out_blocks_driving_into_the_user():
    c = RobotController(Limits(keep_out_m=0.8))
    pose = {"x": 0.0, "y": 0.5, "heading": 0.0}
    # The target is on the far side of a user standing at the origin: straight-line motion goes through them.
    vx, vy, _ = c.command(pose, Target(0.0, -1.0, 0.0), obstacles=[(0.0, 0.0, 0.8)])
    assert vy >= -1e-9  # no velocity toward the user (world -y = body -left for heading 0)
    # Sideways motion is still allowed, so it can slide around them.
    vx, vy, _ = c.command(pose, Target(1.0, -1.0, 0.0), obstacles=[(0.0, 0.0, 0.8)])
    assert vx > 0 and vy >= -1e-9


def _msg(robots, user, frame=1):
    return {"t": 0.0, "frame": frame, "robots": robots, "user": user}


def test_compute_commands_stops_when_anything_is_missing():
    ctl = {1: RobotController()}
    robot = {"id": 1, "x": 1.0, "y": 2.5, "heading": 0.0, "age": 0.1}
    user = {"x": 0.0, "y": 4.0, "age": 0.1}
    p = planner(sides={1: "left"})
    assert compute_commands(None, None, p, ctl)[1][1] == (STOP, "no vision")
    assert compute_commands(_msg([robot], user), 5.0, p, ctl)[1][1] == (STOP, "no vision")
    assert compute_commands(_msg([robot], None), 0.0, p, ctl)[1][1] == (STOP, "no user")
    assert compute_commands(_msg([robot], {**user, "age": 3.0}), 0.0, p, ctl)[1][1] == (STOP, "no user")
    assert compute_commands(_msg([{**robot, "age": 2.0}], user), 0.0, p, ctl)[1][1] == (STOP, "robot not seen")
    assert compute_commands(_msg([], user), 0.0, p, ctl)[1][1] == (STOP, "robot not seen")
    targets, out = compute_commands(_msg([robot], user), 0.0, p, ctl)
    cmd, reason = out[1]
    assert reason == "moving" and cmd != STOP and 1 in targets


def simulate(start, user, seconds=40.0, delay_s=0.5, camera_hz=4.0, actuator_scale=1.3, rate_hz=10.0,
             limits=None, strafe_turns_m=None):
    """Closed loop: controller at rate_hz, camera frames at camera_hz delivered delay_s late, and wheels
    that move actuator_scale times as far as commanded (badly calibrated counts-per-meter).
    strafe_turns_m: model robot 2, whose sideways command turns into clockwise rotation instead of
    sideways motion (yaw rate = -vy / strafe_turns_m)."""
    p = planner(sides={1: "left"})
    ctl = {1: RobotController(limits)}
    x, y, h = start
    dt = 1.0 / rate_hz
    frames = []  # (deliver_time, x, y, heading)
    next_frame = 0.0
    msg = None
    path = []
    for i in range(int(seconds * rate_hz)):
        t = i * dt
        if t >= next_frame:
            frames.append((t + delay_s, x, y, h))
            next_frame += 1.0 / camera_hz
        ready = [f for f in frames if f[0] <= t]
        if ready:
            _, mx, my, mh = ready[-1]
            msg = _msg([{"id": 1, "x": mx, "y": my, "heading": math.degrees(mh), "age": 0.1}],
                       {"x": user[0], "y": user[1], "age": 0.1}, frame=len(ready))
        _, out = compute_commands(msg, 0.0 if msg else None, p, ctl)
        vx, vy, wz = (actuator_scale * v for v in out[1][0])
        if strafe_turns_m is not None:
            wz, vy = wz - vy / strafe_turns_m, 0.0
        x += (math.cos(h) * vx - math.sin(h) * vy) * dt
        y += (math.sin(h) * vx + math.cos(h) * vy) * dt
        h += wz * dt
        path.append((x, y, h))
    return p.targets[1], path


def test_closed_loop_reaches_target_despite_delay_and_scale_error():
    target, path = simulate(start=(1.5, 1.5, 0.0), user=(0.0, 4.0))
    x, y, h = path[-1]
    assert math.hypot(x - target.x, y - target.y) < 0.1
    assert abs((math.degrees(h) - target.heading_deg + 180) % 360 - 180) < 10
    # Settled: the last 5 s barely move
    tail = path[-50:]
    assert max(math.hypot(a[0] - tail[0][0], a[1] - tail[0][1]) for a in tail) < 0.03


NO_STRAFE = Limits(allow_strafe=False)


def test_no_strafe_turns_first_and_never_strafes():
    c = RobotController(NO_STRAFE)
    pose = {"x": 0.0, "y": 0.0, "heading": 0.0}
    vx, vy, wz = c.command(pose, Target(0.0, 1.0, 0.0))  # target 90 deg to the left: turn, don't drive yet
    assert vy == 0.0 and vx == pytest.approx(0.0) and wz > 0
    vx, vy, wz = c.command(pose, Target(1.0, 0.05, 0.0))  # nearly straight ahead: drive forward
    assert vy == 0.0 and vx > 0.1
    vx, vy, wz = c.command(pose, Target(-1.0, 0.0, 0.0))  # straight behind: back up instead of turning around
    assert vy == 0.0 and vx < -0.1 and abs(wz) < 1e-9


def test_no_strafe_parks_then_turns_to_face_user():
    c = RobotController(NO_STRAFE)
    pose = {"x": 0.0, "y": 0.0, "heading": 0.0}
    vx, vy, wz = c.command(pose, Target(0.02, 0.0, 90.0))  # at the target: only turn toward the user
    assert (vx, vy) == (0.0, 0.0) and wz > 0
    # A small error after parking (under no_strafe_restart_m) does not restart driving
    vx, vy, wz = c.command(pose, Target(0.12, 0.0, 0.0))
    assert vx == 0.0


def test_no_strafe_keep_out():
    c = RobotController(NO_STRAFE)
    pose = {"x": 0.0, "y": 0.5, "heading": -90.0}  # facing the user at the origin, target beyond them
    vx, _, _ = c.command(pose, Target(0.0, -1.0, 0.0), obstacles=[(0.0, 0.0, 0.8)])
    assert vx <= 1e-9


def test_limits_from_config_reads_booleans():
    L = Limits.from_config({"allow_strafe": False, "max_speed_m_s": 0.1})
    assert L.allow_strafe is False and L.allow_reverse is True and L.max_speed_m_s == 0.1


def test_no_strafe_closed_loop_on_a_base_that_cannot_strafe():
    target, path = simulate(start=(1.5, 1.5, 0.0), user=(0.0, 4.0), seconds=60.0, limits=NO_STRAFE,
                            strafe_turns_m=0.15)
    x, y, h = path[-1]
    assert math.hypot(x - target.x, y - target.y) < 0.1
    assert abs((math.degrees(h) - target.heading_deg + 180) % 360 - 180) < 10
    tail = path[-50:]
    assert max(math.hypot(a[0] - tail[0][0], a[1] - tail[0][1]) for a in tail) < 0.03


def test_vision_listener_keeps_newest_message():
    listener = VisionListener(port=0)
    port = listener.sock.getsockname()[1]
    pub = UdpPublisher("127.0.0.1", port)
    try:
        for frame in (1, 2, 3):
            pub.send(make_message(time.time(), frame, [{"id": 1, "x": 1.0, "y": 2.0, "heading": 0.0, "age": 0.0}]))
        deadline = time.time() + 2.0
        msg = None
        while time.time() < deadline:
            msg, _ = listener.poll()
            if msg and msg["frame"] == 3:
                break
            time.sleep(0.01)
        assert msg["frame"] == 3 and msg["robots"][0]["id"] == 1
    finally:
        pub.close()
        listener.close()


def test_hallway_limits_keep_targets_off_the_walls():
    p = FormationPlanner({1: "right", 2: "left"}, distance_m=1.0, spread_deg=60, facing_deg=-90,
                         x_limits=[-0.52, 0.55], y_limits=[0.6, 6.0])
    centered = p.targets_for(0.0, 2.5)
    assert centered[1].x == pytest.approx(-0.5) and centered[2].x == pytest.approx(0.5)
    assert math.hypot(centered[1].x - centered[2].x, centered[1].y - centered[2].y) == pytest.approx(1.0)  # equilateral
    for ux in (-0.8, 0.8):  # user against either wall
        for t in p.targets_for(ux, 2.0).values():
            assert -0.52 <= t.x <= 0.55
    near_camera = p.targets_for(0.0, 0.9)
    assert all(t.y >= 0.6 for t in near_camera.values())
    with pytest.raises(ValueError):
        FormationPlanner({1: "left"}, x_limits=[1, 0])
