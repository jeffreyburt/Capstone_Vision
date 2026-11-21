"""Follow-mode feed-forward: robots move with a walking user instead of chasing them."""

import math

import pytest

from control.controller import Limits, RobotController
from control.main import compute_commands, user_feedforward
from control.planner import FormationPlanner, Target

ON = {"enabled": True, "gain": 1.0, "min_user_speed_m_s": 0.05, "max_m_s": 0.15, "lead_s": 0.4}


def test_user_feedforward_off_standing_walking_and_capped():
    walking = {"x": 1.0, "y": 4.0, "vx": 0.1, "vy": 0.0, "age": 0.1}
    assert user_feedforward(walking, None) == (None, None)                      # disabled by default
    assert user_feedforward({**walking, "vx": 0.02}, ON) == (None, None)        # standing still (noise)
    ff, plan_from = user_feedforward(walking, ON)
    assert ff == pytest.approx((0.1, 0.0)) and plan_from == pytest.approx((1.04, 4.0))  # 0.4 s lead
    ff, _ = user_feedforward({**walking, "vx": 0.6, "vy": 0.8}, ON)            # 1.0 m/s -> capped at 0.15
    assert math.hypot(*ff) == pytest.approx(0.15)


def test_planner_force_replans_every_step():
    p = FormationPlanner({1: "left"}, replan_m=0.3)
    first = p.update((0.0, 4.0))
    assert p.update((0.1, 4.0)) == first                       # standing: hysteresis keeps the targets
    moved = p.update((0.1, 4.0), force=True)                   # walking: targets follow every step
    assert moved[1].x == pytest.approx(first[1].x + 0.1)


def test_controller_adds_feedforward_and_caps_total():
    c = RobotController(Limits(max_speed_m_s=0.15))
    pose = {"x": 0.0, "y": 0.0, "heading": 0.0}
    vx, vy, _ = c.command(pose, Target(0.0, 0.0, 0.0), feedforward=(0.08, 0.0))  # on target, target moving
    assert vx == pytest.approx(0.08) and vy == pytest.approx(0.0)
    vx, vy, _ = c.command(pose, Target(1.0, 0.0, 0.0), feedforward=(0.1, 0.0))   # far + moving: capped
    assert math.hypot(vx, vy) == pytest.approx(0.15)


def test_no_strafe_feedforward_only_along_the_chassis_when_aligned():
    pose = {"x": 0.0, "y": 0.0, "heading": 0.0}
    plain, _, _ = RobotController(Limits(allow_strafe=False)).command(pose, Target(0.1, 0.0, 0.0))
    c = RobotController(Limits(allow_strafe=False))
    vx, vy, _ = c.command(pose, Target(0.1, 0.0, 0.0), feedforward=(0.05, 0.05))  # aligned with the target
    assert vy == 0.0 and vx == pytest.approx(plain + 0.05)  # only the forward 0.05 of the feed-forward is added
    c = RobotController(Limits(allow_strafe=False))
    vx, vy, wz = c.command(pose, Target(0.0, 1.0, 0.0), feedforward=(0.1, 0.0))   # target 90 deg off: turn only
    assert vx == pytest.approx(0.0) and vy == 0.0 and wz > 0


def _msg(robot, user, frame):
    return {"t": 0.0, "frame": frame, "robots": [robot], "user": user}


def walk(feedforward, limits=None, user_speed=0.1, seconds=25.0, delay_s=0.5, camera_hz=4.0, rate_hz=10.0):
    """User walks along +x at user_speed; the camera reports user and robot delay_s late at camera_hz. Returns
    the robot's mean distance to where its target really is (no delay), over the last 10 s."""
    p = FormationPlanner({1: "right"}, distance_m=1.5)
    ctl = {1: RobotController(limits)}
    ux0, uy = -1.5, 4.0
    x, y = p.targets_for(ux0, uy)[1].x, p.targets_for(ux0, uy)[1].y  # start on the target
    h = math.radians(p.targets_for(ux0, uy)[1].heading_deg)
    dt = 1.0 / rate_hz
    frames, next_frame, msg, errors = [], 0.0, None, []
    for i in range(int(seconds * rate_hz)):
        t = i * dt
        ux = ux0 + user_speed * t
        if t >= next_frame:
            frames.append((t + delay_s, x, y, h, ux))
            next_frame += 1.0 / camera_hz
        ready = [f for f in frames if f[0] <= t]
        if ready:
            _, mx, my, mh, mux = ready[-1]
            msg = _msg({"id": 1, "x": mx, "y": my, "heading": math.degrees(mh), "age": 0.1},
                       {"x": mux, "y": uy, "vx": user_speed, "vy": 0.0, "age": 0.1}, len(ready))
        _, out = compute_commands(msg, 0.0 if msg else None, p, ctl, feedforward=feedforward)
        vx, vy, wz = out[1][0]
        x += (math.cos(h) * vx - math.sin(h) * vy) * dt
        y += (math.sin(h) * vx + math.cos(h) * vy) * dt
        h += wz * dt
        true_target = p.targets_for(ux, uy)[1]
        if t >= seconds - 10:
            errors.append(math.hypot(x - true_target.x, y - true_target.y))
    return sum(errors) / len(errors)


def test_feedforward_cuts_tracking_error_for_a_walking_user():
    without = walk(None)
    with_ff = walk(ON)
    assert with_ff < 0.5 * without, (with_ff, without)
    assert with_ff < 0.08


def test_feedforward_no_strafe_mode_also_improves():
    limits = Limits(allow_strafe=False)
    assert walk(ON, limits) < walk(None, limits)
