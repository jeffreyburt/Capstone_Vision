"""Follow-test fixes: no turning on a held heading, targets kept inside the camera view, robots swap sides
instead of crossing."""

import pytest

from control.controller import Limits, RobotController
from control.main import compute_commands
from control.planner import FormationPlanner, Target
from vision.tracking import RobotTracker


def _robot(rid, x, y, heading=0.0, heading_age=0.0):
    return {"id": rid, "x": x, "y": y, "heading": heading, "age": 0.0, "heading_age": heading_age}


def _msg(robots, user=(0.0, 3.0), view=None):
    m = {"t": 0.0, "frame": 1, "robots": robots, "user": {"x": user[0], "y": user[1], "age": 0.0}}
    if view:
        m["view"] = {"polygon": view}
    return m


def test_tracker_reports_how_old_the_heading_is():
    tr = RobotTracker([1])
    tr.update(0.0, [type("O", (), {"robot_id": 1, "x": 0.0, "y": 1.0, "heading_deg": 90.0})()])
    for t in (0.1, 0.2, 0.3, 0.4):  # tag-centre fixes: position only
        tr.update(t, [type("O", (), {"robot_id": 1, "x": 0.0, "y": 1.0, "heading_deg": None})()])
    (r,) = tr.state(0.4)
    assert r["heading"] == 90.0 and r["heading_age"] == pytest.approx(0.4) and r["age"] == pytest.approx(0.0)


def test_held_heading_translates_but_never_turns():
    planner = FormationPlanner({1: "right"}, distance_m=1.0)
    for age, turning in ((0.1, True), (2.0, False)):
        ctl = {1: RobotController(Limits())}
        _, out = compute_commands(_msg([_robot(1, 0.0, 1.0, heading=0.0, heading_age=age)]), 0.0, planner, ctl)
        (vx, vy, wz), why = out[1]
        assert (abs(wz) > 0) == turning
        if not turning:
            assert why == "heading held, not turning" and abs(vx) + abs(vy) > 0


def test_targets_pulled_inside_the_camera_view():
    planner = FormationPlanner({1: "left", 2: "right"}, distance_m=1.5, x_limits=[-3, 3])
    view = [[-0.8, 0.3], [0.8, 0.3], [2.0, 4.0], [-2.0, 4.0]]  # narrow near the camera
    ctl = {1: RobotController(Limits()), 2: RobotController(Limits())}
    targets, _ = compute_commands(_msg([_robot(1, 0.5, 1.5), _robot(2, -0.5, 1.5)], user=(0.0, 2.0), view=view),
                                  0.0, planner, ctl)
    from control.view_guard import CameraView
    v = CameraView(view)
    for t in targets.values():
        assert v.boundary(t.x, t.y)[0] >= 0.35 - 1e-3


def test_robots_on_the_wrong_sides_swap_targets_with_hysteresis():
    p = FormationPlanner({1: "right", 2: "left"})
    targets = {1: Target(-1.0, 1.0, 90.0), 2: Target(1.0, 1.0, 90.0)}
    right = {1: {"x": -0.9, "y": 1.0}, 2: {"x": 0.9, "y": 1.0}}
    crossed = {1: {"x": 0.9, "y": 1.0}, 2: {"x": -0.9, "y": 1.0}}
    assert p.assign(targets, right) == targets
    swapped = p.assign(targets, crossed)
    assert swapped[1] is targets[2] and swapped[2] is targets[1]
    middle = {1: {"x": 0.05, "y": 1.0}, 2: {"x": -0.05, "y": 1.0}}  # no clear winner: keep the current pairing
    assert p.assign(targets, middle)[1] is targets[2]
    assert p.assign(targets, right) == targets


def _line_planner():
    return FormationPlanner({1: "right", 2: "left"}, x_limits=[-1.15, 1.15], y_limits=[0.6, 6.0], line_y_m=1.0,
                            min_side_m=0.8, max_turn_deg=20)


def test_user_near_a_wall_never_puts_both_robots_on_one_spot():
    p = _line_planner()
    for ux in (0.9, 1.27, 1.6, -1.4):
        t = p.targets_for(ux, 2.1)
        assert abs(t[1].x - t[2].x) >= 0.8 - 1e-6
        assert all(-1.15 - 1e-6 <= v.x <= 1.15 + 1e-6 for v in t.values())


def test_one_frame_user_jump_is_ignored_but_a_real_move_is_followed():
    p = _line_planner()
    first = p.update((0.0, 2.4))
    assert p.update((0.0, 7.5)) == first             # one bad frame: targets unchanged
    assert p.update((0.0, 2.45)) == first
    for _ in range(p.JUMP_CONFIRM):                   # the user really is over there now
        moved = p.update((1.0, 4.5))
    assert moved != first
