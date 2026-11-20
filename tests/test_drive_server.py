"""Robot-side drive server: wheel mixing, channel mapping, ramp, watchdog, command parsing, UDP round trip."""

import json
import math
import time

import pytest

from control.link import RobotLink
from robot.drive_server import DriveParams, DriveServer, PrintMotors, parse_command, to_motors, wheel_counts

# Nate's config/teleop.json mapping: FL=M1, FR=M3, RL=M2, RR=M4, Hiwonder forward pattern [+,-,-,+]
TELEOP = dict(wheel_channels=[1, 3, 2, 4], motor_signs=[1, -1, -1, 1])


def params(**kw):
    return DriveParams(**{**TELEOP, "counts_per_meter": 13000.0, "turn_radius_m": 0.2, "max_counts": 25, **kw})


def test_forward_matches_hiwonder_forward_pattern():
    p = params()
    wheels = wheel_counts(0.1, 0.0, 0.0, p)
    assert wheels == pytest.approx([13.0] * 4)
    assert to_motors(wheels, p) == [13, -13, -13, 13]


def test_strafe_and_turn_patterns():
    p = params()
    fl, fr, rl, rr = wheel_counts(0.0, 0.1, 0.0, p)  # strafe left
    assert fl < 0 < fr and rl > 0 > rr
    fl, fr, rl, rr = wheel_counts(0.0, 0.0, 0.5, p)  # spin counter-clockwise: left wheels back, right forward
    assert fl < 0 and rl < 0 and fr > 0 and rr > 0


def test_speed_cap_keeps_direction():
    p = params()
    wheels = wheel_counts(1.0, 0.5, 0.0, p)
    assert max(abs(w) for w in wheels) == pytest.approx(25)
    raw = wheel_counts(0.1, 0.05, 0.0, params(max_counts=1000))
    assert [w / wheels[0] for w in wheels] == pytest.approx([r / raw[0] for r in raw])


@pytest.mark.parametrize("msg", [
    {"type": "drive", "robot_id": 2, "vx": 0.1, "vy": 0, "wz": 0},      # wrong robot
    {"type": "drive", "robot_id": 1, "vx": float("nan"), "vy": 0, "wz": 0},
    {"type": "drive", "robot_id": 1, "vx": True, "vy": 0, "wz": 0},
    {"type": "drive", "robot_id": 1, "vx": 0.1, "vy": 0},                # missing wz
    {"type": "dance", "robot_id": 1},
])
def test_parse_rejects_bad_commands(msg):
    with pytest.raises(ValueError):
        parse_command(json.dumps(msg), robot_id=1)


class FakeClock:
    def __init__(self):
        self.t = 100.0

    def __call__(self):
        return self.t


def make_server(**kw):
    clock = FakeClock()
    motors = PrintMotors()
    srv = DriveServer(motors, params(), robot_id=1, port=0, host="127.0.0.1", clock=clock, log=lambda s: None, **kw)
    return srv, motors, clock


def send(srv, clock, **msg):
    srv.handle(json.dumps({"robot_id": 1, **msg}).encode(), ("127.0.0.1", 9), clock())


def test_ramps_up_then_watchdog_stops_immediately():
    srv, motors, clock = make_server(timeout_s=0.5)
    try:
        send(srv, clock, type="drive", vx=0.1, vy=0.0, wz=0.0)
        srv.tick(clock())
        first = motors.values
        assert 0 < first[0] < 13  # ramping, not a step
        for _ in range(8):  # keep commanding for 0.4 s
            clock.t += 0.05
            send(srv, clock, type="drive", vx=0.1, vy=0.0, wz=0.0)
            srv.tick(clock())
        assert motors.values == (13, -13, -13, 13)
        clock.t += 0.6  # commands stop arriving
        srv.tick(clock())
        assert motors.values == (0, 0, 0, 0)
        assert "no commands" in srv.state
    finally:
        srv.sock.close()


def test_stop_command_and_speed_clamp():
    srv, motors, clock = make_server(max_speed_m_s=0.1)
    try:
        send(srv, clock, type="drive", vx=5.0, vy=0.0, wz=0.0)  # absurd speed is clamped to 0.1 m/s
        assert srv.target[0] == pytest.approx(0.1)
        for _ in range(20):
            clock.t += 0.05
            send(srv, clock, type="drive", vx=5.0, vy=0.0, wz=0.0)
            srv.tick(clock())
        assert motors.values == (13, -13, -13, 13)
        send(srv, clock, type="stop")
        srv.tick(clock())
        assert motors.values == (0, 0, 0, 0)
    finally:
        srv.sock.close()


def test_udp_round_trip_with_robot_link():
    srv, motors, clock = make_server()
    link = RobotLink({1: ("127.0.0.1", srv.port)})
    try:
        link.drive(1, 0.1, 0.0, 0.0)
        srv.sock.settimeout(2.0)
        data, addr = srv.sock.recvfrom(4096)
        srv.handle(data, addr, clock())
        assert srv.state == "driving" and srv.target == pytest.approx((0.1, 0.0, 0.0))
        deadline = time.time() + 2.0
        while time.time() < deadline and link.poll()[1] is None:
            time.sleep(0.01)
        assert link.status[1]["state"] == "driving" and link.status[1]["error"] is None
    finally:
        link.close()
        srv.sock.close()


def test_link_poll_before_any_send():
    link = RobotLink({1: ("127.0.0.1", 9)})  # dry run: control.main polls without ever sending
    try:
        assert link.poll() == {1: None}
    finally:
        link.close()


def test_rejects_commands_from_other_hosts():
    srv, motors, clock = make_server(allow_from="192.168.50.10")
    try:
        send(srv, clock, type="drive", vx=0.1, vy=0.0, wz=0.0)  # arrives from 127.0.0.1
        assert srv.target == (0.0, 0.0, 0.0) and srv.rejected == 1
        assert not math.isnan(srv.received)
    finally:
        srv.sock.close()


def test_three_wheel_mode_masks_failed_channel_and_caps_active_wheels():
    p = DriveParams([3, 1, 4, 2], [1, 1, -1, -1], disabled_channels=[1])
    assert p.max_counts == 6
    for velocity in ((1, 0, 0), (0, 1, 0), (0, 0, 1), (1, -1, 1)):
        wheels = wheel_counts(*velocity, p)
        motors = to_motors(wheels, p)
        assert motors[0] == 0
        assert max(abs(v) for v in motors) <= 6
    # The final output gate also blocks stale or unexpected wheel values.
    assert to_motors([5, 100, 5, 5], p) == [0, -5, 5, -5]


@pytest.mark.parametrize('disabled', [[0], [5], [True], [1, 2], [1, 1]])
def test_invalid_disabled_channels_rejected(disabled):
    with pytest.raises(ValueError):
        params(disabled_channels=disabled)


def test_three_wheel_ramping_and_watchdog_keep_failed_output_zero():
    clock = FakeClock()
    motors = PrintMotors()
    p = DriveParams([3, 1, 4, 2], [1, 1, -1, -1], disabled_channels=[1])
    srv = DriveServer(motors, p, robot_id=1, port=0, host='127.0.0.1', clock=clock, log=lambda s: None)
    try:
        for _ in range(10):
            send(srv, clock, type='drive', vx=0.1, vy=0, wz=0)
            srv.tick(clock())
            assert motors.values[0] == 0
            clock.t += 0.05
        assert motors.values == (0, -6, 6, -6)
        assert srv._status_reply(None, clock())['disabled_channels'] == [1]
        clock.t += 1
        srv.tick(clock())
        assert motors.values == (0, 0, 0, 0)
    finally:
        srv.sock.close()
