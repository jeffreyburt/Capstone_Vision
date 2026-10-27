import json
import socket

import numpy as np
import pytest

from vision.output import UdpPublisher, make_message
from vision.robots import RobotObservation
from vision.tracking import RobotTracker, Track


def test_track_smooths_noise():
    rng = np.random.default_rng(0)
    tr = Track(alpha=0.3, beta=0.02)
    for i in range(100):
        tr.update(i * 0.05, np.array([1.0, 2.0]) + rng.normal(0, 0.03, 2))
    assert np.allclose(tr.pos, [1.0, 2.0], atol=0.03)


def test_track_follows_motion():
    tr = Track(alpha=0.5, beta=0.1)
    for i in range(100):
        t = i * 0.05
        tr.update(t, [0.5 * t, 0.0])  # 0.5 m/s
    assert tr.pos[0] == pytest.approx(0.5 * 99 * 0.05, abs=0.02)
    assert tr.vel[0] == pytest.approx(0.5, abs=0.05)


def test_single_outlier_rejected_then_reset_if_persistent():
    tr = Track(max_jump_m=0.5)
    tr.update(0.0, [0, 0])
    assert not tr.update(0.1, [3, 0])
    assert np.allclose(tr.pos, [0, 0])
    assert not tr.update(0.2, [3, 0])
    assert tr.update(0.3, [3, 0])  # third time in a row -> accept, the object really is there
    assert np.allclose(tr.pos, [3, 0])


def test_heading_wraps():
    tr = Track(heading_alpha=0.5)
    tr.update(0.0, [0, 0], 170)
    tr.update(0.1, [0, 0], -170)
    assert abs(tr.heading) > 170  # moved toward 180, not back through 0


def test_tracker_forgets_old_robots():
    rt = RobotTracker([1, 2], {"forget_after_s": 2.0})
    rt.update(0.0, [RobotObservation(1, 1.0, 2.0, 90.0, [0])])
    assert [r["id"] for r in rt.state(1.0)] == [1]
    assert rt.state(3.0) == []


def test_udp_message_round_trip():
    rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    rx.bind(("127.0.0.1", 0))
    rx.settimeout(2)
    pub = UdpPublisher("127.0.0.1", rx.getsockname()[1])
    msg = make_message(12.3456, 7, [{"id": 1, "x": 1.23456, "y": 2.0, "heading": 45.06, "age": 0.01}])
    pub.send(msg)
    got = json.loads(rx.recv(65536))
    assert got["robots"][0] == {"id": 1, "x": 1.235, "y": 2.0, "heading": 45.1, "vx": 0.0, "vy": 0.0, "age": 0.01}
    assert got["user"] is None
    pub.close()
    rx.close()
