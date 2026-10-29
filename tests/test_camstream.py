import socket
import time

import numpy as np
import pytest

zmq = pytest.importorskip("zmq")

from camstream.protocol import pack, unpack
from camstream.receiver import FrameReceiver
from camstream.sender import CameraSender
from vision.camera import StreamSource, TestPatternSource


def free_port():
    s = socket.socket()
    s.bind(("127.0.0.1", 0))
    port = s.getsockname()[1]
    s.close()
    return port


@pytest.fixture
def stream_cfg():
    port = free_port()
    return {"streams": {"cam": {"host": "127.0.0.1", "port": port, "quality": 80}}}, port


def test_protocol_round_trip():
    h = {"camera": "robot1", "seq": 5, "meta": {"pan_deg": 1.5}}
    got_h, got_p = unpack(pack(h, b"\xff\xd8jpegdata"))
    assert got_h == h and got_p == b"\xff\xd8jpegdata"
    with pytest.raises(ValueError):
        unpack(b"garbage")


def test_sender_to_receiver(stream_cfg):
    cfg, port = stream_cfg
    sender = CameraSender("cam", port, TestPatternSource(320, 240, fps=30, label="cam")).start()
    sender.update_meta(pan_deg=10.0, tilt_deg=-2.0)
    rx = FrameReceiver(cfg, ["cam"])
    try:
        f1 = rx.wait_new("cam", None, timeout=5)
        assert f1 is not None, "no frame received"
        f2 = rx.wait_new("cam", f1.seq, timeout=5)
        assert f2.seq > f1.seq
        assert f2.image.shape == (240, 320, 3)
        assert f2.header["camera"] == "cam"
        assert f2.meta == {"pan_deg": 10.0, "tilt_deg": -2.0}
        assert 0 <= f2.t_received - f2.t_capture < 1.0  # same machine, same clock
        time.sleep(0.5)
        s = rx.stats("cam")
        assert s["fps"] > 10 and s["mbps"] > 0
    finally:
        rx.close()
        sender.stop()


def test_slow_consumer_gets_newest_frame(stream_cfg):
    """A consumer that reads rarely must get a fresh frame, not a stale queued one."""
    cfg, port = stream_cfg
    sender = CameraSender("cam", port, TestPatternSource(320, 240, fps=30)).start()
    rx = FrameReceiver(cfg, ["cam"])
    try:
        assert rx.wait_new("cam", None, timeout=5) is not None
        time.sleep(1.0)
        f = rx.get_latest("cam")
        assert time.time() - f.t_capture < 0.2
    finally:
        rx.close()
        sender.stop()


def test_stream_source_for_vision(stream_cfg):
    cfg, port = stream_cfg
    sender = CameraSender("cam", port, TestPatternSource(320, 240, fps=30)).start()
    src = StreamSource(cfg, "cam")
    try:
        img, t = src.read()
        img2, t2 = src.read()
        assert img.shape == (240, 320, 3) and t2 > t
    finally:
        src.close()
        sender.stop()


def test_rate_limit_does_not_halve_fps(stream_cfg):
    """Camera rate == max_fps must give the full rate (timing jitter used to drop every other frame)."""
    cfg, port = stream_cfg
    sender = CameraSender("cam", port, TestPatternSource(320, 240, fps=20), max_fps=20).start()
    rx = FrameReceiver(cfg, ["cam"])
    try:
        assert rx.wait_new("cam", None, timeout=5) is not None
        time.sleep(2.0)
        assert rx.stats("cam")["fps"] > 17
    finally:
        rx.close()
        sender.stop()
