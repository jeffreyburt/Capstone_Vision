"""Control black box (control/blackbox.py) and its reader (tools/blackbox.py)."""

import gzip
import json
import re
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import pytest

from control.blackbox import BlackBox, grab_jpeg, reason_kind
from control.main import DeadReckoner
from control.planner import Target
from tools import blackbox as reader

WEDGE = [[0.0, 0.0], [5.0, 5.0], [0.0, 8.0], [-5.0, 5.0]]


class Clock:
    def __init__(self, t=1_800_000_000.0):
        self.t = t

    def __call__(self):
        return self.t


def _run_scenario(tmp_path, snapshot_url=None):
    clock = Clock()
    bb = BlackBox({"dir": str(tmp_path), "snapshot_url": snapshot_url, "snapshot_min_interval_s": 0},
                  argv=["control.main", "--waypoints"], robots=[1], clock=clock)

    class Guard:
        state = {}

    guard, reck = Guard(), {1: DeadReckoner()}
    nav = {"armed": False, "follow": True, "targets": {}, "manual": {"enabled": False}, "mark": {"seq": 3, "note": "old"}}
    t0 = clock.t
    for k in range(60):  # 3 s at 20 Hz: seen, then lost and returned by the view guard, then a Mark
        now = t0 + k * 0.05
        seen = k < 20
        robot = {"id": 1, "x": -0.5 - 0.01 * k, "y": 1.0, "heading": 90.0, "age": 0.0}
        msg = {"t": now, "frame": k, "robots": [robot] if seen else [], "user": {"x": 0, "y": 2.5, "age": 0.1},
               "view": {"polygon": WEDGE, "heights_m": [0.25, 0.35]}}
        if seen:
            reck[1].observe(robot, now)
            out = {1: ((0.0, 0.1, 0.0), "following")}
        else:
            guard.state = {1: "returning"}
            reck[1].estimate(now, max_s=4)
            out = {1: ((0.0, -0.1, 0.0), f"returning to camera view (estimate {now - t0 - 1:.1f} s old)")}
        if k == 50:
            nav = {**nav, "mark": {"seq": 4, "note": "R1 went off screen"}}
        # Head tracker state as the black box polls it: mostly the head camera, every 4th step the main camera.
        head = {"active": True, "mode": "observed", "source": "head camera" if k % 4 else "main camera",
                "faceAge": 0.1}
        bb.record(now, mode="ARMED", navigation_mode="follow", msg=msg, msg_age=0.05, nav=nav,
                  targets={1: Target(-0.6, 1.5, 90.0)}, results=out, link_ages={1: 0.1}, link_status={1: "ok"},
                  reckoners=reck, guard=guard, heads={"1": head})
    bb.event(t0 + 3.0, "stopped")
    bb.close()
    return bb, t0


def test_reason_kind_ignores_numbers():
    assert reason_kind("returning (estimate 1.2 s old, +0.10 m)") == reason_kind("returning (estimate 3.4 s old, -0.20 m)")


def test_session_files_and_events(tmp_path):
    bb, t0 = _run_scenario(tmp_path)
    meta = json.loads((bb.dir / "meta.json").read_text())
    assert meta["argv"] == ["control.main", "--waypoints"] and meta["robots"] == [1]
    events = [json.loads(line) for line in (bb.dir / "events.jsonl").read_text().splitlines()]
    kinds = [e["event"] for e in events]
    assert kinds.count("view_changed") == 1 and kinds.count("mode") == 1
    assert kinds.count("robot_lost") == 1 and kinds.count("view_return") == 1
    # the reason changed twice in kind (following -> returning), not on every changing number
    assert [e["reason"].split(" (")[0] for e in events if e["event"] == "reason"] == ["following", "returning to camera view"]
    mark = [e for e in events if e["event"] == "mark"]
    assert len(mark) == 1 and mark[0]["note"] == "R1 went off screen"  # the old seq 3 was not replayed
    lost = next(e for e in events if e["event"] == "robot_lost")
    assert lost["t"] == pytest.approx(t0 + 1.0) and lost["last"][0] == pytest.approx(-0.69)
    ticks = list(reader.read_ticks(bb.dir))
    assert len(ticks) == 60 and ticks[30]["reckon"]["1"]["since"] > 0 and ticks[30]["guard"] == {"1": "returning"}


def test_reader_commands(tmp_path, capsys):
    bb, _ = _run_scenario(tmp_path)
    base = str(tmp_path)
    reader.main(["--dir", base, "list"])
    reader.main(["--dir", base, "summary"])
    out = capsys.readouterr().out
    assert bb.dir.name in out and "lost 1x, view-guard returns 1x" in out and "R1 went off screen" in out
    assert "closest to the camera-view edge while seen: +0.22 m at" in out  # (-0.69, 1) vs the line y = -x
    assert re.search(r"head aimed by, while driving: head camera 7\d%, main camera 2\d%", out)
    reader.main(["--dir", base, "events", "--kind", "robot_lost,mark"])
    out = capsys.readouterr().out
    assert "robot_lost" in out and "mark" in out and "reason" not in out
    reader.main(["--dir", base, "window", "--at", "mark", "--before", "0.1", "--after", "0"])
    out = capsys.readouterr().out
    assert "R1 unseen" in out and "guard=returning" in out and "est (" in out


def test_truncated_segment_is_still_readable(tmp_path):
    bb, _ = _run_scenario(tmp_path)
    seg = bb.dir / "ticks-000.jsonl.gz"
    data = seg.read_bytes()
    seg.write_bytes(data[: len(data) * 2 // 3])  # as after a crash mid-write
    ticks = list(reader.read_ticks(bb.dir))
    assert 0 < len(ticks) < 60


def test_old_sessions_are_pruned(tmp_path):
    old = tmp_path / "20200101_000000"
    old.mkdir()
    (old / "meta.json").write_text("{}")
    (old / "ticks-000.jsonl.gz").write_bytes(b"x" * 2_000_000)
    bb = BlackBox({"dir": str(tmp_path), "max_total_mb": 1})
    bb.close()
    assert not old.exists() and bb.dir.exists()


def test_snapshot_from_mjpeg(tmp_path):
    jpeg = b"\xff\xd8 fake jpeg body \xff\xd9"

    class MJPEG(BaseHTTPRequestHandler):
        def log_message(self, *a):
            pass

        def do_GET(self):
            self.send_response(200)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.end_headers()
            self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + jpeg + b"\r\n--frame\r\n")

    server = ThreadingHTTPServer(("127.0.0.1", 0), MJPEG)
    threading.Thread(target=server.serve_forever, daemon=True).start()
    try:
        url = f"http://127.0.0.1:{server.server_port}/video/birdseye"
        assert grab_jpeg(url) == jpeg
        bb, _ = _run_scenario(tmp_path, snapshot_url=url)
        for _ in range(50):  # snapshot threads finish shortly after
            snaps = sorted((bb.dir / "snapshots").glob("*.jpg"))
            if len(snaps) >= 3:
                break
            threading.Event().wait(0.05)
        names = " ".join(p.name for p in snaps)
        assert "robot_lost_r1" in names and "view_return_r1" in names and "mark" in names
        assert snaps[0].read_bytes() == jpeg
    finally:
        server.shutdown()
        server.server_close()
    assert grab_jpeg("http://127.0.0.1:9/none", timeout=0.2) is None


def test_head_states_poll_the_dashboard():
    from control.blackbox import HeadStates

    class Reply:
        def __init__(self, body):
            self.body = body

        def __enter__(self):
            return self

        def __exit__(self, *exc):
            return False

        def read(self, *a):
            return self.body

    def urlopen(url, timeout):
        if "robot2" in url:
            raise OSError("unreachable")
        return Reply(json.dumps({"positions": [400, 700], "camera": {"exposure": {"exposure_ms": 9.9}},
                                 "tracking": {"active": True, "mode": "predicted", "source": "prediction",
                                              "faceAge": 0.3, "config": {"big": 1}}}).encode())

    hs = HeadStates("http://x", [1, 2], interval_s=0.01, urlopen=urlopen)
    import time
    deadline = time.time() + 2
    while len(hs.snapshot()) < 2 and time.time() < deadline:
        time.sleep(0.01)
    hs.close()
    snap = hs.snapshot()
    assert snap["1"] == {"mode": "predicted", "source": "prediction", "faceAge": 0.3, "active": True,
                         "positions": [400, 700], "exposure": {"exposure_ms": 9.9}}
    assert snap["2"] is None
