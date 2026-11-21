"""System dashboard relay to the robot head-tracker dashboards (no hardware)."""
import json
import threading
import urllib.error
import urllib.request
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import pytest

from monitor.heads import enabled_robots
from monitor.server import CameraRelay, SystemMonitor, make_server

SAVED = {"prediction": 0.8, "limits": [[0, 1000], [433, 812]], "signs": [-1, 1], "speed": 60,
         "deadzone": 0.06, "settlezone": 0.04}


class FakeRobotDashboard:
    """Stands in for head_tracker/dashboard.py and records every POST."""
    def __init__(self):
        self.posts = []
        fake = self

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, *args):
                pass

            def do_GET(self):
                if self.path == "/api/state":
                    body = json.dumps({"status": "Ready", "positions": [476, 582], "position_time": 10.0,
                                       "serverTime": 10.5, "torque": [1, 1], "running": False, "samples": [1] * 5,
                                       "camera": {"live": True}, "tracking": {"active": False, "config": SAVED}}).encode()
                    kind, extra = "application/json", {}
                elif self.path == "/api/camera/frame":
                    body, kind, extra = b"\xff\xd8jpeg", "image/jpeg", {"X-Frame-Time": "123.5"}
                else:
                    self.send_error(404)
                    return
                self.send_response(200)
                self.send_header("Content-Type", kind)
                for k, v in extra.items():
                    self.send_header(k, v)
                self.end_headers()
                self.wfile.write(body)

            def do_POST(self):
                size = int(self.headers.get("Content-Length", 0))
                fake.posts.append((self.path, json.loads(self.rfile.read(size)) if size else None,
                                   self.headers.get("Origin")))
                if self.path == "/api/tracking/observation":
                    self.send_error(400, "Camera observation is stale")
                    return
                self.send_response(204)
                self.end_headers()

        self.server = ThreadingHTTPServer(("127.0.0.1", 0), Handler)
        self.thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.thread.start()

    def close(self):
        self.server.shutdown()
        self.server.server_close()
        self.thread.join()


@pytest.fixture
def setup():
    robots = {1: FakeRobotDashboard(), 2: FakeRobotDashboard()}
    cfg = {"control": {"robots": {rid: {"host": "127.0.0.1"} for rid in robots}},
           "system": {"robots": {1: {"dashboard_port": robots[1].server.server_port, "head_tracking": False,
                                     "head_tracking_reason": "servos not connected"},
                                 2: {"dashboard_port": robots[2].server.server_port}}}}
    monitor = SystemMonitor(cfg, listener=False, start=False)
    server = make_server(monitor, CameraRelay(cfg), "127.0.0.1", 0)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    base = f"http://127.0.0.1:{server.server_port}"
    try:
        yield base, robots
    finally:
        server.shutdown()
        server.server_close()
        thread.join()
        for r in robots.values():
            r.close()


def post(base, path, data=None):
    body = json.dumps(data).encode() if data is not None else b""
    request = urllib.request.Request(base + path, data=body, method="POST", headers={"Content-Type": "application/json"})
    try:
        with urllib.request.urlopen(request) as response:
            return response.status, json.load(response)
    except urllib.error.HTTPError as error:
        return error.code, json.load(error)


def get(base, path):
    with urllib.request.urlopen(base + path) as response:
        return response.read(), response.headers


def test_enabled_flags_default_on():
    flags = enabled_robots({"system": {"robots": {1: {"head_tracking": False}, 2: {}}}})
    assert flags["robot1"][0] is False and "disabled" in flags["robot1"][1]
    assert flags["robot2"] == (True, None)


def test_state_is_relayed_without_history(setup):
    base, _ = setup
    body, _ = get(base, "/api/head/robot2/state")
    state = json.loads(body)
    assert state["enabled"] and state["reachable"] and state["positions"] == [476, 582]
    assert state["tracking"]["config"]["speed"] == 60 and "samples" not in state
    disabled = json.loads(get(base, "/api/head/robot1/state")[0])
    assert disabled["enabled"] is False and disabled["disabled_reason"] == "servos not connected"
    assert disabled["positions"] == [476, 582]  # still shown read-only


def test_disabled_robot_refuses_motion_but_accepts_stop(setup):
    base, robots = setup
    for action in ("engage", "start", "observation", "disengage"):
        code, body = post(base, f"/api/head/robot1/{action}", {"center": [.5, .5], "frameTime": 1, "target": "face"})
        assert code == 409 and body["message"] == "servos not connected"
    assert robots[1].posts == []
    assert post(base, "/api/head/robot1/stop")[1]["ok"]
    assert [p[0] for p in robots[1].posts] == ["/api/stop"]
    with pytest.raises(urllib.error.HTTPError) as error:
        get(base, "/api/head/robot1/frame")
    assert error.value.code == 409


def test_start_uses_robot_saved_settings_and_sends_no_origin(setup):
    base, robots = setup
    assert post(base, "/api/head/robot2/engage")[1]["ok"]
    assert post(base, "/api/head/robot2/start")[1]["ok"]
    (engage, _, origin), (start, data, _) = robots[2].posts
    assert engage == "/api/engage" and origin is None
    assert start == "/api/tracking/start"
    assert data == {"target": "face", "limits": [[0, 1000], [433, 812]], "signs": [-1, 1], "speed": 60, "prediction": 0.8}


def test_robot_errors_and_frames_pass_through(setup):
    base, _ = setup
    code, body = post(base, "/api/head/robot2/observation", {"center": None, "frameTime": 1, "target": "face"})
    assert code == 400 and body["message"] == "Camera observation is stale"
    frame, headers = get(base, "/api/head/robot2/frame")
    assert frame == b"\xff\xd8jpeg" and headers["X-Frame-Time"] == "123.5"
    assert post(base, "/api/head/robot2/reboot")[0] == 404


def test_cross_origin_head_post_rejected(setup):
    base, robots = setup
    request = urllib.request.Request(base + "/api/head/robot2/engage", data=b"", method="POST",
                                     headers={"Origin": "http://evil.example"})
    with pytest.raises(urllib.error.HTTPError) as error:
        urllib.request.urlopen(request)
    assert error.value.code == 403 and robots[2].posts == []


def test_detector_assets_served_without_traversal(setup):
    base, _ = setup
    body, headers = get(base, "/head-assets/vision_bundle.mjs")
    assert headers["Content-Type"] == "text/javascript" and len(body) > 1000
    with pytest.raises(urllib.error.HTTPError):
        get(base, "/head-assets/../../config.yaml")
