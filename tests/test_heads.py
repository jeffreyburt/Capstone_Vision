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


def test_disabled_robot_refuses_tracking_but_allows_stop_and_torque(setup):
    base, robots = setup
    for action in ("start", "observation"):
        code, body = post(base, f"/api/head/robot1/{action}", {"center": [.5, .5], "frameTime": 1, "target": "face"})
        assert code == 409 and body["message"] == "servos not connected"
    assert robots[1].posts == []
    assert post(base, "/api/head/robot1/stop")[1]["ok"]
    assert post(base, "/api/head/robot1/disengage")[1]["ok"]
    assert [p[0] for p in robots[1].posts] == ["/api/stop", "/api/disengage"]
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
    assert data == {"target": "face", "limits": [[0, 1000], [433, 812]], "signs": [-1, 1], "speed": 60, "prediction": 0.8,
                    "units_per_frame": [170.0, 117.0],
                    "aim_offset_m": 0.0, "face_width_m": 0.15}
    assert post(base, "/api/head/robot2/start", {"speed": 300, "prediction": 1.5})[1]["ok"]
    assert robots[2].posts[-1][1] == {"target": "face", "limits": [[0, 1000], [433, 812]], "signs": [-1, 1],
                                      "speed": 300, "prediction": 1.5, "units_per_frame": [170.0, 117.0],
                    "aim_offset_m": 0.0, "face_width_m": 0.15}


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


def test_config_tracking_limits_override_saved_limits():
    from monitor.heads import HeadRelay, tracking_limits
    cfg = {"system": {"robots": {2: {"head_tracking_limits": [[311, 471], [690, 785]]}}}}
    assert tracking_limits(cfg) == {"robot2": [[311, 471], [690, 785]]}
    relay = HeadRelay(cfg, lambda h, p: (h, p))
    relay.state = lambda node: {"tracking": {"config": SAVED}}
    node = type("N", (), {"key": "robot2"})()
    start = relay._start_request(node, {"speed": 300})
    assert start["limits"] == [[311, 471], [690, 785]] and start["signs"] == [-1, 1] and start["speed"] == 300
    with pytest.raises(ValueError):
        tracking_limits({"system": {"robots": {2: {"head_tracking_limits": [[500, 400], [0, 10]]}}}})


def test_main_camera_aim_points_the_head_at_the_user():
    from monitor.heads import aim_from_main_camera
    head = {"neutral_units": [391, 770], "degrees_per_unit": [0.271, 0.226],
            "pan_pivot_m": [0, 0, 0], "tilt_pivot_m": [0, 0, 0.32]}
    robot = {"x": 0.0, "y": 1.0, "heading": 90.0}
    # User straight ahead at head height of the camera: no pan, no tilt.
    aim = aim_from_main_camera(head, robot, {"x": 0.0, "y": 3.0, "height": 0.44}, [391, 770], [-1, 1])
    assert aim["units"] == [391, 770] and aim["center"] == pytest.approx([.5, .5])
    # User 20 deg to the robot's left and above: pan up (toward chassis left), tilt negative (up).
    import math
    ux, uy = 2 * math.cos(math.radians(110)), 1 + 2 * math.sin(math.radians(110))
    aim = aim_from_main_camera(head, robot, {"x": ux, "y": uy, "height": 1.6}, [391, 770], [-1, 1])
    assert aim["units"][0] == pytest.approx(391 + 20 / 0.271, abs=0.2)
    assert aim["units"][1] < 770
    assert aim["center"][0] < .5 and aim["center"][1] < .5  # face appears left of and above image centre
    # Clamped to the tracking limits.
    aim = aim_from_main_camera(head, robot, {"x": ux, "y": uy, "height": 1.6}, [391, 770], [-1, 1],
                               limits=[[311, 471], [690, 785]])
    assert aim["units"] == [pytest.approx(464.8, abs=.2), 690]  # 20 deg pan fits; the tilt is clamped


def test_start_without_saved_settings_uses_config_limits_and_signs():
    from monitor.heads import HeadRelay
    cfg = {"system": {"robots": {1: {"head_tracking_limits": [[327, 980], [692, 980]], "head_tracking_signs": [-1, -1],
                                     "head_tracking_units_per_frame": [168, 116]}}}}
    relay = HeadRelay(cfg, lambda h, p: (h, p))
    relay.state = lambda node: {"tracking": {"config": None}}
    node = type("N", (), {"key": "robot1"})()
    start = relay._start_request(node, {"speed": 300, "prediction": 1.5})
    assert start == {"target": "face", "limits": [[327, 980], [692, 980]], "signs": [-1, -1], "speed": 300, "prediction": 1.5,
                     "units_per_frame": [168.0, 116.0], "aim_offset_m": 0.0, "face_width_m": 0.15}
    with pytest.raises(ValueError):
        HeadRelay({"system": {"robots": {1: {"head_tracking_signs": [2, 1]}}}}, lambda h, p: (h, p))


def test_main_camera_aim_points_the_speaker_not_the_camera():
    from monitor.heads import aim_from_main_camera
    head = {"neutral_units": [391, 770], "degrees_per_unit": [0.271, 0.226],
            "pan_pivot_m": [0, 0, 0], "tilt_pivot_m": [0, 0, 0.32]}
    robot, user = {"x": 0.0, "y": 1.0, "heading": 90.0}, {"x": 0.0, "y": 2.0, "height": 1.0}
    camera = aim_from_main_camera(head, robot, user, [391, 770], [-1, 1])
    speaker = aim_from_main_camera(head, robot, user, [391, 770], [-1, 1], speaker_offset_m=0.1016)
    import math
    extra_up = math.degrees(math.atan2(0.88 - 0.2184, 1.0)) - math.degrees(math.atan2(0.88 - 0.32, 1.0))
    assert speaker["units"][1] == pytest.approx(camera["units"][1] - extra_up / 0.226, abs=0.2)  # tilts further up
    assert speaker["units"][0] == camera["units"][0]


def test_home_moves_each_head_to_its_calibrated_forward_pose_inside_the_limits():
    robot = FakeRobotDashboard()
    cfg = {"control": {"robots": {1: {"host": "127.0.0.1"}, 3: {"host": "127.0.0.1"}}},
           "robots": [{"id": 1, "head_pose": {"neutral_units": [688, 687], "degrees_per_unit": [0.255, -0.241]}}],
           "system": {"robots": {1: {"dashboard_port": robot.server.server_port,
                                     "head_tracking_limits": [[327, 980], [692, 980]]},
                                 3: {"dashboard_port": robot.server.server_port}}}}
    monitor = SystemMonitor(cfg, listener=False, start=False)
    server = make_server(monitor, CameraRelay(cfg), "127.0.0.1", 0)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    base = f"http://127.0.0.1:{server.server_port}"
    try:
        assert post(base, "/api/head/robot1/home")[1]["ok"]
        assert robot.posts[-1][:2] == ("/api/home", {"positions": [688, 692], "duration": 1.5})  # tilt kept in limits
        code, body = post(base, "/api/head/robot3/home")  # no calibrated forward pose: refused, nothing sent
        assert code == 400 and "Calibrate robots" in body["message"] and len(robot.posts) == 1
    finally:
        server.shutdown()
        server.server_close()
        thread.join()
        robot.close()
