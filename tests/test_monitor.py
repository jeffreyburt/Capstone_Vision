"""System GUI: health parsing, drive server status query, heartbeats, status assessment, probes on localhost."""

import json
import socket
import threading
import time
import shlex
import subprocess
import urllib.request
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import pytest

from monitor import assess
from monitor.beacon import StatusBeacon, StatusListener
from monitor.probes import decode_throttled, drive_status, gateway_probe, http_json, parse_health, parse_process, tcp_check
from monitor.server import CameraRelay, Node, SystemMonitor, build_nodes, make_server
from robot.drive_server import DriveParams, DriveServer, PrintMotors
from vision.config import load_config

MASTER_OUT = """hostname=vision
model=Raspberry Pi 4 Model B Rev 1.5
os=Debian GNU/Linux 13 (trixie)
uptime=7322.51
load=0.52 0.40 0.31
ncpu=4
temp_mc=54530
throttled=0x50000
mem_total_kb=3884000
mem_avail_kb=2913000
disk_used_pct=41
ip_eth0=192.168.1.20/24
ip_wlan0=192.168.50.1/24
wifi_mode=AP
wifi_ssid=capstone-robots
wifi_channel=6
station=dc:a6:32:00:00:12 -48
station=e4:5f:01:02:10:80
neigh=192.168.50.12 DC:A6:32:00:00:12 REACHABLE
neigh=192.168.50.11  FAILED
proc=1234   3600 /home/vision/venv/bin/python -m camstream.sender birdseye
time=1000.250
"""

ROBOT_OUT = """hostname=raspberrypi
temp_mc=81200
throttled=0x1
load=4.5 2.0 1.0
ncpu=4
wifi_mode=managed
wifi_ssid=capstone-robots
wifi_signal=-71
proc=900 120 /usr/bin/python3 -u /home/capstone-pi2/Capstone_Vision/head_tracker/dashboard.py --pose-host 192.168.1.10
proc=950 30 .venv/bin/python /home/capstone-pi2/drive_server.py --robot-id 2 --hardware
time=999.0
"""


def test_parse_master_health():
    h = parse_health(MASTER_OUT, returned_at=1000.0)
    assert h["temp_c"] == pytest.approx(54.5)
    assert h["load"] == [0.52, 0.40, 0.31] and h["ncpu"] == 4
    assert h["mem_used_pct"] == pytest.approx(25.0, abs=0.1)
    assert h["throttled"] == {"raw": "0x50000", "now": [], "since_boot": ["under-voltage", "throttled"]}
    assert h["ips"] == {"eth0": "192.168.1.20", "wlan0": "192.168.50.1"}
    assert h["wifi"]["mode"] == "AP" and h["wifi"]["channel"] == 6
    assert h["stations"] == [{"mac": "dc:a6:32:00:00:12", "ip": "192.168.50.12", "signal_dbm": -48.0},
                             {"mac": "e4:5f:01:02:10:80", "ip": None, "signal_dbm": None}]  # no signal reported
    assert h["clock_offset_s"] == pytest.approx(0.25)
    assert [p["detail"] for p in h["processes"]] == ["camera sender birdseye"]


def test_parse_process_roles():
    assert parse_process("5 10 python -m camstream.sender robot1 --source test")["detail"] == \
        "camera sender robot1 (test pattern)"
    assert parse_process("5 10 python ~/drive_server.py --robot-id 1")["detail"] == "drive server, robot 1 (simulation)"
    assert "head lock" in parse_process("5 10 python drive_with_head_lock.py --neutral 500 500 -- --robot-id 1")["detail"]
    assert parse_process("5 10 python something_else.py") is None


def test_decode_throttled():
    assert decode_throttled("0x50005") == {"raw": "0x50005", "now": ["under-voltage", "throttled"],
                                           "since_boot": ["under-voltage", "throttled"]}
    assert decode_throttled("") is None and decode_throttled("junk") is None


# ---- drive server status query ----

def make_drive_server(**kw):
    params = DriveParams([1, 3, 2, 4], [1, -1, -1, 1])
    return DriveServer(PrintMotors(), params, robot_id=2, port=0, host="127.0.0.1", log=lambda s: None, **kw)


def test_status_query_changes_nothing():
    srv = make_drive_server(allow_from="192.168.1.10")
    try:
        srv.handle(json.dumps({"type": "status"}).encode(), ("127.0.0.1", 9), 100.0)
        assert srv.received == 0 and srv.rejected == 0 and srv.error is None and srv.last_cmd is None
        assert srv.state == "stopped"
    finally:
        srv.sock.close()


def test_drive_status_probe_round_trip():
    srv = make_drive_server()
    srv.sock.settimeout(2.0)

    def serve_one():
        data, addr = srv.sock.recvfrom(4096)
        srv.handle(data, addr, srv.clock())

    t = threading.Thread(target=serve_one)
    t.start()
    try:
        got = drive_status("127.0.0.1", srv.port, timeout=2.0)
        t.join(2)
        rep = got["reply"]
        assert got["error"] is None and rep["robot_id"] == 2 and rep["hardware"] is False
        assert rep["state"] == "stopped" and rep["last_cmd_age_s"] is None and rep["received"] == 0
    finally:
        srv.sock.close()


# ---- heartbeats ----

def test_beacon_reaches_listener_and_reports_exit():
    listener = StatusListener(port=0)
    beacon = StatusBeacon("vision", listener.port, period_s=0.05, extra=lambda: {"live": 1})
    try:
        beacon.update({"state": "running", "fps": 9.5, "source": "not the program name"})  # can't rename it
        deadline = time.time() + 2
        while time.time() < deadline and listener.get("vision")[0] is None:
            time.sleep(0.02)
        msg, age = listener.get("vision")
        assert msg["fps"] == 9.5 and msg["live"] == 1 and age < 1
        assert assess.heartbeat(msg, age) is not None
        beacon.close()
        deadline = time.time() + 2
        while time.time() < deadline and listener.get("vision")[0].get("state") != "exited":
            time.sleep(0.02)
        assert assess.heartbeat(*listener.get("vision")) is None
    finally:
        listener.close()


# ---- assessment ----

ROBOT = Node("robot2", "Robot 2", "robot", "192.168.50.12", "capstone-pi2", robot_id=2, drive_port=5700,
             dashboard_port=8080, streams={"robot2": 5602})
MASTER = Node("master", "Master Pi", "master", "192.168.1.20", "vision", wifi_host="192.168.50.1",
              streams={"birdseye": 5600})
UP = {"up": True, "open": True, "ms": 3.0, "error": None}
DOWN = {"up": False, "open": False, "ms": None, "error": "timeout"}


def levels(node):
    return {c["label"]: c["level"] for c in node["checks"]}


def text(node, label):
    return next(c["text"] for c in node["checks"] if c["label"] == label)


def test_offline_robot_is_bad():
    out = assess.assess_robot(ROBOT, {"ping": DOWN}, {}, None, None, None, None)
    assert out["level"] == assess.BAD and out["summary"] == "offline"


def test_robot_with_hot_undervolted_pi_and_wrong_id():
    raw = {"ping": UP, "health": parse_health(ROBOT_OUT, 1000.0), "ports": {"robot2": {**UP, "open": False}},
           "drive": {"reply": {"type": "status", "robot_id": 1, "state": "stopped"}, "error": None},
           "dashboard": {"data": None, "error": "not running"}}
    out = assess.assess_robot(ROBOT, raw, {}, None, None, None, None)
    lv = levels(out)
    assert lv["System"] == assess.BAD          # 81 C and load above the core count
    assert lv["Power"] == assess.BAD           # under-voltage now
    assert lv["WiFi"] == assess.WARN           # -71 dBm
    assert lv["Clock"] == assess.WARN          # 1 s behind
    assert lv["Drive server"] == assess.BAD    # answers as robot 1
    assert lv["Head tracker"] == assess.OFF
    assert out["level"] == assess.BAD


def test_old_drive_server_and_armed_control():
    old = {"type": "status", "robot_id": 2, "state": "stopped", "motors": [0, 0, 0, 0],
           "error": "unknown command type 'status'"}
    raw = {"ping": UP, "drive": {"reply": old, "error": None}}
    control = {"state": "running", "mode": "ARMED",
               "robots": {"2": {"target": [1.0, 2.0, 90.0], "cmd": [0.1, 0, 0], "reason": "moving", "link_age_s": None}}}
    vision = {"state": "running", "message": {"robots": [{"id": 2, "x": 0.5, "y": 2.0, "heading": 10.0, "age": 0.1}]}}
    out = assess.assess_robot(ROBOT, raw, {}, vision, 0.1, control, 0.1)
    lv = levels(out)
    assert lv["Drive server"] == assess.WARN and "update robot/drive_server.py" in text(out, "Drive server")
    assert "unknown command type 'status'" in text(out, "Drive server")
    assert lv["Seen by vision"] == assess.OK
    assert lv["Control"] == assess.BAD         # armed, but no reply from the drive server
    assert "drive on" in out["summary"] and "seen" in out["summary"]


def test_drive_server_down_only_matters_when_armed():
    raw = {"ping": UP, "drive": {"reply": None, "error": "not running"}}
    assert levels(assess.assess_robot(ROBOT, raw, {}, None, None, None, None))["Drive server"] == assess.OFF
    control = {"mode": "ARMED", "robots": {"2": {"reason": "moving", "cmd": [0, 0, 0], "link_age_s": None}}}
    assert levels(assess.assess_robot(ROBOT, raw, {}, None, None, control, 0.1))["Drive server"] == assess.BAD


def test_master_access_point_and_route():
    raw = {"ping": UP, "ping_wifi": DOWN, "health": parse_health(MASTER_OUT, 1000.0),
           "ports": {"birdseye": UP}}
    vision = {"state": "running", "camera": "stream://birdseye", "gui_port": 8080,
              "stream": {"fps": 4.1, "mbps": 80.0, "kb_per_frame": 2400.0, "dropped": 3, "since_last_s": 0.2}}
    out = assess.assess_master(MASTER, raw, vision, 0.1, {"192.168.50.12": "Robot 2"})
    lv = levels(out)
    assert lv["Route to robots"] == assess.BAD
    assert lv["Power"] == assess.WARN          # under-voltage earlier, not now
    assert lv["Camera sender"] == assess.OK and "up 60 min" in text(out, "Camera sender")
    assert lv["Frames to vision"] == assess.OK
    wifi = next(c for c in out["checks"] if c["label"] == "Robot WiFi")
    assert "Robot 2 -48 dBm" in wifi["text"]
    assert out["camera"]["path"] == "/video/birdseye" and out["camera"]["port"] == 8080


def test_laptop_levels():
    out = assess.assess_laptop(None, None, None, None, now=100.0)
    assert out["level"] == assess.OFF and out["summary"] == "nothing running"
    vision = {"state": "running", "fps": 9.0, "camera": "stream://birdseye", "last_frame_t": 99.9, "markers": [4, 5],
              "calibrated": False, "people": True, "message": {"robots": [], "user": None}}
    out = assess.assess_laptop(vision, 0.1, None, None, now=100.0)
    assert levels(out)["Floor calibration"] == assess.BAD and out["level"] == assess.BAD
    assert levels(out)["Person detection"] == assess.WARN


def test_worst_ordering():
    rows = lambda *lv: [assess.check("x", v, "") for v in lv]  # noqa: E731
    assert assess.worst(rows(assess.OK, assess.OFF)) == assess.OK
    assert assess.worst(rows(assess.OFF, assess.UNKNOWN)) == assess.UNKNOWN
    assert assess.worst(rows(assess.OK, assess.WARN, assess.UNKNOWN)) == assess.WARN
    assert assess.worst([]) == assess.UNKNOWN


# ---- nodes from config, and the whole monitor against services on localhost ----

def test_build_nodes_from_repo_config():
    nodes = {n.key: n for n in build_nodes(load_config())}
    assert nodes["master"].host == "192.168.1.20" and nodes["master"].ssh_user == "vision"
    assert nodes["master"].streams == {"birdseye": 5600}
    assert (nodes["robot1"].host, nodes["robot1"].ssh_user) == ("192.168.50.105", "robot-pi1")
    r2 = nodes["robot2"]
    assert (r2.host, r2.drive_port, r2.ssh_user, r2.streams) == ("192.168.50.12", 5700, "capstone-pi2", {"robot2": 5602})


class FakeDashboard(BaseHTTPRequestHandler):
    def log_message(self, *args):
        pass

    def do_GET(self):
        body = json.dumps({"status": "Ready", "positions": [500, 498], "torque": [1, 1],
                           "tracking": {"active": False}, "camera": {"live": True, "model": "imx708"}}).encode()
        self.send_response(200)
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)


def test_monitor_probes_local_services():
    ssh_port = socket.socket()          # stands in for the Pi's SSH server (only connected to, never spoken to)
    ssh_port.bind(("127.0.0.1", 0))
    ssh_port.listen()
    dash = ThreadingHTTPServer(("127.0.0.1", 0), FakeDashboard)
    threading.Thread(target=dash.serve_forever, daemon=True).start()
    srv = make_drive_server()
    srv.sock.settimeout(0.2)
    stop = threading.Event()

    def serve():
        while not stop.is_set():
            try:
                data, addr = srv.sock.recvfrom(4096)
            except (socket.timeout, OSError):
                continue
            srv.handle(data, addr, srv.clock())

    t = threading.Thread(target=serve, daemon=True)
    t.start()
    cfg = {"system": {"master": {"host": "127.0.0.1", "ssh_user": "vision", "ssh_port": ssh_port.getsockname()[1]},
                      "robots": {2: {"ssh_user": "pi", "ssh_port": ssh_port.getsockname()[1],
                                     "dashboard_port": dash.server_address[1]}}},
           "control": {"robots": {2: {"host": "127.0.0.1", "port": srv.port}}}}
    fake_ssh = lambda ssh, user, host, port: (ROBOT_OUT if user == "pi" else MASTER_OUT, None, 1000.0)  # noqa: E731
    mon = SystemMonitor(cfg, ssh_runner=fake_ssh, listener=False, start=False)
    try:
        assert tcp_check("127.0.0.1", ssh_port.getsockname()[1])["open"]
        assert http_json(f"http://127.0.0.1:{dash.server_address[1]}/api/state")["data"]["status"] == "Ready"
        for n in mon.nodes:
            mon.probe_once(n)
            mon.ssh_once(n)
        st = mon.snapshot()
        robot = next(n for n in st["nodes"] if n["key"] == "robot2")
        lv = levels(robot)
        assert lv["Network"] == assess.OK and lv["Drive server"] == assess.OK
        assert "simulation" in text(robot, "Drive server")
        assert lv["Head tracker"] == assess.OK and lv["Camera"] == assess.OK
        assert robot["camera"]["url"].endswith("/camera.mjpg")
        assert [link["to"] for link in st["links"]] == ["master", "robot2"]
        json.dumps(st)  # the page gets it as JSON
    finally:
        mon.close()
        stop.set()
        t.join(1)
        srv.sock.close()
        dash.shutdown()
        ssh_port.close()


def test_gateway_probe_runs_same_read_only_checks_and_uses_laptop_time(monkeypatch):
    import monitor.probes as probes
    real_run = subprocess.run
    calls = []

    def local_gateway(cmd, **kwargs):
        calls.append(cmd)
        # Execute exactly the Python program SSH would run, against a local fake service.
        return real_run(shlex.split(cmd[-1]), **kwargs)

    monkeypatch.setattr(probes.subprocess, "run", local_gateway)
    dash = ThreadingHTTPServer(("127.0.0.1", 0), FakeDashboard)
    threading.Thread(target=dash.serve_forever, daemon=True).start()
    try:
        before = time.time()
        result = gateway_probe("ssh", "vision@main", dict(host="127.0.0.1",
                               ssh_port=dash.server_port, dashboard_port=dash.server_port))
        assert result["ping"]["open"]
        assert result["dashboard"]["data"]["positions"] == [500, 498]
        assert before <= result["at"] <= time.time()
        assert result["probe_via"] == "vision@main"
        assert "BatchMode=yes" in calls[0]
    finally:
        dash.shutdown()


def test_gateway_auth_failure_clears_stale_results(monkeypatch):
    import monitor.probes as probes
    monkeypatch.setattr(probes.subprocess, "run", lambda *a, **kw:
                        subprocess.CompletedProcess(a[0], 255, b"", b"Permission denied (publickey)."))
    result = gateway_probe("ssh", "vision@main", {"host": "robot"})
    assert not result["ping"]["up"] and not result["ping_wifi"]["up"]
    assert "SSH key login refused" in result["ping"]["error"]
    assert result["dashboard"] is None and result["health"] is None


def test_camera_failure_explains_hardware_status():
    raw = {"ping": UP, "dashboard": {"data": {"camera": {
        "live": False, "status": "Camera stopped delivering frames"}}, "error": None}}
    out = assess.assess_robot(ROBOT, raw, {}, None, None, None, None)
    assert levels(out)["Camera"] == assess.WARN
    assert text(out, "Camera") == "Camera stopped delivering frames"


def test_gateway_camera_uses_local_proxy_and_preserves_robot_address(monkeypatch):
    import monitor.server as server
    cfg = {"system": {"probe_via": "vision@main", "robots": {1: {"host": "robot", "ssh_user": "pi"}}}}
    result = {"at": time.time(), "probe_via": "vision@main", "ping": UP,
              "dashboard": {"data": {"positions": [10, 20], "camera": {"live": True}}, "error": None}}
    monkeypatch.setattr(server, "gateway_probe", lambda *a: result)
    mon = SystemMonitor(cfg, listener=False, start=False)
    try:
        mon.probe_once(mon.nodes[0])
        robot = next(n for n in mon.snapshot()["nodes"] if n["key"] == "robot1")
        assert robot["address"] == "robot"
        assert robot["camera"]["url"] == "/camera/robot1"
    finally:
        mon.close()


def test_camera_proxy_relays_jpeg_stream_without_control_requests():
    frame = b"--frame\r\nContent-Type: image/jpeg\r\n\r\njpeg-test\r\n"
    requests = []

    class JPEG(BaseHTTPRequestHandler):
        def log_message(self, *args):
            pass

        def do_GET(self):
            requests.append((self.command, self.path))
            self.send_response(200)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.send_header("Content-Length", str(len(frame)))
            self.end_headers()
            self.wfile.write(frame)

    upstream = ThreadingHTTPServer(("127.0.0.1", 0), JPEG)
    threading.Thread(target=upstream.serve_forever, daemon=True).start()
    cfg = {"system": {"robots": {1: {"host": "127.0.0.1", "dashboard_port": upstream.server_port}}}}
    mon = SystemMonitor(cfg, listener=False, start=False)
    relay = CameraRelay(cfg)
    httpd = make_server(mon, relay, "127.0.0.1", 0)
    threading.Thread(target=httpd.serve_forever, daemon=True).start()
    try:
        with urllib.request.urlopen(f"http://127.0.0.1:{httpd.server_port}/camera/robot1", timeout=2) as r:
            assert "multipart/x-mixed-replace" in r.headers["Content-Type"]
            assert r.read() == frame
        assert requests == [("GET", "/camera.mjpg")]
    finally:
        httpd.shutdown()
        httpd.server_close()
        upstream.shutdown()
        upstream.server_close()
        relay.tunnels.close()
        mon.close()
