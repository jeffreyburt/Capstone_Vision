"""System GUI: one page with the status of the laptop programs, the master Pi and every robot.

    python -m monitor                  # then open http://localhost:8090/
    python -m monitor --no-ssh         # skip the SSH health checks (temperature, power, WiFi, processes)
    python -m monitor --bind 0.0.0.0   # also reachable from other machines (e.g. a phone)

Runs on its own, so it keeps working when vision.main or control.main are stopped or crash. Status
probes are read-only. Grid clicks store waypoints for a separate controller; motor diagnostics explicitly pause navigation and temporarily take exclusive wheel control. Where the information comes from:

    laptop programs   heartbeats from vision.main and control.main (UDP 127.0.0.1:5007, monitor/beacon.py)
    every Pi          TCP connect to SSH (is it up?), SSH health script with the laptop's key (monitor/probes.py)
    master Pi         camera sender port, robot WiFi access point and its clients
    robots            drive server status query (UDP 5700), head tracker dashboard (HTTP /api/state),
                      camera sender port, pose from vision, command and target from control
Settings: config.yaml `system:`.

HTTP:
    /              the page (monitor/page.html)
    /state         JSON: every machine's status rows, the vision message, control targets, map settings
    /video/<name>  a robot's camstream as MJPEG (subscribes only while someone is watching)
"""

import argparse
import copy
import json
import os
import signal
import subprocess
import sys
import tempfile
import threading
import time
import urllib.request
from dataclasses import dataclass, field
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

from vision.config import load_config

from . import assess
from .beacon import StatusListener, status_port
from .calibration import CalibrationManager
from .probes import find_ssh, gateway_probe, network_probe, parse_health, run_ssh, tcp_check
from .tunnels import CameraTunnels
from .motors import MotorTests
from control.waypoints import WaypointStore, waypoint_feedback
from control.manual import ManualControl

PAGE = Path(__file__).with_name("page.html")
REPO = Path(__file__).resolve().parent.parent
CALIBRATION_PAGE = Path(__file__).with_name("calibration.html")


def main_camera_view(message, heartbeat_age, now):
    """Use laptop frame-receipt time, never the unsynchronized Pi capture clock."""
    vision = assess.heartbeat(message, heartbeat_age)
    view = {"live": False, "url": None, "status": "Vision not running; start python -m vision.main --gui",
            "markers": [], "robots": [], "people_count": None, "user": None, "fps": None,
            "frame_age_s": None, "calibrated": False, "detection_error": None}
    if vision is None:
        return view
    if vision.get("camera") != "stream://birdseye":
        view["status"] = "Vision is using a different source; select stream://birdseye for the main camera"
        return view
    last_frame = vision.get("last_frame_t")
    age = max(0, now - last_frame) if last_frame is not None else None
    view["frame_age_s"] = age
    if age is None or age > 3:
        view["status"] = "Waiting for main camera frames" if age is None else f"Main camera stalled; last frame {age:.1f} s ago"
        return view  # do not present old targets or an old JPEG as live
    if not vision.get("gui_port"):
        view["status"] = "Camera processing is running; restart vision with --gui to show overlays"
        return view
    msg = vision.get("message") or {}
    view.update(live=True, url="/video/birdseye", status="Live", markers=vision.get("markers", []),
                robots=[r for r in msg.get("robots", []) if r.get("age", 0) <= 1],
                user=msg.get("user") if (msg.get("user") or {}).get("age", 0) <= 1 else None,
                people_count=vision.get("people_count"), fps=vision.get("fps"),
                calibrated=bool(vision.get("calibrated")), detection_error=vision.get("detection_error"))
    view["position_quality"] = msg.get("position_quality")
    view["approximation"] = msg.get("approximation")
    return view


@dataclass
class Node:
    key: str                     # "master", "robot1", ...
    name: str                    # shown in the GUI
    kind: str                    # "master" | "robot"
    host: str                    # address used for SSH and the reachability check
    ssh_user: str = None
    ssh_port: int = 22
    wifi_host: str = None        # master only: its robot-WiFi address (checks the laptop's route)
    robot_id: int = None
    drive_port: int = None
    dashboard_port: int = None
    streams: dict = field(default_factory=dict)   # camstream name -> port
    stream_hosts: dict = field(default_factory=dict)


def _int_keys(d):
    return {int(k): v for k, v in (d or {}).items()}


def build_nodes(cfg):
    """The Pis to watch, from config `system:` plus the addresses already in `streams:` and `control.robots`."""
    sysc = cfg.get("system") or {}
    streams = cfg.get("streams") or {}
    override = os.environ.get("CAMSTREAM_HOST")  # same testing override as camstream.receiver

    def stream_entry(name):
        s = streams.get(name)
        return ({name: int(s["port"])}, {name: override or s["host"]}) if s else ({}, {})

    m = sysc.get("master") or {}
    ports, hosts = stream_entry("birdseye")
    master = Node("master", m.get("name", "Master Pi"), "master", m.get("host") or hosts.get("birdseye"),
                  m.get("ssh_user"), int(m.get("ssh_port", 22)), m.get("wifi_host"), streams=ports, stream_hosts=hosts)
    nodes = [master] if master.host else []

    control = _int_keys((cfg.get("control") or {}).get("robots"))
    extra = _int_keys(sysc.get("robots"))
    for rid in sorted(set(control) | set(extra)):
        c, s = control.get(rid) or {}, extra.get(rid) or {}
        ports, hosts = stream_entry(f"robot{rid}")
        host = s.get("host") or c.get("host") or hosts.get(f"robot{rid}")
        if not host:
            continue
        nodes.append(Node(f"robot{rid}", s.get("name", f"Robot {rid}"), "robot", host, s.get("ssh_user"),
                          int(s.get("ssh_port", 22)), robot_id=rid, drive_port=int(c.get("port", 5700)),
                          dashboard_port=s.get("dashboard_port", 8080), streams=ports, stream_hosts=hosts))
    return nodes


class SystemMonitor:
    """Runs the probes in background threads and assembles the GUI state."""

    def __init__(self, cfg, *, ssh=True, ssh_runner=run_ssh, listener=True, start=True):
        self.cfg = cfg
        self.waypoints = WaypointStore(_int_keys((cfg.get('control') or {}).get('robots')), cfg.get('area'))
        self.manual = ManualControl(self.waypoints.robot_ids)
        sysc = cfg.get("system") or {}
        self.nodes = build_nodes(cfg)
        self.probe_interval = float(sysc.get("probe_interval_s", 2.0))
        self.ssh_interval = float(sysc.get("ssh_interval_s", 10.0)) if ssh else 0.0
        self.ssh = find_ssh(sysc.get("ssh"))
        self.probe_via = sysc.get("probe_via")
        self.ssh_runner = ssh_runner
        self.raw = {n.key: {"ssh_enabled": self.ssh_interval > 0 and bool(n.ssh_user)} for n in self.nodes}
        self.lock = threading.Lock()
        self.listener = StatusListener(status_port(cfg)) if listener else None
        self._stop = threading.Event()
        self.threads = []
        if start:
            for n in self.nodes:
                self._spawn(self._probe_loop, n)
                if self.ssh_interval > 0 and n.ssh_user:
                    self._spawn(self._ssh_loop, n)

    def _spawn(self, fn, node):
        t = threading.Thread(target=fn, args=(node,), daemon=True, name=f"{fn.__name__}-{node.key}")
        t.start()
        self.threads.append(t)

    # ---- probes ----
    def probe_once(self, node):
        spec = dict(host=node.host, ssh_port=node.ssh_port, wifi_host=node.wifi_host,
                    streams={name: {"host": node.stream_hosts[name], "port": port}
                             for name, port in node.streams.items()},
                    drive_port=node.drive_port, dashboard_port=node.dashboard_port)
        r = gateway_probe(self.ssh, self.probe_via, spec) if self.probe_via else network_probe(**spec)
        if self.probe_via and node.kind == "master":
            r["ping"] = tcp_check(node.host, node.ssh_port)  # actual laptop-to-master Ethernet check
        with self.lock:
            self.raw[node.key].update(r)

    def ssh_once(self, node):
        with self.lock:
            ping = self.raw[node.key].get("ping")
        if ping is not None and not ping["up"]:
            return  # offline: no point waiting for an SSH timeout
        out, err, returned = self.ssh_runner(self.ssh, node.ssh_user, node.host, node.ssh_port)
        health = None if err else parse_health(out, returned)
        with self.lock:
            self.raw[node.key].update({"health": health, "health_error": err, "health_at": time.time()})

    def _probe_loop(self, node):
        while not self._stop.is_set():
            t0 = time.time()
            self.probe_once(node)
            self._stop.wait(max(0.2, self.probe_interval - (time.time() - t0)))

    def _ssh_loop(self, node):
        self._stop.wait(0.5)  # let the first reachability check finish
        while not self._stop.is_set():
            t0 = time.time()
            self.ssh_once(node)
            self._stop.wait(max(1.0, self.ssh_interval - (time.time() - t0)))

    # ---- state for the page ----
    def heartbeats(self):
        if self.listener is None:
            return (None, None), (None, None)
        return self.listener.get("vision"), self.listener.get("control")

    def snapshot(self, now=None):
        now = time.time() if now is None else now
        (vmsg, vage), (cmsg, cage) = self.heartbeats()
        with self.lock:
            raw = {k: dict(v) for k, v in self.raw.items()}
        robot_names = {n.host: n.name for n in self.nodes if n.kind == "robot"}
        master_raw = next((raw[n.key] for n in self.nodes if n.kind == "master"), {})

        out_nodes = [assess.assess_laptop(vmsg, vage, cmsg, cage, now)]
        for n in self.nodes:
            if n.kind == "master":
                out_nodes.append(assess.assess_master(n, raw[n.key], vmsg, vage, robot_names))
            else:
                out_nodes.append(assess.assess_robot(n, raw[n.key], master_raw, vmsg, vage, cmsg, cage))
        if self.probe_via:
            for item in out_nodes:
                camera = item.get("camera")
                if item["kind"] == "robot" and camera and camera.get("url"):
                    camera["url"] = f"/camera/{item['key']}"

        links = []
        master = next((n for n in self.nodes if n.kind == "master"), None)
        if master is not None:
            ping = master_raw.get("ping")
            links.append({"from": "laptop", "to": master.key, "kind": "Ethernet", **_link(ping)})
        for n in self.nodes:
            if n.kind == "robot":
                link = _link(raw[n.key].get("ping"))
                dbm = next(c for c in out_nodes if c["key"] == n.key).get("wifi_dbm")
                if dbm is not None and link["level"] == assess.OK:
                    link["level"], link["text"] = assess.wifi_level(dbm), f"{dbm:.0f} dBm"
                links.append({"from": master.key if master else "laptop", "to": n.key, "kind": "WiFi", **link})

        vision, control = assess.heartbeat(vmsg, vage), assess.heartbeat(cmsg, cage)
        dbg = self.cfg.get("debug") or {}
        robots_cfg = self.cfg.get("robots") or []
        limits = (self.cfg.get("control") or {}).get("limits") or {}
        readiness = {}
        poses = {r['id']: r for r in ((vision or {}).get('message') or {}).get('robots', [])}
        message = (vision or {}).get('message') or {}
        local_clock = message.get('frame_clock') == 'local'
        capture_age = max(0., now-message.get('t',now)-(vage or 0.)) if local_clock else 0.  # lag when the frame arrived
        pose_timeout = min(.65,message.get('max_frame_age_s',.25)) if local_clock else .25
        for node in self.nodes:
            if node.kind != 'robot':
                continue
            pose = poses.get(node.robot_id, {})
            reply = ((raw[node.key].get('drive') or {}).get('reply') or {})
            reason = None
            if not vision or now - vision.get('last_frame_t', 0) > 1:
                reason = 'Fresh camera positions required'
            elif local_clock and not 0 <= capture_age <= pose_timeout:
                reason = 'Camera capture too old'
            elif ((vision.get('message') or {}).get('position_quality') == 'approximate' or
                  (vision.get('message') or {}).get('driving_allowed') is False):
                reason = 'Approximate positions — driving disabled'
            elif not pose.get('pose_compensated') or pose.get('heading') is None or pose.get('age',999)+capture_age > pose_timeout:
                reason = pose.get('pose_status') or 'Calibrated compensated chassis pose required'
            elif not reply:
                reason = 'Drive server not responding'
            elif reply.get('hardware') is not True:
                reason = 'Drive server hardware mode not verified'
            elif not reply.get('require_compensated_pose'):
                reason = 'Drive server must require compensated pose'
            hardware_ready = bool(reply and reply.get('hardware') is True and reply.get('require_compensated_pose') is True)
            can_accept = bool(hardware_ready and vision and vision.get('calibrated') and
                              now-vision.get('last_frame_t',0)<=1 and pose.get('heading') is not None and
                              (pose.get('pose_compensated') or pose.get('pose_status')=='Camera capture too old'))
            readiness[str(node.robot_id)] = {'hardware_ready': hardware_ready, 'can_accept_target': can_accept,
                'ready': reason is None, 'reason': reason or 'Ready for an explicitly armed waypoint controller'}
        return {
            "t": now,
            "main_camera": main_camera_view(vmsg, vage, now),
            "navigation": {"targets": self.waypoints.snapshot(), "readiness": readiness,
                           "armed": self.waypoints.armed, "session": self.waypoints.session,
                           "manual": self.manual.snapshot(),
                           "feedback": waypoint_feedback((vision or {}).get('message'), vage, self.waypoints.snapshot(),
                                                         [n.robot_id for n in self.nodes if n.kind == 'robot'])},
            "nodes": out_nodes,
            "links": links,
            "vision": None if vision is None else {"message": vision.get("message"), "scene": vision.get("scene"),
                                                   "gui_port": vision.get("gui_port")},
            "control": None if control is None else {"mode": control.get("mode"), "robots": control.get("robots"),
                                                     "navigation_mode": control.get("navigation_mode")},
            "map": {"x": dbg.get("map_x", [-3, 3]), "y": dbg.get("map_y", [-0.5, 7]),
                    "robot_size_m": 2 * max([r.get("center_offset_m", 0.15) for r in robots_cfg] or [0.15]),
                    "keep_out_m": limits.get("keep_out_m"), "area": self.cfg.get("area")},
        }

    # ---- recovery actions (no motion: only restart the camera sender and the laptop vision program) ----
    CAMERA_RESTART_SCRIPT = (
        'pkill -u "$USER" -f "camstream[.]sender birdseye" || echo "no sender was running"\n'
        'for i in 1 2 3 4 5 6 7 8 9 10 11 12; do sleep 1; p=$(pgrep -u "$USER" -f "camstream[.]sender birdseye")'
        ' && { echo "sender running again after ${i}s (pid $p)"; exit 0; }; done\n'
        'echo "sender did not come back; check: journalctl -u camstream-birdseye"; exit 3\n')

    def restart_camera(self):
        """Restart the main camera sender on the master Pi. It runs as the SSH user under systemd
        (Restart=always), so ending it makes systemd start a fresh one; no sudo needed."""
        master = next((n for n in self.nodes if n.kind == "master"), None)
        if master is None or not master.ssh_user:
            raise RuntimeError("no master Pi with an SSH user is configured")
        out, err, _ = self.ssh_runner(self.ssh, master.ssh_user, master.host, master.ssh_port,
                                      self.CAMERA_RESTART_SCRIPT, 30.0)
        if err:
            raise RuntimeError(err)
        text = out.strip().splitlines()[-1] if out.strip() else "restarted"
        if "again after" not in text:
            raise RuntimeError(text)
        return {"ok": True, "message": text + ". Vision reconnects on its own; if frames stay stale, restart vision."}

    def restart_vision(self, spawn=subprocess.Popen, kill=os.kill):
        """Replace the laptop's vision.main (the one that sent a heartbeat) with a fresh one that has the GUI."""
        (vmsg, vage), _ = self.heartbeats()
        pid = (vmsg or {}).get("pid")
        if isinstance(pid, int) and pid > 1:
            ps = subprocess.run(["ps", "-p", str(pid), "-o", "args="], capture_output=True, text=True).stdout
            if "vision.main" in ps:        # never signal an unrelated process that reused the PID
                kill(pid, signal.SIGTERM)
                for _ in range(50):
                    if subprocess.run(["ps", "-p", str(pid)], capture_output=True).returncode != 0:
                        break
                    time.sleep(0.2)
        log = Path(tempfile.gettempdir()) / "capstone-vision.log"
        proc = spawn([sys.executable, "-u", "-m", "vision.main", "--gui"], cwd=str(REPO),
                     stdout=open(log, "ab"), stderr=subprocess.STDOUT, start_new_session=True)
        return {"ok": True, "message": f"Vision restarted (pid {proc.pid}); log {log}. Allow ~20 s for the first frame."}

    def close(self):
        self._stop.set()
        if self.listener is not None:
            self.listener.close()


def _link(ping):
    if ping is None:
        return {"level": assess.UNKNOWN, "text": "checking"}
    if ping["up"]:
        return {"level": assess.OK, "text": f"{ping['ms']:.0f} ms"}
    return {"level": assess.BAD, "text": "no answer"}


class CameraRelay:
    """Robot camstreams as MJPEG for the page. Subscribes only while at least one browser is watching, so the
    robot WiFi doesn't carry frames nobody looks at."""

    def __init__(self, cfg):
        self.cfg = cfg
        self.lock = threading.Lock()
        self.rx, self.viewers = {}, {}
        sysc = cfg.get("system") or {}
        self.tunnels = CameraTunnels(find_ssh(sysc.get("ssh")), sysc.get("probe_via"))

    def names(self):
        return [n for n in (self.cfg.get("streams") or {}) if n != "birdseye"]  # birds-eye: 12 MP, use vision's GUI

    def open(self, name):
        with self.lock:
            if name not in self.rx:
                from camstream.receiver import FrameReceiver
                cfg = copy.deepcopy(self.cfg)
                stream = cfg["streams"][name]
                stream["host"], stream["port"] = self.tunnels.endpoint(stream["host"], stream["port"])
                self.rx[name] = FrameReceiver(cfg, [name])
            self.viewers[name] = self.viewers.get(name, 0) + 1
            return self.rx[name]

    def release(self, name):
        with self.lock:
            self.viewers[name] -= 1
            rx = self.rx.pop(name) if self.viewers[name] == 0 else None
        if rx is not None:
            rx.close()


def make_calibration(monitor):
    def vision_gui():
        (vmsg, vage), _ = monitor.heartbeats()
        v = assess.heartbeat(vmsg, vage)
        return (v or {}).get("gui_port")
    return CalibrationManager(monitor.cfg, live=monitor.snapshot, vision_gui=vision_gui)


CALIBRATION_ACTIONS = {"measurements", "confirm", "lens", "floor_measure", "floor_apply", "head_capture", "head_fit",
                       "head_apply", "verify_run", "stop_job", "snapshot"}


def calibration_action(cal, name, data):
    """Run one dashboard calibration action; returns a JSON-able result. Raises ValueError/RuntimeError."""
    if name == "measurements":
        return {"measurements": cal.set_measurements(data.get("values") or {})}
    if name == "confirm":
        cal.confirm(str(data.get("step")), data.get("note", ""), bool(data.get("value", True)))
    elif name == "lens":
        return {"lens": cal.lens(str(data.get("command")))}
    elif name == "floor_measure":
        cal.floor_measure()
    elif name == "floor_apply":
        return cal.floor_apply()
    elif name == "head_capture":
        cal.head_capture(str(data.get("role")), data.get("confirmed") is True)
    elif name == "head_fit":
        cal.head_fit()
    elif name == "head_apply":
        return cal.head_apply(data.get("confirmed") is True)
    elif name == "verify_run":
        cal.verify_run()
    elif name == "stop_job":
        cal.stop_job()
    elif name == "snapshot":
        return {"snapshot": cal.write_snapshot("manual export")}
    return {"ok": True}


def make_server(monitor, relay, host, port, calibration=None):
    calibration = calibration or make_calibration(monitor)
    motor_tests = MotorTests(monitor)

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args):
            pass

        def _send(self, code, ctype, body):
            self.send_response(code)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            path = self.path.split("?")[0]
            if path == "/":
                self._send(200, "text/html; charset=utf-8", PAGE.read_bytes())
            elif path == "/api/motors":
                self._send(200, "application/json", json.dumps(motor_tests.snapshot()).encode())
            elif path == "/state":
                self._send(200, "application/json", json.dumps(monitor.snapshot()).encode())
            elif path == "/calibration":
                self._send(200, "text/html; charset=utf-8", CALIBRATION_PAGE.read_bytes())
            elif path == "/api/calibration":
                self._send(200, "application/json", json.dumps(calibration.state(), default=str).encode())
            elif path == "/api/calibration/lens":
                try:
                    self._send(200, "application/json", json.dumps({"lens": calibration.lens()}).encode())
                except (RuntimeError, OSError, ValueError) as error:
                    self._send(200, "application/json", json.dumps({"lens": None, "error": str(error)}).encode())
            elif path == '/api/navigation':
                self._send(200, 'application/json', json.dumps(monitor.snapshot()['navigation']).encode())
            elif path == "/video/birdseye":
                snapshot = monitor.snapshot()
                vision = snapshot.get("vision") or {}
                if not snapshot["main_camera"]["live"]:
                    self.send_error(503, snapshot["main_camera"]["status"])
                else:
                    self._proxy_camera(f"http://127.0.0.1:{int(vision['gui_port'])}/video/birdseye", main=True)
            elif path.startswith("/video/") and path[7:] in relay.names():
                self._mjpeg(path[7:])
            elif path.startswith("/camera/"):
                node = next((n for n in monitor.nodes if n.key == path[8:] and n.kind == "robot"), None)
                if node and node.dashboard_port:
                    self._dashboard_camera(node)
                else:
                    self.send_error(404)
            else:
                self.send_error(404)

        def _calibration_post(self):
            origin = self.headers.get('Origin')
            if origin and origin != 'http://' + self.headers.get('Host', ''):
                self.send_error(403)
                return
            name = self.path[len('/api/calibration/'):]
            if name not in CALIBRATION_ACTIONS:
                self.send_error(404)
                return
            try:
                size = int(self.headers.get('Content-Length', '0'))
                if not 0 <= size <= 4096:
                    raise ValueError('Oversized JSON request')
                data = json.loads(self.rfile.read(size) or b'{}')
                if not isinstance(data, dict):
                    raise ValueError('JSON object required')
                result = calibration_action(calibration, name, data)
                self._send(200, 'application/json', json.dumps(result, default=str).encode())
            except (ValueError, TypeError, KeyError, RuntimeError, OSError) as error:
                self._send(400, 'application/json', json.dumps({'error': str(error)}).encode())

        def waypoint_manual_enable(self,data):
            monitor.manual.enable(data['robot_id'])
            monitor.waypoints.arm(False)

        def do_POST(self):
            if self.path in ('/api/manual/enable','/api/manual/command','/api/manual/disable'):
                origin=self.headers.get('Origin')
                if origin and origin!='http://'+self.headers.get('Host',''):
                    self.send_error(403)
                    return
                try:
                    size=int(self.headers.get('Content-Length','0'))
                    if not 0<size<=2048:raise ValueError('Missing or oversized JSON request')
                    data=json.loads(self.rfile.read(size))
                    if not isinstance(data,dict):raise ValueError('JSON object required')
                    with motor_tests.lock:
                        if motor_tests.snapshot()['running']:
                            raise ValueError('Stop the motor test before enabling keyboard control')
                        if self.path.endswith('/enable'):
                            self.waypoint_manual_enable(data)
                            result=monitor.manual.snapshot()
                        elif self.path.endswith('/disable'):
                            result=monitor.manual.disable(data.get('token',''))
                        else:
                            result=monitor.manual.command(data)
                    self._send(200,'application/json',json.dumps(result).encode())
                except (ValueError,TypeError,KeyError) as error:
                    self._send(400,'application/json',json.dumps({'error':str(error)}).encode())
                return
            if self.path in ('/api/motors/start', '/api/motors/stop', '/api/motors/settings'):
                origin = self.headers.get('Origin')
                if origin and origin != 'http://' + self.headers.get('Host', ''):
                    self.send_error(403)
                    return
                try:
                    size = int(self.headers.get('Content-Length', '0'))
                    if not 0 < size <= 2048:
                        raise ValueError('Missing or oversized JSON request')
                    data = json.loads(self.rfile.read(size))
                    if not isinstance(data, dict):
                        raise ValueError('JSON object required')
                    if self.path.endswith('/settings'):
                        result = motor_tests.save_settings(data)
                    else:
                        result = motor_tests.start(data) if self.path.endswith('/start') else motor_tests.stop()
                    self._send(200, 'application/json', json.dumps(result).encode())
                except (ValueError, TypeError, KeyError, OSError, StopIteration) as error:
                    self._send(400, 'application/json', json.dumps({'error': str(error)}).encode())
                return
            if self.path.startswith('/api/calibration/'):
                self._calibration_post()
                return
            if self.path in ('/api/camera/restart', '/api/vision/restart'):
                origin = self.headers.get('Origin')
                if origin and origin != 'http://' + self.headers.get('Host', ''):
                    self.send_error(403)
                    return
                try:
                    action = monitor.restart_camera if 'camera' in self.path else monitor.restart_vision
                    self._send(200, 'application/json', json.dumps(action()).encode())
                except (RuntimeError, OSError) as error:
                    self._send(502, 'application/json', json.dumps({'error': str(error)}).encode())
                return
            if self.path not in ('/api/navigation/target', '/api/navigation/clear', '/api/navigation/arm'):
                self.send_error(404)
                return
            origin = self.headers.get('Origin')
            if origin and origin != 'http://' + self.headers.get('Host', ''):
                self.send_error(403)
                return
            try:
                size = int(self.headers.get('Content-Length', '0'))
                if not 0 < size <= 2048:
                    raise ValueError('Missing or oversized JSON request')
                data = json.loads(self.rfile.read(size))
                if motor_tests.snapshot()['running']:
                    raise ValueError('Stop the motor test before changing navigation')
                if monitor.manual.snapshot()['enabled']:
                    raise ValueError('Disable keyboard control before changing navigation')
                if self.path.endswith('/arm'):
                    monitor.waypoints.arm(data['enabled'])
                elif self.path.endswith('/clear'):
                    monitor.waypoints.clear()
                else:
                    readiness = monitor.snapshot()['navigation']['readiness'].get(str(data['robot_id']), {})
                    ready = readiness.get('ready',False) or readiness.get('can_accept_target',False)
                    monitor.waypoints.set(data['robot_id'], data['x'], data['y'], preview_only=not ready or not monitor.waypoints.armed)
                self._send(200, 'application/json', json.dumps(monitor.snapshot()['navigation']).encode())
            except (ValueError, TypeError, KeyError) as error:
                self._send(400, 'application/json', json.dumps({'error': str(error)}).encode())

        def _dashboard_camera(self, node):
            try:
                host, port = relay.tunnels.endpoint(node.host, node.dashboard_port)
            except OSError:
                self.send_error(502, "Robot camera SSH connection unavailable")
                return
            self._proxy_camera(f"http://{host}:{port}/camera.mjpg")

        def _proxy_camera(self, url, main=False):
            headers_sent = False
            try:
                with urllib.request.urlopen(url, timeout=10) as upstream:
                    self.send_response(200)
                    self.send_header("Content-Type", upstream.headers.get("Content-Type", "application/octet-stream"))
                    self.send_header("Cache-Control", "no-store")
                    self.end_headers()
                    headers_sent = True
                    while chunk := upstream.read1(65536):
                        if main and not monitor.snapshot()["main_camera"]["live"]:
                            break
                        self.wfile.write(chunk)
            except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError):
                pass
            except OSError:
                if not headers_sent:
                    self.send_error(502, "Camera unavailable; check its dashboard status")
                else:
                    self.close_connection = True

        def _mjpeg(self, name):
            try:
                rx = relay.open(name)
            except OSError:
                self.send_error(502, "Camera SSH connection unavailable")
                return
            try:
                self.send_response(200)
                self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                last = None
                while True:
                    f = rx.wait_new(name, last, timeout=5)
                    if f is None:
                        continue
                    last = f.seq
                    self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: "
                                     + str(len(f.payload)).encode() + b"\r\n\r\n" + f.payload + b"\r\n")
            except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError):
                pass
            finally:
                relay.release(name)

    httpd = ThreadingHTTPServer((host, port), Handler)
    httpd.daemon_threads = True
    return httpd


def main(argv=None):
    ap = argparse.ArgumentParser(description="System GUI: status of every Pi, robot and laptop program")
    ap.add_argument("--config", help="path to config.yaml (default: repo config.yaml)")
    ap.add_argument("--port", type=int, help="HTTP port (default: config system.gui_port, 8090)")
    ap.add_argument("--bind", default="127.0.0.1", help="address to serve on (0.0.0.0 = every network)")
    ap.add_argument("--no-ssh", action="store_true", help="skip the SSH health checks")
    args = ap.parse_args(argv)

    cfg = load_config(args.config)
    port = args.port or int((cfg.get("system") or {}).get("gui_port", 8090))
    try:
        monitor = SystemMonitor(cfg, ssh=not args.no_ssh)
    except OSError as e:
        ap.exit(1, f"Can't listen for status heartbeats on UDP {status_port(cfg)} ({e}). "
                   "Is another system GUI already running?\n")
    relay = CameraRelay(cfg)
    httpd = make_server(monitor, relay, args.bind, port)
    for n in monitor.nodes:
        ssh = f"ssh {n.ssh_user}@{n.host}" if n.ssh_user and monitor.ssh_interval > 0 else "no ssh checks"
        print(f"  {n.name:10s} {n.host:15s} {ssh}")
    if monitor.ssh_interval > 0 and not monitor.ssh:
        print("  (no ssh client found: health checks will show an error)")
    print(f"System GUI: http://localhost:{port}/   Ctrl+C to stop.")
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        httpd.server_close()
        monitor.close()
        relay.tunnels.close()
    return 0
