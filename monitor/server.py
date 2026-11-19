"""System GUI: one page with the status of the laptop programs, the master Pi and every robot.

    python -m monitor                  # then open http://localhost:8090/
    python -m monitor --no-ssh         # skip the SSH health checks (temperature, power, WiFi, processes)
    python -m monitor --bind 0.0.0.0   # also reachable from other machines (e.g. a phone)

Runs on its own, so it keeps working when vision.main or control.main are stopped or crash. Read-only: it
never starts, stops or commands anything. Where the information comes from:

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
import json
import os
import threading
import time
from dataclasses import dataclass, field
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

from vision.config import load_config

from . import assess
from .beacon import StatusListener, status_port
from .probes import drive_status, find_ssh, http_json, parse_health, run_ssh, tcp_check

PAGE = Path(__file__).with_name("page.html")


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
        sysc = cfg.get("system") or {}
        self.nodes = build_nodes(cfg)
        self.probe_interval = float(sysc.get("probe_interval_s", 2.0))
        self.ssh_interval = float(sysc.get("ssh_interval_s", 10.0)) if ssh else 0.0
        self.ssh = find_ssh(sysc.get("ssh"))
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
        r = {"at": time.time(), "ping": tcp_check(node.host, node.ssh_port)}
        if node.wifi_host:
            r["ping_wifi"] = tcp_check(node.wifi_host, node.ssh_port)
        if r["ping"]["up"]:
            r["ports"] = {name: tcp_check(node.stream_hosts[name], port) for name, port in node.streams.items()}
            if node.drive_port:
                r["drive"] = drive_status(node.host, node.drive_port)
            if node.dashboard_port:
                r["dashboard"] = http_json(f"http://{node.host}:{node.dashboard_port}/api/state")
        else:  # offline: forget the last results instead of showing them as current
            r.update({"ports": None, "drive": None, "dashboard": None, "health": None, "health_error": None})
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
        return {
            "t": now,
            "nodes": out_nodes,
            "links": links,
            "vision": None if vision is None else {"message": vision.get("message"), "scene": vision.get("scene"),
                                                   "gui_port": vision.get("gui_port")},
            "control": None if control is None else {"mode": control.get("mode"), "robots": control.get("robots")},
            "map": {"x": dbg.get("map_x", [-3, 3]), "y": dbg.get("map_y", [-0.5, 7]),
                    "robot_size_m": 2 * max([r.get("center_offset_m", 0.15) for r in robots_cfg] or [0.15]),
                    "keep_out_m": limits.get("keep_out_m"), "area": self.cfg.get("area")},
        }

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

    def names(self):
        return [n for n in (self.cfg.get("streams") or {}) if n != "birdseye"]  # birds-eye: 12 MP, use vision's GUI

    def open(self, name):
        with self.lock:
            if name not in self.rx:
                from camstream.receiver import FrameReceiver
                self.rx[name] = FrameReceiver(self.cfg, [name])
            self.viewers[name] = self.viewers.get(name, 0) + 1
            return self.rx[name]

    def release(self, name):
        with self.lock:
            self.viewers[name] -= 1
            rx = self.rx.pop(name) if self.viewers[name] == 0 else None
        if rx is not None:
            rx.close()


def make_server(monitor, relay, host, port):
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
            elif path == "/state":
                self._send(200, "application/json", json.dumps(monitor.snapshot()).encode())
            elif path.startswith("/video/") and path[7:] in relay.names():
                self._mjpeg(path[7:])
            else:
                self.send_error(404)

        def _mjpeg(self, name):
            rx = relay.open(name)
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
    httpd = make_server(monitor, CameraRelay(cfg), args.bind, port)
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
    return 0
