"""Live browser GUI: camera streams + a 3D view of the robots and user.

    python -m vision.main --gui          # then open http://localhost:8080/

Served from the laptop with no extra Python packages (the 3D view uses three.js, loaded by the browser):
    /                      the page (vision/gui.html)
    /video/birdseye        birds-eye camera with detection overlays (MJPEG)
    /video/<stream>        raw robot camera streams, e.g. /video/robot1 (MJPEG, passed through as-is)
    /state                 JSON: latest positions, camera pose and scene info, stream stats, calibration
    POST /calib/<command>  lens calibration: start, stop, capture, auto_on, auto_off, undo, reset, solve
"""

import json
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import cv2
import numpy as np

PAGE = Path(__file__).with_name("gui.html")


def scene_info(cfg, geometry, frame_size):
    """Static scene description for the 3D view (world frame: meters, z up)."""
    robots = cfg.get("robots", [])
    m = cfg.get("markers", {})
    info = {
        "robot_size_m": 2 * max([r.get("center_offset_m", 0.15) for r in robots] or [0.15]),
        "robot_height_m": max(0.3, m.get("center_height_m", 0.30) + 0.1),
        "area": cfg.get("area"),
        "camera": None,
    }
    if geometry is None:
        return info
    info["approximation"] = geometry.approximation
    w, h = frame_size
    corners = np.array([[0, 0], [w - 1, 0], [w - 1, h - 1], [0, h - 1]], float)
    rays = geometry.camera.pixels_to_rays(corners) @ geometry.pose.R  # world-frame directions
    rays /= np.linalg.norm(rays, axis=1, keepdims=True)
    # Where the image edges meet the floor (the visible floor area), clipped to a sensible range.
    n = 24
    border = np.vstack([np.column_stack([np.linspace(0, w - 1, n), np.full(n, h - 1)]),
                        np.column_stack([np.full(n, w - 1), np.linspace(h - 1, 0, n)]),
                        np.column_stack([np.linspace(w - 1, 0, n), np.zeros(n)]),
                        np.column_stack([np.zeros(n), np.linspace(0, h - 1, n)])])
    floor = geometry.pixels_to_floor(border, 0.0)
    cam = geometry.camera_position
    ok = ~np.isnan(floor[:, 0]) & (np.linalg.norm(floor - cam[:2], axis=1) < 12)
    info["camera"] = {
        "position": cam.round(3).tolist(),
        "frustum_dirs": rays.round(4).tolist(),        # image corners: top-left, top-right, bottom-right, bottom-left
        "forward": (np.array([0, 0, 1.0]) @ geometry.pose.R).round(4).tolist(),
        "floor_footprint": floor[ok].round(3).tolist(),
    }
    return info


class GuiServer:
    def __init__(self, cfg, port=8080, robot_streams=None, quality=75, max_width=1600):
        from .config import resolve_path
        from .lens_calibration import LensCalibrator

        self.quality, self.max_width = quality, max_width
        self._cond = threading.Condition()
        self._jpeg, self._seq = None, 0
        self._raw = None  # latest full-resolution birds-eye frame (for calibration captures)
        self._state = {"message": None, "scene": None, "fps": None, "streams": {}}
        self.calib = LensCalibrator(resolve_path(cfg, cfg["calibration"]["intrinsics"]))
        self.intrinsics_changed = False  # set after a calibration; the main loop reloads the geometry

        names = [n for n in (robot_streams if robot_streams is not None else cfg.get("streams", {})) if n != "birdseye"]
        self.rx = None
        if names:
            from camstream.receiver import FrameReceiver
            self.rx = FrameReceiver(cfg, names)
        server = self

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

            def _mjpeg(self, next_jpeg):
                self.send_response(200)
                self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
                self.end_headers()
                last = None
                try:
                    while True:
                        data, last = next_jpeg(last)
                        if data is None:
                            continue
                        self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: "
                                         + str(len(data)).encode() + b"\r\n\r\n" + data + b"\r\n")
                except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError):
                    pass

            def do_GET(self):
                path = self.path.split("?")[0]
                if path == "/":
                    self._send(200, "text/html; charset=utf-8", PAGE.read_bytes())
                elif path == "/state":
                    self._send(200, "application/json", json.dumps(server.state()).encode())
                elif path == "/video/birdseye":
                    self._mjpeg(server._next_birdseye)
                elif path.startswith("/video/") and server.rx and path[7:] in server.rx.streams:
                    name = path[7:]
                    self._mjpeg(lambda last: server._next_stream(name, last))
                else:
                    self.send_error(404)

            def do_POST(self):
                path = self.path.split("?")[0]
                if not path.startswith("/calib/"):
                    self.send_error(404)
                    return
                ok = server.calib_command(path[len("/calib/"):])
                self._send(200 if ok else 400, "application/json", json.dumps(server.calib.status()).encode())

        self.httpd = ThreadingHTTPServer(("0.0.0.0", port), Handler)
        self.httpd.daemon_threads = True
        threading.Thread(target=self.httpd.serve_forever, daemon=True).start()
        print(f"GUI: http://localhost:{port}/")

    def _next_birdseye(self, last):
        with self._cond:
            self._cond.wait_for(lambda: self._seq != last, timeout=5)
            return self._jpeg, self._seq

    def _next_stream(self, name, last):
        f = self.rx.wait_new(name, last, timeout=5)
        return (f.payload, f.seq) if f is not None else (None, last)

    def set_scene(self, scene):
        with self._cond:
            self._state["scene"] = scene

    def calib_command(self, cmd):
        c = self.calib
        if cmd == "start":
            c.active, c.message = True, "hold the checkerboard in view"
        elif cmd == "stop":
            c.active = False
        elif cmd in ("auto_on", "auto_off"):
            c.auto = cmd == "auto_on"
        elif cmd == "capture":
            with self._cond:
                raw = self._raw
            if raw is None:
                c.message = "no camera frame yet"
                return False
            return c.capture(raw)
        elif cmd == "undo":
            c.undo()
        elif cmd == "reset":
            c.reset()
        elif cmd == "solve":
            if c.calibrate() is None:
                return False
            self.intrinsics_changed = True
        else:
            return False
        return True

    def update(self, overlay, message, fps=None, raw=None):
        """overlay: annotated birds-eye frame (BGR). message: the pipeline's output message.
        raw: the unannotated full-resolution frame (used for calibration captures)."""
        if raw is not None:
            with self._cond:
                self._raw = raw
        if overlay.shape[1] > self.max_width:
            s = self.max_width / overlay.shape[1]
            overlay = cv2.resize(overlay, (self.max_width, int(overlay.shape[0] * s)), interpolation=cv2.INTER_AREA)
        jpeg = cv2.imencode(".jpg", overlay, [cv2.IMWRITE_JPEG_QUALITY, self.quality])[1].tobytes()
        with self._cond:
            self._jpeg = jpeg
            self._seq += 1
            self._state["message"], self._state["fps"] = message, fps
            self._cond.notify_all()

    def state(self):
        with self._cond:
            st = dict(self._state)
        st["streams"] = {n: self.rx.stats(n) for n in self.rx.streams} if self.rx else {}
        st["calib"] = self.calib.status()
        st["server_time"] = time.time()
        return st

    def close(self):
        self.httpd.shutdown()
        if self.rx:
            self.rx.close()
