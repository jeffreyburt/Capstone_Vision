"""Debug visualisation: camera overlay + top-down floor map, shown locally or streamed to a browser.

The Pi usually runs headless, so MjpegServer serves the debug view over HTTP:
    http://<pi-ip>:8080/              live view
    http://<pi-ip>:8080/snapshot.jpg  one raw camera frame (useful for floor calibration)
"""

import math
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
import numpy as np

GREEN, RED, YELLOW, GREY, WHITE, BLUE = (0, 200, 0), (0, 0, 255), (0, 220, 255), (150, 150, 150), (255, 255, 255), (255, 140, 0)
ROBOT_COLORS = [(255, 140, 0), (0, 140, 255), (200, 0, 200), (0, 200, 200)]


def draw_overlay(frame, result, geometry=None, fps=None):
    img = frame.copy()
    for m in result.markers:
        pts = m.corners.astype(np.int32)
        cv2.polylines(img, [pts], True, GREEN, 2)
        cv2.circle(img, tuple(pts[0]), 4, RED, -1)  # top-left corner
        cv2.putText(img, f"{m.id} ({m.side_px:.0f}px)", tuple(pts[0] + [0, -8]),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, GREEN, 2)
    if geometry is not None:
        for o in result.observations:  # project the estimated robot center back into the image
            u, v = geometry.world_to_pixels([[o.x, o.y, 0.0]])[0]
            if 0 <= u < img.shape[1] and 0 <= v < img.shape[0]:
                cv2.drawMarker(img, (int(u), int(v)), YELLOW, cv2.MARKER_CROSS, 30, 3)
                cv2.putText(img, f"R{o.robot_id}", (int(u) + 10, int(v) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.8, YELLOW, 2)
    for d in result.people:
        x1, y1, x2, y2 = (int(v) for v in d.box)
        cv2.rectangle(img, (x1, y1), (x2, y2), RED, 2)
        cv2.putText(img, f"person {d.confidence:.2f}", (x1, y1 - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.7, RED, 2)
        cv2.circle(img, (int((x1 + x2) / 2), y2), 6, RED, -1)  # foot point used for the floor position
    text = f"markers: {len(result.markers)}  people: {len(result.people)}"
    if fps:
        text = f"{fps:.1f} fps   " + text
    if geometry is None:
        text += "   (no floor calibration)"
    cv2.putText(img, text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.9, WHITE, 2)
    return img


class FloorMap:
    """Top-down map of the floor in world coordinates (x right, y up the image = away from the camera)."""

    def __init__(self, x_range=(-3, 3), y_range=(-0.5, 7), px_per_m=70, geometry=None, frame_size=None):
        self.x0, self.x1 = x_range
        self.y0, self.y1 = y_range
        self.s = px_per_m
        self.size = (int((self.x1 - self.x0) * self.s), int((self.y1 - self.y0) * self.s))
        self.background = self._draw_background(geometry, frame_size)

    def to_px(self, x, y):
        return int((x - self.x0) * self.s), int((self.y1 - y) * self.s)

    def _draw_background(self, geometry, frame_size):
        w, h = self.size
        img = np.full((h, w, 3), 30, np.uint8)
        for x in range(math.ceil(self.x0), math.floor(self.x1) + 1):
            cv2.line(img, self.to_px(x, self.y0), self.to_px(x, self.y1), (60, 60, 60) if x else (110, 110, 110), 1)
        for y in range(math.ceil(self.y0), math.floor(self.y1) + 1):
            cv2.line(img, self.to_px(self.x0, y), self.to_px(self.x1, y), (60, 60, 60) if y else (110, 110, 110), 1)
            cv2.putText(img, f"{y}m", self.to_px(self.x0, y + 0.05), cv2.FONT_HERSHEY_SIMPLEX, 0.4, GREY, 1)
        if geometry is not None and frame_size is not None:
            # Outline of the floor area the camera can see.
            fw, fh = frame_size
            n = 30
            border = np.vstack([
                np.column_stack([np.linspace(0, fw - 1, n), np.full(n, fh - 1)]),       # bottom edge
                np.column_stack([np.full(n, fw - 1), np.linspace(fh - 1, 0, n)]),       # right edge
                np.column_stack([np.linspace(fw - 1, 0, n), np.zeros(n)]),               # top edge
                np.column_stack([np.zeros(n), np.linspace(0, fh - 1, n)]),               # left edge
            ])
            floor = geometry.pixels_to_floor(border, 0.0)
            cam = geometry.camera_position
            ok = ~np.isnan(floor[:, 0]) & (np.linalg.norm(floor - cam[:2], axis=1) < 25)
            pts = np.array([self.to_px(x, y) for x, y in floor[ok]], np.int32)
            if len(pts) > 2:
                cv2.polylines(img, [pts], True, (80, 80, 40), 1)
            cx, cy = self.to_px(cam[0], cam[1])
            cv2.circle(img, (cx, cy), 6, WHITE, -1)
            cv2.putText(img, f"cam {cam[2]:.2f}m high", (cx + 8, cy), cv2.FONT_HERSHEY_SIMPLEX, 0.4, WHITE, 1)
        return img

    def draw(self, message, robot_size_m=0.30, lost_after_s=1.0):
        img = self.background.copy()
        for i, r in enumerate(message.get("robots", [])):
            color = ROBOT_COLORS[i % len(ROBOT_COLORS)] if r["age"] <= lost_after_s else GREY
            hd = math.radians(r["heading"] or 0.0)
            c, s = math.cos(hd), math.sin(hd)
            half = robot_size_m / 2
            corners = [(r["x"] + c * dx - s * dy, r["y"] + s * dx + c * dy)
                       for dx, dy in [(half, half), (-half, half), (-half, -half), (half, -half)]]
            cv2.polylines(img, [np.array([self.to_px(*p) for p in corners], np.int32)], True, color, 2)
            if r["heading"] is not None:  # arrow from center out through the front face
                cv2.arrowedLine(img, self.to_px(r["x"], r["y"]), self.to_px(r["x"] + c * 0.35, r["y"] + s * 0.35), color, 2, tipLength=0.3)
            cv2.putText(img, f"R{r['id']} ({r['x']:.2f}, {r['y']:.2f}) {r['heading'] or 0:.0f}deg",
                        self.to_px(r["x"] + 0.25, r["y"] + 0.2), cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1)
        user = message.get("user")
        if user:
            color = RED if user["age"] <= lost_after_s else GREY
            cv2.circle(img, self.to_px(user["x"], user["y"]), 12, color, 2)
            h = f" {user['height']:.2f}m tall" if user.get("height") else ""
            cv2.putText(img, f"user ({user['x']:.2f}, {user['y']:.2f}){h}", self.to_px(user["x"] + 0.25, user["y"] - 0.1),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1)
        return img


def compose(overlay, map_img, max_width=1280):
    """Overlay and map side by side, scaled to fit max_width."""
    h = overlay.shape[0]
    m = cv2.resize(map_img, (int(map_img.shape[1] * h / map_img.shape[0]), h))
    out = np.hstack([overlay, m])
    if out.shape[1] > max_width:
        scale = max_width / out.shape[1]
        out = cv2.resize(out, (max_width, int(out.shape[0] * scale)), interpolation=cv2.INTER_AREA)
    return out


class MjpegServer:
    """Tiny HTTP server streaming the latest debug image (no extra dependencies)."""

    def __init__(self, port=8080, quality=70):
        self.quality = quality
        self._cond = threading.Condition()
        self._jpeg, self._raw_jpeg, self._seq = None, None, 0
        server = self

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, *args):
                pass

            def do_GET(self):
                if self.path == "/":
                    body = b"<html><body style='margin:0;background:#111'><img src='/stream' style='width:100%'></body></html>"
                    self.send_response(200)
                    self.send_header("Content-Type", "text/html")
                    self.end_headers()
                    self.wfile.write(body)
                elif self.path.startswith("/snapshot"):
                    with server._cond:
                        data = server._raw_jpeg
                    if data is None:
                        self.send_error(503, "No frame yet")
                        return
                    self.send_response(200)
                    self.send_header("Content-Type", "image/jpeg")
                    self.send_header("Content-Length", str(len(data)))
                    self.end_headers()
                    self.wfile.write(data)
                elif self.path.startswith("/stream"):
                    self.send_response(200)
                    self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
                    self.end_headers()
                    seen = -1
                    try:
                        while True:
                            with server._cond:
                                server._cond.wait_for(lambda: server._seq != seen, timeout=5)
                                data, seen = server._jpeg, server._seq
                            if data is None:
                                continue
                            self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: "
                                             + str(len(data)).encode() + b"\r\n\r\n" + data + b"\r\n")
                    except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError):
                        pass
                else:
                    self.send_error(404)

        self.httpd = ThreadingHTTPServer(("0.0.0.0", port), Handler)
        self.httpd.daemon_threads = True
        threading.Thread(target=self.httpd.serve_forever, daemon=True).start()
        print(f"Debug stream: http://<this-machine-ip>:{port}/  (raw frame: /snapshot.jpg)")

    def update(self, img, raw_frame=None):
        jpeg = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, self.quality])[1].tobytes()
        raw = None
        if raw_frame is not None:
            raw = cv2.imencode(".jpg", raw_frame, [cv2.IMWRITE_JPEG_QUALITY, 95])[1].tobytes()
        with self._cond:
            self._jpeg = jpeg
            if raw is not None:
                self._raw_jpeg = raw
            self._seq += 1
            self._cond.notify_all()

    def close(self):
        self.httpd.shutdown()
