"""Relay between the system dashboard and each robot's head-tracker dashboard (port 8080).

The robot's own dashboard owns the servo UART, the camera and the tracking loop. Face detection runs in
a browser; this relay lets the system dashboard's browser tab be that browser for every robot, so the
operator does not have to open each robot's page. Robots can be turned off with
`system.robots.<id>.head_tracking: false` (for example while their servos are disconnected): their
state is still shown, and torque can be switched on/off and motion stopped, but tracking is refused here.
"""
import json
import urllib.error
import urllib.request

ACTIONS = {"engage": "/api/engage", "disengage": "/api/disengage", "stop": "/api/stop",
           "start": "/api/tracking/start", "observation": "/api/tracking/observation"}
ALWAYS_ALLOWED = {"stop", "engage", "disengage"}  # stop and torque on/off work even with head tracking disabled
                                                 # (e.g. releasing a head to find its travel limits by hand)
STATE_FIELDS = ("status", "positions", "position_time", "position_error", "torque", "running",
                "controlling", "camera", "serverTime", "servoPort")


def tracking_limits(cfg):
    """robot key -> [[pan lo, hi], [tilt lo, hi]] from `system.robots.<id>.head_tracking_limits`, if set."""
    out = {}
    for rid, entry in ((cfg.get("system") or {}).get("robots") or {}).items():
        lim = (entry or {}).get("head_tracking_limits")
        if lim is not None:
            if (not isinstance(lim, list) or len(lim) != 2 or
                    any(not isinstance(p, list) or len(p) != 2 or not 0 <= p[0] < p[1] <= 1000 for p in lim)):
                raise ValueError(f"robot {rid}: head_tracking_limits must be [[pan lo, hi], [tilt lo, hi]] in 0-1000")
            out[f"robot{int(rid)}"] = [[int(v) for v in p] for p in lim]
    return out


def tracking_signs(cfg):
    """robot key -> [pan sign, tilt sign] from `system.robots.<id>.head_tracking_signs`, if set."""
    out = {}
    for rid, entry in ((cfg.get("system") or {}).get("robots") or {}).items():
        signs = (entry or {}).get("head_tracking_signs")
        if signs is not None:
            if not isinstance(signs, list) or len(signs) != 2 or any(v not in (-1, 1) for v in signs):
                raise ValueError(f"robot {rid}: head_tracking_signs must be [±1, ±1]")
            out[f"robot{int(rid)}"] = list(signs)
    return out


def enabled_robots(cfg):
    """robot key -> (enabled, reason) from `system.robots.<id>.head_tracking` (default enabled)."""
    out = {}
    for rid, entry in ((cfg.get("system") or {}).get("robots") or {}).items():
        entry = entry or {}
        on = entry.get("head_tracking", True) is not False
        out[f"robot{int(rid)}"] = (on, None if on else entry.get("head_tracking_reason")
                                   or "Head tracking is disabled for this robot in config.yaml")
    return out


class HeadRelay:
    def __init__(self, cfg, endpoint, *, urlopen=urllib.request.urlopen):
        self.flags = enabled_robots(cfg)
        self.limits = tracking_limits(cfg)
        self.signs = tracking_signs(cfg)
        self.endpoint = endpoint  # (host, port) -> reachable (host, port), e.g. through the SSH tunnel
        self.urlopen = urlopen

    def enabled(self, node):
        return self.flags.get(node.key, (True, None))

    def _url(self, node, path):
        host, port = self.endpoint(node.host, node.dashboard_port)
        return f"http://{host}:{port}{path}"

    def state(self, node):
        on, reason = self.enabled(node)
        out = {"robot": node.key, "name": node.name, "enabled": on, "disabled_reason": reason}
        try:
            with self.urlopen(self._url(node, "/api/state"), timeout=3) as response:
                raw = json.load(response)
        except (OSError, ValueError) as error:
            out.update(reachable=False, error=f"Head tracker unreachable: {error}")
            return out
        tracking = raw.get("tracking") or {}
        out.update(reachable=True, **{k: raw.get(k) for k in STATE_FIELDS},
                   tracking={k: tracking.get(k) for k in ("active", "status", "mode", "center", "config", "targets")})
        return out

    def frame(self, node):
        """(jpeg bytes, robot frame time) for browser face detection."""
        if not self.enabled(node)[0]:
            raise PermissionError(self.enabled(node)[1])
        with self.urlopen(self._url(node, "/api/camera/frame"), timeout=2.5) as response:
            return response.read(), response.headers.get("X-Frame-Time")

    def post(self, node, action, data=None):
        """Forward one control request. Returns (HTTP status, message)."""
        if action not in ACTIONS:
            return 404, "Unknown head-tracker action"
        on, reason = self.enabled(node)
        if not on and action not in ALWAYS_ALLOWED:
            return 409, reason
        if action == "start":
            data = self._start_request(node, data or {})
        body = json.dumps(data).encode() if data is not None else b""
        request = urllib.request.Request(self._url(node, ACTIONS[action]), data=body, method="POST",
                                         headers={"Content-Type": "application/json"})
        try:
            with self.urlopen(request, timeout=6 if action in ("engage", "disengage") else 2) as response:
                return response.status, "OK"
        except urllib.error.HTTPError as error:
            return error.code, _message(error)
        except OSError as error:
            return 502, f"Head tracker unreachable: {error}"

    def _start_request(self, node, data):
        """Start with the settings saved on the robot (taught limits, directions, speed) unless given."""
        if {"limits", "signs"} <= set(data):
            return {**data, "target": "face"}
        saved = (self.state(node).get("tracking") or {}).get("config")
        if not saved and node.key in self.limits and node.key in self.signs:
            saved = {"limits": self.limits[node.key], "signs": self.signs[node.key]}  # nothing saved on the robot yet
        if not saved:
            raise ValueError("No saved tracking settings on this robot; teach limits on its own dashboard first")
        # Limits: config head_tracking_limits (e.g. the range the camera pose model is calibrated for), else the
        # robot's taught limits. Directions always come from the robot; speed and prediction may be overridden.
        return {"target": "face", "limits": self.limits.get(node.key, saved["limits"]),
                "signs": self.signs.get(node.key, saved["signs"]),
                "speed": data.get("speed", saved.get("speed", 180)),
                "prediction": data.get("prediction", saved.get("prediction", 1.2))}


def _message(error):
    """The robot dashboard reports errors as the http.server HTML page; pull out its message line."""
    try:
        text = error.read().decode(errors="replace")
    except OSError:
        text = ""
    for line in text.splitlines():
        if "Message:" in line:
            return line.split("Message:", 1)[1].split("</p>")[0].strip().rstrip(".")
    return error.reason or f"HTTP {error.code}"


UNITS_PER_FRAME = (216.0, 171.0)  # servo units across the robot camera image (same estimate as tracking_control)
FACE_BELOW_TOP_M = 0.12           # face centre below the top of the head


def aim_from_main_camera(head_pose, robot, user, positions, signs, limits=None, units_per_frame=UNITS_PER_FRAME):
    """Where the user's face should appear in the robot's head camera, from the main camera's view.

    head_pose: the robot's `head_pose` config (neutral_units, degrees_per_unit, pan_pivot_m, tilt_pivot_m).
    robot: {x, y, heading} on the floor; user: {x, y, height}. positions: current [pan, tilt] servo units;
    signs: the tracking directions. Returns ({"center": [u, v], "units": [pan, tilt]}) with the target clamped
    to `limits`, or None. Positive pan turns the head toward chassis left; positive tilt turns it down.
    """
    import math
    neutral = head_pose["neutral_units"]
    dpu = head_pose["degrees_per_unit"]
    pivot = head_pose["pan_pivot_m"]
    tilt_z = head_pose["tilt_pivot_m"][2]
    h = math.radians(robot["heading"])
    cx = robot["x"] + pivot[0] * math.cos(h) - pivot[1] * math.sin(h)
    cy = robot["y"] + pivot[0] * math.sin(h) + pivot[1] * math.cos(h)
    dx, dy = user["x"] - cx, user["y"] - cy
    distance = math.hypot(dx, dy)
    if distance < 0.2:
        return None
    pan_deg = (math.degrees(math.atan2(dy, dx) - h) + 180) % 360 - 180
    face_z = float(user.get("height") or 1.6) - FACE_BELOW_TOP_M
    tilt_deg = -math.degrees(math.atan2(face_z - tilt_z, distance))  # looking up is negative tilt
    units = [neutral[0] + pan_deg / dpu[0], neutral[1] + tilt_deg / dpu[1]]
    if limits:
        units = [min(max(u, lo), hi) for u, (lo, hi) in zip(units, limits)]
    center = [max(-.5, min(1.5, .5 + (t - p) / (s * u)))
              for t, p, s, u in zip(units, positions, signs, units_per_frame)]
    return {"center": center, "units": [round(u, 1) for u in units]}
