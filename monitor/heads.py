"""Relay between the system dashboard and each robot's head-tracker dashboard (port 8080).

The robot's own dashboard owns the servo UART, the camera and the tracking loop. Face detection runs in
a browser; this relay lets the system dashboard's browser tab be that browser for every robot, so the
operator does not have to open each robot's page. Robots can be turned off with
`system.robots.<id>.head_tracking: false` (for example while their servos are disconnected): their
state is still shown, but every request that could move a servo is refused here.
"""
import json
import urllib.error
import urllib.request

ACTIONS = {"engage": "/api/engage", "disengage": "/api/disengage", "stop": "/api/stop",
           "start": "/api/tracking/start", "observation": "/api/tracking/observation"}
ALWAYS_ALLOWED = {"stop"}  # stopping is never refused, even for a disabled robot
STATE_FIELDS = ("status", "positions", "position_time", "position_error", "torque", "running",
                "controlling", "camera", "serverTime")


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
        if not saved:
            raise ValueError("No saved tracking settings on this robot; teach limits on its own dashboard first")
        # Taught limits and directions always come from the robot; speed and prediction may be overridden.
        return {"target": "face", "limits": saved["limits"], "signs": saved["signs"],
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
