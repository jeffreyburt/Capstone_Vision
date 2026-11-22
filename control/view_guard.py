"""Keep the robots inside the main camera's view.

Vision publishes `view.polygon`: the floor area where a robot's tags appear inside the main camera's image
(vision/pipeline.py camera_view, from the existing lens + floor calibration; no extra calibration). Every
control step, after the mode (follow / waypoints / keyboard) has chosen its commands, ViewGuard.apply:

1. Soft stop. For a robot the camera sees, the part of its velocity that points out of the view is scaled
   down linearly from full speed `slow_m` inside the edge to zero at `stop_m` inside it. Motion along the
   edge or back inward is untouched, so a robot slides along the edge instead of leaving.
2. Return. A robot the camera has lost, whose dead-reckoned estimate (DeadReckoner in control/main.py:
   last camera pose + the commands sent since) is closer than `stop_m` to the edge or outside it, drives
   slowly (`return_speed_m_s`) toward a point `return_inset_m` inside that line, holding its heading and
   still avoiding the user and the other robot. It stops as soon as the camera sees it again (normal control
   resumes) or the estimate says it is back inside.
3. Watchdog. The estimate drifts (mecanum wheels slip), so returning stops for good once the last camera pose
   is older than `return_timeout_s` or the estimate has moved `return_max_m`. The robot then stays stopped
   with a "bring it back by hand" reason until the camera sees it again.

Returning needs a live camera: never with no / old / approximate vision, and only in modes that are already
allowed to drive by themselves (follow, armed waypoints). Keyboard driving is soft-stopped but never returned.
Without a heading (uncalibrated head) the velocity can't be put in floor coordinates, so nothing is limited.
"""

import math

from .controller import remove_approach, world_to_body

STOP = (0.0, 0.0, 0.0)
GUARD_DEFAULTS = {"enabled": True, "stop_m": 0.20, "slow_m": 0.50, "return_speed_m_s": 0.10,
                  "return_inset_m": 0.15, "return_timeout_s": 4.0, "return_max_m": 0.6}


class CameraView:
    """A closed floor polygon [[x, y], ...] with signed-distance helpers (pure Python, ~tens of vertices)."""

    def __init__(self, polygon):
        self.poly = [(float(x), float(y)) for x, y in polygon]
        if len(self.poly) < 3:
            raise ValueError("camera view polygon needs at least 3 points")

    @classmethod
    def from_message(cls, msg):
        poly = ((msg or {}).get("view") or {}).get("polygon")
        try:
            return cls(poly) if poly else None
        except (TypeError, ValueError):
            return None

    def contains(self, x, y):
        inside = False
        p = self.poly
        for (x1, y1), (x2, y2) in zip(p, p[1:] + p[:1]):
            if (y1 > y) != (y2 > y) and x < x1 + (y - y1) * (x2 - x1) / (y2 - y1):
                inside = not inside
        return inside

    def boundary(self, x, y):
        """(signed distance to the edge, positive inside; outward unit normal (nx, ny) at the nearest edge point)."""
        best = None
        p = self.poly
        for (x1, y1), (x2, y2) in zip(p, p[1:] + p[:1]):
            dx, dy = x2 - x1, y2 - y1
            seg = dx * dx + dy * dy
            t = 0.0 if seg == 0 else min(1.0, max(0.0, ((x - x1) * dx + (y - y1) * dy) / seg))
            qx, qy = x1 + t * dx, y1 + t * dy
            d = math.hypot(x - qx, y - qy)
            if best is None or d < best[0]:
                best = (d, qx, qy, dx, dy)
        d, qx, qy, dx, dy = best
        inside = self.contains(x, y)
        if d > 1e-9:
            nx, ny = ((qx - x) / d, (qy - y) / d) if inside else ((x - qx) / d, (y - qy) / d)
        else:  # exactly on the edge: use the edge's own normal, pointing away from the polygon
            length = math.hypot(dx, dy) or 1.0
            nx, ny = dy / length, -dx / length
            if self.contains(x + 1e-3 * nx, y + 1e-3 * ny):
                nx, ny = -nx, -ny
        return (d if inside else -d), (nx, ny)

    def inward_point(self, x, y, depth):
        """A point at least `depth` inside the edge, reached by stepping in along the nearest edge's normal."""
        for _ in range(12):
            d, (nx, ny) = self.boundary(x, y)
            if d >= depth - 1e-3:
                break
            step = depth - d + 0.01
            x, y = x - nx * step, y - ny * step
        return x, y


def soft_limit(wvx, wvy, d, normal, stop_m, slow_m):
    """World velocity with its outward part scaled for a robot `d` inside the edge. Returns (vx, vy, factor)."""
    nx, ny = normal
    out = wvx * nx + wvy * ny
    if out <= 0 or d >= slow_m:
        return wvx, wvy, 1.0
    factor = min(1.0, max(0.0, (d - stop_m) / max(1e-6, slow_m - stop_m)))
    return wvx - (1 - factor) * out * nx, wvy - (1 - factor) * out * ny, factor


def body_to_world(vx, vy, heading_deg):
    h = math.radians(heading_deg)
    c, s = math.cos(h), math.sin(h)
    return c * vx - s * vy, s * vx + c * vy


def _finite(*values):
    return all(isinstance(v, (int, float)) and math.isfinite(v) for v in values)


class ViewGuard:
    """Soft stop at the camera's view edge and return-from-outside for every driven robot (see module doc)."""

    def __init__(self, settings=None):
        self.s = {**GUARD_DEFAULTS, **(settings or {})}
        if not 0 <= self.s["stop_m"] < self.s["slow_m"]:
            raise ValueError("view_guard: need 0 <= stop_m < slow_m")
        self.state = {}  # robot id -> "returning" | "lost" while the camera can't see it

    @property
    def enabled(self):
        return self.s["enabled"] is not False

    def apply(self, msg, results, controllers, reckoners, *, fresh, vision_ok, allow_return, now, user=None):
        """Post-process one control step's {rid: (body command, reason)}.

        fresh: {rid: pose dict} for robots the camera located reliably this step (also fed to the reckoners).
        vision_ok: the vision message is current and calibrated (positions usable for driving).
        allow_return: the current mode may drive by itself (follow, or armed waypoints).
        user: (x, y) of a recently seen user, for the return's keep-out."""
        for rid, pose in fresh.items():
            if rid in reckoners:
                reckoners[rid].observe(pose, now)
            self.state.pop(rid, None)
        view = CameraView.from_message(msg) if vision_ok and self.enabled else None
        if view is None:
            return results
        s = self.s
        out = dict(results)
        for rid, (cmd, reason) in results.items():
            ctrl = controllers[rid]
            pose = fresh.get(rid)
            reck = reckoners.get(rid)
            if pose is None:
                if reck is None or reck.pose is None:
                    continue
                age = now - reck.t
                estimate = reck.estimate(now, max_s=s["return_timeout_s"])
                if estimate is None or reck.travelled > s["return_max_m"]:
                    if self.state.get(rid) in ("returning", "lost"):
                        self.state[rid] = "lost"
                        ctrl.reset()
                        out[rid] = (STOP, f"lost outside the camera view; position estimate too old ({age:.1f} s) "
                                          "— bring it back into view by hand or keyboard")
                    continue
                pose = estimate
                d, normal = view.boundary(pose["x"], pose["y"])
                if d < s["stop_m"]:
                    if not allow_return:
                        continue
                    self.state[rid] = "returning"
                    ctrl.reset()
                    out[rid] = (self._return_command(view, pose, ctrl, rid, user, fresh),
                                f"returning to camera view (estimate {age:.1f} s old, {-d:+.2f} m past the safe edge)")
                    continue
                if self.state.get(rid) == "returning":  # back inside by the estimate but still unseen: wait
                    ctrl.reset()
                    out[rid] = (STOP, f"back inside the camera view by estimate; waiting for the camera ({age:.1f} s)")
                    continue
            # Soft stop (seen robots, and unseen ones moving on a recent estimate).
            if cmd == STOP or pose.get("heading") is None or not _finite(pose.get("x"), pose.get("y")):
                continue
            d, normal = view.boundary(pose["x"], pose["y"])
            if d >= s["slow_m"]:
                continue
            wvx, wvy = body_to_world(cmd[0], cmd[1], pose["heading"])
            if ctrl.limits.allow_strafe:
                lvx, lvy, factor = soft_limit(wvx, wvy, d, normal, s["stop_m"], s["slow_m"])
                vx, vy = world_to_body(lvx, lvy, pose["heading"])
            else:  # straight-drive chassis: scale the forward speed instead of adding a sideways part
                factor = soft_limit(wvx, wvy, d, normal, s["stop_m"], s["slow_m"])[2]
                vx, vy = cmd[0] * factor, cmd[1]
            if factor < 1.0:
                limited = (vx, vy, cmd[2])
                note = "holding at the camera view edge" if factor == 0 else "slowing near the camera view edge"
                out[rid] = (STOP if max(abs(v) for v in limited) < 1e-4 else limited, f"{reason}; {note}")
        return out

    def _return_command(self, view, pose, ctrl, rid, user, fresh):
        s = self.s
        tx, ty = view.inward_point(pose["x"], pose["y"], s["stop_m"] + s["return_inset_m"])
        ex, ey = tx - pose["x"], ty - pose["y"]
        dist = math.hypot(ex, ey)
        if dist < 1e-6:
            return STOP
        speed = min(s["return_speed_m_s"], ctrl.limits.max_speed_m_s)
        wvx, wvy = ex / dist * speed, ey / dist * speed
        lim = ctrl.limits
        obstacles = [(o["x"], o["y"], lim.robot_clearance_m) for oid, o in fresh.items() if oid != rid]
        if user is not None:
            obstacles.append((user[0], user[1], lim.keep_out_m))
        wvx, wvy = remove_approach(wvx, wvy, pose["x"], pose["y"], obstacles)
        vx, vy = world_to_body(wvx, wvy, pose["heading"])
        if not lim.allow_strafe:
            vy = 0.0
        return STOP if math.hypot(vx, vy) < 1e-4 else (vx, vy, 0.0)
