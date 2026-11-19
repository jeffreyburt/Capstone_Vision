"""Per-robot position + heading controller, run on the laptop from the camera's pose.

Each control step:
  1. Position error in the room (world) frame -> a world velocity toward the target, proportional to the
     distance and capped at max_speed_m_s.
  2. Keep-out: near the user (or another robot), the part of that velocity that points *toward* them is
     removed, so a robot slides around a person instead of driving into them.
  3. Heading error -> a turn rate, proportional and capped.
  4. The world velocity is rotated into the robot's body frame (forward / left) using the camera heading,
     because that is what the wheels understand.

Deadbands with hysteresis: once a robot is within tolerance it stops, and only starts again when the error
grows to twice the tolerance. That stops it twitching on camera noise.

The camera pose arrives a few hundred ms late (frame rate, smoothing, YOLO), so the gains are deliberately
low. A proportional loop with gain k and delay T stays well behaved while k*T is well under 1.
"""

import math
from dataclasses import dataclass


@dataclass
class Limits:
    max_speed_m_s: float = 0.15
    max_yaw_rad_s: float = 0.4
    position_gain: float = 0.8      # (m/s) per m of error
    yaw_gain: float = 1.0           # (rad/s) per rad of error
    position_tolerance_m: float = 0.05
    yaw_tolerance_deg: float = 5.0
    keep_out_m: float = 0.8         # never move toward the user when closer than this
    robot_clearance_m: float = 0.6  # ...or toward another robot when closer than this (center to center)

    @classmethod
    def from_config(cls, d):
        return cls(**{k: float(v) for k, v in (d or {}).items() if k in cls.__dataclass_fields__})


def wrap_rad(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def world_to_body(vx, vy, heading_deg):
    """World-frame velocity -> (forward, left) for a robot whose front points along heading_deg."""
    h = math.radians(heading_deg)
    c, s = math.cos(h), math.sin(h)
    return c * vx + s * vy, -s * vx + c * vy


def remove_approach(vx, vy, x, y, obstacles):
    """Drop the velocity component toward any obstacle (ox, oy, radius) the robot is inside of."""
    for ox, oy, radius in obstacles:
        dx, dy = x - ox, y - oy
        r = math.hypot(dx, dy)
        if r >= radius or r < 1e-6:
            continue
        nx, ny = dx / r, dy / r  # unit vector pointing away from the obstacle
        toward = vx * nx + vy * ny
        if toward < 0:
            vx, vy = vx - toward * nx, vy - toward * ny
    return vx, vy


class RobotController:
    """Holds the deadband state for one robot."""

    def __init__(self, limits=None):
        self.limits = limits or Limits()
        self.position_settled = False
        self.heading_settled = False

    def reset(self):
        self.position_settled = self.heading_settled = False

    def command(self, pose, target, obstacles=()):
        """pose: dict with x, y, heading (deg), as in the vision message. target: planner.Target.
        obstacles: [(x, y, radius)] to keep out of. Returns body (vx forward, vy left, wz CCW)."""
        L = self.limits
        ex, ey = target.x - pose["x"], target.y - pose["y"]
        dist = math.hypot(ex, ey)
        if self.position_settled and dist > 2 * L.position_tolerance_m:
            self.position_settled = False
        elif not self.position_settled and dist <= L.position_tolerance_m:
            self.position_settled = True
        if self.position_settled:
            wvx = wvy = 0.0
        else:
            speed = min(L.max_speed_m_s, L.position_gain * dist)
            wvx, wvy = ex / dist * speed, ey / dist * speed
        wvx, wvy = remove_approach(wvx, wvy, pose["x"], pose["y"], obstacles)

        eh = wrap_rad(math.radians(target.heading_deg - pose["heading"]))
        tol = math.radians(L.yaw_tolerance_deg)
        if self.heading_settled and abs(eh) > 2 * tol:
            self.heading_settled = False
        elif not self.heading_settled and abs(eh) <= tol:
            self.heading_settled = True
        wz = 0.0 if self.heading_settled else max(-L.max_yaw_rad_s, min(L.max_yaw_rad_s, L.yaw_gain * eh))

        vx, vy = world_to_body(wvx, wvy, pose["heading"])
        return vx, vy, wz
