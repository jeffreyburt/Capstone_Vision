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

No-strafe mode (`allow_strafe: false`): for a base that can drive straight and turn but can't move sideways
reliably (robot 2's strafe turns into rotation). The robot turns toward the target (or away from it and
drives in reverse, whichever is the smaller turn), drives straight once roughly aligned, and only at the
target turns to face the user. It never sends a sideways speed.

The camera pose arrives a few hundred ms late (frame rate, smoothing, YOLO), so the gains are deliberately
low. A proportional loop with gain k and delay T stays well behaved while k*T is well under 1.
"""

import math
from dataclasses import dataclass


@dataclass
class Limits:
    max_speed_m_s: float = 0.15
    keyboard_max_speed_m_s: float = 0.0  # WASD-only cap; 0 means use max_speed_m_s
    max_yaw_rad_s: float = 0.4
    position_gain: float = 0.8      # (m/s) per m of error
    yaw_gain: float = 1.0           # (rad/s) per rad of error
    position_tolerance_m: float = 0.05
    yaw_tolerance_deg: float = 5.0
    keep_out_m: float = 0.8         # never move toward the user when closer than this
    robot_clearance_m: float = 0.6  # ...or toward another robot when closer than this (center to center)
    allow_strafe: bool = True       # False: turn-then-drive, never sideways (no-strafe mode)
    allow_reverse: bool = True      # no-strafe mode may drive backward instead of turning around
    drive_heading_deadband_deg: float = 0.0
    drive_alignment_release_deg: float = 0.0
    reverse_hysteresis_deg: float = 0.0
    drive_align_deg: float = 25.0   # no-strafe mode: only drive while pointing within this of the travel line
    no_strafe_restart_m: float = 0.15  # no-strafe mode: once parked, only move again past this error

    @classmethod
    def from_config(cls, d):
        out = {}
        for k, v in (d or {}).items():
            if k in cls.__dataclass_fields__:
                out[k] = bool(v) if isinstance(cls.__dataclass_fields__[k].default, bool) else float(v)
        return cls(**out)


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
        self.travel_aligned = False
        self.travel_reverse = None

    def reset(self):
        self.position_settled = self.heading_settled = False
        self.travel_aligned = False
        self.travel_reverse = None

    def _turn_toward(self, error_rad, deadband_deg=0.0):
        L = self.limits
        deadband = math.radians(deadband_deg)
        error_rad = math.copysign(max(0.,abs(error_rad)-deadband),error_rad)
        return max(-L.max_yaw_rad_s, min(L.max_yaw_rad_s, L.yaw_gain * error_rad))

    def _face_target_heading(self, pose, target):
        """Turn in place to the target heading, with the heading deadband. Returns wz."""
        L = self.limits
        eh = wrap_rad(math.radians(target.heading_deg - pose["heading"]))
        tol = math.radians(L.yaw_tolerance_deg)
        if self.heading_settled and abs(eh) > 2 * tol:
            self.heading_settled = False
        elif not self.heading_settled and abs(eh) <= tol:
            self.heading_settled = True
        return 0.0 if self.heading_settled else self._turn_toward(eh)

    def _command_no_strafe(self, pose, target, obstacles):
        L = self.limits
        ex, ey = target.x - pose["x"], target.y - pose["y"]
        dist = math.hypot(ex, ey)
        restart = max(2 * L.position_tolerance_m, L.no_strafe_restart_m)
        if self.position_settled and dist > restart:
            self.position_settled = False
        elif not self.position_settled and dist <= L.position_tolerance_m:
            self.position_settled = True
        if self.position_settled:  # parked: only turn to face the user
            return 0.0, 0.0, self._face_target_heading(pose, target)

        self.heading_settled = False  # face the user again after this leg
        h = math.radians(pose["heading"])
        bearing = math.atan2(ey, ex)
        ahead, behind = wrap_rad(bearing - h), wrap_rad(bearing + math.pi - h)
        reverse = L.allow_reverse and abs(behind) < abs(ahead)
        if L.reverse_hysteresis_deg > 0:
            if self.travel_reverse is None:
                self.travel_reverse = reverse
            elif L.allow_reverse:
                kept,other = (behind,ahead) if self.travel_reverse else (ahead,behind)
                if abs(other)+math.radians(L.reverse_hysteresis_deg) < abs(kept):
                    self.travel_reverse = not self.travel_reverse
            reverse = self.travel_reverse
        err = behind if reverse else ahead
        speed = 0.0
        angle = abs(math.degrees(err))
        if L.drive_alignment_release_deg > L.drive_align_deg:
            if not self.travel_aligned and angle < L.drive_align_deg:
                self.travel_aligned = True
            elif self.travel_aligned and angle > L.drive_alignment_release_deg:
                self.travel_aligned = False
            blend = max(0., min(1., (L.drive_alignment_release_deg-angle)/
                                (L.drive_alignment_release_deg-L.drive_align_deg)))
            blend = blend*blend*(3-2*blend)
            if self.travel_aligned:
                speed = min(L.max_speed_m_s,L.position_gain*dist)*max(0.,math.cos(err))*blend
        elif angle < L.drive_align_deg:
            speed = min(L.max_speed_m_s,L.position_gain*dist)*math.cos(err)
        v = -speed if reverse else speed
        # Keep-out: remove motion toward the user/robots, then keep only what a straight drive can do.
        wvx, wvy = remove_approach(v * math.cos(h), v * math.sin(h), pose["x"], pose["y"], obstacles)
        vx = wvx * math.cos(h) + wvy * math.sin(h)
        return vx, 0.0, self._turn_toward(err,L.drive_heading_deadband_deg)

    def command(self, pose, target, obstacles=()):
        """pose: dict with x, y, heading (deg), as in the vision message. target: planner.Target.
        obstacles: [(x, y, radius)] to keep out of. Returns body (vx forward, vy left, wz CCW)."""
        L = self.limits
        if not L.allow_strafe:
            return self._command_no_strafe(pose, target, obstacles)
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
        wz = self._face_target_heading(pose, target)
        vx, vy = world_to_body(wvx, wvy, pose["heading"])
        return vx, vy, wz
