"""Smoothing and bookkeeping for tracked objects.

Each object gets a small alpha-beta filter: a lightweight cousin of the Kalman filter that keeps a
position and a velocity estimate. On each new measurement it predicts where the object should be,
then nudges the prediction toward the measurement:
    alpha = how much to trust a new position (1.0 = no smoothing)
    beta  = how fast the velocity estimate adapts
"""

import math

import numpy as np

from .robots import wrap_deg


class Track:
    def __init__(self, alpha=0.5, beta=0.05, heading_alpha=0.4, lost_after_s=1.0, max_jump_m=0.75,
                 rejects_before_reset=3):
        self.alpha, self.beta, self.heading_alpha = alpha, beta, heading_alpha
        self.lost_after_s, self.max_jump_m = lost_after_s, max_jump_m
        self.rejects_before_reset = rejects_before_reset
        self.pos = None  # np.array([x, y])
        self.vel = np.zeros(2)
        self.heading = None
        self.t = None  # time of the last accepted measurement
        self.rejects = 0

    def _reset(self, t, xy, heading):
        self.pos, self.vel, self.heading, self.t, self.rejects = xy, np.zeros(2), heading, t, 0

    def update(self, t, xy, heading=None):
        """Feed a measurement. Returns False if it was rejected as an outlier."""
        xy = np.asarray(xy, dtype=float)
        if self.pos is None or self.age(t) > self.lost_after_s:
            self._reset(t, xy, heading)
            return True

        dt = max(t - self.t, 1e-3)
        pred = self.pos + self.vel * dt
        residual = xy - pred
        if np.linalg.norm(residual) > self.max_jump_m:
            # A single big jump is probably a bad detection. If it keeps happening, the object really moved.
            self.rejects += 1
            if self.rejects >= self.rejects_before_reset:
                self._reset(t, xy, heading)
                return True
            return False

        self.rejects = 0
        self.pos = pred + self.alpha * residual
        self.vel = self.vel + (self.beta / dt) * residual
        if heading is not None:
            if self.heading is None:
                self.heading = heading
            else:
                self.heading = wrap_deg(self.heading + self.heading_alpha * wrap_deg(heading - self.heading))
        self.t = t
        return True

    def age(self, t):
        return math.inf if self.t is None else t - self.t


class UserTracker:
    """Tracks the one user. Position is smoothed like the robots; height and confidence with a simple average."""

    def __init__(self, tracking_cfg=None):
        c = dict(tracking_cfg or {})
        self.forget_after_s = c.pop("forget_after_s", 5.0)
        self.track = Track(**c)
        self.height, self.confidence = None, None

    def update(self, t, observations):
        from .people import select_user

        o = select_user(observations, self.track, t, self.track.lost_after_s)
        if o is None:
            return None
        fresh = self.track.age(t) > self.track.lost_after_s
        if self.track.update(t, (o.x, o.y)):
            a = 1.0 if fresh or self.height is None else 0.3
            self.height = o.height if self.height is None else (1 - a) * self.height + a * o.height
            self.confidence = o.confidence
        return o

    def state(self, t):
        """User dict for the output message, or None if not seen recently."""
        age = self.track.age(t)
        if age > self.forget_after_s:
            return None
        p, v = self.track.pos, self.track.vel
        return {"x": float(p[0]), "y": float(p[1]), "height": self.height, "vx": float(v[0]),
                "vy": float(v[1]), "age": age, "confidence": self.confidence}


class RobotTracker:
    def __init__(self, robot_ids, tracking_cfg=None):
        c = dict(tracking_cfg or {})
        self.forget_after_s = c.pop("forget_after_s", 5.0)
        self.tracks = {rid: Track(**c) for rid in robot_ids}

    def update(self, t, observations):
        for o in observations:
            if o.robot_id in self.tracks:
                self.tracks[o.robot_id].update(t, (o.x, o.y), o.heading_deg)

    def state(self, t):
        """Robots seen within forget_after_s, as plain dicts for the output message."""
        out = []
        for rid, tr in sorted(self.tracks.items()):
            age = tr.age(t)
            if age <= self.forget_after_s:
                out.append({"id": rid, "x": float(tr.pos[0]), "y": float(tr.pos[1]),
                            "heading": tr.heading, "vx": float(tr.vel[0]), "vy": float(tr.vel[1]),
                            "age": age})
        return out
