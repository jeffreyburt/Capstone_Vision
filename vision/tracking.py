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
        self.heading_t = None  # time of the last measurement that carried a heading
        self.rejects = 0

    def _reset(self, t, xy, heading):
        self.pos, self.vel, self.t, self.rejects = xy, np.zeros(2), t, 0
        if heading is not None or self.heading_t is None or t - self.heading_t > self.lost_after_s:
            self.heading, self.heading_t = heading, (t if heading is not None else None)

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
            self.heading_t = t
            if self.heading is None:
                self.heading = heading
            else:
                self.heading = wrap_deg(self.heading + self.heading_alpha * wrap_deg(heading - self.heading))
        self.t = t
        return True

    def age(self, t):
        return math.inf if self.t is None else t - self.t

    def heading_age(self, t):
        """Seconds since a measurement last carried a heading (a tag-centre fix moves the position only)."""
        return math.inf if self.heading_t is None else t - self.heading_t


class UserTracker:
    """Tracks the one user. Position is smoothed like the robots; height and confidence with a simple average.

    Who the user is (see select): only people inside `area` (the hallway) count. While the user is tracked, other
    people are ignored: only a detection within the jump gate of the predicted position is the user. After the
    user is lost, someone near the last position is preferred, the gate growing at walk_speed_m_s, until
    forget_after_s. With no user, the person nearest the camera becomes the user once seen in confirm_frames
    consecutive detections (not the most confident one: that was often a fully visible person far behind)."""

    def __init__(self, tracking_cfg=None, area=None):
        c = dict(tracking_cfg or {})
        self.forget_after_s = c.pop("forget_after_s", 5.0)
        self.walk_speed_m_s = c.pop("walk_speed_m_s", 0.5)
        self.confirm_frames = int(c.pop("confirm_frames", 2))
        self.track = Track(**c)
        self.area = area
        self.height, self.confidence = None, None
        self.candidate = None  # (x, y, consecutive count, t) of a possible new user

    def select(self, observations, t):
        from .people import in_area

        obs = [o for o in observations if in_area(o, self.area)]
        tr = self.track
        if tr.pos is not None and tr.age(t) <= self.forget_after_s:
            age = tr.age(t)
            pred = tr.pos + tr.vel * min(age, tr.lost_after_s)
            gate = tr.max_jump_m + self.walk_speed_m_s * max(0.0, age - tr.lost_after_s)
            dist = {id(o): math.hypot(o.x - pred[0], o.y - pred[1]) for o in obs}
            near = [o for o in obs if dist[id(o)] <= gate]
            return min(near, key=lambda o: dist[id(o)]) if near else None
        if not obs:
            self.candidate = None
            return None
        best = min(obs, key=lambda o: math.hypot(o.x, o.y))
        c = self.candidate
        count = c[2] + 1 if c and t - c[3] <= 1.0 and math.hypot(best.x - c[0], best.y - c[1]) <= 0.6 else 1
        self.candidate = (best.x, best.y, count, t)
        return best if count >= self.confirm_frames else None

    def update(self, t, observations):
        o = self.select(observations, t)
        if o is None:
            return None
        if self.track.age(t) > self.track.lost_after_s:
            self.track.pos = None  # (re)acquired: start fresh from this person, no velocity from before
        if self.track.update(t, (o.x, o.y)):
            self.candidate = None
            if o.height is not None and 1.0 <= o.height <= 2.2:
                self.height = o.height if self.height is None else 0.8 * self.height + 0.2 * o.height
            self.confidence = o.confidence
        return o

    def head_height(self, default):
        """Learned user height for placing a person by the head when their feet are hidden or out of view."""
        return self.height if self.height is not None else default

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
                            "age": age, "heading_age": (round(tr.heading_age(t), 3)
                                                        if tr.heading is not None else None)})
        return out
