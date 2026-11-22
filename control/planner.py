"""Where the speakers should go: a listening triangle around the user.

The user stands at one corner of the triangle and the two speakers at the other two. Both speakers are
`distance_m` from the user, `spread_deg` apart as seen from the user (60 = equilateral triangle), centered
on the direction the user faces. We don't know which way a person is really facing, so `facing_deg` is a
fixed direction in the room (degrees CCW from +x). Facing the camera (-90 with the default floor frame,
where +y points away from the camera) keeps the robots between the user and the camera, where the
markers are closest and largest in the image.

Each robot's target heading points its front (the speaker) at the user.

Targets only move when the user has moved more than `replan_m` from where they were when the targets were
last computed. That lets the robots settle and stop instead of chasing every small sway.
"""

import math
from dataclasses import dataclass


@dataclass
class Target:
    x: float
    y: float
    heading_deg: float


class FormationPlanner:
    def __init__(self, sides, distance_m=1.5, spread_deg=60.0, facing_deg=-90.0, replan_m=0.3, area=None,
                 area_margin_m=0.3, x_limits=None, y_limits=None, max_turn_deg=None):
        """sides: {robot_id: "left" | "right" | "center"}, from the user's point of view.
        area: {"x": [min, max], "y": [min, max]} floor rectangle the targets must stay inside (or None)."""
        for rid, side in sides.items():
            if side not in ("left", "right", "center"):
                raise ValueError(f"robot {rid}: side must be left, right or center, not {side!r}")
        self.sides = dict(sides)
        self.distance_m, self.spread_deg, self.facing_deg = distance_m, spread_deg, facing_deg
        self.replan_m = replan_m
        self.area, self.area_margin_m = area, area_margin_m
        # Hard bounds on robot centers, e.g. a hallway: [min, max] meters, or None. Applied after the area margin.
        self.x_limits, self.y_limits = x_limits, y_limits
        # Robots face the user, but turn at most this far from facing straight back along facing_deg (None = no limit).
        # Keeps the camera-side markers nearly square-on; a tracking head covers the rest of the angle.
        self.max_turn_deg = max_turn_deg
        for lim in (x_limits, y_limits):
            if lim is not None and not (len(lim) == 2 and lim[0] < lim[1]):
                raise ValueError("x_limits / y_limits must be [min, max] with min < max")
        self.anchor = None  # user position the current targets were computed for
        self.targets = {}

    def _clamp(self, x, y):
        if self.area:
            m = self.area_margin_m
            (x0, x1), (y0, y1) = self.area["x"], self.area["y"]
            x, y = min(max(x, x0 + m), x1 - m), min(max(y, y0 + m), y1 - m)
        if self.x_limits:
            x = min(max(x, self.x_limits[0]), self.x_limits[1])
        if self.y_limits:
            y = min(max(y, self.y_limits[0]), self.y_limits[1])
        return x, y

    def targets_for(self, ux, uy):
        """Targets for a user standing at (ux, uy), ignoring hysteresis."""
        offsets = {"left": self.spread_deg / 2, "right": -self.spread_deg / 2, "center": 0.0}
        out = {}
        for rid, side in self.sides.items():
            a = math.radians(self.facing_deg + offsets[side])  # left = counter-clockwise of the facing direction
            x, y = self._clamp(ux + self.distance_m * math.cos(a), uy + self.distance_m * math.sin(a))
            heading = math.degrees(math.atan2(uy - y, ux - x))
            if self.max_turn_deg is not None:
                base = self.facing_deg + 180
                off = (heading - base + 180) % 360 - 180
                heading = base + max(-self.max_turn_deg, min(self.max_turn_deg, off))
            out[rid] = Target(x, y, heading)
        return out

    def update(self, user, force=False):
        """user: (x, y) or None. Returns {robot_id: Target}, or {} if there is no user to plan around.
        force: re-plan now regardless of replan_m (used while the user is walking, so targets move smoothly
        with them instead of in replan_m steps)."""
        if user is None:
            return {}
        ux, uy = user
        if force or self.anchor is None or math.hypot(ux - self.anchor[0], uy - self.anchor[1]) > self.replan_m:
            self.anchor = (ux, uy)
            self.targets = self.targets_for(ux, uy)
        return self.targets
