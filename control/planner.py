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
                 area_margin_m=0.3):
        """sides: {robot_id: "left" | "right" | "center"}, from the user's point of view.
        area: {"x": [min, max], "y": [min, max]} floor rectangle the targets must stay inside (or None)."""
        for rid, side in sides.items():
            if side not in ("left", "right", "center"):
                raise ValueError(f"robot {rid}: side must be left, right or center, not {side!r}")
        self.sides = dict(sides)
        self.distance_m, self.spread_deg, self.facing_deg = distance_m, spread_deg, facing_deg
        self.replan_m = replan_m
        self.area, self.area_margin_m = area, area_margin_m
        self.anchor = None  # user position the current targets were computed for
        self.targets = {}

    def _clamp(self, x, y):
        if not self.area:
            return x, y
        m = self.area_margin_m
        (x0, x1), (y0, y1) = self.area["x"], self.area["y"]
        return min(max(x, x0 + m), x1 - m), min(max(y, y0 + m), y1 - m)

    def targets_for(self, ux, uy):
        """Targets for a user standing at (ux, uy), ignoring hysteresis."""
        offsets = {"left": self.spread_deg / 2, "right": -self.spread_deg / 2, "center": 0.0}
        out = {}
        for rid, side in self.sides.items():
            a = math.radians(self.facing_deg + offsets[side])  # left = counter-clockwise of the facing direction
            x, y = self._clamp(ux + self.distance_m * math.cos(a), uy + self.distance_m * math.sin(a))
            out[rid] = Target(x, y, math.degrees(math.atan2(uy - y, ux - x)))
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
