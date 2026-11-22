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
                 area_margin_m=0.3, x_limits=None, y_limits=None, max_turn_deg=None, line_y_m=None, min_side_m=0.8):
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
        # Line mode: robots stay on a line line_y_m in front of the camera and the user is the third corner of an
        # equilateral triangle, so the triangle opens as the user walks away and closes as they come nearer. When the
        # robots would cross x_limits (or y_limits), the triangle stops growing and they step off the line toward the
        # user instead, still equilateral; they return to the line as soon as it fits. Never smaller than min_side_m.
        self.line_y_m, self.min_side_m = line_y_m, min_side_m
        for lim in (x_limits, y_limits):
            if lim is not None and not (len(lim) == 2 and lim[0] < lim[1]):
                raise ValueError("x_limits / y_limits must be [min, max] with min < max")
        # Camera view (control.view_guard.CameraView) the targets must stay view_margin_m inside; set each step by
        # control.main from the vision message. Like the walls, it shrinks the triangle for BOTH robots (equal
        # distances to the user) instead of pulling one robot in on its own.
        self.view, self.view_margin_m = None, 0.35
        self.anchor = None  # user position the current targets were computed for
        self.targets = {}
        self.swapped = False
        self.jumps = 0  # consecutive steps with the user far from the planned position (see update)  # two robots trade sides rather than cross each other (see assign)

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

    VIEW_STEP_M = 0.02

    def set_view(self, view, margin_m=None):
        """New camera view (or None). A changed view re-plans on the next update."""
        poly = None if view is None else view.poly
        if margin_m is not None:
            self.view_margin_m = margin_m
        if poly != (None if self.view is None else self.view.poly):
            self.view, self.anchor = view, None

    def _offsets(self):
        return {"left": self.spread_deg / 2, "right": -self.spread_deg / 2, "center": 0.0}

    def _in_view(self, x, y):
        return self.view is None or self.view.boundary(x, y)[0] >= self.view_margin_m - 1e-9

    def _angles(self, rot_deg=0.0):
        """Direction (radians) from the user to each side's robot; rot_deg turns the whole triangle about the user."""
        offsets = self._offsets()
        return {s: math.radians(self.facing_deg + offsets[s] + rot_deg) for s in set(self.sides.values())}

    def _fits(self, ux, uy, side, rot_deg=0.0):
        """Every robot's spot at this triangle side is inside the camera view (walls are handled by side_for)."""
        for a in self._angles(rot_deg).values():
            x, y = self._clamp(ux + side * math.cos(a), uy + side * math.sin(a))
            if not self._in_view(x, y):
                return False
        return True

    def side_for(self, ux, uy, rot_deg=0.0):
        """Triangle side (user-to-robot distance) for a user at (ux, uy), the triangle turned rot_deg."""
        angles = list(self._angles(rot_deg).values())
        if self.line_y_m is None:
            side = self.distance_m
        else:
            # Distance from the user to the robots' line along the robots' direction from the user.
            depth = max(-math.sin(a) for a in angles)
            side = (uy - self.line_y_m) / depth if depth > 1e-6 else self.distance_m
            for a in angles:  # largest side that keeps every robot inside the limits (then they move off the line)
                for lim, d, u in ((self.x_limits, math.cos(a), ux), (self.y_limits, math.sin(a), uy)):
                    if lim and d > 1e-6:
                        side = min(side, (lim[1] - u) / d)
                    elif lim and d < -1e-6:
                        side = min(side, (lim[0] - u) / d)
            side = max(side, self.min_side_m)
        if self.view is not None and not self._fits(ux, uy, side, rot_deg):
            # A spot outside the camera view: bring BOTH robots in (one can't go further back, so the other comes
            # forward too) until both fit, keeping their distances to the user equal.
            # Coarse steps down to the first size that fits, then fine steps back up to the largest.
            s = side
            while s > self.min_side_m + 1e-9:
                s = max(self.min_side_m, s - 5 * self.VIEW_STEP_M)
                if self._fits(ux, uy, s, rot_deg):
                    while s + self.VIEW_STEP_M < side and self._fits(ux, uy, s + self.VIEW_STEP_M, rot_deg):
                        s += self.VIEW_STEP_M
                    return s
        return side

    MAX_ROTATION_DEG = 60
    ROTATION_STEP_DEG = 5

    def layout(self, ux, uy):
        """(side, rotation) of the triangle. Normally the full size, unrotated. When a robot's spot is outside the
        camera view the triangle first shrinks (both robots, equal distances); if even min_side_m doesn't fit (the
        user near the edge of the view, where it is narrow), it also turns about the user, the least that fits.
        Nothing fits: full size, unrotated, and control.main pulls the outside robot into the view on its own."""
        side = self.side_for(ux, uy)
        if self.view is None or self._fits(ux, uy, side):
            return side, 0.0
        for d in range(self.ROTATION_STEP_DEG, self.MAX_ROTATION_DEG + 1, self.ROTATION_STEP_DEG):
            fits = [(s, r) for r in (d, -d) for s in [self.side_for(ux, uy, r)] if self._fits(ux, uy, s, r)]
            if fits:
                return max(fits)  # of the two directions, the larger triangle
        return side, 0.0

    def plan_x(self, ux, uy):
        """The user's x for planning: kept far enough from the walls that the smallest triangle still fits between
        them. Otherwise a user standing near (or reported past) a wall clamps both robots onto the same spot."""
        if not self.x_limits:
            return ux
        offsets = {"left": self.spread_deg / 2, "right": -self.spread_deg / 2, "center": 0.0}
        dx = [self.min_side_m * math.cos(math.radians(self.facing_deg + offsets[s])) for s in set(self.sides.values())]
        lo, hi = self.x_limits[0] - min(dx + [0.0]), self.x_limits[1] - max(dx + [0.0])
        return ux if lo > hi else min(max(ux, lo), hi)

    def targets_for(self, ux, uy):
        """Targets for a user standing at (ux, uy), ignoring hysteresis."""
        ux = self.plan_x(ux, uy)
        side, rot = self.layout(ux, uy)
        angles = self._angles(rot)
        out = {}
        for rid, s in self.sides.items():
            a = angles[s]  # left = counter-clockwise of the facing direction
            x, y = self._clamp(ux + side * math.cos(a), uy + side * math.sin(a))
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
        if self.anchor is not None and math.hypot(ux - self.anchor[0], uy - self.anchor[1]) > self.JUMP_M:
            # A user "jumping" metres in one step is usually a bad detection (e.g. y 2.4 -> 7.5 for one frame):
            # keep the current targets unless the new position holds for JUMP_CONFIRM steps.
            self.jumps += 1
            if self.jumps < self.JUMP_CONFIRM:
                return self.targets
            force = True
        self.jumps = 0
        if force or self.anchor is None or math.hypot(ux - self.anchor[0], uy - self.anchor[1]) > self.replan_m:
            self.anchor = (ux, uy)
            self.targets = self.targets_for(ux, uy)
        return self.targets

    JUMP_M = 1.5
    JUMP_CONFIRM = 5
    SWAP_GAIN_M = 0.5  # hysteresis: the other pairing must save this much total driving to switch

    def assign(self, targets, poses):
        """With two robots, give each the target nearer to it when that saves real driving: a robot that ends up on
        the other side (after a lost track, or the user stepping past) then trades places with its partner instead
        of both driving across the user and each other. poses: {robot_id: {"x", "y"}} of robots seen recently.
        Sticky: once swapped it stays swapped until the original pairing is clearly better again."""
        if len(targets) != 2 or len(self.sides) != 2:
            return targets
        a, b = sorted(targets)
        if a in poses and b in poses:
            def dist(rid, t):
                return math.hypot(poses[rid]["x"] - t.x, poses[rid]["y"] - t.y)
            straight = dist(a, targets[a]) + dist(b, targets[b])
            crossed = dist(a, targets[b]) + dist(b, targets[a])
            if not self.swapped and crossed < straight - self.SWAP_GAIN_M:
                self.swapped = True
            elif self.swapped and straight < crossed - self.SWAP_GAIN_M:
                self.swapped = False
        self.jumps = 0  # consecutive steps with the user far from the planned position (see update)
        return {a: targets[b], b: targets[a]} if self.swapped else targets
