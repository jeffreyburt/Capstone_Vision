"""Point targets and closed-loop navigation. HTTP targets never command wheels directly."""

import json
import math
import threading
import urllib.request

from .planner import Target

STOP = (0., 0., 0.)


class WaypointStore:
    def __init__(self, robot_ids, area=None):
        self.robot_ids, self.area = set(robot_ids), area
        self.lock = threading.Lock()
        self.targets = {}

    def set(self, robot_id, x, y, *, preview_only=True):
        if type(robot_id) is not int or robot_id not in self.robot_ids:
            raise ValueError("Select a configured robot")
        if any(type(v) not in (int, float) or not math.isfinite(v) for v in (x, y)):
            raise ValueError("Target coordinates must be finite numbers")
        if not self.area or not (self.area['x'][0] <= x <= self.area['x'][1] and
                                 self.area['y'][0] <= y <= self.area['y'][1]):
            raise ValueError("Target is outside the configured working area")
        with self.lock:
            self.targets[str(robot_id)] = {"x": float(x), "y": float(y), "preview_only": bool(preview_only)}

    def clear(self):
        with self.lock:
            self.targets.clear()

    def snapshot(self):
        with self.lock:
            return {k: dict(v) for k, v in self.targets.items()}


def read_waypoints(url):
    try:
        with urllib.request.urlopen(url, timeout=.2) as response:
            return json.load(response)
    except (OSError, ValueError):
        return None  # Never reuse old targets when the target service disappears.


def compute_waypoints(msg, age, navigation, controllers, timing=None):
    timing = {"vision_timeout_s": 1., "max_pose_age_s": .7, **(timing or {})}
    poses = {r['id']: r for r in (msg or {}).get('robots', [])}
    goals = (navigation or {}).get('targets') or {}
    targets = {}
    for rid in controllers:
        goal = goals.get(str(rid))
        if goal:
            targets[rid] = Target(goal['x'], goal['y'], poses.get(rid, {}).get('heading'))
    blocked = None
    if navigation is None:
        blocked = "waypoint service unavailable"
    elif msg is None or age is None or age > timing['vision_timeout_s']:
        blocked = "no vision"
    elif msg.get('position_quality') == 'approximate' or msg.get('driving_allowed') is False:
        blocked = "approximate positions — driving disabled"
    results = {}
    for rid, ctrl in controllers.items():
        pose = poses.get(rid)
        reason = blocked
        if reason is None and rid not in targets:
            reason = "no target"
        if reason is None and goals[str(rid)].get('preview_only', True):
            reason = 'preview target only; select a new destination after navigation is ready'
        if reason is None and (not pose or pose.get('heading') is None or pose.get('age', 999) > timing['max_pose_age_s']):
            reason = "fresh chassis position and heading required"
        readiness = ((navigation or {}).get('readiness') or {}).get(str(rid), {})
        if reason is None and not readiness.get('ready'):
            reason = readiness.get('reason', 'navigation readiness unavailable')
        if reason:
            ctrl.reset()
            results[rid] = (STOP, reason)
            continue
        # Recheck compensated pose on the controller's own fresh vision input.
        if not pose.get('pose_compensated') or age > .25:
            ctrl.reset()
            results[rid] = (STOP, 'compensated chassis pose required')
            continue
        user = msg.get('user')
        obstacles = [(r['x'], r['y'], ctrl.limits.robot_clearance_m) for r in poses.values()
                     if r['id'] != rid and r.get('age', 999) <= timing['max_pose_age_s']]
        if user and user.get('age', 999) <= 2:
            obstacles.append((user['x'], user['y'], ctrl.limits.keep_out_m))
        cmd = ctrl.command(pose, targets[rid], obstacles)
        results[rid] = (cmd, 'holding' if cmd == STOP else 'moving to waypoint')
    return targets, results
