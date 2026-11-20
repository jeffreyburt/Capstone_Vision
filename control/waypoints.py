"""Point targets and closed-loop navigation. HTTP targets never command wheels directly."""

import json
import math
import threading
import time
import urllib.request

from .planner import Target

STOP = (0., 0., 0.)


class WaypointStore:
    def __init__(self, robot_ids, area=None):
        self.robot_ids, self.area = set(robot_ids), area
        self.lock = threading.Lock()
        self.targets = {}
        self.revision = 0

    def set(self, robot_id, x, y, *, preview_only=True):
        if type(robot_id) is not int or robot_id not in self.robot_ids:
            raise ValueError("Select a configured robot")
        if any(type(v) not in (int, float) or not math.isfinite(v) for v in (x, y)):
            raise ValueError("Target coordinates must be finite numbers")
        if not self.area or not (self.area['x'][0] <= x <= self.area['x'][1] and
                                 self.area['y'][0] <= y <= self.area['y'][1]):
            raise ValueError("Target is outside the configured working area")
        with self.lock:
            self.revision += 1
            self.targets[str(robot_id)] = {"x": float(x), "y": float(y), "preview_only": bool(preview_only),
                                           "revision": self.revision}

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


class WaypointProgress:
    """Latch a stop when translation fails to reduce camera distance; a new click resets it."""
    def __init__(self, timeout_s=6., improvement_m=.03):
        self.timeout_s, self.improvement_m = timeout_s, improvement_m
        self.key = None
        self.fault = False
        self.best = self.since = None

    def check(self, goal, distance, translating, now):
        key = (goal.get('revision'), goal['x'], goal['y'])
        if key != self.key:
            self.key, self.fault, self.best, self.since = key, False, distance, now
        if self.fault:
            return False
        if not translating or distance <= self.best - self.improvement_m:
            self.best, self.since = distance, now
        elif now - self.since >= self.timeout_s:
            self.fault = True
        return not self.fault

    def pause(self):
        # Missing vision must stop immediately, and must not count as commanded translation.
        self.since = None


def waypoint_feedback(msg, frame_age, goals, robot_ids):
    """Display measured versus desired positions, including approximate data with explicit quality."""
    poses = {r['id']: r for r in (msg or {}).get('robots', [])}
    output = {}
    for rid in robot_ids:
        pose, goal = poses.get(rid), goals.get(str(rid))
        valid = pose is not None and all(isinstance(pose.get(k), (int, float)) and math.isfinite(pose[k])
                                         for k in ('x', 'y'))
        age = None if not valid or frame_age is None else max(0., frame_age) + max(0., pose.get('age', 0))
        error = None if not valid or not goal else [goal['x']-pose['x'], goal['y']-pose['y']]
        output[str(rid)] = dict(actual=[pose['x'], pose['y']] if valid else None,
            target=[goal['x'], goal['y']] if goal else None, error=error,
            distance_m=math.hypot(*error) if error else None, tracking_age_s=age,
            fresh=age is not None and age <= .25,
            quality=(msg or {}).get('position_quality', 'calibrated' if valid else 'unavailable'),
            reference=(pose or {}).get('reference', 'chassis'),
            heading=(pose or {}).get('heading'), revision=(goal or {}).get('revision'))
    return output


def compute_waypoints(msg, age, navigation, controllers, timing=None, *, progress=None, now=None):
    now = time.monotonic() if now is None else now
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
            if progress is not None:
                progress[rid].pause()
            ctrl.reset()
            results[rid] = (STOP, reason)
            continue
        # Recheck compensated pose on the controller's own fresh vision input.
        if not pose.get('pose_compensated') or age > .25:
            if progress is not None:
                progress[rid].pause()
            ctrl.reset()
            results[rid] = (STOP, 'compensated chassis pose required')
            continue
        user = msg.get('user')
        obstacles = [(r['x'], r['y'], ctrl.limits.robot_clearance_m) for r in poses.values()
                     if r['id'] != rid and r.get('age', 999) <= timing['max_pose_age_s']]
        if user and user.get('age', 999) <= 2:
            obstacles.append((user['x'], user['y'], ctrl.limits.keep_out_m))
        cmd = ctrl.command(pose, targets[rid], obstacles)
        if progress is not None:
            guard = progress[rid]
            if guard.since is None:
                guard.best = math.hypot(targets[rid].x-pose['x'], targets[rid].y-pose['y'])
                guard.since = now
            distance = math.hypot(targets[rid].x-pose['x'], targets[rid].y-pose['y'])
            if not guard.check(goals[str(rid)], distance, math.hypot(*cmd[:2]) > .001, now):
                ctrl.reset()
                results[rid] = (STOP, 'no camera progress; stopped — select a new destination to retry')
                continue
        results[rid] = (cmd, 'holding' if cmd == STOP else 'moving to waypoint')
    return targets, results
