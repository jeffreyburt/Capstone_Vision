"""Point targets and closed-loop navigation. HTTP targets never command wheels directly."""

import json
import math
import threading
import time
import uuid
import urllib.request

from .planner import Target
from .controller import remove_approach, world_to_body

STOP = (0., 0., 0.)


class WaypointStore:
    def __init__(self, robot_ids, area=None):
        self.robot_ids, self.area = set(robot_ids), area
        self.lock = threading.Lock()
        self.targets = {}
        self.revision = 0
        self.armed = False
        self.session = uuid.uuid4().hex

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

    def arm(self, enabled):
        if type(enabled) is not bool:
            raise ValueError('enabled must be a boolean')
        with self.lock:
            self.targets.clear()  # an old destination can never resume when enabling driving
            self.armed = enabled

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


class WaypointMotion:
    """One continuous approach per click; arrival stays latched until a new goal.

    Normal motion is filtered and acceleration limited. Safety stops call halt(),
    immediately zeroing commands without forgetting that a goal was completed.
    """
    def __init__(self, linear_accel=.12, linear_decel=.18, angular_accel=.6, filter_s=.2):
        values=(linear_accel,linear_decel,angular_accel,filter_s)
        if any(not math.isfinite(v) or v <= 0 for v in values):
            raise ValueError('Motion smoothing parameters must be positive and finite')
        self.linear_accel,self.linear_decel,self.angular_accel,self.filter_s=values
        self.key=None
        self.arrived=False
        self.current=[0.,0.,0.]
        self.filtered=[0.,0.,0.]
        self.last_time=None

    def activate(self, goal, session, ctrl):
        key=(session,goal.get('revision'),goal['x'],goal['y'])
        if key != self.key:
            self.key=key
            self.arrived=False
            ctrl.reset()

    def halt(self):
        self.current=[0.,0.,0.]
        self.filtered=[0.,0.,0.]
        self.last_time=None

    def clear(self):
        self.halt()
        self.key=None
        self.arrived=False

    def command(self, desired, pose, target, tolerance, now, obstacles=()):
        distance=math.hypot(target.x-pose['x'],target.y-pose['y'])
        if distance <= tolerance:
            self.arrived=True
        desired=list(STOP if self.arrived else desired)
        speed=math.hypot(*desired[:2])
        braking=math.sqrt(2*self.linear_decel*max(0.,distance-tolerance))
        if speed > braking:
            desired[:2]=[v*braking/speed for v in desired[:2]]
        dt=.1 if self.last_time is None else min(.2,max(0.,now-self.last_time))
        self.last_time=now
        alpha=1-math.exp(-dt/self.filter_s)
        self.filtered=[f+alpha*(d-f) for f,d in zip(self.filtered,desired)]
        delta=[self.filtered[i]-self.current[i] for i in (0,1)]
        peak=math.hypot(*delta)
        rate=self.linear_accel if math.hypot(*self.filtered[:2])>math.hypot(*self.current[:2]) else self.linear_decel
        scale=min(1.,rate*dt/peak) if peak else 1.
        self.current[:2]=[self.current[i]+delta[i]*scale for i in (0,1)]
        change=self.filtered[2]-self.current[2]
        self.current[2]+=max(-self.angular_accel*dt,min(self.angular_accel*dt,change))
        # Filtering must never retain motion toward an obstacle after keep-out blocks it.
        h=math.radians(pose['heading']);c,s=math.cos(h),math.sin(h)
        world=(c*self.current[0]-s*self.current[1],s*self.current[0]+c*self.current[1])
        clipped=remove_approach(*world,pose['x'],pose['y'],obstacles)
        self.current[:2]=world_to_body(*clipped,pose['heading'])
        if max(abs(v) for v in self.current)<.001:
            self.current=[0.,0.,0.]
            if self.arrived:self.filtered=[0.,0.,0.]
        return tuple(self.current)


class WaypointProgress:
    """Latch a stop when translation fails to reduce camera distance; a new click resets it."""
    def __init__(self, timeout_s=6., improvement_m=.03):
        self.timeout_s, self.improvement_m = timeout_s, improvement_m
        self.key = None
        self.fault = False
        self.best = self.since = None

    def check(self, goal, distance, translating, now):
        key = (goal.get('session'), goal.get('revision'), goal['x'], goal['y'])
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
        capture_age = None
        if (msg or {}).get('frame_clock') == 'local' and isinstance((msg or {}).get('t'),(int,float)):
            capture_age=time.time()-msg['t']
        observed_age=frame_age
        if capture_age is not None:
            observed_age=max(frame_age or 0.,capture_age)
        age = None if not valid or observed_age is None else max(0., observed_age) + max(0., pose.get('age', 0))
        cutoff=min(.65,(msg or {}).get('max_frame_age_s',.25)) if capture_age is not None else .25
        error = None if not valid or not goal else [goal['x']-pose['x'], goal['y']-pose['y']]
        output[str(rid)] = dict(actual=[pose['x'], pose['y']] if valid else None,
            target=[goal['x'], goal['y']] if goal else None, error=error,
            distance_m=math.hypot(*error) if error else None, tracking_age_s=age,
            fresh=age is not None and age <= cutoff and (capture_age is None or capture_age>=0) and (pose or {}).get('pose_compensated') is not False,
            quality=(msg or {}).get('position_quality', 'calibrated' if valid else 'unavailable'),
            reference=(pose or {}).get('reference', 'chassis'),
            heading=(pose or {}).get('heading'), revision=(goal or {}).get('revision'))
    return output


def compute_waypoints(msg, age, navigation, controllers, timing=None, *, progress=None, motions=None, now=None):
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
    elif navigation.get("armed") is not True:
        blocked = "dashboard driving disabled"
    elif msg is None or age is None or age > timing['vision_timeout_s']:
        blocked = "no vision"
    elif msg.get('frame_clock') == 'local' and not 0 <= time.time()-msg['t'] <= min(.65,msg.get('max_frame_age_s',.25)):
        blocked = 'camera capture too old'
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
        hardware_ready = readiness.get('hardware_ready',readiness.get('ready'))
        if reason is None and not hardware_ready:
            reason = readiness.get('reason', 'navigation readiness unavailable')
        if reason:
            if motions is not None:
                if reason in ('dashboard driving disabled','no target'):
                    motions[rid].clear()
                    ctrl.reset()
                else:
                    motions[rid].halt()
            if progress is not None:
                progress[rid].pause()
            if motions is None:
                ctrl.reset()
            results[rid] = (STOP, reason)
            continue
        # Recheck compensated pose on the controller's own fresh vision input.
        pose_expired = msg.get('frame_clock') == 'local' and not 0 <= time.time()-msg['t']+pose.get('age',0) <= min(.65,msg.get('max_frame_age_s',.25))
        if not pose.get('pose_compensated') or pose_expired or (msg.get('frame_clock') != 'local' and age > .25):
            if progress is not None:
                progress[rid].pause()
            if motions is not None:
                motions[rid].halt()
            else:
                ctrl.reset()
            results[rid] = (STOP, 'compensated chassis pose required')
            continue
        user = msg.get('user')
        obstacles = [(r['x'], r['y'], ctrl.limits.robot_clearance_m) for r in poses.values()
                     if r['id'] != rid and r.get('age', 999) <= timing['max_pose_age_s']]
        if user and user.get('age', 999) <= 2:
            obstacles.append((user['x'], user['y'], ctrl.limits.keep_out_m))
        if motions is not None:
            motions[rid].activate(goals[str(rid)],navigation.get('session'),ctrl)
        cmd = ctrl.command(pose, targets[rid], obstacles)
        if motions is not None:
            cmd = motions[rid].command(cmd,pose,targets[rid],ctrl.limits.position_tolerance_m,now,obstacles)
        if progress is not None:
            guard = progress[rid]
            if guard.since is None:
                guard.best = math.hypot(targets[rid].x-pose['x'], targets[rid].y-pose['y'])
                guard.since = now
            distance = math.hypot(targets[rid].x-pose['x'], targets[rid].y-pose['y'])
            if not guard.check({**goals[str(rid)], 'session': navigation.get('session')}, distance, math.hypot(*cmd[:2]) > .001, now):
                if motions is not None: motions[rid].halt()
                ctrl.reset()
                results[rid] = (STOP, 'no camera progress; stopped — select a new destination to retry')
                continue
        reason = ('arriving' if cmd != STOP else 'arrived') if motions is not None and motions[rid].arrived else 'holding' if cmd == STOP else 'moving to waypoint'
        results[rid] = (cmd, reason)
    return targets, results
