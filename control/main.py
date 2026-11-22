"""MVP control loop: the robots follow the user, keeping a listening triangle and facing them.

Start the vision system first (it publishes robot + user positions to 127.0.0.1:5005), start
robot/drive_server.py on each robot Pi, then on the laptop:

    python -m control.main                      # dry run: prints targets and commands, sends nothing
    python -m control.main --arm --robots 1     # drive robot 1 only
    python -m control.main --arm                # drive every robot in config.yaml control.robots

Keys (Windows console): Space = pause / resume (paused robots are sent stop), q = quit. Ctrl+C also quits.
On exit every robot is sent stop; each robot's watchdog also stops it within 0.5 s if commands stop.

A robot is stopped whenever the camera hasn't seen it recently, the user is lost, or vision goes quiet.
In every mode, robots are soft-stopped at the edge of the main camera's view, and in the self-driving modes a
robot that leaves it anyway drives back in from its last estimated position (control/view_guard.py).
"""

import argparse
import math
import sys
import time

from monitor.beacon import StatusBeacon, status_port
from vision.config import load_config

from .controller import Limits, RobotController
from .link import RobotLink, VisionListener
from .planner import FormationPlanner, Target
from .view_guard import CameraView, ViewGuard
from .blackbox import BlackBox
from .manual import manual_commands
from .waypoints import WaypointMotion, frame_current, WaypointProgress, compute_waypoints, read_waypoints

STOP = (0.0, 0.0, 0.0)
TIMING_DEFAULTS = {"vision_timeout_s": 1.0, "max_pose_age_s": 0.7, "user_lost_s": 2.0, "max_heading_age_s": 0.5}
# Formation targets stay at least this far inside the camera's floor view, so the view guard never has to hold a
# robot at the edge short of a target it can't reach (and its tags stay in frame).
VIEW_TARGET_MARGIN_M = 0.35
FEEDFORWARD_DEFAULTS = {"enabled": False, "gain": 1.0, "min_user_speed_m_s": 0.05, "max_m_s": 0.15, "lead_s": 0.4}


class DeadReckoner:
    """Carries a robot's pose forward from its last fresh camera pose using the commands sent since.

    Used only for a short gap (a tag briefly out of view); commands are scaled down meanwhile, and after
    `max_s` the robot stops as before. Mecanum wheels slip, so this is a bridge, not navigation."""
    def __init__(self, max_s=1.5, speed_scale=0.5):
        self.max_s, self.speed_scale = max_s, speed_scale
        self.pose = None   # dict x, y, heading (deg)
        self.t = None      # time of the last fresh camera pose
        self.last = None   # (time, body command) of the last integration step
        self.travelled = 0.0  # distance integrated since the last fresh camera pose (view-guard watchdog)

    def observe(self, pose, now):
        self.pose = {"x": pose["x"], "y": pose["y"], "heading": pose["heading"]}
        self.t = now
        self.last = (now, STOP)
        self.travelled = 0.0

    def estimate(self, now, max_s=None):
        """Pose integrated up to now, or None when there is nothing recent enough to carry forward.
        max_s overrides the age limit (the view guard's return uses its own, longer watchdog)."""
        if self.pose is None or now - self.t > (self.max_s if max_s is None else max_s):
            return None
        t0, (vx, vy, wz) = self.last
        dt = max(0.0, now - t0)
        h = math.radians(self.pose["heading"])
        self.pose["x"] += (vx * math.cos(h) - vy * math.sin(h)) * dt
        self.pose["y"] += (vx * math.sin(h) + vy * math.cos(h)) * dt
        self.travelled += math.hypot(vx, vy) * dt
        self.pose["heading"] = (self.pose["heading"] + math.degrees(wz * dt) + 180) % 360 - 180
        self.last = (now, self.last[1])
        return dict(self.pose)

    def sent(self, cmd, now):
        if self.pose is not None:
            self.last = (now, cmd)


def vision_usable(msg, msg_age_s, timing=None):
    """True when the newest vision message is current and calibrated enough to drive on."""
    timing = {**TIMING_DEFAULTS, **(timing or {})}
    if msg is None or msg_age_s is None or msg_age_s > timing["vision_timeout_s"]:
        return False
    if msg.get("frame_clock") == "local" and not frame_current(msg, msg_age_s):
        return False
    return msg.get("position_quality") != "approximate" and msg.get("driving_allowed") is not False


def inside_view(view, target):
    """The target moved in along the camera view's edge until it is VIEW_TARGET_MARGIN_M inside."""
    d, _ = view.boundary(target.x, target.y)
    if d >= VIEW_TARGET_MARGIN_M:
        return target
    x, y = view.inward_point(target.x, target.y, VIEW_TARGET_MARGIN_M)
    return Target(x, y, target.heading_deg)


def fresh_poses(msg, msg_age_s, robot_ids, timing=None, compensated_ids=()):
    """{rid: pose} for robots the camera located reliably in this (usable) message: finite position, a heading,
    recent, and compensated for head motion where the robot has a moving head."""
    timing = {**TIMING_DEFAULTS, **(timing or {})}
    out = {}
    for r in (msg or {}).get("robots", []):
        rid = r.get("id")
        if rid not in robot_ids or r.get("heading") is None or r.get("age", 999) > timing["max_pose_age_s"]:
            continue
        if not all(isinstance(r.get(k), (int, float)) and math.isfinite(r[k]) for k in ("x", "y", "heading")):
            continue
        if rid in compensated_ids and (r.get("pose_compensated") is not True or (
                msg.get("frame_clock") == "local" and not frame_current(msg, msg_age_s, r.get("age", 0)))):
            continue
        out[rid] = r
    return out


def user_feedforward(user, settings):
    """While the user walks, the formation's targets move with the user's velocity (the facing direction is
    fixed). Returns (world feed-forward velocity, user position to plan from) or (None, None) when off or
    standing still. The planning position leads by lead_s to cover the camera + processing delay."""
    s = {**FEEDFORWARD_DEFAULTS, **(settings or {})}
    if not s["enabled"] or user is None:
        return None, None
    vx, vy = float(user.get("vx") or 0.0), float(user.get("vy") or 0.0)
    speed = math.hypot(vx, vy)
    if not math.isfinite(speed) or speed < s["min_user_speed_m_s"]:
        return None, None
    fx, fy = vx * s["gain"], vy * s["gain"]
    size = math.hypot(fx, fy)
    if size > s["max_m_s"]:
        fx, fy = fx * s["max_m_s"] / size, fy * s["max_m_s"] / size
    return (fx, fy), (user["x"] + vx * s["lead_s"], user["y"] + vy * s["lead_s"])


def compute_commands(msg, msg_age_s, planner, controllers, timing=None, compensated_ids=(), reckoners=None, now=None,
                     feedforward=None):
    """One control step, without any I/O.

    msg: newest vision message (or None); msg_age_s: seconds since it arrived on this laptop.
    controllers: {robot_id: RobotController} for the robots we drive.
    feedforward: config.yaml control.feedforward (see user_feedforward); None/disabled = proportional only.
    Returns (targets, {robot_id: ((vx, vy, wz), reason)}); a stopped robot gets STOP.
    """
    timing = {**TIMING_DEFAULTS, **(timing or {})}
    reckoners = reckoners or {}
    now = time.time() if now is None else now
    if msg is None or msg_age_s is None or msg_age_s > timing["vision_timeout_s"]:
        for c in controllers.values():
            c.reset()
        return {}, {rid: (STOP, "no vision") for rid in controllers}

    if msg.get('frame_clock') == 'local' and not frame_current(msg, msg_age_s):
        for ctrl in controllers.values():
            ctrl.reset()
        return {}, {rid: (STOP, 'camera capture too old') for rid in controllers}

    robots = {r["id"]: r for r in msg.get("robots", [])}
    if msg.get("position_quality") == "approximate" or msg.get("driving_allowed") is False:
        for ctrl in controllers.values():
            ctrl.reset()
        return {}, {rid: (STOP, "approximate positions — driving disabled") for rid in controllers}
    u = msg.get("user")
    user = (u["x"], u["y"]) if u is not None and u["age"] <= timing["user_lost_s"] else None
    ff, plan_from = user_feedforward(u if user is not None else None, feedforward)
    targets = planner.update(plan_from or user, force=ff is not None)
    if targets:
        view = CameraView.from_message(msg)
        if view is not None:
            targets = {rid: inside_view(view, t) for rid, t in targets.items()}
        targets = planner.assign(targets, {rid: r for rid, r in robots.items() if rid in controllers
                                           and r.get("age", 999) <= timing["max_pose_age_s"]})

    out = {}
    for rid, ctrl in controllers.items():
        pose = robots.get(rid)
        if not targets:
            ctrl.reset()
            out[rid] = (STOP, "no user")
            continue
        pose_expired = (msg.get('frame_clock') == 'local' and
                        not frame_current(msg, msg_age_s, (pose or {}).get('age',0)))
        problem = None
        if rid in compensated_ids and ((pose or {}).get("pose_compensated") is not True or pose_expired or
                                      (msg.get('frame_clock') != 'local' and msg_age_s > .25)):
            problem = "compensated pose unavailable"
        elif pose is None or pose.get("heading") is None or pose["age"] > timing["max_pose_age_s"]:
            problem = "robot not seen"
        reck = reckoners.get(rid)
        scale = 1.0
        if problem is None:
            if reck is not None:
                reck.observe(pose, now)
        else:
            estimate = reck.estimate(now) if reck is not None else None
            if estimate is None:
                ctrl.reset()
                out[rid] = (STOP, problem)
                continue
            pose, scale = estimate, reck.speed_scale
            problem = f"dead reckoning ({problem}, {now - reck.t:.1f} s)"
        lim = ctrl.limits
        obstacles = [(user[0], user[1], lim.keep_out_m)]
        obstacles += [(o["x"], o["y"], lim.robot_clearance_m) for oid, o in robots.items()
                      if oid != rid and o["age"] <= timing["max_pose_age_s"]]
        cmd = ctrl.command(pose, targets[rid], obstacles, ff)
        held = problem is None and (pose.get("heading_age") or 0) > timing["max_heading_age_s"]
        if held:
            # The heading is held from an older fit, not measured: it can't see the chassis turn, so turning on it
            # would spin forever toward a heading it never reports. Translate only (a held heading stays right).
            cmd = (cmd[0], cmd[1], 0.0)
            problem = "heading held, not turning"
        if scale != 1.0:
            cmd = tuple(v * scale for v in cmd)
        if reck is not None:
            reck.sent(cmd, now)
        out[rid] = (cmd, problem or ("holding" if cmd == STOP else ("following" if ff else "moving")))
    return targets, out


class Keys:
    """Non-blocking single-key reads from the Windows console (no-op elsewhere: use Ctrl+C)."""

    def __init__(self):
        try:
            import msvcrt
            self.msvcrt = msvcrt if sys.stdin.isatty() else None
        except ImportError:
            self.msvcrt = None

    def read(self):
        keys = []
        while self.msvcrt is not None and self.msvcrt.kbhit():
            keys.append(self.msvcrt.getwch().lower())
        return keys


def _fmt_status(mode, msg, targets, results, link_ages):
    u = (msg or {}).get("user")
    parts = [f"[{mode}]", f"user ({u['x']:.2f}, {u['y']:.2f})" if u else "user -"]
    robots = {r["id"]: r for r in (msg or {}).get("robots", [])}
    for rid, (cmd, reason) in results.items():
        r, t = robots.get(rid), targets.get(rid)
        heading = "heading unknown" if not r or r.get("heading") is None else f"{r['heading']:.0f}deg"
        pose = f"({r['x']:.2f}, {r['y']:.2f}, {heading})" if r else "(unseen)"
        goal = f" -> ({t.x:.2f}, {t.y:.2f})" if t else ""
        age = link_ages.get(rid)
        link = "no reply" if age is None else ("link ok" if age < 1.0 else f"link {age:.0f}s old")
        parts.append(f"R{rid} {pose}{goal} cmd ({cmd[0]:+.2f}, {cmd[1]:+.2f}, {cmd[2]:+.2f}) {reason}, {link}")
    return "  |  ".join(parts)


def status_report(mode, msg_age_s, targets, results, link_ages, link_status):
    """Fields for the system GUI's heartbeat (see monitor/beacon.py). JSON keys must be strings."""
    robots = {}
    for rid, (cmd, reason) in results.items():
        t = targets.get(rid)
        robots[str(rid)] = {
            "target": None if t is None else [round(t.x, 3), round(t.y, 3), None if t.heading_deg is None else round(t.heading_deg, 1)],
            "cmd": [round(v, 3) for v in cmd], "reason": reason,
            "link_age_s": link_ages.get(rid), "reply": link_status.get(rid)}
    return {"state": "running", "mode": mode, "vision_age_s": msg_age_s, "robots": robots}


def main(argv=None):
    ap = argparse.ArgumentParser(description="Robots follow the user (laptop closes the loop)")
    ap.add_argument("--config", help="path to config.yaml (default: repo config.yaml)")
    ap.add_argument("--robots", help="comma-separated robot IDs to drive (default: all in control.robots)")
    ap.add_argument("--arm", action="store_true", help="send drive commands (default: dry run)")
    ap.add_argument("--waypoints", action="store_true", help="use dashboard grid targets instead of following the person")
    ap.add_argument("--no-blackbox", action="store_true", help="don't record this run (config.yaml blackbox; see control/blackbox.py)")
    args = ap.parse_args(argv)

    cfg = load_config(args.config)
    c = cfg.get("control", {})
    compensated_ids = {int(r["id"]) for r in cfg.get("robots", []) if r.get("head_pose") is not None}
    robots_cfg = {int(k): v for k, v in c.get("robots", {}).items()}
    driven = [int(x) for x in args.robots.split(",")] if args.robots else sorted(robots_cfg)
    unknown = [rid for rid in driven if rid not in robots_cfg]
    if unknown:
        ap.error(f"robots {unknown} are not in config control.robots")

    f = c.get("formation", {})
    planner = FormationPlanner({rid: robots_cfg[rid].get("side", "center") for rid in driven},
                               f.get("distance_m", 1.5), f.get("spread_deg", 60.0), f.get("facing_deg", -90.0),
                               f.get("replan_m", 0.3), cfg.get("area"),
                               x_limits=f.get("x_limits_m"), y_limits=f.get("y_limits_m"),
                               max_turn_deg=f.get("max_turn_deg"), line_y_m=f.get("line_y_m"),
                               min_side_m=f.get("min_side_m", 0.8))
    limits = Limits.from_config(c.get("limits"))
    controllers = {rid: RobotController(Limits.from_config({
        **(c.get('limits') or {}), **(robots_cfg[rid].get('limits') or {})})) for rid in driven}
    progress = {rid: WaypointProgress() for rid in driven}
    motions = {rid: WaypointMotion(**(c.get("waypoint_motion") or {})) for rid in driven}
    timing = {k: c[k] for k in TIMING_DEFAULTS if k in c}
    # Always kept: follow mode bridges short tag gaps with them (dead_reckon_s; 0 = off) and the view guard
    # returns a robot that left the camera view from their estimate.
    reckoners = {rid: DeadReckoner(float(c.get("dead_reckon_s", 1.5))) for rid in driven}
    guard = ViewGuard(c.get("view_guard"))
    period = 1.0 / float(c.get("rate_hz", 10))

    vision = VisionListener(int(c.get("vision_port", cfg.get("output", {}).get("port", 5005))))
    endpoints = {rid: (robots_cfg[rid]["host"], robots_cfg[rid].get("port",5700)) for rid in driven}
    if c.get('drive_via'):
        from .ssh_gateway import GatewayRobotLink
        link = GatewayRobotLink(endpoints,c['drive_via'])
    else:
        link = RobotLink(endpoints)
    keys = Keys()
    beacon = StatusBeacon("control", status_port(cfg))  # status for the system GUI (python -m monitor)
    bb_cfg = {**(cfg.get("blackbox") or {})}
    bb_cfg.setdefault("snapshot_url",
                      f"http://127.0.0.1:{int((cfg.get('debug') or {}).get('stream_port', 8080))}/video/birdseye")
    blackbox = None
    if bb_cfg.get("enabled", True) and not args.no_blackbox:
        from vision.config import resolve_path
        blackbox = BlackBox(bb_cfg, argv=["control.main", *(sys.argv[1:] if argv is None else argv)], robots=driven,
                            config_path=args.config or resolve_path(cfg, "config.yaml"))
        print(f"Black box recording to {blackbox.dir}")
    head_states = None
    if blackbox and args.waypoints:  # the dashboard (monitor) is running: record the head trackers too
        from .blackbox import HeadStates
        head_states = HeadStates(f"http://127.0.0.1:{int((cfg.get('system') or {}).get('gui_port', 8090))}", driven)

    print(("ARMED: driving robots " if args.arm else "DRY RUN (nothing is sent; add --arm to drive): robots ")
          + ", ".join(f"{rid} @ {robots_cfg[rid]['host']}" for rid in driven))
    if keys.msvcrt:
        print("Space = pause/resume, q = quit, Ctrl+C = quit.")
    paused = False
    last_print = 0.0
    try:
        next_t = time.time()
        while True:
            for k in keys.read():
                if k == " ":
                    paused = not paused
                    print("PAUSED: robots stopped" if paused else "RESUMED")
                    if blackbox:
                        blackbox.event(time.time(), "paused" if paused else "resumed")
                elif k == "q":
                    return 0
            msg, received_at = vision.poll()
            now = time.time()
            msg_age = None if received_at is None else now - received_at
            manual_mode = False
            follow_mode = False
            nav = None
            if args.waypoints:
                nav = read_waypoints(f"http://127.0.0.1:{int((cfg.get('system') or {}).get('gui_port',8090))}/api/navigation")
                manual = manual_commands(nav, controllers)
                if manual is not None:
                    targets, results = {}, manual
                    manual_mode = True
                    for motion in motions.values():motion.halt()
                elif (nav or {}).get('follow') is True:
                    # Dashboard "Follow user": keep the listening triangle around the user.
                    targets, results = compute_commands(msg, msg_age, planner, controllers, timing, compensated_ids,
                                                        reckoners, now, feedforward=c.get("feedforward"))
                    follow_mode = True
                    for motion in motions.values():motion.halt()
                else:
                    targets, results = compute_waypoints(msg, msg_age, nav, controllers, timing, progress=progress, motions=motions)
            else:
                targets, results = compute_commands(msg, msg_age, planner, controllers, timing, compensated_ids, reckoners,
                                                    now, feedforward=c.get("feedforward"))
            # Camera view guard: soft stop at the view edge (every mode); self-driving modes also bring back a
            # robot that left the view, from its dead-reckoned estimate, until the watchdog gives up.
            usable = vision_usable(msg, msg_age, timing)
            u = (msg or {}).get("user")
            results = guard.apply(
                msg, results, controllers, reckoners,
                fresh=fresh_poses(msg, msg_age, controllers, timing, compensated_ids) if usable else {},
                vision_ok=usable, now=now,
                allow_return=not paused and not manual_mode and (
                    not args.waypoints or follow_mode or (nav or {}).get("armed") is True),
                user=(u["x"], u["y"]) if u and u.get("age", 999) <= timing.get("user_lost_s", 2.0) else None)
            for rid, (cmd, _) in results.items():  # what was really sent (nothing in a dry run or while paused)
                reckoners[rid].sent(STOP if paused or not args.arm else cmd, now)
            if args.arm:
                for rid, (cmd, _) in results.items():
                    if paused or cmd == STOP:
                        link.stop(rid)
                    elif manual_mode:
                        link.manual(rid, *cmd)
                    else:
                        link.drive(rid, *cmd, pose_compensated=(args.waypoints and not follow_mode) or rid in compensated_ids)
            link_ages = link.poll()
            mode = "PAUSED" if paused else ("ARMED" if args.arm else "DRY RUN")
            report = status_report(mode, msg_age, targets, results, link_ages, link.status)
            report['navigation_mode'] = 'keyboard' if manual_mode else ('follow' if follow_mode or not args.waypoints else 'waypoints')
            if blackbox:
                blackbox.record(now, mode=mode, navigation_mode=report['navigation_mode'], msg=msg, msg_age=msg_age,
                                nav=nav, targets=targets, results=results, link_ages=link_ages,
                                link_status=link.status, reckoners=reckoners, guard=guard,
                                heads=head_states.snapshot() if head_states else None)
                report['blackbox'] = blackbox.status()
            beacon.update(report)
            if now - last_print >= 1.0:
                print(_fmt_status(mode, msg, targets, results, link_ages))
                last_print = now
            next_t += period
            time.sleep(max(0.0, next_t - time.time()))
            if time.time() - next_t > period:  # fell behind (e.g. laptop hiccup): don't try to catch up
                next_t = time.time()
    except KeyboardInterrupt:
        pass
    finally:
        if args.arm:
            link.stop_all()
            print("Sent stop to all robots.")
        if head_states:
            head_states.close()
        if blackbox:
            blackbox.event(time.time(), "stopped")
            blackbox.close()
            print(f"Black box saved: {blackbox.dir}")
        beacon.close()
        vision.close()
        link.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
