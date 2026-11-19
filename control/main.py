"""MVP control loop: the robots follow the user, keeping a listening triangle and facing them.

Start the vision system first (it publishes robot + user positions to 127.0.0.1:5005), start
robot/drive_server.py on each robot Pi, then on the laptop:

    python -m control.main                      # dry run: prints targets and commands, sends nothing
    python -m control.main --arm --robots 1     # drive robot 1 only
    python -m control.main --arm                # drive every robot in config.yaml control.robots

Keys (Windows console): Space = pause / resume (paused robots are sent stop), q = quit. Ctrl+C also quits.
On exit every robot is sent stop; each robot's watchdog also stops it within 0.5 s if commands stop.

A robot is stopped whenever the camera hasn't seen it recently, the user is lost, or vision goes quiet.
"""

import argparse
import sys
import time

from vision.config import load_config

from .controller import Limits, RobotController
from .link import RobotLink, VisionListener
from .planner import FormationPlanner

STOP = (0.0, 0.0, 0.0)
TIMING_DEFAULTS = {"vision_timeout_s": 1.0, "max_pose_age_s": 0.7, "user_lost_s": 2.0}


def compute_commands(msg, msg_age_s, planner, controllers, timing=None, compensated_ids=()):
    """One control step, without any I/O.

    msg: newest vision message (or None); msg_age_s: seconds since it arrived on this laptop.
    controllers: {robot_id: RobotController} for the robots we drive.
    Returns (targets, {robot_id: ((vx, vy, wz), reason)}); a stopped robot gets STOP.
    """
    timing = {**TIMING_DEFAULTS, **(timing or {})}
    if msg is None or msg_age_s is None or msg_age_s > timing["vision_timeout_s"]:
        for c in controllers.values():
            c.reset()
        return {}, {rid: (STOP, "no vision") for rid in controllers}

    robots = {r["id"]: r for r in msg.get("robots", [])}
    u = msg.get("user")
    user = (u["x"], u["y"]) if u is not None and u["age"] <= timing["user_lost_s"] else None
    targets = planner.update(user)

    out = {}
    for rid, ctrl in controllers.items():
        pose = robots.get(rid)
        if not targets:
            ctrl.reset()
            out[rid] = (STOP, "no user")
            continue
        if rid in compensated_ids and ((pose or {}).get("pose_compensated") is not True or msg_age_s > 0.25):
            ctrl.reset()
            out[rid] = (STOP, "compensated pose unavailable")
            continue
        if pose is None or pose.get("heading") is None or pose["age"] > timing["max_pose_age_s"]:
            ctrl.reset()
            out[rid] = (STOP, "robot not seen")
            continue
        lim = ctrl.limits
        obstacles = [(user[0], user[1], lim.keep_out_m)]
        obstacles += [(o["x"], o["y"], lim.robot_clearance_m) for oid, o in robots.items()
                      if oid != rid and o["age"] <= timing["max_pose_age_s"]]
        cmd = ctrl.command(pose, targets[rid], obstacles)
        out[rid] = (cmd, "holding" if cmd == STOP else "moving")
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
        pose = f"({r['x']:.2f}, {r['y']:.2f}, {r['heading'] or 0:.0f}deg)" if r else "(unseen)"
        goal = f" -> ({t.x:.2f}, {t.y:.2f}, {t.heading_deg:.0f}deg)" if t else ""
        age = link_ages.get(rid)
        link = "no reply" if age is None else ("link ok" if age < 1.0 else f"link {age:.0f}s old")
        parts.append(f"R{rid} {pose}{goal} cmd ({cmd[0]:+.2f}, {cmd[1]:+.2f}, {cmd[2]:+.2f}) {reason}, {link}")
    return "  |  ".join(parts)


def main(argv=None):
    ap = argparse.ArgumentParser(description="Robots follow the user (laptop closes the loop)")
    ap.add_argument("--config", help="path to config.yaml (default: repo config.yaml)")
    ap.add_argument("--robots", help="comma-separated robot IDs to drive (default: all in control.robots)")
    ap.add_argument("--arm", action="store_true", help="send drive commands (default: dry run)")
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
                               f.get("replan_m", 0.3), cfg.get("area"))
    limits = Limits.from_config(c.get("limits"))
    controllers = {rid: RobotController(limits) for rid in driven}
    timing = {k: c[k] for k in TIMING_DEFAULTS if k in c}
    period = 1.0 / float(c.get("rate_hz", 10))

    vision = VisionListener(int(c.get("vision_port", cfg.get("output", {}).get("port", 5005))))
    link = RobotLink({rid: (robots_cfg[rid]["host"], robots_cfg[rid].get("port", 5700)) for rid in driven})
    keys = Keys()

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
                elif k == "q":
                    return 0
            msg, received_at = vision.poll()
            now = time.time()
            targets, results = compute_commands(msg, None if received_at is None else now - received_at,
                                                planner, controllers, timing, compensated_ids)
            if args.arm:
                for rid, (cmd, _) in results.items():
                    if paused or cmd == STOP:
                        link.stop(rid)
                    else:
                        link.drive(rid, *cmd, pose_compensated=rid in compensated_ids)
            link_ages = link.poll()
            if now - last_print >= 1.0:
                mode = "PAUSED" if paused else ("ARMED" if args.arm else "DRY RUN")
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
        vision.close()
        link.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
