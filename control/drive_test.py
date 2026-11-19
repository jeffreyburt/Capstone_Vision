"""Drive one robot open-loop for a few seconds and measure what it really did with the camera.

Use it to (1) check the wheel directions: the robot must move the way it is told *relative to the face the
vision system calls its front*, and (2) calibrate robot/drive_server.py's --counts-per-meter and
--turn-radius (do the straight-line tests first; the turn test depends on counts-per-meter).

    python -m control.drive_test --robot 1 --vx 0.1 --seconds 3     # forward ~30 cm
    python -m control.drive_test --robot 1 --vy 0.1 --seconds 3     # strafe left ~30 cm
    python -m control.drive_test --robot 1 --wz 0.3 --seconds 4     # turn ~70 deg counter-clockwise

The vision system and the robot's drive server must be running. Leave ~1 m of clear floor around the robot.
"""

import argparse
import math
import time

from vision.config import load_config

from .controller import world_to_body
from .link import RobotLink, VisionListener


def wrap_deg(a):
    return (a + 180.0) % 360.0 - 180.0


def measure_pose(vision, rid, seconds=1.0, max_age=0.3, wait=5.0):
    """Average the robot's pose over `seconds` of fresh camera frames. Returns (x, y, heading_deg)."""
    start, frames, xs, ys, hc, hs = time.time(), set(), [], [], 0.0, 0.0
    t_end = None
    while True:
        msg, _ = vision.poll()
        r = next((r for r in (msg or {}).get("robots", []) if r["id"] == rid), None)
        if r is not None and r["age"] <= max_age and r["heading"] is not None and msg["frame"] not in frames:
            frames.add(msg["frame"])
            xs.append(r["x"])
            ys.append(r["y"])
            hc += math.cos(math.radians(r["heading"]))
            hs += math.sin(math.radians(r["heading"]))
            t_end = t_end or time.time() + seconds
        if t_end and time.time() >= t_end:
            break
        if not xs and time.time() - start > wait:
            raise RuntimeError(f"robot {rid} not seen by the camera (is vision.main running?)")
        time.sleep(0.03)
    return sum(xs) / len(xs), sum(ys) / len(ys), math.degrees(math.atan2(hs, hc))


def main(argv=None):
    ap = argparse.ArgumentParser(description="Open-loop drive test measured by the camera")
    ap.add_argument("--config")
    ap.add_argument("--robot", type=int, required=True)
    ap.add_argument("--vx", type=float, default=0.0, help="forward m/s")
    ap.add_argument("--vy", type=float, default=0.0, help="left m/s")
    ap.add_argument("--wz", type=float, default=0.0, help="counter-clockwise rad/s")
    ap.add_argument("--seconds", type=float, default=3.0)
    args = ap.parse_args(argv)
    if args.seconds > 10 or math.hypot(args.vx, args.vy) > 0.3 or abs(args.wz) > 1.0:
        ap.error("keep tests short and slow: <= 10 s, <= 0.3 m/s, <= 1 rad/s")

    cfg = load_config(args.config)
    c = cfg.get("control", {})
    rc = {int(k): v for k, v in c.get("robots", {}).items()}[args.robot]
    vision = VisionListener(int(c.get("vision_port", cfg.get("output", {}).get("port", 5005))))
    link = RobotLink({args.robot: (rc["host"], rc.get("port", 5700))})
    try:
        x0, y0, h0 = measure_pose(vision, args.robot)
        print(f"Start: ({x0:.3f}, {y0:.3f}) m, heading {h0:.1f} deg. Driving in 2 s ... (Ctrl+C aborts)")
        time.sleep(2.0)
        t_end = time.time() + args.seconds
        while time.time() < t_end:
            link.drive(args.robot, args.vx, args.vy, args.wz)
            time.sleep(0.1)
        link.stop_all()
        if link.poll()[args.robot] is None:
            print("WARNING: no reply from the drive server; check its address and that it is running.")
        time.sleep(1.5)  # let it coast to a stop and the camera catch up
        x1, y1, h1 = measure_pose(vision, args.robot)
    except KeyboardInterrupt:
        link.stop_all()
        print("Aborted; robot stopped.")
        return 1
    finally:
        vision.close()
        link.close()

    fwd, left = world_to_body(x1 - x0, y1 - y0, h0)  # displacement in the robot's starting frame
    turn = wrap_deg(h1 - h0)
    T = args.seconds
    print(f"End:   ({x1:.3f}, {y1:.3f}) m, heading {h1:.1f} deg")
    print(f"Commanded: forward {args.vx * T:+.3f} m, left {args.vy * T:+.3f} m, turn {math.degrees(args.wz * T):+.1f} deg")
    print(f"Measured:  forward {fwd:+.3f} m, left {left:+.3f} m, turn {turn:+.1f} deg")
    print("(Speeds ramp up, so expect slightly less than commanded. Position is the marker head's center.)")

    want = (args.vx * T, args.vy * T)
    dist = math.hypot(*want)
    if dist > 0.05:
        along = (fwd * want[0] + left * want[1]) / dist
        across = (-fwd * want[1] + left * want[0]) / dist
        if along < 0.3 * dist:
            print("PROBLEM: the robot did not move the way it was told. Check motor_signs / wheel_channels "
                  "in the robot's teleop.json, and that the vision 'front' face really is the robot's front.")
        else:
            print(f"Distance ratio measured/commanded = {along / dist:.2f}  ->  multiply the drive server's "
                  f"--counts-per-meter by {dist / along:.2f}")
            if abs(across) > 0.25 * dist:
                print(f"Note: drifted {across:+.3f} m sideways; a wheel may be weak or mis-signed.")
    if abs(args.wz * T) > math.radians(20):
        want_turn = math.degrees(args.wz * T)
        if turn * want_turn <= 0 or abs(turn) < 0.3 * abs(want_turn):
            print("PROBLEM: the robot did not turn the way it was told (positive wz = counter-clockwise "
                  "seen from above). Check motor_signs / wheel_channels.")
        else:
            print(f"Turn ratio measured/commanded = {turn / want_turn:.2f}  ->  multiply --turn-radius by "
                  f"{want_turn / turn:.2f}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
