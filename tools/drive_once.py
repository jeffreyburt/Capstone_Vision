"""Send one short, slow body-velocity command to a robot's drive server, then stop. No camera needed.

    python tools/drive_once.py --robot 2 --vx 0.05 --seconds 5      # forward
    python tools/drive_once.py --robot 2 --vy 0.05 --seconds 20     # strafe left (watch the wheels)
    python tools/drive_once.py --robot 2 --wz 0.25 --seconds 20     # rotate counter-clockwise

The robot's address comes from config.yaml control.robots. Commands go out at 10 Hz; stop is sent at the end
(and on Ctrl+C), and the drive server's 0.5 s watchdog stops the wheels if this program dies. Use it with the
wheels lifted first. For camera-measured floor tests use `python -m control.drive_test`.
"""

import argparse
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from control.link import RobotLink  # noqa: E402
from vision.config import load_config  # noqa: E402


def main():
    ap = argparse.ArgumentParser(description="One short drive command, then stop")
    ap.add_argument("--config")
    ap.add_argument("--robot", type=int, required=True)
    ap.add_argument("--vx", type=float, default=0.0, help="forward m/s")
    ap.add_argument("--vy", type=float, default=0.0, help="left m/s")
    ap.add_argument("--wz", type=float, default=0.0, help="counter-clockwise rad/s")
    ap.add_argument("--seconds", type=float, default=2.0)
    args = ap.parse_args()
    if args.seconds > 30 or max(abs(args.vx), abs(args.vy)) > 0.15 or abs(args.wz) > 0.6:
        ap.error("keep it slow and short: <= 30 s, <= 0.15 m/s, <= 0.6 rad/s")
    rc = {int(k): v for k, v in load_config(args.config)["control"]["robots"].items()}[args.robot]
    link = RobotLink({args.robot: (rc["host"], rc.get("port", 5700))})
    print(f"robot {args.robot} @ {rc['host']}: vx {args.vx} vy {args.vy} wz {args.wz} for {args.seconds} s")
    try:
        end = time.time() + args.seconds
        while time.time() < end:
            link.drive(args.robot, args.vx, args.vy, args.wz)
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("aborted")
    finally:
        link.stop_all()
    time.sleep(0.2)
    link.poll()
    print("last reply:", link.status.get(args.robot) or "none (is the drive server running?)")


if __name__ == "__main__":
    main()
