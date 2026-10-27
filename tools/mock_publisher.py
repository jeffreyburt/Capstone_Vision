"""Fake vision output so the controls/robot team can develop before the real system is ready.

    python tools/mock_publisher.py --host <controller-laptop-ip>

Sends exactly the same JSON message format as vision.main at 10 Hz: a user swaying side to side,
and two robots following on either side of them, turned to face them.
"""

import argparse
import math
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from vision.config import load_config
from vision.output import UdpPublisher, make_message


def main():
    out = load_config().get("output", {})
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--host", default=out.get("host", "127.0.0.1"))
    ap.add_argument("--port", type=int, default=out.get("port", 5005))
    ap.add_argument("--rate", type=float, default=10.0)
    ap.add_argument("--print", dest="print_msgs", action="store_true")
    args = ap.parse_args()

    pub = UdpPublisher(args.host, args.port, broadcast=args.host.endswith(".255"))
    print(f"Sending mock data to {args.host}:{args.port} at {args.rate:.0f} Hz. Ctrl+C to stop.")
    start, frame = time.time(), 0
    try:
        while True:
            t = time.time()
            s = t - start
            ux, uy = 1.0 * math.sin(0.3 * s), 4.5
            robots = []
            for rid, side in ((1, -1), (2, 1)):
                # trail the user a little, with some noise-free smooth motion
                lag = s - 1.0
                tx, ty = 1.0 * math.sin(0.3 * lag) + side * 1.2, 3.2
                heading = math.degrees(math.atan2(uy - ty, ux - tx))  # front face toward the user
                vx = 0.3 * math.cos(0.3 * lag)
                robots.append({"id": rid, "x": tx, "y": ty, "heading": heading, "vx": vx, "vy": 0.0, "age": 0.03})
            user = {"x": round(ux, 3), "y": uy, "age": 0.1, "confidence": 0.9}
            msg = make_message(t, frame, robots, user)
            pub.send(msg)
            if args.print_msgs:
                print(msg)
            frame += 1
            time.sleep(max(0.0, 1.0 / args.rate - (time.time() - t)))
    except KeyboardInterrupt:
        pass
    finally:
        pub.close()


if __name__ == "__main__":
    main()
