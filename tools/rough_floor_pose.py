"""Rough floor calibration from a measured camera height and tilt (no tape marks needed).

    python tools/rough_floor_pose.py --height 1.0 --tilt 5

Assumes the camera looks straight ahead along +y (no sideways pan or roll), with the origin on the
floor directly below the camera, x to the right and y away from it. Good enough to get positions in
meters for testing; small tilt errors give large distance errors far away, so use
tools/calibrate_floor.py for real measurements.
"""

import argparse
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from vision.config import load_config, resolve_path
from vision.geometry import FloorPose


def main():
    cfg = load_config()
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--height", type=float, required=True, help="lens height above the floor (m)")
    ap.add_argument("--tilt", type=float, required=True, help="degrees the camera points below horizontal")
    ap.add_argument("--out", default=str(resolve_path(cfg, cfg["calibration"]["floor_pose"])))
    args = ap.parse_args()

    t = math.radians(args.tilt)
    pose = FloorPose.look_at([0.0, 0.0, args.height], [0.0, math.cos(t), args.height - math.sin(t)])
    pose.save(args.out, method="rough (height + tilt, not measured with floor marks)",
              height_m=args.height, tilt_deg=args.tilt)
    print(f"Saved {args.out}: camera {args.height} m high, tilted {args.tilt} deg down, looking along +y")


if __name__ == "__main__":
    main()
