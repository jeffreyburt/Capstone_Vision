"""Marker range test: how reliably (and how steadily) are markers detected at each distance?

    python tools/range_test.py --seconds 10 --stream

Place a robot (or just a marker on a box) at a distance, type a label such as "15ft" and press Enter.
The tool measures for --seconds and prints, per marker ID:
    detection rate, marker size in pixels, pixels per marker cell (aim for >= 4-5),
and, if the floor is calibrated, the robot position mean and jitter (standard deviation).
Rows are appended to results/range_test.csv for your report.
"""

import argparse
import csv
import math
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import numpy as np

from vision.camera import open_source
from vision.config import load_config
from vision.markers import get_dictionary
from vision.pipeline import VisionPipeline


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--source", help="camera source (default from config)")
    ap.add_argument("--seconds", type=float, default=10.0)
    ap.add_argument("--csv", default="results/range_test.csv")
    ap.add_argument("--stream", nargs="?", const=8080, type=int, metavar="PORT", help="preview over HTTP")
    args = ap.parse_args()

    cfg = load_config()
    cfg["people"] = {"enabled": False}  # markers only
    cells = get_dictionary(cfg["markers"].get("dictionary", "DICT_4X4_50")).markerSize + 2
    pipeline = VisionPipeline(cfg)
    src = open_source(cfg, args.source)
    server = None
    if args.stream:
        from vision.debug_view import MjpegServer, draw_overlay
        server = MjpegServer(args.stream)

    Path(args.csv).parent.mkdir(parents=True, exist_ok=True)
    new_file = not Path(args.csv).exists()
    with open(args.csv, "a", newline="") as f:
        writer = csv.writer(f)
        if new_file:
            writer.writerow(["label", "resolution", "kind", "id", "frames", "detect_rate", "side_px", "px_per_cell",
                             "x_mean", "y_mean", "x_std_cm", "y_std_cm", "heading_std_deg"])
        try:
            while True:
                label = input("\nDistance label (e.g. 15ft), blank to quit: ").strip()
                if not label:
                    break
                frames, sizes, poses = 0, {}, {}
                start = time.time()
                while time.time() - start < args.seconds:
                    got = src.read()
                    if got is None:
                        break
                    frame, ts = got
                    res = pipeline.process(frame, ts)
                    frames += 1
                    for m in res.markers:
                        sizes.setdefault(m.id, []).append(m.side_px)
                    for o in res.observations:
                        poses.setdefault(o.robot_id, []).append((o.x, o.y, o.heading_deg))
                    if server:
                        server.update(draw_overlay(frame, res, pipeline.geometry))
                res_str = f"{frame.shape[1]}x{frame.shape[0]}"
                print(f"  {frames} frames in {args.seconds:.0f} s ({frames / args.seconds:.1f} fps)")
                if not sizes:
                    print("  no markers detected")
                for mid, s in sorted(sizes.items()):
                    rate, side = len(s) / frames, float(np.mean(s))
                    print(f"  marker {mid:2d}: detected {rate * 100:5.1f}%  size {side:5.1f} px  {side / cells:4.1f} px/cell")
                    writer.writerow([label, res_str, "marker", mid, frames, round(rate, 3), round(side, 1),
                                     round(side / cells, 2), "", "", "", "", ""])
                for rid, p in sorted(poses.items()):
                    p = np.array(p)
                    h = np.radians(p[:, 2])
                    # circular standard deviation of heading
                    r = math.hypot(np.mean(np.cos(h)), np.mean(np.sin(h)))
                    h_std = math.degrees(math.sqrt(max(0.0, -2 * math.log(max(r, 1e-9)))))
                    print(f"  robot {rid}: seen {len(p) / frames * 100:5.1f}%  at ({p[:, 0].mean():.3f}, {p[:, 1].mean():.3f}) m  "
                          f"jitter x {p[:, 0].std() * 100:.1f} cm, y {p[:, 1].std() * 100:.1f} cm, heading {h_std:.1f} deg")
                    writer.writerow([label, res_str, "robot", rid, frames, round(len(p) / frames, 3), "", "",
                                     round(p[:, 0].mean(), 3), round(p[:, 1].mean(), 3), round(p[:, 0].std() * 100, 2),
                                     round(p[:, 1].std() * 100, 2), round(h_std, 2)])
                f.flush()
        except (KeyboardInterrupt, EOFError):
            pass
        finally:
            src.close()
            if server:
                server.close()
    print(f"Results appended to {args.csv}")


if __name__ == "__main__":
    main()
