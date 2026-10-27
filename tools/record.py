"""Record test clips or snapshots from the camera (works headless on the Pi).

    python tools/record.py --out clips/test1.mp4 --seconds 30
    python tools/record.py --snapshot calib/floor.jpg

Clips can be replayed on a laptop:  python -m vision.main --source clips/test1.mp4 --show
"""

import argparse
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import cv2

from vision.camera import open_source
from vision.config import load_config


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--source", help="camera source (default from config)")
    ap.add_argument("--out", help="video file to write (.mp4 or .avi)")
    ap.add_argument("--seconds", type=float, default=20.0)
    ap.add_argument("--snapshot", metavar="FILE", help="save a single frame instead")
    args = ap.parse_args()
    if not (args.out or args.snapshot):
        ap.error("give --out or --snapshot")

    cfg = load_config()
    src = open_source(cfg, args.source)
    try:
        if args.snapshot:
            for _ in range(15):  # let exposure / focus settle
                frame, _ = src.read()
            Path(args.snapshot).parent.mkdir(parents=True, exist_ok=True)
            cv2.imwrite(args.snapshot, frame)
            print(f"Saved {args.snapshot} ({frame.shape[1]}x{frame.shape[0]})")
            return

        frame, t0 = src.read()
        h, w = frame.shape[:2]
        fps = float(cfg["camera"].get("fps", 30))
        Path(args.out).parent.mkdir(parents=True, exist_ok=True)
        fourcc = cv2.VideoWriter_fourcc(*("mp4v" if args.out.lower().endswith(".mp4") else "MJPG"))
        writer = cv2.VideoWriter(args.out, fourcc, fps, (w, h))
        n, start = 0, time.time()
        print(f"Recording {w}x{h} to {args.out} for {args.seconds:.0f} s ...")
        while time.time() - start < args.seconds:
            writer.write(frame)
            n += 1
            got = src.read()
            if got is None:
                break
            frame = got[0]
        writer.release()
        real_fps = n / (time.time() - start)
        print(f"Wrote {n} frames ({real_fps:.1f} fps actual).")
        if abs(real_fps - fps) > 3:
            print(f"  Note: the file is tagged {fps:.0f} fps, so replays run {fps / real_fps:.1f}x real speed.")
    finally:
        src.close()


if __name__ == "__main__":
    main()
