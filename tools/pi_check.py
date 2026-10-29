"""Self-check for the Pi: packages, detection on a synthetic 20 ft scene, camera capture, and speed.

    python tools/pi_check.py              # full check (camera must be connected)
    python tools/pi_check.py --no-camera  # skip the camera steps

Needs no printed markers. Saves a camera snapshot to results/pi_check.jpg.
"""

import argparse
import platform
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

results = []


def report(name, ok, detail=""):
    results.append((name, ok))
    print(f"  [{'PASS' if ok else 'FAIL'}] {name}" + (f": {detail}" if detail else ""))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--no-camera", action="store_true")
    ap.add_argument("--source", default="picam", help="camera source (default: picam)")
    ap.add_argument("--frames", type=int, default=60)
    args = ap.parse_args()

    print("[1/4] Python and packages")
    model = Path("/proc/device-tree/model")
    print(f"  {model.read_text().strip(chr(0)) if model.exists() else platform.platform()}, Python {platform.python_version()}")
    try:
        import cv2
        import numpy as np
        import yaml  # noqa: F401
        report("numpy / opencv / pyyaml", True, f"numpy {np.__version__}, opencv {cv2.__version__}")
        report("cv2.aruco ArucoDetector", hasattr(cv2, "aruco") and hasattr(cv2.aruco, "ArucoDetector"))
    except Exception as e:  # noqa: BLE001
        report("numpy / opencv / pyyaml", False, repr(e))
        return finish()
    if not args.no_camera and args.source == "picam":
        try:
            import picamera2  # noqa: F401
            report("picamera2", True)
        except Exception as e:  # noqa: BLE001
            report("picamera2", False, f"{e!r} (see docs/PI_GUIDE.md troubleshooting)")

    from vision.config import load_config
    from vision.geometry import CameraModel, FloorPose, Geometry
    from vision.pipeline import VisionPipeline
    from vision.robots import robot_specs_from_config
    from vision.sim import render_scene

    cfg = load_config()
    cfg["people"] = {"enabled": False}  # person detection runs on the laptop, not the Pi
    w, h = cfg["camera"]["width"], cfg["camera"]["height"]

    print(f"\n[2/4] Detection self-test: synthetic robots at 20 ft (6.1 m), {w}x{h}")
    geom = Geometry(CameraModel.from_fov(w, h, 66), FloorPose.look_at([0, 0, 2.0], [0, 4.5, 0.3]))
    specs = robot_specs_from_config(cfg)
    truth = [(1, -0.8, 6.1, -60.0), (2, 0.9, 6.1, -120.0)]
    img = render_scene(geom, truth, specs, (w, h), cfg["markers"]["dictionary"], blur=0.8, noise=3)
    pipe = VisionPipeline(cfg, geometry=geom)
    pipe.process(img, 0.0)  # warm-up
    t0 = time.perf_counter()
    for i in range(10):
        res = pipe.process(img, 0.1 * (i + 1))
    ms = (time.perf_counter() - t0) / 10 * 1000
    found = {o.robot_id: o for o in res.observations}
    errs = [float(np.hypot(found[r].x - x, found[r].y - y)) if r in found else None for r, x, y, _ in truth]
    ok = all(e is not None and e < 0.10 for e in errs)
    report("both robots found within 10 cm", ok, ", ".join(f"{e * 100:.1f} cm" if e is not None else "missed" for e in errs))
    report("processing speed", ms < 200, f"{ms:.0f} ms/frame -> up to {1000 / ms:.1f} fps")

    if args.no_camera:
        return finish()

    print(f"\n[3/4] Camera capture ({args.frames} frames)")
    from vision.camera import open_source
    try:
        src = open_source(cfg, args.source)
    except Exception as e:  # noqa: BLE001
        report("open camera", False, repr(e))
        return finish()
    try:
        frames = []
        t0 = time.time()
        for _ in range(args.frames):
            got = src.read()
            if got is None:
                break
            frames.append(got[0])
        dt = time.time() - t0
        if not frames:
            report("capture frames", False, "no frames received")
            return finish()
        fh, fw = frames[-1].shape[:2]
        report("capture frames", True, f"{len(frames)} frames at {fw}x{fh}, {len(frames) / dt:.1f} fps")
        brightness = float(frames[-1].mean())
        report("image not black/white", 15 < brightness < 240, f"mean brightness {brightness:.0f}/255")
        sharp = cv2.Laplacian(cv2.cvtColor(frames[-1], cv2.COLOR_BGR2GRAY), cv2.CV_64F).var()
        print(f"  sharpness score {sharp:.0f} (compare between focus settings; higher = sharper)")
        Path("results").mkdir(exist_ok=True)
        cv2.imwrite("results/pi_check.jpg", frames[-1])
        print("  saved results/pi_check.jpg")

        print("\n[4/4] Full pipeline on live frames")
        live = VisionPipeline(cfg)
        t0 = time.time()
        n = 0
        while time.time() - t0 < 5:
            got = src.read()
            if got is None:
                break
            res = live.process(*got)
            n += 1
        fps = n / (time.time() - t0)
        report("live pipeline rate", fps >= 5, f"{fps:.1f} fps (camera + detection), markers seen: {[m.id for m in res.markers]}")
    finally:
        src.close()
    return finish()


def finish():
    bad = [n for n, ok in results if not ok]
    print("\nALL CHECKS PASSED" if not bad else f"\n{len(bad)} CHECK(S) FAILED: {', '.join(bad)}")
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
