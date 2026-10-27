"""Floor (extrinsic) calibration: tells the system where the camera is relative to the floor.

1. Put 4+ tape marks on the floor (more = better; spread them over the whole area, near and far).
   Measure their (x, y) in meters and list them in calib/floor_points.yaml, in the order you will click them.
   Recommended axes: x to the right as seen from the camera, y away from the camera.
2. Get a picture from the camera at the running resolution (the camera must not move afterwards):
     on the Pi:   python tools/record.py --snapshot calib/floor.jpg
     or, while vision.main --stream runs, save http://<pi-ip>:8080/snapshot.jpg
3. Click the marks in order:   python tools/calibrate_floor.py --image calib/floor.jpg
   Keys: U = undo last click, S/Enter = solve and save, Q = quit.
A 1 m grid is then drawn on the image; it should line up with the real floor.
Redo this whenever the camera is moved or re-aimed.
"""

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import cv2
import numpy as np
import yaml

from vision.camera import open_source
from vision.config import load_config, resolve_path
from vision.geometry import CameraModel, Geometry, solve_floor_pose

DISPLAY_W = 1280


def draw_grid(img, geom, extent=8):
    for v in range(-extent, extent + 1):
        for a, b in (([v, 0, 0], [v, extent * 1.5, 0]), ([-extent, v, 0], [extent, v, 0])):
            pts = np.linspace(a, b, 60)
            # keep only points in front of the camera
            cam_pts = (geom.pose.R @ pts.T + geom.pose.tvec).T
            pts = pts[cam_pts[:, 2] > 0.1]
            if len(pts) > 1:
                px = geom.world_to_pixels(pts).astype(np.int32)
                cv2.polylines(img, [px], False, (0, 255, 255) if v == 0 else (0, 180, 0), 2)


def main():
    cfg = load_config()
    cal = cfg["calibration"]
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--image", help="camera snapshot (default: grab a frame from the camera source)")
    ap.add_argument("--source", help="camera source to grab a frame from")
    ap.add_argument("--points", default=str(resolve_path(cfg, "calib/floor_points.yaml")))
    ap.add_argument("--out", default=str(resolve_path(cfg, cal["floor_pose"])))
    args = ap.parse_args()

    if args.image:
        frame = cv2.imread(args.image)
        if frame is None:
            raise SystemExit(f"Could not read {args.image}")
    else:
        src = open_source(cfg, args.source)
        for _ in range(10):  # let exposure settle
            got = src.read()
        src.close()
        frame = got[0]
    h, w = frame.shape[:2]

    intr = resolve_path(cfg, cal["intrinsics"])
    if intr.exists():
        camera = CameraModel.load(intr).scaled_to(w, h)
    else:
        print("WARNING: no lens calibration yet; using a rough FOV model (less accurate)")
        camera = CameraModel.from_fov(w, h, cfg["camera"].get("hfov_deg", 66))

    world = None
    if Path(args.points).exists():
        with open(args.points) as f:
            world = [tuple(p) for p in yaml.safe_load(f)["points"]]
        print(f"Loaded {len(world)} floor points from {args.points}")
    else:
        print(f"No {args.points}; you'll type each point's coordinates after clicking it.")
        world = []

    scale = DISPLAY_W / w
    clicks = []

    def on_mouse(event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            if args_points_mode and len(clicks) >= len(world):
                return
            clicks.append((x / scale, y / scale))
            if not args_points_mode:
                text = input(f"World x, y (m) of point {len(clicks)}: ")
                world.append(tuple(float(v) for v in text.replace(",", " ").split()))

    args_points_mode = len(world) > 0
    cv2.namedWindow("floor")
    cv2.setMouseCallback("floor", on_mouse)
    geom = None
    while True:
        view = frame.copy()
        if geom is not None:
            draw_grid(view, geom)
        for i, (u, v) in enumerate(clicks):
            cv2.drawMarker(view, (int(u), int(v)), (0, 0, 255), cv2.MARKER_CROSS, 40, 3)
            cv2.putText(view, f"{i + 1} {world[i]}", (int(u) + 12, int(v) - 12), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)
        if args_points_mode and len(clicks) < len(world):
            hint = f"Click point {len(clicks) + 1}/{len(world)} at {world[len(clicks)]}   (U=undo, Q=quit)"
        else:
            hint = "S/Enter = solve & save, U = undo, Q = quit" if geom is None else "Saved. Check the grid. Q = quit"
        cv2.putText(view, hint, (20, 50), cv2.FONT_HERSHEY_SIMPLEX, 1.4, (255, 255, 255), 3)
        cv2.imshow("floor", cv2.resize(view, (DISPLAY_W, int(h * scale))))
        key = cv2.waitKey(30) & 0xFF
        if key in (ord("q"), 27):
            break
        if key == ord("u") and clicks:
            clicks.pop()
            if not args_points_mode:
                world.pop()
            geom = None
        if key in (ord("s"), 13) and len(clicks) >= 4:
            pose, rms = solve_floor_pose(camera, clicks, world[:len(clicks)])
            geom = Geometry(camera, pose)
            cam = pose.camera_position
            print(f"Reprojection error {rms:.2f} px. Camera at x={cam[0]:.2f} y={cam[1]:.2f} m, height {cam[2]:.2f} m")
            if rms > 5:
                print("  Large error: check the click order matches floor_points.yaml and the measurements.")
            if cam[2] < 0:
                print("  Camera came out BELOW the floor: your x/y axes are probably mirrored (swap x sign).")
            pose.save(args.out, image_size=[w, h], rms_px=round(rms, 3),
                      image_points=[list(map(float, c)) for c in clicks],
                      world_points=[list(map(float, p)) for p in world[:len(clicks)]])
            print(f"Saved {args.out}")
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
