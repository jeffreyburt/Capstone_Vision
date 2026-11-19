"""Floor calibration from ArUco markers lying flat on the floor: no tape marks, no clicking.

    python tools/floor_from_markers.py --ids 0 1 --forward-id 0 --size-cm 15

Needs the lens calibration (tools/calibrate_camera.py or the GUI) first. Each flat marker's four corners
give the floor plane relative to the camera; with two or more markers spread apart it's well determined.
The world frame it writes: origin on the floor directly below the camera, z up, +y along the floor
towards the --forward-id marker, +x to the right. Measure a marker's distance along the floor with a
tape to check the printed numbers.
"""

import argparse
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import cv2
import numpy as np

from vision.camera import open_source
from vision.config import load_config, resolve_path
from vision.geometry import CameraModel, FloorPose
from vision.markers import MarkerDetector


def collect_corners(source, detector, ids, frames, timeout_s=30):
    """Average each marker's corners over several frames (the markers are still, so this cuts noise)."""
    seen = {i: [] for i in ids}
    t0, size = time.time(), None
    while time.time() - t0 < timeout_s and min(len(v) for v in seen.values()) < frames:
        got = source.read()
        if got is None:
            break
        size = (got[0].shape[1], got[0].shape[0])
        for d in detector.detect(got[0]):
            if d.id in seen:
                seen[d.id].append(d.corners)
    missing = [i for i, v in seen.items() if not v]
    if missing:
        raise SystemExit(f"marker(s) {missing} not seen; check they're in view, flat and well lit")
    return {i: np.mean(v, axis=0) for i, v in seen.items()}, size


def floor_pose_from_markers(camera, corners, size_m, forward_id):
    """FloorPose plus per-marker results, from {id: (4, 2) corner pixels} of markers lying flat on the floor."""
    s = size_m / 2
    obj = np.array([[-s, s, 0], [s, s, 0], [s, -s, 0], [-s, -s, 0]])  # ArUco corner order, marker frame
    pts, centers, normals = [], {}, {}
    for mid, c in corners.items():
        ok, rvec, tvec = cv2.solvePnP(obj, c, camera.K, camera.dist, flags=cv2.SOLVEPNP_IPPE_SQUARE)
        if not ok:
            raise SystemExit(f"could not solve the pose of marker {mid}")
        R, t = cv2.Rodrigues(rvec)[0], tvec.ravel()
        pts.append((R @ obj.T).T + t)
        centers[mid] = t
        normals[mid] = R[:, 2] if R[:, 2] @ -t > 0 else -R[:, 2]  # pointing up, towards the camera

    # Floor plane through every corner (camera frame), normal pointing up towards the camera.
    P = np.vstack(pts)
    centroid = P.mean(axis=0)
    n = np.linalg.svd(P - centroid)[2][2]
    if n @ -centroid < 0:
        n = -n
    height = float(n @ -centroid)
    foot = -height * n  # floor point directly below the camera, in camera coordinates

    def along_floor(p):
        v = p - foot
        return v - (v @ n) * n

    y = along_floor(centers[forward_id])
    y /= np.linalg.norm(y)
    x = np.cross(y, n)
    R_wc = np.column_stack([x, y, n])  # world axes expressed in camera coordinates (world -> camera rotation)
    pose = FloorPose(cv2.Rodrigues(R_wc)[0], foot)

    report = {"height_m": height, "pitch_deg": float(np.degrees(np.arcsin(np.clip(-n[2], -1, 1)))),
              "markers": {}}
    for mid, c in centers.items():
        w = R_wc.T @ (c - foot)
        tilt = float(np.degrees(np.arccos(np.clip(normals[mid] @ n, -1, 1))))
        report["markers"][mid] = {"x": float(w[0]), "y": float(w[1]), "floor_dist_m": float(np.hypot(w[0], w[1])),
                                  "off_plane_deg": tilt}
    return pose, report


def main():
    cfg = load_config()
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--ids", type=int, nargs="+", required=True, help="IDs of the markers lying on the floor")
    ap.add_argument("--forward-id", type=int, help="marker that defines +y (default: the first of --ids)")
    ap.add_argument("--size-cm", type=float, required=True, help="black-square edge of the floor markers")
    ap.add_argument("--source", help="camera source (default from config)")
    ap.add_argument("--frames", type=int, default=10, help="frames to average")
    ap.add_argument("--out", default=str(resolve_path(cfg, cfg["calibration"]["floor_pose"])))
    args = ap.parse_args()
    forward = args.forward_id if args.forward_id is not None else args.ids[0]
    if forward not in args.ids:
        raise SystemExit("--forward-id must be one of --ids")

    intr = resolve_path(cfg, cfg["calibration"]["intrinsics"])
    if not intr.exists():
        raise SystemExit(f"no lens calibration at {intr}; calibrate the lens first")
    source = open_source(cfg, args.source)
    try:
        corners, size = collect_corners(source, MarkerDetector(cfg["markers"]["dictionary"]), args.ids, args.frames)
    finally:
        source.close()
    camera = CameraModel.load(intr).scaled_to(*size)
    pose, rep = floor_pose_from_markers(camera, corners, args.size_cm / 100, forward)

    print(f"camera height {rep['height_m']:.3f} m, tilted {rep['pitch_deg']:.2f} deg below horizontal")
    for mid, m in rep["markers"].items():
        print(f"  marker {mid}: x {m['x']:+.3f}  y {m['y']:.3f} m   {m['floor_dist_m']:.3f} m along the floor from below the camera"
              f"   (off the floor plane by {m['off_plane_deg']:.1f} deg)")
    ids = list(rep["markers"])
    for i, a in enumerate(ids):
        for b in ids[i + 1:]:
            ma, mb = rep["markers"][a], rep["markers"][b]
            print(f"  marker {a} to {b}: {np.hypot(ma['x'] - mb['x'], ma['y'] - mb['y']):.3f} m apart")
    pose.save(args.out, method=f"floor markers {args.ids} ({args.size_cm} cm), +y towards marker {forward}",
              height_m=round(rep["height_m"], 4), pitch_deg=round(rep["pitch_deg"], 3))
    print(f"Saved {args.out}")


if __name__ == "__main__":
    main()
