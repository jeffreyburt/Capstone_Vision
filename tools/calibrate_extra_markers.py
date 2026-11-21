"""Calibrate a robot's extra head tags from live frames where an already-calibrated tag is also visible.

    python tools/calibrate_extra_markers.py --robot 1 --marker 3 --mount fixed
    python tools/calibrate_extra_markers.py --robot 2 --marker 5 --mount pan --apply

Keep the robot still. For each frame, the calibrated tags give the chassis pose; the extra tag's own pose
(solvePnP on its 10 cm square) is moved into the chassis frame (and, for a pan-mounted tag, rotated back to
the pan neutral using the live servo reading). The averaged corners become a new `head_pose.markers` entry
(center_m + rotation_vector_rad, the same form as the existing tags). Validation: the new tag alone must
reproduce the chassis pose from the existing tags. Never moves wheels or servos.
"""
import argparse
import json
import math
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

import cv2
import numpy as np


def kabsch(a, b):
    """Rotation R minimizing |a @ R.T - b| for centered point sets a, b (N x 3)."""
    h = a.T @ b
    u, _, vt = np.linalg.svd(h)
    d = np.sign(np.linalg.det(vt.T @ u.T))
    return vt.T @ np.diag([1, 1, d]) @ u.T


def nominal_offsets(face_deg, size):
    """Corner offsets of a vertical tag facing face_deg in the chassis frame (same as HeadGeometry)."""
    yaw = math.radians(face_deg)
    right = np.array([-math.sin(yaw), math.cos(yaw), 0])
    up = np.array([0, 0, 1])
    half = size / 2
    return half * np.array([up - right, up + right, -up + right, -up - right])


def main():
    from camstream.receiver import FrameReceiver
    from vision.config import load_config
    from vision.geometry import load_geometry
    from vision.joint_pose import _rotate, fit_chassis
    from vision.pipeline import VisionPipeline
    from vision.robots import estimate_robots

    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--robot", type=int, required=True)
    ap.add_argument("--marker", type=int, required=True)
    ap.add_argument("--mount", choices=["fixed", "pan"], required=True)
    ap.add_argument("--frames", type=int, default=40)
    ap.add_argument("--apply", action="store_true", help="write the tag into calib/robot<N>_head_pose.yaml and config.yaml")
    args = ap.parse_args()

    cfg = load_config(None)
    cfg["people"] = {"enabled": False}
    pipe = VisionPipeline(cfg)
    spec = next(s for s in pipe.specs if s.id == args.robot)
    if args.marker not in spec.faces:
        raise SystemExit(f"marker {args.marker} is not one of robot {args.robot}'s faces {sorted(spec.faces)}")
    if args.marker in spec.head.mounts:
        raise SystemExit(f"marker {args.marker} is already calibrated")
    if args.mount == "pan" and not spec.head.needs_feedback:
        raise SystemExit("a pan-mounted tag needs a robot with servo feedback")
    size = spec.marker_size_m
    s = size / 2
    obj = np.array([[-s, s, 0], [s, s, 0], [s, -s, 0], [-s, -s, 0]])  # ArUco marker frame
    rx = FrameReceiver(cfg, ["birdseye"])
    time.sleep(3)  # joint feedback warm-up
    samples, last, deadline = [], None, time.time() + 120
    geometry = None
    while len(samples) < args.frames and time.time() < deadline:
        f = rx.wait_new("birdseye", last, 3)
        if f is None:
            continue
        last = f.seq
        img = f.image
        if geometry is None:
            geometry = load_geometry(cfg, (img.shape[1], img.shape[0]))
        t = f.local_capture_time or time.time()
        markers = [m for m in pipe.detector.detect(img) if m.id not in pipe.floor_marker_ids]
        extra = [m for m in markers if m.id == args.marker]
        known = [m for m in markers if m.id in spec.head.mounts]
        if len(extra) != 1 or not known:
            continue
        joints = pipe.joints.positions_at(t) if pipe.joints else {}
        units = (joints or {}).get(args.robot)
        if spec.head.needs_feedback and units is None:
            continue
        poses = estimate_robots(known, [spec], geometry, joints or {}, {})
        if not poses:
            continue
        pose = poses[0]
        ok, rvecs, tvecs, errs = cv2.solvePnPGeneric(obj, extra[0].corners, geometry.camera.K, geometry.camera.dist,
                                                     flags=cv2.SOLVEPNP_IPPE_SQUARE)
        if not ok:
            continue
        floor = geometry.pose
        h = math.radians(pose.heading)
        c, sn = math.cos(h), math.sin(h)
        to_chassis = np.array([[c, sn, 0], [-sn, c, 0], [0, 0, 1]])
        best = None
        for rvec, tvec in zip(rvecs, tvecs):
            cam = (cv2.Rodrigues(rvec)[0] @ obj.T).T + tvec.ravel()
            world = (floor.R.T @ (cam - floor.tvec.ravel()).T).T
            chassis = (world - [pose.x, pose.y, 0]) @ to_chassis.T
            if args.mount == "pan":
                pan = math.radians((units[0] - spec.head.neutral_units[0]) * spec.head.degrees_per_unit[0])
                chassis = _rotate(chassis, spec.head.pan_pivot, -pan, "pan")
            # Pick the IPPE solution whose outward direction is closest to the tag's nominal face direction.
            normal = np.cross(chassis[1] - chassis[0], chassis[3] - chassis[0])
            normal /= np.linalg.norm(normal)
            face = math.radians(spec.faces[args.marker])
            score = normal @ np.array([math.cos(face), math.sin(face), 0])
            if best is None or abs(score) > abs(best[0]):
                best = score, chassis
        samples.append({"corners": best[1], "pose": (pose.x, pose.y, pose.heading),
                        "units": None if units is None else [float(v) for v in units]})
        print(f"  frame {len(samples)}/{args.frames}: chassis ({pose.x:.3f}, {pose.y:.3f}, {pose.heading:.1f})", flush=True)
    rx.close()
    if len(samples) < max(10, args.frames // 2):
        raise SystemExit(f"only {len(samples)} usable frames; keep both tags in view and the robot still")

    corners = np.median([smp["corners"] for smp in samples], axis=0)
    center = corners.mean(axis=0)
    nominal = nominal_offsets(spec.faces[args.marker], size)
    rotation = kabsch(nominal, corners - center)
    rvec = cv2.Rodrigues(rotation)[0].ravel()
    spread = float(np.percentile([np.linalg.norm(smp["corners"].mean(axis=0) - center) for smp in samples], 95))
    entry = {"mount": args.mount, "center_m": [round(float(v), 4) for v in center],
             "rotation_vector_rad": [round(float(v), 4) for v in rvec]}

    # Validation: the new tag alone, through the same fit as live tracking, vs the existing tags' pose.
    from vision.joint_pose import HeadGeometry
    head_cfg = next(r for r in cfg["robots"] if int(r["id"]) == args.robot)["head_pose"]
    trial_cfg = json.loads(json.dumps(head_cfg, default=float))
    trial_cfg["markers"] = {str(args.marker): entry}
    trial = HeadGeometry.from_config(trial_cfg, spec)
    rx = FrameReceiver(cfg, ["birdseye"])
    checks, last, deadline = [], None, time.time() + 40
    while len(checks) < 15 and time.time() < deadline:
        f = rx.wait_new("birdseye", last, 3)
        if f is None:
            continue
        last = f.seq
        t = f.local_capture_time or time.time()
        markers = [m for m in pipe.detector.detect(f.image) if m.id not in pipe.floor_marker_ids]
        joints = pipe.joints.positions_at(t) if pipe.joints else {}
        units = (joints or {}).get(args.robot) if trial.needs_feedback else trial.neutral_units
        known = estimate_robots([m for m in markers if m.id in spec.head.mounts], [spec], geometry, joints or {}, {})
        extra = [m for m in markers if m.id == args.marker]
        if not known or len(extra) != 1 or units is None:
            continue
        c4 = trial.corners(args.marker, units)
        fit = None if c4 is None else fit_chassis(extra[0], c4, geometry, math.radians(known[0].heading), 30.0,
                                                  spec.head.heading_window)
        if fit is None:
            continue
        (x, y), hd = fit
        checks.append((math.hypot(x - known[0].x, y - known[0].y),
                       abs((math.degrees(hd) - known[0].heading + 180) % 360 - 180)))
    rx.close()
    pos_err = float(np.percentile([c[0] for c in checks], 95)) if checks else None
    head_err = float(np.percentile([c[1] for c in checks], 95)) if checks else None
    report = {"robot": args.robot, "marker": args.marker, "entry": entry, "frames": len(samples),
              "center_spread_p95_m": spread, "validation_frames": len(checks),
              "validation_position_p95_m": pos_err, "validation_heading_p95_deg": head_err}
    print(json.dumps(report, indent=1))
    out = ROOT / "calib" / "candidates" / f"robot{args.robot}_marker{args.marker}.json"
    out.parent.mkdir(exist_ok=True)
    out.write_text(json.dumps(report, indent=1) + "\n")
    ok = checks and pos_err <= 0.10 and head_err <= 12 and spread <= 0.03
    if not ok:
        raise SystemExit("REJECTED: needs validation p95 <= 10 cm / 12 deg and corner spread <= 3 cm")
    if args.apply:
        import yaml
        p = ROOT / "calib" / f"robot{args.robot}_head_pose.yaml"
        data = yaml.safe_load(p.read_text())
        data["head_pose"]["markers"][str(args.marker)] = entry
        p.write_text(yaml.safe_dump(data, sort_keys=False))
        print(f"applied to {p.relative_to(ROOT)}; mirror it in config.yaml and restart vision.main")


if __name__ == "__main__":
    main()
