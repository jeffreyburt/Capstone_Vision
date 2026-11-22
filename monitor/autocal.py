"""One-button calibration for the hallway setup, run from the dashboard's Calibration page.

Step 1, floor (robots off the floor): the two floor tags plus the owner's tape measurements (distance in front of
the camera, distance left/right of centre, lens height) give the camera-to-floor pose. The marker fit gives the
floor plane; the tape fixes its scale. The same scale tells how many pixels larger detected squares look than they
are (corner blur), which corrects distances measured from robot tag sizes.

Step 2, robots (placed facing forward, away from the camera, heads facing forward too): every visible head tag is
measured in the chassis frame (chassis heading = straight down the hallway; the rear tag sits a known distance
behind the chassis centre). Robot 2's head is then swept through pan and tilt with the chassis still, and its pan
and tilt pivots and scales are fitted so moving-head tags stay usable while the head tracks.

Every write backs up the previous file to calib/history/ first; nothing measured earlier is deleted.
"""
import copy
import math
import re
import threading
import time
from datetime import datetime
from pathlib import Path

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parent.parent

DEFAULTS = {"tag_size_m": 0.15, "left_id": 1, "right_id": 0, "rear_offset_m": 0.14, "frames": 15}


# ----------------------------------------------------------------------------------------------- pure geometry
def kabsch(a, b):
    """Rotation R minimizing |a @ R.T - b| for centered point sets a, b (N x 3)."""
    u, _, vt = np.linalg.svd(a.T @ b)
    d = np.sign(np.linalg.det(vt.T @ u.T))
    return vt.T @ np.diag([1, 1, d]) @ u.T


def nominal_offsets(face_deg, size):
    """Corner offsets of a vertical tag facing face_deg in the chassis frame (same order as HeadGeometry)."""
    yaw = math.radians(face_deg)
    right = np.array([-math.sin(yaw), math.cos(yaw), 0])
    up = np.array([0, 0, 1])
    half = size / 2
    return half * np.array([up - right, up + right, -up + right, -up - right])


def tag_entry(corners, face_deg, size, mount):
    """head_pose marker entry (center_m + rotation_vector_rad) reproducing chassis-frame corners."""
    import cv2
    corners = np.asarray(corners, float)
    center = corners.mean(axis=0)
    rotation = kabsch(nominal_offsets(face_deg, size), corners - center)
    rvec = cv2.Rodrigues(rotation)[0].ravel()
    return {"mount": mount, "center_m": [round(float(v), 4) for v in center],
            "rotation_vector_rad": [round(float(v), 4) for v in rvec]}


def fit_floor(camera, corners, size_m, layout, lens_height_m):
    """Floor pose from flat tags {id: (4, 2) pixels} and tape positions {id: (x, y)}.

    Returns (FloorPose, report). The marker fit sets the plane and direction; the tape sets the scale."""
    from tools.floor_from_markers import floor_pose_from_markers
    from vision.geometry import FloorPose
    pose, rep = floor_pose_from_markers(camera, corners, size_m, None)
    ratios = [lens_height_m / rep["height_m"]]
    ratios += [math.hypot(*layout[i]) / rep["markers"][i]["floor_dist_m"] for i in layout]
    scale = float(np.median(ratios))
    if not 0.85 <= scale <= 1.15:
        raise ValueError(f"tags and tape disagree by {abs(scale - 1) * 100:.0f}% (scale {scale:.3f}); "
                         "check the measurements, the tag size and that the tags lie flat")
    report = {"scale": scale, "height_m": rep["height_m"] * scale, "pitch_deg": rep["pitch_deg"], "tags": {}}
    for tid, (x, y) in layout.items():
        m = rep["markers"][tid]
        fx, fy = m["x"] * scale, m["y"] * scale
        if x and (fx > 0) != (x > 0):
            raise ValueError(f"tag {tid} appears on the other side of the camera: swap the left/right tag IDs")
        err = math.hypot(fx - x, fy - y)
        report["tags"][int(tid)] = {"measured": [round(x, 3), round(y, 3)], "fitted": [round(fx, 3), round(fy, 3)],
                                    "error_m": round(err, 3), "tilt_deg": round(m["off_plane_deg"], 2)}
        if err > 0.15:
            raise ValueError(f"tag {tid} fits {err * 100:.0f} cm from its measured spot; check the measurements")
        if m["off_plane_deg"] > 5:
            raise ValueError(f"tag {tid} is {m['off_plane_deg']:.1f} deg off the floor plane: lay it flat")
    if abs(report["height_m"] - lens_height_m) > 0.05:
        raise ValueError(f"fitted camera height {report['height_m']:.3f} m vs measured {lens_height_m:.3f} m")
    side = float(np.median([np.mean(np.linalg.norm(c - np.roll(c, -1, axis=0), axis=1)) for c in corners.values()]))
    report["corner_blur_px"] = round(max(0.0, side * (1 - 1 / scale)), 2)
    return FloorPose(pose.rvec, np.asarray(pose.tvec) * scale), report


def chassis_reference(tag_worlds, rear_id, rear_offset_m, heading_deg=90.0):
    """Chassis (x, y) from the rear tag's world centre, the chassis heading and the tag's offset behind the centre,
    plus every tag's corners in the chassis frame. tag_worlds: {tag id: [(4, 3) world corners per frame]}."""
    h = math.radians(heading_deg)
    c, s = math.cos(h), math.sin(h)
    rear = np.median(np.asarray(tag_worlds[rear_id]), axis=0).mean(axis=0)
    cx, cy = rear[0] + rear_offset_m * c, rear[1] + rear_offset_m * s  # rear tag is behind: -offset along heading

    def to_chassis(world):
        d = np.asarray(world) - [cx, cy, 0]
        return np.column_stack([c * d[:, 0] + s * d[:, 1], -s * d[:, 0] + c * d[:, 1], d[:, 2]])

    tags = {tid: to_chassis(np.median(np.asarray(v), axis=0)) for tid, v in tag_worlds.items()}
    return (cx, cy), tags, to_chassis


def predict_head_corners(neutral_corners, mount, units, neutral_units, params):
    """Corners of a head tag at `units` with params (pan_px, pan_py, tilt_px, tilt_pz, dpu_pan, dpu_tilt)."""
    from vision.joint_pose import _rotate
    ppx, ppy, tpx, tpz, dpu_p, dpu_t = params
    pan, tilt = np.radians((np.asarray(units, float) - neutral_units) * [dpu_p, dpu_t])
    corners = np.asarray(neutral_corners, float)
    if mount in ("tilt", "tilt_only"):
        corners = _rotate(corners, np.array([tpx, 0, tpz]), tilt, "tilt")
    if mount in ("pan", "tilt"):
        corners = _rotate(corners, np.array([ppx, ppy, 0]), pan, "pan")
    return corners


def fit_head(samples, neutral_corners, mounts, neutral_units, init):
    """Least-squares pan/tilt pivots and scales from sweep samples [(units, {tag: chassis corners})].
    Returns (params, rms_m)."""
    from scipy.optimize import least_squares
    neutral_units = np.asarray(neutral_units, float)

    def residual(p):
        out = []
        for units, tags in samples:
            for tid, measured in tags.items():
                if tid in neutral_corners:
                    out.append((predict_head_corners(neutral_corners[tid], mounts[tid], units, neutral_units, p)
                                - measured).ravel())
        return np.concatenate(out)

    result = least_squares(residual, np.asarray(init, float), loss="soft_l1", f_scale=0.02)
    rms = float(np.sqrt(np.mean(residual(result.x) ** 2)))
    return [float(v) for v in result.x], rms


def replace_head_pose_block(text, robot_id, head_pose):
    """config.yaml text with robot `robot_id`'s embedded head_pose block replaced (other text kept)."""
    lines = text.splitlines(keepends=True)
    start = next(i for i, l in enumerate(lines) if re.match(rf"^  - id: {robot_id}\s*$", l))
    hp = next(i for i in range(start + 1, len(lines)) if lines[i].startswith("    head_pose:"))
    end = hp + 1
    while end < len(lines) and (lines[end].startswith("      ") or not lines[end].strip()):
        end += 1
    dumped = yaml.safe_dump(head_pose, sort_keys=False, default_flow_style=None)
    block = ["    head_pose:  # written by the dashboard calibration\n"] + ["      " + l + "\n" for l in dumped.splitlines()]
    return "".join(lines[:hp] + block + lines[end:])


def set_marker_setting(text, key, value, comment):
    """Set `markers.<key>` in config.yaml text, adding the line under `markers:` if missing."""
    pattern = rf"^  {key}:.*$"
    line = f"  {key}: {value}  # {comment}"
    if re.search(pattern, text, flags=re.M):
        return re.sub(pattern, line, text, count=1, flags=re.M)
    found = re.search(r"^markers:[^\n]*\n", text, flags=re.M)
    if found is None:
        raise ValueError("config has no markers: section")
    return text[:found.end()] + line + "\n" + text[found.end():]


def wall_limits(width_m, robot_half_m=0.15, clearance_m=0.20):
    """Robot-centre x limits for a usable space `width_m` wide centred on the camera: walls at +-width/2, robot
    centres kept half a robot plus the clearance inside them. Returns ([lo, hi], [left wall, right wall])."""
    half = width_m / 2
    limit = half - robot_half_m - clearance_m
    if limit < 0.1:
        raise ValueError(f"{width_m:.2f} m leaves no room: robot centres would have to stay within {limit:+.2f} m of "
                         "the centre line; reduce the clearance or check the width")
    return [round(-limit, 3), round(limit, 3)], [round(-half, 3), round(half, 3)]


def set_formation_setting(text, key, value, comment):
    """Set `control.formation.<key>` in config.yaml text (4-space indent), adding it after `formation:` if missing."""
    line = f"    {key}: {value}  # {comment}"
    pattern = rf"^    {key}:.*$"
    if re.search(pattern, text, flags=re.M):
        return re.sub(pattern, lambda _: line, text, count=1, flags=re.M)
    found = re.search(r"^  formation:[^\n]*\n", text, flags=re.M)
    if found is None:
        raise ValueError("config has no control.formation section")
    return text[:found.end()] + line + "\n" + text[found.end():]


# ----------------------------------------------------------------------------------------------- live runner
class AutoCalibration:
    def __init__(self, monitor, heads, calibration):
        self.monitor, self.heads, self.cal = monitor, heads, calibration
        self.lock = threading.Lock()
        self.thread = None
        self.cancel = threading.Event()
        self.state = {"step": None, "running": False, "log": [], "error": None, "floor": None, "robots": None,
                      "inputs": None}

    # ---- state
    def snapshot(self):
        with self.lock:
            return copy.deepcopy(self.state)

    def _log(self, text):
        with self.lock:
            self.state["log"].append(f"{datetime.now():%H:%M:%S} {text}")
            del self.state["log"][:-200]

    def _run(self, step, fn, *args):
        with self.lock:
            if self.state["running"]:
                raise RuntimeError("a calibration step is already running")
            self.state.update(step=step, running=True, error=None, log=[])
        self.cancel.clear()

        def work():
            try:
                result = fn(*args)
                with self.lock:
                    self.state[step] = result
                self._log("done")
            except Exception as error:  # reported on the page; nothing half-applied (writes come last)
                with self.lock:
                    self.state["error"] = str(error)
                self._log(f"stopped: {error}")
            finally:
                with self.lock:
                    self.state["running"] = False
        self.thread = threading.Thread(target=work, daemon=True)
        self.thread.start()

    def stop(self):
        self.cancel.set()

    # ---- shared helpers
    def _cfg(self):
        from vision.config import load_config
        d = (self.monitor.cfg or {}).get("_dir")
        return load_config(str(Path(d) / "config.yaml") if d else None)

    def _frames(self, cfg, n, timeout=30):
        from camstream.receiver import FrameReceiver
        rx = FrameReceiver(cfg, ["birdseye"])
        out, last, end = [], None, time.time() + timeout
        try:
            while len(out) < n and time.time() < end:
                if self.cancel.is_set():
                    raise RuntimeError("cancelled")
                f = rx.wait_new("birdseye", last, 2)
                if f is not None:
                    last = f.seq
                    out.append(f.image)
        finally:
            rx.close()
        if len(out) < n:
            raise RuntimeError("the main camera is not sending frames")
        return out

    def _detector(self, cfg):
        from vision.markers import MarkerDetector
        m = cfg.get("markers", {})
        return MarkerDetector(m.get("dictionary", "DICT_4X4_50"), adaptive_thresh_constant=m.get("adaptive_thresh_constant", 3))

    # ---- step 1: floor
    def start_floor(self, data):
        inputs = {**DEFAULTS, **{k: v for k, v in data.items() if k in DEFAULTS}}
        for key in ("distance_m", "offset_m", "lens_height_m"):
            v = data.get(key)
            if isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v):
                raise ValueError(f"{key} must be a number in meters")
            inputs[key] = float(v)
        if not 0.5 <= inputs["distance_m"] <= 8 or not 0.1 <= inputs["offset_m"] <= 3 or not 0.1 <= inputs["lens_height_m"] <= 2:
            raise ValueError("distance 0.5-8 m, offset 0.1-3 m, lens height 0.1-2 m")
        if inputs["left_id"] == inputs["right_id"]:
            raise ValueError("left and right tags need different IDs")
        with self.lock:
            self.state["inputs"] = inputs
        self._run("floor", self._floor, inputs)

    def _floor(self, p):
        from vision.geometry import CameraModel
        from vision.config import resolve_path
        cfg = self._cfg()
        self._log("collecting frames of the floor tags")
        images = self._frames(cfg, p["frames"])
        detector = self._detector(cfg)
        ids = (p["left_id"], p["right_id"])
        seen = {i: [] for i in ids}
        for img in images:
            dets = detector.detect(img)
            for i in ids:
                match = [d for d in dets if d.id == i]
                if len(match) > 1:
                    raise RuntimeError(f"tag {i} is visible more than once: take the robots off the floor for this step")
                if match:
                    seen[i].append(match[0].corners)
        missing = [i for i, v in seen.items() if len(v) < len(images) // 2]
        if missing:
            raise RuntimeError(f"floor tag(s) {missing} not seen; check they are in view, flat and lit")
        corners = {i: np.mean(v, axis=0) for i, v in seen.items()}
        h, w = images[0].shape[:2]
        camera = CameraModel.load(resolve_path(cfg, cfg["calibration"]["intrinsics"])).scaled_to(w, h)
        layout = {p["left_id"]: (-p["offset_m"], p["distance_m"]), p["right_id"]: (p["offset_m"], p["distance_m"])}
        pose, report = fit_floor(camera, corners, p["tag_size_m"], layout, p["lens_height_m"])
        self._log(f"floor fit: camera {report['height_m']:.3f} m high, {report['pitch_deg']:.1f} deg down; "
                  f"tag errors {', '.join(f'{v['error_m'] * 100:.1f} cm' for v in report['tags'].values())}; "
                  f"corner blur {report['corner_blur_px']:.1f} px")
        # --- apply (backups first)
        target = resolve_path(cfg, cfg["calibration"]["floor_pose"])
        self.cal._backup(target)
        pose.save(target, method=f"dashboard calibration: tags {list(layout)} ({p['tag_size_m']} m), "
                                 f"+y towards their midpoint, scaled to tape", height_m=round(report["height_m"], 4),
                  pitch_deg=round(report["pitch_deg"], 3), tape=layout and {int(k): list(v) for k, v in layout.items()})
        cfg_path = Path(cfg["_dir"]) / "config.yaml"
        self.cal._backup(cfg_path)
        text = cfg_path.read_text()
        text = set_marker_setting(text, "corner_blur_px", report["corner_blur_px"],
                                  "px detected squares look larger (dashboard floor calibration)")
        text = set_marker_setting(text, "size_distance_scale", 1.0, "superseded by corner_blur_px")
        cfg_path.write_text(text)
        self.cal.set_measurements({"lens_height_m": p["lens_height_m"], "floor_marker_size_m": p["tag_size_m"],
                                   "floor_marker_ids": [p["left_id"], p["right_id"]], "floor_forward_mid": True,
                                   "floor_tag_spacing_owner_m": 2 * p["offset_m"]})
        self.cal.log("floor-applied", f"dashboard calibration; height {report['height_m']:.3f} m")
        report["snapshot"] = self.cal.write_snapshot("dashboard floor calibration")
        self._log("restarting vision with the new floor")
        try:
            self.monitor.restart_vision()
        except Exception as error:
            self._log(f"restart vision yourself: {error}")
        return report

    # ---- usable space (walls)
    def start_walls(self, data):
        width, clearance = data.get("width_m"), data.get("clearance_m", 0.20)
        line = data.get("line_y_m")
        if line is not None and (isinstance(line, bool) or not isinstance(line, (int, float)) or not 0.5 <= line <= 10):
            raise ValueError("robots' line distance must be 0.5-10 m")
        for name, v in (("width_m", width), ("clearance_m", clearance)):
            if isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v):
                raise ValueError(f"{name} must be a number in meters")
        if not 0.5 <= width <= 20 or not 0 <= clearance <= 1:
            raise ValueError("usable width 0.5-20 m, clearance 0-1 m")
        self._run("walls", self._walls, float(width), float(clearance), None if line is None else float(line))

    def _walls(self, width, clearance, line=None):
        cfg = self._cfg()
        half = max([float(r.get("center_offset_m", 0.15)) for r in cfg.get("robots", [])] or [0.15])
        limits, walls = wall_limits(width, half, clearance)
        formation = (cfg.get("control") or {}).get("formation") or {}
        side = float(formation.get("distance_m", 1.0))
        report = {"width_m": width, "walls_m": walls, "x_limits_m": limits, "clearance_m": clearance,
                  "triangle_side_m": side, "fits_centred": side / 2 <= limits[1]}
        self._log(f"walls at x = {walls[0]:+.2f} / {walls[1]:+.2f} m; robot centres kept within "
                  f"{limits[0]:+.2f} .. {limits[1]:+.2f} m ({half:.2f} m half robot + {clearance:.2f} m clearance)")
        if not report["fits_centred"]:
            self._log(f"note: the {side:.2f} m triangle puts robots {side / 2:.2f} m to each side, beyond the limit; "
                      "they will be held at the limit (narrower triangle)")
        cfg_path = Path(cfg["_dir"]) / "config.yaml"
        self.cal._backup(cfg_path)
        text = cfg_path.read_text()
        text = set_formation_setting(text, "usable_width_m", width, "usable space centred on the camera (dashboard calibration)")
        text = set_formation_setting(text, "x_limits_m", limits,
                                     f"robot centres: walls at +-{width / 2:.2f} m minus {half:.2f} m half robot and {clearance:.2f} m clearance")
        if line is not None:
            text = set_formation_setting(text, "line_y_m", line, "robots' line: distance in front of the camera (dashboard calibration)")
            report["line_y_m"] = line
            self._log(f"robots' line {line:.2f} m in front of the camera; the triangle opens and closes from there")
        cfg_path.write_text(text)
        live = self.monitor.cfg.setdefault("control", {}).setdefault("formation", {})
        live.update(usable_width_m=width, x_limits_m=limits, **({} if line is None else {"line_y_m": line}))
        self.cal.set_measurements({"usable_width_m": width, "wall_clearance_m": clearance})
        self.cal.log("walls-applied", f"usable width {width:.2f} m, robot x limits {limits}")
        self._log("restarting the robot controller so following uses the new limits")
        try:
            self._log(self.monitor.restart_control().get("message", "controller restarted"))
        except Exception as error:
            self._log(f"restart control.main yourself: {error}")
        return report

    # ---- step 2: robots
    def start_robots(self, data):
        if data.get("confirmed") is not True:
            raise ValueError("confirm that the robots face forward and Robot 2's head may move")
        with self.lock:
            inputs = dict(self.state.get("inputs") or DEFAULTS)
        offset = data.get("rear_offset_m", inputs.get("rear_offset_m", 0.14))
        if isinstance(offset, bool) or not isinstance(offset, (int, float)) or not 0 <= offset <= 0.5:
            raise ValueError("rear_offset_m must be 0-0.5 m")
        inputs["rear_offset_m"] = float(offset)
        self._run("robots", self._robots, inputs)

    def _tag_worlds(self, cfg, specs, geometry, images, blur):
        from vision.joint_pose import tag_pose_world
        detector = self._detector(cfg)
        floor_ids = set((cfg.get("calibration") or {}).get("floor_marker_ids", []))
        worlds = {s.id: {} for s in specs}
        by_tag = {mid: s for s in specs for mid in s.faces}
        for img in images:
            for d in detector.detect(img):
                spec = by_tag.get(d.id)
                if spec is None or d.id in floor_ids:
                    continue
                w = tag_pose_world(d, spec.marker_size_m, geometry, blur)
                if w is not None:
                    worlds[spec.id].setdefault(d.id, []).append(w)
        return worlds

    def _robots(self, p):
        from vision.geometry import load_geometry
        from vision.robots import robot_specs_from_config
        cfg = self._cfg()
        specs = robot_specs_from_config(cfg)
        blur = float((cfg.get("markers") or {}).get("corner_blur_px", 0.0))
        moving = [s for s in specs if s.head is not None and s.head.needs_feedback]
        nodes = {n.robot_id: n for n in self.monitor.nodes if n.kind == "robot"}
        neutral = {}
        for s in moving:
            node = nodes[s.id]
            self._log(f"robot {s.id}: stopping head tracking and holding the head where you set it")
            self.heads.post(node, "stop")
            time.sleep(0.5)
            code, msg = self.heads.post(node, "engage")
            if code >= 300:
                raise RuntimeError(f"robot {s.id} head: {msg}")
            time.sleep(1.0)
            units = self.heads.state(node).get("positions")
            if not units or None in units:
                raise RuntimeError(f"robot {s.id}: no head servo feedback")
            neutral[s.id] = [int(u) for u in units]
            self._log(f"robot {s.id}: head forward at pan {units[0]}, tilt {units[1]}")
        self._log("measuring every robot tag with the robots facing forward")
        images = self._frames(cfg, p.get("frames", 15))
        geometry = load_geometry(cfg, (images[0].shape[1], images[0].shape[0]))
        worlds = self._tag_worlds(cfg, specs, geometry, images, blur)
        report, head_poses = {}, {}
        rear_offset = float(p.get("rear_offset_m", 0.14))
        for s in specs:
            rear_id = next((mid for mid, face in s.faces.items() if abs((face - 180 + 180) % 360 - 180) < 1), None)
            tags = {tid: v for tid, v in worlds[s.id].items() if len(v) >= len(images) // 2}
            if rear_id not in tags:
                raise RuntimeError(f"robot {s.id}: its rear tag {rear_id} is not clearly visible; face it away from the camera")
            (cx, cy), local, to_chassis = chassis_reference(tags, rear_id, rear_offset)
            old = next(r for r in cfg["robots"] if int(r["id"]) == s.id).get("head_pose") or {}
            entries = {}
            for tid, corners in local.items():
                mount = "fixed" if s.id not in neutral else ((old.get("markers") or {}).get(str(tid), (old.get("markers") or {}).get(tid, {})).get("mount")
                                                               or ("tilt" if tid == rear_id else "pan"))
                entry = tag_entry(corners, s.faces[tid], s.marker_size_m, mount)
                if s.id in neutral and tid != rear_id:
                    entry["center_only"] = True  # side tags on a moving head: position only (pan/heading trade-off)
                entries[str(tid)] = entry
            hp = {k: old[k] for k in ("max_rms_px", "heading_window_deg") if k in old}
            if s.id in neutral:
                hp.update(neutral_units=neutral[s.id], degrees_per_unit=list(old.get("degrees_per_unit", [0.24, 0.24])),
                          pan_pivot_m=list(old.get("pan_pivot_m", [0, 0, 0])), tilt_pivot_m=list(old.get("tilt_pivot_m", [0, 0, 0.3])))
            else:
                hp.update(neutral_units=[0, 0], degrees_per_unit=[1, 1], pan_pivot_m=[0, 0, 0], tilt_pivot_m=[0, 0, 0])
            hp["markers"] = entries
            head_poses[s.id] = hp
            report[s.id] = {"position": [round(cx, 3), round(cy, 3)], "tags": sorted(int(t) for t in entries)}
            self._log(f"robot {s.id}: at ({cx:.2f}, {cy:.2f}), tags {sorted(int(t) for t in entries)}")
            if s.id in neutral:
                report[s.id].update(self._sweep(cfg, s, nodes[s.id], neutral[s.id], hp, to_chassis, geometry, blur, local))
        self._apply_robots(cfg, head_poses)
        return {str(k): v for k, v in report.items()}

    def _sweep(self, cfg, spec, node, neutral, hp, to_chassis, geometry, blur, local):
        """Move the head through pan and tilt, fit the pivots/scales, validate, update hp in place."""
        limits = ((cfg.get("system") or {}).get("robots") or {}).get(spec.id, {}).get("head_tracking_limits") \
            or [[neutral[0] - 150, neutral[0] + 150], [neutral[1] - 150, min(1000, neutral[1] + 30)]]
        clamp = lambda u, i: int(min(max(u, limits[i][0]), limits[i][1]))
        targets = [[clamp(neutral[0] + d, 0), neutral[1]] for d in (-180, -110, -50, 50, 110, 180)]
        targets += [[neutral[0], clamp(neutral[1] + d, 1)] for d in (-220, -150, -80, 25)]
        targets += [[clamp(neutral[0] + a, 0), clamp(neutral[1] + b, 1)] for a, b in ((-100, -120), (100, -120))]
        samples = []
        url = self.heads._url(node, "/api/start")
        import json
        import urllib.request
        try:
            for t in targets:
                if self.cancel.is_set():
                    raise RuntimeError("cancelled")
                body = json.dumps({"a": t, "b": [t[0] + 1, t[1]], "duration": 1.5, "cycles": 1, "pause": 0.2}).encode()
                urllib.request.urlopen(urllib.request.Request(url, data=body, method="POST",
                                                              headers={"Content-Type": "application/json"}), timeout=3)
                end = time.time() + 10
                while time.time() < end and self.heads.state(node).get("running"):
                    time.sleep(0.2)
                time.sleep(0.4)
                units = self.heads.state(node).get("positions")
                worlds = self._tag_worlds(cfg, [spec], geometry, self._frames(cfg, 5), blur)[spec.id]
                tags = {str(tid): to_chassis(np.median(np.asarray(v), axis=0)) for tid, v in worlds.items() if len(v) >= 3}
                if units and None not in units and tags:
                    samples.append((list(units), tags))
                self._log(f"robot {spec.id} head at pan {units[0] if units else '?'}, tilt {units[1] if units else '?'}: "
                          f"tags {sorted(int(t) for t in tags)}")
        finally:
            body = json.dumps({"a": [neutral[0], neutral[1]], "b": [neutral[0] + 1, neutral[1]], "duration": 1.5,
                               "cycles": 1, "pause": 0.2}).encode()
            try:
                urllib.request.urlopen(urllib.request.Request(url, data=body, method="POST",
                                                              headers={"Content-Type": "application/json"}), timeout=3)
            except OSError:
                pass
        if len(samples) < 6:
            raise RuntimeError(f"only {len(samples)} usable head positions; keep Robot {spec.id} in clear view")
        neutral_corners = {tid: np.asarray(local[int(tid)]) for tid in hp["markers"]}
        mounts = {tid: hp["markers"][tid]["mount"] for tid in hp["markers"]}
        init = [hp["pan_pivot_m"][0], hp["pan_pivot_m"][1], hp["tilt_pivot_m"][0], hp["tilt_pivot_m"][2],
                hp["degrees_per_unit"][0], hp["degrees_per_unit"][1]]
        params, rms = fit_head(samples, neutral_corners, mounts, neutral, init)
        self._log(f"robot {spec.id} head fit: corner RMS {rms * 100:.1f} cm, pan {params[4]:.3f} deg/unit, "
                  f"tilt {params[5]:.3f} deg/unit")
        if rms > 0.03:
            raise RuntimeError(f"head model error {rms * 100:.1f} cm is too large (limit 3 cm); check the head is "
                               "tight and nothing blocked the tags")
        hp.update(pan_pivot_m=[round(params[0], 4), round(params[1], 4), 0.0],
                  tilt_pivot_m=[round(params[2], 4), 0.0, round(params[3], 4)],
                  degrees_per_unit=[round(params[4], 5), round(params[5], 5)], valid_units=[list(l) for l in limits])
        swept = [[min(u[0] for u, _ in samples), max(u[0] for u, _ in samples)],
                 [min(u[1] for u, _ in samples), max(u[1] for u, _ in samples)]]
        return {"head_fit_rms_m": round(rms, 4), "head_positions": len(samples), "swept_units": swept,
                "degrees_per_unit": hp["degrees_per_unit"]}

    def _apply_robots(self, cfg, head_poses):
        cfg_path = Path(cfg["_dir"]) / "config.yaml"
        self.cal._backup(cfg_path)
        text = cfg_path.read_text()
        stamp = datetime.now().strftime("%Y-%m-%d")
        for rid, hp in head_poses.items():
            path = Path(cfg["_dir"]) / "calib" / f"robot{rid}_head_pose.yaml"
            old = yaml.safe_load(path.read_text()) if path.exists() else {}
            self.cal._backup(path)
            new = {"status": f"active_dashboard_calibration_{stamp}", "head_pose": hp, "reference_date": stamp,
                   "user_rear_offset_m": old.get("user_rear_offset_m", -0.14)}
            path.write_text(yaml.safe_dump(new, sort_keys=False))
            text = replace_head_pose_block(text, rid, hp)
        cfg_path.write_text(text)
        self.cal.log("robots-applied", "dashboard calibration: head tags and Robot 2 head motion")
        self.cal.write_snapshot("dashboard robot calibration")
        self._log("restarting vision with the new robot calibration")
        try:
            self.monitor.restart_vision()
        except Exception as error:
            self._log(f"restart vision yourself: {error}")
