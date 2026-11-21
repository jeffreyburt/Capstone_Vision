"""Calibration checklist and stored calibration data for the system dashboard (/calibration).

Everything a setup needs is reachable from the browser: a step-by-step checklist with live status, every
stored calibration value, and buttons that run the existing tools (never wheel motion).

Stored data (all in Git, under calib/):
    calibration_record.json   owner measurements, operator confirmations, history  (written by this module)
    camera_intrinsics.yaml    lens model              floor_pose_36cm.yaml   camera-to-floor transform
    robot<ID>_head_pose.yaml  head geometry; an `active*` status overrides config.yaml (vision.config)
    snapshots/                consolidated copies of everything above, written on every apply/export
    history/                  backups of any file an apply replaced
    candidates/, recordings/  unapplied results and raw head recordings

Safety: floor/lens/head fits write *candidates*; only an explicit Apply copies them into the active files
(old one backed up). The head capture moves only the right head and needs an explicit confirmation. No
function here drives wheels or arms navigation.
"""

import copy
import hashlib
import json
import math
import os
import shutil
import subprocess
import sys
import threading
import time
import urllib.error
import urllib.request
from datetime import datetime, timezone
from pathlib import Path

import yaml

from vision.config import resolve_path

ROOT = Path(__file__).resolve().parent.parent

DEFAULT_MEASUREMENTS = {
    "lens_height_m": 0.395,            # owner: camera lens height above the floor
    "robot_marker_size_m": 0.10,       # black square on robot heads
    "floor_marker_size_m": 0.15,       # borrowed floor tags
    "floor_marker_ids": [0, 1],
    "floor_forward_id": 0,             # marker that defines +y
    "floor_forward_mid": False,        # true: +y points at the tags' midpoint (tags placed left/right of straight ahead)
    "floor_min_y_fraction": 0.0,       # ignore marker copies above this image fraction (duplicate IDs on robots)
    "floor_tag_spacing_owner_m": 1.03,
    "marker_center_height_m": 0.30,
    "rear_tag_offset_m": {"x": -0.14, "y": 0.0},
    "robot_body_size_m": 0.30,
}

STEP_ORDER = ["network", "measure", "lens", "floor", "head1", "head2", "verify", "wheels"]


def now_iso():
    return datetime.now(timezone.utc).astimezone().isoformat(timespec="seconds")


def _read_yaml(path):
    try:
        with open(path) as f:
            return yaml.safe_load(f) or {}
    except (OSError, yaml.YAMLError):
        return None


def _atomic_write(path, text):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_suffix(path.suffix + ".tmp")
    tmp.write_text(text)
    os.replace(tmp, path)


def _sha(path):
    try:
        return hashlib.sha256(Path(path).read_bytes()).hexdigest()[:12]
    except OSError:
        return None


class Job:
    """One background tool run with a rolling log. Only one job runs at a time."""

    def __init__(self, kind, argv, cwd):
        self.kind, self.argv, self.started, self.ended = kind, argv, time.time(), None
        self.lines, self.returncode, self.proc = [], None, None
        self._lock = threading.Lock()
        self.proc = subprocess.Popen(argv, cwd=cwd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
                                     bufsize=1, env={**os.environ, "PYTHONUNBUFFERED": "1"})
        threading.Thread(target=self._pump, daemon=True).start()

    def _pump(self):
        for line in self.proc.stdout:
            with self._lock:
                self.lines.append(line.rstrip())
                del self.lines[:-400]
        self.returncode = self.proc.wait()
        self.ended = time.time()

    @property
    def running(self):
        return self.returncode is None

    def stop(self):
        if self.running:
            self.proc.terminate()

    def status(self):
        with self._lock:
            tail = list(self.lines[-60:])
        return {"kind": self.kind, "running": self.running, "returncode": self.returncode, "started": self.started,
                "ended": self.ended, "log": tail}


class CalibrationManager:
    def __init__(self, cfg, live=None, vision_gui=None):
        """live: callable returning the monitor snapshot (nodes, vision message). vision_gui: callable returning
        the vision GUI port (for lens calibration), or None."""
        self.cfg = cfg
        self.dir = resolve_path(cfg, "calib")
        self.record_path = self.dir / "calibration_record.json"
        self.live = live or (lambda: {})
        self.vision_gui = vision_gui or (lambda: None)
        self.job = None
        self.verify = None
        self._lock = threading.Lock()
        self.record = self._load_record()

    # ------------------------------------------------------------------ record (owner values, confirmations)
    def _load_record(self):
        try:
            rec = json.loads(self.record_path.read_text())
        except (OSError, ValueError):
            rec = {}
        rec.setdefault("version", 1)
        rec["measurements"] = {**copy.deepcopy(DEFAULT_MEASUREMENTS), **rec.get("measurements", {})}
        rec.setdefault("confirmations", {})
        rec.setdefault("history", [])
        rec.setdefault("verification", None)
        return rec

    def _save_record(self):
        _atomic_write(self.record_path, json.dumps(self.record, indent=2) + "\n")

    def log(self, event, detail=""):
        self.record["history"].append({"at": now_iso(), "event": event, "detail": detail})
        del self.record["history"][:-200]
        self._save_record()

    def set_measurements(self, values):
        """Validate and store owner measurements."""
        m = self.record["measurements"]
        new = copy.deepcopy(m)
        for key, value in values.items():
            if key not in DEFAULT_MEASUREMENTS:
                raise ValueError(f"unknown measurement {key!r}")
            default = DEFAULT_MEASUREMENTS[key]
            if key == "floor_marker_ids":
                if (not isinstance(value, list) or not 2 <= len(value) <= 8 or
                        not all(isinstance(v, int) and 0 <= v < 1000 for v in value) or len(set(value)) != len(value)):
                    raise ValueError("floor_marker_ids needs 2-8 distinct marker IDs")
            elif key == "rear_tag_offset_m":
                if not isinstance(value, dict) or not all(isinstance(value.get(a), (int, float)) and abs(value[a]) < 1
                                                          for a in ("x", "y")):
                    raise ValueError("rear_tag_offset_m needs numeric x and y in meters (|v| < 1)")
                value = {"x": float(value["x"]), "y": float(value["y"])}
            elif key == "floor_forward_id":
                if not isinstance(value, int):
                    raise ValueError("floor_forward_id must be an integer marker ID")
            elif key == "floor_forward_mid":
                if not isinstance(value, bool):
                    raise ValueError("floor_forward_mid must be true or false")
            else:
                if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
                    raise ValueError(f"{key} must be a finite number")
                value = float(value)
                limit = 1.0 if key == "floor_min_y_fraction" else 10.0
                low = 0.0 if key == "floor_min_y_fraction" else 0.001
                if not low <= value < limit or (key != "floor_min_y_fraction" and value <= 0):
                    raise ValueError(f"{key} is out of range")
            new[key] = value
        if new["floor_forward_id"] not in new["floor_marker_ids"]:
            raise ValueError("floor_forward_id must be one of floor_marker_ids")
        self.record["measurements"] = new
        self.log("measurements", ", ".join(sorted(values)))
        return new

    def confirm(self, step, note="", value=True):
        if step not in STEP_ORDER:
            raise ValueError("unknown step")
        if value:
            self.record["confirmations"][step] = {"at": now_iso(), "note": str(note)[:300]}
        else:
            self.record["confirmations"].pop(step, None)
        self.log("confirm" if value else "unconfirm", step)

    # ------------------------------------------------------------------ stored values
    def collect(self):
        """Every calibration value the system uses, read live from the stored files."""
        cfg, d = self.cfg, self.dir
        out = {"generated": now_iso(), "measurements": self.record["measurements"]}

        intr_path = resolve_path(cfg, (cfg.get("calibration") or {}).get("intrinsics"))
        intr = _read_yaml(intr_path) if intr_path else None
        if intr:
            K = intr.get("K") or [[None] * 3] * 3
            out["intrinsics"] = {"file": _rel(intr_path), "image_size": intr.get("image_size"),
                                 "fx": K[0][0], "fy": K[1][1], "cx": K[0][2], "cy": K[1][2],
                                 "dist": intr.get("dist"), "rms_px": intr.get("rms_px"), "views": intr.get("views"),
                                 "sha": _sha(intr_path)}
        else:
            out["intrinsics"] = None

        floor_path = resolve_path(cfg, (cfg.get("calibration") or {}).get("floor_pose"))
        floor = _read_yaml(floor_path) if floor_path else None
        out["floor"] = None if not floor else {
            "file": _rel(floor_path), "rvec": floor.get("rvec"), "tvec": floor.get("tvec"),
            "camera_position_m": floor.get("camera_position_m"), "height_m": floor.get("height_m"),
            "pitch_deg": floor.get("pitch_deg"), "method": floor.get("method"), "sha": _sha(floor_path)}

        robots = []
        for r in cfg.get("robots") or []:
            rid = int(r["id"])
            f = d / f"robot{rid}_head_pose.yaml"
            fy = _read_yaml(f) or {}
            hp = r.get("head_pose") or {}
            robots.append({"id": rid, "faces": r.get("faces"), "center_offset_m": r.get("center_offset_m"),
                           "file": _rel(f), "status": fy.get("status"), "reference_date": fy.get("reference_date"),
                           "head_pose": hp, "live_validation": _scalars(fy.get("live_validation")),
                           "sha": _sha(f)})
        out["robots"] = robots

        wheels = {}
        for path in sorted((ROOT / "robot" / "config").glob("*.json")):
            try:
                wheels[path.name] = json.loads(path.read_text())
            except (OSError, ValueError):
                wheels[path.name] = None
        out["wheels"] = wheels
        out["markers"] = cfg.get("markers")
        out["limits"] = {rid: (c or {}).get("limits") for rid, c in ((cfg.get("control") or {}).get("robots") or {}).items()}
        out["waypoint_motion"] = (cfg.get("control") or {}).get("waypoint_motion")
        out["files"] = [{"file": _rel(p), "bytes": p.stat().st_size, "modified": datetime.fromtimestamp(
            p.stat().st_mtime).isoformat(timespec="seconds")}
            for p in sorted(d.glob("*")) if p.is_file()]
        return out

    def write_snapshot(self, reason):
        data = self.collect()
        data["reason"] = reason
        name = datetime.now().strftime("calibration_%Y%m%d_%H%M%S.json")
        _atomic_write(self.dir / "snapshots" / name, json.dumps(data, indent=2, default=str) + "\n")
        return _rel(self.dir / "snapshots" / name)

    # ------------------------------------------------------------------ checklist
    def steps(self):
        data, live = self.collect(), (self.live() or {})
        m, conf = self.record["measurements"], self.record["confirmations"]
        nodes = {n["key"]: n for n in live.get("nodes", [])}
        vision = (live.get("vision") or {}).get("message") or {}
        poses = {r["id"]: r for r in vision.get("robots", [])}
        main_cam = live.get("main_camera") or {}
        steps = []

        def step(sid, title, why, how, status, detail, actions=(), manual=False):
            c = conf.get(sid)
            steps.append({"id": sid, "title": title, "why": why, "how": list(how), "status": status,
                          "detail": detail, "actions": list(actions), "manual": manual, "confirmed": c})

        # 1 network
        keys = ["master"] + [k for k in nodes if k.startswith("robot")]
        down = [nodes[k]["name"] for k in keys if k in nodes and nodes[k]["level"] == "bad"]
        if not live.get("nodes"):
            st, det = "todo", "Dashboard has no probe data yet."
        elif down:
            st, det = "attention", "Not reachable: " + ", ".join(down)
        elif all(k in nodes and nodes[k]["level"] in ("ok", "warn") for k in keys):
            st, det = "done", "Main Pi and both robots answer."
        else:
            st, det = "todo", "Waiting for node status."
        step("network", "Power up and check connections",
             "Every later step talks to the Pis through the main Pi.",
             ["Power the main Pi (camera) and both robot Pis; wait about a minute.",
              "Connect the laptop to the main Pi's Ethernet.",
              "The Network diagram on the main dashboard should show every machine green."], st, det)

        # 2 measurements
        rear = m["rear_tag_offset_m"]
        step("measure", "Record physical measurements",
             "Tape-measure values anchor the camera fit and head geometry; the fit is checked against them.",
             ["Measure the camera lens height above the floor.",
              "Check the black-square size on one robot tag and one floor tag.",
              "Measure how far behind the chassis center the rear tag sits.",
              "Enter the values below and press Save. Re-measure whenever something is moved."],
             "done" if "measure" in conf else "manual",
             f"lens {m['lens_height_m']:.3f} m · robot tag {m['robot_marker_size_m']*100:.1f} cm · floor tag "
             f"{m['floor_marker_size_m']*100:.1f} cm · rear tag x {rear['x']:+.2f} m",
             [{"id": "measurements", "label": "Edit measurements"}], manual=True)

        # 3 lens
        intr = data["intrinsics"]
        cam_cfg = (((self.cfg.get("streams") or {}).get("birdseye") or {}).get("camera") or {})
        want = [cam_cfg.get("width"), cam_cfg.get("height")]
        if not intr:
            st, det = "todo", "No lens calibration file."
        elif want[0] and list(intr["image_size"] or []) != want:
            st, det = "attention", f"Calibrated at {intr['image_size']}, camera streams {want}: recalibrate."
        elif (intr.get("rms_px") or 99) > 1.0:
            st, det = "attention", f"RMS {intr['rms_px']:.2f} px is above 1 px: recapture with a flatter board."
        else:
            st, det = "done", f"{intr['views']} views, RMS {intr['rms_px']:.3f} px, fx {intr['fx']:.0f}, size {intr['image_size']}."
        step("lens", "Calibrate the main camera lens",
             "Removes lens distortion. Needed once, and again if the lens, focus or resolution changes.",
             ["Start vision with its GUI (python -m vision.main --gui). Print a 9x6-inner-corner checkerboard on flat card.",
              "Press Start, hold the board in view; it auto-captures when held still in a new spot.",
              "Cover the whole frame including corners, tilting the board. Collect about 20 views.",
              "Press Calibrate. Aim for RMS under 1 px. It is saved at once."], st, det,
             [{"id": "lens", "label": "Open lens calibration"}])

        # 4 floor
        floor = data["floor"]
        if not floor:
            st, det = "todo", "No floor pose file."
        else:
            dh = abs((floor.get("height_m") or 0) - m["lens_height_m"])
            if dh > 0.05:
                st, det = "attention", (f"Fitted camera height {floor['height_m']:.3f} m differs from the measured "
                                         f"{m['lens_height_m']:.3f} m by {dh*100:.1f} cm: recheck tags and re-measure.")
            else:
                st, det = "done", (f"Camera height {floor['height_m']:.3f} m (measured {m['lens_height_m']:.3f}), "
                                   f"pitch {floor['pitch_deg']:.1f}° down; {floor['method']}")
        step("floor", "Calibrate the floor from the floor tags",
             "Tells the system where the camera is relative to the floor. Redo if the camera is bumped or moved.",
             ["Lay the floor tags flat and fixed (IDs and size are in the measurements). Do not move the camera after.",
              "Remove or cover duplicate copies of those IDs, or set the 'ignore top fraction' value so only floor tags count.",
              "Press Measure floor; wait for the log to finish. Check the tag spacing against a tape measure.",
              "If it looks right, press Apply floor. The previous file is backed up."], st, det,
             [{"id": "floor_measure", "label": "Measure floor"}, {"id": "floor_apply", "label": "Apply candidate"}])

        # 5 head1
        r1 = next((r for r in data["robots"] if r["id"] == 1), None)
        pose1 = poses.get(1) or {}
        if not r1 or not r1["head_pose"]:
            st, det = "todo", "No Robot 1 head geometry."
        elif "head1" not in conf:
            st, det = "manual", "Confirm the left head is mechanically secured and its rear tag is visible."
        elif live.get("nodes") and pose1.get("pose_status") not in (None, "Secured fixed head"):
            st, det = "attention", f"Live pose status: {pose1.get('pose_status')}"
        else:
            st, det = "done", f"Fixed rear tag 2. Live: {pose1.get('pose_status') or 'not seen yet'}"
        step("head1", "Robot 1: secure the left head",
             "Robot 1 has no working head servos, so its tag must not move relative to the chassis.",
             ["Zip-tie or otherwise lock the left head level and facing the chassis front.",
              "Make sure the rear tag (ID 2) faces backward and is clear of obstructions.",
              "Tick the confirmation, then check the live pose status below turns to 'Secured fixed head'."],
             st, det, [{"id": "confirm:head1", "label": "Confirm secured"}], manual=True)

        # 6 head2
        r2 = next((r for r in data["robots"] if r["id"] == 2), None)
        hp = (r2 or {}).get("head_pose") or {}
        pose2 = poses.get(2) or {}
        node2 = nodes.get("robot2") or {}
        units = None
        for chk in node2.get("checks", []):
            if chk["label"] == "Head tracker":
                units = chk["text"]
        if not hp:
            st, det = "todo", "No Robot 2 head geometry."
        elif live.get("nodes") and pose2.get("pose_status") in ("Head outside calibrated movement range",
                                                                 "Calibrated marker not reliably visible"):
            st, det = "attention", f"Live pose status: {pose2['pose_status']}"
        else:
            vr = hp.get("valid_units") or []
            lv = (r2 or {}).get("live_validation") or {}
            det = (f"Neutral pan/tilt {hp.get('neutral_units')}, tested range {vr}. "
                   f"Last validation: {_fmt_validation(lv)}. Live: {pose2.get('pose_status') or 'not seen yet'}")
            st = "done" if (r2 or {}).get("status", "").startswith("active") else "attention"
        step("head2", "Robot 2: calibrate the moving head",
             "The right head pans and tilts; the camera sees its tags move. The fit learns where the real chassis is.",
             ["Park Robot 2 on clear floor, wheels OFF, in view of the camera, facing the same way as at reference.",
              "Put the head at its neutral position (pan 391, tilt 770) with torque on; stop head tracking on its dashboard.",
              "Press Record training sweep and wait; the head moves slowly inside the tested range. Keep hands clear.",
              "Press Record validation sweep (a second, independent run).",
              "Press Fit model, check the validation numbers, then Apply fit. Outside the tested range the pose is dropped."],
             st, det,
             [{"id": "head_capture", "label": "Record sweep"}, {"id": "head_fit", "label": "Fit model"},
              {"id": "head_apply", "label": "Apply fit"}])
        steps[-1]["live_units"] = units

        # 7 verify
        v = self.record.get("verification")
        both = all(poses.get(i, {}).get("pose_compensated") for i in (1, 2))
        if v and v.get("p95_position_m") is not None and v["p95_position_m"] <= 0.06 and v.get("frames", 0) >= 20:
            st, det = "done", (f"{v['at']}: {v['frames']} frames per robot, p95 jitter R1 "
                               f"{v['robots'].get('1', {}).get('p95_position_m', float('nan')) * 100:.1f} cm, "
                               f"R2 {v['robots'].get('2', {}).get('p95_position_m', float('nan')) * 100:.1f} cm")
        elif v:
            st, det = "attention", f"Last check {v['at']} exceeded the 6 cm limit or had too few frames: recheck calibration."
        elif both:
            st, det = "todo", "Both robots currently have compensated poses; run the stationary check."
        else:
            st, det = "todo", "Needs both robots visible with compensated poses."
        step("verify", "Verify with robots stationary",
             "A final check that calibrated positions stay still when the robots are still.",
             ["Leave both robots stationary in view and heads at neutral. Do not enable click driving.",
              "Press Run stationary check. It samples for 15 seconds and records the spread (p95 under 6 cm passes)."],
             st, det, [{"id": "verify_run", "label": "Run stationary check"}])

        # 8 wheels
        w = data["wheels"]
        step("wheels", "Confirm wheel configuration",
             "Wheel order and signs live in each robot's JSON and are loaded by its drive service.",
             ["Check the values below match the robots (Robot 1: M1 disconnected, three wheels).",
              "Wheel direction and speed tests move motors and are done on the floor by a person using "
              "control/drive_test.py, not from this page.",
              "Tick the confirmation after a person has verified direction on the floor."],
             "done" if "wheels" in conf else "manual",
             "Files: " + ", ".join(sorted(w)) if w else "No wheel config files found.",
             [{"id": "confirm:wheels", "label": "Confirm verified"}], manual=True)
        return steps

    # ------------------------------------------------------------------ jobs
    def job_status(self):
        return None if self.job is None else self.job.status()

    def _start(self, kind, argv):
        if self.job is not None and self.job.running:
            raise RuntimeError(f"another calibration job is running ({self.job.kind}); wait or stop it")
        self.job = Job(kind, [sys.executable, *argv], str(ROOT))
        self.log("job-start", kind)

    def stop_job(self):
        if self.job is not None:
            self.job.stop()

    def floor_measure(self):
        m = self.record["measurements"]
        out = self.dir / "candidates" / "floor_pose_candidate.yaml"
        out.parent.mkdir(parents=True, exist_ok=True)
        out.unlink(missing_ok=True)
        forward = ["--forward-mid"] if m.get("floor_forward_mid") else ["--forward-id", str(m["floor_forward_id"])]
        argv = [str(ROOT / "tools" / "floor_from_markers.py"), "--ids", *map(str, m["floor_marker_ids"]),
                *forward, "--size-cm", f"{m['floor_marker_size_m'] * 100:.3f}",
                "--min-y-fraction", str(m["floor_min_y_fraction"]), "--out", str(out)]
        self._start("floor_measure", argv)

    def floor_candidate(self):
        c = _read_yaml(self.dir / "candidates" / "floor_pose_candidate.yaml")
        return c

    def _backup(self, path):
        path = Path(path)
        if path.exists():
            dest = self.dir / "history" / f"{datetime.now():%Y%m%d_%H%M%S}_{path.name}"
            dest.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(path, dest)

    def floor_apply(self):
        if self.job is not None and self.job.running:
            raise RuntimeError("a job is still running")
        cand_path = self.dir / "candidates" / "floor_pose_candidate.yaml"
        cand = self.floor_candidate()
        if not cand or "rvec" not in cand:
            raise RuntimeError("no floor candidate: run Measure floor first")
        height = cand.get("height_m") or 0
        measured = self.record["measurements"]["lens_height_m"]
        if abs(height - measured) > 0.10:
            raise RuntimeError(f"candidate camera height {height:.3f} m is more than 10 cm from the measured "
                               f"{measured:.3f} m; fix the tags or measurement before applying")
        target = resolve_path(self.cfg, self.cfg["calibration"]["floor_pose"])
        self._backup(target)
        shutil.copy2(cand_path, target)
        self.log("floor-applied", f"height {height:.3f} m")
        return {"snapshot": self.write_snapshot("floor applied"), "restart": "restart vision.main to load the new floor pose"}

    def head_capture(self, role, confirmed):
        if not confirmed:
            raise RuntimeError("confirmation required: the right head will move")
        if role not in ("train", "validation"):
            raise ValueError("role must be train or validation")
        (self.dir / "recordings").mkdir(parents=True, exist_ok=True)
        out = self.dir / "recordings" / f"right_head_{role}_{datetime.now():%Y%m%d_%H%M%S}.json"
        self._start(f"head_capture_{role}", [str(ROOT / "tools" / "capture_right_head_motion.py"),
                                              "--execute", "--out", str(out)])

    def recordings(self):
        files = sorted((self.dir / "recordings").glob("right_head_*.json"))
        out = {"train": [], "validation": []}
        for f in files:
            kind = "validation" if "_validation_" in f.name else "train"
            out[kind].append(f.name)
        return out

    def head_fit(self):
        rec = self.recordings()
        if not rec["train"] or not rec["validation"]:
            raise RuntimeError("need at least one training and one validation recording")
        rd = self.dir / "recordings"
        argv = [str(ROOT / "tools" / "fit_head_motion.py"), "--train", *[str(rd / n) for n in rec["train"]],
                "--validation", str(rd / rec["validation"][-1]),
                "--out", str(self.dir / "candidates" / "right_head_fit_candidate.json")]
        self._start("head_fit", argv)

    def head_candidate(self):
        try:
            return json.loads((self.dir / "candidates" / "right_head_fit_candidate.json").read_text())
        except (OSError, ValueError):
            return None

    def head_apply(self, confirmed):
        cand = self.head_candidate()
        if not confirmed:
            raise RuntimeError("confirmation required")
        if self.job is not None and self.job.running:
            raise RuntimeError("a job is still running")
        if not cand or "candidate_head_pose" not in cand:
            raise RuntimeError("no fit candidate: run Fit model first")
        val = (cand.get("independent_validation") or {}).get("rms_px")
        vs = cand.get("validation_stability") or {}
        if val is None or val > 3.0 or vs.get("position_drift_p95_m", 1) > 0.10:
            raise RuntimeError("candidate fails the acceptance limits (validation RMS ≤ 3 px, p95 drift ≤ 10 cm); "
                               "recapture and refit")
        target = self.dir / "robot2_head_pose.yaml"
        old = _read_yaml(target) or {}
        self._backup(target)
        new = {"status": "active_limited_range_dashboard_fit", "head_pose": _plain(cand["candidate_head_pose"]),
               "reference_date": datetime.now().strftime("%Y-%m-%d"),
               "user_rear_offset_m": old.get("user_rear_offset_m", self.record["measurements"]["rear_tag_offset_m"]["x"]),
               "live_validation": {"source": "dashboard fit", "validation_rms_px": val,
                                   "validation_position_drift_p95_m": vs.get("position_drift_p95_m"),
                                   "validation_heading_deviation_p95_deg": vs.get("heading_deviation_p95_deg")}}
        _atomic_write(target, yaml.safe_dump(new, sort_keys=False))
        self.log("head-applied", f"validation rms {val:.2f} px")
        return {"snapshot": self.write_snapshot("head2 applied"), "restart": "restart vision.main to load the new head geometry"}

    # ------------------------------------------------------------------ stationary verification (read-only)
    def verify_run(self, seconds=15.0, interval=0.25):
        with self._lock:
            if self.verify and self.verify.get("running"):
                raise RuntimeError("check already running")
            self.verify = {"running": True, "started": time.time(), "seconds": seconds, "frames": 0}
        threading.Thread(target=self._verify, args=(seconds, interval), daemon=True).start()

    def _verify(self, seconds, interval):
        samples = {1: [], 2: []}
        end, last_t = time.time() + seconds, None
        while time.time() < end:
            msg = ((self.live() or {}).get("vision") or {}).get("message") or {}
            t = msg.get("t")
            if t != last_t:
                last_t = t
                for r in msg.get("robots", []):
                    if r.get("id") in samples and r.get("pose_compensated") and r.get("x") is not None:
                        samples[r["id"]].append((r["x"], r["y"], r.get("heading")))
            with self._lock:
                self.verify["frames"] = min(len(v) for v in samples.values())
            time.sleep(interval)
        result = {"at": now_iso(), "robots": {}, "frames": 0, "p95_position_m": None}
        worst, frames = 0.0, []
        for rid, pts in samples.items():
            if len(pts) < 5:
                continue
            xs, ys = sorted(p[0] for p in pts), sorted(p[1] for p in pts)
            cx, cy = xs[len(xs) // 2], ys[len(ys) // 2]
            dist = sorted(math.hypot(p[0] - cx, p[1] - cy) for p in pts)
            p95 = dist[min(len(dist) - 1, int(0.95 * len(dist)))]
            hs = [p[2] for p in pts if p[2] is not None]
            span = (max(hs) - min(hs)) if hs else None
            result["robots"][str(rid)] = {"frames": len(pts), "p95_position_m": p95, "max_position_m": dist[-1],
                                          "heading_span_deg": span}
            worst, frames = max(worst, p95), frames + [len(pts)]
        if len(result["robots"]) == 2:
            result["p95_position_m"], result["frames"] = worst, min(frames)
        with self._lock:
            self.verify = {"running": False, **result}
        self.record["verification"] = result if result["p95_position_m"] is not None else self.record.get("verification")
        self.log("verify", f"p95 {worst*100:.1f} cm" if result["p95_position_m"] is not None else "insufficient frames")

    # ------------------------------------------------------------------ lens (proxied to the vision GUI)
    def lens(self, command=None):
        port = self.vision_gui()
        if not port:
            raise RuntimeError("vision GUI not running: start python -m vision.main --gui")
        base = f"http://127.0.0.1:{int(port)}"
        if command is None:
            with urllib.request.urlopen(base + "/state", timeout=3) as r:
                return json.loads(r.read()).get("calib")
        if command not in ("start", "stop", "capture", "auto_on", "auto_off", "undo", "reset", "solve"):
            raise ValueError("unknown lens command")
        req = urllib.request.Request(f"{base}/calib/{command}", data=b"{}", method="POST")
        try:
            with urllib.request.urlopen(req, timeout=60) as r:
                result = json.loads(r.read())
        except urllib.error.HTTPError as e:
            result = json.loads(e.read() or b"{}")
        if command == "solve" and (result.get("result") or {}):
            self.log("lens-solved", f"rms {result['result'].get('rms_px')}")
            self.write_snapshot("lens solved")
        return result

    # ------------------------------------------------------------------ everything for the page
    def state(self):
        return {"steps": self.steps(), "data": self.collect(), "job": self.job_status(),
                "recordings": self.recordings(), "verify": self.verify, "record": {
                    "confirmations": self.record["confirmations"], "history": self.record["history"][-15:],
                    "verification": self.record.get("verification")},
                "candidates": {"floor": self.floor_candidate(), "head": _head_summary(self.head_candidate())}}


def _rel(path):
    try:
        return str(Path(path).resolve().relative_to(ROOT))
    except ValueError:
        return str(path)


def _scalars(d):
    if not isinstance(d, dict):
        return None
    return {k: v for k, v in d.items() if isinstance(v, (int, float, str, bool)) or v is None}


def _fmt_validation(lv):
    if not lv:
        return "none recorded"
    bits = []
    for key, label in (("valid_frames", "frames"), ("position_drift_p95_m", "p95 drift m"), ("position_drift_max_m", "max drift m"),
                       ("heading_deviation_p95_deg", "p95 heading °"), ("validation_rms_px", "rms px"),
                       ("validation_position_drift_p95_m", "p95 drift m"), ("validation_heading_deviation_p95_deg", "p95 heading °")):
        if lv.get(key) is not None:
            bits.append(f"{label} {lv[key]:.3g}" if isinstance(lv[key], float) else f"{label} {lv[key]}")
    return ", ".join(bits) or "recorded"


def _plain(obj):
    if isinstance(obj, dict):
        return {str(k): _plain(v) for k, v in obj.items()}
    if isinstance(obj, (list, tuple)):
        return [_plain(v) for v in obj]
    if isinstance(obj, float):
        return float(obj)
    return obj


def _head_summary(c):
    if not c:
        return None
    return {"training_rms_px": c.get("training_rms_px"), "validation": c.get("independent_validation"),
            "validation_stability": c.get("validation_stability"), "side_mount": c.get("side_mount"),
            "head_pose": c.get("candidate_head_pose")}
