"""Black box: keep a record of every control step so a failed test can be examined afterwards.

control.main writes one session per run under `blackbox.dir` (default logs/blackbox/, not in Git):

    <YYYYmmdd_HHMMSS>/
        meta.json            start time, command line, Git commit/branch/dirty, robots driven
        config.yaml          copy of the config the run used
        events.jsonl         plain-text timeline: mode changes, robots seen/lost, reason changes, vision/user lost,
                             view-guard returns, dashboard Marks (small; read this first)
        ticks-000.jsonl.gz   every control step (20 Hz): vision message, dashboard navigation state, targets,
                             commands and reasons, link replies, dead-reckoning estimates, view-guard state.
                             Rotated every `segment_s`; flushed every second, so a crash loses at most ~1 s.
        snapshots/           annotated main-camera JPEGs (from vision.main --gui) at important events and Marks

Read it with `python -m tools.blackbox` (list / events / window / summary). Old sessions are deleted once the
folder exceeds `max_total_mb`. Writing happens on a background thread so a slow disk never delays the loop.
"""

import datetime
import gzip
import json
import math
import queue
import re
import shutil
import subprocess
import sys
import threading
import time
import urllib.request
from pathlib import Path

DEFAULTS = {"enabled": True, "dir": "logs/blackbox", "max_total_mb": 500, "segment_s": 600, "snapshots": True,
            "snapshot_url": None, "snapshot_min_interval_s": 2.0, "max_snapshots": 200}
NUMBER = re.compile(r"[-+]?\d+(?:\.\d+)?")
SNAPSHOT_EVENTS = {"robot_lost", "view_return", "view_lost", "mark", "vision_lost"}


def _round(v, n=3):
    if isinstance(v, float):
        return round(v, n) if math.isfinite(v) else None
    if isinstance(v, dict):
        return {k: _round(x, n) for k, x in v.items()}
    if isinstance(v, (list, tuple)):
        return [_round(x, n) for x in v]
    return v


def reason_kind(reason):
    """A reason without its changing numbers ("estimate 1.2 s old" -> "estimate # s old"), for change events."""
    return NUMBER.sub("#", reason or "")


def _git(*args):
    try:
        return subprocess.run(["git", *args], capture_output=True, text=True, timeout=3,
                              cwd=Path(__file__).resolve().parent.parent).stdout.strip()
    except (OSError, subprocess.SubprocessError):
        return None


def grab_jpeg(url, timeout=2.0, limit=8_000_000):
    """First JPEG frame from an MJPEG stream (or a plain JPEG URL), or None."""
    try:
        with urllib.request.urlopen(url, timeout=timeout) as response:
            data = b""
            while len(data) < limit:
                chunk = response.read(65536)
                if not chunk:
                    break
                data += chunk
                start = data.find(b"\xff\xd8")
                end = data.find(b"\xff\xd9", start + 2) if start >= 0 else -1
                if end >= 0:
                    return data[start:end + 2]
    except (OSError, ValueError):
        return None
    return None


class BlackBox:
    def __init__(self, settings=None, root=None, argv=None, config_path=None, robots=(), clock=time.time):
        self.s = {**DEFAULTS, **(settings or {})}
        self.clock = clock
        base = Path(self.s["dir"])
        if not base.is_absolute():
            base = Path(root or Path(__file__).resolve().parent.parent) / base
        self.base = base
        start = clock()
        self.dir = base / datetime.datetime.fromtimestamp(start).strftime("%Y%m%d_%H%M%S")
        n = 1
        while self.dir.exists():
            n += 1
            self.dir = base / (datetime.datetime.fromtimestamp(start).strftime("%Y%m%d_%H%M%S") + f"_{n}")
        self.dir.mkdir(parents=True)
        (self.dir / "snapshots").mkdir()
        meta = {"start": start, "start_local": datetime.datetime.fromtimestamp(start).isoformat(timespec="seconds"),
                "argv": list(argv if argv is not None else sys.argv), "robots": list(robots),
                "git": {"commit": _git("rev-parse", "HEAD"), "branch": _git("rev-parse", "--abbrev-ref", "HEAD"),
                        "dirty": bool(_git("status", "--porcelain", "--untracked-files=no"))}}
        (self.dir / "meta.json").write_text(json.dumps(meta, indent=2))
        if config_path and Path(config_path).exists():
            shutil.copyfile(config_path, self.dir / "config.yaml")
        self.ticks = self.events = self.snapshots = 0
        self.prev = {}               # last tick's summary, for change detection
        self.view_key = None
        self.mark_seq = None         # dashboard Mark counter seen at startup (old marks are not replayed)
        self.last_snapshot = -math.inf
        self._q = queue.Queue(maxsize=2000)
        self._segment, self._segment_start, self._gz = 0, None, None
        self._thread = threading.Thread(target=self._writer, name="blackbox", daemon=True)
        self._thread.start()
        self._prune()

    # ---- public API (called from the control loop; never blocks on disk) ----

    def record(self, now, *, mode, navigation_mode, msg, msg_age, nav, targets, results, link_ages, link_status,
               reckoners=None, guard=None):
        robots = {r["id"]: r for r in (msg or {}).get("robots", [])}
        view = (msg or {}).get("view")
        key = json.dumps(view, sort_keys=True) if view else None
        if key != self.view_key:
            self.view_key = key
            self.event(now, "view_changed", polygon=(view or {}).get("polygon"), heights_m=(view or {}).get("heights_m"))
        tick = {"t": now, "mode": mode, "nav_mode": navigation_mode, "msg_age": msg_age,
                "vision": None if msg is None else {k: msg.get(k) for k in (
                    "t", "frame", "frame_clock", "position_quality", "driving_allowed")},
                "user": (msg or {}).get("user"),
                "people": (msg or {}).get("people"),  # every located person; "user": true = the one tracked
                "robots": {str(rid): {k: r.get(k) for k in ("x", "y", "heading", "heading_age", "age", "vx", "vy", "position_source",
                                                           "pose_compensated", "pose_status", "head_units")}
                           for rid, r in robots.items()},
                "nav": None if nav is None else {
                    "armed": nav.get("armed"), "follow": nav.get("follow"), "targets": nav.get("targets"),
                    "manual": {k: (nav.get("manual") or {}).get(k) for k in ("enabled", "robot_id", "keys", "speed")}},
                "targets": {str(rid): None if t is None else [t.x, t.y, t.heading_deg] for rid, t in targets.items()},
                "out": {str(rid): {"cmd": list(cmd), "reason": reason} for rid, (cmd, reason) in results.items()},
                "links": {str(rid): {"age": link_ages.get(rid), "reply": link_status.get(rid)} for rid in results},
                "reckon": {str(rid): None if r.pose is None else {"pose": r.pose, "since": now - r.t, "travelled": r.travelled}
                           for rid, r in (reckoners or {}).items()},
                "guard": {str(k): v for k, v in (getattr(guard, "state", {}) or {}).items()}}
        self._put(("tick", _round(tick)))
        self.ticks += 1
        self._detect(now, tick, msg, nav)

    def event(self, now, kind, robot=None, **detail):
        item = {"t": now, "local": datetime.datetime.fromtimestamp(now).strftime("%H:%M:%S.%f")[:-3], "event": kind}
        if robot is not None:
            item["robot"] = robot
        item.update(detail)
        self._put(("event", _round(item)))
        self.events += 1
        if self.s["snapshots"] and kind in SNAPSHOT_EVENTS:
            self._snapshot(now, kind if robot is None else f"{kind}_r{robot}")

    def status(self):
        return {"dir": str(self.dir), "ticks": self.ticks, "events": self.events, "snapshots": self.snapshots}

    def close(self):
        self._q.put(None)
        self._thread.join(timeout=5)

    # ---- change detection ----

    def _detect(self, now, tick, msg, nav):
        p = self.prev
        if p.get("mode") != (tick["mode"], tick["nav_mode"]):
            self.event(now, "mode", mode=tick["mode"], navigation_mode=tick["nav_mode"],
                       armed=(nav or {}).get("armed"), follow=(nav or {}).get("follow"))
        vision_ok = msg is not None and tick["msg_age"] is not None and tick["msg_age"] <= 1.0
        if "vision_ok" in p and p["vision_ok"] != vision_ok:
            self.event(now, "vision_back" if vision_ok else "vision_lost", msg_age=tick["msg_age"])
        user = tick["user"] is not None and (tick["user"].get("age") or 0) <= 2.0
        if "user" in p and p["user"] != user:
            self.event(now, "user_seen" if user else "user_lost",
                       **({"x": tick["user"]["x"], "y": tick["user"]["y"]} if user else {}))
        seen = {}
        for rid, out in tick["out"].items():
            r = tick["robots"].get(rid)
            seen[rid] = r is not None and r.get("heading") is not None and (r.get("age") or 0) <= 0.7
            if rid in p.get("seen", {}) and p["seen"][rid] != seen[rid]:
                last = p["poses"].get(rid)
                self.event(now, "robot_seen" if seen[rid] else "robot_lost", robot=int(rid),
                           **({"x": r["x"], "y": r["y"], "heading": r["heading"]} if seen[rid] else {"last": last}))
            kind = reason_kind(out["reason"])
            if p.get("reasons", {}).get(rid) != kind:
                self.event(now, "reason", robot=int(rid), reason=out["reason"], cmd=out["cmd"])
        for rid, state in tick["guard"].items():
            if p.get("guard", {}).get(rid) != state:
                self.event(now, "view_return" if state == "returning" else "view_lost", robot=int(rid),
                           estimate=(tick["reckon"].get(rid) or {}).get("pose"))
        mark = (nav or {}).get("mark") or {}
        if self.mark_seq is None:
            self.mark_seq = mark.get("seq", 0)
        elif mark.get("seq", 0) != self.mark_seq:
            self.mark_seq = mark.get("seq", 0)
            self.event(now, "mark", note=mark.get("note", ""))
        self.prev = {"mode": (tick["mode"], tick["nav_mode"]), "vision_ok": vision_ok, "user": user, "seen": seen,
                     "reasons": {rid: reason_kind(o["reason"]) for rid, o in tick["out"].items()},
                     "guard": dict(tick["guard"]),
                     "poses": {rid: [r.get("x"), r.get("y"), r.get("heading")] for rid, r in tick["robots"].items()}
                              | {rid: v for rid, v in p.get("poses", {}).items() if rid not in tick["robots"]}}

    # ---- snapshots ----

    def _snapshot(self, now, label):
        url = self.s.get("snapshot_url")
        if not url or self.snapshots >= self.s["max_snapshots"] or now - self.last_snapshot < self.s["snapshot_min_interval_s"]:
            return
        self.last_snapshot = now
        self.snapshots += 1
        name = datetime.datetime.fromtimestamp(now).strftime("%H%M%S_%f")[:-3] + f"_{label}.jpg"

        def fetch():
            jpeg = grab_jpeg(url)
            if jpeg:
                (self.dir / "snapshots" / name).write_bytes(jpeg)
        threading.Thread(target=fetch, name="blackbox-snapshot", daemon=True).start()

    # ---- writer thread ----

    def _put(self, item):
        try:
            self._q.put_nowait(item)
        except queue.Full:  # disk stalled: drop rather than delay the control loop
            pass

    def _writer(self):
        events = open(self.dir / "events.jsonl", "a", encoding="utf-8")
        last_flush = time.monotonic()
        try:
            while True:
                try:
                    item = self._q.get(timeout=0.5)
                except queue.Empty:
                    item = ()
                if item is None:
                    break
                if item:
                    kind, data = item
                    line = json.dumps(data, separators=(",", ":")) + "\n"
                    if kind == "event":
                        events.write(line)
                    else:
                        self._tick_file(data["t"]).write(line.encode())
                if time.monotonic() - last_flush >= 1.0:
                    last_flush = time.monotonic()
                    events.flush()
                    if self._gz is not None:
                        self._gz.flush()  # gzip sync flush: everything so far is readable after a crash
        finally:
            events.close()
            if self._gz is not None:
                self._gz.close()

    def _tick_file(self, t):
        if self._gz is None or t - self._segment_start >= self.s["segment_s"]:
            if self._gz is not None:
                self._gz.close()
                self._segment += 1
            self._segment_start = t
            self._gz = gzip.open(self.dir / f"ticks-{self._segment:03d}.jsonl.gz", "wb", compresslevel=6)
        return self._gz

    def _prune(self):
        """Delete the oldest sessions (never this one) while the folder is over max_total_mb."""
        try:
            sessions = sorted(p for p in self.base.iterdir() if p.is_dir() and p != self.dir)
            size = lambda d: sum(f.stat().st_size for f in d.rglob("*") if f.is_file())
            total = sum(size(d) for d in sessions)
            limit = self.s["max_total_mb"] * 1e6
            while sessions and total > limit:
                oldest = sessions.pop(0)
                total -= size(oldest)
                shutil.rmtree(oldest, ignore_errors=True)
        except OSError:
            pass
