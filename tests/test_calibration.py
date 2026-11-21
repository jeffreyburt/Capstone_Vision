"""Dashboard calibration checklist: stored values, validation, apply guards, HTTP routes, config overlay."""
import json
import shutil
import urllib.error
import urllib.request
from pathlib import Path

import pytest
import yaml

from monitor.calibration import CalibrationManager
from monitor.server import SystemMonitor, calibration_action, make_calibration, make_server
from vision.config import load_config

REPO = Path(__file__).resolve().parent.parent


@pytest.fixture
def cfg(tmp_path):
    shutil.copy(REPO / "config.yaml", tmp_path / "config.yaml")
    (tmp_path / "calib").mkdir()
    for f in (REPO / "calib").glob("*.yaml"):
        shutil.copy(f, tmp_path / "calib" / f.name)
    return load_config(tmp_path / "config.yaml")


def test_repo_state_is_consistent_and_all_values_collected(cfg):
    cal = CalibrationManager(cfg)
    data = cal.collect()
    assert data["intrinsics"]["image_size"] == [4608, 2592] and data["intrinsics"]["rms_px"] < 1
    assert data["floor"]["height_m"] == pytest.approx(0.363, abs=0.01)  # 2026-11-21 recalibration
    assert {r["id"] for r in data["robots"]} == {1, 2}
    assert data["robots"][1]["head_pose"]["valid_units"] == [[141, 641], [450, 812]]  # widened 2026-11-21
    assert data["wheels"]["robot1_three_wheels.json"]["disabled_channels"] == [1]
    assert [s["id"] for s in cal.steps()][0] == "network"
    states = {s["id"]: s["status"] for s in cal.steps()}
    assert states["lens"] == states["floor"] == "done"


def test_checked_in_record_marks_current_setup_done():
    cal = CalibrationManager(load_config())
    conf = cal.record["confirmations"]
    assert {"measure", "head1", "wheels"} <= set(conf)
    assert (REPO / "calib" / "calibration_record.json").exists()


def test_measurement_validation_and_persistence(cfg):
    cal = CalibrationManager(cfg)
    cal.set_measurements({"lens_height_m": 0.41, "floor_marker_ids": [0, 1, 7], "rear_tag_offset_m": {"x": -0.15, "y": 0.01}})
    assert CalibrationManager(cfg).record["measurements"]["lens_height_m"] == 0.41
    for bad in ({"lens_height_m": -1}, {"lens_height_m": float("nan")}, {"nope": 1}, {"floor_marker_ids": [1, 1]},
                {"floor_marker_ids": [3, 4]}, {"rear_tag_offset_m": {"x": 5, "y": 0}}, {"floor_min_y_fraction": 1.0},
                {"floor_forward_id": "a"}):
        with pytest.raises(ValueError):
            cal.set_measurements(bad)
    assert cal.record["measurements"]["lens_height_m"] == 0.41  # failed saves change nothing


def test_floor_apply_guards_and_backup(cfg):
    cal = CalibrationManager(cfg)
    with pytest.raises(RuntimeError, match="no floor candidate"):
        cal.floor_apply()
    cand = cal.dir / "candidates"
    cand.mkdir()
    (cand / "floor_pose_candidate.yaml").write_text(yaml.safe_dump({"rvec": [0, 0, 0], "tvec": [0, 0, 0], "height_m": 0.9}))
    with pytest.raises(RuntimeError, match="10 cm"):
        cal.floor_apply()
    (cand / "floor_pose_candidate.yaml").write_text(yaml.safe_dump({"rvec": [1, 0, 0], "tvec": [0, 1, 0], "height_m": 0.40}))
    result = cal.floor_apply()
    assert "restart" in result and list((cal.dir / "history").glob("*floor_pose_36cm.yaml"))
    assert yaml.safe_load((cal.dir / "floor_pose_36cm.yaml").read_text())["height_m"] == 0.40
    assert Path(cal.dir / "snapshots").exists()


def test_head_apply_requires_confirmation_and_acceptable_fit(cfg):
    cal = CalibrationManager(cfg)
    cand = {"candidate_head_pose": {"neutral_units": [391, 770]}, "independent_validation": {"rms_px": 1.5},
            "validation_stability": {"position_drift_p95_m": 0.03, "heading_deviation_p95_deg": 2.0}}
    path = cal.dir / "candidates"
    path.mkdir()
    (path / "right_head_fit_candidate.json").write_text(json.dumps(cand))
    with pytest.raises(RuntimeError, match="confirmation"):
        cal.head_apply(False)
    cand["independent_validation"]["rms_px"] = 9
    (path / "right_head_fit_candidate.json").write_text(json.dumps(cand))
    with pytest.raises(RuntimeError, match="acceptance"):
        cal.head_apply(True)
    cand["independent_validation"]["rms_px"] = 1.5
    (path / "right_head_fit_candidate.json").write_text(json.dumps(cand))
    cal.head_apply(True)
    saved = yaml.safe_load((cal.dir / "robot2_head_pose.yaml").read_text())
    assert saved["status"].startswith("active") and saved["head_pose"]["neutral_units"] == [391, 770]
    # the dashboard-written file becomes the runtime geometry
    assert load_config(Path(cfg["_dir"]) / "config.yaml")["robots"][1]["head_pose"] == saved["head_pose"]


def test_head_capture_needs_explicit_confirmation_and_fit_needs_recordings(cfg):
    cal = CalibrationManager(cfg)
    with pytest.raises(RuntimeError, match="confirmation"):
        cal.head_capture("train", False)
    with pytest.raises(ValueError):
        cal.head_capture("bogus", True)
    with pytest.raises(RuntimeError, match="training and one validation"):
        cal.head_fit()
    assert cal.job is None


def test_inactive_head_file_does_not_override_config(cfg):
    f = Path(cfg["_dir"]) / "calib" / "robot2_head_pose.yaml"
    data = yaml.safe_load(f.read_text())
    data["status"], data["head_pose"]["neutral_units"] = "candidate_only", [1, 2]
    f.write_text(yaml.safe_dump(data))
    assert load_config(Path(cfg["_dir"]) / "config.yaml")["robots"][1]["head_pose"]["neutral_units"] == [391, 770]


def test_committed_head_files_match_embedded_config():
    cfg = load_config()
    raw = yaml.safe_load((REPO / "config.yaml").read_text())
    for loaded, embedded in zip(cfg["robots"], raw["robots"]):
        assert loaded["head_pose"] == embedded["head_pose"]  # overlay changes nothing today


def test_http_routes(cfg):
    monitor = SystemMonitor(cfg, ssh=False, listener=False, start=False)
    cal = make_calibration(monitor)
    httpd = make_server(monitor, None, "127.0.0.1", 0, calibration=cal)
    import threading
    threading.Thread(target=httpd.serve_forever, daemon=True).start()
    base = f"http://127.0.0.1:{httpd.server_address[1]}"
    try:
        assert b"Calibration checklist" in urllib.request.urlopen(base + "/calibration/advanced").read()
        assert b"Calibrate floor" in urllib.request.urlopen(base + "/calibration").read()
        state = json.loads(urllib.request.urlopen(base + "/api/calibration").read())
        assert len(state["steps"]) == 8 and state["data"]["intrinsics"]

        def post(path, body, origin=None):
            req = urllib.request.Request(base + path, json.dumps(body).encode(), {"Content-Type": "application/json",
                                         **({"Origin": origin} if origin else {})})
            try:
                return 200, json.loads(urllib.request.urlopen(req).read())
            except urllib.error.HTTPError as e:
                body = e.read()
                return e.code, (json.loads(body) if body.startswith(b"{") else {})

        assert post("/api/calibration/measurements", {"values": {"lens_height_m": 0.4}})[0] == 200
        assert post("/api/calibration/measurements", {"values": {"lens_height_m": -3}})[0] == 400
        assert post("/api/calibration/head_capture", {"role": "train"})[0] == 400   # unconfirmed
        assert post("/api/calibration/head_apply", {"confirmed": "yes"})[0] == 400   # must be literal true
        assert post("/api/calibration/wheels_drive", {})[0] == 404
        assert post("/api/calibration/snapshot", {}, origin="http://evil.example")[0] == 403
        assert post("/api/calibration/lens", {"command": "solve"})[0] == 400          # vision GUI not running
    finally:
        httpd.shutdown()
        httpd.server_close()


def test_calibration_action_rejects_unknown_confirm_step(cfg):
    cal = CalibrationManager(cfg)
    with pytest.raises(ValueError):
        calibration_action(cal, "confirm", {"step": "drive_robots"})


def test_restart_camera_uses_ssh_without_sudo_and_reports_failure(cfg):
    calls = []

    def runner(ssh, user, host, port, script=None, timeout=10.0):
        calls.append((user, host, script))
        return "sender running again after 4s (pid 9)\n", None, 0

    monitor = SystemMonitor(cfg, ssh=True, ssh_runner=runner, listener=False, start=False)
    assert monitor.restart_camera()["ok"] and calls[0][0] == "vision" and "sudo" not in calls[0][2]
    monitor.ssh_runner = lambda *a, **k: ("sender did not come back\n", None, 0)
    with pytest.raises(RuntimeError, match="did not come back"):
        monitor.restart_camera()
    monitor.ssh_runner = lambda *a, **k: ("", "ssh timed out", 0)
    with pytest.raises(RuntimeError, match="timed out"):
        monitor.restart_camera()


def test_restart_vision_only_signals_a_vision_process(cfg, monkeypatch):
    import monitor.server as server
    monitor = SystemMonitor(cfg, ssh=False, listener=False, start=False)
    killed, spawned = [], []
    monitor.heartbeats = lambda: (({"pid": 12345}, 0.1), (None, None))
    monkeypatch.setattr(server.subprocess, "run", lambda argv, **k: type("R", (), {"stdout": "python -m vision.main --gui"
                        if "args=" in argv else "", "returncode": 1})())

    class P:
        pid = 777
    out = monitor.restart_vision(spawn=lambda *a, **k: spawned.append(a) or P(), kill=lambda *a: killed.append(a))
    assert killed and spawned and spawned[0][0][-3:] == ["-m", "vision.main", "--gui"] and out["ok"]
    killed.clear()
    monkeypatch.setattr(server.subprocess, "run", lambda argv, **k: type("R", (), {"stdout": "/usr/bin/other", "returncode": 1})())
    monitor.restart_vision(spawn=lambda *a, **k: P(), kill=lambda *a: killed.append(a))
    assert not killed  # a reused PID belonging to something else is never signalled
