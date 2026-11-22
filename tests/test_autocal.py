"""One-button dashboard calibration (no hardware): synthetic scenes with known answers."""
import json
import math
import threading
import urllib.error
import urllib.request

import numpy as np
import pytest

from monitor.autocal import (chassis_reference, fit_floor, fit_head, predict_head_corners, replace_head_pose_block,
                             set_marker_setting, tag_entry)
from vision.geometry import CameraModel, FloorPose, Geometry
from vision.joint_pose import HeadGeometry, tag_pose_world
from vision.markers import MarkerDetection
from vision.robots import RobotSpec


def flat_tag(center, size):
    h = size / 2
    x, y = center
    return np.array([[x - h, y + h, 0], [x + h, y + h, 0], [x + h, y - h, 0], [x - h, y - h, 0]])


def blurred(pixels, blur_px):
    """Detected squares look blur_px larger than they are."""
    c = pixels.mean(axis=0)
    side = np.mean(np.linalg.norm(pixels - np.roll(pixels, -1, axis=0), axis=1))
    return c + (pixels - c) * (1 + blur_px / side)


def test_floor_fit_matches_the_tape_and_measures_corner_blur():
    camera = CameraModel.from_fov(2304, 1296, 66)
    truth = Geometry(camera, FloorPose.look_at([0, 0, 0.37], [0, 3, 0]))
    layout = {1: (-0.7, 2.0), 0: (0.7, 2.0)}
    corners = {i: blurred(truth.world_to_pixels(flat_tag(xy, 0.15)), 3.0) for i, xy in layout.items()}
    pose, report = fit_floor(camera, corners, 0.15, layout, 0.37)
    assert report["height_m"] == pytest.approx(0.37, abs=0.004)
    assert all(t["error_m"] < 0.02 for t in report["tags"].values())
    assert report["corner_blur_px"] == pytest.approx(3.0, abs=1.0)
    fitted = Geometry(camera, pose)
    assert fitted.pixel_to_floor(*truth.world_to_pixels(np.array([[0.3, 1.2, 0]]))[0]) == pytest.approx((0.3, 1.2), abs=0.02)
    with pytest.raises(ValueError):  # IDs swapped
        fit_floor(camera, corners, 0.15, {1: (0.7, 2.0), 0: (-0.7, 2.0)}, 0.37)
    with pytest.raises(ValueError):  # tape far off
        fit_floor(camera, corners, 0.15, {1: (-0.7, 3.0), 0: (0.7, 3.0)}, 0.37)


def test_robot_tags_measured_facing_forward_reproduce_the_chassis():
    camera = CameraModel.from_fov(2304, 1296, 66)
    geometry = Geometry(camera, FloorPose.look_at([0, 0, 0.37], [0, 3, 0]))
    spec = RobotSpec(1, {2: 180, 3: 270}, 0.15, 0.10, 0.30)
    true_hp = {"neutral_units": [0, 0], "degrees_per_unit": [1, 1], "pan_pivot_m": [0, 0, 0], "tilt_pivot_m": [0, 0, 0],
               "markers": {2: {"mount": "fixed", "center_m": [-0.14, 0, 0.29], "rotation_vector_rad": [0.05, 0, 0]},
                           3: {"mount": "fixed", "center_m": [-0.03, -0.11, 0.24]}}}
    head = HeadGeometry.from_config(true_hp, spec)
    rot = np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]])  # heading 90 deg
    worlds = {}
    for tid in (2, 3):
        world = head.corners(tid, None) @ rot.T + [0.3, 1.5, 0]
        det = MarkerDetection(tid, blurred(geometry.world_to_pixels(world), 2.0))
        worlds[tid] = [tag_pose_world(det, 0.10, geometry, blur_px=2.0)]
    (cx, cy), local, _ = chassis_reference(worlds, 2, 0.14)
    assert (cx, cy) == pytest.approx((0.3, 1.5), abs=0.03)
    for tid in (2, 3):
        entry = tag_entry(local[tid], spec.faces[tid], 0.10, "fixed")
        assert entry["center_m"] == pytest.approx(true_hp["markers"][tid]["center_m"], abs=0.03)


def test_head_fit_recovers_pivots_and_scales():
    rng = np.random.default_rng(0)
    neutral = np.array([391, 770])
    true = [-0.01, -0.07, -0.05, 0.32, 0.27, 0.226]
    spec = RobotSpec(2, {5: 90, 6: 180}, 0.15, 0.10, 0.30)
    neutral_corners = {"6": tag_entry_corners(spec, 6, [-0.14, 0, 0.29]), "5": tag_entry_corners(spec, 5, [-0.05, 0.16, 0.23])}
    mounts = {"6": "tilt", "5": "pan"}
    samples = []
    for units in ([300, 770], [480, 770], [391, 650], [391, 560], [300, 650], [480, 700], [391, 790], [560, 770]):
        tags = {tid: predict_head_corners(c, mounts[tid], units, neutral, true) + rng.normal(0, 0.002, (4, 3))
                for tid, c in neutral_corners.items()}
        samples.append((units, tags))
    params, rms = fit_head(samples, neutral_corners, mounts, neutral, [0, 0, 0, 0.3, 0.24, 0.24])
    assert rms < 0.005
    assert params[4] == pytest.approx(0.27, abs=0.01) and params[5] == pytest.approx(0.226, abs=0.01)
    assert params[3] == pytest.approx(0.32, abs=0.03)


def tag_entry_corners(spec, tid, center):
    hp = {"neutral_units": [0, 0], "degrees_per_unit": [1, 1], "pan_pivot_m": [0, 0, 0], "tilt_pivot_m": [0, 0, 0],
          "markers": {tid: {"mount": "fixed", "center_m": center}}}
    return HeadGeometry.from_config(hp, spec).corners(tid, None)


def test_config_text_edits_keep_everything_else():
    text = ("markers:\n  size_m: 0.10\n\nrobots:\n  - id: 1\n    head_pose:\n      neutral_units:\n      - 0\n"
            "      - 0\n    moving_head_markers: true  # keep me\n    faces: {2: 180}\n  - id: 2\n    head_pose:\n"
            "      neutral_units: [391, 770]\n    faces: {6: 180}\n")
    out = replace_head_pose_block(text, 1, {"neutral_units": [0, 0], "markers": {"2": {"mount": "fixed"}}})
    assert "moving_head_markers: true  # keep me" in out and "neutral_units: [391, 770]" in out
    assert "      markers:\n" in out and out.count("head_pose:") == 2
    out = set_marker_setting(out, "corner_blur_px", 2.5, "test")
    assert out.startswith("markers:\n  corner_blur_px: 2.5  # test\n  size_m: 0.10")
    assert set_marker_setting(out, "corner_blur_px", 3.0, "x").count("corner_blur_px") == 1


def test_dashboard_endpoints_validate_inputs():
    from monitor.server import CameraRelay, SystemMonitor, make_server
    cfg = {"control": {"robots": {1: {"host": "127.0.0.1"}}}}
    monitor = SystemMonitor(cfg, listener=False, start=False)
    server = make_server(monitor, CameraRelay(cfg), "127.0.0.1", 0)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    base = f"http://127.0.0.1:{server.server_port}"
    try:
        with urllib.request.urlopen(base + "/calibration") as r:
            assert b"Calibrate floor" in r.read()
        with urllib.request.urlopen(base + "/calibration/advanced") as r:
            assert r.status == 200
        state = json.load(urllib.request.urlopen(base + "/api/autocal"))
        assert state["running"] is False
        request = urllib.request.Request(base + "/api/autocal/floor", data=json.dumps({"distance_m": 2}).encode(),
                                         headers={"Content-Type": "application/json"})
        with pytest.raises(urllib.error.HTTPError) as error:
            urllib.request.urlopen(request)
        assert error.value.code == 400 and "offset_m" in json.load(error.value)["error"]
        request = urllib.request.Request(base + "/api/autocal/robots", data=b"{}", headers={"Content-Type": "application/json"})
        with pytest.raises(urllib.error.HTTPError) as error:
            urllib.request.urlopen(request)
        assert "confirm" in json.load(error.value)["error"]
    finally:
        server.shutdown()
        server.server_close()
        thread.join()


def test_wall_limits_from_usable_width():
    from monitor.autocal import set_formation_setting, wall_limits
    limits, walls = wall_limits(1.8)
    assert walls == [-0.9, 0.9] and limits == [-0.55, 0.55]  # 0.15 half robot + 0.20 clearance
    assert wall_limits(1.8, clearance_m=0.1)[0] == [-0.65, 0.65]
    with pytest.raises(ValueError):
        wall_limits(0.8)  # 0.4 - 0.35 leaves only 5 cm
    text = "control:\n  formation:\n    distance_m: 1.0\n    x_limits_m: [-0.52, 0.55]  # old\n  limits:\n    a: 1\n"
    out = set_formation_setting(text, "x_limits_m", [-0.55, 0.55], "new")
    out = set_formation_setting(out, "usable_width_m", 1.8, "width")
    assert "    x_limits_m: [-0.55, 0.55]  # new" in out and "# old" not in out
    assert out.index("usable_width_m") < out.index("distance_m") and "  limits:\n    a: 1" in out


def test_walls_endpoint_validates_and_map_reports_walls():
    from monitor.server import CameraRelay, SystemMonitor, make_server
    cfg = {"control": {"robots": {1: {"host": "127.0.0.1"}}, "formation": {"usable_width_m": 2.0, "x_limits_m": [-0.65, 0.65]}}}
    monitor = SystemMonitor(cfg, listener=False, start=False)
    assert monitor.snapshot()["map"]["walls"] == [-1.0, 1.0] and monitor.snapshot()["map"]["x_limits"] == [-0.65, 0.65]
    server = make_server(monitor, CameraRelay(cfg), "127.0.0.1", 0)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    base = f"http://127.0.0.1:{server.server_port}"
    try:
        state = json.load(urllib.request.urlopen(base + "/api/autocal"))
        assert state["current"]["walls"]["walls_m"] == [-1.0, 1.0]
        request = urllib.request.Request(base + "/api/autocal/walls", data=json.dumps({"width_m": 99}).encode(),
                                         headers={"Content-Type": "application/json"})
        with pytest.raises(urllib.error.HTTPError) as error:
            urllib.request.urlopen(request)
        assert "usable width" in json.load(error.value)["error"]
    finally:
        server.shutdown()
        server.server_close()
        thread.join()


def test_head_sweep_tilts_up_in_each_robots_own_direction(monkeypatch):
    """Robot 1 looks up with +units, Robot 2 with -units: the sweep must reach well up on both."""
    from monitor import autocal
    planned = {}

    class Stop(Exception):
        pass

    def fake_urlopen(req, timeout=0):
        body = json.loads(req.data)
        planned.setdefault("targets", []).append(body["a"])
        if len(planned["targets"]) >= 12:
            raise Stop
        class R:
            def read(self): return b""
        return R()

    monkeypatch.setattr(urllib.request, "urlopen", fake_urlopen)
    for rid, signs, limits, neutral, up in ((1, [-1, -1], [[327, 980], [692, 980]], [686, 705], 1),
                                            (2, [-1, 1], [[141, 641], [450, 812]], [388, 743], -1)):
        planned.clear()
        cfg = {"system": {"robots": {rid: {"head_tracking_signs": signs, "head_tracking_limits": limits}}}}
        heads = type("H", (), {"_url": lambda self, n, p: "http://x" + p,
                               "state": lambda self, n: {"running": False, "positions": [0, 0]}})()
        runner = autocal.AutoCalibration(type("M", (), {"cfg": cfg, "nodes": []})(), heads, None)
        monkeypatch.setattr(runner, "_frames", lambda *a, **k: [])
        monkeypatch.setattr(runner, "_tag_worlds", lambda *a, **k: {rid: {}})
        spec = type("S", (), {"id": rid})()
        with pytest.raises((Stop, RuntimeError)):
            runner._sweep(cfg, spec, None, neutral, {}, None, None, 0, {})
        tilts = [t[1] for t in planned["targets"]]
        assert max((t - neutral[1]) * up for t in tilts) >= 200  # reached far up
        assert all(limits[1][0] <= t <= limits[1][1] for t in tilts)
