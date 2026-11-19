"""LensCalibrator: render checkerboard views through a known camera, check the calibration recovers it."""

import cv2
import numpy as np

from vision.geometry import CameraModel
from vision.lens_calibration import LensCalibrator

W, H = 1920, 1080
K_TRUE = np.array([[1000.0, 0, W / 2], [0, 1000.0, H / 2], [0, 0, 1]])
BOARD = (9, 6)
SQ_PX = 60  # board texture: pixels per square


def board_texture():
    cols, rows = BOARD[0] + 1, BOARD[1] + 1
    img = np.full(((rows + 2) * SQ_PX, (cols + 2) * SQ_PX), 255, np.uint8)  # one-square white margin
    for r in range(rows):
        for c in range(cols):
            if (r + c) % 2 == 0:
                img[(r + 1) * SQ_PX:(r + 2) * SQ_PX, (c + 1) * SQ_PX:(c + 2) * SQ_PX] = 0
    return img


def render(tex, rvec, tvec, square_m=0.025):
    """Image of the board (board plane z = 0, origin at the texture's top-left) seen by K_TRUE."""
    th, tw = tex.shape
    s = square_m / SQ_PX  # meters per texture pixel
    plane = np.array([[0, 0, 0], [tw * s, 0, 0], [tw * s, th * s, 0], [0, th * s, 0]], float)
    img_pts, _ = cv2.projectPoints(plane, rvec, tvec, K_TRUE, np.zeros(5))
    M = cv2.getPerspectiveTransform(np.float32([[0, 0], [tw, 0], [tw, th], [0, th]]), img_pts.reshape(4, 2).astype(np.float32))
    out = cv2.warpPerspective(tex, M, (W, H), borderValue=180)
    return cv2.cvtColor(cv2.GaussianBlur(out, (3, 3), 0), cv2.COLOR_GRAY2BGR)


def test_calibration_recovers_known_camera(tmp_path):
    tex = board_texture()
    out = tmp_path / "intr.yaml"
    cal = LensCalibrator(out, BOARD)
    cal.active = True
    rng = np.random.default_rng(1)
    for i in range(14):
        rvec = np.array([np.radians(rng.uniform(-30, 30)), np.radians(rng.uniform(-30, 30)), np.radians(rng.uniform(-10, 10))])
        tvec = np.array([rng.uniform(-0.25, 0.05), rng.uniform(-0.15, 0.0), rng.uniform(0.45, 0.8)])
        frame = render(tex, rvec, tvec)
        assert cal.capture(frame), f"view {i}: board not found"
    result = cal.calibrate()
    assert result is not None and result["views"] == 14
    assert result["rms_px"] < 0.5
    assert abs(result["fx"] - 1000) < 15 and abs(result["fy"] - 1000) < 15
    model = CameraModel.load(out)
    assert model.size == (W, H)
    assert abs(model.K[0, 2] - W / 2) < 15 and abs(model.K[1, 2] - H / 2) < 15


def test_needs_enough_views_and_auto_capture_skips_repeats(tmp_path):
    tex = board_texture()
    cal = LensCalibrator(tmp_path / "intr.yaml", BOARD, auto_interval_s=0.0)
    cal.active = True
    frame = render(tex, np.array([0.1, -0.2, 0.0]), np.array([-0.1, -0.08, 0.6]))
    assert cal.calibrate() is None  # no views yet
    for _ in range(4):              # the same still pose repeatedly: captured once, then skipped
        assert cal.preview(frame) is not None
    assert len(cal.views) == 1
    cal.undo()
    assert len(cal.views) == 0
