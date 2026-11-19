"""Overlay identifies the selected listener without drawing stale floor estimates."""

from types import SimpleNamespace

import numpy as np
import pytest

from vision.debug_view import draw_overlay
from vision.pipeline import FrameResult


@pytest.mark.parametrize("age,calibrated,visible", [(.2, True, True), (2, True, False), (.2, False, False)])
def test_listener_overlay_requires_recent_calibrated_track(age, calibrated, visible):
    frame = np.zeros((240, 320, 3), dtype=np.uint8)
    result = FrameResult(1, 1, message={"user": {"x": 1, "y": 2, "age": age}})
    geometry = SimpleNamespace(world_to_pixels=lambda points: np.array([[100., 100.]])) if calibrated else None
    overlay = draw_overlay(frame, result, geometry)
    cyan = np.array([255, 255, 0])
    assert bool(np.any(np.all(overlay == cyan, axis=-1))) is visible
    assert not np.any(frame)  # rendering must not modify the frame used for detection
