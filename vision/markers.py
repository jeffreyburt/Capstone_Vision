"""ArUco / AprilTag marker detection (OpenCV's built-in cv2.aruco module)."""

from dataclasses import dataclass

import cv2
import numpy as np


@dataclass
class MarkerDetection:
    id: int
    corners: np.ndarray  # (4, 2) pixels, in the marker's own order: top-left, top-right, bottom-right, bottom-left

    @property
    def center(self):
        return self.corners.mean(axis=0)

    @property
    def side_px(self):
        """Average edge length in pixels (a direct measure of how 'big' the marker looks)."""
        return float(np.mean(np.linalg.norm(self.corners - np.roll(self.corners, -1, axis=0), axis=1)))


def get_dictionary(name):
    if not hasattr(cv2.aruco, name):
        raise ValueError(f"Unknown marker dictionary {name!r} (examples: DICT_4X4_50, DICT_APRILTAG_36h11)")
    return cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, name))


class MarkerDetector:
    def __init__(self, dictionary="DICT_4X4_50", allowed_ids=None):
        """allowed_ids: only report these IDs (everything else is treated as a false detection)."""
        params = cv2.aruco.DetectorParameters()
        params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX  # sub-pixel corners -> steadier positions
        params.minMarkerPerimeterRate = 0.02  # allow small (far away) markers
        self.detector = cv2.aruco.ArucoDetector(get_dictionary(dictionary), params)
        self.allowed_ids = set(allowed_ids) if allowed_ids is not None else None

    def detect(self, frame):
        gray = frame if frame.ndim == 2 else cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self.detector.detectMarkers(gray)
        if ids is None:
            return []
        out = []
        for c, i in zip(corners, ids.ravel()):
            if self.allowed_ids is None or int(i) in self.allowed_ids:
                out.append(MarkerDetection(int(i), c.reshape(4, 2).astype(float)))
        return out
