"""Camera geometry: lens model, camera pose relative to the floor, and pixel <-> floor conversion.

World frame: meters, z up, floor at z = 0. The x/y axes are whatever the floor calibration
points define (recommended: x to the right as seen from the camera, y away from the camera).

The key idea: a pixel defines a ray leaving the camera. If we know the *height* of the thing in
that pixel (feet = 0 m, marker center = 0.30 m, ...), the ray hits that height at exactly one
floor (x, y) point. This is far more accurate at long range than estimating distance from how
big something looks.
"""

import math
from pathlib import Path

import cv2
import numpy as np
import yaml


def _save_yaml(path, data):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w") as f:
        yaml.safe_dump(data, f, sort_keys=False)


class CameraModel:
    """Lens intrinsics: camera matrix K, distortion coefficients, and the image size they belong to."""

    def __init__(self, K, dist, size):
        self.K = np.asarray(K, dtype=float).reshape(3, 3)
        self.dist = np.asarray(dist, dtype=float).ravel()
        self.size = (int(size[0]), int(size[1]))

    @classmethod
    def from_fov(cls, width, height, hfov_deg):
        """Rough pinhole model from the lens field of view (no distortion). Good enough to start with."""
        f = (width / 2) / math.tan(math.radians(hfov_deg) / 2)
        K = [[f, 0, width / 2], [0, f, height / 2], [0, 0, 1]]
        return cls(K, np.zeros(5), (width, height))

    @classmethod
    def load(cls, path):
        with open(path) as f:
            d = yaml.safe_load(f)
        return cls(d["K"], d["dist"], d["image_size"])

    def save(self, path, **extra):
        _save_yaml(path, {"image_size": list(self.size), "K": self.K.tolist(), "dist": self.dist.tolist(), **extra})

    def scaled_to(self, width, height):
        """Same lens at a different output resolution (must be the same aspect ratio / field of view)."""
        if (width, height) == self.size:
            return self
        sx, sy = width / self.size[0], height / self.size[1]
        if abs(sx - sy) > 0.01:
            print(f"WARNING: calibration was {self.size}, frames are {(width, height)} (different aspect "
                  "ratio). Recalibrate at the running resolution for accurate positions.")
        K = self.K.copy()
        K[0] *= sx
        K[1] *= sy
        return CameraModel(K, self.dist, (width, height))

    def pixels_to_rays(self, pts):
        """Undistorted viewing rays (camera frame, z = 1) for an (N, 2) array of pixels."""
        pts = np.asarray(pts, dtype=float).reshape(-1, 1, 2)
        norm = cv2.undistortPoints(pts, self.K, self.dist).reshape(-1, 2)
        return np.column_stack([norm, np.ones(len(norm))])


class FloorPose:
    """Where the camera is and how it is rotated, as an OpenCV world->camera transform (rvec, tvec)."""

    def __init__(self, rvec, tvec):
        self.rvec = np.asarray(rvec, dtype=float).reshape(3, 1)
        self.tvec = np.asarray(tvec, dtype=float).reshape(3, 1)
        self.R = cv2.Rodrigues(self.rvec)[0]

    @property
    def camera_position(self):
        return (-self.R.T @ self.tvec).ravel()

    @classmethod
    def look_at(cls, camera_pos, target):
        """Pose for a camera at camera_pos looking at target (handy for tests and simulation)."""
        pos, target = np.asarray(camera_pos, float), np.asarray(target, float)
        forward = target - pos
        forward /= np.linalg.norm(forward)
        right = np.cross(forward, [0.0, 0.0, 1.0])
        right /= np.linalg.norm(right)
        down = np.cross(forward, right)
        R = np.vstack([right, down, forward])  # rows = camera x (right), y (down), z (forward)
        return cls(cv2.Rodrigues(R)[0], -R @ pos)

    @classmethod
    def load(cls, path):
        with open(path) as f:
            d = yaml.safe_load(f)
        return cls(d["rvec"], d["tvec"])

    def save(self, path, **extra):
        _save_yaml(path, {"rvec": self.rvec.ravel().tolist(), "tvec": self.tvec.ravel().tolist(),
                          "camera_position_m": self.camera_position.round(3).tolist(), **extra})


class Geometry:
    """Converts between image pixels and world/floor coordinates."""

    def __init__(self, camera: CameraModel, pose: FloorPose):
        self.camera, self.pose = camera, pose

    @property
    def camera_position(self):
        return self.pose.camera_position

    def pixels_to_floor(self, pts, heights=0.0):
        """(N, 2) pixels -> (N, 2) world (x, y) where each pixel's ray reaches the given height.

        Rows are NaN if the ray never reaches that height (e.g. a pixel above the horizon).
        """
        rays = self.camera.pixels_to_rays(pts) @ self.pose.R  # camera-frame rays rotated into world frame
        cam = self.camera_position
        heights = np.broadcast_to(np.asarray(heights, dtype=float), (len(rays),))
        with np.errstate(divide="ignore", invalid="ignore"):
            s = (heights - cam[2]) / rays[:, 2]
        out = cam[:2] + s[:, None] * rays[:, :2]
        out[~(s > 0)] = np.nan
        return out

    def pixel_to_floor(self, u, v, height=0.0):
        """Single pixel -> (x, y) tuple, or None if the ray doesn't reach that height."""
        x, y = self.pixels_to_floor([[u, v]], height)[0]
        return None if np.isnan(x) else (float(x), float(y))

    def world_to_pixels(self, xyz):
        """(N, 3) world points -> (N, 2) pixels (for drawing and tests)."""
        xyz = np.asarray(xyz, dtype=float).reshape(-1, 3)
        px, _ = cv2.projectPoints(xyz, self.pose.rvec, self.pose.tvec, self.camera.K, self.camera.dist)
        return px.reshape(-1, 2)


def solve_floor_pose(camera: CameraModel, image_pts, world_pts):
    """Camera pose from >= 4 known floor points and where they appear in the image.

    world_pts: (N, 2) floor coordinates (z = 0) or (N, 3). Returns (FloorPose, rms_error_px).
    """
    img = np.asarray(image_pts, dtype=float).reshape(-1, 2)
    obj = np.asarray(world_pts, dtype=float)
    if obj.shape[1] == 2:
        obj = np.column_stack([obj, np.zeros(len(obj))])
    if len(obj) < 4:
        raise ValueError("Need at least 4 floor points")
    planar = np.allclose(obj[:, 2], 0)
    flags = cv2.SOLVEPNP_IPPE if planar else cv2.SOLVEPNP_ITERATIVE
    ok, rvec, tvec = cv2.solvePnP(obj, img, camera.K, camera.dist, flags=flags)
    if not ok:
        raise RuntimeError("solvePnP failed")
    rvec, tvec = cv2.solvePnPRefineLM(obj, img, camera.K, camera.dist, rvec, tvec)
    pose = FloorPose(rvec, tvec)
    proj = Geometry(camera, pose).world_to_pixels(obj)
    rms = float(np.sqrt(np.mean(np.sum((proj - img) ** 2, axis=1))))
    return pose, rms


def load_geometry(cfg, frame_size):
    """Build Geometry from the calibration files named in the config.

    Falls back to a rough field-of-view lens model if the camera hasn't been calibrated yet.
    Returns None if there is no floor calibration (positions can't be computed without it).
    """
    from .config import resolve_path

    cal = cfg.get("calibration", {})
    w, h = frame_size
    intr_path = resolve_path(cfg, cal.get("intrinsics"))
    if intr_path and intr_path.exists():
        camera = CameraModel.load(intr_path).scaled_to(w, h)
    else:
        print(f"NOTE: no lens calibration at {intr_path}; using a rough {cfg['camera'].get('hfov_deg', 66)} deg FOV model")
        camera = CameraModel.from_fov(w, h, cfg["camera"].get("hfov_deg", 66))

    pose_path = resolve_path(cfg, cal.get("floor_pose"))
    if not (pose_path and pose_path.exists()):
        print(f"NOTE: no floor calibration at {pose_path}; run tools/calibrate_floor.py. "
              "Only pixel detections will be shown.")
        return None
    return Geometry(camera, FloorPose.load(pose_path))
