"""Joint feedback and marker geometry for articulated speaker mounts."""

import json
import math
import socket
import time
from collections import deque
from dataclasses import dataclass

import numpy as np


def _vector(values, length):
    vector = np.asarray(values, dtype=float)
    if vector.shape != (length,) or not np.isfinite(vector).all():
        raise ValueError(f"expected {length} finite values, got {values!r}")
    return vector


def _rotate(points, pivot, angle, axis):
    """Rotate points around chassis +Z (pan) or neutral +Y (tilt)."""
    c, s = math.cos(angle), math.sin(angle)
    rotation = ((c, -s, 0), (s, c, 0), (0, 0, 1)) if axis == "pan" else (
        (c, 0, s), (0, 1, 0), (-s, 0, c)
    )
    return (points - pivot) @ np.asarray(rotation).T + pivot


@dataclass
class HeadGeometry:
    neutral_units: np.ndarray
    degrees_per_unit: np.ndarray
    pan_pivot: np.ndarray
    tilt_pivot: np.ndarray
    mounts: dict

    @classmethod
    def from_config(cls, cfg, spec):
        neutral = _vector(cfg["neutral_units"], 2)
        scale = _vector(cfg["degrees_per_unit"], 2)
        if np.any((neutral < 0) | (neutral > 1000)) or np.any(scale == 0):
            raise ValueError("invalid servo neutral_units or degrees_per_unit")
        mounts = {}
        for key, marker in cfg["markers"].items():
            marker_id = int(key)
            mount = marker["mount"]
            if marker_id not in spec.faces or mount not in ("pan", "tilt", "tilt_only", "fixed"):
                raise ValueError(f"invalid articulated marker {marker_id}: {mount!r}")
            center = _vector(marker["center_m"], 3)
            yaw = math.radians(spec.faces[marker_id])
            right = np.array([-math.sin(yaw), math.cos(yaw), 0])
            up = np.array([0, 0, 1])
            half = spec.marker_size_m / 2
            corners = center + half * np.array([up - right, up + right, -up + right, -up - right])
            mounts[marker_id] = mount, corners
        if not mounts:
            raise ValueError("head_pose.markers must list at least one measured marker")
        return cls(neutral, scale, _vector(cfg["pan_pivot_m"], 3), _vector(cfg["tilt_pivot_m"], 3), mounts)

    def corners(self, marker_id, units):
        if marker_id not in self.mounts:
            return None
        mount, neutral_corners = self.mounts[marker_id]
        pan, tilt = np.radians((units - self.neutral_units) * self.degrees_per_unit)
        corners = neutral_corners
        if mount in ("tilt", "tilt_only"):
            corners = _rotate(corners, self.tilt_pivot, tilt, "tilt")
        if mount in ("pan", "tilt"):
            corners = _rotate(corners, self.pan_pivot, pan, "pan")
        return corners


def fit_chassis(marker, corners, geometry):
    """Match observed horizontal coordinates to the joint-transformed chassis model."""
    measured = geometry.pixels_to_floor(marker.corners, corners[:, 2])
    if not np.isfinite(measured).all():
        return None
    model = corners[:, :2]
    model_center, measured_center = model.mean(axis=0), measured.mean(axis=0)
    a, b = model - model_center, measured - measured_center
    dot = np.sum(a * b)
    cross = np.sum(a[:, 0] * b[:, 1] - a[:, 1] * b[:, 0])
    if math.hypot(dot, cross) < 0.25 * np.sum(a * a):
        return None
    heading = math.atan2(cross, dot)
    c, s = math.cos(heading), math.sin(heading)
    rotation = np.array([[c, s], [-s, c]])
    if np.max(np.linalg.norm(a @ rotation - b, axis=1)) > 0.035:
        return None
    return measured_center - model_center @ rotation, heading


class JointFeedback:
    """Keep a short ordered history of valid measured positions from each robot."""

    def __init__(self, port, sources):
        self.sources = sources
        self.history = {rid: deque(maxlen=16) for rid in sources}
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            self.sock.bind(("0.0.0.0", port))
            self.sock.setblocking(False)
        except OSError:
            self.sock.close()
            raise

    def poll(self):
        now = time.time()
        for _ in range(128):
            try:
                packet, addr = self.sock.recvfrom(1024)
            except (BlockingIOError, ConnectionResetError):
                break
            try:
                msg = json.loads(packet)
                rid, stamp, units = msg["robot_id"], msg["t"], msg["units"]
                if type(rid) is not int or self.sources.get(rid) != addr[0]:
                    continue
                if type(stamp) not in (float, int) or not math.isfinite(stamp) or abs(now - stamp) > 0.4:
                    continue
                if not isinstance(units, list) or len(units) != 2:
                    continue
                if any(type(value) is not int or not 0 <= value <= 1000 for value in units):
                    continue
            except (ValueError, KeyError, TypeError):
                continue
            samples = self.history[rid]
            if not samples or stamp > samples[-1][0]:
                samples.append((stamp, np.array(units)))

    def positions_at(self, stamp):
        self.poll()
        if abs(time.time() - stamp) > 0.4:
            return {}
        result = {}
        for rid, samples in self.history.items():
            if samples:
                sample_time, units = min(samples, key=lambda item: abs(item[0] - stamp))
                if abs(sample_time - stamp) <= 0.12:
                    result[rid] = units
        return result

    def close(self):
        self.sock.close()
