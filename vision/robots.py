"""Robot position + heading from the markers on their sides.

Each robot carries one marker per face. For every detected marker:
  1. The marker's 4 corners are at known heights (center height +/- half the marker size), so each
     corner can be projected onto its height plane -> the face's real position on the floor.
  2. The marker's horizontal edge gives the direction the face is pointing (its outward normal).
  3. Robot center = face center pushed inward by center_offset_m; robot heading = face direction
     minus that face's angle on the robot (front = 0, left = 90, back = 180, right = 270).
If several faces of one robot are visible, their estimates are averaged.
"""

import math
from dataclasses import dataclass

import numpy as np


@dataclass
class RobotSpec:
    id: int
    faces: dict  # marker id -> face angle in degrees CCW from the robot's front
    center_offset_m: float
    marker_size_m: float
    marker_center_height_m: float


@dataclass
class RobotObservation:
    robot_id: int
    x: float
    y: float
    heading_deg: float
    marker_ids: list


def wrap_deg(a):
    """Wrap an angle to (-180, 180]."""
    return -((-a + 180.0) % 360.0 - 180.0)


def robot_specs_from_config(cfg):
    m = cfg.get("markers", {})
    specs = []
    for r in cfg.get("robots", []):
        specs.append(RobotSpec(
            id=int(r["id"]),
            faces={int(k): float(v) for k, v in r["faces"].items()},
            center_offset_m=float(r.get("center_offset_m", 0.15)),
            marker_size_m=float(r.get("marker_size_m", m.get("size_m", 0.15))),
            marker_center_height_m=float(r.get("marker_center_height_m", m.get("center_height_m", 0.30))),
        ))
    return specs


def face_on_floor(marker, spec, geometry):
    """Floor position (x, y) of a marker's center and the direction (deg) its face points, or None."""
    half = spec.marker_size_m / 2
    h = spec.marker_center_height_m
    # corners: top-left, top-right, bottom-right, bottom-left
    pts = geometry.pixels_to_floor(marker.corners, [h + half, h + half, h - half, h - half])
    if np.isnan(pts).any():
        return None
    top, bottom = pts[1] - pts[0], pts[2] - pts[3]  # left -> right edges
    # Sanity check: both edges should come out roughly the real marker size. Wildly wrong sizes mean a
    # false detection or a bad calibration.
    for e in (top, bottom):
        if not (0.5 * spec.marker_size_m < np.linalg.norm(e) < 2.0 * spec.marker_size_m):
            return None
    edge = top / np.linalg.norm(top) + bottom / np.linalg.norm(bottom)
    # Outward normal = edge direction rotated -90 deg (points back toward the camera, away from the robot).
    normal = np.array([edge[1], -edge[0]])
    normal /= np.linalg.norm(normal)
    center = pts.mean(axis=0)
    return center, normal


def estimate_robots(markers, specs, geometry):
    """Detected markers -> one RobotObservation per robot that has at least one usable marker."""
    by_marker = {mid: spec for spec in specs for mid in spec.faces}
    groups = {}
    for m in markers:
        spec = by_marker.get(m.id)
        if spec is None:
            continue
        face = face_on_floor(m, spec, geometry)
        if face is None:
            continue
        center, normal = face
        robot_center = center - normal * spec.center_offset_m
        face_dir = math.degrees(math.atan2(normal[1], normal[0]))
        heading = math.radians(face_dir - spec.faces[m.id])
        weight = m.side_px  # bigger (closer / more head-on) markers are more trustworthy
        groups.setdefault(spec.id, []).append((robot_center, heading, weight, m.id))

    out = []
    for rid, items in groups.items():
        w = np.array([i[2] for i in items])
        xy = np.average([i[0] for i in items], axis=0, weights=w)
        hx = np.sum(w * np.cos([i[1] for i in items]))
        hy = np.sum(w * np.sin([i[1] for i in items]))
        out.append(RobotObservation(rid, float(xy[0]), float(xy[1]),
                                    wrap_deg(math.degrees(math.atan2(hy, hx))), [i[3] for i in items]))
    return out
