"""Robot position + heading from the markers on their sides.

Each robot carries one marker per face. For every detected marker:
  1. The marker's 4 corners are at known heights (center height +/- half the marker size), so each
     corner can be projected onto its height plane -> the face's real position on the floor.
  2. The marker's horizontal edge gives the direction the face is pointing (its outward normal).
  3. Robot center = face center pushed inward by center_offset_m; robot heading = face direction
     minus that face's angle on the robot (front = 0, left = 90, back = 180, right = 270).
If several faces of one robot are visible, their estimates are averaged.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np
import cv2

from .joint_pose import HeadGeometry, center_ray_position, fit_chassis, tag_center_world, tag_pose_world


@dataclass
class RobotSpec:
    id: int
    faces: dict  # marker id -> face angle in degrees CCW from the robot's front
    center_offset_m: float
    marker_size_m: float
    marker_center_height_m: float
    head: HeadGeometry | None = None
    size_distance_scale: float = 1.0  # tag-size (PnP) distances x this (legacy; corner_blur_px is the calibrated model)
    corner_blur_px: float = 0.0       # detected squares look this many px larger (calibrated from the floor tags)


@dataclass
class RobotObservation:
    robot_id: int
    x: float
    y: float
    heading_deg: float | None
    marker_ids: list
    approximate: bool = False  # position from tag-centre rays only (no tag passed the full chassis fit)
    source: str = "fit"        # "fit", "two tags" or "tag centre"


def wrap_deg(a):
    """Wrap an angle to (-180, 180]."""
    return -((-a + 180.0) % 360.0 - 180.0)


def robot_specs_from_config(cfg):
    m = cfg.get("markers", {})
    specs = []
    for r in cfg.get("robots", []):
        spec = RobotSpec(
            id=int(r["id"]),
            faces={int(k): float(v) for k, v in r["faces"].items()},
            center_offset_m=float(r.get("center_offset_m", 0.15)),
            marker_size_m=float(r.get("marker_size_m", m.get("size_m", 0.10))),
            marker_center_height_m=float(r.get("marker_center_height_m", m.get("center_height_m", 0.30))),
            size_distance_scale=float(r.get("size_distance_scale", m.get("size_distance_scale", 1.0))),
            corner_blur_px=float(r.get("corner_blur_px", m.get("corner_blur_px", 0.0))),
        )
        if r.get("head_pose") is not None:
            spec.head = HeadGeometry.from_config(r["head_pose"], spec)
        specs.append(spec)
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


SINGLE_TAG_MAX_PAN_DEG = 35.0  # one-tag headings only while the head is within this of forward


def _single_tag_heading(tags, geometry, size_m=0.10, blur_px=0.0):
    """Chassis heading (radians) from one tag's own pose: the horizontal direction of its edge (corner 0 -> 1) in
    the world, minus the same edge in the chassis frame at the current head angles (so the head's pan is taken
    out). Noisier than a full fit or two tags, but works with a single oblique tag. Largest tag wins."""
    best = None
    for m, corners, *_ in tags:
        world = tag_pose_world(m, size_m, geometry, blur_px)
        if world is None:
            continue
        w = world[1, :2] - world[0, :2]
        c = np.asarray(corners)[1, :2] - np.asarray(corners)[0, :2]
        if np.hypot(*w) < 1e-6 or np.hypot(*c) < 1e-6:
            continue
        heading = math.atan2(w[1], w[0]) - math.atan2(c[1], c[0])
        if best is None or m.side_px > best[0]:
            best = m.side_px, math.atan2(math.sin(heading), math.cos(heading))
    return None if best is None else best[1]


def _two_tag_heading(tags, geometry, size_m=0.10, blur_px=0.0):
    """Chassis heading (radians) from two visible tags' centre rays: the world direction between the two tag
    centres minus the same direction in the chassis frame. None unless two tags are far enough apart."""
    best = None
    for i in range(len(tags)):
        for j in range(i + 1, len(tags)):
            (ma, ca, *_), (mb, cb, *_) = tags[i], tags[j]
            ca, cb = np.asarray(ca).mean(axis=0), np.asarray(cb).mean(axis=0)
            wa, wb = tag_center_world(ma, size_m, geometry, blur_px), tag_center_world(mb, size_m, geometry, blur_px)
            if wa is None or wb is None:
                continue
            pa, pb = wa[:2], wb[:2]
            local = cb[:2] - ca[:2]
            if not (np.isfinite(pa).all() and np.isfinite(pb).all()) or np.hypot(*local) < 0.08:
                continue
            world = pb - pa
            heading = math.atan2(world[1], world[0]) - math.atan2(local[1], local[0])
            spread = np.hypot(*local)
            if best is None or spread > best[0]:
                best = spread, math.atan2(math.sin(heading), math.cos(heading))
    return None if best is None else best[1]


def estimate_robots(markers, specs, geometry, joint_positions=None, prior_headings=None, stale_headings=()):
    """Detected markers -> one RobotObservation per robot that has at least one usable marker.

    Head-mounted tags, in order of preference:
      two or more tags visible -> each tag's centre ray gives a position, the direction between the tags gives
                                  the heading (robust to head wobble and to the single-tag pan/heading trade-off);
      one tag that passes the full image-space chassis fit -> that fit;
      otherwise -> the tag's centre ray, swung by the tracker's last heading (or the expected facing direction).
    """
    joint_positions = joint_positions or {}
    prior_headings = prior_headings or {}
    by_marker = {mid: spec for spec in specs for mid in spec.faces}
    groups, sources = {}, {}
    head_tags = {}  # robot id -> [(marker, corners, spec)]
    for m in markers:
        spec = by_marker.get(m.id)
        if spec is None:
            continue
        if spec.head is not None:
            units = joint_positions.get(spec.id) if spec.head.needs_feedback else spec.head.neutral_units
            corners = None if units is None else spec.head.corners(m.id, units, own_limits=False)
            if corners is not None:  # a tag's own joint range limits only the full fit, not its centre position
                fit_ok = spec.head.corners(m.id, units) is not None
                head_tags.setdefault(spec.id, []).append((m, corners, spec, fit_ok))
            continue
        face = face_on_floor(m, spec, geometry)
        if face is None:
            continue
        center, normal = face
        robot_center = center - normal * spec.center_offset_m
        face_dir = math.degrees(math.atan2(normal[1], normal[0]))
        heading = math.radians(face_dir - spec.faces[m.id])
        groups.setdefault(spec.id, []).append((robot_center, heading, m.side_px, m.id))
        sources[spec.id] = "fit"

    for rid, tags in head_tags.items():
        head = tags[0][2].head
        size = tags[0][2].marker_size_m * tags[0][2].size_distance_scale
        blur = tags[0][2].corner_blur_px
        heading, source = _two_tag_heading(tags, geometry, size, blur), "two tags"
        if heading is not None and head.heading_window is not None and abs(
                (math.degrees(heading) - head.heading_window[0] + 180) % 360 - 180) > head.heading_window[1]:
            heading = None
        if heading is None:
            fits = []
            for m, corners, _, fit_ok in tags:
                if m.id in head.center_only or not fit_ok:
                    continue
                pose = fit_chassis(m, corners, geometry, prior_headings.get(rid), head.max_rms_px, head.heading_window)
                if pose is not None:
                    fits.append((pose[0], pose[1], m.side_px, m.id))
            if fits:
                groups[rid], sources[rid] = fits, "fit"
                continue
            source = "tag centre"
            own = _single_tag_heading(tags, geometry, size, blur)
            units = joint_positions.get(rid) if head.needs_feedback else None
            if own is not None and units is not None and abs(
                    (units[0] - head.neutral_units[0]) * head.degrees_per_unit[0]) > SINGLE_TAG_MAX_PAN_DEG:
                own = None  # head turned far aside: the tag is too oblique and the pan model too extrapolated to trust
            if own is not None and head.heading_window is not None and abs(
                    (math.degrees(own) - head.heading_window[0] + 180) % 360 - 180) > head.heading_window[1]:
                own = None
            if own is not None and (rid not in prior_headings or rid in stale_headings):
                # No fit/two-tag heading lately: the held heading goes stale as the chassis turns, so measure it.
                heading, source = own, "one tag"
            elif rid in prior_headings:
                heading = prior_headings[rid]
            elif own is not None:
                heading, source = own, "one tag"  # no history yet: the tag's own orientation gives the heading
            elif head.heading_window is not None:
                heading = math.radians(head.heading_window[0])
            else:
                continue
        for m, corners, *_ in tags:
            xy = center_ray_position(m, corners, geometry, heading, size, blur)
            if xy is not None:
                groups.setdefault(rid, []).append((xy, heading if source in ("two tags", "one tag") else None, m.side_px, m.id))
                sources[rid] = source

    out = []
    for rid, items in groups.items():
        w = np.array([i[2] for i in items])
        xy = np.average([i[0] for i in items], axis=0, weights=w)
        headed = [i for i in items if i[1] is not None]
        heading = None
        if headed:
            hw = np.array([i[2] for i in headed])
            heading = wrap_deg(math.degrees(math.atan2(np.sum(hw * np.sin([i[1] for i in headed])),
                                                       np.sum(hw * np.cos([i[1] for i in headed])))))
        out.append(RobotObservation(rid, float(xy[0]), float(xy[1]), heading, [i[3] for i in items],
                                    sources[rid] == "tag centre", sources[rid]))
    return out


def estimate_marker_positions(markers, specs, geometry):
    """Display-only robot reference from visible marker centers, including tilted/top markers.

    PnP uses measured black-square size rather than an assumed marker height plane. These are not
    chassis centers: unknown head offsets are not compensated, and no chassis heading is invented.
    """
    by_marker = {mid: spec for spec in specs for mid in spec.faces}
    groups = {}
    for marker in markers:
        spec = by_marker.get(marker.id)
        if spec is None:
            continue
        half = spec.marker_size_m / 2
        obj = np.array([[-half, half, 0], [half, half, 0], [half, -half, 0], [-half, -half, 0]], float)
        ok, rvec, tvec = cv2.solvePnP(obj, marker.corners, geometry.camera.K, geometry.camera.dist,
                                    flags=cv2.SOLVEPNP_IPPE_SQUARE)
        if not ok or not np.isfinite(tvec).all() or tvec[2, 0] <= .1:
            continue
        projected = cv2.projectPoints(obj, rvec, tvec, geometry.camera.K, geometry.camera.dist)[0].reshape(4, 2)
        if np.sqrt(np.mean((projected - marker.corners) ** 2)) > max(3, marker.side_px * .05):
            continue
        world = (geometry.pose.R.T @ (tvec - geometry.pose.tvec)).ravel()
        if not np.isfinite(world).all():
            continue
        groups.setdefault(spec.id, []).append((world[:2], marker.side_px, marker.id))
    return [RobotObservation(rid, *map(float, np.average([i[0] for i in items], axis=0,
                             weights=[i[1] for i in items])), None, [i[2] for i in items])
            for rid, items in groups.items()]
