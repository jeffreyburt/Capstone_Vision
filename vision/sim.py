"""Synthetic camera images of robots with markers, for tests and laptop development without hardware.

render_scene() draws each robot's camera-facing sides as white panels with the right marker warped
onto them, as the calibrated camera would see them.
"""

import math

import cv2
import numpy as np

from .markers import get_dictionary


def face_corners_world(x, y, heading_deg, face_deg, offset, size, center_height):
    """3D corners (TL, TR, BR, BL as seen from outside the face) of a square of `size` on a robot face."""
    a = math.radians(heading_deg + face_deg)
    n = np.array([math.cos(a), math.sin(a)])  # outward normal
    e = np.array([-n[1], n[0]])                # left -> right when looking at the face from outside
    fc = np.array([x, y]) + n * offset
    half = size / 2
    zt, zb = center_height + half, center_height - half
    l, r = fc - e * half, fc + e * half
    return np.array([[*l, zt], [*r, zt], [*r, zb], [*l, zb]]), fc, n


def render_scene(geometry, robots, specs, frame_size=(1920, 1080), dictionary="DICT_4X4_50",
                 panel_margin=1.6, blur=0.0, noise=0.0, rng=None):
    """robots: list of (robot_id, x, y, heading_deg). specs: RobotSpec list. Returns a BGR image."""
    w, h = frame_size
    img = np.full((h, w, 3), 90, np.uint8)
    # Floor a bit darker than the background "walls" so the image isn't uniform.
    horizon = geometry.world_to_pixels([[0, 50, 0]])[0][1]
    img[int(np.clip(horizon, 0, h)):] = 70

    d = get_dictionary(dictionary)
    spec_by_id = {s.id: s for s in specs}
    cam = geometry.camera_position
    faces = []
    for rid, x, y, hd in robots:
        s = spec_by_id[rid]
        for mid, face_deg in s.faces.items():
            corners, fc, n = face_corners_world(x, y, hd, face_deg, s.center_offset_m, s.marker_size_m,
                                                s.marker_center_height_m)
            if np.dot(n, cam[:2] - fc) <= 0:
                continue  # face points away from the camera
            panel, _, _ = face_corners_world(x, y, hd, face_deg, s.center_offset_m,
                                             s.marker_size_m * panel_margin, s.marker_center_height_m)
            faces.append((np.linalg.norm(cam[:2] - fc), mid, corners, panel))

    for _, mid, corners, panel in sorted(faces, key=lambda f: -f[0]):  # far to near
        cv2.fillConvexPoly(img, geometry.world_to_pixels(panel).astype(np.int32), (235, 235, 235), cv2.LINE_AA)
        dst = geometry.world_to_pixels(corners).astype(np.float32)
        m = 240
        marker = cv2.cvtColor(cv2.aruco.generateImageMarker(d, mid, m), cv2.COLOR_GRAY2BGR)
        src = np.float32([[0, 0], [m, 0], [m, m], [0, m]])
        H = cv2.getPerspectiveTransform(src, dst)
        warped = cv2.warpPerspective(marker, H, (w, h), flags=cv2.INTER_AREA)
        mask = cv2.warpPerspective(np.full((m, m), 255, np.uint8), H, (w, h))
        img[mask > 127] = warped[mask > 127]

    if blur > 0:
        img = cv2.GaussianBlur(img, (0, 0), blur)
    if noise > 0:
        rng = rng or np.random.default_rng(0)
        img = np.clip(img + rng.normal(0, noise, img.shape), 0, 255).astype(np.uint8)
    return img
