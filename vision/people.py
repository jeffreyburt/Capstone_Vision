"""Person detection (YOLO via Ultralytics, on the laptop) and locating the person on the floor.

The person's floor position is where their FEET are: the bottom-center of the bounding box, projected
onto the floor (height 0). If the feet are cut off by the bottom of the image, the top of the box (head)
is projected at an assumed head height instead. Their height is estimated from the top of the box.
"""

import threading
from dataclasses import dataclass

import numpy as np


@dataclass
class PersonDetection:
    box: tuple  # (x1, y1, x2, y2) pixels
    confidence: float

    @property
    def foot_px(self):
        x1, _, x2, y2 = self.box
        return ((x1 + x2) / 2, y2)

    @property
    def head_px(self):
        x1, y1, x2, _ = self.box
        return ((x1 + x2) / 2, y1)


@dataclass
class PersonObservation:
    x: float
    y: float
    height: float      # estimated top-of-head height (m)
    confidence: float
    from_feet: bool    # False = feet out of view or hidden behind a robot: position came from the head


class PersonDetector:
    def __init__(self, model="yolo11n.pt", imgsz=640, conf=0.4, device=None):
        from ultralytics import YOLO  # laptop only: pip install -r requirements-laptop.txt

        self.model = YOLO(model)  # .pt models are downloaded automatically on first use
        self.imgsz, self.conf, self.device = imgsz, conf, device

    def detect(self, frame):
        r = self.model.predict(frame, imgsz=self.imgsz, conf=self.conf, classes=[0], device=self.device,
                               verbose=False)[0]
        boxes = r.boxes.xyxy.cpu().numpy()
        confs = r.boxes.conf.cpu().numpy()
        return [PersonDetection(tuple(float(v) for v in b), float(c)) for b, c in zip(boxes, confs)]


class AsyncDetector:
    """Runs a slow detect(frame) in a background thread on the newest submitted frame.

    submit() hands over a frame if the worker is idle (otherwise the frame is skipped); poll() returns
    (timestamp, detections) once per finished run, else None.
    """

    def __init__(self, detect_fn):
        self.detect_fn = detect_fn
        self._cond = threading.Condition()
        self._job, self._result = None, None
        self._busy, self._running = False, True
        self.error = None
        threading.Thread(target=self._loop, daemon=True).start()

    def submit(self, frame, t):
        with self._cond:
            if self._busy or self._job is not None:
                return False
            self._job = (frame, t)
            self._cond.notify_all()
            return True

    def poll(self):
        with self._cond:
            r, self._result = self._result, None
            return r

    def _loop(self):
        while self._running:
            with self._cond:
                self._cond.wait_for(lambda: self._job is not None or not self._running)
                if not self._running:
                    return
                (frame, t), self._job = self._job, None
                self._busy = True
            try:
                dets = self.detect_fn(frame)
            except Exception as e:  # noqa: BLE001 - report, don't kill the pipeline
                self.error, dets = e, []
            with self._cond:
                self._result, self._busy = (t, dets), False

    def close(self):
        with self._cond:
            self._running = False
            self._cond.notify_all()


# Robot volumes that can hide a person's feet, in metres around the robot centre: (half width, z bottom, z top).
# Chassis with wheels, then the narrower mast and head (the head reaches above this ~0.33 m camera).
ROBOT_PARTS = ((0.20, 0.0, 0.18), (0.12, 0.18, 0.55))


def robot_image_boxes(geometry, robots, parts=ROBOT_PARTS):
    """Image outlines (convex hulls, Nx2 pixel arrays) of each robot's chassis and mast/head. A person's foot point
    inside one is probably hidden behind that robot: the box bottom is then the robot's edge, not the floor."""
    import cv2

    hulls = []
    for r in robots or ():
        for half, z0, z1 in parts:
            pts = [[r["x"] + dx, r["y"] + dy, z] for dx in (-half, half) for dy in (-half, half) for z in (z0, z1)]
            try:
                px = np.asarray(geometry.world_to_pixels(pts), dtype=np.float32)
            except Exception:  # noqa: BLE001 - a robot the camera model can't project is simply not an occluder
                continue
            if px.shape == (8, 2) and np.isfinite(px).all():
                hulls.append(cv2.convexHull(px).reshape(-1, 2))
    return hulls


def _inside(hull, u, v, margin_px):
    import cv2

    return cv2.pointPolygonTest(np.asarray(hull, dtype=np.float32).reshape(-1, 1, 2), (float(u), float(v)), True) >= -margin_px


def locate_person(det, geometry, frame_height, head_height_m=1.70, edge_px=3, occluders=(), margin_px=8):
    """PersonDetection -> PersonObservation on the floor, or None if it can't be placed.

    Feet in view (and not behind a robot in `occluders`, see robot_image_boxes) -> the box bottom on the floor.
    Otherwise the box top is placed at head_height_m (the user's learned height when known); with neither end
    usable, the feet are still used rather than losing the person. Height is measured
    only when both ends are in view: this camera sits ~0.33 m up, so a near person's head is often cut off."""
    x1, y1, x2, y2 = det.box
    # A head cut off by the top edge often still leaves the box top a few pixels down: only trust a top clearly
    # inside the frame (3% of its height), or the "head" is really the image edge and reads ~0.3 m off.
    head_visible = y1 > max(edge_px, 0.03 * frame_height)
    feet_visible = y2 < frame_height - edge_px
    u, v = det.foot_px
    hidden = feet_visible and any(_inside(h, u, v, margin_px) for h in occluders)
    if feet_visible and (not hidden or not head_visible):  # hidden but no head either: the feet are all we have
        xy = geometry.pixel_to_floor(u, v, 0.0)
        if xy is None:
            return None
        height = geometry.height_above(*det.head_px, *xy) if head_visible else None
        return PersonObservation(xy[0], xy[1], height, det.confidence, True)
    if not head_visible:
        return None
    xy = geometry.pixel_to_floor(*det.head_px, head_height_m)
    if xy is None:
        return None
    return PersonObservation(xy[0], xy[1], None, det.confidence, False)


def in_area(o, area):
    return area is None or (area["x"][0] <= o.x <= area["x"][1] and area["y"][0] <= o.y <= area["y"][1])
