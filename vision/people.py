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
    from_feet: bool    # False = feet were out of view, position came from the head


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


def locate_person(det, geometry, frame_height, head_height_m=1.70, edge_px=3):
    """PersonDetection -> PersonObservation on the floor, or None if it can't be placed."""
    feet_visible = det.box[3] < frame_height - edge_px
    if feet_visible:
        xy = geometry.pixel_to_floor(*det.foot_px, 0.0)
        if xy is None:
            return None
        height = geometry.height_above(*det.head_px, *xy)
    else:
        xy = geometry.pixel_to_floor(*det.head_px, head_height_m)
        if xy is None:
            return None
        height = head_height_m
    return PersonObservation(xy[0], xy[1], height, det.confidence, feet_visible)


def select_user(observations, track, t, lost_after_s):
    """Pick the user: the person closest to the current user track, else the most confident one."""
    if not observations:
        return None
    if track.pos is not None and track.age(t) <= lost_after_s:
        return min(observations, key=lambda o: np.hypot(o.x - track.pos[0], o.y - track.pos[1]))
    return max(observations, key=lambda o: o.confidence)
