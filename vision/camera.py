"""Frame sources: Pi Camera (Picamera2), webcams / network streams (OpenCV) and video files.

Every source has the same interface:
    read()  -> (frame_bgr, timestamp) or None when there are no more frames
    close()

Live sources grab frames in a background thread and read() returns the newest frame, so a slow
pipeline never falls behind the camera. Video files are read frame by frame (nothing is skipped)
and get synthetic timestamps based on the file's frame rate, so replays behave like live video.
"""

import threading
import time
from pathlib import Path

import cv2


class _LiveSource:
    """Base class: subclasses implement _grab() (blocking, returns a BGR frame or None)."""

    def _start(self):
        self._cond = threading.Condition()
        self._frame, self._ts = None, 0.0
        self._seq, self._read_seq = 0, 0
        self._running, self._ended = True, False
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def _loop(self):
        while self._running:
            frame = self._grab()
            with self._cond:
                if frame is None:
                    self._ended = True
                else:
                    self._frame, self._ts = frame, time.time()
                    self._seq += 1
                self._cond.notify_all()
            if frame is None:
                break

    def read(self, timeout=3.0):
        with self._cond:
            self._cond.wait_for(lambda: self._seq != self._read_seq or self._ended, timeout)
            if self._seq == self._read_seq:
                return None  # stream ended or timed out
            self._read_seq = self._seq
            return self._frame, self._ts

    def close(self):
        self._running = False
        self._thread.join(timeout=2.0)
        self._release()


class PiCameraSource(_LiveSource):
    def __init__(self, width, height, fps=30, sensor_mode=None, focus_dioptres=None, controls=None):
        from picamera2 import Picamera2  # only available on the Pi (apt install python3-picamera2)

        self.cam = Picamera2()
        kwargs = {
            # Picamera2's "RGB888" is stored in BGR order, which is what OpenCV expects.
            "main": {"size": (int(width), int(height)), "format": "RGB888"},
            "controls": {"FrameRate": float(fps)},
        }
        if sensor_mode:
            kwargs["raw"] = {"size": tuple(sensor_mode)}
        self.cam.configure(self.cam.create_video_configuration(**kwargs))

        ctrl = dict(controls or {})
        if "AfMode" in self.cam.camera_controls:  # Camera Module 3 has autofocus, older ones don't
            if focus_dioptres is None:
                ctrl.setdefault("AfMode", 2)  # continuous
            else:
                ctrl.setdefault("AfMode", 0)  # manual
                ctrl.setdefault("LensPosition", float(focus_dioptres))
        self.cam.start()
        if ctrl:
            self.cam.set_controls(ctrl)
        self._start()

    def _grab(self):
        return self.cam.capture_array("main")

    def _release(self):
        self.cam.stop()
        self.cam.close()


class OpenCVSource(_LiveSource):
    """Webcam (integer index) or network stream URL."""

    def __init__(self, device, width=None, height=None, fps=None):
        self.cap = cv2.VideoCapture(device)
        if not self.cap.isOpened():
            raise RuntimeError(f"Could not open camera/stream {device!r}")
        if isinstance(device, int):
            if width and height:
                self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
                self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
            if fps:
                self.cap.set(cv2.CAP_PROP_FPS, fps)
        self._start()

    def _grab(self):
        ok, frame = self.cap.read()
        return frame if ok else None

    def _release(self):
        self.cap.release()


class VideoFileSource:
    def __init__(self, path, loop=False):
        self.path, self.loop = str(path), loop
        self.cap = cv2.VideoCapture(self.path)
        if not self.cap.isOpened():
            raise RuntimeError(f"Could not open video {path}")
        self.fps = self.cap.get(cv2.CAP_PROP_FPS) or 30.0
        self.t0 = time.time()
        self.index = 0

    def read(self):
        ok, frame = self.cap.read()
        if not ok and self.loop:
            self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
            ok, frame = self.cap.read()
        if not ok:
            return None
        ts = self.t0 + self.index / self.fps
        self.index += 1
        return frame, ts

    def close(self):
        self.cap.release()


def open_source(cfg, source=None, loop=False):
    """Open the frame source named by `source` (or config camera.source).

    "picam" -> Pi Camera, "webcam" or a number -> webcam, an existing file -> video file,
    anything else (e.g. tcp://pi:8888) -> OpenCV stream URL.
    """
    cam = cfg.get("camera", {})
    source = str(source if source is not None else cam.get("source", "picam"))
    w, h, fps = cam.get("width", 1280), cam.get("height", 720), cam.get("fps", 30)

    if source == "picam":
        return PiCameraSource(w, h, fps, cam.get("sensor_mode"), cam.get("focus_dioptres"), cam.get("controls"))
    if source == "webcam":
        return OpenCVSource(int(cam.get("webcam_index", 0)), w, h, fps)
    if source.isdigit():
        return OpenCVSource(int(source), w, h, fps)
    if Path(source).is_file():
        return VideoFileSource(source, loop=loop)
    in_cfg_dir = Path(cfg.get("_dir", ".")) / source  # file paths in a config are relative to that config
    if in_cfg_dir.is_file():
        return VideoFileSource(in_cfg_dir, loop=loop)
    return OpenCVSource(source)
