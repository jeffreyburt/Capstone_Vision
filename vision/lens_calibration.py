"""Live lens (intrinsic) calibration with a printed checkerboard, driven from the GUI.

Same board and output as tools/calibrate_camera.py (9x6 inner corners by default). The square size
doesn't matter for the lens model, only that the board is flat. Workflow:
    start -> hold the board in view (auto-capture or the Capture button) -> ~20 views -> calibrate

Detection runs on a downscaled copy for the live preview (fast), and each captured view is refined to
sub-pixel accuracy on the full-resolution frame.
"""

import threading
import time

import cv2
import numpy as np

from .geometry import CameraModel

PREVIEW_WIDTH = 1280
MIN_VIEWS, GOOD_VIEWS = 8, 20


class LensCalibrator:
    def __init__(self, out_path, board=(9, 6), auto_interval_s=1.5):
        self.out_path, self.board, self.auto_interval_s = out_path, tuple(board), auto_interval_s
        self._lock = threading.Lock()
        self.active, self.auto = False, True
        self.views = []          # full-resolution corner arrays (N, 1, 2)
        self.size = None         # (w, h) of the frames the views came from
        self.message = ""
        self.result = None       # dict after a successful calibration
        self._preview = None     # (corners in full-res pixels, time) of the last preview detection
        self._prev_corners = None
        self._last_capture_t = 0.0

    # ---- live preview (called by the main loop for every displayed frame) ----
    def _detect(self, frame):
        """Find the board on a small copy of the frame; returns corners in full-res pixels, or None."""
        h, w = frame.shape[:2]
        s = min(1.0, PREVIEW_WIDTH / w)
        small = cv2.cvtColor(cv2.resize(frame, (int(w * s), int(h * s)), interpolation=cv2.INTER_AREA),
                             cv2.COLOR_BGR2GRAY)
        # The "SB" detector is ~3x faster than findChessboardCorners on cluttered frames with no board.
        ok, c = cv2.findChessboardCornersSB(small, self.board, cv2.CALIB_CB_NORMALIZE_IMAGE)
        return c / s if ok else None

    def preview(self, frame):
        """Live detection for display; also does auto-capture when the board is held still somewhere new."""
        w = frame.shape[1]
        corners = self._detect(frame)
        with self._lock:
            still = (corners is not None and self._prev_corners is not None
                     and np.abs(corners - self._prev_corners).mean() < 0.004 * w)
            self._prev_corners = corners
            self._preview = (corners, time.time())
            want_auto = (self.active and self.auto and still and self._is_new_pose(corners, w)
                         and time.time() - self._last_capture_t >= self.auto_interval_s)
        if want_auto:
            self.capture(frame, corners)
        return corners

    def _is_new_pose(self, corners, w):
        """True if the board is noticeably somewhere else (or at a different angle) than every capture."""
        return all(np.abs(corners.reshape(-1, 2) - v.reshape(-1, 2)).mean() > 0.04 * w for v in self.views)

    # ---- commands (from the GUI buttons) ----
    def capture(self, frame, corners=None):
        h, w = frame.shape[:2]
        if corners is None:
            corners = self._detect(frame)
        if corners is None:
            self.message = "board not found: hold the whole board in view, flat and well lit"
            return False
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        win = max(5, int(round(w / PREVIEW_WIDTH * 4)))
        refined = cv2.cornerSubPix(gray, corners.astype(np.float32), (win, win), (-1, -1),
                                   (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 40, 0.01))
        with self._lock:
            if self.size not in (None, (w, h)):
                self.views, self.result = [], None   # resolution changed: old views no longer apply
            self.size = (w, h)
            self.views.append(refined)
            self._last_capture_t = time.time()
            n = len(self.views)
            self.message = f"captured view {n}: move the board somewhere new" + \
                           ("" if n < GOOD_VIEWS else " (enough views: press Calibrate)")
        return True

    def undo(self):
        with self._lock:
            if self.views:
                self.views.pop()
            self.message = f"removed last view ({len(self.views)} left)"

    def reset(self):
        with self._lock:
            self.views, self.result, self.message = [], None, "all views cleared"

    def calibrate(self):
        with self._lock:
            views, size = list(self.views), self.size
        if len(views) < MIN_VIEWS:
            self.message = f"need at least {MIN_VIEWS} views ({GOOD_VIEWS} recommended), have {len(views)}"
            return None
        self.message = "calibrating..."
        objp = np.zeros((self.board[0] * self.board[1], 3), np.float32)
        objp[:, :2] = np.mgrid[0:self.board[0], 0:self.board[1]].T.reshape(-1, 2)  # unit squares: scale is irrelevant
        rms, K, dist, _, _ = cv2.calibrateCamera([objp] * len(views), views, size, None, None)
        hfov = float(2 * np.degrees(np.arctan(size[0] / (2 * K[0, 0]))))
        vfov = float(2 * np.degrees(np.arctan(size[1] / (2 * K[1, 1]))))
        CameraModel(K, dist, size).save(self.out_path, rms_px=float(rms), views=len(views))
        result = {"rms_px": round(float(rms), 3), "views": len(views), "fx": round(float(K[0, 0]), 1),
                  "fy": round(float(K[1, 1]), 1), "hfov_deg": round(hfov, 1), "vfov_deg": round(vfov, 1),
                  "size": list(size), "saved": str(self.out_path), "good": bool(rms < 1.0)}
        with self._lock:
            self.result = result
            self.message = (f"saved: RMS {rms:.2f} px, horizontal FOV {hfov:.0f} deg" +
                            ("" if rms < 1.0 else "  (RMS > 1 px: check the board is flat and in focus, then recapture)"))
        return result

    # ---- drawing and status ----
    def draw(self, img):
        """Draw captured-view coverage and the live board detection onto img (full-res pixel coords)."""
        h, w = img.shape[:2]
        thick = max(2, w // 800)
        with self._lock:
            views = list(self.views)
            corners = self._preview[0] if self._preview else None
        if views:
            shade = img.copy()
            for v in views:
                quad = self._outline(v)
                cv2.fillPoly(shade, [quad], (0, 160, 255))
            cv2.addWeighted(shade, 0.18, img, 0.82, 0, img)
            for v in views:
                cv2.polylines(img, [self._outline(v)], True, (0, 160, 255), thick)
        if corners is not None:  # sized to the image, so it stays visible when the view is scaled down
            grid = corners.reshape(self.board[1], self.board[0], 2).astype(np.int32)
            for row in grid:
                cv2.polylines(img, [row], False, (0, 255, 0), thick)
            for pt in grid.reshape(-1, 2):
                cv2.circle(img, tuple(int(v) for v in pt), thick * 2, (0, 0, 255), -1)
            cv2.polylines(img, [self._outline(corners)], True, (0, 255, 0), thick * 2)
        return img

    def _outline(self, corners):
        c = corners.reshape(self.board[1], self.board[0], 2)
        return np.array([c[0, 0], c[0, -1], c[-1, -1], c[-1, 0]], np.int32)

    def status(self):
        with self._lock:
            seen = self._preview is not None and self._preview[0] is not None and time.time() - self._preview[1] < 2
            return {"active": self.active, "auto": self.auto, "views": len(self.views), "min_views": MIN_VIEWS,
                    "good_views": GOOD_VIEWS, "board_visible": seen, "board": list(self.board),
                    "message": self.message, "result": self.result}
