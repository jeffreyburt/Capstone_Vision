"""Per-frame processing: markers -> robot observations -> tracks -> output message."""

from dataclasses import dataclass, field

from .geometry import load_geometry
from .markers import MarkerDetector
from .output import make_message
from .robots import estimate_robots, robot_specs_from_config
from .tracking import RobotTracker


@dataclass
class FrameResult:
    t: float
    frame_index: int
    markers: list = field(default_factory=list)       # MarkerDetection
    observations: list = field(default_factory=list)  # RobotObservation (this frame, unsmoothed)
    message: dict = None                               # output message (smoothed tracks)


class VisionPipeline:
    def __init__(self, cfg, geometry=None):
        """geometry: pass one in for tests; otherwise it is loaded from the calibration files on the first frame."""
        self.cfg = cfg
        self.specs = robot_specs_from_config(cfg)
        marker_ids = [mid for s in self.specs for mid in s.faces]
        self.detector = MarkerDetector(cfg.get("markers", {}).get("dictionary", "DICT_4X4_50"), marker_ids)
        self.tracker = RobotTracker([s.id for s in self.specs], cfg.get("tracking"))
        self.geometry = geometry
        self._geometry_loaded = geometry is not None
        self.frame_index = 0

    def _in_area(self, x, y):
        area = self.cfg.get("area")
        if not area:
            return True
        return area["x"][0] <= x <= area["x"][1] and area["y"][0] <= y <= area["y"][1]

    def process(self, frame, t):
        if not self._geometry_loaded:
            h, w = frame.shape[:2]
            self.geometry = load_geometry(self.cfg, (w, h))
            self._geometry_loaded = True

        markers = self.detector.detect(frame)
        observations = []
        if self.geometry is not None:
            observations = [o for o in estimate_robots(markers, self.specs, self.geometry) if self._in_area(o.x, o.y)]
        self.tracker.update(t, observations)

        result = FrameResult(t, self.frame_index, markers, observations,
                             make_message(t, self.frame_index, self.tracker.state(t)))
        self.frame_index += 1
        return result
