"""Per-frame processing: markers -> robots, people -> user, both -> tracks -> output message."""

from dataclasses import dataclass, field

from .geometry import load_geometry
from .joint_pose import JointFeedback
from .markers import MarkerDetector
from .output import make_message
from .robots import estimate_marker_positions, estimate_robots, robot_specs_from_config
from .tracking import RobotTracker, UserTracker


@dataclass
class FrameResult:
    t: float
    frame_index: int
    markers: list = field(default_factory=list)       # MarkerDetection
    observations: list = field(default_factory=list)  # RobotObservation (this frame, unsmoothed)
    people: list = field(default_factory=list)        # PersonDetection (most recent detector result)
    user_observation: object = None                   # PersonObservation chosen as the user (if new this frame)
    message: dict = None                               # output message (smoothed tracks)


class VisionPipeline:
    def __init__(self, cfg, geometry=None, person_detector=None):
        """geometry: pass one in for tests; otherwise it is loaded from the calibration files on the first frame.
        person_detector: any object with detect(frame) -> [PersonDetection]; default from config `people:`."""
        self.cfg = cfg
        self.specs = robot_specs_from_config(cfg)
        marker_ids = [mid for s in self.specs for mid in s.faces]
        marker_cfg = cfg.get("markers", {})
        self.detector = MarkerDetector(marker_cfg.get("dictionary", "DICT_4X4_50"), marker_ids,
                                       adaptive_thresh_constant=marker_cfg.get("adaptive_thresh_constant", 3))
        self.tracker = RobotTracker([s.id for s in self.specs], cfg.get("tracking"))
        self.user_tracker = UserTracker(cfg.get("user_tracking", cfg.get("tracking")))
        self.geometry = geometry
        self._geometry_loaded = geometry is not None
        self.frame_index = 0

        p = cfg.get("people", {})
        self.head_height_m = p.get("head_height_m", 1.70)
        self.people = person_detector
        if self.people is None and p.get("enabled", False):
            self.people = self._make_person_detector(p)
        self.async_people = None
        if self.people is not None and p.get("async", True):
            from .people import AsyncDetector
            self.async_people = AsyncDetector(self.people.detect)
        self.last_people = []
        endpoints = {int(rid): item["host"] for rid, item in cfg.get("control", {}).get("robots", {}).items()}
        sources = {spec.id: endpoints[spec.id]
                   for spec in self.specs if spec.head is not None}
        self.joints = JointFeedback(cfg.get("joint_feedback_port", 5006), sources) if sources else None
        self.compensated_ids = set(sources)

    @staticmethod
    def _make_person_detector(p):
        try:
            from .people import PersonDetector
            return PersonDetector(p.get("model", "yolo11n.pt"), p.get("imgsz", 640), p.get("conf", 0.4),
                                  p.get("device"))
        except ImportError:
            print("NOTE: person detection disabled: 'ultralytics' is not installed "
                  "(laptop: pip install -r requirements-laptop.txt)")
            return None

    def _in_area(self, x, y):
        area = self.cfg.get("area")
        if not area:
            return True
        return area["x"][0] <= x <= area["x"][1] and area["y"][0] <= y <= area["y"][1]

    def _people_step(self, frame, t):
        """Run (or collect) person detection. Returns (detection time, detections) or None if nothing new."""
        if self.people is None:
            return None
        if self.async_people is None:
            return t, self.people.detect(frame)
        self.async_people.submit(frame, t)
        return self.async_people.poll()

    def process(self, frame, t):
        if not self._geometry_loaded:
            h, w = frame.shape[:2]
            self.geometry = load_geometry(self.cfg, (w, h))
            self._geometry_loaded = True

        markers = self.detector.detect(frame)
        observations = []
        if self.geometry is not None:
            joints = self.joints.positions_at(t) if self.joints is not None else None
            estimates = (estimate_marker_positions(markers, self.specs, self.geometry) if self.geometry.approximation
                         else estimate_robots(markers, self.specs, self.geometry, joints))
            observations = [o for o in estimates
                            if self._in_area(o.x, o.y)]
        self.tracker.update(t, observations)

        user_obs = None
        got = self._people_step(frame, t)
        if got is not None:
            t_det, self.last_people = got
            if self.geometry is not None:
                from .people import locate_person
                detections = self.last_people
                if self.geometry.approximation:
                    detections = [d for d in detections if d.box[3] < frame.shape[0] - 3]
                cands = [locate_person(d, self.geometry, frame.shape[0], self.head_height_m) for d in detections]
                cands = [c for c in cands if c is not None and self._in_area(c.x, c.y)]
                user_obs = self.user_tracker.update(t_det, cands)

        message = make_message(t, self.frame_index, self.tracker.state(t), self.user_tracker.state(t))
        if self.geometry is not None and self.geometry.approximation:
            message.update(position_quality="approximate", driving_allowed=False,
                           approximation=self.geometry.approximation)
            for robot in message["robots"]:
                robot.update(heading=None, reference="visible_marker_centers")
        if self.compensated_ids:
            observed = {o.robot_id for o in observations}
            for robot in message["robots"]:
                if robot["id"] in self.compensated_ids:
                    robot["pose_compensated"] = robot["id"] in observed and robot["age"] == 0
        result = FrameResult(t, self.frame_index, markers, observations, self.last_people, user_obs, message)
        self.frame_index += 1
        return result

    def reload_geometry(self):
        """Re-read the calibration files on the next frame (e.g. after a new lens calibration)."""
        self._geometry_loaded = False

    def close(self):
        if self.joints is not None:
            self.joints.close()
        if self.async_people is not None:
            self.async_people.close()
