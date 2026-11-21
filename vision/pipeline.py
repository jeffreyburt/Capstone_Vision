"""Per-frame processing: markers -> robots, people -> user, both -> tracks -> output message."""

from dataclasses import dataclass, field
import time
import math

from .geometry import load_geometry
from .joint_pose import JointFeedback, DashboardJointFeedback
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
        self.floor_marker_ids = set((cfg.get("calibration") or {}).get("floor_marker_ids", []))
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
                   for spec in self.specs if spec.head is not None and spec.head.needs_feedback}
        self.joints = None
        if sources:
            if cfg.get('joint_feedback_transport') == 'dashboard_http':
                if not (cfg.get('camera') or {}).get('local_frame_timestamps'):
                    raise ValueError('dashboard_http joints require local_frame_timestamps')
                self.joints = DashboardJointFeedback(sources,cfg)
            else:
                self.joints = JointFeedback(cfg.get("joint_feedback_port",5006),sources)
        self.compensated_ids = {spec.id for spec in self.specs if spec.head is not None}
        self.pending_head_ids = {int(r["id"]) for r in cfg.get("robots", [])
                                 if r.get("moving_head_markers") and r.get("head_pose") is None}

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
        robot_markers = [m for m in markers if m.id not in self.floor_marker_ids]
        joints = {}
        if self.geometry is not None:
            joints = self.joints.positions_at(t) if self.joints is not None else None
            priors = {rid: math.radians(track.heading) for rid,track in self.tracker.tracks.items()
                      if track.heading is not None and track.age(t)<=.65}
            estimates = (estimate_marker_positions(robot_markers, self.specs, self.geometry) if self.geometry.approximation
                         else estimate_robots(robot_markers, self.specs, self.geometry, joints,priors))
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
                    frame_fresh = (not (self.cfg.get('camera') or {}).get('local_frame_timestamps') or
                                   0 <= time.time()-t+robot['age'] <= float(self.cfg.get('compensated_frame_timeout_s',.25)))
                    robot["pose_compensated"] = frame_fresh and (robot["id"] in observed or ((self.cfg.get("camera") or {}).get("local_frame_timestamps") and robot["age"] > 0))
                    head = next(spec.head for spec in self.specs if spec.id == robot['id'])
                    units = (joints or {}).get(robot['id']) if head.needs_feedback else None
                    if head.needs_feedback and units is None:
                        robot['pose_compensated'] = False
                        robot['pose_status'] = 'Fresh head feedback unavailable'
                    elif head.needs_feedback and head.unit_limits is not None and any(
                            not lo <= value <= hi for value,(lo,hi) in zip(units,head.unit_limits)):
                        robot['pose_compensated'] = False
                        robot['pose_status'] = 'Head outside calibrated movement range'
                    elif not frame_fresh:
                        robot['pose_status'] = 'Camera capture too old'
                    elif not robot['pose_compensated']:
                        robot['pose_status'] = 'Calibrated marker not reliably visible'
                    else:
                        robot['pose_status'] = 'Recent calibrated observation' if robot['age']>0 else 'Calibrated moving head' if head.needs_feedback else 'Secured fixed head'
                    if units is not None:
                        robot['head_units'] = units.tolist()
        approximate = {o.robot_id for o in observations if getattr(o, 'approximate', False)}
        source = {o.robot_id: getattr(o, 'source', 'fit') for o in observations}
        for robot in message['robots']:
            if robot['id'] in source:
                robot['position_source'] = source[robot['id']]
            if robot['id'] in approximate:
                if robot.get('pose_status') == 'Calibrated marker not reliably visible' or robot.get('pose_compensated'):
                    robot['pose_status'] = 'Tag-centre position (heading held from earlier fits)'
            if robot['id'] in self.pending_head_ids:
                robot.update(heading=None, pose_compensated=False, reference='uncalibrated_head_markers')
        if (self.cfg.get('camera') or {}).get('local_frame_timestamps'):
            message['frame_clock'] = 'local'
            message['max_frame_age_s'] = float(self.cfg.get('compensated_frame_timeout_s',.25))
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
