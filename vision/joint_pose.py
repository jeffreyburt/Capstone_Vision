"""Joint feedback and marker geometry for articulated speaker mounts."""

import json
import math
import socket
import time
from collections import deque
from dataclasses import dataclass

import numpy as np
import cv2


def _vector(values, length):
    vector = np.asarray(values, dtype=float)
    if vector.shape != (length,) or not np.isfinite(vector).all():
        raise ValueError(f"expected {length} finite values, got {values!r}")
    return vector


def _rotate(points, pivot, angle, axis):
    """Rotate points around chassis +Z (pan) or neutral +Y (tilt)."""
    c, s = math.cos(angle), math.sin(angle)
    rotation = ((c, -s, 0), (s, c, 0), (0, 0, 1)) if axis == "pan" else (
        (c, 0, s), (0, 1, 0), (-s, 0, c)
    )
    return (points - pivot) @ np.asarray(rotation).T + pivot


@dataclass
class HeadGeometry:
    neutral_units: np.ndarray
    degrees_per_unit: np.ndarray
    pan_pivot: np.ndarray
    tilt_pivot: np.ndarray
    mounts: dict
    unit_limits: np.ndarray | None = None
    max_rms_px: float = 3.0  # reject chassis fits whose corner reprojection RMS exceeds this (wobbly heads need more)
    heading_window: tuple | None = None  # (center_deg, half_width_deg): reject fits facing outside it (e.g. a hallway)
    center_only: frozenset = frozenset()  # tags only good for a tag-centre position (never a full chassis fit)
    marker_limits: dict = None  # marker id -> [[pan lo, hi], [tilt lo, hi]]: a tag's own measured joint range

    @property
    def needs_feedback(self):
        return any(mount != "fixed" for mount, _ in self.mounts.values())

    @classmethod
    def from_config(cls, cfg, spec):
        neutral = _vector(cfg["neutral_units"], 2)
        scale = _vector(cfg["degrees_per_unit"], 2)
        if np.any((neutral < 0) | (neutral > 1000)) or np.any(scale == 0):
            raise ValueError("invalid servo neutral_units or degrees_per_unit")
        mounts, center_only, marker_limits = {}, set(), {}
        for key, marker in cfg["markers"].items():
            marker_id = int(key)
            mount = marker["mount"]
            if marker_id not in spec.faces or mount not in ("pan", "tilt", "tilt_only", "fixed"):
                raise ValueError(f"invalid articulated marker {marker_id}: {mount!r}")
            center = _vector(marker["center_m"], 3)
            yaw = math.radians(spec.faces[marker_id])
            right = np.array([-math.sin(yaw), math.cos(yaw), 0])
            up = np.array([0, 0, 1])
            half = spec.marker_size_m / 2
            offsets = half * np.array([up - right, up + right, -up + right, -up - right])
            if 'rotation_vector_rad' in marker:
                rotation = cv2.Rodrigues(_vector(marker['rotation_vector_rad'], 3))[0]
                offsets = offsets @ rotation.T
            corners = center + offsets
            mounts[marker_id] = mount, corners
            if marker.get('center_only'):
                center_only.add(marker_id)
            if marker.get('valid_units') is not None:
                own = np.asarray(marker['valid_units'], dtype=float)
                if own.shape != (2, 2) or not np.isfinite(own).all() or np.any(own[:, 0] >= own[:, 1]):
                    raise ValueError(f'marker {marker_id} valid_units must be two increasing ranges')
                marker_limits[marker_id] = own
        if not mounts:
            raise ValueError("head_pose.markers must list at least one measured marker")
        limits = None
        if 'valid_units' in cfg:
            limits = np.asarray(cfg['valid_units'], dtype=float)
            if limits.shape != (2,2) or not np.isfinite(limits).all() or np.any(limits[:,0] >= limits[:,1]):
                raise ValueError('valid_units must contain two finite increasing ranges')
        max_rms = float(cfg.get('max_rms_px', 3.0))
        if not 0 < max_rms <= 30:
            raise ValueError('max_rms_px must be in (0, 30]')
        window = cfg.get('heading_window_deg')
        if window is not None:
            if len(window) != 2 or not all(math.isfinite(float(v)) for v in window) or not 0 < float(window[1]) < 180:
                raise ValueError('heading_window_deg must be [center_deg, half_width_deg] with 0 < half_width < 180')
            window = (float(window[0]), float(window[1]))
        return cls(neutral, scale, _vector(cfg["pan_pivot_m"], 3), _vector(cfg["tilt_pivot_m"], 3), mounts, limits, max_rms, window,
                   frozenset(center_only), marker_limits)

    def corners(self, marker_id, units, own_limits=True):
        if marker_id not in self.mounts:
            return None
        mount, neutral_corners = self.mounts[marker_id]
        if mount == 'fixed':
            return neutral_corners
        units = _vector(units, 2)
        if self.unit_limits is not None and np.any((units < self.unit_limits[:,0]) | (units > self.unit_limits[:,1])):
            return None
        own = (self.marker_limits or {}).get(marker_id) if own_limits else None
        if own is not None and np.any((units < own[:,0]) | (units > own[:,1])):
            return None  # outside the range this tag's model was measured over
        pan, tilt = np.radians((units - self.neutral_units) * self.degrees_per_unit)
        corners = neutral_corners
        if mount in ("tilt", "tilt_only"):
            corners = _rotate(corners, self.tilt_pivot, tilt, "tilt")
        if mount in ("pan", "tilt"):
            corners = _rotate(corners, self.pan_pivot, pan, "pan")
        return corners


def tag_pose_world(marker, size_m, geometry, blur_px=0.0):
    """World corners (4, 3) of a square tag from its own image (PnP), with the distance corrected for corner blur:
    detected squares look `blur_px` pixels larger than they are, which matters more the smaller the tag looks."""
    h = size_m / 2
    obj = np.array([[-h, h, 0], [h, h, 0], [h, -h, 0], [-h, -h, 0]], float)
    ok, rvec, tvec = cv2.solvePnP(obj, np.asarray(marker.corners, dtype=float), geometry.camera.K, geometry.camera.dist,
                                  flags=cv2.SOLVEPNP_IPPE_SQUARE)
    if not ok or not np.isfinite(tvec).all() or tvec[2, 0] <= .1:
        return None
    side = marker.side_px
    scale = side / (side - blur_px) if 0 <= blur_px < side * .5 else 1.0
    cam = (cv2.Rodrigues(rvec)[0] @ obj.T).T + tvec.ravel() * scale
    world = (geometry.pose.R.T @ (cam - geometry.pose.tvec.ravel()).T).T
    return world if np.isfinite(world).all() else None


def tag_center_world(marker, size_m, geometry, blur_px=0.0):
    """World (x, y, z) of a tag's centre from its own image: direction from the pixel, distance from its size.

    The main camera sits barely above the tags, so intersecting the ray with an assumed tag height is badly
    conditioned (1 cm of height = ~15 cm along the floor at 2 m). The square's apparent size fixes the distance
    instead, independent of how high the tag is. None if the tag can't be solved."""
    h = size_m / 2
    obj = np.array([[-h, h, 0], [h, h, 0], [h, -h, 0], [-h, -h, 0]], float)
    ok, rvec, tvec = cv2.solvePnP(obj, np.asarray(marker.corners, dtype=float), geometry.camera.K, geometry.camera.dist,
                                  flags=cv2.SOLVEPNP_IPPE_SQUARE)
    if not ok or not np.isfinite(tvec).all() or tvec[2, 0] <= .1:
        return None
    side = marker.side_px
    scale = side / (side - blur_px) if 0 <= blur_px < side * .5 else 1.0
    world = geometry.pose.R.T @ (tvec.ravel() * scale - geometry.pose.tvec.ravel())
    return world if np.isfinite(world).all() else None


def center_ray_position(marker, corners, geometry, heading, size_m=0.10, blur_px=0.0):
    """Chassis (x, y) from a single tag's centre in the world and the tag's offset on the chassis.

    Robust to head wobble and oblique views: the tag centre moves little, and an error in the assumed heading only
    swings the small tag-to-chassis offset."""
    center = np.asarray(corners, dtype=float).mean(axis=0)
    world = tag_center_world(marker, size_m, geometry, blur_px)
    if world is None:
        return None
    point = world[:2]
    c, s = math.cos(heading), math.sin(heading)
    return point - np.array([c*center[0] - s*center[1], s*center[0] + c*center[1]])


def fit_chassis(marker, corners, geometry, prior_heading=None, max_rms_px=3.0, heading_window=None):
    """Fit a level chassis directly in image space.

    Intersecting each corner ray with its assumed height is ill-conditioned when the
    camera is close to the marker's upper edge. Fit X/Y/yaw with all pixels instead.
    IPPE supplies starting poses; fixed floor roll/pitch reject planar pose flips.
    """
    corners = np.asarray(corners, dtype=float)
    observed = np.asarray(marker.corners, dtype=float)
    camera, floor = geometry.camera, geometry.pose
    try:
        solutions = cv2.solvePnPGeneric(corners, observed, camera.K, camera.dist,
                                        flags=cv2.SOLVEPNP_IPPE)
    except cv2.error:
        return None

    def residual(state):
        x,y,h = state
        c,s = math.cos(h),math.sin(h)
        rotation = np.array([[c,-s,0],[s,c,0],[0,0,1]])
        world = corners @ rotation.T + [x,y,0]
        points = cv2.projectPoints(world, floor.rvec, floor.tvec, camera.K, camera.dist)[0].reshape(4,2)
        return (points-observed).ravel()

    seeds = []
    measured = geometry.pixels_to_floor(observed,corners[:,2])
    if np.isfinite(measured).all():
        a = corners[:,:2]-corners[:,:2].mean(axis=0)
        b = measured-measured.mean(axis=0)
        dot = np.sum(a*b)
        cross = np.sum(a[:,0]*b[:,1]-a[:,1]*b[:,0])
        if math.hypot(dot,cross) > 1e-10:
            h = math.atan2(cross,dot)
            c,s = math.cos(h),math.sin(h)
            rotation = np.array([[c,s],[-s,c]])
            center = measured.mean(axis=0)-corners[:,:2].mean(axis=0)@rotation
            seeds.append(np.array([center[0],center[1],h]))
    for rvec,tvec in zip(solutions[1], solutions[2]):
        world_rotation = floor.R.T @ cv2.Rodrigues(rvec)[0]
        position = floor.R.T @ (tvec.ravel()-floor.tvec.ravel())
        if world_rotation[2,2] < .7 or abs(position[2]) > .15:
            continue
        seeds.append(np.array([position[0],position[1],math.atan2(world_rotation[1,0],world_rotation[0,0])]))
    best = None
    for state in seeds:
        for _ in range(25):
            error = residual(state)
            if not np.isfinite(error).all():
                break
            eps = 1e-5
            jac = np.column_stack([(residual(state+np.eye(3)[i]*eps)-error)/eps for i in range(3)])
            delta = np.linalg.lstsq(jac, -error, rcond=None)[0]
            delta[:2] = np.clip(delta[:2], -.1, .1)
            delta[2] = np.clip(delta[2], -.2, .2)
            improved = False
            for scale in (1., .5, .25, .125):
                trial = state + scale*delta
                if np.sum(residual(trial)**2) < np.sum(error**2):
                    state, improved = trial, True
                    break
            if not improved or np.linalg.norm(delta) < 1e-7:
                break
        error = residual(state)
        if np.isfinite(error).all():
            score = float(np.mean(error**2))
            if score > max_rms_px**2:  # never trade reprojection validity for temporal continuity
                continue
            if heading_window is not None and abs((math.degrees(state[2])-heading_window[0]+180)%360-180) > heading_window[1]:
                continue  # a mirror-image solution facing the wrong way
            selection = score
            if prior_heading is not None:
                difference=(state[2]-prior_heading+math.pi)%(2*math.pi)-math.pi
                selection += (.5*math.degrees(difference)/10.)**2
            if best is None or selection < best[0]:
                best = selection, state
    if best is None:
        return None
    return best[1][:2], float(best[1][2])


class JointFeedback:
    """Keep a short ordered history of valid measured positions from each robot."""

    def __init__(self, port, sources):
        self.sources = sources
        self.history = {rid: deque(maxlen=16) for rid in sources}
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            self.sock.bind(("0.0.0.0", port))
            self.sock.setblocking(False)
        except OSError:
            self.sock.close()
            raise

    def poll(self):
        now = time.time()
        for _ in range(128):
            try:
                packet, addr = self.sock.recvfrom(1024)
            except (BlockingIOError, ConnectionResetError):
                break
            try:
                msg = json.loads(packet)
                rid, stamp, units = msg["robot_id"], msg["t"], msg["units"]
                if type(rid) is not int or self.sources.get(rid) != addr[0]:
                    continue
                if type(stamp) not in (float, int) or not math.isfinite(stamp) or abs(now - stamp) > 0.4:
                    continue
                if not isinstance(units, list) or len(units) != 2:
                    continue
                if any(type(value) is not int or not 0 <= value <= 1000 for value in units):
                    continue
            except (ValueError, KeyError, TypeError):
                continue
            samples = self.history[rid]
            if not samples or stamp > samples[-1][0]:
                samples.append((stamp, np.array(units)))

    def positions_at(self, stamp):
        self.poll()
        if abs(time.time() - stamp) > 0.4:
            return {}
        result = {}
        for rid, samples in self.history.items():
            if samples:
                sample_time, units = min(samples, key=lambda item: abs(item[0] - stamp))
                if abs(sample_time - stamp) <= 0.12:
                    result[rid] = units
        return result

    def close(self):
        self.sock.close()


class DashboardJointFeedback:
    """Read measured joints through the existing SSH gateway, mapped to the laptop clock.

    Remote clocks need not agree: reply serverTime minus position_time gives sample age.
    Slow round trips and stale/error replies are rejected. Samples retain their age rather
    than becoming fresh on every HTTP poll. No commands or torque changes are sent.
    """
    def __init__(self, sources, cfg):
        import shutil
        import threading
        from monitor.tunnels import CameraTunnels
        self.sources = sources
        self.max_frame_age = float(cfg.get('compensated_frame_timeout_s',.4))
        self.history = {rid: deque(maxlen=64) for rid in sources}
        self.lock = threading.Lock()
        self.stop = threading.Event()
        system = cfg.get('system') or {}
        self.robot_cfg = {int(k):v for k,v in (system.get('robots') or {}).items()}
        self.tunnels = CameraTunnels(shutil.which('ssh') or 'ssh',system.get('probe_via'))
        self.urls = {}
        self.thread = threading.Thread(target=self._run,daemon=True,name='joint-http-feedback')
        self.thread.start()

    @staticmethod
    def decode(reply, started, ended):
        values = reply.get('positions')
        if reply.get('position_error') or not isinstance(values,list) or len(values)!=2:
            return None
        if any(type(v) is not int or not 0 <= v <= 1000 for v in values):
            return None
        server, measured = reply.get('serverTime'),reply.get('position_time')
        if not all(type(v) in (int,float) and math.isfinite(v) for v in (server,measured,started,ended)):
            return None
        age = server-measured
        if not 0 <= ended-started <= .15 or not 0 <= age <= .35:
            return None
        return (started+ended)/2-age, np.array(values)

    def _run(self):
        import urllib.request
        while not self.stop.is_set():
            for rid,host in self.sources.items():
                if self.stop.is_set():
                    break
                try:
                    if rid not in self.urls:
                        port=self.robot_cfg.get(rid,{}).get('dashboard_port',8080)
                        endpoint,port=self.tunnels.endpoint(host,port)
                        self.urls[rid]=f'http://{endpoint}:{port}/api/state'
                    started=time.time()
                    with urllib.request.urlopen(self.urls[rid],timeout=.3) as response:
                        reply=json.load(response)
                    sample=self.decode(reply,started,time.time())
                    if sample is not None:
                        with self.lock:
                            history=self.history[rid]
                            if not history or sample[0]>history[-1][0]:
                                history.append(sample)
                except (OSError,ValueError,TypeError):
                    # Forget the URL: its SSH tunnel may have died (e.g. the main Pi rebooted); the next poll asks
                    # for a fresh one instead of failing on the dead port forever.
                    self.urls.pop(rid,None)
            self.stop.wait(.04)

    @staticmethod
    def interpolate(samples, stamp):
        for (before,a),(after,b) in zip(samples,samples[1:]):
            if before <= stamp <= after and 0 < after-before <= .3:
                fraction=(stamp-before)/(after-before)
                return a+(b-a)*fraction
        if samples:
            # Servo readings reach the laptop 0.1-0.2 s old, so the newest one is often older than the frame: carry
            # it forward along its recent velocity (at most 0.25 s) instead of dropping the robot's pose.
            (before,a),(after,b)=samples[-2:] if len(samples)>1 else (samples[-1],samples[-1])
            if 0 < stamp-after <= .25:
                rate=(b-a)/(after-before) if 0 < after-before <= .3 else 0*b
                return b+rate*(stamp-after)
            when,units=min(samples,key=lambda sample:abs(sample[0]-stamp))
            if abs(when-stamp) <= .12:
                return units.copy()
        return None

    def positions_at(self, stamp):
        if not 0 <= time.time()-stamp <= self.max_frame_age:
            return {}
        result={}
        with self.lock:
            for rid,samples in self.history.items():
                units=self.interpolate(list(samples),stamp)
                if units is not None:
                    result[rid]=units
        return result

    def close(self):
        self.stop.set()
        self.thread.join(timeout=6)
        self.tunnels.close()
