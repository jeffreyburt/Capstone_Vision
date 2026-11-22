import json
from unittest.mock import patch

import numpy as np
import pytest

from robot.drive_server import parse_command
from vision.geometry import CameraModel, FloorPose, Geometry
from vision.joint_pose import JointFeedback
from vision.markers import MarkerDetection
from vision.robots import estimate_robots, robot_specs_from_config, wrap_deg


@pytest.fixture
def articulated():
    return robot_specs_from_config({"robots": [{
        "id": 1, "faces": {1: 90, 2: 180, 3: 270, 0: 0},
        "head_pose": {
            "neutral_units": [500, 500], "degrees_per_unit": [1, -1],
            "pan_pivot_m": [0.1, 0, 0.3], "tilt_pivot_m": [0.1, 0, 0.4],
            "markers": {
                1: {"mount": "pan", "center_m": [0.1, 0.15, 0.3]},
                2: {"mount": "tilt", "center_m": [-0.1, 0, 0.4]},
                3: {"mount": "pan", "center_m": [0.1, -0.15, 0.3]},
            },
        },
    }]})


@pytest.mark.parametrize("marker_id, corners", [
    (1, [[-.05, .05, .35], [-.05, -.05, .35], [-.05, -.05, .25], [-.05, .05, .25]]),
    (3, [[.25, -.05, .35], [.25, .05, .35], [.25, .05, .25], [.25, -.05, .25]]),
    (2, [[.05, .05, .60], [.15, .05, .60], [.15, -.05, .60], [.05, -.05, .60]]),
])
def test_recovers_chassis_from_each_articulated_marker(articulated, marker_id, corners):
    # Independently calculated corners for +90° pan and +90° tilt.
    geometry = Geometry(CameraModel.from_fov(1920, 1080, 66), FloorPose.look_at([0, 0, 2], [0, 4, .3]))
    angle = np.radians(35)
    rotation = np.array([[np.cos(angle), np.sin(angle)], [-np.sin(angle), np.cos(angle)]])
    corners = np.array(corners)
    corners[:, :2] = corners[:, :2] @ rotation + [.3, 4]
    detect = MarkerDetection(marker_id, geometry.world_to_pixels(corners))
    assert estimate_robots([detect], articulated, geometry) == []
    obs, = estimate_robots([detect], articulated, geometry, {1: np.array([590, 410])})
    assert (obs.x, obs.y) == pytest.approx((.3, 4), abs=1e-5)
    assert abs(wrap_deg(obs.heading_deg - 35)) < 1e-4


def test_feedback_requires_fresh_samples_from_configured_address():
    with patch("vision.joint_pose.socket.socket") as socket, patch("vision.joint_pose.time.time", return_value=100):
        def packet(t, source):
            return json.dumps({"robot_id": 1, "t": t, "units": [450, 520]}).encode(), (source, 10000)

        socket.return_value.recvfrom.side_effect = [
            packet(99.97, "192.168.50.12"), packet(99.5, "192.168.50.11"),
            packet(99.94, "192.168.50.11"), BlockingIOError, BlockingIOError,
        ]
        feedback = JointFeedback(5006, {1: "192.168.50.11"})
        assert list(feedback.positions_at(100)[1]) == [450, 520]
        assert feedback.positions_at(100.2) == {}
        feedback.close()


def test_compensated_drive_requires_marker_pose_and_accepts_stop():
    drive = {"type": "drive", "robot_id": 1, "vx": .1, "vy": 0, "wz": 0}
    with pytest.raises(ValueError, match="compensated pose"):
        parse_command(json.dumps(drive), 1, require_compensated_pose=True)
    drive["pose_compensated"] = True
    assert parse_command(json.dumps(drive), 1, require_compensated_pose=True)[0] == "drive"
    assert parse_command('{"type":"stop","robot_id":1}', 1, require_compensated_pose=True)[0] == "stop"


def test_fixed_head_geometry_needs_no_live_servo_feedback():
    cfg={'robots':[{'id':1,'faces':{2:180},'head_pose':{
        'neutral_units':[0,0],'degrees_per_unit':[1,1],
        'pan_pivot_m':[0,0,0],'tilt_pivot_m':[0,0,0],
        'markers':{2:{'mount':'fixed','center_m':[-.14,0,.295]}}}}]}
    spec=robot_specs_from_config(cfg)[0]
    assert not spec.head.needs_feedback
    geometry=Geometry(CameraModel.from_fov(1600,900,102),FloorPose.look_at([0,0,.395],[0,3,0]))
    corners=spec.head.corners(2,None)+[0,2,0]
    marker=MarkerDetection(2,geometry.world_to_pixels(corners))
    obs,=estimate_robots([marker],[spec],geometry)
    assert (obs.x,obs.y)==pytest.approx((0,2),abs=1e-5)


def test_calibrated_joint_envelope_rejects_extrapolation(articulated):
    head=articulated[0].head
    head.unit_limits=np.array([[450,550],[450,550]])
    assert head.corners(2,np.array([500,500])) is not None
    assert head.corners(2,np.array([551,500])) is None
    assert head.corners(2,np.array([500,449])) is None


def test_http_feedback_maps_clock_skew_but_preserves_sample_age():
    from vision.joint_pose import DashboardJointFeedback
    reply={'positions':[391,770],'position_error':None,'serverTime':1_000_000.,'position_time':999_999.9}
    stamp,units=DashboardJointFeedback.decode(reply,100.,100.04)
    assert stamp==pytest.approx(99.92)
    assert list(units)==[391,770]
    assert DashboardJointFeedback.decode(reply,100.,100.3) is None
    reply['position_time']=999_998.
    assert DashboardJointFeedback.decode(reply,100.,100.04) is None


def test_single_missed_frame_retains_recent_fixed_pose_without_resetting_its_age(monkeypatch):
    from vision.pipeline import VisionPipeline
    from vision.robots import RobotObservation
    import vision.pipeline as module
    geometry=Geometry(CameraModel.from_fov(640,480,100),FloorPose.look_at([0,0,.395],[0,2,0]))
    cfg={'camera':{'local_frame_timestamps':True},'compensated_frame_timeout_s':.65,
         'robots':[{'id':1,'faces':{2:180},'head_pose':{
             'neutral_units':[0,0],'degrees_per_unit':[1,1],
             'pan_pivot_m':[0,0,0],'tilt_pivot_m':[0,0,0],
             'markers':{2:{'mount':'fixed','center_m':[-.14,0,.295]}}}}]}
    observed=[True]
    monkeypatch.setattr(module,'estimate_robots',lambda *args:[RobotObservation(1,0,2,90,[2])] if observed[0] else [])
    wall=[100.1];monkeypatch.setattr(module.time,'time',lambda:wall[0])
    pipeline=VisionPipeline(cfg,geometry=geometry)
    try:
        frame=np.zeros((480,640,3),np.uint8)
        assert pipeline.process(frame,100).message['robots'][0]['pose_compensated']
        observed[0]=False;wall[0]=100.4
        robot=pipeline.process(frame,100.25).message['robots'][0]
        assert robot['pose_compensated'] and robot['age']==.25
        assert robot['x']==0 and robot['y']==2  # no invented position/extrapolation
        wall[0]=100.8
        assert not pipeline.process(frame,100.6).message['robots'][0]['pose_compensated']
    finally:pipeline.close()


def test_tag_centre_fallback_keeps_position_when_the_full_fit_fails():
    cfg={'robots':[{'id':1,'faces':{2:180,3:270},'head_pose':{
        'neutral_units':[0,0],'degrees_per_unit':[1,1],'pan_pivot_m':[0,0,0],'tilt_pivot_m':[0,0,0],
        'heading_window_deg':[90,60],
        'markers':{2:{'mount':'fixed','center_m':[-.14,0,.295]},
                   3:{'mount':'fixed','center_only':True,'center_m':[-.02,-.108,.24]}}}}]}
    spec=robot_specs_from_config(cfg)[0]
    geometry=Geometry(CameraModel.from_fov(1600,900,102),FloorPose.look_at([0,0,.395],[0,3,0]))
    angle=np.radians(90)  # robot facing +y at (0.2, 2.0)
    rot=np.array([[np.cos(angle),-np.sin(angle),0],[np.sin(angle),np.cos(angle),0],[0,0,1]])
    def seen(mid, wobble=0.0):
        c=spec.head.corners(mid,None)@rot.T+[.2,2,0]
        if wobble:  # a wobbling head: the tag rolls in its own plane and sags, but stays a rigid square
            centre=c.mean(axis=0); normal=np.cross(c[1]-c[0],c[3]-c[0]); normal/=np.linalg.norm(normal)
            ang=np.radians(8); K=np.array([[0,-normal[2],normal[1]],[normal[2],0,-normal[0]],[-normal[1],normal[0],0]])
            R=np.eye(3)+np.sin(ang)*K+(1-np.cos(ang))*K@K
            c=(c-centre)@R.T+centre-[0,0,wobble]
        return MarkerDetection(mid,geometry.world_to_pixels(c))
    obs,=estimate_robots([seen(3)],[spec],geometry)  # side tag only, no history: its own orientation gives the heading
    assert obs.source=='one tag' and abs(wrap_deg(obs.heading_deg-90))<1
    assert (obs.x,obs.y)==pytest.approx((.2,2),abs=.01)
    obs,=estimate_robots([seen(2,wobble=.02)],[spec],geometry,prior_headings={1:np.radians(90)})
    assert obs.approximate and (obs.x,obs.y)==pytest.approx((.2,2),abs=.04)  # height-plane rays were ~30 cm off
    obs,=estimate_robots([seen(2),seen(3)],[spec],geometry)  # two tags: centre rays + heading between them
    assert obs.source=='two tags' and not obs.approximate and abs(wrap_deg(obs.heading_deg-90))<1.5
    obs,=estimate_robots([seen(2)],[spec],geometry)  # one good tag: the full fit
    assert obs.source=='fit' and abs(wrap_deg(obs.heading_deg-90))<1e-3


def test_two_visible_tags_give_a_heading_without_a_full_fit():
    cfg={'robots':[{'id':1,'faces':{2:180,3:270},'head_pose':{
        'neutral_units':[0,0],'degrees_per_unit':[1,1],'pan_pivot_m':[0,0,0],'tilt_pivot_m':[0,0,0],
        'markers':{2:{'mount':'fixed','center_only':True,'center_m':[-.14,0,.295]},
                   3:{'mount':'fixed','center_only':True,'center_m':[-.02,-.108,.24]}}}}]}
    spec=robot_specs_from_config(cfg)[0]
    geometry=Geometry(CameraModel.from_fov(1600,900,102),FloorPose.look_at([0,0,.395],[0,3,0]))
    angle=np.radians(70)
    rot=np.array([[np.cos(angle),-np.sin(angle),0],[np.sin(angle),np.cos(angle),0],[0,0,1]])
    tags=[MarkerDetection(mid,geometry.world_to_pixels(spec.head.corners(mid,None)@rot.T+[.2,2,0])) for mid in (2,3)]
    obs,=estimate_robots(tags,[spec],geometry)
    assert obs.source=='two tags' and abs(wrap_deg(obs.heading_deg-70))<1.5  # corner-mean pixel ~ projected centre
    assert (obs.x,obs.y)==pytest.approx((.2,2),abs=.01)


def test_one_oblique_tag_still_gives_a_heading_without_history():
    cfg={'robots':[{'id':1,'faces':{2:180},'head_pose':{
        'neutral_units':[500,500],'degrees_per_unit':[0.24,0.24],'pan_pivot_m':[0,0,0],'tilt_pivot_m':[-.05,0,.32],
        'heading_window_deg':[90,60],'max_rms_px':0.01,  # force the full fit to fail (as with a wobbling head)
        'markers':{2:{'mount':'tilt','center_m':[-.14,0,.295]}}}}]}
    spec=robot_specs_from_config(cfg)[0]
    geometry=Geometry(CameraModel.from_fov(1600,900,102),FloorPose.look_at([0,0,.395],[0,3,0]))
    units=np.array([600.,540.])  # head panned 24 deg and tilted
    angle=np.radians(80)
    rot=np.array([[np.cos(angle),-np.sin(angle),0],[np.sin(angle),np.cos(angle),0],[0,0,1]])
    pixels=geometry.world_to_pixels(spec.head.corners(2,units)@rot.T+[.2,1.6,0])
    tag=MarkerDetection(2,pixels+np.random.default_rng(0).normal(0,.5,pixels.shape))  # detection noise: full fit fails
    obs,=estimate_robots([tag],[spec],geometry,{1:units})
    assert obs.source=='one tag' and abs(wrap_deg(obs.heading_deg-80))<3
    assert (obs.x,obs.y)==pytest.approx((.2,1.6),abs=.03)


def test_one_tag_heading_is_not_trusted_with_the_head_turned_far_aside():
    cfg={'robots':[{'id':1,'faces':{2:180},'head_pose':{
        'neutral_units':[500,500],'degrees_per_unit':[0.24,0.24],'pan_pivot_m':[0,0,0],'tilt_pivot_m':[-.05,0,.32],
        'heading_window_deg':[90,60],'max_rms_px':0.01,
        'markers':{2:{'mount':'tilt','center_m':[-.14,0,.295]}}}}]}
    spec=robot_specs_from_config(cfg)[0]
    geometry=Geometry(CameraModel.from_fov(1600,900,102),FloorPose.look_at([0,0,.395],[0,3,0]))
    units=np.array([700.,500.])  # head panned 48 deg
    pixels=geometry.world_to_pixels(spec.head.corners(2,units)+[.2,1.6,0])
    tag=MarkerDetection(2,pixels+np.random.default_rng(0).normal(0,.5,pixels.shape))
    obs,=estimate_robots([tag],[spec],geometry,{1:units})
    assert obs.heading_deg is None and obs.source=='tag centre'  # position only; the robot waits for a heading
