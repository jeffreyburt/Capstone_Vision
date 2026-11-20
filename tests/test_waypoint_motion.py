import math
import numpy as np
import pytest

from control.controller import Limits,RobotController
from control.planner import Target
from control.waypoints import WaypointMotion,compute_waypoints
from vision.joint_pose import DashboardJointFeedback


def test_continuous_acceleration_and_soft_arrival_without_parking_corrections():
    motion=WaypointMotion()
    pose={'x':0.,'y':0.,'heading':0.}
    target=Target(1.,0.,0.)
    ctrl=RobotController()
    motion.activate({'x':1,'y':0,'revision':1},'session',ctrl)
    previous=0
    for i in range(15):
        cmd=motion.command((.12,0,0),pose,target,.1,i*.1)
        assert 0 < cmd[0]-previous <= .012+1e-9
        previous=cmd[0]
        if previous >= .119:break
    assert previous>.1
    # Arrive once, then allow normal gentle braking instead of an immediate stop.
    pose['x']=.95
    first=motion.command((.12,0,.3),pose,target,.1,1.6)
    assert motion.arrived and 0 < first[0] < previous
    for i in range(40):
        pose['x']=.88 if i%2 else .94  # camera noise crosses the arrival threshold
        cmd=motion.command((.12,0,.3),pose,target,.1,1.7+i*.1)
    assert cmd==(0,0,0)  # no repeated turn/drive bursts after arrival
    motion.halt()
    assert motion.arrived  # temporary tracking loss cannot forget completion
    motion.activate({'x':2,'y':0,'revision':2},'session',ctrl)
    assert not motion.arrived


def test_emergency_halt_is_immediate_and_filtered_motion_cannot_enter_keepout():
    motion=WaypointMotion()
    pose={'x':0.,'y':0.,'heading':0.};goal=Target(2.,0.,0.)
    for i in range(20):motion.command((.12,0,0),pose,goal,.1,i*.1)
    cmd=motion.command((0,0,0),pose,goal,.1,2,obstacles=[(.2,0,.8)])
    assert cmd[0]==0
    motion.halt()
    assert motion.current==[0,0,0]


def test_fresh_controller_pose_not_blocked_by_older_dashboard_vision_snapshot():
    pose={'id':1,'x':0,'y':0,'heading':0,'age':0,'pose_compensated':True}
    msg={'robots':[pose]}
    nav={'armed':True,'targets':{'1':{'x':2,'y':0,'preview_only':False}},
         'readiness':{'1':{'ready':False,'hardware_ready':True,'reason':'Camera capture too old'}}}
    _,out=compute_waypoints(msg,0,nav,{1:RobotController()})
    assert out[1][0][0]>0
    nav['readiness']['1']['hardware_ready']=False
    assert compute_waypoints(msg,0,nav,{1:RobotController()})[1][1][0]==(0,0,0)


def test_arrival_latch_survives_feedback_gaps_in_full_waypoint_loop():
    ctrl=RobotController(Limits(position_tolerance_m=.1))
    motion=WaypointMotion();motions={1:motion}
    nav={'armed':True,'targets':{'1':{'x':0,'y':1,'preview_only':False,'revision':1}},
         'readiness':{'1':{'ready':True}}}
    pose={'id':1,'x':0,'y':.95,'heading':90,'age':0,'pose_compensated':True}
    compute_waypoints({'robots':[pose]},0,nav,{1:ctrl},motions=motions,now=0)
    assert motion.arrived
    compute_waypoints(None,2,nav,{1:ctrl},motions=motions,now=1)
    pose['y']=.87
    _,out=compute_waypoints({'robots':[pose]},0,nav,{1:ctrl},motions=motions,now=2)
    assert out[1][0]==(0,0,0)


def test_steering_hysteresis_does_not_toggle_forward_reverse_or_alignment_on_noise():
    ctrl=RobotController(Limits(allow_strafe=False,drive_heading_deadband_deg=5,
                              drive_alignment_release_deg=45,reverse_hysteresis_deg=20))
    goal=Target(2,0,0)
    pose={'x':0.,'y':0.,'heading':24.}
    assert ctrl.command(pose,goal)[0]>0
    pose['heading']=27.
    assert ctrl.command(pose,goal)[0]>0  # used to hard-stop at 25 degrees
    ctrl.reset()
    for heading in (89,91,88,92):
        pose['heading']=heading;ctrl.command(pose,goal)
        assert ctrl.travel_reverse is False
    ctrl.reset();pose['heading']=3.
    assert ctrl.command(pose,goal)[2]==0


def test_joint_samples_interpolate_only_between_close_measured_readings():
    samples=[(100.,np.array([390,770])),(100.28,np.array([418,742]))]
    assert DashboardJointFeedback.interpolate(samples,100.14)==pytest.approx([404,756])
    assert DashboardJointFeedback.interpolate(samples,100.5) is None
    distant=[(100.,np.array([390,770])),(101.,np.array([418,742]))]
    assert DashboardJointFeedback.interpolate(distant,100.5) is None


def test_noisy_delayed_four_fps_feedback_produces_one_continuous_approach(monkeypatch):
    import control.waypoints as module
    rng=np.random.default_rng(7)
    clock=[100.]
    monkeypatch.setattr(module.time,'time',lambda:clock[0])
    limits=Limits(max_speed_m_s=.12,max_yaw_rad_s=.3,position_gain=.6,
                  position_tolerance_m=.1,allow_strafe=False,drive_heading_deadband_deg=5,
                  drive_alignment_release_deg=45,reverse_hysteresis_deg=20)
    ctrl=RobotController(limits);motion=WaypointMotion()
    nav={'armed':True,'session':'test','targets':{'1':{'x':1.,'y':0.,'revision':1,'preview_only':False}},
         'readiness':{'1':{'ready':False,'hardware_ready':True,'reason':'older dashboard snapshot'}}}
    x=y=heading=0.;history=[];message=None;received=100.;next_frame=0.;moving=[]
    for step in range(300):
        elapsed=step*.1;clock[0]=100+elapsed
        history.append((elapsed,x,y,heading))
        if elapsed >= next_frame:
            captured=max(0.,elapsed-.3)
            row=min(history,key=lambda h:abs(h[0]-captured))
            pose={'id':1,'x':row[1]+rng.normal(0,.02),'y':row[2]+rng.normal(0,.02),
                  'heading':math.degrees(row[3])+rng.normal(0,2.),'age':0,'pose_compensated':True}
            message={'robots':[pose],'frame_clock':'local','t':100+row[0],'max_frame_age_s':.65}
            received=clock[0];next_frame+=.25
        _,results=compute_waypoints(message,clock[0]-received,nav,{1:ctrl},motions={1:motion},now=elapsed)
        vx,vy,wz=results[1][0]
        moving.append(max(abs(vx),abs(vy),abs(wz))>.001)
        x+=(math.cos(heading)*vx-math.sin(heading)*vy)*.1
        y+=(math.sin(heading)*vx+math.cos(heading)*vy)*.1
        heading+=wz*.1
    runs=sum(active and (i==0 or not moving[i-1]) for i,active in enumerate(moving))
    assert runs==1
    assert motion.arrived
    assert math.hypot(x-1,y)<.15
    assert not any(moving[-30:])
