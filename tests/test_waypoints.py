import pytest
import json
import threading
import urllib.request
import urllib.error

from control.controller import RobotController
from control.waypoints import WaypointStore, compute_waypoints
from monitor.server import CameraRelay, SystemMonitor, make_server
from monitor import assess
from monitor.server import Node


def test_store_validates_targets_and_clears_both_robots():
    store = WaypointStore([1, 2], {'x': [-2, 2], 'y': [0, 5]})
    store.set(1, -1, 2)
    store.set(2, 1, 3)
    assert store.snapshot() == {'1': {'x': -1., 'y': 2., 'preview_only': True, 'revision': 1}, '2': {'x': 1., 'y': 3., 'preview_only': True, 'revision': 2}}
    for args in [(3, 0, 1), (True, 0, 1), (1, float('nan'), 1), (1, 3, 1), (1, 0, -1)]:
        with pytest.raises(ValueError):
            store.set(*args)
    snapshot = store.snapshot()
    snapshot['1']['x'] = 100
    assert store.snapshot()['1']['x'] == -1
    store.clear()
    assert store.snapshot() == {}


def inputs():
    msg = {'robots': [{'id': 1, 'x': 0, 'y': 1, 'heading': 90, 'age': 0, 'pose_compensated': True},
                      {'id': 2, 'x': 1, 'y': 1, 'heading': 90, 'age': 0, 'pose_compensated': True}]}
    nav = {'armed': True, 'targets': {'1': {'x': 0, 'y': 3, 'preview_only': False}, '2': {'x': 1, 'y': 3, 'preview_only': False}},
           'readiness': {'1': {'ready': True}, '2': {'ready': True}}}
    return msg, nav, {1: RobotController(), 2: RobotController()}


def test_independent_point_targets_without_a_person():
    msg, nav, controllers = inputs()
    targets, commands = compute_waypoints(msg, 0, nav, controllers)
    assert set(targets) == {1, 2}
    assert commands[1][0][0] > 0 and commands[2][0][0] > 0
    nav['targets'].pop('1')
    _, commands = compute_waypoints(msg, 0, nav, controllers)
    assert commands[1][0] == (0, 0, 0)
    assert commands[2][0][0] > 0


@pytest.mark.parametrize('case', ['approximate', 'stale', 'no_heading', 'no_compensation', 'offline_service', 'unready'])
def test_navigation_stops_for_unusable_feedback(case):
    msg, nav, controllers = inputs()
    age = 0
    if case == 'approximate': msg['position_quality'] = 'approximate'
    if case == 'stale': age = 2
    if case == 'no_heading': msg['robots'][0]['heading'] = None
    if case == 'no_compensation': msg['robots'][0]['pose_compensated'] = False
    if case == 'offline_service': nav = None
    if case == 'unready': nav['readiness']['1'] = {'ready': False, 'reason': 'Drive server not responding'}
    _, commands = compute_waypoints(msg, age, nav, controllers)
    assert commands[1][0] == (0, 0, 0)


def test_approximate_preview_retains_target_without_heading_or_wheel_command():
    msg, nav, controllers = inputs()
    msg.update(position_quality='approximate', driving_allowed=False)
    msg['robots'][0]['heading'] = None
    targets, commands = compute_waypoints(msg, 0, nav, controllers)
    assert targets[1].heading_deg is None
    assert targets[1].y == 3
    assert all(command[0] == (0, 0, 0) for command in commands.values())


def test_dashboard_formats_position_only_target():
    node = Node('robot1', 'Robot 1', 'robot', 'robot', robot_id=1)
    control = {'state': 'running', 'mode': 'DRY RUN', 'robots': {
        '1': {'target': [0, 3, None], 'cmd': [0, 0, 0], 'reason': 'approximate positions'}}}
    result = assess.assess_robot(node, {}, {}, None, None, control, 0)
    assert 'heading unspecified' in next(c['text'] for c in result['checks'] if c['label'] == 'Control')


def test_preview_targets_do_not_become_drive_commands_when_calibration_recovers():
    msg, nav, controllers = inputs()
    nav['targets']['1']['preview_only'] = True
    _, commands = compute_waypoints(msg, 0, nav, controllers)
    assert commands[1][0] == (0, 0, 0)
    assert 'preview target only' in commands[1][1]


def test_http_targets_are_preview_only_and_reject_cross_origin_requests():
    cfg = {'area': {'x': [-2, 2], 'y': [0, 5]},
           'control': {'robots': {1: {'host': '127.0.0.1'}, 2: {'host': '127.0.0.2'}}}}
    monitor = SystemMonitor(cfg, listener=False, start=False)
    relay = CameraRelay(cfg)
    server = make_server(monitor, relay, '127.0.0.1', 0)
    threading.Thread(target=server.serve_forever, daemon=True).start()
    base = f'http://127.0.0.1:{server.server_port}'

    def post(path, data, origin=None):
        headers = {'Content-Type': 'application/json'}
        if origin: headers['Origin'] = origin
        request = urllib.request.Request(base + path, data=json.dumps(data).encode(), headers=headers, method='POST')
        with urllib.request.urlopen(request, timeout=2) as response:
            return json.load(response)

    try:
        state = post('/api/navigation/target', {'robot_id': 2, 'x': 1, 'y': 3})
        assert state['targets']['2'] == {'x': 1., 'y': 3., 'preview_only': True, 'revision': 1}
        assert not state['readiness']['2']['ready']
        assert state['armed'] is False
        state = post('/api/navigation/arm', {'enabled': True})
        assert state['armed'] is True and state['targets'] == {}
        state = post('/api/navigation/arm', {'enabled': False})
        assert state['armed'] is False and state['targets'] == {}
        with pytest.raises(urllib.error.HTTPError) as error:
            post('/api/navigation/target', {'robot_id': 2, 'x': 100, 'y': 3})
        assert error.value.code == 400
        with pytest.raises(urllib.error.HTTPError) as error:
            post('/api/navigation/clear', {}, 'https://untrusted.example')
        assert error.value.code == 403
        assert post('/api/navigation/clear', {})['targets'] == {}
        assert all(raw.get('drive') is None for raw in monitor.raw.values())
    finally:
        server.shutdown()
        server.server_close()
        monitor.close()
        relay.tunnels.close()


def test_feedback_reports_camera_error_and_marks_approximate_stale_data():
    from control.waypoints import waypoint_feedback
    msg, nav, _ = inputs()
    msg['position_quality'] = 'approximate'
    msg['robots'][0]['age'] = .4
    result = waypoint_feedback(msg, .1, nav['targets'], [1, 2])
    assert result['1']['actual'] == [0, 1]
    assert result['1']['target'] == [0, 3]
    assert result['1']['error'] == [0, 2]
    assert result['1']['distance_m'] == 2
    assert result['1']['tracking_age_s'] == pytest.approx(.5)
    assert not result['1']['fresh']
    assert result['1']['quality'] == 'approximate'
    assert waypoint_feedback(None, None, {}, [1])['1']['actual'] is None


def test_no_progress_latches_stop_until_a_new_click_even_at_same_coordinates():
    from control.waypoints import WaypointProgress
    msg, nav, controllers = inputs()
    guards = {rid: WaypointProgress() for rid in controllers}
    nav['targets']['1']['revision'] = 1
    _, commands = compute_waypoints(msg, 0, nav, controllers, progress=guards, now=0)
    assert commands[1][0] != (0, 0, 0)
    _, commands = compute_waypoints(msg, 0, nav, controllers, progress=guards, now=7)
    assert commands[1][0] == (0, 0, 0)
    assert 'no camera progress' in commands[1][1]
    msg['robots'][0]['y'] = 2
    _, commands = compute_waypoints(msg, 0, nav, controllers, progress=guards, now=8)
    assert commands[1][0] == (0, 0, 0)
    nav['targets']['1']['revision'] = 2
    _, commands = compute_waypoints(msg, 0, nav, controllers, progress=guards, now=9)
    assert commands[1][0] != (0, 0, 0)


def test_progress_timer_pauses_for_unusable_camera_and_turning():
    from control.waypoints import WaypointProgress
    guard = WaypointProgress()
    goal = {'x':0, 'y':3}
    assert guard.check(goal, 2, True, 0)
    assert guard.check(goal, 1.9, True, 5)
    assert guard.check(goal, 1.9, True, 10)
    assert guard.check(goal, 1.9, False, 15)
    assert guard.check(goal, 1.9, True, 20)
    msg, nav, controllers = inputs()
    guards = {rid: WaypointProgress() for rid in controllers}
    compute_waypoints(msg, 0, nav, controllers, progress=guards, now=0)
    compute_waypoints(None, 10, nav, controllers, progress=guards, now=20)
    _, commands = compute_waypoints(msg, 0, nav, controllers, progress=guards, now=30)
    assert commands[1][0] != (0,0,0)


def test_dashboard_disarm_stops_commands_and_old_goals_never_resume():
    msg, nav, controllers = inputs()
    nav['armed'] = False
    _, commands = compute_waypoints(msg, 0, nav, controllers)
    assert all(cmd[0] == (0,0,0) for cmd in commands.values())
    assert 'dashboard driving disabled' in commands[1][1]
    store=WaypointStore([1],{'x':[-2,2],'y':[0,5]})
    store.set(1,0,3,preview_only=False)
    store.arm(True)
    assert store.snapshot()=={}
    store.set(1,0,3,preview_only=False)
    store.arm(False)
    assert not store.armed and store.snapshot()=={}
    with pytest.raises(ValueError):store.arm('true')


def test_capture_age_stops_navigation_even_with_new_heartbeats(monkeypatch):
    msg, nav, controllers = inputs()
    msg.update(frame_clock='local',t=100.,max_frame_age_s=.65)
    monkeypatch.setattr('control.waypoints.time.time',lambda:100.8)
    _,commands=compute_waypoints(msg,0,nav,controllers)
    assert all(v[0]==(0,0,0) for v in commands.values())
    assert commands[1][1]=='camera capture too old'


def test_progress_fault_can_reset_after_dashboard_restart_at_same_target_revision():
    from control.waypoints import WaypointProgress
    guard=WaypointProgress()
    goal={'session':'first','revision':1,'x':0,'y':3}
    assert guard.check(goal,2,True,0)
    assert not guard.check(goal,2,True,7)
    assert guard.check({**goal,'session':'restarted'},2,True,8)


def test_target_can_wait_for_transient_camera_age_without_becoming_preview():
    cfg={'area':{'x':[-2,2],'y':[0,5]},'control':{'robots':{1:{'host':'127.0.0.1'}}}}
    monitor=SystemMonitor(cfg,listener=False,start=False)
    relay=CameraRelay(cfg);monitor.waypoints.arm(True)
    def snapshot():
        return {'navigation':{'armed':monitor.waypoints.armed,'targets':monitor.waypoints.snapshot(),
            'readiness':{'1':{'ready':False,'hardware_ready':True,'can_accept_target':True,
                              'reason':'Camera capture too old'}}}}
    monitor.snapshot=snapshot
    server=make_server(monitor,relay,'127.0.0.1',0)
    threading.Thread(target=server.serve_forever,daemon=True).start()
    try:
        request=urllib.request.Request(f'http://127.0.0.1:{server.server_port}/api/navigation/target',
            data=json.dumps({'robot_id':1,'x':0,'y':3}).encode(),
            headers={'Content-Type':'application/json'},method='POST')
        with urllib.request.urlopen(request,timeout=2) as response:nav=json.load(response)
        assert nav['targets']['1']['preview_only'] is False
        # Storing intent does not bypass the controller's own stale-capture stop.
        msg,_,controllers=inputs();msg.update(frame_clock='local',t=0,max_frame_age_s=.65)
        assert compute_waypoints(msg,0,nav,controllers)[1][1][0]==(0,0,0)
    finally:
        server.shutdown();server.server_close();monitor.close();relay.tunnels.close()


def test_feedback_age_includes_real_camera_delay(monkeypatch):
    from control.waypoints import waypoint_feedback
    msg,nav,_=inputs();msg.update(frame_clock='local',t=100.,max_frame_age_s=.65)
    monkeypatch.setattr('control.waypoints.time.time',lambda:100.5)
    feedback=waypoint_feedback(msg,.01,nav['targets'],[1])['1']
    assert feedback['tracking_age_s']==pytest.approx(.5)
    assert feedback['fresh']
    monkeypatch.setattr('control.waypoints.time.time',lambda:100.8)
    assert not waypoint_feedback(msg,.01,nav['targets'],[1])['1']['fresh']


def test_recent_observation_cannot_outlive_total_capture_plus_observation_budget(monkeypatch):
    msg,nav,controllers=inputs();msg.update(frame_clock='local',t=100.4,max_frame_age_s=.65)
    msg['robots'][0]['age']=.2
    monkeypatch.setattr('control.waypoints.time.time',lambda:100.9)
    _,commands=compute_waypoints(msg,0,nav,controllers)
    assert commands[1][0]==(0,0,0)  # .5 frame age + .2 observation age exceeds .65
    assert commands[2][0]!=(0,0,0)


def test_frames_between_updates_do_not_halt_motion(monkeypatch):
    """Frames arrive ~4 Hz with ~0.45 s lag: the newest frame ages past 0.65 s between updates. That is normal
    and must not stop the robot (it caused stop-start motion); real lag or a long gap still must."""
    msg, nav, controllers = inputs()
    msg.update(frame_clock='local', t=100., max_frame_age_s=.65)
    def moving(now, age):
        monkeypatch.setattr('control.waypoints.time.time', lambda: now)
        _, results = compute_waypoints(msg, age, nav, controllers)
        return results[2][0] != (0, 0, 0)
    assert moving(100.45, 0.0)          # fresh frame with 0.45 s lag
    assert moving(100.80, 0.35)         # same frame 0.35 s later: total 0.8 s, still current
    assert not moving(101.10, 0.65)     # over the 1.0 s hard limit
    assert not moving(100.80, 0.0)      # arrived 0.8 s late: real lag


def test_waypoint_move_holds_the_heading_present_at_the_click():
    from control.controller import Limits
    from control.waypoints import WaypointMotion
    ctrl = RobotController(Limits(allow_strafe=True, yaw_tolerance_deg=2.0))
    msg, nav, _ = inputs()
    nav['targets'] = {'2': {'x': 2.0, 'y': 1.0, 'preview_only': False, 'revision': 1}}
    nav['session'] = 's'
    motions = {2: WaypointMotion()}
    targets, _ = compute_waypoints(msg, 0, nav, {2: ctrl}, motions=motions, now=0.)
    assert targets[2].heading_deg == 90
    msg['robots'][1]['heading'] = 100            # the chassis rotates during the move
    targets, results = compute_waypoints(msg, 0, nav, {2: ctrl}, motions=motions, now=0.1)
    assert targets[2].heading_deg == 90          # still the heading from the click, not the drifted one
    assert results[2][0][2] < 0                  # yaw command turns back toward it


def test_heading_drift_stops_a_holonomic_move_until_a_new_destination():
    from control.controller import Limits
    from control.waypoints import WaypointMotion
    ctrl = RobotController(Limits(allow_strafe=True))
    msg, nav, _ = inputs()
    nav['targets'] = {'2': {'x': 2.0, 'y': 1.0, 'preview_only': False, 'revision': 1}}
    nav['session'] = 's'
    motions = {2: WaypointMotion()}
    compute_waypoints(msg, 0, nav, {2: ctrl}, motions=motions, now=0.)
    msg['robots'][1]['heading'] = 125            # 35 degrees away from the 90 held
    _, results = compute_waypoints(msg, 0, nav, {2: ctrl}, motions=motions, now=0.1)
    assert results[2][0] == (0., 0., 0.) and 'heading drifted' in results[2][1]
    msg['robots'][1]['heading'] = 90             # back in range: still latched
    assert compute_waypoints(msg, 0, nav, {2: ctrl}, motions=motions, now=0.2)[1][2][0] == (0., 0., 0.)
    nav['targets']['2']['revision'] = 2          # a new click clears it
    assert compute_waypoints(msg, 0, nav, {2: ctrl}, motions=motions, now=0.3)[1][2][0] != (0., 0., 0.)
