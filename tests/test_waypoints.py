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
    nav = {'targets': {'1': {'x': 0, 'y': 3, 'preview_only': False}, '2': {'x': 1, 'y': 3, 'preview_only': False}},
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
