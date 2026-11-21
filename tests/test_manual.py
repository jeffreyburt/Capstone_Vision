import math
from types import SimpleNamespace
import pytest
from control.manual import ManualControl,manual_commands


def controllers():
    return {1:SimpleNamespace(limits=SimpleNamespace(max_speed_m_s=.075)),
            2:SimpleNamespace(limits=SimpleNamespace(max_speed_m_s=.12))}


def test_diagonal_normalized_and_only_selected_robot_moves():
    session=ManualControl([1,2],clock=lambda:100)
    token=session.enable(1)['token']
    state=session.command(dict(token=token,robot_id=1,seq=1,keys=['w','a'],speed=.12))
    result=manual_commands({'manual':state},controllers(),now=100)
    command=result[1][0]
    assert command==pytest.approx((.075/math.sqrt(2),.075/math.sqrt(2),0))
    assert result[2][0]==(0,0,0)
    state=session.command(dict(token=token,robot_id=1,seq=2,keys=['w','s'],speed=.05))
    assert manual_commands({'manual':state},controllers(),now=100)[1][0]==(0,0,0)


def test_expiry_revokes_session_and_stale_packets_cannot_restart():
    now=[100.]
    session=ManualControl([1],clock=lambda:now[0])
    token=session.enable(1)['token']
    data=dict(token=token,robot_id=1,seq=1,keys=['w'],speed=.05)
    state=session.command(data)
    assert manual_commands({'manual':state},controllers(),now=101)[1][0]==(0,0,0)
    now[0]+=1
    assert session.snapshot()=={'enabled':False}
    with pytest.raises(ValueError):session.command({**data,'seq':2})


def test_switch_robot_invalidates_previous_token_and_rejects_out_of_order():
    session=ManualControl([1,2],clock=lambda:100)
    old=session.enable(1)['token']
    token=session.enable(2)['token']
    data=dict(token=token,robot_id=2,seq=2,keys=['d'],speed=.05)
    session.command(data)
    with pytest.raises(ValueError):session.command({**data,'seq':1,'keys':['w']})
    with pytest.raises(ValueError):session.disable(old)
    assert session.snapshot()['robot_id']==2
    session.disable(token)
    assert session.snapshot()=={'enabled':False}


@pytest.mark.parametrize('change',[{'speed':float('nan')},{'speed':True},{'speed':1},{'keys':['x']},{'robot_id':2}])
def test_invalid_keyboard_input(change):
    session=ManualControl([1,2],clock=lambda:100)
    token=session.enable(1)['token']
    with pytest.raises(ValueError):
        session.command(dict(token=token,robot_id=1,seq=1,keys=['w'],speed=.05)|change)


def test_http_keyboard_session_disarms_waypoints_and_blocks_conflicting_navigation():
    import json
    import threading
    import urllib.request
    import urllib.error
    from monitor.server import SystemMonitor,CameraRelay,make_server
    cfg={'control':{'robots':{1:{'host':'127.0.0.1'}}}}
    monitor=SystemMonitor(cfg,listener=False,start=False)
    monitor.waypoints.arm(True)
    server=make_server(monitor,CameraRelay(cfg),'127.0.0.1',0)
    thread=threading.Thread(target=server.serve_forever,daemon=True);thread.start()
    base=f'http://127.0.0.1:{server.server_port}'
    def post(path,data):
        request=urllib.request.Request(base+path,data=json.dumps(data).encode(),headers={'Content-Type':'application/json'})
        with urllib.request.urlopen(request) as response:return json.load(response)
    try:
        session=post('/api/manual/enable',{'robot_id':1})
        assert not monitor.waypoints.armed
        with pytest.raises(urllib.error.HTTPError) as error:
            post('/api/navigation/arm',{'enabled':True})
        assert error.value.code==400
        result=post('/api/manual/command',dict(robot_id=1,token=session['token'],seq=1,keys=['w','d'],speed=.05))
        assert result['keys']==['d','w']
        post('/api/manual/disable',{'token':session['token']})
        assert not monitor.manual.snapshot()['enabled']
        post('/api/navigation/arm',{'enabled':True})
        assert monitor.waypoints.armed
    finally:
        server.shutdown();server.server_close();thread.join()


def test_keyboard_limit_overrides_waypoint_limit_for_wasd_only():
    session=ManualControl([2],clock=lambda:100)
    token=session.enable(2)['token']
    state=session.command(dict(token=token,robot_id=2,seq=1,keys=['w'],speed=.5))
    ctrl={2:SimpleNamespace(limits=SimpleNamespace(max_speed_m_s=.23,keyboard_max_speed_m_s=.53))}
    assert manual_commands({'manual':state},ctrl,now=100)[2][0]==pytest.approx((.5,0,0))
    ctrl={2:SimpleNamespace(limits=SimpleNamespace(max_speed_m_s=.23,keyboard_max_speed_m_s=0.))}
    assert manual_commands({'manual':state},ctrl,now=100)[2][0]==pytest.approx((.23,0,0))
