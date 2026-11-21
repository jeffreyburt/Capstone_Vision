"""Exercise exclusive ownership, bounded requests, and output cleanup without hardware."""
import io
import json
import sys
from unittest.mock import Mock

import pytest
from robot import motor_diagnostic as diagnostic


@pytest.mark.parametrize('change', [{'seconds':float('nan')}, {'seconds':6}, {'speed':0},
    {'speed':True}, {'robot_id':3}, {'channel':'M1'}, {'robot_id':2,'channel':'TB6612'}])
def test_reject_invalid_motor_requests(change):
    with pytest.raises(ValueError):
        diagnostic.request(dict(robot_id=1,channel='TB6612',speed=25,seconds=3,**{}) | change)


def test_hiwonder_exclusive_cleanup_on_read_failure(monkeypatch, tmp_path):
    from types import SimpleNamespace
    events=[]
    board=Mock()
    board.config.max_speed_counts=10
    board.__enter__=Mock(return_value=board)
    board.__exit__=Mock(side_effect=lambda *a: events.append('close'))
    board.initialize.side_effect=lambda:events.append('initialize')
    board.read_encoder_counts.side_effect=[(0,0,0,0),(0,0,0,0),(0,0,0,0),OSError('encoder read failed')]
    board.set_motor_speed_counts.side_effect=lambda v:events.append(v)
    board.stop.side_effect=lambda:events.append('stop')
    module=SimpleNamespace(HiwonderController=lambda _:board,HiwonderConfig=type('Config',(),{
        '__dataclass_fields__':{},'__init__':lambda self,**kw:None}))
    monkeypatch.setitem(sys.modules,'capstone_base',SimpleNamespace())
    monkeypatch.setitem(sys.modules,'capstone_base.hiwonder_i2c',module)
    cfg=tmp_path/'config.json';cfg.write_text('{}')
    real_expand=diagnostic.os.path.expanduser
    monkeypatch.setattr(diagnostic.os.path,'expanduser',lambda p:str(cfg) if p.endswith('.json') else real_expand(p))
    real_open=open
    monkeypatch.setattr('builtins.open',lambda p,*a,**kw:real_open(tmp_path/'lock' if p=='/tmp/capstone-motor-diagnostic.lock' else p,*a,**kw))
    def run(argv,**kw):
        events.append(argv[-2:] if 'sudo' in argv else 'active')
        return SimpleNamespace(returncode=0)
    monkeypatch.setattr(diagnostic.subprocess,'run',run)
    monkeypatch.setattr(diagnostic.select,'select',lambda *a:([],[],[]))
    monkeypatch.setattr(sys,'argv',['test',json.dumps(dict(robot_id=2,channel='M3',speed=25,seconds=3))])
    monkeypatch.setattr(sys,'stdin',io.StringIO())
    with pytest.raises(OSError,match='encoder read failed'):
        diagnostic.main()
    assert events.index(['stop','capstone-drive-2.service']) < events.index('initialize')
    assert (0,0,2,0) in events
    assert events.index('stop') < events.index('close') < events.index(['start','capstone-drive-2.service'])


def test_dashboard_motor_endpoints_do_not_start_on_get(monkeypatch):
    import threading
    import urllib.request
    import urllib.error
    from monitor.server import SystemMonitor, CameraRelay, make_server
    from monitor.motors import MotorTests
    started=[]
    monkeypatch.setattr(MotorTests,'start',lambda self,data:started.append(data))
    monitor=SystemMonitor({},listener=False,start=False)
    server=make_server(monitor,CameraRelay({}),'127.0.0.1',0)
    thread=threading.Thread(target=server.serve_forever,daemon=True);thread.start()
    url=f'http://127.0.0.1:{server.server_port}'
    try:
        with urllib.request.urlopen(url+'/api/motors') as response:
            assert json.load(response)['running'] is False
        assert not started
        request=urllib.request.Request(url+'/api/motors/start',data=b'{}',
            headers={'Origin':'http://untrusted.example'},method='POST')
        with pytest.raises(urllib.error.HTTPError) as error:
            urllib.request.urlopen(request)
        assert error.value.code==403
        assert not started
    finally:
        server.shutdown();server.server_close();thread.join()
