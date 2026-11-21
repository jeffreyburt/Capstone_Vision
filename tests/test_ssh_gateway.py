import json
import subprocess
import sys
import time

from control.ssh_gateway import GatewayRobotLink,RELAY
from robot.drive_server import DriveParams,DriveServer,PrintMotors


def test_ssh_relay_bootstraps_with_stop_then_drives_and_collects_real_udp_reply():
    motors=PrintMotors()
    server=DriveServer(motors,DriveParams([1,2,3,4],[1,1,1,1]),1,host='127.0.0.1',port=0,log=lambda s:None)
    endpoints={1:('127.0.0.1',server.port)}
    def local_factory(command,**kwargs):
        return subprocess.Popen([sys.executable,'-u','-c',RELAY,json.dumps(endpoints)],**kwargs)
    link=GatewayRobotLink(endpoints,'unused',process_factory=local_factory)
    server.sock.settimeout(2)
    try:
        link.drive(1,.03,0,0,pose_compensated=True)
        packet,addr=server.sock.recvfrom(4096)
        assert json.loads(packet)['type']=='stop'  # no fresh gateway ACK yet
        server.handle(packet,addr,time.monotonic())
        end=time.monotonic()+2
        while link.poll()[1] is None and time.monotonic()<end:time.sleep(.01)
        assert link.status[1]['state'].startswith('stopped')
        link.drive(1,.03,0,0,pose_compensated=True)
        packet,addr=server.sock.recvfrom(4096)
        assert json.loads(packet)['type']=='drive'
        assert json.loads(packet)['pose_compensated'] is True
        server.handle(packet,addr,time.monotonic())
        server.tick(time.monotonic())
        assert any(motors.values)
        link.status_at[1]=time.time()-1
        link.drive(1,.03,0,0)
        packet,addr=server.sock.recvfrom(4096)
        # A status already buffered may refresh ACK freshness; drain before explicitly aging it.
        link.poll();link.status_at[1]=time.time()-1
        link.drive(1,.03,0,0)
        packet,addr=server.sock.recvfrom(4096)
        assert json.loads(packet)['type']=='stop'
    finally:
        link.close();server.sock.close()


def test_gateway_reconnects_after_ssh_exits_and_requires_fresh_ack():
    motors=PrintMotors()
    server=DriveServer(motors,DriveParams([1,2,3,4],[1,1,1,1]),1,host='127.0.0.1',port=0,log=lambda s:None)
    endpoints={1:('127.0.0.1',server.port)}
    started=[]
    def local_factory(command,**kwargs):
        process=subprocess.Popen([sys.executable,'-u','-c',RELAY,json.dumps(endpoints)],**kwargs)
        started.append(process)
        return process
    now=[1000.0]
    link=GatewayRobotLink(endpoints,'unused',process_factory=local_factory,clock=lambda:now[0])
    server.sock.settimeout(2)
    try:
        link.drive(1,.03,0,0)
        packet,addr=server.sock.recvfrom(4096)
        server.handle(packet,addr,time.monotonic())
        end=time.monotonic()+2
        while link.poll()[1] is None and time.monotonic()<end:time.sleep(.01)
        assert link.poll()[1] is not None
        started[0].kill();started[0].wait()  # the main Pi restarted
        now[0]+=.5
        link.poll()
        assert len(started)==1  # waits before reconnecting
        now[0]+=2
        assert link.poll()[1] is None and len(started)==2 and link.reconnects==1
        assert link.status[1]['state']=='reconnecting'
        link.drive(1,.03,0,0)
        packet,addr=server.sock.recvfrom(4096)
        assert json.loads(packet)['type']=='stop'  # no motion until the new connection is acknowledged
    finally:
        link.close();server.sock.close()
