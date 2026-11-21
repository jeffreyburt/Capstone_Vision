"""Robot UDP link over one authenticated SSH connection to the main Pi.

The Mac needs no route to the robot subnet. The relay only targets configured IP/ports.
It drains queued input before forwarding the newest command per robot; stale feedback
replaces drive requests with stops. Robot-side watchdogs remain the final stop backstop.

If the SSH connection ends (for example the main Pi restarts), it is reopened automatically, at most once
every RECONNECT_S seconds. A new connection starts with no robot acknowledgements, so motion stays
replaced by stops until each robot answers again.
"""
import ipaddress
import json
import os
import shlex
import subprocess
import time

from .link import RobotLink

RECONNECT_S=2.0

RELAY = r'''
import sys,json,socket,select
robots={int(k):(v[0],int(v[1])) for k,v in json.loads(sys.argv[1]).items()}
sock=socket.socket(socket.AF_INET,socket.SOCK_DGRAM);sock.bind(('0.0.0.0',0));sock.setblocking(False)
buffer=b''
while True:
    readable,_,_=select.select([sys.stdin.buffer,sock],[],[],.1)
    if sys.stdin.buffer in readable:
        chunk=sys.stdin.buffer.read1(65536)
        if not chunk:break
        buffer+=chunk
        if len(buffer)>65536:break
        latest={}
        while b'\n' in buffer:
            line,buffer=buffer.split(b'\n',1)
            try:
                msg=json.loads(line);rid=msg['robot_id']
                if type(rid) is int and rid in robots:latest[rid]=msg
            except (ValueError,KeyError,TypeError):pass
        for rid,msg in latest.items():sock.sendto(json.dumps(msg).encode(),robots[rid])
    if sock in readable:
        while True:
            try:data,addr=sock.recvfrom(4096)
            except BlockingIOError:break
            try:
                msg=json.loads(data);rid=msg['robot_id']
                if type(rid) is not int or robots.get(rid)!=addr:continue
                print(json.dumps(msg),flush=True)
            except (ValueError,KeyError,TypeError):pass
sock.close()
'''


class GatewayRobotLink(RobotLink):
    def __init__(self, robots, gateway, *, process_factory=subprocess.Popen, clock=time.time):
        self.addrs={int(rid):(str(ipaddress.IPv4Address(host)),int(port)) for rid,(host,port) in robots.items()}
        if any(not 1<=p<=65535 for _,p in self.addrs.values()):
            raise ValueError('Invalid robot UDP port')
        if not gateway or gateway.startswith('-'):
            raise ValueError('Invalid SSH gateway')
        self.seq=0;self.status={};self.status_at={};self.clock=clock
        self.gateway,self.process_factory=gateway,process_factory
        self.reconnects=0
        self._start()

    def _start(self):
        self.buffer=b''
        self.status_at={}  # every robot must acknowledge over this connection before it may move
        self.started_at=self.clock()
        command='python3 -u -c '+shlex.quote(RELAY)+' '+shlex.quote(json.dumps(self.addrs))
        self.process=self.process_factory(['ssh','-T','-o','BatchMode=yes','-o','ConnectTimeout=3',
            '-o','ServerAliveInterval=5','-o','ServerAliveCountMax=2',self.gateway,command],
            stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=subprocess.DEVNULL,bufsize=0)
        os.set_blocking(self.process.stdin.fileno(),False)
        os.set_blocking(self.process.stdout.fileno(),False)

    def _reconnect_if_closed(self):
        if self.process.poll() is None or self.clock()-self.started_at<RECONNECT_S:
            return
        for pipe in (self.process.stdin,self.process.stdout):
            try:pipe.close()
            except OSError:pass
        self.reconnects+=1
        for rid in self.addrs:
            self.status[rid]={'state':'reconnecting','error':'SSH drive gateway closed; reconnecting'}
        self._start()

    def _send(self,rid,msg):
        ages=self.poll()
        # New or stalled SSH sessions must receive a stop acknowledgement before motion.
        if msg.get('type') in ('drive','manual') and (ages.get(rid) is None or ages[rid]>.3):
            msg={'type':'stop'}
        self.seq+=1
        packet=(json.dumps({'robot_id':rid,'seq':self.seq,**msg})+'\n').encode()
        try:
            if self.process.poll() is not None:raise OSError('SSH drive gateway exited')
            if os.write(self.process.stdin.fileno(),packet)!=len(packet):
                self.process.terminate()
                raise OSError('Partial gateway command; connection closed')
        except OSError as error:
            self.status[rid]={'state':'send failed','error':str(error)}

    def poll(self):
        self._reconnect_if_closed()
        while True:
            try:chunk=os.read(self.process.stdout.fileno(),65536)
            except (BlockingIOError,OSError):break
            if not chunk:break
            self.buffer+=chunk
            if len(self.buffer)>65536:
                self.buffer=b'';break
            while b'\n' in self.buffer:
                line,self.buffer=self.buffer.split(b'\n',1)
                try:
                    msg=json.loads(line);rid=msg['robot_id']
                    if type(rid) is int and rid in self.addrs:
                        self.status[rid]=msg;self.status_at[rid]=self.clock()
                except (ValueError,KeyError,TypeError):pass
        now=self.clock()
        return {rid:now-self.status_at[rid] if rid in self.status_at else None for rid in self.addrs}

    def close(self):
        try:self.process.stdin.close()
        except OSError:pass
        try:self.process.wait(timeout=1)
        except subprocess.TimeoutExpired:
            self.process.terminate()
            try:self.process.wait(timeout=1)
            except subprocess.TimeoutExpired:self.process.kill();self.process.wait()
        self.process.stdout.close()
