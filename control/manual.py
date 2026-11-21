"""Short-lived browser keyboard commands, consumed by the existing drive controller."""
import math
import threading
import time
import uuid

STOP=(0.,0.,0.)


class ManualControl:
    def __init__(self, robot_ids, clock=time.time):
        self.robot_ids=set(robot_ids)
        self.clock=clock
        self.lock=threading.RLock()
        self.data=None

    def disable(self, token=None):
        with self.lock:
            if token is not None and self.data and token!=self.data['token']:
                raise ValueError('This keyboard session no longer owns the controls')
            self.data=None
            return self.snapshot()

    def enable(self, robot_id):
        if type(robot_id) is not int or robot_id not in self.robot_ids:
            raise ValueError('Select a configured robot')
        with self.lock:
            self.data=dict(enabled=True,robot_id=robot_id,token=uuid.uuid4().hex,seq=-1,
                           keys=[],speed=.23,expires_at=self.clock()+.5)
            return self.snapshot()

    def command(self,data):
        with self.lock:
            current=self.snapshot()
            if not current['enabled'] or data.get('token')!=current['token']:
                raise ValueError('Enable keyboard control again')
            if data.get('robot_id')!=current['robot_id']:
                raise ValueError('Robot selection changed; enable keyboard control again')
            seq=data.get('seq');keys=data.get('keys');speed=data.get('speed')
            if type(seq) is not int or seq<=current['seq']:
                raise ValueError('Out-of-order keyboard command')
            if not isinstance(keys,list) or len(keys)>4 or any(k not in ('w','a','s','d') for k in keys):
                raise ValueError('Keys must be W, A, S, or D')
            if type(speed) not in (int,float) or not math.isfinite(speed) or not .02<=speed<=.53:
                raise ValueError('Keyboard speed must be 0.02–0.53 m/s')
            self.data.update(seq=seq,keys=sorted(set(keys)),speed=speed,expires_at=self.clock()+.4)
            return self.snapshot()

    def snapshot(self):
        with self.lock:
            if self.data and self.clock()>=self.data['expires_at']:
                self.data=None # Expired sessions cannot be revived by delayed requests.
            return {**self.data,'keys':list(self.data['keys'])} if self.data else {'enabled':False}


def manual_commands(navigation,controllers,now=None):
    """Return one selected body-frame velocity and stops for the other robots, or None."""
    manual=(navigation or {}).get('manual') or {}
    if not manual.get('enabled'):
        return None
    result={rid:(STOP,'keyboard: not selected') for rid in controllers}
    now=time.time() if now is None else now
    if not 0<manual.get('expires_at',0)-now<=.6:
        return {rid:(STOP,'keyboard command expired') for rid in controllers}
    rid=manual.get('robot_id')
    if rid not in controllers:return result
    keys=set(manual.get('keys',[]))
    forward=int('w' in keys)-int('s' in keys)
    left=int('a' in keys)-int('d' in keys)
    length=math.hypot(forward,left)
    limits=controllers[rid].limits
    cap=getattr(limits,'keyboard_max_speed_m_s',0.) or limits.max_speed_m_s
    speed=min(float(manual.get('speed',0)),cap,.53)
    command=(speed*forward/length,speed*left/length,0.) if length else STOP
    result[rid]=(command,'keyboard driving' if length else 'keyboard ready')
    return result
