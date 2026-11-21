"""Robot 1: logical M1 is calibrated TB6612; physical Hiwonder M1 always stays zero."""
from collections import deque
from contextlib import ExitStack
import math
import time

try:
    from .motor_speed import PrecisionPWM, SpeedMatch
except ImportError:
    from motor_speed import PrecisionPWM, SpeedMatch


class TB6612Motor:
    def __init__(self, profile, pins, *, clock=time.monotonic):
        required={'pwm','standby','in1','in2','encoder_a','encoder_b'}
        if set(pins)!=required or any(type(v) is not int or not 0<=v<=27 for v in pins.values()) or len(set(pins.values()))!=6:
            raise ValueError('TB6612 needs six distinct BCM GPIO pins')
        self.profile=profile
        self.clock=clock
        self.stack=ExitStack()
        self.matcher=None
        self.command=0
        self.edges=[0,0]
        self.history=deque()
        self.rate=None
        self.started=self.previous=clock()
        self.last_edge_time=clock()
        self.last_edges=0
        self.phase_baseline=[0,0]
        try:
            from gpiozero import DigitalOutputDevice,DigitalInputDevice
            from gpiozero.pins.lgpio import LGPIOFactory
            factory=self.stack.enter_context(LGPIOFactory())
            self.pwm=self.stack.enter_context(PrecisionPWM(pins['pwm']))
            def device(cls,name,**kwargs):
                return self.stack.enter_context(cls(pins[name],pin_factory=factory,**kwargs))
            self.standby=device(DigitalOutputDevice,'standby',initial_value=False)
            self.in1=device(DigitalOutputDevice,'in1',initial_value=False)
            self.in2=device(DigitalOutputDevice,'in2',initial_value=False)
            self.a=device(DigitalInputDevice,'encoder_a',pull_up=True)
            self.b=device(DigitalInputDevice,'encoder_b',pull_up=True)
            def edge_a():self.edges[0]+=1
            def edge_b():self.edges[1]+=1
            self.a.when_activated=self.a.when_deactivated=edge_a
            self.b.when_activated=self.b.when_deactivated=edge_b
            self.stack.callback(self.stop)
        except BaseException:
            self.stack.close()
            raise

    def set(self, command):
        now=self.clock()
        if not command:
            self.stop()
            return
        matcher=SpeedMatch(self.profile,command) # Validate before enabling any output.
        reversing=self.command*command<0
        if reversing:
            self.stop()
            self.command=command
            self.matcher=matcher
            self.started=now+.1 # Zero PWM before changing H-bridge direction.
            self.previous=now
            self.history.clear()
            self.last_edges=sum(self.edges)
            self.phase_baseline=self.edges.copy()
            self.last_edge_time=now
            return
        if self.command==0:
            self.command=command;self.matcher=matcher
            self.started=self.previous=now
            self.history.clear();self.last_edges=sum(self.edges);self.last_edge_time=now
            self.phase_baseline=self.edges.copy()
        elif command!=self.command:
            matcher.integral=self.matcher.integral # Preserve correction across ramped speed changes.
            self.matcher=matcher;self.command=command
        elapsed=now-self.started
        if elapsed<0:return
        self.in1.value=command>0;self.in2.value=command<0
        self.standby.on()
        total=sum(self.edges)
        if total!=self.last_edges:
            self.last_edges=total;self.last_edge_time=now
        phase_counts=[v-z for v,z in zip(self.edges,self.phase_baseline)]
        if elapsed>=.75 and sum(phase_counts)>=40 and min(phase_counts)<.7*max(phase_counts):
            raise RuntimeError('TB6612 encoder phases disagree; all wheels stopped')
        if elapsed>=1 and now-self.last_edge_time>.75:
            raise RuntimeError('TB6612 encoder pulses stopped; all wheels stopped')
        self.history.append((now,total))
        while len(self.history)>2 and now-self.history[1][0]>=.3:self.history.popleft()
        if elapsed<.2:
            duty=max(self.matcher.feedforward,min(.08,self.matcher.maximum))
        elif elapsed<.5:
            duty=self.matcher.feedforward
        else:
            first=self.history[0]
            self.rate=(total-first[1])/max(now-first[0],.001)
            duty=self.matcher.update(self.rate,min(max(now-self.previous,0),.1))
        self.pwm.value=duty
        self.previous=now

    def stop(self):
        try:self.pwm.off()
        finally:
            self.standby.off();self.in1.off();self.in2.off()
        self.command=0;self.matcher=None;self.rate=None;self.history.clear()

    def telemetry(self):
        return dict(command=self.command,edges=self.edges.copy(),measured_count_rate=self.rate,
                    target_count_rate=self.matcher.target if self.matcher else 0,
                    duty_percent=self.pwm.value*100)

    def close(self):self.stack.close()


class MixedMotors:
    """Single control-loop owner for both motor drivers, sharing one calibrated count unit."""
    def __init__(self,board,tb,profile,*,direction=1):
        if direction not in (-1,1):raise ValueError('TB6612 direction must be +1 or -1')
        self.board,self.tb,self.profile,self.direction=board,tb,profile,direction
        scale=profile['reference_counts_per_command']
        self.minimum=math.ceil(profile['minimum_count_rate']/scale-1e-9)
        self.maximum=math.floor(profile['maximum_count_rate']/scale+1e-9)
        self.last_values=[0]*4
        self.guard=None
        self.fault=None
        self.fault_at=None

    def stop(self):
        try:self.tb.stop()
        finally:self.board.stop()
        self.last_values=[0]*4

    def set_motor_speed_counts(self,values):
        values=list(values)
        if len(values)!=4 or any(type(v) is not int or abs(v)>self.maximum for v in values):
            self.stop();raise ValueError('Mixed wheel command exceeds calibrated speed range')
        if self.fault:
            self.stop();return
        if values[0] and abs(values[0])<self.minimum:
            self.guard='TB6612 below calibrated speed; whole chassis held stationary'
            self.stop()
            return
        self.guard=None
        try:
            # Physical Hiwonder M1 is NEVER used, even though logical M1 is now restored.
            self.tb.set(values[0]*self.direction)
            self.board.set_motor_speed_counts((0,*values[1:]))
            self.last_values=values
        except Exception as error:
            self.fault=str(error);self.fault_at=time.time()
            self.stop()

    def telemetry(self):
        return dict(backend='hiwonder+tb6612',hiwonder_commands=[0,*self.last_values[1:]],
                    tb6612=self.tb.telemetry(),guard=self.guard,fault=self.fault,
                    minimum_counts=self.minimum,maximum_counts=self.maximum,
                    direction=self.direction)
