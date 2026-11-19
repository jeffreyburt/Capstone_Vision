"""Rate-limited independent pan/tilt control in servo position units."""
import math

def validate(config):
 if not isinstance(config,dict):raise ValueError('Tracking configuration required')
 limits=config.get('limits')
 if isinstance(limits,list) and len(limits)==2:
  limits=[[0,1000],limits[1]]
 if not isinstance(limits,list) or len(limits)!=2:raise ValueError('Teach limits for both servos')
 for pair in limits:
  if not isinstance(pair,list) or len(pair)!=2 or any(type(v) is not int for v in pair) or not 0<=pair[0]<pair[1]<=1000:
   raise ValueError('Each servo needs two distinct limits within 0–1000')
 signs=config.get('signs')
 if not isinstance(signs,list) or len(signs)!=2 or any(type(v) is not int or v not in (-1,1) for v in signs):raise ValueError('Pan and tilt directions must be +1 or -1')
 speed=config.get('speed',180)
 if type(speed) not in (int,float) or not math.isfinite(speed) or not 5<=speed<=400:raise ValueError('Tracking speed must be 5–400 units per second')
 prediction=config.get('prediction',1.2)
 if type(prediction) not in (int,float) or not math.isfinite(prediction) or not 0<=prediction<=3:raise ValueError('Prediction time must be 0–3 seconds')
 return {'prediction':prediction,'limits':limits,'signs':signs,'speed':speed,'deadzone':0.06,'settlezone':0.04}

def correction(positions, velocities, center, config, dt, active=None):
 """Velocity ramp, proportional image error, and hard taught limits."""
 targets=[];speeds=[]
 if active is None:active=[False,False]
 dt=max(.001,min(.1,dt))
 for i,(position,velocity) in enumerate(zip(positions,velocities)):
  error=center[i]-.5
  inner=config.get('settlezone',0.04)
  if active[i] and abs(error)<inner:active[i]=False
  elif not active[i] and abs(error)>config['deadzone']:active[i]=True
  magnitude=max(0,abs(error)-inner)
  desired=(config['signs'][i]*config['speed']*math.copysign(min(1,magnitude/.35),error)) if active[i] else 0
  acceleration=config['speed']*4
  v=velocity+max(-acceleration*dt,min(acceleration*dt,desired-velocity))
  low,high=config['limits'][i]
  target=max(low,min(high,position+v*dt))
  if target==low and v<0 or target==high and v>0:v=0
  targets.append(target);speeds.append(v)
 return targets,speeds


class TargetMemory:
 """Bounded constant-velocity prediction from recent real camera observations."""
 def __init__(self):
  self.center=None
  self.stamp=None
  self.velocity=[0.,0.]
 def observe(self,center,stamp):
  if self.stamp is not None and stamp<=self.stamp:return
  if self.center is not None:
   dt=stamp-self.stamp
   if .02<=dt<=.7:
    measured=[max(-1.5,min(1.5,(a-b)/dt)) for a,b in zip(center,self.center)]
    self.velocity=[.4*old+.6*new for old,new in zip(self.velocity,measured)]
   else:self.velocity=[0.,0.]
  self.center=list(center);self.stamp=stamp
 def predict(self,now,horizon):
  if self.center is None or horizon<=0:return None
  elapsed=max(0.,now-self.stamp)
  if elapsed>horizon:return None
  return [max(-.5,min(1.5,p+v*elapsed)) for p,v in zip(self.center,self.velocity)]
