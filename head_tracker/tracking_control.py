"""Rate-limited independent pan/tilt control in servo position units."""
import math

FULL_SPEED_ERROR=.2  # image-fraction error (beyond the settle zone) that commands full speed; was .35
ACCEL_PER_SPEED=8    # acceleration = speed * this, per second; was 4

def validate(config):
 if not isinstance(config,dict):raise ValueError('Tracking configuration required')
 limits=config.get('limits')
 if isinstance(limits,list) and len(limits)==2 and not isinstance(limits[0],list):
  limits=[[0,1000],limits[1]]  # legacy: tilt pair only, pan uses its full range
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
 units=config.get('units_per_frame',list(UNITS_PER_FRAME))
 if not isinstance(units,(list,tuple)) or len(units)!=2 or any(type(v) not in (int,float) or not 50<=v<=600 for v in units):
  raise ValueError('units_per_frame must be [width, height] servo units, 50-600')
 search=config.get('search_up_units',100)
 if type(search) not in (int,float) or not math.isfinite(search) or not 0<=search<=400:raise ValueError('search_up_units must be 0-400')
 offset=config.get('aim_offset_m',0.0);face=config.get('face_width_m',0.15)
 if type(offset) not in (int,float) or not math.isfinite(offset) or not -0.5<=offset<=0.5:raise ValueError('aim_offset_m must be -0.5 to 0.5 m')
 if type(face) not in (int,float) or not math.isfinite(face) or not 0.08<=face<=0.3:raise ValueError('face_width_m must be 0.08-0.3 m')
 return {'prediction':prediction,'limits':limits,'signs':signs,'speed':speed,'deadzone':0.06,'settlezone':0.04,
         'units_per_frame':[float(v) for v in units],'aim_offset_m':float(offset),'face_width_m':float(face),
         'search_up_units':float(search)}


def search_goal(positions,config):
 """Where to look before any face has been seen: the same pan, tilted up by search_up_units (a standing person's
 face is above a speaker at floor level). 'Up' is the opposite of the tilt tracking direction: a face above the
 image centre (negative error) moves the tilt by -sign."""
 up=-config['signs'][1]
 (pl,ph),(tl,th)=config['limits']
 return [max(pl,min(ph,positions[0])),max(tl,min(th,positions[1]+up*config.get('search_up_units',100)))]


def aim_offset_fraction(face_size,config):
 """How far below the image centre the face should sit so a speaker `aim_offset_m` below the camera points at it.

 face_size: the face box width as a fraction of the image width. The face subtends face_width_m and the speaker
 offset subtends aim_offset_m at the same distance, so the offset angle is the face's angular width times their
 ratio; no distance or degrees-per-unit needed. Returned as a fraction of the image height (positive = lower)."""
 if not face_size or not config.get('aim_offset_m'):return 0.0
 w,h=config.get('units_per_frame',UNITS_PER_FRAME)
 units=face_size*w*config['aim_offset_m']/config.get('face_width_m',0.15)
 return max(-.45,min(.45,units/h))  # up to ~0.4 m away; closer faces leave the frame anyway

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
  desired=(config['signs'][i]*config['speed']*math.copysign(min(1,magnitude/FULL_SPEED_ERROR),error)) if active[i] else 0
  acceleration=config['speed']*ACCEL_PER_SPEED
  v=velocity+max(-acceleration*dt,min(acceleration*dt,desired-velocity))
  low,high=config['limits'][i]
  target=max(low,min(high,position+v*dt))
  if target==low and v<0 or target==high and v>0:v=0
  targets.append(target);speeds.append(v)
 return targets,speeds


# Servo units spanned by the full image width / height. Measured 2026-11-22 on both robots by moving each joint
# 40 units and tracking the image shift (Robot 1 168 x 116, Robot 2 171 x 117; the earlier 216 x 171 estimate made
# every correction and prediction overshoot by 30-50%). Each robot can override it: tracking config 'units_per_frame'.
UNITS_PER_FRAME=(170.,117.)


def image_to_servo(center,positions,signs,units=None):
 units=units or UNITS_PER_FRAME
 """Servo positions that would center this image point, i.e. where the face is in head coordinates."""
 return [p+s*(c-.5)*u for c,p,s,u in zip(center,positions,signs,units)]


def servo_to_image(target,positions,signs,units=None):
 units=units or UNITS_PER_FRAME
 """Image point where a head-coordinate target appears from the current servo positions."""
 return [max(-.5,min(1.5,.5+(t-p)/(s*u))) for t,p,s,u in zip(target,positions,signs,units)]


APPROACH_GAIN=10.0  # 1/s: speed = gain x remaining servo-unit error, capped at the tracking speed (sim: no overshoot up to 16)


def approach(positions, velocities, goal, config, dt, active=None, units=None):
 """Move each axis toward a goal in servo units (where the face is), slowing as it gets close.

 The goal comes from the newest detection mapped through where the head pointed when that frame was taken, so it
 does not go stale while the head turns: unlike chasing an image-space error, the head cannot keep accelerating
 on an old frame and overshoot. Same dead zone/hysteresis (as fractions of the image), acceleration limit and
 hard limits as correction()."""
 units=units or UNITS_PER_FRAME
 if active is None:active=[False,False]
 dt=max(.001,min(.1,dt))
 targets=[];speeds=[]
 for i,(position,velocity) in enumerate(zip(positions,velocities)):
  error=goal[i]-position
  inner=config.get('settlezone',.04)*units[i];outer=config['deadzone']*units[i]
  if active[i] and abs(error)<inner:active[i]=False
  elif not active[i] and abs(error)>outer:active[i]=True
  desired=math.copysign(min(config['speed'],APPROACH_GAIN*max(0,abs(error)-inner)),error) if active[i] else 0
  acceleration=config['speed']*ACCEL_PER_SPEED
  v=velocity+max(-acceleration*dt,min(acceleration*dt,desired-velocity))
  if v*error<0 and active[i]:v=0  # never drive away from the goal
  if abs(v*dt)>abs(error) and active[i]:v=error/dt  # land on the goal instead of stepping past it
  low,high=config['limits'][i]
  target=max(low,min(high,position+v*dt))
  if target==low and v<0 or target==high and v>0:v=0
  targets.append(target);speeds.append(v)
 return targets,speeds


class TargetMemory:
 """Bounded constant-velocity prediction from recent real camera observations.

 Default units are image fractions. The head tracker uses servo units (limit 400 units/s, bounds 0-1000) so a
 prediction accounts for the head having turned since the face was last seen."""
 def __init__(self,max_speed=1.5,bounds=(-.5,1.5),window_s=None,travel_s=None):
  """window_s: estimate velocity across this much recent history instead of the last two frames (detection jitter
  between frames 33 ms apart looks like fast motion). travel_s: a prediction moves for at most this long, then
  holds the predicted spot until the horizon expires (a long extrapolation turns the head away from the person)."""
  self.max_speed,self.bounds,self.window_s,self.travel_s=max_speed,bounds,window_s,travel_s
  self.recent=[]
  self.center=None
  self.stamp=None
  self.velocity=[0.,0.]
 def observe(self,center,stamp):
  if self.stamp is not None and stamp<=self.stamp:return
  if self.center is not None:
   dt=stamp-self.stamp
   if .02<=dt<=.7:
    ref,ref_t=self.center,self.stamp
    if self.window_s:  # oldest sample still inside the window: a longer baseline averages out jitter
     older=[(c,t) for c,t in self.recent if stamp-t<=self.window_s]
     if older:ref,ref_t=older[0]
    span=max(stamp-ref_t,dt)
    measured=[max(-self.max_speed,min(self.max_speed,(a-b)/span)) for a,b in zip(center,ref)]
    self.velocity=[.4*old+.6*new for old,new in zip(self.velocity,measured)]
   else:self.velocity=[0.,0.];self.recent=[]
  self.center=list(center);self.stamp=stamp
  self.recent=[(c,t) for c,t in self.recent if stamp-t<=(self.window_s or 0)]+[(list(center),stamp)]
 def predict(self,now,horizon):
  if self.center is None or horizon<=0:return None
  elapsed=max(0.,now-self.stamp)
  if elapsed>horizon:return None
  if self.travel_s is not None:elapsed=min(elapsed,self.travel_s)
  return [max(self.bounds[0],min(self.bounds[1],p+v*elapsed)) for p,v in zip(self.center,self.velocity)]
