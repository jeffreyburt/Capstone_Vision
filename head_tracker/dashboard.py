import json, time, threading, collections
from http.server import ThreadingHTTPServer, BaseHTTPRequestHandler
from pathlib import Path
import serial
import mimetypes
import math
import argparse
import socket
if __package__:
 from .tracking_control import validate as validate_tracking, correction, approach, TargetMemory, image_to_servo, servo_to_image
 from .camera_feed import CameraFeed, parse_size as parse_camera_size
else:
 from tracking_control import validate as validate_tracking, correction, approach, TargetMemory, image_to_servo, servo_to_image
 from camera_feed import CameraFeed, parse_size as parse_camera_size
camera_feed=None

def servo_port():
 """Servo bus: CAPSTONE_SERVO_PORT if set, else a USB BusLinker (CH340) if plugged in, else the Pi UART."""
 import os,glob
 if os.environ.get('CAPSTONE_SERVO_PORT'):return os.environ['CAPSTONE_SERVO_PORT']
 usb=sorted(glob.glob('/dev/serial/by-id/*1a86*')+glob.glob('/dev/serial/by-id/*CH340*'))
 return usb[0] if usb else '/dev/serial0'
SERIAL_PORT=servo_port()
feedback_socket=None
feedback_target=None
feedback_robot_id=None

lock=threading.Lock()
stop=threading.Event()
bus_lock=threading.Lock()
state={'running':False,'status':'Ready','positions':[None,None],'position_time':None,'position_error':None,'torque':[None,None],'controlling':False,'samples':[],'config':{'a':[10,961],'b':[200,771],'duration':8,'cycles':3,'pause':0.5}}
history=collections.deque(maxlen=1800)
tracking_observation=None
state['tracking']={'active':False,'status':'Off','center':None,'targets':None}
settings_path=Path(__file__).with_name('tracking-settings.json')
try:state['tracking']['config']=validate_tracking(json.loads(settings_path.read_text()))
except (OSError,ValueError,TypeError):pass

def send(p,s,c,args=()):
 b=[s,len(args)+3,c,*args]; p.write(bytes([85,85,*b,(~sum(b))&255]));p.flush()

def read(p,s,command=28):
 p.reset_input_buffer();send(p,s,command); data=bytearray();end=time.monotonic()+.12
 while time.monotonic()<end:
  data.extend(p.read(p.in_waiting or 1))
  length=8 if command==28 else 7
  for i in range(max(0,len(data)-length+1)):
   q=data[i:i+length]
   if q[:2]==b'\x55\x55' and q[2]==s and q[3:5]==bytes([length-3,command]) and sum(q[2:])%256==255:
    return int.from_bytes(q[5:-1],'little',signed=(command==28))
 return None

def publish_positions(values, error=None):
 with lock:
  state['positions']=values
  stamp=time.time()
  state['position_time']=stamp
  state['position_error']=error or ('No response from servo' if None in values else None)

 if feedback_socket is not None and all(type(v) is int and 0<=v<=1000 for v in values):
  packet=json.dumps({'robot_id':feedback_robot_id,'t':stamp,'units':values}).encode()
  try:feedback_socket.sendto(packet,feedback_target)
  except OSError:pass

def monitor():
 while True:
  with lock:running=state['running']
  if not running and bus_lock.acquire(blocking=False):
   try:
    with lock:running=state['running']
    if not running:
     try:
      with serial.Serial(SERIAL_PORT,115200,timeout=.01,exclusive=True) as p:
       publish_positions([read(p,s) for s in (1,2)])
       torque=[read(p,s,32) for s in (1,2)]
       with lock:state['torque']=torque
     except Exception as error:publish_positions([None,None],str(error))
   finally:bus_lock.release()
  time.sleep(.07 if feedback_socket is not None else .2)

def ease(f):
 f=max(0.0,min(1.0,f))
 return f*f*f*(10+f*(-15+6*f))

def trajectory(initial, targets, elapsed, duration):
 fraction=ease(elapsed/duration)
 return [a+(b-a)*fraction for a,b in zip(initial,targets)]

def sweep(config):
 bus_lock.acquire()
 p=None
 try:
  p=serial.Serial(SERIAL_PORT,115200,timeout=.01,exclusive=True)
  start=time.monotonic()
  actual=[read(p,s) for s in (1,2)]
  publish_positions(actual)
  if None in actual: raise RuntimeError('Missing servo feedback')
  if any(read(p,s,32)!=1 for s in (1,2)):
   raise RuntimeError('Engage both servos before starting a sweep')
  for cycle in range(config['cycles']):
   for label in ('a','b'):
    if stop.is_set():break
    targets=config[label];initial=actual[:]; duration=config['duration']
    with lock:state['status']=f"Cycle {cycle+1}/{config['cycles']} · Easing to {label.upper()} · {targets[0]} / {targets[1]}"
    t0=time.monotonic();bad=0;last_feedback=-1;last_sent=None
    # Small synchronized linear segments approximate a quintic S curve.
    # Zero velocity and acceleration at both endpoints reduce reversal shock.
    while True:
     if stop.is_set():break
     tick=time.monotonic();elapsed=tick-t0
     if elapsed<duration:
      horizon=min(duration,elapsed+0.08)
      commanded=[round(v) for v in trajectory(initial,targets,horizon,duration)]
      milliseconds=max(1,round((horizon-elapsed)*1000))
     else:
      commanded=targets;milliseconds=80
     if commanded!=last_sent:
      for sid,value in enumerate(commanded,1):
       send(p,sid,7,[value&255,value>>8,milliseconds&255,milliseconds>>8])
      send(p,254,11)
      last_sent=commanded[:]
     if elapsed-last_feedback>=0.16 or elapsed>=duration+1:
      actual=[read(p,sid) for sid in (1,2)]
      publish_positions(actual)
      elapsed=time.monotonic()-t0;last_feedback=elapsed
      expected=[round(v,1) for v in trajectory(initial,targets,elapsed,duration)]
      sample={'t':round(time.monotonic()-start,2),'actual':actual,'expected':expected,'target':targets}
      with lock:history.append(sample)
      bad=bad+1 if None in actual else 0
      if bad>=3:raise RuntimeError('Feedback lost; stopped')
      if all(v is not None for v in actual) and max(abs(a-b) for a,b in zip(actual,expected))>60:
       raise RuntimeError('Tracking error exceeded 60 units; stopped')
     if elapsed>=duration+1:break
     stop.wait(max(0,0.08-(time.monotonic()-tick)))
    if stop.is_set():break
    if any(a is None or abs(a-b)>15 for a,b in zip(actual,targets)):
     raise RuntimeError('Endpoint missed by more than 15 units; stopped')
    stop.wait(config.get('pause',0.5))
   if stop.is_set():break
  with lock:state['status']='Stopped' if stop.is_set() else 'Sweep completed'
 except Exception as e:
  with lock:state['status']=str(e)
 finally:
  if p:
   try:send(p,254,12)
   except Exception:pass
   finally:p.close()
  bus_lock.release()
  with lock:state['running']=False

def track_head(config):
 global tracking_observation
 p=None
 bus_lock.acquire()
 try:
  p=serial.Serial(SERIAL_PORT,115200,timeout=.01,exclusive=True)
  positions=[read(p,sid) for sid in (1,2)]
  publish_positions(positions)
  if any(v is None for v in positions):raise RuntimeError('Missing servo feedback')
  if any(read(p,sid,32)!=1 for sid in (1,2)):raise RuntimeError('Engage both servos before tracking')
  if any(not low<=v<=high for v,(low,high) in zip(positions,config['limits'])):
   raise RuntimeError('Current positions are outside the taught limits; recapture limits before tracking')
  targets=list(map(float,positions));velocities=[0.,0.];previous=time.monotonic();last_feedback=previous;last_sent=None;holding=False;misses=0;filtered=None;active_axes=[False,False];last_command=0
  # Remember the face in head (servo) coordinates so prediction allows for the head's own movement.
  memory=TargetMemory(max_speed=400,bounds=(0,1000));aim=collections.deque(maxlen=80)
  while not stop.is_set():
   now=time.monotonic();dt=now-previous;previous=now
   aim.append((time.time(),targets[:]))
   with lock:observation=tracking_observation
   fresh=(observation is not None and now-observation['received']<.6 and camera_feed.info()['live'])
   mode='holding';control_center=None;goal=None;age=None
   if fresh:
    if observation['center'] is not None:
     # Where the head was aimed when that frame was captured, not now: the goal is where the face IS in servo
     # units, which stays right however far the head has turned since.
     then=next((t for s,t in reversed(aim) if s<=observation['frameTime']),aim[0][1])
     goal=image_to_servo(observation['center'],then,config['signs'])
     memory.observe(goal,observation['frameTime'])
     control_center=observation['center'];mode='observed'
    else:
     predicted=memory.predict(time.time(),config.get('prediction',1.2))
     if predicted is not None:
      goal=predicted;control_center=servo_to_image(predicted,targets,config['signs']);mode='predicted'
     elif max(map(abs,velocities))>.1:goal=targets[:];control_center=[.5,.5];mode='braking'
    if memory.stamp is not None:age=max(0,time.time()-memory.stamp)
   live=goal is not None
   if not live:
    if not holding:
     send(p,254,12);holding=True;velocities=[0.,0.];last_sent=None;filtered=None;active_axes=[False,False]
    message='Prediction expired — holding last position' if fresh and memory.center is not None else 'Camera paused or no target history — holding position'
   else:
    if holding:
     positions=[read(p,sid) for sid in (1,2)]
     if None in positions:raise RuntimeError('Feedback unavailable; tracking stopped')
     targets=list(map(float,positions));holding=False
    if filtered is None or mode=='braking':filtered=goal[:]
    else:  # light smoothing of the goal (servo units) against detection jitter
     alpha=1-math.exp(-min(dt,.1)/.05)
     filtered=[old+alpha*(new-old) for old,new in zip(filtered,goal)]
    targets,velocities=approach(targets,velocities,filtered,config,dt,active_axes)
    commanded=[round(v) for v in targets]
    changed=last_sent is None or max(abs(a-b) for a,b in zip(commanded,last_sent))>=2
    if changed and now-last_command>=.025:
     for sid,value in enumerate(commanded,1):send(p,sid,7,[value&255,value>>8,35,0])
     send(p,254,11);last_sent=commanded;last_command=now
    errors=[(g-t)/u for g,t,u in zip(filtered,targets,(216.,171.))]  # as fractions of the image
    if all(abs(e)<=config['deadzone'] for e in errors):message='Head centered — holding'
    elif any(targets[i]<=config['limits'][i][0] or targets[i]>=config['limits'][i][1] for i in (0,1)):message='Taught limit reached'
    else:message='Following head'
    if mode=='predicted':message=f'Predicting offscreen head · {max(0,config.get("prediction",1.2)-age):.1f}s remaining'
    elif mode=='braking':message='Prediction expired — slowing to hold'
   if now-last_feedback>=(.07 if feedback_socket is not None else .18):
    positions=[read(p,sid) for sid in (1,2)];publish_positions(positions);last_feedback=time.monotonic()
    misses=misses+1 if None in positions else 0
    if misses>=2:raise RuntimeError('Servo feedback lost; tracking stopped')
    limit=max(45,.3*config['speed']) # the servo lags further behind its target at higher tracking speeds
    if live and None not in positions and any(abs(a-b)>limit for a,b in zip(positions,targets)):
     raise RuntimeError(f'Servo tracking error exceeded {limit:.0f} units; stopped')
    if holding and None not in positions:targets=list(map(float,positions))
   with lock:
    state['tracking'].update(status=message,mode=mode,center=control_center if live else None,lastSeenAge=round(age,2) if age is not None else None,targets=[round(v,1) for v in targets])
    state['status']='Head tracking · '+message
   stop.wait(max(0,.025-(time.monotonic()-now)))
 except Exception as error:
  with lock:state['status']=str(error);state['tracking']['status']=str(error)
 finally:
  if p:
   try:send(p,254,12)
   except Exception:pass
   finally:p.close()
  bus_lock.release()
  with lock:
   state['running']=False;state['tracking']['active']=False
   if stop.is_set():state['tracking']['status']='Stopped';state['status']='Stopped'

class Handler(BaseHTTPRequestHandler):
 def log_message(self,*args):pass
 def send_error(self,code,message=None,explain=None):
  # The status line must be latin-1; messages like '5–400' crashed the handler and dropped the connection.
  if message is not None:message=str(message).replace('\u2013','-').encode('latin-1','replace').decode('latin-1')
  super().send_error(code,message,explain)
 def do_GET(self):
  self.path=self.path.split('?',1)[0]
  if self.path=='/api/state':
   with lock:body=json.dumps({**state,'samples':list(history),'camera':camera_feed.info(),'serverTime':time.time(),'servoPort':SERIAL_PORT}).encode()
   kind='application/json'
  elif self.path.startswith('/assets/'):
   root=Path(__file__).with_name('assets').resolve()
   asset=(root/self.path[len('/assets/'):]).resolve()
   if not asset.is_relative_to(root) or not asset.is_file():self.send_error(404);return
   body=asset.read_bytes()
   kind={'.mjs':'text/javascript','.js':'text/javascript','.wasm':'application/wasm','.task':'application/octet-stream'}.get(asset.suffix,mimetypes.guess_type(str(asset))[0] or 'application/octet-stream')
  elif self.path=='/api/camera/frame':
   with camera_feed.condition:
    frame=camera_feed.frame
    frame_time=camera_feed.updated
    fresh=time.time()-camera_feed.updated<3
   if not frame or not fresh:self.send_error(503,'Camera unavailable');return
   body=frame;kind='image/jpeg'
  elif self.path=='/camera.mjpg':
   self.send_response(200);self.send_header('Content-Type','multipart/x-mixed-replace; boundary=FRAME');self.send_header('Cache-Control','no-store');self.end_headers()
   try:
    last=0
    while True:
     with camera_feed.condition:
      camera_feed.condition.wait_for(lambda:camera_feed.updated>last,timeout=5)
      frame=camera_feed.frame;stamp=camera_feed.updated
     if not frame or time.time()-stamp>3:break
     if stamp==last:continue
     last=stamp
     self.wfile.write(b'--FRAME\r\nContent-Type: image/jpeg\r\nContent-Length: '+str(len(frame)).encode()+b'\r\n\r\n'+frame+b'\r\n')
     self.wfile.flush()
   except (BrokenPipeError,ConnectionResetError,TimeoutError):pass
   self.close_connection=True
   return
  elif self.path in ('/','/index.html'):

   body=Path(__file__).with_name('index.html').read_bytes();kind='text/html'
  else:self.send_error(404);return
  self.send_response(200);self.send_header('Content-Type',kind);self.send_header('Cache-Control','no-store');
  if self.path=='/api/camera/frame':self.send_header('X-Frame-Time',str(frame_time))
  self.end_headers();self.wfile.write(body)
 def do_POST(self):
  global tracking_observation
  # Only same-origin browser controls are accepted.
  origin=self.headers.get('Origin')
  if origin and origin!='http://'+self.headers.get('Host',''):
   self.send_error(403);return
  if self.path=='/api/stop':stop.set()
  elif self.path in ('/api/tracking/start','/api/tracking/observation'):
   try:
    size=int(self.headers.get('Content-Length',0))
    if not 0<size<=2048:raise ValueError('Invalid request size')
    data=json.loads(self.rfile.read(size))
    if data.get('target')!='face':raise ValueError('Refresh the dashboard to use face tracking')
    if self.path.endswith('/start'):
     config=validate_tracking(data)
     with lock:
      if state['running'] or state['controlling']:self.send_error(409,'Stop current motion first');return
      if state['torque']!=[1,1]:self.send_error(409,'Engage both servos first');return
      if not camera_feed.info()['live']:self.send_error(409,'Camera must be live');return
      positions=state['positions']
      if time.time()-(state['position_time'] or 0)>1 or any(v is None for v in positions):raise ValueError('Fresh servo positions required')
      if any(not lo<=v<=hi for v,(lo,hi) in zip(positions,config['limits'])):raise ValueError('Current positions must be inside the taught limits')
      try:settings_path.write_text(json.dumps(config))
      except OSError as error:self.send_error(503,'Cannot save tracking settings: '+str(error));return
      tracking_observation=None;state['tracking']={'active':True,'status':'Waiting for face','center':None,'targets':positions,'config':config}
      state['running']=True;state['status']='Head tracking · Waiting for face';stop.clear()
     threading.Thread(target=track_head,args=(config,),daemon=True).start()
    else:
     center=data.get('center');stamp=data.get('frameTime')
     if center is not None and (not isinstance(center,list) or len(center)!=2 or any(type(v) not in (int,float) or not math.isfinite(v) or not 0<=v<=1 for v in center)):raise ValueError('Invalid face center')
     if type(stamp) not in (int,float) or not math.isfinite(stamp) or not 0<=time.time()-stamp<.7:raise ValueError('Camera observation is stale')
     with lock:
      if state['tracking']['active']:
       if tracking_observation is None or stamp>tracking_observation['frameTime']:
        tracking_observation={'center':center,'received':time.monotonic(),'frameTime':stamp}
   except (ValueError,TypeError,AttributeError) as error:self.send_error(400,str(error));return
  elif self.path=='/api/capture':
   with lock:
    if state['running'] or state['controlling']:
     self.send_error(409,'Stop motion before capturing');return
   if not bus_lock.acquire(timeout=1):
    self.send_error(503,'Servo interface busy');return
   try:
    with lock:
     if state['running'] or state['controlling']:
      self.send_error(409,'Stop motion before capturing');return
    with serial.Serial(SERIAL_PORT,115200,timeout=.01,exclusive=True) as p:
     positions=[read(p,s) for s in (1,2)]
    publish_positions(positions)
    if any(v is None or not 0<=v<=1000 for v in positions):
     self.send_error(503,'Valid feedback required from both servos');return
    body=json.dumps({'positions':positions}).encode()
    self.send_response(200);self.send_header('Content-Type','application/json');self.send_header('Cache-Control','no-store');self.end_headers();self.wfile.write(body)
    return
   except Exception as error:self.send_error(503,str(error));return
   finally:bus_lock.release()
  elif self.path in ('/api/engage','/api/disengage'):
   with lock:
    if state['controlling']:self.send_error(409,'Torque control in progress');return
    state['controlling']=True
   stop.set()
   acquired=False
   try:
    acquired=bus_lock.acquire(timeout=4)
    if not acquired:raise RuntimeError('Waiting for sweep to stop; try again')
    engage=self.path=='/api/engage'
    with serial.Serial(SERIAL_PORT,115200,timeout=.01,exclusive=True) as p:
     send(p,254,12)
     if engage:
      positions=[read(p,s) for s in (1,2)]
      if any(v is None or not 0<=v<=1000 for v in positions):
       raise RuntimeError('Cannot engage without valid current positions')
      for sid,value in enumerate(positions,1):
       send(p,sid,7,[value&255,value>>8,244,1])
      send(p,254,11)
     send(p,254,31,[1 if engage else 0])
     time.sleep(.05)
     torque=[read(p,s,32) for s in (1,2)]
     with lock:
      state['torque']=torque
      state['status']='Torque engaged' if engage else 'Torque disengaged — manual positioning'
     if torque!=[1 if engage else 0]*2:
      raise RuntimeError('Torque command sent, but both servos did not confirm it')
   except Exception as error:
    self.send_error(503,str(error));return
   finally:
    if acquired:bus_lock.release()
    with lock:state['controlling']=False
  elif self.path=='/api/start':
   try:
    size=int(self.headers.get('Content-Length',0))
    if not 0<size<=2048:raise ValueError('Missing or oversized configuration')
    config=json.loads(self.rfile.read(size))
    for label in ('a','b'):
     values=config.get(label)
     if not isinstance(values,list) or len(values)!=2 or any(type(v) is not int or not 0<=v<=1000 for v in values):
      raise ValueError('Each setpoint needs two integer positions from 0 to 1000')
    duration=config.get('duration');cycles=config.get('cycles')
    if type(duration) not in (int,float) or not 1<=duration<=30:raise ValueError('Move time must be 1–30 seconds')
    if type(cycles) is not int or not 1<=cycles<=100:raise ValueError('Cycles must be 1–100')
    pause=config.get('pause',0.5)
    if type(pause) not in (int,float) or not 0<=pause<=10:raise ValueError('Pause must be 0–10 seconds')
    config['pause']=pause
    if config['a']==config['b']:raise ValueError('Setpoints must differ')
   except (ValueError,TypeError) as error:
    self.send_error(400,str(error));return
   with lock:
    if state['running'] or state['controlling']:self.send_error(409);return
    if state['torque']!=[1,1]:self.send_error(409,'Engage both servos first');return
    state['config']=config
    state['running']=True;state['status']='Starting';history.clear();stop.clear()
   threading.Thread(target=sweep,args=(config,),daemon=True).start()
  else:self.send_error(404);return
  self.send_response(204);self.end_headers()

if __name__=='__main__':
 parser=argparse.ArgumentParser(description="Camera and servo dashboard")
 parser.add_argument('--pose-host',help='laptop IPv4 address for joint feedback')
 parser.add_argument('--pose-port',type=int,default=5006)
 parser.add_argument('--robot-id',type=int)
 parser.add_argument('--camera-size',default='320x240',
                     help='robot camera frame size, e.g. 320x240 (default; faster tracking) or 640x480')
 parser.add_argument('--camera-fps',type=int,default=30,
                     help='robot camera frame rate, 5-60; above 30 is untested (try with --camera-size 320x240)')
 args=parser.parse_args()
 try:camera_size=parse_camera_size(args.camera_size)
 except ValueError as error:parser.error(str(error))
 if not 5<=args.camera_fps<=60:parser.error('--camera-fps must be 5-60')
 if (args.pose_host is None)!=(args.robot_id is None):
  parser.error('--pose-host and --robot-id must be supplied together')
 if args.pose_host is not None:
  if not 1<=args.pose_port<=65535:
   parser.error('--pose-port must be in 1–65535')
  try:socket.inet_pton(socket.AF_INET,args.pose_host)
  except OSError:parser.error('--pose-host must be an IPv4 address')
  feedback_target=(args.pose_host,args.pose_port)
  feedback_robot_id=args.robot_id
  feedback_socket=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
  feedback_socket.setblocking(False)
 camera_feed=CameraFeed(camera_size,args.camera_fps)
 threading.Thread(target=monitor,daemon=True).start()
 print('Servo dashboard: http://0.0.0.0:8080 · servo bus '+SERIAL_PORT,flush=True)
 ThreadingHTTPServer(('0.0.0.0',8080),Handler).serve_forever()
