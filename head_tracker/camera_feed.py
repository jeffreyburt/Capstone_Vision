"""Shared live camera frames; no recording or motion control.

Frame size matters for tracking rate: every frame is JPEG-encoded on the Pi, sent over WiFi and decoded and
redrawn in the browser, while the face detector (BlazeFace short range) shrinks its input to 128x128 anyway.
320x240 therefore keeps detection quality and cuts the per-frame cost to about a quarter of 640x480.
"""
import io
import subprocess
import threading
import time
from pathlib import Path

DEFAULT_SIZE=(320,240)


def parse_size(text):
 """'320x240' -> (320, 240). Even sizes from 160x120 to 1280x960 (the JPEG encoder wants even sizes)."""
 try:
  width,height=(int(v) for v in str(text).lower().split('x'))
 except ValueError:
  raise ValueError('Camera size must look like 320x240') from None
 if not (160<=width<=1280 and 120<=height<=960) or width%2 or height%2:
  raise ValueError('Camera size must be even, from 160x120 to 1280x960')
 return width,height


class CameraFeed:
 def __init__(self,size=DEFAULT_SIZE,fps=30):
  self.size,self.fps=tuple(size),fps
  self.condition=threading.Condition()
  self.frame=None
  self.updated=0
  self.status='Looking for camera…'
  self.model=None
  threading.Thread(target=self.run,daemon=True).start()
 def publish(self,frame):
  with self.condition:
   self.frame=bytes(frame);self.updated=time.time();self.status='Live';self.condition.notify_all()
 def info(self):
  with self.condition:
   return {'status':self.status,'model':self.model,'updated':self.updated,'size':list(self.size),'fps':self.fps,
           'live':bool(self.frame and time.time()-self.updated<3)}
 def run(self):
  while True:
   camera=None
   try:
    from picamera2 import Picamera2
    from picamera2.encoders import JpegEncoder
    from picamera2.outputs import FileOutput
    cameras=Picamera2.global_camera_info()
    if cameras:
     camera=Picamera2(cameras[0]['Num'])
     self.model=cameras[0].get('Model','Pi camera')
     camera.configure(camera.create_video_configuration(main={'size':self.size},controls={'FrameRate':self.fps}))
     owner=self
     class Output(io.BufferedIOBase):
      def write(self,buf):owner.publish(buf);return len(buf)
     camera.start_recording(JpegEncoder(),FileOutput(Output()))
     while True:
      time.sleep(2)
      if not self.info()['live']:raise RuntimeError('Camera stopped delivering frames')
    else:
     # Only USB capture nodes, not the Pi's codec/ISP video devices.
     nodes=[p for p in sorted(Path('/sys/class/video4linux').glob('video*')) if 'usb' in str((p/'device').resolve()) and (p/'index').read_text().strip()=='0']
     if not nodes:raise RuntimeError('No camera detected — check connection and power')
     self.model=(nodes[0]/'name').read_text().strip()
     command=['ffmpeg','-nostdin','-loglevel','error','-f','v4l2','-i','/dev/'+nodes[0].name,'-vf',f'scale={self.size[0]}:-2','-r',str(min(15,self.fps)),'-f','image2pipe','-vcodec','mjpeg','-threads','1','-']
     with subprocess.Popen(command,stdout=subprocess.PIPE,stderr=subprocess.DEVNULL) as process:
      buffer=bytearray()
      while True:
       chunk=process.stdout.read1(16384)
       if not chunk:raise RuntimeError('USB camera stream unavailable')
       buffer.extend(chunk)
       while True:
        a=buffer.find(b'\xff\xd8');b=buffer.find(b'\xff\xd9',a+2) if a>=0 else -1
        if a<0 or b<0:break
        self.publish(buffer[a:b+2]);del buffer[:b+2]
       if len(buffer)>4_000_000:buffer.clear()
   except Exception as error:
    with self.condition:self.status=str(error);self.frame=None
   finally:
    if camera:
     try:camera.stop_recording();camera.close()
     except Exception:pass
   time.sleep(5)
