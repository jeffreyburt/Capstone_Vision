import {FilesetResolver, FaceDetector} from './vision_bundle.mjs';
const status=document.getElementById('headStatus');
const toggle=document.getElementById('headToggle');
const image=document.getElementById('cameraFeed');
const overlay=document.getElementById('headOverlay');
const ctx=overlay.getContext('2d');
const input=document.createElement('canvas');input.width=640;input.height=480;
const capture=input.getContext('2d');
let detector=null,enabled=true,frames=0,failures=0;let lastCenter=null;
function clear(){ctx.clearRect(0,0,640,480)}
toggle.onclick=()=>{enabled=!enabled;if(!enabled&&window.headTrackingActive){window.headTrackingActive=false;fetch('/api/stop',{method:'POST'}).catch(()=>{})};toggle.textContent=enabled?'Hide head detection':'Show head detection';clear();status.textContent=enabled?'Waiting for camera…':'Head detection off'};
try{
 status.textContent='Loading head detection model…';
 const vision=await FilesetResolver.forVisionTasks('/assets/wasm');
 detector=await FaceDetector.createFromOptions(vision,{
  baseOptions:{modelAssetPath:'/assets/blaze_face_short_range.tflite',delegate:'CPU'},
  runningMode:'IMAGE',minDetectionConfidence:0.6,minSuppressionThreshold:0.3
 });
 status.textContent='Head detection ready';
 document.documentElement.dataset.headDetection='ready';
}catch(error){status.textContent='Head detection unavailable: '+error.message;document.documentElement.dataset.headDetection='error';console.error(error)}
async function loop(){
 const started=performance.now();let bitmap=null;
 try{
 if(!detector||!enabled||document.hidden)return;
 const response=await fetch('/api/camera/frame?t='+Date.now(),{cache:'no-store',signal:AbortSignal.timeout(2500)});
 if(!response.ok)throw Error('Waiting for a fresh camera frame');
 const frameTime=Number(response.headers.get('X-Frame-Time'));
 bitmap=await createImageBitmap(await response.blob());
 if(!enabled)return;
  // Match the displayed 180-degree camera rotation before inference.
  capture.setTransform(-1,0,0,-1,640,480);capture.drawImage(bitmap,0,0,640,480);capture.resetTransform();
  const result=detector.detect(input);clear();
  ctx.strokeStyle='rgba(255,255,255,.55)';ctx.lineWidth=1;ctx.setLineDash([5,5]);ctx.strokeRect(640*.44,480*.44,640*.12,480*.12);ctx.setLineDash([]);
  const candidates=result.detections.map(d=>{const box=d.boundingBox;return {center:[Math.max(0,Math.min(1,(box.originX+box.width/2)/640)),Math.max(0,Math.min(1,(box.originY+box.height/2)/480))],area:box.width*box.height,box}});
  candidates.sort((a,b)=>lastCenter?Math.hypot(...a.center.map((v,i)=>v-lastCenter[i]))-Math.hypot(...b.center.map((v,i)=>v-lastCenter[i])):b.area-a.area);
  const selected=candidates[0]?.center||null;lastCenter=selected;
  if(window.headTrackingActive){
   const r=await fetch('/api/tracking/observation',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({center:selected,frameTime,target:'face'}),signal:AbortSignal.timeout(1000)});
   if(!r.ok)throw Error('Tracking paused: frame too old or controller unavailable');
  }

  for(const {box} of candidates){
   const left=Math.max(0,box.originX-8),top=Math.max(0,box.originY-8);
   const right=Math.min(640,box.originX+box.width+8),bottom=Math.min(480,box.originY+box.height+8);
   ctx.strokeStyle='#63e3ba';ctx.lineWidth=3;ctx.strokeRect(left,top,right-left,bottom-top);
   ctx.font='bold 16px system-ui';const labelY=Math.max(22,top);
   ctx.fillStyle='#63e3ba';ctx.fillRect(left,labelY-22,68,22);ctx.fillStyle='#08251c';ctx.fillText('Head',left+8,labelY-6);
  }
  if(!candidates.length&&window.headPrediction){
   const [x,y]=window.headPrediction;
   const px=Math.max(14,Math.min(626,x*640)),py=Math.max(24,Math.min(466,y*480));
   ctx.strokeStyle='#ffbf69';ctx.lineWidth=3;ctx.setLineDash([4,4]);ctx.beginPath();ctx.arc(px,py,10,0,Math.PI*2);ctx.stroke();ctx.setLineDash([]);
   ctx.fillStyle='#ffbf69';ctx.font='bold 15px system-ui';ctx.fillText('Predicted target',Math.min(px+14,510),Math.max(18,py-14));
  }
  status.textContent=candidates.length?`${candidates.length} face${candidates.length>1?'s':''} detected`:'No face detected';
  frames++;status.textContent+=' · live update '+frames;
  document.documentElement.dataset.headCount=String(candidates.length);document.documentElement.dataset.headFrames=String(frames);failures=0;
 }catch(error){clear();failures++;status.textContent='Head detection retrying: '+error.message;
 }finally{if(bitmap)bitmap.close();setTimeout(loop,failures?500:Math.max(0,50-(performance.now()-started)))}
}
loop();
document.addEventListener('visibilitychange',()=>{if(document.hidden)clear()});
