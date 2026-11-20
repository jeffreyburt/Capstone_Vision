"""Opt-in right-head experiment recorder. Never drives wheels.

Keep the chassis stationary and the head at the saved neutral 391/770. Run with --execute
and a NEW --out JSON path after checking clearance. The right dashboard must own the UART;
tracking and sweeps must be stopped. This uses its existing bounded eased-sweep API and records
camera corners plus actual servo positions. It temporarily replaces dashboard A/B sweep settings
(saved in original_state); it does not change saved tracking limits. On a failure it sends stop
and keeps torque engaged for support, so inspect the reported final state before touching the head.
All commands use the previously tested pan 331..451 and tilt 710..770 envelope.
"""

def main():
    import sys,json,time,threading,subprocess,urllib.request,signal
    from pathlib import Path
    sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
    import cv2
    from camstream.receiver import FrameReceiver
    from vision.config import load_config
    from vision.markers import MarkerDetector
    import argparse
    parser=argparse.ArgumentParser(description='Stationary Robot 2 pan/tilt capture; wheels must remain off. Returns near neutral and leaves torque on if a motion check fails.')
    parser.add_argument('--execute',action='store_true',help='explicitly engage and move the right head')
    parser.add_argument('--out',required=True,help='new recording JSON path (existing files are never overwritten)')
    parser.add_argument('--ssh-target',default='capstone-pi2@192.168.50.12')
    parser.add_argument('--local-port',type=int,default=18083)
    args=parser.parse_args()
    if not args.execute:parser.error('--execute is required to move physical hardware')
    if not 1024 <= args.local_port <= 65535:parser.error('local-port must be 1024..65535')
    ROOT=Path(__file__).resolve().parents[1]
    OUT=Path(args.out).resolve()
    if OUT.exists():raise RuntimeError('Refusing to overwrite existing recording')
    base='http://127.0.0.1:'+str(args.local_port)
    report={'status':'collecting_not_validated','neutral':[391,770],'marker_size_m':.1,'phases':[], 'samples':[],'original_state':None,'owns_head':False}
    finished=threading.Event(); failure=[]; phase={'name':'baseline'}
    def request(path,data=None):
        payload=None if data is None else json.dumps(data).encode()
        req=urllib.request.Request(base+path,data=payload,headers={'Content-Type':'application/json'})
        with urllib.request.urlopen(req,timeout=3) as r:
            b=r.read();return json.loads(b) if b else None
    
    def worker():
        try:
            state=request('/api/state'); report['original_state']={k:state.get(k) for k in ('positions','torque','config','tracking')}
            if state['running'] or state.get('controlling') or max(abs(a-b) for a,b in zip(state['positions'],[391,770]))>10:
                raise RuntimeError('Expected idle head at confirmed neutral 391/770')
            report['owns_head']=True
            request('/api/engage',{})
            time.sleep(1.5)
            for name,target in [('pan_plus',[451,770]),('pan_minus',[331,770]),('tilt_minus',[391,710])]:
                phase['name']=name
                config={'a':target,'b':[391,770],'duration':2,'cycles':1,'pause':1.2}
                report['phases'].append({'name':name,'config':config,'started_local':time.time()})
                print('START',name,target,flush=True)
                request('/api/start',config)
                end=time.monotonic()+16
                while True:
                    time.sleep(.15)
                    state=request('/api/state')
                    if state.get('position_error') or any(v is None for v in state['positions']):
                        raise RuntimeError('Servo feedback lost')
                    if not state['running']:
                        if state['status']!='Sweep completed':raise RuntimeError('Motion failed: '+state['status'])
                        if max(abs(a-b) for a,b in zip(state['positions'],[391,770]))>5:
                            raise RuntimeError('Did not return to neutral')
                        break
                    if time.monotonic()>end:raise RuntimeError('Motion timed out')
                print('COMPLETE',name,state['positions'],flush=True)
                phase['name']='neutral_after_'+name
                time.sleep(1.5)
            report['status']='captured_pending_fit'
        except BaseException as e:
            report['status']='failed';failure.append(str(e));print('ERROR',str(e),flush=True)
            if report['owns_head']:
                try:request('/api/stop',{})
                except Exception:pass
        finally:
            try:
                state=request('/api/state')
                report['final_positions']=state.get('positions')
                # Restore original torque state only after successful return to neutral.
                if report['status']=='captured_pending_fit' and report['original_state']['torque']==[0,0]:
                    request('/api/disengage',{})
                    report['final_torque']=[0,0]
            except Exception as e:failure.append(str(e))
            finished.set()
    
    log=open(OUT.with_suffix('.tunnel.log'),'ab')
    tunnel=subprocess.Popen(['ssh','-N','-o','BatchMode=yes','-o','ExitOnForwardFailure=yes','-o','ServerAliveInterval=5','-L','127.0.0.1:'+str(args.local_port)+':127.0.0.1:8080',args.ssh_target],stdin=subprocess.DEVNULL,stdout=log,stderr=log)
    rx=None;thread=None
    try:
        for _ in range(30):
            try:request('/api/state');break
            except Exception:
                if tunnel.poll() is not None:raise RuntimeError('SSH tunnel failed')
                time.sleep(.2)
        else:raise RuntimeError('Dashboard unreachable')
        cfg=load_config();rx=FrameReceiver(cfg,['birdseye']);detector=MarkerDetector();last=None
        frame=rx.wait_new('birdseye',timeout=5)
        if frame is None:raise RuntimeError('Main camera unavailable; no motion started')
        if 6 not in [m.id for m in detector.detect(frame.image)]:raise RuntimeError('Rear tag 6 unavailable; no motion started')
        thread=threading.Thread(target=worker);thread.start()
        while not finished.is_set():
            frame=rx.wait_new('birdseye',last,timeout=1)
            if frame is None:continue
            last=frame.seq
            state=request('/api/state')
            markers=[{'id':m.id,'corners':m.corners.tolist()} for m in detector.detect(frame.image) if m.id in (2,5,6,7)]
            report['samples'].append({'local_received':frame.t_received,'capture_t':frame.t_capture,'seq':frame.seq,
              'phase':phase['name'],'units':state['positions'],'position_time':state.get('position_time'),
              'servo_server_time':state.get('serverTime'),'motion_running':state['running'],'motion_status':state['status'],'markers':markers})
    finally:
        if thread and thread.is_alive():
            if report['owns_head']:
                try:request('/api/stop',{})
                except Exception:pass
            thread.join(timeout=5)
        if rx:rx.close()
        report['errors']=failure
        OUT.write_text(json.dumps(report,indent=2)+'\n')
        tunnel.terminate()
        try:tunnel.wait(timeout=3)
        except subprocess.TimeoutExpired:tunnel.kill()
        log.close()
        print('SAVED',OUT,'samples',len(report['samples']),'status',report['status'],flush=True)


if __name__ == '__main__':
    main()
