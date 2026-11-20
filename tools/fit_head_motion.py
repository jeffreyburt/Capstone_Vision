"""Offline fit of the 2026-11-20 right-head experiment; never commands hardware.

Run: python tools/fit_head_motion.py. Raw recordings, lens/floor files, neutral 391/770,
10 cm markers, rear offset -0.14 m, and face IDs 5/6 belong to this experiment.
The independent smaller-movement recording is NOT used to train the model.
Candidate parameters are saved, never activated or written into config.yaml.
"""
import json
import math
import sys
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from vision.geometry import CameraModel, FloorPose

ROOT = Path(__file__).resolve().parents[1]
TRAIN = ['right_head_motion_2026-11-20.json', 'right_head_offset_tilt_2026-11-20.json']
VALIDATION = 'right_head_validation_2026-11-20.json'
NEUTRAL = np.array([391, 770])


def rotate(points, pivot, angle, axis):
    c, s = math.cos(angle), math.sin(angle)
    R = np.array([[c,-s,0],[s,c,0],[0,0,1]]) if axis == 'pan' else np.array([[c,0,s],[0,1,0],[-s,0,c]])
    return (points-pivot) @ R.T + pivot


def project(parameters, units, marker_id, camera, floor, side_mount='tilt'):
    """Articulated square in chassis coordinates -> camera pixels, with a level chassis."""
    x,y,h,z,px,py,tx,tz,kp,kt,sx,sy,sz = parameters[:13]
    center = np.array([-.14,0,z]) if marker_id == 6 else np.array([sx,sy,sz])
    yaw = math.pi if marker_id == 6 else math.pi/2
    right, up = np.array([-math.sin(yaw),math.cos(yaw),0]), np.array([0,0,1])
    local = .05*np.array([up-right,up+right,-up+right,-up-right])
    if len(parameters) > 13:
        angles = [parameters[13],parameters[14],0] if marker_id == 6 else parameters[15:18]
        local = local @ cv2.Rodrigues(np.asarray(angles,dtype=float))[0].T
    q = center + local
    if marker_id == 6 or side_mount == 'tilt':
        q = rotate(q, np.array([tx,0,tz]), math.radians((units[1]-770)*kt), 'tilt')
    q = rotate(q, np.array([px,py,0]), math.radians((units[0]-391)*kp), 'pan')
    q = rotate(q, np.zeros(3), h, 'pan') + np.array([x,y,0])
    return cv2.projectPoints(q,floor.rvec,floor.tvec,camera.K,camera.dist)[0].reshape(4,2)


def stable_rows(data):
    """Use low measured joint velocity to reduce unsynchronized HTTP/frame timing error."""
    rows=[]
    for i,sample in enumerate(data['samples']):
        if i < 2 or any(v is None for v in sample['units']):
            continue
        window=[s['units'] for s in data['samples'][i-2:i+1]]
        if any(any(v is None for v in u) for u in window) or np.max(np.ptp(window,axis=0)) > 2:
            continue
        for marker in sample['markers']:
            if marker['id'] in (5,6):
                rows.append((np.array(sample['units']),marker['id'],np.array(marker['corners']),sample['phase'], sample['seq']))
    return rows


def rms(values):
    return float(np.sqrt(np.mean(np.asarray(values)**2)))


def stability(parameters, rows, camera, floor, side_mount):
    """Recover each stationary chassis pose anew; test whether motion compensation holds it still."""
    from scipy.optimize import least_squares
    # Group the two faces from the same observation before fitting a chassis pose.
    groups={}
    for u,mid,corners,phase,seq in rows:
        groups.setdefault((tuple(u),phase,seq),[]).append((mid,corners))
    poses=[]
    for (u,phase,seq), observations in groups.items():
        def residual(state):
            p=list(parameters);p[:3]=state
            return np.concatenate([(project(p,u,mid,camera,floor,side_mount)-c).ravel() for mid,c in observations])
        result=least_squares(residual,parameters[:3],max_nfev=100)
        poses.append(result.x)
    poses=np.array(poses)
    center=np.median(poses[:,:2],axis=0)
    distance=np.linalg.norm(poses[:,:2]-center,axis=1)
    heading=np.degrees(poses[:,2]-np.median(poses[:,2]))
    return {'groups':len(poses),'position_drift_p95_m':float(np.percentile(distance,95)),
            'position_drift_max_m':float(max(distance)),
            'heading_deviation_p95_deg':float(np.percentile(abs(heading),95)),
            'heading_span_deg':float(np.ptp(heading))}


def main():
    from scipy.optimize import least_squares
    camera=CameraModel.load(ROOT/'calib/camera_intrinsics.yaml')
    floor=FloorPose.load(ROOT/'calib/floor_pose_36cm.yaml')
    rows=[]
    for name in TRAIN:
        rows.extend(stable_rows(json.loads((ROOT/'calib'/name).read_text())))
    validation=stable_rows(json.loads((ROOT/'calib'/VALIDATION).read_text()))
    if not rows or not validation:
        raise SystemExit('Training and independent validation recordings are required')
    bounds=([.5,1.2,0,.15,-.3,-.2,-.3,.1,-.6,-.6,-.4,-.4,.15],
            [2,3.5,math.pi,.5,.3,.2,.3,.6,.6,.6,.4,.4,.6])
    bounds=(bounds[0]+[-.25,-.25,-.4,-.4,-.7],bounds[1]+[.25,.25,.4,.4,.7])
    best=None
    for mount in ('tilt','pan'):
        def residual(p):
            return np.concatenate([(project(p,u,mid,camera,floor,mount)-c).ravel() for u,mid,c,_,_ in rows])
        initial=[1.305,2.312,1.265,.294,.038,.028,-.080,.320,.24,.24,0,.10,.30,.04,-.03,.04,-.09,0]
        fit=least_squares(residual,initial,bounds=bounds,loss='soft_l1',f_scale=1,x_scale='jac',max_nfev=700)
        score=rms(fit.fun)
        print('Model',mount,'training RMS px',score,flush=True)
        if best is None or score < best['training_rms_px']:
            best={'side_mount':mount,'parameters':fit.x.tolist(),'training_rms_px':score,
                  'fit_converged':bool(fit.success),'jacobian_singular_values':np.linalg.svd(fit.jac,compute_uv=False).tolist()}
    p=best['parameters'];mount=best['side_mount']
    vr=np.concatenate([(project(p,u,mid,camera,floor,mount)-c).ravel() for u,mid,c,_,_ in validation])
    best['independent_validation']={'rows':len(validation),'rms_px':rms(vr)}
    best['training_stability']=stability(p,rows,camera,floor,mount)
    best['validation_stability']=stability(p,validation,camera,floor,mount)
    x,y,h,z,px,py,tx,tz,kp,kt,sx,sy,sz=p[:13]
    best['candidate_head_pose']={'neutral_units':[391,770],'degrees_per_unit':[kp,kt],
        'pan_pivot_m':[px,py,0.0],'tilt_pivot_m':[tx,0.0,tz],
        'valid_units':[[331,451],[710,775]],
        'markers':{'6':{'mount':'tilt','center_m':[-.14,0.0,z],'rotation_vector_rad':[p[13],p[14],0]},
                   '5':{'mount':mount,'center_m':[sx,sy,sz],'rotation_vector_rad':p[15:18]}}}
    best['unobservable_axis_coordinates']=['pan_pivot_m[2]','tilt_pivot_m[1]']
    best['limitations']=['Only small tested joint ranges are covered',
        'Stable samples reduce but do not solve live camera/joint clock alignment',
        'Right chassis must remain stationary during collection; baseline scale comes from existing lens/floor calibration',
        'Chassis origin depends on approximate owner rear-tag offset; pivot values are fitted, not tape measurements']
    reasons=[]
    if not best['fit_converged']:reasons.append('fit did not converge')
    if best['independent_validation']['rms_px'] > 3:reasons.append('independent corner error exceeds 3 px')
    for key in ('training_stability','validation_stability'):
        if best[key]['position_drift_p95_m'] > .04:reasons.append(key+': position drift exceeds 4 cm (below 10 cm stopping tolerance)')
        if best[key]['position_drift_max_m'] > .08:reasons.append(key+': peak position drift exceeds 8 cm')
        if best[key]['heading_deviation_p95_deg'] > 5:reasons.append(key+': heading deviation exceeds 5 degrees')
    best['validation_failures']=reasons
    best['status']='rejected_for_navigation' if reasons else 'offline_candidate_passed_live_validation_required'
    print(json.dumps(best,indent=2))
    (ROOT/'calib/right_head_pixel_model_fit_2026-11-20.json').write_text(json.dumps(best,indent=2)+'\n')


if __name__ == '__main__':
    main()
