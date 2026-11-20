"""Offline calibration geometry, stable-sample filtering, and hardware opt-in guard."""
import sys
from unittest.mock import patch

import numpy as np
import pytest

from tools.fit_head_motion import project, stable_rows
from vision.geometry import CameraModel, FloorPose, Geometry
from vision.robots import robot_specs_from_config


@pytest.mark.parametrize('mount', ['pan','tilt'])
def test_pixel_model_matches_independent_runtime_geometry_for_both_faces(mount):
    camera=CameraModel.from_fov(1600,900,100)
    floor=FloorPose.look_at([0,0,.395],[0,3,.1])
    geometry=Geometry(camera,floor)
    p=[.2,2.5,.7,.29,.03,.02,-.08,.32,.22,-.24,-.02,.11,.24,.04,-.03,.04,-.09,.02]
    spec=robot_specs_from_config({'robots':[{'id':2,'faces':{5:90,6:180},'head_pose':{
        'neutral_units':[391,770],'degrees_per_unit':[.22,-.24],
        'pan_pivot_m':[.03,.02,0],'tilt_pivot_m':[-.08,0,.32],
        'markers':{6:{'mount':'tilt','center_m':[-.14,0,.29],'rotation_vector_rad':[.04,-.03,0]},
                   5:{'mount':mount,'center_m':[-.02,.11,.24],'rotation_vector_rad':[.04,-.09,.02]}}}}]})[0]
    c,s=np.cos(.7),np.sin(.7)
    body=np.array([[c,-s,0],[s,c,0],[0,0,1]])
    for units in ([391,770],[445,718],[336,770]):
        for mid in (5,6):
            expected=spec.head.corners(mid,np.array(units)) @ body.T + [.2,2.5,0]
            np.testing.assert_allclose(project(p,units,mid,camera,floor,mount),
                                       geometry.world_to_pixels(expected),atol=1e-7)


def test_stable_rows_reject_motion_and_missing_feedback_and_keep_frame_identity():
    samples=[]
    for seq,units in enumerate(([391,770],[392,770],[391,770],[410,770],[None,770])):
        samples.append({'seq':seq,'units':units,'phase':'test',
                        'markers':[{'id':6,'corners':[[0,0]]*4},{'id':2,'corners':[[0,0]]*4}]})
    rows=stable_rows({'samples':samples})
    assert len(rows)==1
    assert rows[0][1]==6 and rows[0][4]==2


def test_capture_tool_requires_explicit_hardware_opt_in(tmp_path,monkeypatch):
    from tools.capture_right_head_motion import main
    monkeypatch.setattr(sys,'argv',['capture','--out',str(tmp_path/'record.json')])
    with patch('subprocess.Popen',side_effect=AssertionError('must not start SSH')):
        with pytest.raises(SystemExit) as error:
            main()
    assert error.value.code==2
    assert not (tmp_path/'record.json').exists()
