"""Recording must preserve control calls and capture identity exactly."""
import importlib.util
from pathlib import Path
from types import SimpleNamespace
import numpy as np
from maicon26.types import Detection,Pose2D,Frame
from maicon26.config import Config
from maicon26.mission.aim import AimController

SPEC=importlib.util.spec_from_file_location('aim_diagnostics',Path(__file__).parents[1]/'scripts/aim_diagnostics.py')
MODULE=importlib.util.module_from_spec(SPEC);SPEC.loader.exec_module(MODULE)


def fixture():
    calls=[]
    a=SimpleNamespace(aim=AimController(Config()),stack=SimpleNamespace(pose=Pose2D(1.,2.,.3)),
        _det_ctx=(8.,Pose2D(.9,2.,.2),10.,2.),_det_consumed=100.,_tick_no=1,
        mode=SimpleNamespace(value='STRIKE'),_may_fire=lambda now:calls.append(now) or False)
    return a,calls


def test_diagnostic_does_not_change_aim_result_or_call_guards_again():
    a,calls=fixture();plain=AimController(Config());original=a.aim.update
    det=Detection('RADAR',.95,(200.,160.,250.,220.),stamp=100.)
    diagnostic=MODULE.install_aim_diagnostics(a)
    for tick in range(3):
        a._tick_no=tick
        actual=a.aim.update(det,10.,2.,8.1+tick*.05,1.1)
        assert actual==plain.update(det,10.,2.,8.1+tick*.05,1.1)
    assert calls==[]
    assert a._may_fire(8.2) is False and calls==[8.2]
    row=diagnostic.snapshot()
    assert row['same_detection_reuse_count']==2
    assert row['capture_robot_time']==8. and row['capture_pan_tilt']==[10.,2.]
    assert row['guards_called_this_tick']['_may_fire']['value'] is False
    assert '_shot_geometry_ok' not in row['guards_called_this_tick']
    a._tick_no+=1
    assert not diagnostic.snapshot()['updated_this_tick']
    assert diagnostic.snapshot()['guards_called_this_tick']=={}
    diagnostic.close()
    assert a.aim.update==original


def test_camera_evidence_requires_exact_capture_stamp_and_owns_pixels():
    a,_=fixture();diagnostic=MODULE.install_aim_diagnostics(a)
    pixels=np.zeros((3,4,3),dtype=np.uint8)
    diagnostic.observe_frame(Frame(pixels,99.))
    det=Detection('RADAR',.95,(200.,160.,250.,220.),stamp=100.)
    a.aim.update(det,10.,2.,8.1,1.1)
    assert diagnostic.exact_source_image() is None
    pixels[:]=12
    diagnostic.observe_frame(Frame(pixels,100.))
    pixels[:]=31
    assert np.all(diagnostic.exact_source_image()==12)
    saved=diagnostic.exact_source_image();saved[:]=50
    assert np.all(diagnostic.exact_source_image()==12)
    diagnostic.close()
