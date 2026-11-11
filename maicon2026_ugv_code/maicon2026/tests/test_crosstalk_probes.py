import importlib.util
from pathlib import Path

import numpy as np


def module(name):
    path=Path(__file__).resolve().parents[1]/"tools"/(name+".py")
    spec=importlib.util.spec_from_file_location(name,path)
    mod=importlib.util.module_from_spec(spec); spec.loader.exec_module(mod)
    return mod


def test_crosstalk_only_one_or_two_scan_unexplained_returns():
    probe=module("lidar_crosstalk_probe")
    base=[dict(t=i*.1,ranges=[2.,2.,2.,2.],angle_min=-.2,angle_increment=.1) for i in range(8)]
    rows=[dict(x) for x in base]
    rows[2]=dict(rows[2],ranges=[2.,1.,2.,2.],intensities=[0.,75.,0.,0.])
    for i in range(1,6): rows[i]=dict(rows[i],ranges=list(rows[i]["ranges"])) ; rows[i]["ranges"][2]=1.
    result=probe.analyse(rows,base)
    assert result["transient_point_count"]==1
    assert result["events"][0]["intensity"]==[75.]
    assert not result["acceptance"]["accepted"]


def test_independently_tracked_object_is_not_an_interference_candidate():
    probe=module("lidar_crosstalk_probe")
    rows=[dict(t=i*.1,ranges=[2.,2.],angle_min=0.,angle_increment=.1) for i in range(6)]
    baseline=[dict(x) for x in rows]
    rows[2]=dict(rows[2],ranges=[1.,2.],tracked_ranges=[dict(angle_min=-.05,angle_max=.05,range_min=.8,range_max=1.2)])
    assert probe.analyse(rows,baseline)["transient_point_count"]==0


def test_acceptance_needs_off_control_and_advantage_over_dynamic_tracker():
    probe=module("lidar_crosstalk_probe")
    events=[dict(t=.3)]
    good=[dict(start_s=0.,end_s=1.,present=True,lidar_detected_s=.4),
          dict(start_s=1.,end_s=61.,present=False)]
    assert probe.labelled_metrics(events,good)["accepted"]
    assert not probe.labelled_metrics(events,good[:1])["accepted"]
    good[0]["lidar_detected_s"]=.2
    assert not probe.labelled_metrics(events,good)["accepted"]


def test_camera_static_white_not_flash_but_new_bright_pixel_is_candidate():
    probe=module("camera_ir_probe")
    frame=np.zeros((10,10,3),dtype=np.uint8); frame[0,0]=255
    _,prev=probe.brightness(frame)
    row,_=probe.brightness(frame,prev)
    assert row["max_brightness"]==255 and row["flash_pixels"]==0
    frame[5,5]=250
    row,_=probe.brightness(frame,prev)
    assert row["flash_pixels"]==1


def test_contiguous_false_flash_frames_count_as_one_event_not_frame_rate():
    probe=module("lidar_crosstalk_probe")
    result=probe.labelled_metrics([dict(t=i*.1) for i in range(20)],
        [dict(start_s=0.,end_s=60.,present=False)])
    assert result["measured_false_events_per_minute"]==1.
