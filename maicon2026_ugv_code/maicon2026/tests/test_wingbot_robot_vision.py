"""Actual Gazebo RGB fixture and calibrated pixel-only robot observations."""
from pathlib import Path
import datetime as dt
import io

import numpy as np
import pytest
cv2=pytest.importorskip("cv2")

from maicon26.perception.wingbot_vision import WingbotGroundDetector, WingbotUavDetector
from maicon26.perception.uav_calibration import load_calibration, _project
from maicon26.link.http import HttpServerLink
from maicon26.link.base import UavDet
from maicon26.sim.mockserver import FakeClock, MockServer
from maicon26.types import Frame

ROOT=Path(__file__).resolve().parents[1]
CAL=ROOT/"ros2/maicon_arena_gz/config/uav_grid_calibration.json"


def rgb(path):
    return cv2.imdecode(np.frombuffer(Path(path).read_bytes(),np.uint8),cv2.IMREAD_COLOR)


def synthetic_uav(team="BLUE", pair=True, dome=True):
    cal=load_calibration(CAL)
    image=np.zeros((cal.image_size[1],cal.image_size[0],3),np.uint8)
    color=(230,65,20) if team=="BLUE" else (20,20,230)
    body=_project(np.linalg.inv(cal.plane_homography(.122)),np.array([[1.92,1.9],[2.12,2.1]]))
    lo=np.floor(body.min(axis=0)).astype(int);hi=np.ceil(body.max(axis=0)).astype(int)
    cv2.rectangle(image,tuple(lo),tuple(hi),(90,90,90),-1)
    for y in ([2.-.107,2.+.107] if pair else [2.-.107]):
        point=_project(np.linalg.inv(cal.plane_homography(.217)),np.array([[2.,y]]))[0]
        cv2.circle(image,tuple(np.rint(point).astype(int)),2,color,-1)
    if dome:
        for y in [2.-.103,2.+.103]:
            point=_project(np.linalg.inv(cal.plane_homography(.122)),np.array([[2.045,y]]))[0]
            cv2.circle(image,tuple(np.rint(point).astype(int)),3,(230,230,230),-1)
    return image,cal


@pytest.mark.parametrize("scale",[1.,.5,.25,.125])
def test_actual_gazebo_red_photo_needs_sleeves_and_white_receiver(scale):
    image=rgb(ROOT/"tests/fixtures/wingbot_gazebo_red_rgb.png")
    image=cv2.resize(image,None,fx=scale,fy=scale)
    result=WingbotGroundDetector().detect(Frame(image,7.25,meta={"sim_detections":[]}))
    assert len(result)==1 and result[0].team=="RED" and result[0].stamp==7.25
    assert result[0].extra["sensors"]
    # The picked dome is the actual left-side white receiver, not the plaque.
    u,v,_,_=result[0].extra["sensors"][0]
    assert 250*scale<u<305*scale and 505*scale<v<565*scale


def test_blank_and_photo_texture_negatives_ignore_oracle_metadata():
    detector=WingbotGroundDetector()
    assert detector.detect(Frame(np.zeros((480,640,3),np.uint8),1.,
        meta={"sim_detections":[{"cls":"UGV","team":"RED"}]}))==[]
    for path in (ROOT/"ros2/maicon_arena_gz/media/textures/faces").glob("*.jpg"):
        assert detector.detect(Frame(rgb(path),1.))==[],path.name


@pytest.mark.parametrize("team",["RED","BLUE"])
def test_uav_uses_two_actual_pixels_on_sleeve_height_plane(team):
    image,cal=synthetic_uav(team)
    detector=WingbotUavDetector(cal)
    result=detector.detect(image)
    assert len(result)==1 and result[0].team==team and result[0].ident is None
    assert np.linalg.norm(np.array([result[0].x,result[0].y])-[2.,2.])<.025
    assert detector.last_observations[0]["bounded_body_offset_m"]==.04


@pytest.mark.parametrize("pair,dome",[(False,True),(True,False),(False,False)])
def test_ambiguous_single_sleeve_or_absent_dome_cannot_identify_uav_robot(pair,dome):
    image,cal=synthetic_uav(pair=pair,dome=dome)
    assert WingbotUavDetector(cal).detect(image)==[]


def test_http_image_robot_callback_has_capture_age_and_safe_failure_contract():
    clock=FakeClock(10.); mock=MockServer({"K7M2Q9XA":"B-2"},clock=clock)
    mock.set_phase("running")
    image,cal=synthetic_uav()
    jpeg=cv2.imencode(".jpg",image)[1].tobytes()
    link=HttpServerLink("http://fixture.invalid","K7M2Q9XA",clock=clock,transport=mock,
                       uav_robot_detector=WingbotUavDetector(cal).detect)
    link.me(); link._started_bg=True; mock.capture(jpeg)
    frame=link.get_uav()
    assert frame and len(frame.detections)==1 and frame.detections[0].team=="BLUE"
    assert frame.age_s==pytest.approx(0.)
    clock.advance(1.)
    link.uav_robot_detector=lambda _: [UavDet(float("nan"),1.,"RED")]
    mock.capture(jpeg)
    assert link.get_uav().detections==[]
    assert link.stats["drone_robots_rejected"]==1


def test_referee_heading_sweep_uses_repeatable_generated_geometry():
    import importlib.util
    path=ROOT/"scripts/study_receiver_exposure.py"
    spec=importlib.util.spec_from_file_location("receiver_study",path)
    module=importlib.util.module_from_spec(spec); spec.loader.exec_module(module)
    result=module.evaluate(pan_samples=21,tilt_samples=21)
    assert not result["orientation_cost_promoted"]
    assert not result["conditions"]["continuous_dwell_simulated"]
    for distance in (.6,1.,1.5):
        rows={r["receiver_heading"]:r for r in result["rows"] if r["range_m"]==distance}
        left=rows["left_dome_toward_shooter"]["receiver_intersections"]
        right=rows["right_dome_toward_shooter"]["receiver_intersections"]
        assert left==right and left>0
        assert rows["front_toward_shooter"]["receiver_intersections"]<left


@pytest.mark.parametrize("quality",[None,100,95,90,85])
def test_actual_six_wingbot_overhead_has_three_each_and_no_phantom_from_floor(quality):
    image=rgb(ROOT/"tests/fixtures/wingbot_gazebo_six_uav_rgb.png")
    if quality is not None:
        image=cv2.imdecode(cv2.imencode(".jpg",image,[cv2.IMWRITE_JPEG_QUALITY,quality])[1],cv2.IMREAD_COLOR)
    result=WingbotUavDetector(load_calibration(CAL)).detect(image)
    assert len(result)==6
    assert sum(d.team=="BLUE" for d in result)==3
    assert sum(d.team=="RED" for d in result)==3
    for d in result:
        # Independent fixture expectations only; these never enter detector.
        x=.34 if d.team=="BLUE" else 5.66
        assert abs(d.x-x)<.08
        assert min(abs(d.y-y) for y in [.8,2.4,4.])<.08

@pytest.mark.parametrize('capture_age',[1.,2.,5.,6.,13.])
def test_uav_robot_observations_keep_capture_age_and_reject_expired_images(capture_age):
    clock=FakeClock(10.);mock=MockServer({'K7M2Q9XA':'B-2'},clock=clock)
    mock.set_phase('running')
    image,cal=synthetic_uav();jpeg=cv2.imencode('.jpg',image)[1].tobytes()
    calls=[]
    def detect(frame):
        calls.append(True)
        return [UavDet(3.,2.,'RED')]
    link=HttpServerLink('http://fixture.invalid','K7M2Q9XA',clock=clock,transport=mock,uav_robot_detector=detect)
    link.me();link._started_bg=True;mock.capture(jpeg);clock.advance(capture_age)
    frame=link.get_uav()
    assert frame is not None
    if capture_age<=12.:
        assert calls and len(frame.detections)==1 and frame.age_s==pytest.approx(capture_age)
    else:
        assert not calls and frame.detections==[]
        assert link.stats['drone_robots_rejected']==1


@pytest.mark.parametrize('destroyed,period',[(0,1.),(1,2.),(2,5.)])
def test_uav_degraded_one_two_five_second_cadence_has_no_replayed_detection(destroyed,period):
    clock=FakeClock(10.);mock=MockServer({'K7M2Q9XA':'B-2'},clock=clock)
    mock.set_phase('running');mock.destroyed['BLUE']=destroyed
    image,cal=synthetic_uav();jpeg=cv2.imencode('.jpg',image)[1].tobytes()
    link=HttpServerLink('http://fixture.invalid','K7M2Q9XA',clock=clock,transport=mock,
                       uav_robot_detector=lambda image:[UavDet(3.,2.,'RED')])
    link.me();link._started_bg=True;mock.capture(jpeg)
    frame=link.get_uav()
    assert frame.interval_s==period and len(frame.detections)==1
    clock.advance(period)
    assert link.get_uav() is None, 'An old HTTP image is not new robot evidence'
    mock.capture(jpeg);clock.advance(period)
    assert link.get_uav().stamp>frame.stamp


def test_uav_403_keeps_lidar_only_risk_without_fabricating_enemy_or_using_depth():
    from test_fast_threat import setup,scene_scan
    from maicon26.perception.lidar_blobs import BlobDetector
    from maicon26.perception.threat_board import ThreatBoard,EV_OBJECT
    from maicon26.mission.threat_field import build_field
    cfg,grid,pose=setup();cfg.depth.enable=cfg.map3d.enable=False
    clock=FakeClock(10.);mock=MockServer({'K7M2Q9XA':'B-2'},clock=clock)
    mock.set_phase('running');mock.destroyed['BLUE']=3
    def forbidden_inference(image):
        raise AssertionError('403 has no usable UAV image')
    link=HttpServerLink('http://fixture.invalid','K7M2Q9XA',clock=clock,transport=mock,
                       uav_robot_detector=forbidden_inference)
    link.me();link._started_bg=True
    assert link.get_uav() is None and link._drone_next==15.
    detector=BlobDetector(cfg);detector.update(scene_scan(pose),pose,grid,9.9)
    scan=scene_scan(pose,[(2.,2.,.13)],10.)
    blobs=detector.update(scan,pose,grid,10.)
    tracks=ThreatBoard(cfg,'BLUE').update(None,blobs,10.,pose,use_risk_weights=True)
    assert len(tracks)==1 and tracks[0].evidence==EV_OBJECT and tracks[0].p_enemy==.5
    field=build_field(tracks,grid,cfg.avoid,3.,10.,cfg.nav.inflate_hard,risk_cfg=cfg.risk)
    assert field.risk.max()>0 and field.body.any()
