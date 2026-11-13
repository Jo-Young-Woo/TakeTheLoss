"""A bright laser-like pixel/line alone is not a geometric robot observation."""
from pathlib import Path
import numpy as np
import pytest
cv2=pytest.importorskip('cv2')
from maicon26.perception.wingbot_vision import WingbotGroundDetector
from maicon26.perception.tracks import UgvTracker
from maicon26.config import Config
from maicon26.types import Frame,Pose2D


@pytest.mark.parametrize('color',[(0,0,255),(255,0,0),(0,255,0)])
@pytest.mark.parametrize('shape',['point','white_core','line','diagonal','vertical'])
def test_laser_like_signal_does_not_create_ugv_or_obstacle_track(color,shape):
    bgr=np.full((480,640,3),80,np.uint8)
    # Include neutral highlights so the test does not rely on missing white.
    cv2.circle(bgr,(320,260),14,(230,230,230),-1)
    if shape in ('point','white_core'):
        cv2.circle(bgr,(320,220),8,color,-1)
        if shape=='white_core':cv2.circle(bgr,(320,220),3,(255,255,255),-1)
    else:
        ends={'line':((100,220),(540,220)),
              'diagonal':((100,100),(540,380)),
              'vertical':((320,100),(320,240))}
        cv2.line(bgr,*ends[shape],color,3)
    frame=Frame(bgr,1.)
    dets=WingbotGroundDetector().detect(frame)
    tracker=UgvTracker(Config(),'BLUE')
    tracker.update_camera(dets,Pose2D(1.,2.,0.),frame,None)
    assert not dets and not tracker.tracks


def test_real_robot_still_detected_with_laser_like_dot_in_view():
    path=Path(__file__).parent/'fixtures/wingbot_gazebo_red_rgb.png'
    bgr=cv2.imdecode(np.frombuffer(path.read_bytes(),np.uint8),cv2.IMREAD_COLOR)
    cv2.circle(bgr,(40,40),8,(0,0,255),-1)
    cv2.circle(bgr,(40,40),3,(255,255,255),-1)
    dets=WingbotGroundDetector().detect(Frame(bgr,1.))
    assert len(dets)==1 and dets[0].cls=='UGV' and dets[0].team=='RED'
