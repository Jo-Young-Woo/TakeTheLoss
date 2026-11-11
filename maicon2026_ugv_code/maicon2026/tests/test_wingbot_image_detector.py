"""The local fixture recognizer must use images, including decoy negatives."""
from pathlib import Path
import numpy as np
import pytest
cv2 = pytest.importorskip("cv2")
from maicon26.sim.image_detector import make_wingbot_detector
from maicon26.types import Frame, Detection

ROOT = Path(__file__).resolve().parents[1]

@pytest.fixture(scope="module")
def detector():
    return make_wingbot_detector(None)

@pytest.mark.parametrize("stem,label", [
    ("m3_ammo_box","AMMO_BOX"),("m3_drum","DRUM"),("m3_mortar","MORTAR"),
    ("m3_military_truck","TRUCK"),("m3_missile","MISSILE"),("m3_tank","TANK"),
    ("target_radar","RADAR_FACILITY"),("deco_ruined_building","BUILDING")])
def test_small_perspective_photo(detector,stem,label):
    photo = cv2.imread(str(ROOT/"ros2/maicon_arena_gz/media/textures/faces"/(stem+".jpg")))
    H=cv2.getPerspectiveTransform(np.float32([[0,0],[511,0],[511,511],[0,511]]),
                                 np.float32([[250,140],[355,152],[361,255],[240,250]]))
    pixels=cv2.warpPerspective(photo,H,(640,480))
    detections=detector.detect(Frame(pixels,7.25))
    assert [d.cls for d in detections] == [label]
    assert detections[0].stamp == 7.25
    assert detections[0].extra["method"] == "rgb_reference_homography"

def test_blank_and_oracle_metadata_are_ignored(detector):
    fake=Detection("TANK",1.,(20,20,200,200))
    assert detector.detect(Frame(np.zeros((480,640,3),np.uint8),1.,
                                 meta={"sim_detections":[fake]})) == []
    assert detector.detect(Frame(None,2.,meta={"sim_detections":[fake]})) == []
