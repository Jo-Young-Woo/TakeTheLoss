from pathlib import Path
from types import SimpleNamespace

import cv2
import numpy as np
import pytest
import yaml

from maicon26.arena import ArenaGeometry
from maicon26.perception.uav_calibration import load_calibration
from maicon26.perception.uav_face_ledger import UavImageFaceLedger, draw_uav_image_ledger
from maicon26.perception.uav_face_ledger import sector_box_summary

ROOT = Path(__file__).resolve().parents[1]
SHARE = ROOT / 'ros2/maicon_arena_gz'


@pytest.fixture
def ledger():
    cal=load_calibration(SHARE/'config/uav_grid_calibration.json')
    arena=ArenaGeometry.from_spec(yaml.safe_load((SHARE/'config/arena_spec.yaml').read_text()))
    obj=UavImageFaceLedger(cal,arena)
    # Geometry tests use explicit image-detector outputs, not world model reads.
    boxes=[SimpleNamespace(cx=x,cy=y,sx=.4,sy=.4,confidence=.95) for x,y in ((2.,2.),(2.4,2.),(4.,3.))]
    obj.detector=SimpleNamespace(detect=lambda image: boxes)
    obj.ugv_detector=SimpleNamespace(detect=lambda image: [])
    return obj


def blank(): return np.zeros((720,1280,3),np.uint8)


def test_project_back_to_original_pixels_on_both_planes(ledger):
    cal=ledger.calibration
    xy=np.array([[1.,1.],[3.,2.4],[5.,3.8]])
    for z in (0.,.4):
        px=cal.arena_to_pixels(xy,z)
        assert np.allclose(cal.project(px,z),xy,atol=1e-8)
    assert np.linalg.norm(cal.arena_to_pixels(xy,0.)[0]-cal.arena_to_pixels(xy,.4)[0])>20


def test_two_frames_confirm_geometry_but_never_identify_a_picture(ledger):
    assert ledger.update(blank(),1.)['boxes_confirmed']==0
    s=ledger.update(blank(),2.)
    assert s['boxes_confirmed']==3 and len(s['faces'])==12
    assert all(f['state'] in ('pending','excluded') and f['ground_state'] is None for f in s['faces'])
    assert not s['geometry_uses_lidar'] and not s['geometry_uses_world_truth']
    assert ledger.update(blank(),2.) is None
    assert sum(r['boxes'] for r in s['sector_summary'].values())==3
    assert sum(r['identified_boxes'] for r in s['sector_summary'].values())==0
    for f in s['faces']:
        expected=ledger.arena.sector_of(*f['center'])
        assert f['sector']==('CM' if expected=='COMMON' else expected)


def test_sector_box_counts_deduplicate_faces_and_keep_unknown_incomplete():
    def face(bid,d,state,sector='CM',excluded=False):
        return dict(box=bid,face=d,sector=sector,ground_state=state,
                    image_state='excluded' if excluded else 'side_identity_unobserved')
    faces=[face(1,'E','identified'),face(1,'N','identified'),face(1,'S','unreachable'),
           face(2,'E','empty'),face(2,'N','identified'),face(2,'W',None,excluded=True),
           face(3,'E','empty','R-S1'),face(4,'E',None,'R-S1')]
    s=sector_box_summary(faces+[faces[0]])
    assert s['CM']['boxes']==2 and s['CM']['identified_boxes']==2
    assert s['CM']['identified_faces']==3 and s['CM']['resolved_boxes']==1
    assert s['R-S1']['boxes']==2 and s['R-S1']['identified_boxes']==0
    assert s['R-S1']['resolved_boxes']==1


def test_image_geometry_cannot_follow_a_shifted_lidar_box(ledger):
    ledger.update(blank(),1.); a=ledger.update(blank(),2.)
    for b in ledger.coverage.boxes.values():
        b.kind='full';b.cx+=.4;b.cy-=.3;b.sx=.8
    b=ledger.coverage.aerial_face_map()
    assert [f['segments'] for f in a['faces']]==[f['segments'] for f in b['faces']]
    assert all(f['source']=='uav_image' for f in b['faces'])


def test_lidar_only_boxes_do_not_enter_the_uav_inventory(ledger):
    from maicon26.perception.coverage import BoxModel
    ledger.update(blank(),1.);a=ledger.update(blank(),2.)
    local=BoxModel(999,3.,4.,'full',[],1.,2.)
    ledger.coverage._make_faces(local)
    ledger.coverage.boxes[999]=local
    b=ledger.coverage.aerial_face_map()
    assert len(a['faces'])==len(b['faces']) and all(f['box']!=999 for f in b['faces'])


def test_ground_identity_is_separate_and_future_or_old_states_are_rejected(ledger):
    ledger.update(blank(),1.); s=ledger.update(blank(),2.)
    row=next(f for f in s['faces'] if f['image_state']!='excluded')
    ground={'uav_map':{'outer_faces':{'faces':[dict(row,state='identified',box=333)]}}}
    a=ledger.update(blank(),3.,ground,2.5)
    match=next(f for f in a['faces'] if f['key']==row['key'])
    assert match['ground_state']=='identified' and match['image_state']=='side_identity_unobserved'
    assert match['ground_box']==333
    assert all(f['ground_state'] is None for f in ledger.update(blank(),4.,ground,5.)['faces'])
    assert all(f['ground_state'] is None for f in ledger.update(blank(),8.,ground,2.5)['faces'])


def test_touching_seam_has_no_outer_line_and_does_not_count_as_seen(ledger):
    ledger.update(blank(),1.); s=ledger.update(blank(),2.)
    inner=[f for f in s['faces'] if not f['segments']]
    assert len(inner)==2 and all(f['image_state']=='excluded' for f in inner)
    assert all(f['side_visibility'] in ('unverified','back_facing') for f in s['faces'])


def test_expired_image_inventory_is_unavailable(ledger):
    ledger.update(blank(),1.);ledger.update(blank(),2.)
    ledger.coverage.tick_uav_layout(30.)
    assert ledger.coverage.aerial_face_map()['faces']==[]


def test_actual_uav_fixture_detection_and_native_canvas_dimensions():
    cal=load_calibration(SHARE/'config/uav_grid_calibration.json')
    arena=ArenaGeometry.from_spec(yaml.safe_load((SHARE/'config/arena_spec.yaml').read_text()))
    obj=UavImageFaceLedger(cal,arena)
    rgb=cv2.imread(str(SHARE/'validation/uav_render_6ugv.png'))
    assert rgb is not None
    obj.update(rgb,1.); s=obj.update(rgb,2.)
    assert s['boxes_confirmed']>=10
    assert all(f['ground_state'] is None for f in s['faces'])
    canvas=draw_uav_image_ledger(rgb,s,cal)
    assert canvas.shape==(720,1700,3)
    assert np.array_equal(canvas[50:100,:100],rgb[50:100,:100])
