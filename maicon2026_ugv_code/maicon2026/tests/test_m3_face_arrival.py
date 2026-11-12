"""Tiny deterministic fixtures; no Gazebo or simulator is started.

The saved candidate03 B2 point is used solely to reproduce a range-boundary
navigation error. The controller receives an observed FaceRec, never truth.
"""
import math
from pathlib import Path
import sys
from types import SimpleNamespace

import numpy as np
import pytest


from maicon26.config import Config
from maicon26.mission.manager import Agent as CandidateAgent
from maicon26.mission import face_arrival as helper
from maicon26.mission.explore_online import Cand, OnlineExplorer
from maicon26.nav.grid import OccupancyGrid
from maicon26.perception.coverage import CoverageMap, BoxModel, FaceRec
from maicon26.types import Pose2D


SAVED_VP = (4.916601208277928, .7244125009152487)
SAVED_STOP = (4.9843746912396725, .8748174303290333)
OBSERVED_FACE = (4.623588, 1.024901)


def make_agent(cls=CandidateAgent, enabled=True, face_xy=OBSERVED_FACE, known=True):
    cfg = Config()
    cfg.progress.enabled = enabled
    cfg.explore.view_max_incidence_deg = 70.
    cfg.explore.complete_m3_faces = False
    cov = CoverageMap(cfg)
    fc = FaceRec(1, 3, 0., -1., *face_xy)
    box = BoxModel(1, face_xy[0], face_xy[1]+.2, 'full', [], 0., 100.)
    box.faces = [FaceRec(1, i, nx, ny, box.cx+.2*nx, box.cy+.2*ny)
                 for i, (nx,ny) in enumerate([(1,0),(0,1),(-1,0),(0,-1)])]
    box.faces[3] = fc
    box.full_seen = 3
    cov.boxes = {1:box}
    grid = OccupancyGrid(cfg.arena)
    if known:
        grid.L.fill(-2.)
    cand = Cand('face', SAVED_VP, math.atan2(fc.cy-SAVED_VP[1],fc.cx-SAVED_VP[0]), 'look', faces=[fc.key], primary_face=fc.key)
    a = object.__new__(cls)
    a.cfg, a.coverage, a.explorer = cfg, cov, OnlineExplorer(cfg, cov)
    a.stack = SimpleNamespace(pose=Pose2D(*SAVED_STOP, 0.), grid=grid, clear_goal=lambda:None)
    a.stats = {}
    a._ex_phase, a._ex_cand, a._ex_sel_t, a._ex_goal_t0 = 'travel', cand, 100., 100.
    a._rc = {}
    a._progress_arrived = False
    a._deadlock_enabled = lambda: enabled
    a._dwell_expired = lambda now: False
    a.progress = SimpleNamespace(eligible=lambda goal,now:True)
    a._goal_key = lambda point:point
    a._sweep_pan = lambda *args:0.
    a.outputs = []
    a._apply = lambda *args:a.outputs.append(args)
    a._stop_output = lambda:a.outputs.append(('stop',))
    a.nav_calls = []
    def nav(now,dt,goal,tol,**kw):
        a.nav_calls.append((goal,tol,kw))
        a._progress_arrived = True
        return SimpleNamespace(status='reached',v=0.,w=0.)
    a._nav_to = nav
    return a, fc


def step(a, now=100.):
    a._mode_explore_online(now, .05, now)


def test_saved_goal_valid_but_actual_stop_outside_existing_range():
    a, f = make_agent()
    p = SimpleNamespace(x=SAVED_VP[0], y=SAVED_VP[1])
    assert helper.face_arrival_view(a._ex_cand,p,a.coverage,a.stack.grid).valid
    actual = helper.face_arrival_view(a._ex_cand,a.stack.pose,a.coverage,a.stack.grid)
    assert not actual.valid and actual.reason == 'range'
    assert math.hypot(*(x-y for x,y in zip(SAVED_VP,SAVED_STOP))) < .18


def test_navigation_reached_does_not_enter_observation_outside_face_cone():
    a, f = make_agent()
    step(a)
    assert a._ex_phase != 'observe'


def test_candidate_refines_once_without_fake_view_or_progress():
    a, f = make_agent()
    step(a)
    assert a._ex_phase == 'travel'
    assert not a._progress_arrived
    assert a.stats['face_arrival_refinements'] == 1
    assert f.views == f.det_n == f.arrive_attempts == 0
    target = a._ex_arrival_refine['target']
    assert math.dist(target,SAVED_VP) <= .25
    assert math.dist(target,SAVED_STOP) <= .4
    for angle in np.linspace(0,2*math.pi,73):
        p=SimpleNamespace(x=target[0]+.03*math.cos(angle),y=target[1]+.03*math.sin(angle))
        assert helper.face_arrival_view(a._ex_cand,p,a.coverage,a.stack.grid).valid


def test_valid_refinement_enters_observe_and_retargets_actual_heading():
    a, f = make_agent()
    step(a)
    target = a._ex_arrival_refine['target']
    a.stack.pose = Pose2D(*target, .6)
    step(a,100.1)
    assert a.nav_calls[-1][0] == target and a.nav_calls[-1][1] == .03
    assert a._ex_phase == 'observe' and a._ex_arrival_refine is None
    assert a._ex_cand.heading == pytest.approx(math.atan2(f.cy-target[1],f.cx-target[0]))
    assert f.views == f.det_n == f.arrive_attempts == 0


def test_one_failed_refinement_defers_without_repeating_or_consuming_face():
    a, f = make_agent()
    step(a)
    step(a,100.1)  # navigator claims reached, actual pose still invalid
    assert a._ex_cand is None and a._ex_phase == 'travel'
    assert a._ex_arrival_refine is None
    assert a.stats['face_arrival_refinements'] == a.stats['face_arrival_deferred'] == 1
    assert not a._progress_arrived
    assert f.views == f.det_n == f.arrive_attempts == 0
    assert a.explorer._is_tabu(SAVED_VP,100.2)


def test_refinement_timeout_defers_before_another_navigation_output():
    a, f = make_agent()
    step(a)
    step(a,103.01)
    assert len(a.nav_calls)==1 and a.outputs==[('stop',)]
    assert a._ex_cand is None
    assert a.stats['face_arrival_last_reason']=='refinement_timeout'
    assert f.arrive_attempts==0


def test_unknown_refinement_space_is_not_assumed_free():
    a, f = make_agent(known=False)
    step(a)
    assert a._ex_cand is None
    assert a.stats['face_arrival_deferred']==1
    assert 'face_arrival_refinements' not in a.stats
    assert f.arrive_attempts==0


def test_eligible_filter_respected_and_no_alternate_goal_if_all_excluded():
    a, f = make_agent()
    a.progress.eligible=lambda goal,now:False
    step(a)
    assert a._ex_cand is None and f.arrive_attempts==0


def test_incidence_outside_70_is_never_accepted_or_relaxed():
    a, f=make_agent(face_xy=(4.6,1.0))
    v=helper.face_arrival_view(a._ex_cand,a.stack.pose,a.coverage,a.stack.grid)
    assert not v.valid and v.reason=='incidence'
    assert a.cfg.explore.view_max_incidence_deg==70.
    step(a)
    assert a._ex_phase=='travel'


def test_occlusion_remains_disqualifying():
    a, f=make_agent()
    a.stack.pose=Pose2D(f.cx,f.cy-.8,0.)
    a.stack.grid.add_static_box((f.cx-.1,f.cy-.45,f.cx+.1,f.cy-.35))
    v=helper.face_arrival_view(a._ex_cand,a.stack.pose,a.coverage,a.stack.grid)
    assert not v.valid and v.reason=='occluded'


def test_strict_picture_fov_checker_is_used_when_picture_pending():
    a, f=make_agent()
    a.coverage.picture_pending=lambda face:True
    a.coverage.picture_visible=lambda *args:(False,'fov')
    a.stack.pose=Pose2D(f.cx,f.cy-.8,0.)
    v=helper.face_arrival_view(a._ex_cand,a.stack.pose,a.coverage,a.stack.grid)
    assert not v.valid and v.reason=='fov'
    step(a)
    assert a._ex_cand is None


def test_missing_face_never_claims_success():
    a, f=make_agent()
    a.coverage.boxes.clear()
    step(a)
    assert a._ex_cand is None and a.stats['face_arrival_last_reason']=='missing_face'


def test_frontier_and_legacy_are_unmodified():
    a, f=make_agent()
    a._ex_cand.kind='frontier'
    step(a)
    assert a._ex_phase=='observe'
    a, f=make_agent(enabled=False)
    step(a)
    assert a._ex_phase=='observe'


def test_refreshed_face_candidate_without_primary_keeps_pending_observed_subject():
    a, f=make_agent()
    a._ex_cand.primary_face=None
    step(a)
    assert a._ex_phase=='travel'
    assert a._ex_cand is not None and a._ex_cand.primary_face==f.key
    assert a.stats['face_arrival_refinements']==1
    assert f.views==f.arrive_attempts==0


def test_missing_primary_uses_original_direction_not_current_nearest_face():
    a, f=make_agent()
    original=a._ex_cand
    original.primary_face=None
    # A second observed face is closer to the displaced robot but behind the
    # original camera direction. Pending-face fallback must retain f.
    other=FaceRec(2,0,0.,1.,5.,.8)
    box=BoxModel(2,5.,.6,'face',[],0.,100.,faces=[other])
    a.coverage.boxes[2]=box
    original.faces=[other.key,f.key]
    assert math.hypot(other.cx-a.stack.pose.x,other.cy-a.stack.pose.y)<math.hypot(f.cx-a.stack.pose.x,f.cy-a.stack.pose.y)
    view=helper.face_arrival_view(original,a.stack.pose,a.coverage,a.stack.grid)
    assert view.face_key==f.key


def test_arrival_metadata_records_actual_ledger_geometry_without_truth():
    import json
    a, f=make_agent()
    step(a)
    event=a._m3_arrival_events[-1]
    assert event['outcome']=='refine' and event['view_reason']=='range'
    assert event['estimated_pose']==[a.stack.pose.x,a.stack.pose.y,a.stack.pose.th]
    assert event['primary_face']==f.key
    assert event['face_geometry']=={'cx':f.cx,'cy':f.cy,'nx':f.nx,'ny':f.ny}
    assert event['original_goal_error_m']==pytest.approx(math.dist(SAVED_STOP,SAVED_VP))
    assert event['refinement_target']==a._ex_arrival_refine['target']
    assert not event['uses_world_truth']
    json.dumps(event)


def test_arrival_metadata_is_bounded_and_cannot_change_view_counts():
    a, f=make_agent()
    c=a._ex_cand
    v=helper.face_arrival_view(c,a.stack.pose,a.coverage,a.stack.grid)
    for i in range(150):
        a._record_face_arrival(float(i),c,v,'diagnostic')
    assert len(a._m3_arrival_events)==128
    assert a._m3_arrival_events[0]['t']==22.
    assert f.views==f.det_n==f.arrive_attempts==0
