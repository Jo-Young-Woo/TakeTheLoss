"""Sensor-only fast path, soft risk and real worker latency contracts."""
import math
import time
from types import SimpleNamespace

import numpy as np

from maicon26.config import Config
from maicon26.nav.grid import OccupancyGrid
from maicon26.nav.risk_astar import plan
from maicon26.perception.lidar_blobs import BlobDetector
from maicon26.perception.threat_board import ThreatBoard, ThreatTrack, EV_OBJECT
from maicon26.perception.tracks import UgvTracker
from maicon26.mission.threat_field import build_field, bottleneck_mask
from maicon26.mission.risk_worker import RiskFieldWorker
from maicon26.types import Pose2D, Scan


def scene_scan(pose, bodies=(), stamp=0.):
    angles = np.linspace(-math.pi,math.pi,640,endpoint=False)
    r = np.full(640,8.)
    dx,dy = np.cos(angles),np.sin(angles)
    for x,y,radius in bodies:
        px,py=x-pose.x,y-pose.y
        b=px*dx+py*dy
        disc=b*b-(px*px+py*py-radius*radius)
        hit=(disc>=0)&(b>0)
        r[hit]=np.minimum(r[hit],b[hit]-np.sqrt(disc[hit]))
    return Scan(-math.pi,2*math.pi/640,r,stamp,8.)


def setup():
    cfg=Config(); cfg.progress.enabled=True
    grid=OccupancyGrid(cfg.arena); grid.L[:]=grid.L_MIN
    return cfg,grid,Pose2D(1.,2.,0.)


def track(now=0., **kw):
    values=dict(tid=1,x=3.,y=2.,vx=0.,vy=0.,sigma=.06,evidence=EV_OBJECT,
                friend=False,p_enemy=.5,t_last=now,weight=.7)
    values.update(kw)
    return ThreatTrack(**values)


def test_fast_lidar_detects_blocker_without_motion_or_yolo_and_keeps_it():
    cfg,grid,pose=setup(); detector=BlobDetector(cfg)
    detector.update(scene_scan(pose),pose,grid,0.)
    for i in range(1,60):
        sc=scene_scan(pose,[(2.,2.,.13)],i*.1)
        grid.update(sc,pose)
        found=detector.update(sc,pose,grid,i*.1)
        assert len(found)==1
        assert math.hypot(found[0].x-2.,found[0].y-2.)<.15
    assert np.percentile(detector.latency_ms,95)<100.


def test_fast_lidar_confirmed_box_is_subtracted_but_adjacent_robot_survives():
    cfg,grid,pose=setup(); detector=BlobDetector(cfg)
    detector.update(scene_scan(pose),pose,grid,0.)
    box=SimpleNamespace(cx=2.,cy=2.,sx=.4,sy=.4)
    sc=scene_scan(pose,[(2.,2.,.13),(2.,2.7,.13)],.1)
    found=detector.update(sc,pose,grid,.1,static_boxes=[box])
    assert len(found)==1 and found[0].y>2.4


def test_kalman_estimates_velocity_and_expires_missing_returns():
    cfg,grid,pose=setup(); detector=BlobDetector(cfg)
    detector.update(scene_scan(pose),pose,grid,0.)
    for i in range(1,25):
        now=i*.1
        found=detector.update(scene_scan(pose,[(2.+.3*now,2.,.13)],now),pose,grid,now)
    assert len(found)==1 and abs(found[0].vx-.3)<.08 and abs(found[0].vy)<.03
    assert detector.update(scene_scan(pose),pose,grid,4.)==[]


def test_stale_uav_identity_cannot_be_renewed_by_lidar_only():
    cfg,grid,pose=setup(); tracker=UgvTracker(cfg,"BLUE")
    tracker._upsert(3.,2.,0.,"RED",1.,"uav",ident="R-UGV-2")
    board=ThreatBoard(cfg,"BLUE")
    fresh=board.update(tracker,[],.1,pose)[0]
    assert fresh.p_enemy>.8
    from maicon26.perception.lidar_blobs import Blob
    stale=board.update(tracker,[Blob(1,3.,2.,0.,0.,.25,5,7.,0.)],7.,pose)[0]
    assert stale.p_enemy<.8 and stale.t_src["lidar"]==7.
    assert not stale.friend


def test_cost_risk_no_metres_wide_hard_ghost_or_hit_keepout():
    cfg,grid,pose=setup()
    fld=build_field([track()],grid,cfg.avoid,3.,6.,cfg.nav.inflate_hard,
                    risk_cfg=cfg.risk,hit_history=[(2.,2.,6.)])
    assert not fld.body.any()
    assert fld.risk.max()>1.
    blocked,cost=grid.cost_maps(cfg.nav.inflate_hard,cfg.nav.inflate_soft,cfg.nav.soft_weight)
    # There is no alternative corridor: even arbitrarily high risk stays finite.
    blocked[:]=True; iy,_=grid.to_cell(1.,2.); blocked[iy,10:-10]=False
    fld.risk[:]=1e4
    result=plan(grid,blocked,cost,(1.,2.),(5.,2.),field=fld,w_risk=10.)
    assert result is not None


def test_body_clearance_is_physical_not_uncertainty_radius():
    cfg,grid,pose=setup()
    a=build_field([track(sigma=.01)],grid,cfg.avoid,3.,0.,cfg.nav.inflate_hard,risk_cfg=cfg.risk)
    b=build_field([track(sigma=2.)],grid,cfg.avoid,3.,0.,cfg.nav.inflate_hard,risk_cfg=cfg.risk)
    assert np.array_equal(a.body,b.body)
    iy,ix=grid.to_cell(3.,2.); assert a.body[0,iy,ix]
    iy,ix=grid.to_cell(4.,2.); assert not a.body[0,iy,ix]


def test_bottleneck_is_only_a_soft_cost():
    cfg,grid,pose=setup()
    grid.add_static_box((2.,1.,3.,1.8)); grid.add_static_box((2.,2.3,3.,3.))
    m=bottleneck_mask(grid.static,grid.res)
    iy,ix=grid.to_cell(2.5,2.05); assert m[iy,ix]
    tr=track(x=2.5,y=2.05)
    fld=build_field([tr],grid,cfg.avoid,3.,2.,cfg.nav.inflate_hard,risk_cfg=cfg.risk)
    assert fld.risk[0,iy,ix]>0 and not fld.body.any(), "Stale occupancy is a cost"


def test_worker_urgent_scan_does_not_wait_for_5hz_timer():
    cfg,grid,pose=setup(); worker=RiskFieldWorker(cfg)
    try:
        for i in range(12):
            worker.submit([track(i*.1)],grid,i*.1,i,urgent=True)
            limit=time.perf_counter()+1.
            while worker.latest()[0]!=i and time.perf_counter()<limit:
                time.sleep(.001)
            assert worker.latest()[0]==i
        assert not worker.errors
        assert np.percentile(worker.latency_ms,95)<100.
    finally:
        worker.close()


def test_worker_snapshots_inputs_and_clears_expired_field():
    cfg,grid,pose=setup(); worker=RiskFieldWorker(cfg)
    try:
        worker.submit([],grid,1.,1,urgent=True)
        grid.L[:]=4.
        limit=time.perf_counter()+1.
        while worker.latest()[0]!=1 and time.perf_counter()<limit:
            time.sleep(.001)
        assert worker.latest()==(1,None)
    finally:
        worker.close()


def test_long_static_corridor_with_pf_edge_jitter_never_becomes_ugv():
    from maicon26.sim.world import _ray_aabb
    cfg=Config(); cfg.progress.enabled=True
    grid=OccupancyGrid(cfg.arena); detector=BlobDetector(cfg)
    rectangles=[(.8,3.2,cfg.arena.W-.8,cfg.arena.H),(.8,0.,cfg.arena.W-.8,1.8)]
    for i in range(120):
        true=Pose2D(1.+i*.025,2.5,0.)
        scan=scene_scan(true,stamp=i*.1)
        for j,angle in enumerate(scan.angles):
            for rect in rectangles:
                hit=_ray_aabb(true.x,true.y,math.cos(angle),math.sin(angle),rect,scan.range_max)
                if hit is not None:
                    scan.ranges[j]=min(scan.ranges[j],hit[0])
        estimated=Pose2D(true.x+.035*math.sin(i*.31),true.y+.035*math.cos(i*.43),.012*math.sin(i*.2))
        grid.update(scan,estimated)
        found=detector.update(scan,estimated,grid,i*.1)
        assert found==[], (i,found)

def test_same_confirmed_robot_two_sources_has_one_risk_body_and_max_confidence():
    from maicon26.perception.threat_board import deduplicate_tracks,EV_ENEMY
    cfg,grid,pose=setup()
    older=track(now=1.,x=3.,evidence=EV_ENEMY,p_enemy=.99,weight=.99,ident='R-UGV-2',t_src={'uav':1.})
    newer=track(now=1.1,x=3.04,sigma=.04,evidence=EV_ENEMY,p_enemy=.95,weight=.95,t_src={'cam':1.1},tid=2)
    fused=deduplicate_tracks([older,newer])
    assert len(fused)==1 and fused[0].x==newer.x and fused[0].sigma==.04
    assert fused[0].p_enemy==.99 and fused[0].ident=='R-UGV-2'
    assert fused[0].t_src=={'uav':1.,'cam':1.1}
    once=build_field(fused,grid,cfg.avoid,3.,1.1,cfg.nav.inflate_hard,risk_cfg=cfg.risk)
    twice=build_field([older,newer],grid,cfg.avoid,3.,1.1,cfg.nav.inflate_hard,risk_cfg=cfg.risk)
    assert once.risk.max()<twice.risk.max()
    assert np.array_equal(once.body,build_field([fused[0]],grid,cfg.avoid,3.,1.1,cfg.nav.inflate_hard,risk_cfg=cfg.risk).body)


def test_duplicate_fusion_retains_unknown_conflicts_distinct_ids_and_real_neighbours():
    from maicon26.perception.threat_board import deduplicate_tracks,EV_ENEMY,EV_UGV
    enemy=lambda ident=None,x=3.:track(x=x,evidence=EV_ENEMY,p_enemy=.99,ident=ident)
    for pair in ([enemy('R-UGV-1'),enemy('R-UGV-2')],
                 [enemy(),enemy(x=3.3)],
                 [track(),enemy()],
                 [enemy(),track(friend=True,p_enemy=.01,evidence=EV_UGV)]):
        assert len(deduplicate_tracks(pair))==2
    assert len(deduplicate_tracks([enemy(x=3.),enemy(x=3.14),enemy(x=3.28)]))==2


def test_board_fusion_is_explicit_candidate_mode_and_never_double_votes():
    cfg,grid,pose=setup(); tracker=UgvTracker(cfg,'BLUE')
    a=tracker._upsert(3.,2.,1.,'RED',5.,'uav',ident='R-UGV-2')
    from maicon26.perception.tracks import Track
    b=Track(2,3.04,2.,1.1,team_votes={'RED':5.},last_cam_t=1.1)
    tracker.tracks.append(b)
    board=ThreatBoard(cfg,'BLUE')
    assert len(board.update(tracker,[],1.2,pose))==2
    out=board.update(tracker,[],1.2,pose,use_risk_weights=True)
    assert len(out)==1 and out[0].x==3.04
    assert a.team_votes['RED']==5. and b.team_votes['RED']==5.

def test_fresh_opposite_colour_overrides_inferred_identity_to_unknown_only():
    cfg,grid,pose=setup(); tracker=UgvTracker(cfg,'BLUE')
    tr=tracker._upsert(3.,2.,1.,'RED',20.,'uav',ident='R-UGV-2')
    # Even one fresh confident opposite colour must prevent a shot. Old votes
    # and an inferred birth-base ID are not stronger than contradictory pixels.
    tracker._upsert(3.02,2.,1.2,'BLUE',.9,'cam')
    board=ThreatBoard(cfg,'BLUE')
    out=board.update(tracker,[],1.3,pose,use_risk_weights=True)
    assert len(out)==1 and out[0].p_enemy==.5 and not out[0].friend
    assert out[0].evidence!=3 and out[0].iff_conflict
    assert out[0].ident=='R-UGV-2' and tr.ident=='R-UGV-2'
    assert board.ident_conflicts[0]['reason']=='fresh_colour_conflicts_with_identity'
    # Renewing a LiDAR position cannot clear the colour disagreement.
    from maicon26.perception.lidar_blobs import Blob
    out=board.update(tracker,[Blob(1,3.03,2.,0.,0.,.25,4,1.4)],1.4,pose,use_risk_weights=True)
    assert out[0].iff_conflict and out[0].p_enemy==.5


def test_v2_identity_policy_is_preserved_and_new_labels_do_not_retrodate():
    cfg,grid,pose=setup(); cfg.progress.enabled=False
    tracker=UgvTracker(cfg,'BLUE')
    tr=tracker._upsert(3.,2.,2.,'RED',1.,'uav',ident='R-UGV-2')
    tracker._upsert(3.,2.,2.1,'BLUE',.9,'cam')
    board=ThreatBoard(cfg,'BLUE')
    assert board.update(tracker,[],2.2,pose)[0].p_enemy>.99
    tracker._upsert(3.,2.,1.9,'RED',1.,'cam')
    assert tr.source_team_labels['cam']==('BLUE',2.1,.9)

def test_disputed_identity_does_not_revive_when_conflicting_colour_ages_out():
    cfg,grid,pose=setup(); tracker=UgvTracker(cfg,'BLUE')
    tracker._upsert(3.,2.,1.,'RED',2.,'uav',ident='R-UGV-2')
    tracker._upsert(3.,2.,1.2,'BLUE',.9,'cam')
    board=ThreatBoard(cfg,'BLUE')
    assert board.update(tracker,[],1.3,pose,use_risk_weights=True)[0].iff_conflict
    from maicon26.perception.lidar_blobs import Blob
    out=board.update(tracker,[Blob(1,3.,2.,0.,0.,.25,4,6.4)],6.4,pose,use_risk_weights=True)
    assert out[0].p_enemy==.5 and out[0].iff_conflict
    tracker._upsert(3.,2.,6.5,'RED',.9,'cam')
    out=board.update(tracker,[],6.6,pose,use_risk_weights=True)
    assert out[0].p_enemy>.8 and not out[0].iff_conflict
def test_m6_disputed_identity_has_no_confident_evidence_and_v2_is_unchanged():
    from maicon26.mission.threat import evidence_from_tracks,score
    cfg,grid,pose=setup();tracker=UgvTracker(cfg,'BLUE')
    tr=tracker._upsert(3.,2.,1.,'RED',20.,'uav',ident='R-UGV-2')
    tr.hits_by_us=2;tr.travel=3.;tr.n_obs=9
    tr.identity_disputed=True
    evidence=evidence_from_tracks(tracker,'BLUE')
    assert set(evidence)=={'R-UGV-1','R-UGV-2','R-UGV-3'}
    assert evidence['R-UGV-2'].identity_uncertain and not evidence['R-UGV-2'].seen
    assert evidence['R-UGV-2'].hits_est==0. and evidence['R-UGV-2'].n_obs==0
    assert len(score(evidence))==3
    cfg.progress.enabled=False
    legacy=evidence_from_tracks(tracker,'BLUE')['R-UGV-2']
    assert legacy.seen and legacy.hits_est==2 and not legacy.identity_uncertain

def test_delayed_uav_cannot_relabel_old_position_as_a_fresh_camera_body():
    cfg,grid,pose=setup();tracker=UgvTracker(cfg,'BLUE')
    tr=tracker._upsert(3.,2.,10.,'RED',1.,'uav',ident='R-UGV-2')
    tracker._upsert(3.2,2.,11.,'RED',1.,'cam')
    vx=tr.vx;travel=tr.travel
    delayed=tracker._upsert(3.05,2.,10.5,'RED',1.,'uav')
    assert delayed is tr and len(tracker.tracks)==1
    assert (tr.x,tr.y,tr.t,tr.vx,tr.travel)==(3.2,2.,11.,vx,travel)
    assert tr.last_uav_t==10.5 and tr.last_cam_t==11.
    before=tr.n_obs
    tracker._upsert(3.1,2.,10.2,'BLUE',9.,'uav')
    assert tr.n_obs==before and tr.last_uav_t==10.5
    assert tr.source_team_labels['uav']==('RED',10.5,1.)
    field=ThreatBoard(cfg,'BLUE').update(tracker,[],11.1,pose,use_risk_weights=True)
    assert field[0].x==3.2 and field[0].t_src['uav']==10.5


def test_five_second_old_uav_remains_old_even_with_fresh_scan_position():
    cfg,grid,pose=setup();tracker=UgvTracker(cfg,'BLUE')
    tracker._upsert(3.,2.,1.,'RED',2.,'uav',ident='R-UGV-2')
    from maicon26.perception.lidar_blobs import Blob
    board=ThreatBoard(cfg,'BLUE')
    fresh=board.update(tracker,[Blob(1,3.,2.,0.,0.,.25,4,7.)],7.,pose,use_risk_weights=True)
    assert fresh[0].t_last==7. and fresh[0].t_src['uav']==1.
    assert fresh[0].p_enemy<.9
