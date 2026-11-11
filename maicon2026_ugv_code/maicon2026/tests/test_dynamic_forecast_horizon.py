"""Finite forecasts guide future routes without relaxing immediate clearance."""
import math
import numpy as np
from maicon26.config import Config
from maicon26.nav.grid import OccupancyGrid
from maicon26.nav.risk_astar import plan
from maicon26.perception.threat_board import ThreatTrack, EV_OBJECT
from maicon26.mission.threat_field import build_field, body_hit


def scene(candidate=True, stamp=0.):
    cfg=Config()
    grid=OccupancyGrid(cfg.arena)
    grid.L[:]=grid.L_MIN
    tr=ThreatTrack(tid=1,x=3.,y=2.,vx=0.,vy=0.,sigma=.06,
        evidence=EV_OBJECT,friend=False,p_enemy=.5,t_last=stamp,weight=.7)
    field=build_field([tr],grid,cfg.avoid,3.,0.,.17,
        risk_cfg=cfg.risk if candidate else None,min_clearance_m=.30)
    return cfg,grid,field


def test_current_physical_body_and_clearance_remain_hard():
    cfg,grid,field=scene()
    assert body_hit(field,grid,3.,2.,0.)
    assert body_hit(field,grid,3.5,2.,0.)
    assert not body_hit(field,grid,3.8,2.,0.)
    assert body_hit(field,grid,3.,2.,.5)


def test_distant_body_forecast_is_not_frozen_into_permanent_corridor_wall():
    cfg,grid,field=scene()
    ys=grid.oy+(np.arange(grid.ny)+.5)*grid.res
    blocked=np.broadcast_to((np.abs(ys-2.)>.12)[:,None],grid.L.shape).copy()
    cost=np.ones_like(grid.L)
    # Arrival at x=3 takes >5s, beyond the evidence horizon. Replanning and
    # actual-clearance guards must decide movement as the encounter approaches.
    result=plan(grid,blocked,cost,(1.,2.),(5.,2.),.35,field,.3,1.5)
    assert result is not None
    assert not body_hit(field,grid,3.,2.,10.)
    iy,ix=grid.to_cell(3.,2.)
    assert field.risk[-1,iy,ix]>0., 'Future exposure remains a finite cost'
    # Replanning immediately beside the same stationary body cannot cross it.
    assert plan(grid,blocked,cost,(2.35,2.),(2.8,2.),.35,field,.3,1.5) is None


def test_prediction_age_shortens_hard_horizon_and_legacy_is_unchanged():
    cfg,grid,field=scene(stamp=-1.)
    assert body_hit(field,grid,3.,2.,0.)
    assert not body_hit(field,grid,3.,2.,.8)
    _,grid,legacy=scene(candidate=False)
    assert math.isinf(legacy.body_horizon_s)
    assert body_hit(legacy,grid,3.,2.,10.)


def test_model_corner_radius_preserves_clearance_for_both_robots():
    cfg=Config()
    grid=OccupancyGrid(cfg.arena,res=.01)
    tr=ThreatTrack(tid=1,x=3.,y=2.,vx=0.,vy=0.,sigma=.01,
        evidence=EV_OBJECT,friend=False,p_enemy=.5,t_last=0.,weight=.7)
    radius=math.hypot(cfg.robot.length,cfg.robot.width)/2.
    field=build_field([tr],grid,cfg.avoid,3.,0.,cfg.nav.inflate_hard,
        risk_cfg=cfg.risk,body_radius_m=radius,min_clearance_m=.30)
    assert body_hit(field,grid,3.+2*radius+.27,2.,0.)
    assert not body_hit(field,grid,3.+2*radius+.33,2.,0.)
