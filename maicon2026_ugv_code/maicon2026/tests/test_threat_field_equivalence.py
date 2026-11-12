"""Compare the pre-optimization implementation without weakening safety bounds."""
import importlib.util
from pathlib import Path
import sys
import numpy as np
import pytest
from maicon26.config import Config
from maicon26.nav.grid import OccupancyGrid
from maicon26.perception.threat_board import ThreatTrack
from maicon26.mission.threat_field import build_field,_ray_offsets
spec=importlib.util.spec_from_file_location('maicon26.mission._scalar_reference',Path(__file__).parent/'fixtures/threat_field_scalar_reference.py')
reference=importlib.util.module_from_spec(spec);sys.modules[spec.name]=reference;spec.loader.exec_module(reference)

@pytest.mark.parametrize('count',[0,1,6,18])
@pytest.mark.parametrize('visibility',[False,True])
@pytest.mark.parametrize('number',[float,np.float64])
def test_optimized_field_is_bitwise_equivalent(count,visibility,number):
    cfg=Config();rng=np.random.default_rng(450+count)
    grid=OccupancyGrid(cfg.arena)
    grid.L[:]=rng.choice([-2.,2.],size=grid.L.shape,p=[.90,.10])
    tracks=[]
    for i in range(count):
        x,y=map(number,rng.uniform(-.3,6.,2))
        vx,vy=map(number,rng.uniform(-.3,.3,2))
        if i%3==0:vx=vy=number(0.)
        tracks.append(ThreatTrack(i,x,y,vx,vy,.06,3,False,.95,number(10-rng.uniform(0,6)),weight=number(rng.uniform(0,1))))
    cfg.risk.cost_table={'exposure':.731,'hit_history':1.273,'bottleneck':1.333,'closing':.427}
    kw=dict(use_visibility=visibility,risk_cfg=cfg.risk,min_clearance_m=.30,hit_history=((1.,2.,8.),),ego_velocity=(.035,.07),body_radius_m=.17)
    expected=reference.build_field(tracks,grid,cfg.avoid,3.,10.,.13,7,**kw)
    actual=build_field(tracks,grid,cfg.avoid,3.,10.,.13,7,**kw)
    for name in ('body','risk','close','mind'):
        np.testing.assert_array_equal(getattr(actual,name),getattr(expected,name))
    assert actual.body_horizon_s==expected.body_horizon_s


def test_visibility_cache_never_reuses_observed_map_or_old_tracks():
    cfg=Config();grid=OccupancyGrid(cfg.arena);grid.L[:]=-2.
    track=ThreatTrack(1,2.,2.,0.,0.,.06,3,False,.95,10.,weight=1.)
    before=build_field([track],grid,cfg.avoid,3.,10.,.13,risk_cfg=cfg.risk)
    grid.L[:,48:52]=4.
    after=build_field([track],grid,cfg.avoid,3.,10.,.13,risk_cfg=cfg.risk)
    assert np.any(after.risk<before.risk)
    assert _ray_offsets.cache_info().maxsize==16
    for value in _ray_offsets(.05,3.,180):assert not value.flags.writeable

@pytest.mark.parametrize('visibility',[False,True])
def test_legacy_visibility_is_bitwise_unchanged(visibility):
    cfg=Config();grid=OccupancyGrid(cfg.arena);grid.L[:]=-2.;grid.L[20:35,30:45]=3.
    tracks=[ThreatTrack(i,.9+i*.7,1.3,.11,-.07,.06,3,False,.95,10.) for i in range(6)]
    a=reference.build_field(tracks,grid,cfg.avoid,3.,10.,.13,use_visibility=visibility)
    b=build_field(tracks,grid,cfg.avoid,3.,10.,.13,use_visibility=visibility)
    for name in ('body','risk','close','mind'):np.testing.assert_array_equal(getattr(a,name),getattr(b,name))
