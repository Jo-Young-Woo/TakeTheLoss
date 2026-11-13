from types import SimpleNamespace
import numpy as np
from maicon26.config import Config
from maicon26.nav.grid import OccupancyGrid
from maicon26.nav import risk_astar
from maicon26.nav.route_diagnostics import blockage_evidence, reachable_mask


def scene():
    grid=OccupancyGrid(Config().arena,res=.1)
    blocked=grid.static.copy()
    return grid,blocked,np.ones(blocked.shape),(.5,2.),(5.,2.)


def test_diagnostics_do_not_change_a_star_route():
    g,b,c,s,t=scene()
    plain=risk_astar.plan(g,b,c,s,t)
    d={}
    traced=risk_astar.plan(g,b,c,s,t,diagnostics=d)
    assert traced==plain
    assert d['reason']=='found' and d['expanded']>0


def test_actual_map_cut_and_clearance_cut_are_distinct():
    g,b,c,s,t=scene()
    wall=b.copy();wall[:,g.to_cell(3.,2.)[1]]=True
    d={}
    assert risk_astar.plan(g,wall,c,s,t,diagnostics=d) is None
    assert d['reason']=='frontier_exhausted'
    assert blockage_evidence(g,wall,wall,s,t,None,d)['classification']=='observed_map_disconnection'
    assert blockage_evidence(g,wall,b,s,t,None,d)['classification']=='clearance_or_policy_disconnection'


def test_dynamic_forecast_failure_is_not_reported_as_static_wall():
    g,b,c,s,t=scene()
    body=np.zeros((1,*b.shape),bool);body[:,:,g.to_cell(3.,2.)[1]]=True
    f=SimpleNamespace(K=1,dtau=1.,body=body,risk=np.zeros_like(body,float),
                      close=np.zeros_like(body,float),body_horizon_s=float('inf'))
    d={}
    assert risk_astar.plan(g,b,c,s,t,field=f,diagnostics=d) is None
    e=blockage_evidence(g,b,b,s,t,f,d)
    assert e['static_connected'] and not e['current_body_connected']
    assert e['classification']=='dynamic_forecast_failure'


def test_budget_exhaustion_is_not_called_no_path():
    g,b,c,s,t=scene();d={}
    assert risk_astar.plan(g,b,c,s,t,max_expand=1,diagnostics=d) is None
    assert blockage_evidence(g,b,b,s,t,None,d)['classification']=='expansion_limit'
    assert risk_astar.plan(g,b,c,s,t) is not None


def test_connectivity_does_not_cut_diagonal_corners():
    b=np.ones((3,3),bool);b[0,0]=b[1,1]=False
    assert reachable_mask(b,(0,0)).sum()==1


def test_no_free_endpoint_is_explicit():
    g,b,c,s,t=scene();b[:]=True;d={}
    assert risk_astar.plan(g,b,c,s,t,diagnostics=d) is None
    assert d['reason']=='no_free_endpoint'
