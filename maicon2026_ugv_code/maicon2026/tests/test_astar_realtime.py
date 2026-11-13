import math
import numpy as np
import pytest

from maicon26.nav.astar_search import AStar
from maicon26.nav.dstar_lite import DStarLite
from maicon26.nav import risk_astar_realtime, risk_dstar
from test_dstar_lite import scene, oracle, path_cost, field_scene


def test_fresh_astar_matches_dijkstra_across_map_and_cost_changes():
    grid, blocked, cost = scene()
    planner = AStar()
    rng = np.random.default_rng(71)
    start, goal = (.7, 2.3), (5.3, 2.3)
    for tick in range(24):
        y, x = rng.integers(2,grid.ny-2), rng.integers(2,grid.nx-2)
        blocked[y-1:y+2,x-1:x+2] = tick % 3 == 0
        cost[y-2:y+3,x-2:x+3] = 1.+tick % 4
        if tick == 12:
            start = (1.3, 1.7)
        blocked[grid.to_cell(*start)] = blocked[grid.to_cell(*goal)] = False
        diag = {}
        route = planner.plan(grid,blocked,cost,start,goal,snap_endpoints=False,diagnostics=diag)
        expected = oracle(grid,blocked,cost,start,goal)
        assert (route is None) == (not math.isfinite(expected))
        if route:
            assert path_cost(grid,route,cost) == pytest.approx(expected)
        assert diag['algorithm'] == 'astar' and not diag['reused']


def test_replan_does_not_keep_an_old_path_when_map_changes_or_budget_expires():
    g,b,c=scene();p=AStar();s,t=(.7,2.3),(5.3,2.3);d={}
    assert p.plan(g,b,c,s,t)
    b[:,g.to_cell(3.,2.)[1]]=True
    assert p.plan(g,b,c,s,t,diagnostics=d) is None
    assert d['reason']=='frontier_exhausted'
    b[:,g.to_cell(3.,2.)[1]]=False
    assert p.plan(g,b,c,s,t,max_expand=1,diagnostics=d) is None
    assert d['reason']=='expansion_limit'
    assert p.plan(g,b,c,s,t,diagnostics=d)
    assert not d['reused'] and d['expanded']>0


def test_body_rotation_mask_and_blocked_corner_prevent_diagonal_shortcuts():
    g,b,c=scene();b[:]=True
    b[4,4]=b[5,5]=False;p=AStar()
    s,t=g.to_world(4,4),g.to_world(5,5)
    assert p.plan(g,b,c,s,t) is None
    b[4,5]=b[5,4]=False
    route=p.plan(g,b,c,s,t,open_mask=np.zeros_like(b))
    assert len(route)==3
    assert len(p.plan(g,b,c,s,t))==2


def test_astar_preserves_same_escape_constraints_and_cost_as_dstar():
    g,b,c,s,t,f=field_scene()
    da,dd={},{}
    a=risk_astar_realtime.plan(g,b,c,s,t,field=f,diagnostics=da)
    d=risk_dstar.plan(g,b,c,s,t,field=f,diagnostics=dd)
    assert a and d and a.egress and d.egress
    _,cost,_,_=risk_astar_realtime.navigation_layers(g,b,c,s,f,3.,1.5)
    # The common first cell uses the measured start; compare abstract grid costs.
    agrid=[g.to_world(*g.to_cell(*point)) for point in a.path]
    dgrid=[g.to_world(*g.to_cell(*point)) for point in d.path]
    assert path_cost(g,agrid,cost)==pytest.approx(path_cost(g,dgrid,cost))
    px,py,collision,margin=f.clearance_disks[0]
    for first,second in zip(a.path,a.path[1:]):
        for u in np.linspace(0,1,11):
            point=(first[0]+u*(second[0]-first[0]),first[1]+u*(second[1]-first[1]))
            assert math.dist(point,(px,py))>collision
        if math.dist(first,(px,py))<=margin:
            assert (first[0]-px)*(second[0]-first[0])+(first[1]-py)*(second[1]-first[1])>=-1e-9
    assert da['algorithm']=='astar' and not da['reused']


@pytest.mark.parametrize('obstruction',['wall','unknown_body','overlapping_ugv'])
def test_escape_cannot_clear_real_obstacles(obstruction):
    g,b,c,s,t,f=field_scene()
    if obstruction=='wall':
        b[:]=True;b[g.to_cell(*s)]=False
    elif obstruction=='unknown_body':
        f.clearance_disks=()
    else:
        f.clearance_disks+=((s[0],s[1],.38,.65),)
    assert risk_astar_realtime.plan(g,b,c,s,t,field=f) is None


def test_production_navigation_and_exploration_work_with_dstar_disabled(monkeypatch):
    from test_risk_waypoint_validity import route_fixture
    from maicon26.mission.explore_online import Cand
    def forbidden(*args,**kwargs):
        pytest.fail('Production requested D* Lite')
    monkeypatch.setattr(DStarLite,'plan',forbidden)
    a,goal,_,_=route_fixture(0.,obstacle=False);a._rc={}
    try:
        assert a._risk_carrot(10.,goal,1.) is not None
        assert a.stats['route_planner']['algorithm']=='astar'
        a.stack.navigator.set_goal(goal)
        assert a.stack.navigator._plan(a.stack.pose,10.)
        ctx=a._av_ctx
        candidate=Cand('frontier',goal,0.,'look',gain_u=2.,t_obs=1.)
        assert a.explorer.evaluate([candidate],a.stack.pose,ctx.grid,ctx.blocked,ctx.cost,10.,(4.,2.))
    finally:
        a.close_deadlock()
