import heapq
import math

import numpy as np
import pytest

from maicon26.config import Config
from maicon26.nav.grid import OccupancyGrid
from maicon26.nav.dstar_lite import DStarLite
from maicon26.nav import risk_dstar
from maicon26.mission.threat_field import build_field
from maicon26.perception.threat_board import ThreatTrack, EV_OBJECT


def scene(res=.2):
    g = OccupancyGrid(Config().arena, res=res)
    return g, g.static.copy(), np.ones_like(g.L, dtype=float)


def oracle(grid, blocked, cost, start, goal):
    # Independent Dijkstra oracle, including no corner cutting.
    s, target = grid.to_cell(*start), grid.to_cell(*goal)
    if blocked[s] or blocked[target]:
        return math.inf
    heap, distance = [(0., s)], {s: 0.}
    while heap:
        value, (y, x) = heapq.heappop(heap)
        if value != distance[(y, x)]:
            continue
        if (y, x) == target:
            return value
        for dy in (-1, 0, 1):
            for dx in (-1, 0, 1):
                yy, xx = y+dy, x+dx
                if not (dy or dx) or not grid.in_bounds(yy, xx) or blocked[yy, xx]:
                    continue
                if dy and dx and (blocked[y, xx] or blocked[yy, x]):
                    continue
                nxt = value+math.hypot(dx, dy)*cost[yy, xx]
                if nxt < distance.get((yy, xx), math.inf):
                    distance[(yy, xx)] = nxt
                    heapq.heappush(heap, (nxt, (yy, xx)))
    return math.inf


def path_cost(g, path, cost):
    return sum(math.dist(a,b)/g.res*cost[g.to_cell(*b)] for a,b in zip(path,path[1:]))


def test_incremental_repairs_match_dijkstra_for_insert_remove_cost_and_moving_start():
    g,b,c=scene(); rng=np.random.default_rng(71); planner=DStarLite()
    s,t=(.7,2.3),(5.3,2.3)
    for tick in range(24):
        y,x=rng.integers(2,g.ny-2),rng.integers(2,g.nx-2)
        b[y-1:y+2,x-1:x+2]=tick%3==0
        c[y-2:y+3,x-2:x+3]=1.+tick%4
        if tick==12:s=(1.3,1.7)
        b[g.to_cell(*s)]=b[g.to_cell(*t)]=False
        d={};p=planner.plan(g,b,c,s,t,snap_endpoints=False,diagnostics=d)
        optimal=oracle(g,b,c,s,t)
        assert (p is None)==(not math.isfinite(optimal))
        if p:assert path_cost(g,p,c)==pytest.approx(optimal)
        assert d['reused']==(tick>0)
    assert planner.resets==1


def test_unchanged_map_reuses_tree_and_budgeted_repair_resumes():
    g,b,c=scene();p=DStarLite();s,t=(.7,2.3),(5.3,2.3);d={}
    assert p.plan(g,b,c,s,t,max_expand=1,diagnostics=d) is None
    assert d['reason']=='expansion_limit'
    assert p.plan(g,b,c,s,t,diagnostics=d)
    assert p.resets==1
    assert p.plan(g,b,c,(.9,2.3),t,diagnostics=d)
    assert d['expanded']==0 and d['changed_vertices']==0
    b[:,g.to_cell(3.,2.)[1]]=True
    assert p.plan(g,b,c,s,t,diagnostics=d) is None
    assert d['reason']=='frontier_exhausted'
    b[:,g.to_cell(3.,2.)[1]]=False
    assert p.plan(g,b,c,s,t,diagnostics=d)
    assert p.resets==1 and d['changed_vertices']>0


def test_diagonal_cannot_pass_blocked_corner_and_goal_change_resets():
    g,b,c=scene(); b[:]=True
    a=(4,4);z=(5,5);b[a]=b[z]=False
    p=DStarLite()
    assert p.plan(g,b,c,g.to_world(*a),g.to_world(*z)) is None
    b[4,5]=False
    assert p.plan(g,b,c,g.to_world(*a),g.to_world(4,5))
    assert p.resets==2


def field_scene():
    cfg=Config();g,b,c=scene(.05);start=(2.,2.);goal=(4.5,2.)
    radius=math.hypot(cfg.robot.length,cfg.robot.width)/2
    tr=ThreatTrack(9,2.,1.45,0.,0.,.02,EV_OBJECT,False,.9,0.)
    f=build_field([tr],g,cfg.avoid,cfg.laser.max_range,0.,cfg.nav.inflate_hard,
                  risk_cfg=cfg.risk,body_radius_m=radius,min_clearance_m=.30,use_visibility=False)
    return g,b,c,start,goal,f


def test_inside_margin_escapes_outward_without_crossing_physical_body():
    g,b,c,s,t,f=field_scene();d={}
    assert f.body[0][g.to_cell(*s)]
    p=risk_dstar.plan(g,b,c,s,t,field=f,diagnostics=d)
    assert p is not None and p.egress
    px,py,collision,margin=f.clearance_disks[0]
    for a,z in zip(p.path,p.path[1:]):
        for u in np.linspace(0,1,11):
            assert math.dist((a[0]+u*(z[0]-a[0]),a[1]+u*(z[1]-a[1])),(px,py))>collision
        if math.dist(a,(px,py))<=margin:
            assert (a[0]-px)*(z[0]-a[0])+(a[1]-py)*(z[1]-a[1])>=-1e-9
    again={};assert risk_dstar.plan(g,b,c,s,t,field=f,diagnostics=again)
    assert again['reused'] and again['expanded']==0


def test_escape_does_not_open_wall_other_body_or_unknown_dynamic_mask():
    g,b,c,s,t,f=field_scene()
    b[:]=True;b[g.to_cell(*s)]=False
    assert risk_dstar.plan(g,b,c,s,t,field=f) is None
    g,b,c,s,t,f=field_scene()
    f.clearance_disks=()
    assert risk_dstar.plan(g,b,c,s,t,field=f) is None
    g,b,c,s,t,f=field_scene()
    # A different UGV's physical body overlaps the start: no escape permission.
    f.clearance_disks += ((s[0],s[1],.38,.65),)
    assert risk_dstar.plan(g,b,c,s,t,field=f) is None


def test_control_policy_default_runs_every_tick(monkeypatch):
    from test_deadlock_integration import agent
    a=agent()
    try:
        stamps=np.arange(0.,1.,.05)
        assert all(a._advance_decision_clock(float(t)) for t in stamps)
        assert a.stats['decision_updates']==len(stamps)
    finally:a.close_deadlock()


def test_new_observation_permits_recovery_after_point_two_seconds():
    from test_deadlock_integration import agent
    from test_mandatory_goal_recovery import clear_observed
    from maicon26.mission.manager import Mode
    a=agent();a.mode=Mode.GOAL;a.m5_base='R-2'
    try:
        a.stack.grid.L[:]=0.
        original=a.geo.base_center('R-2')
        a.progress.retry_at[a._goal_key(original)]=43.
        assert a._mandatory_navigation_goal(13.,original) is None
        clear_observed(a)
        assert a._mandatory_navigation_goal(13.1,original) is None
        assert a._mandatory_navigation_goal(13.21,original) is not None
    finally:a.close_deadlock()
