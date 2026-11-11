"""Route continuity cannot preserve a waypoint in a newly observed wall."""
from types import SimpleNamespace
from unittest.mock import patch
import numpy as np
import pytest
from test_deadlock_integration import agent
from maicon26.nav import risk_astar
from maicon26.mission.manager import Mode


def route_fixture(cached_time,obstacle=True):
    a=agent();grid=a.stack.grid;grid.L[:]=grid.L_MIN
    a.stack.pose.x,a.stack.pose.y=1.,1.
    goal=(3.,1.);old=(1.8,1.)
    if obstacle:grid.add_static_box((1.75,.95,1.85,1.05))
    blocked,cost=grid.cost_maps(a.cfg.nav.inflate_hard,a.cfg.nav.inflate_soft,a.cfg.nav.soft_weight)
    a._av_ctx=SimpleNamespace(blocked=blocked,cost=cost,grid=grid,slack_s=100.)
    a._av_tracks=[];a._field=None
    a._rc={'goal':goal,'t':cached_time,'carrot':old}
    result=risk_astar.PlanResult([(1.,1.),(1.,1.4),(2.,1.4),goal],2.5,0.,0.,0.)
    return a,goal,old,result


@pytest.mark.parametrize('cached_time',[0.,9.99])
def test_new_physical_wall_invalidates_both_recent_and_hysteresis_waypoint(cached_time):
    a,goal,old,result=route_fixture(cached_time)
    try:
        with patch.object(risk_astar,'plan',return_value=result) as planner:
            selected=a._risk_carrot(10.,goal,1.)
        assert selected!=old and planner.called
        assert not a._av_ctx.blocked[a.stack.grid.to_cell(*selected)]
    finally:a.close_deadlock()


def test_free_recent_waypoint_retains_continuity_without_unneeded_replan():
    a,goal,old,result=route_fixture(9.99,obstacle=False)
    try:
        with patch.object(risk_astar,'plan',return_value=result) as planner:
            assert a._risk_carrot(10.,goal,1.)==old
        assert not planner.called
    finally:a.close_deadlock()


def test_outside_map_cached_waypoint_is_not_returned():
    a,goal,_,result=route_fixture(9.99,obstacle=False)
    a._rc['carrot']=(10.,10.)
    try:
        with patch.object(risk_astar,'plan',return_value=result):
            selected=a._risk_carrot(10.,goal,1.)
        assert a.stack.grid.in_bounds(*a.stack.grid.to_cell(*selected))
    finally:a.close_deadlock()


def test_stage_uses_current_observed_map_when_avoidance_cache_is_old():
    a=agent();a.mode=Mode.STAGE;a._av_tracks=[]
    a.stack.grid.L[:]=a.stack.grid.L_MIN
    a._av_ctx=SimpleNamespace(blocked=np.zeros_like(a.stack.grid.L,dtype=bool))
    point=a._stage_point();x,y=point
    a.stack.grid.add_static_box((x-.1,y-.1,x+.1,y+.1))
    try:
        assert a._stage_point_occupied(10.,point)
        assert a._mandatory_navigation_goal(10.,point)!=point
    finally:a.close_deadlock()
