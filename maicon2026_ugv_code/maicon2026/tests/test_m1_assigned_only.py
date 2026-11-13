"""M1-only exploration restricts goals, not perception, transit or submission rules."""
from types import SimpleNamespace
import pytest
from maicon26.config import Config
from maicon26.link.schema import server_sector, scoring_sectors
from maicon26.mission.explore_online import OnlineExplorer, Cand
from maicon26.mission.manager import Agent
from maicon26.types import Pose2D
from test_exhaustive_scoring_route import scene


def setup_scene():
    cfg,cov,grid=scene(((4.2,4.2),(4.2,2.4),(4.2,.8),(3.,2.4)))
    cfg.explore.m1_assigned_only=True
    ex=OnlineExplorer(cfg,cov)
    return cfg,cov,grid,ex


def test_all_candidate_types_stay_within_two_named_targets_and_old_goal_is_dropped():
    cfg,cov,grid,ex=setup_scene()
    ex.m1_sectors={'R-S1','R-S2','CM'}
    blocked,_=cov.planning_cost_maps(grid)
    outside=next(b for b in cov.boxes.values() if server_sector(cov.box_sector(b))=='R-S3')
    old=Cand('face',(4.9,.8),3.14,'look',[outside.faces[0].key],primary_face=outside.faces[0].key,gain_f=5.)
    candidates=ex.build(Pose2D(3.8,3.,0.),grid,blocked,10.,current=old,team='BLUE')
    assert candidates
    assert {server_sector(ex.candidate_sector(c)) for c in candidates} <= {'R-S1','R-S2'}
    assert not any(c.incumbent for c in candidates)
    for c in candidates:
        for key in c.faces:
            assert server_sector(cov.box_sector(cov.boxes[key[0]])) in {'R-S1','R-S2'}
    diag=ex.diagnostics['last_build']
    assert diag['exploration_sectors']==['R-S1','R-S2']
    assert diag['official_m3_scoring_sectors']==list(scoring_sectors('BLUE'))


@pytest.mark.parametrize('sectors',[set(),{'R-S1'},{'R-S1','R-S2','R-S3'},{'invalid','CM'}])
def test_missing_or_ambiguous_assignment_does_not_expand_to_the_entire_arena(sectors):
    _,cov,grid,ex=setup_scene()
    ex.m1_sectors=sectors
    blocked,_=cov.planning_cost_maps(grid)
    assert ex.build(Pose2D(3.8,3.,0.),grid,blocked,10.,team='BLUE')==[]


def test_completion_and_hunt_budget_use_assigned_sectors_without_changing_scoring():
    a=Agent.__new__(Agent)
    a.cfg,a.coverage,_,_=setup_scene()
    a.team='BLUE'
    a.cmd=SimpleNamespace(sectors=['R-S1','R-S2','COMMON'])
    seen=[]
    def progress(sectors):
        seen.append(set(sectors))
        return {'complete':set(sectors)=={'R-S1','R-S2'}}
    a.coverage.sector_progress=progress
    a.stats={'mode_time':{'HUNT':20.,'STRIKE':40.}}
    assert a._coverage_sectors()=={'R-S1','R-S2'}
    assert not a._continue_scoring_coverage(240.)
    assert not a._coverage_defers_m4()
    assert not a._coverage_defers_m2()
    assert all(s=={'R-S1','R-S2'} for s in seen)
    assert set(scoring_sectors('BLUE'))=={'R-S1','R-S2','R-S3','CM'}


def test_real_and_default_profiles_enable_requested_scope():
    from pathlib import Path
    root=Path(__file__).resolve().parents[1]
    for name in ('default.json','waveshare_real.json'):
        assert Config.load(str(root/'config'/name)).explore.m1_assigned_only
