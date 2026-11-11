from types import SimpleNamespace
from maicon26.config import Config
from maicon26.types import Pose2D
from maicon26.sim.sensor_policies import SensorPolicyAgent,policy_assignments,make_sensor_policy


def track(x=2.,y=2.,friend=False,p=.5,evidence=1,age=0.):
    return SimpleNamespace(x=x,y=y,friend=friend,p_enemy=p,evidence=evidence,age=lambda now:age)


def shell(policy,tracks=()):
    agent=object.__new__(SensorPolicyAgent)
    agent.policy=policy;agent.cfg=Config();agent.geo=agent.cfg.arena
    agent.stack=SimpleNamespace(pose=Pose2D(1.,2.,0.))
    agent._av_tracks=list(tracks);agent.team='BLUE';agent.slot=2
    agent._policy_patrol_index=0;agent._policy_goal=None
    agent.moves=[];agent.aims=[]
    agent.aborts=[]
    agent.laser=SimpleNamespace(firing=lambda:False,abort=lambda:agent.aborts.append(True))
    agent._apply=lambda *args,**kw:agent.moves.append(args[:2])
    agent._waypoint=lambda now,dt,point:agent.moves.append(point) or False
    agent._aim_enemy=lambda now,dt,tr:agent.aims.append(tr) or True
    return agent


def test_scenario_assignments_preserve_one_real_team_agent_in_mixed_allies():
    assert set(policy_assignments('selfplay').values())=={'agent'}
    assert sum(v=='agent' for v in policy_assignments('mixed_allies').values())==1
    assert policy_assignments('mixed_allies')['B-2']=='agent'
    assert [policy_assignments('vs_hunters')[s] for s in ('R-1','R-2','R-3')]==['hunter']*3


def test_idle_never_chases_or_aims_even_with_visible_enemy():
    a=shell('idle',[track(p=.99,evidence=3)])
    a._policy_step(1.,.1)
    assert a.moves==[(0.,0.)] and not a.aims


def test_hunter_does_not_target_unknown_or_ally():
    for tr in (track(p=.5),track(p=.99,friend=True,evidence=3),track(p=.99,evidence=3,age=3.)):
        a=shell('hunter',[tr]);a._policy_step(1.,.1)
        assert not a.aims


def test_hunter_only_aims_at_current_confirmed_enemy():
    tr=track(p=.95,evidence=3)
    a=shell('hunter',[tr]);a._policy_step(1.,.1)
    assert a.aims==[tr] and not a.moves


def test_losing_enemy_evidence_aborts_active_emission():
    a=shell('hunter',[track(p=.5)])
    a.laser.firing=lambda:True
    a._policy_step(1.,.1)
    assert a.aborts==[True] and not a.aims


def test_evader_uses_unknown_track_for_escape_but_never_fires():
    a=shell('evader',[track()]);a._policy_step(1.,.1)
    assert not a.aims and a.moves[0][0]<a.stack.pose.x


def test_dummy_configuration_is_private_and_never_enables_team_hunting():
    from maicon26.sim.world import World
    from maicon26.sim.robot import SimRobot,SimServer
    cfg=Config();cfg.progress.enabled=True;cfg.arena.validate_layout=False
    w=World(cfg,seed=0);u=w.add_ugv('RED',2)
    a=make_sensor_policy('hunter',SimRobot(w,u),SimServer(w,u),cfg=cfg,name=u.id)
    assert cfg.progress.enabled is True
    assert a.cfg.progress.enabled is False
    assert a.cfg is not cfg
    assert not a.cfg.mission.engage_ugv and not cfg.mission.engage_ugv
