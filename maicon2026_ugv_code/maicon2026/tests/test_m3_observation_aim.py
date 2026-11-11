"""Arrival error must not change the planned photograph or consume its dwell."""
import math
from types import SimpleNamespace

import pytest

from maicon26.config import Config
from maicon26.mission.explore_online import Cand
from maicon26.mission.manager import Agent
from maicon26.types import Pose2D


def observing_agent(primary=True):
    intended = SimpleNamespace(key=(1, 2), cx=2.8, cy=3.3)
    nearer = SimpleNamespace(key=(2, 3), cx=2.7, cy=2.1)
    faces = {f.key: f for f in (nearer, intended)}
    pending = set(faces)
    outputs, observed, arrivals = [], [], []
    agent = Agent.__new__(Agent)
    agent.cfg = Config()
    agent.cfg.explore.m3_observe_s = 2.
    agent.stack = SimpleNamespace(pose=Pose2D(2.41, 2.42, 0.))
    agent.coverage = SimpleNamespace(
        get_face=faces.get, picture_pending=lambda f: f.key in pending,
        mark_arrival=lambda keys: arrivals.append(keys))
    agent.explorer = SimpleNamespace(note_observation=lambda c, now: observed.append((c, now)))
    agent._apply = lambda *args: outputs.append(args)
    agent._face_views = lambda key: 0
    agent._ex_phase = "observe"
    agent._ex_cand = Cand("face", (2.36, 2.7), math.atan2(.6, .44), "look", list(faces),
                          primary_face=intended.key if primary else None)
    agent._ex_obs_t0 = 0.
    agent._ex_aligned_since = None
    agent._ex_aligned_face = None
    agent._ex_views0 = dict.fromkeys(faces, 0)
    return agent, intended, nearer, pending, outputs, observed


@pytest.mark.parametrize("primary", [True, False])
def test_arrival_keeps_planned_direction_despite_nearer_opposite_picture(primary):
    agent, intended, nearer, _, outputs, observed = observing_agent(primary)
    p = agent.stack.pose
    assert math.hypot(nearer.cx-p.x, nearer.cy-p.y) < math.hypot(intended.cx-p.x, intended.cy-p.y)
    agent._mode_explore_online(1., .1, 1.)
    assert agent._ex_cand.primary_face == intended.key
    assert agent._ex_cand.heading == pytest.approx(math.atan2(intended.cy-p.y, intended.cx-p.x))
    assert outputs[-1][0] == 0. and outputs[-1][1] > 0., "Turn towards the planned upper face"
    assert outputs[-1][2] == 0., "Hold the camera pan steady for photo classification"
    assert not observed


def test_m3_dwell_starts_after_alignment_and_excludes_turn_time():
    agent, intended, _, _, outputs, observed = observing_agent()
    for now in (1., 3.):
        agent._mode_explore_online(now, .1, now)
        assert agent._ex_aligned_since is None
        assert agent._ex_phase == "observe"
    p = agent.stack.pose
    p.th = math.atan2(intended.cy-p.y, intended.cx-p.x)
    agent._mode_explore_online(4., .1, 4.)
    assert agent._ex_aligned_since == 4.
    assert agent._ex_phase == "observe", "Arrival/turn duration must not satisfy the two-second dwell"
    agent._mode_explore_online(5.9, .1, 5.9)
    assert not observed
    agent._mode_explore_online(6.01, .1, 6.01)
    assert agent._ex_phase == "travel" and len(observed) == 1
    assert all(out[0] == out[2] == out[3] == 0. for out in outputs)


def test_alignment_loss_restarts_photo_dwell():
    agent, intended, _, _, _, observed = observing_agent()
    p = agent.stack.pose
    target_heading = math.atan2(intended.cy-p.y, intended.cx-p.x)
    p.th = target_heading
    agent._mode_explore_online(4., .1, 4.)
    p.th = target_heading + .3
    agent._mode_explore_online(5., .1, 5.)
    assert agent._ex_aligned_since is None
    p.th = target_heading
    agent._mode_explore_online(6., .1, 6.)
    agent._mode_explore_online(7.99, .1, 7.99)
    assert not observed
    agent._mode_explore_online(8.01, .1, 8.01)
    assert len(observed) == 1


def test_next_pending_subject_starts_its_own_dwell():
    agent, intended, nearer, pending, _, observed = observing_agent()
    p = agent.stack.pose
    p.th = math.atan2(intended.cy-p.y, intended.cx-p.x)
    agent._mode_explore_online(4., .1, 4.)
    pending.remove(intended.key)
    p.th = math.atan2(nearer.cy-p.y, nearer.cx-p.x)
    agent._mode_explore_online(6., .1, 6.)
    assert agent._ex_cand.primary_face == nearer.key
    assert agent._ex_aligned_since == 6.
    assert not observed


def test_existing_twelve_second_timeout_still_bounds_an_unaligned_stop():
    agent, _, _, _, outputs, observed = observing_agent()
    agent._mode_explore_online(12.01, .1, 12.01)
    assert agent._ex_phase == "travel" and agent._ex_cand is None
    assert len(observed) == 1 and outputs[-1][2] == 0.
