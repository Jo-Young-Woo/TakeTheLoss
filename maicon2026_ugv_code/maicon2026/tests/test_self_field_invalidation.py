"""Removing self must not let an old worker result reintroduce its obstacle."""
from types import SimpleNamespace as NS

import numpy as np

from maicon26.config import Config
from maicon26.mission.manager import Agent
from maicon26.mission.threat_field import build_field, body_hit
from maicon26.nav.grid import OccupancyGrid
from maicon26.perception.threat_board import ThreatTrack, EV_OBJECT
from maicon26.types import Pose2D
from maicon26.mission.avoid import Level


def test_pruned_self_rebuild_keeps_other_body_and_rejects_inflight_stale_field():
    cfg = Config()
    cfg.progress.enabled = cfg.risk.threaded = cfg.avoid.enable = True
    grid = OccupancyGrid(cfg.arena)
    grid.L[:] = grid.L_MIN
    def track(tid, x):
        return ThreatTrack(tid=tid, x=x, y=2., vx=0., vy=0., sigma=.06,
                           evidence=EV_OBJECT, friend=False, p_enemy=.5,
                           t_last=2., weight=.7)
    other, self_ghost = track(1, 3.), track(2, 1.)
    old = build_field([other, self_ghost], grid, cfg.avoid, 3., 2.,
                      cfg.nav.inflate_hard, 5, risk_cfg=cfg.risk)
    assert body_hit(old, grid, 1., 2.)
    worker = NS(latest=lambda: (5, old))
    agent = NS(cfg=cfg, board=NS(update=lambda *a, **kw: [other]),
        tracker=object(), stack=NS(initialized=True, pose=Pose2D(1., 2., 0.),
                                  grid=grid, set_hint=lambda *a: None),
        coverage=NS(confirmed_static_footprints=lambda *a: []), omap=None,
        _v_cmd=0., _blobs=[], _deadlock_enabled=lambda: True,
        _risk_self_invalidated=True, _field_ver=5, _field=old, _field_t=2.,
        _hit_history=[], _risk_worker=worker, _scan_stamp=2.,
        _risk_scan_stamp=2., _scan=None,
        _av_maps=(2., np.zeros_like(grid.L, dtype=bool), np.ones_like(grid.L)),
        _goal_xy=lambda: None, _avoid_goal_flags=lambda *a: (False, False),
        _cur_goal=None, avoid=NS(budget=lambda *a: (1., 10.),
             assess=lambda *a: Level.CLEAR, note_pose=lambda *a, **kw: None))
    Agent._threat_update(agent, 2., .05, 2.)
    assert agent._risk_min_version == 6
    assert not body_hit(agent._field, grid, 1., 2.)
    assert body_hit(agent._field, grid, 3., 2.)
    assert not agent._risk_self_invalidated
