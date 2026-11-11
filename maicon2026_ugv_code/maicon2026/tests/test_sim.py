from maicon26.config import Config
from maicon26.sim.run import build_match, run, score
from maicon26.sim.subgame import run_try


def _cfg():
    cfg = Config()
    cfg.mission.spin_w, cfg.mission.tour_nx, cfg.mission.tour_ny = 1.8, 3, 3
    return cfg


def test_solo_match_m1_m2():
    m = build_match(seed=1, cfg=_cfg(), teams=["B2"])
    run(m, max_t=340)
    v = score(m)["agents"]["B-UGV-2"]
    assert v["m1"] and all(v["m1"].values())
    assert v["m2_ok"]
    assert v["friendly_hits"] == 0


def test_full_match_no_friendly_fire_and_m1():
    m = build_match(seed=2, cfg=_cfg())
    run(m, max_t=340)
    sc = score(m)["agents"]
    assert all(v["m1"] and all(v["m1"].values()) for v in sc.values())
    assert sum(v["friendly_hits"] for v in sc.values()) == 0
    assert m.world.end_time is not None


def test_subgame_fast_and_robust():
    for s in range(4):
        r = run_try(s)
        assert r["ok"] and r["t"] < 20.0


def test_subgame_far_target_search():
    r = run_try(3, dist=3.5)
    assert r["ok"]
