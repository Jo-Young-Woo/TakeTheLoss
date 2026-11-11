"""Small deterministic referee/evidence scenarios for the published game rules."""
import random
from types import SimpleNamespace as NS

import pytest

from maicon26.config import Config
from maicon26.game_rules import alliance_bonus, danger_score, m4_points, m6_points
from maicon26.mission.threat import Evidence, score as estimate_threat
from maicon26.sim.fixed_maicon import BASE_TYPES
from maicon26.sim.robot import SimServer
from maicon26.sim.run import Match, score, score_m3
from maicon26.sim.world import Cube, Item, World
from maicon26.types import Pose2D


@pytest.fixture
def world():
    # Referee tests do not need layout search, camera rendering or robot motion.
    w = object.__new__(World)
    w.cfg = Config()
    w.geo = w.cfg.arena
    w.scenario = "random"
    w.rng = random.Random(3)
    w.t = 0.0
    w.ended = w.finished = False
    w.end_time = None
    w.m5_ends_match = True
    w.goal_base_types = {}
    w.m5_winner = None
    w._base_contacts = {}
    w.base_types = dict(BASE_TYPES)
    w.ugvs, w.cubes, w.events = [], [], []
    w.assign, w.submissions = {}, {}
    return w


def report_for(team, counts=None):
    side = "R" if team == "BLUE" else "B"
    values = {f"{side}-S{n}": {} for n in (1, 2, 3)}
    values["COMMON"] = {}
    values.update(counts or {})
    return {"sectors": values}


def evaluate(world, server):
    a = NS(name=server.u.id, team=server.u.team, laser=NS(wasted=0), stats={"mode_time": {}})
    return score(Match(world, [a], [server], []))["agents"][server.u.id]


@pytest.mark.spec("S5.2", "S5.3", "S5.5", "S5.6", "S11.6")
def test_partial_m1_report_scores_only_reported_items_without_crashing(world):
    server = SimServer(world, world.add_ugv("BLUE", 1))
    server.submit("m1", {"target": server.truth["target"]})
    assert evaluate(world, server)["m1"] == {"target": True, "sectors": False, "goal_base": False}
    server.submit("m1", {"goal_base": server.truth["goal_base"]})
    assert evaluate(world, server)["m1"] == {"target": False, "sectors": False, "goal_base": True}


@pytest.mark.spec("S7.1", "S7.2", "S7.6", "S11.6")
def test_m3_official_points_need_explicit_complete_sector_reports():
    truth = {"R-S1": {"TANK": 2}, "COMMON": {"DRUM": 1}}
    missing = score_m3(truth, {"sectors": {"R-S1": {"TANK": 2}, "COMMON": {"DRUM": 1}}}, "BLUE")
    assert missing["points"] == 16  # unreported empty R-S2/R-S3 do not earn seven each
    assert missing["exact"] and not missing["official_exact"]
    assert not missing["sector_results"]["R-S2"]["reported"]
    complete = score_m3(truth, report_for("BLUE", truth), "BLUE")
    assert complete["official_exact"] and complete["points"] == 30
    assert complete["sector_results"]["CM"]["points"] == 9
    assert score_m3({}, {"sectors": {}}, "BLUE")["points"] == 0
    assert score_m3({}, report_for("BLUE"), "BLUE")["points"] == 30


@pytest.mark.spec("S7.1", "S7.4", "S7.6", "SA.4", "SA.5")
def test_m3_no_partial_points_for_wrong_quantity_kind_or_extra_object():
    truth = {"R-S1": {"TANK": 2}, "COMMON": {"DRUM": 1}}
    for wrong in ({"TANK": 1}, {"TANK": 2, "DRUM": 1}, {"MORTAR": 2}):
        result = score_m3(truth, report_for("BLUE", {"R-S1": wrong, "COMMON": {"DRUM": 1}}), "BLUE")
        assert result["points"] == 23 and result["sector_results"]["R-S1"]["points"] == 0
    # Home-sector reports cannot improve or harm the four official sectors.
    result = score_m3(truth, report_for("BLUE", {**truth, "B-S1": {"MISSILE": 99}}), "BLUE")
    assert result["points"] == 30
    red = score_m3({"B-S3": {"AMMO_BOX": 1}}, report_for("RED", {"B-S3": {"AMMO_BOX": 1}}), "RED")
    assert red["points"] == 30


@pytest.mark.spec("S8.1", "S8.2")
@pytest.mark.parametrize("hits,points", [(0, 15), (1, 12), (2, 8), (3, 3), (4, 0), (8, 0)])
def test_m4_published_survival_points(hits, points):
    assert m4_points(hits) == points


@pytest.mark.spec("S11.2", "S11.5")
def test_m6_equal_weight_formula_and_nonnegative_terms():
    assert danger_score(4, 8, 30) == 3.0
    assert danger_score(1, 0, 0) == danger_score(0, 2, 0) == danger_score(0, 0, 7.5) == .25
    assert danger_score(-2, -1, -3) == 0.0


@pytest.mark.spec("S11.1", "S11.3", "S11.4", "S11.6")
@pytest.mark.parametrize("ranking,points", [(["R-2", "R-1", "R-3"], 15), (["R-2"], 10),
                                           (["R-3", "R-1", "R-2"], 5), (["R-1", "R-2", "R-3"], 5),
                                           (["R-1", "R-3", "R-2"], 0), ([], 0)])
def test_m6_partial_rank_scoring(ranking, points):
    assert m6_points(ranking, ["R-2", "R-1", "R-3"]) == points


@pytest.mark.spec("S11.2", "S11.5", "SA.3")
def test_m6_referee_uses_actual_m3_points_not_travel_and_estimator_stays_observational(world):
    u = world.add_ugv("BLUE", 1)
    server = SimServer(world, u)
    opponents = [world.add_ugv("RED", n) for n in (1, 2, 3)]
    opponents[0].trail = [(1.0, i * .2) for i in range(50)]
    world.submissions[opponents[1].id] = {"m3": {"payload": report_for("RED")}}
    world.submissions[u.id] = {"m6": {"payload": {"ranking": [opponents[1].id, opponents[0].id, opponents[2].id]}}}
    result = evaluate(world, server)["m6"]
    assert result["true"][0] == opponents[1].id
    assert result["true_scores"][opponents[1].id] == 3.0
    assert result["true_scores"][opponents[0].id] == 2.0
    assert result["points"] == 15
    # The mission estimator receives only its observation summaries. Equal
    # normalized evidence contributes equally, without reading world reports.
    survival = Evidence("R-UGV-1", True, hits_est=0, ammo=0, ammo_known=True)
    ammunition = Evidence("R-UGV-2", True, hits_est=4, ammo=1, ammo_known=True)
    recognition = Evidence("R-UGV-3", True, hits_est=4, ammo=0, ammo_known=True, sectors=5, travel=12)
    ranked = estimate_threat({e.ident: e for e in (survival, ammunition, recognition)})
    assert all(e.score == 1.0 for e in ranked)


@pytest.mark.spec("S6.1", "S6.3", "S6.4", "S8.1", "S8.3")
def test_laser_contact_requires_two_continuous_seconds_with_float_tolerance(world):
    shooter, victim = world.add_ugv("BLUE", 1), world.add_ugv("RED", 1)
    key = ("ugv", victim.id)
    for n in range(39):
        world.t = n * .05
        world._contact(shooter, key, .05, ("ugv", victim, 0))
    assert victim.hits_taken == 0
    world.t = 39 * .05
    world._contact(shooter, key, .05, ("ugv", victim, 0))
    assert victim.hits_taken == 1  # accumulated 1.999999... is a complete 2 s


@pytest.mark.spec("S6.3", "S8.3")
def test_subsecond_contact_gap_does_not_join_two_short_bursts(world, monkeypatch):
    shooter, victim = world.add_ugv("BLUE", 1), world.add_ugv("RED", 1)
    key = ("ugv", victim.id)
    shooter.laser_on = True
    world._contact(shooter, key, 1.0, ("ugv", victim, 0))
    world.t = 1.0
    monkeypatch.setattr(world, "_beam_hit", lambda _: (0.0, None))
    world._laser_step(shooter, .05)
    assert key not in shooter.contact
    world.t = 1.05
    world._contact(shooter, key, 1.0, ("ugv", victim, 0))
    assert victim.hits_taken == 0
    # Even direct sensor callbacks cannot bridge an unobserved interval.
    world.t = 2.2
    world._contact(shooter, key, .1, ("ugv", victim, 0))
    assert shooter.contact[key][0] == pytest.approx(.1)


@pytest.mark.spec("S8.1", "S8.2", "S8.3")
def test_friendly_fire_counts_and_ten_second_invulnerability(world):
    shooter, victim = world.add_ugv("BLUE", 1), world.add_ugv("BLUE", 2)
    key = ("ugv", victim.id)
    world._contact(shooter, key, 2., ("ugv", victim, 0))
    assert victim.hits_taken == 1 and shooter.friendly_hits == 1
    assert victim.invuln_left == 10.0 and not shooter.hits_dealt
    world.t = 2.
    world._contact(shooter, key, 2., ("ugv", victim, 0))
    assert victim.hits_taken == 1
    victim.invuln_left = 0.0  # organizer's ten-second protection expired
    world.t = 12.
    world._contact(shooter, key, 2., ("ugv", victim, 0))
    assert victim.hits_taken == 2 and shooter.friendly_hits == 2


@pytest.mark.spec("S6.1", "S6.2", "S6.3", "S6.4", "S6.5")
def test_destroyed_target_is_counted_once_and_benefits_assignee_not_shooter(world):
    blue = world.add_ugv("BLUE", 1)
    server = SimServer(world, blue)
    red = world.add_ugv("RED", 1)
    target = Item("target", server.truth["target"])
    cube = Cube(world.geo.W - 2, 2., {0: target})
    world.cubes = [cube]
    for n in range(2):
        world.t = 2 * n
        world._contact(red, ("item", id(target)), 2., ("item", cube, 0))
    assert target.led_on and target.hit_by == red.id
    assert len([e for e in world.events if e["type"] == "target_hit"]) == 1
    assert blue.targets_hit == []
    assert evaluate(world, server)["m2_ok"] is True
    assert evaluate(world, server)["m2_points"] == 10
    # Three assigned teams get the alliance bonus only after all designated
    # physical targets are destroyed, regardless of who shot each one.
    for slot, kind in ((2, "GUARD_POST"), (3, "COMM_FACILITY")):
        member = world.add_ugv("BLUE", slot)
        world.assign[member.id] = {"target": kind}
        world.cubes.append(Cube(world.geo.W - 2, 1. + slot, {0: Item("target", kind, led_on=True)}))
    assert evaluate(world, server)["m2_points"] == 15


@pytest.mark.spec("S9.1", "S9.3", "S9.4", "S9.8")
def test_m5_is_one_shared_alliance_objective_separate_from_m1(world):
    servers = [SimServer(world, world.add_ugv("BLUE", n)) for n in (1, 2, 3)]
    assert len({server.truth["m5_base"] for server in servers}) == 1
    for server in servers:
        server.truth["goal_base"] = "MAINT_DEPOT" if server.truth["m5_base"] != "MAINT_DEPOT" else "SUPPLY_BASE"
    world.t = world.cfg.mission.t_base_cmd
    for server in servers:
        messages = server.poll()
        assert any(msg.kind == "base_command" for msg in messages)
        assert world.base_types[server.u.goal_base] == server.truth["m5_base"]
        assert world.base_types[server.u.goal_base] != server.truth["goal_base"]


@pytest.mark.spec("S9.2", "S9.7", "S10.1", "S10.2", "S10.4")
def test_any_robot_in_target_sensor_credits_target_alliance_even_own_base(world):
    blue = world.add_ugv("BLUE", 1)
    world.goal_base_types = {"RED": "COMMAND_POST"}
    blue.pose = Pose2D(*world.geo.base_center("B-1"), 0)
    world.t = world.cfg.mission.t_base_cmd - .01
    world._base_sensor_step()
    assert not world.ended
    world.t = world.cfg.mission.t_base_cmd
    world._base_sensor_step()
    assert world.ended and world.m5_winner == "RED"
    assert world.events[0]["by"] == blue.id and world.events[0]["camp"] == "RED"
    assert world.end_time == world.cfg.mission.t_base_cmd


@pytest.mark.spec("S9.7", "S0.15")
def test_non_target_base_does_not_capture_and_dwell_is_explicit_provisional_setting(world):
    blue = world.add_ugv("BLUE", 1)
    world.goal_base_types = {"BLUE": "COMMAND_POST"}
    world.t = world.cfg.mission.t_base_cmd
    blue.pose = Pose2D(*world.geo.base_center("R-2"), 0)
    world._base_sensor_step()
    assert not world.ended
    assert getattr(world.cfg.mission, "m5_dwell_s", 0.) == 0.
    world.cfg.mission.m5_dwell_s = 1.5  # a deliberately supplied rehearsal setting
    blue.pose = Pose2D(*world.geo.base_center("R-1"), 0)
    world._base_sensor_step()
    assert not world.ended
    world.t += 1.
    world._base_sensor_step()
    assert not world.ended
    world.t += .5
    world._base_sensor_step()
    assert world.ended and world.m5_winner == "BLUE"


@pytest.mark.spec("S9.2")
def test_alliance_victory_bonus_and_draw():
    assert alliance_bonus(100, 90) == {"BLUE": 5, "RED": 0}
    assert alliance_bonus(90, 100) == {"BLUE": 0, "RED": 5}
    assert alliance_bonus(100, 100) == {"BLUE": 5, "RED": 5}


@pytest.mark.spec("S0.5", "S5.7", "S7.7", "S10.4", "S11.3", "S11.6")
def test_python_sim_submission_phase_gates_and_six_reports_finish(world):
    servers = [SimServer(world, world.add_ugv(camp, slot)) for camp in ("BLUE", "RED") for slot in (1, 2, 3)]
    first = servers[0]
    assert first.submit("m1", {"target": "RADAR_FACILITY"})
    assert not first.submit("m6", {"ranking": ["R-UGV-1"]})
    world.end()
    assert not first.submit("m1", {"target": "COMM_FACILITY"})
    assert first.last_submit_status == 409
    for i, server in enumerate(servers):
        opponent = "R" if server.u.team == "BLUE" else "B"
        assert server.submit("m6", {"ranking": [f"{opponent}-UGV-1"]})
        assert world.finished is (i == 5)
    assert not first.submit("m6", {"ranking": ["R-UGV-2"]})
    world.t = world.end_time + world.cfg.mission.t_report
    assert not first.submit("m6", {"ranking": ["R-UGV-2"]})
