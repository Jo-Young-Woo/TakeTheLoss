"""Device judging on measured arena geometry; no Gazebo or network required."""
import math
from pathlib import Path
import sys

import pytest

PACKAGE = Path(__file__).resolve().parents[1] / "ros2/maicon_arena_gz"
sys.path.insert(0, str(PACKAGE / "scripts"))
from arena_common import load_yaml  # noqa: E402
from arena_referee import ArenaReferee, beam_intersects_robot, ray_box  # noqa: E402
from perf_probe import summarize  # noqa: E402


def referee(empty=False):
    spec = load_yaml(PACKAGE / "config/arena_spec.yaml")
    layout = {"boxes": []} if empty else load_yaml(PACKAGE / "config/layouts/layout_default.yaml")
    return ArenaReferee(spec, layout, load_yaml(PACKAGE / "config/simulation_assumptions.yaml"),
                        load_yaml(PACKAGE / "config/robot_assumptions.yaml"))


def test_beam_uses_finite_segment_and_rotated_footprint_including_height():
    core = referee()
    beam = core.spec["bases"]["photoelectric_beam"]["segments"]["B-2"]
    assert beam_intersects_robot(beam, .1, (-2.66, 0., 0., .4), core.footprint)
    assert not beam_intersects_robot(beam, .1, (-2.1, 0., 0., 0.), core.footprint)
    assert not beam_intersects_robot(beam, .1, (-2.65, 1.5, 0., 0.), core.footprint)
    assert not beam_intersects_robot(beam, .1, (-2.65, 0., .2, 0.), core.footprint)


@pytest.mark.parametrize("base", ["B-1", "B-2", "B-3", "R-1", "R-2", "R-3"])
def test_every_base_reports_anonymous_entry_and_departure_before_start(base):
    core = referee()
    x, first, last = core.spec["bases"]["photoelectric_beam"]["segments"][base]
    pose = {"B-2": (x, (first+last)/2, 0., 0.)}
    core.step(0., pose)
    core.step(.05, pose)
    core.step(.1, {"B-2": (0., 0., 0., 0.)})
    assert core.events == [
        {"type": "base_beam", "time": 0., "base": base, "blocked": True},
        {"type": "base_beam", "time": .1, "base": base, "blocked": False},
    ]
    assert core.phase == "ready" and not core.capture_since


@pytest.mark.spec("S9.7", "S10.2")
def test_any_ugv_blocking_opposing_objective_ends_match_at_publication():
    core = referee()
    core.start(10., {"RED": "B-2"})
    own_robot = {"B-2": (-2.66, 0., 0., 0.)}
    core.step(249.9, own_robot)
    assert core.phase == "running"
    core.step(250., own_robot)
    assert core.phase == "reporting"
    captured = [e for e in core.events if e["type"] == "base_captured"]
    assert captured == [{"type": "base_captured", "time": 250., "base": "B-2", "alliance": "RED"}]


def test_non_target_beam_does_not_end_match_and_reporting_lasts_thirty_seconds():
    core = referee()
    core.start(0., {"BLUE": "R-1"})
    core.step(240., {"B-2": (-2.66, 0., 0., 0.)})
    assert core.phase == "running"
    core.step(300., {})
    assert core.phase == "reporting" and core.fire("B-2", 300.)[0] == "locked"
    core.step(329.99, {})
    assert core.phase == "reporting"
    core.step(330., {})
    assert core.phase == "finished"


def test_actual_receiver_disk_is_visible_from_open_face_but_back_side_is_blocked():
    core = referee()
    target = next(t for t in core.targets if t["kind"] == "comms" and str(t["camp"]).startswith("B"))
    index = core.targets.index(target)
    receiver = target["receiver"]
    pose = (receiver[0]+.7, receiver[1], 0., math.pi)
    pivot = core.robot_geometry["gimbal"]["pivot"]
    tilt = math.atan2(receiver[2]-pivot[2], .7-pivot[0])
    core.poses = {"R-2": pose}
    core.gimbals["R-2"] = (0., tilt)
    assert core._first_receiver("R-2") == ("target", index)
    core.poses["R-2"] = (receiver[0]-.7, receiver[1], 0., 0.)
    assert core._first_receiver("R-2") != ("target", index)


@pytest.mark.spec("S6.3", "S6.4")
def test_two_continuous_seconds_target_counts_once(monkeypatch):
    core = referee()
    core.start(0.)
    core.fire("B-2", 0.)
    monkeypatch.setattr(core, "_first_receiver", lambda slot: ("target", 0))
    for index in range(40):
        core.step(index*.05, {"B-2": (0., 0., 0., 0.)})
    assert not core.target_hits
    core.step(2., {"B-2": (0., 0., 0., 0.)})
    assert len(core.target_hits) == 1
    for index in range(41, 80):
        core.step(index*.05, {"B-2": (0., 0., 0., 0.)})
    assert sum(core.destroyed.values()) == 1


@pytest.mark.spec("S8.3")
def test_interrupted_or_missing_pose_illumination_does_not_accumulate(monkeypatch):
    core = referee(empty=True)
    core.start(0.)
    core.fire("B-2", 0.)
    monkeypatch.setattr(core, "_first_receiver", lambda slot: ("ugv", "R-2"))
    poses = {"B-2": (0., -1., 0., math.pi/2), "R-2": (0., 0., 0., 0.)}
    for index in range(30):
        core.step(index*.05, poses)
    core.step(1.5, {})
    for index in range(31, 61):
        core.step(index*.05, poses)
    assert core.ugvs["R-2"]["hp"] == 4
    core.step(4., poses)  # Missing intermediate samples also cannot establish a two-second contact.
    assert core.ugvs["R-2"]["hp"] == 4


@pytest.mark.spec("S8.3")
def test_friendly_fire_ten_second_immunity_and_fourth_hit_drive_lock():
    core = referee(empty=True)
    core.start(0.)
    for t in (2., 3., 11.99):
        core._apply_hit("B-1", ("ugv", "B-2"), t)
    assert core.ugvs["B-2"]["hp"] == 3
    assert core.ugvs["B-2"]["lock_until"] == 12.
    assert core.fire("B-2", 5.)[0] == "locked"
    for t in (12., 22., 32.):
        core._apply_hit("B-1", ("ugv", "B-2"), t)
    assert core.ugvs["B-2"]["hp"] == 0 and core.ugvs["B-2"]["drive_lock_until"] == 42.
    assert all(e["friendly_fire"] for e in core.events if e["type"] == "ugv_hit")


def test_magazine_refire_lock_and_finished_state_do_not_allow_extra_shots():
    core = referee(empty=True)
    assert core.fire("B-2", 0.)[0] == "locked"
    core.start(0.)
    assert core.fire("B-2", 0.)[0] == "fired"
    assert core.fire("B-2", 5.) == ("reloading", 5000)
    for t in range(10, 80, 10):
        assert core.fire("B-2", float(t))[0] == "fired"
    assert core.fire("B-2", 80.)[0] == "empty"
    core.report(80.)
    assert core.fire("R-2", 80.)[0] == "locked"


def test_ray_parallel_and_inside_cases_and_perf_report_are_measured():
    assert ray_box((0., 0., 0.), (1., 0., 0.), (1., 2., -1., 1., -1., 1.)) == 1.
    assert ray_box((0., 2., 0.), (1., 0., 0.), (1., 2., -1., 1., -1., 1.)) is None
    stats = summarize([(0., 0.), (1., .9), (2., 1.9), (3., 2.9)], 3)
    assert stats["rtf_mean"] == pytest.approx(2.9/3)
    assert stats["rtf_min_one_second_window"] == .9
    assert stats["gui_fps"] is None and stats["received_camera_fps"] == 1.

def test_zero_hp_is_not_permanent_immunity_and_fourth_hit_pause_is_temporary():
    core = referee()
    for when in (0.,10.,20.,30.):
        core._apply_hit("R-2",("ugv","B-2"),when)
    state=core.ugvs["B-2"]
    assert state["hp"]==0 and state["hits_taken"]==4
    assert state["drive_lock_until"]==40.
    core._apply_hit("R-2",("ugv","B-2"),39.99)
    assert state["hits_taken"]==4
    core._apply_hit("R-2",("ugv","B-2"),40.)
    assert state["hp"]==0 and state["hits_taken"]==5
    assert state["invulnerable_until"]==50.
    assert state["drive_lock_until"]==40.
