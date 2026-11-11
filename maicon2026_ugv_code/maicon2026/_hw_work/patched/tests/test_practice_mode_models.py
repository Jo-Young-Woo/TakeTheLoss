"""2장 — 연습 모드 탄약·피격 모델(하드웨어 PDF p70: 연습 모드에서는 탄약과 HP 가 자동 회복된다). 시뮬 없이 가짜 transport 로 본다.

mission 모드의 계산은 한 줄도 바뀌지 않는다 → 0-1 기준값(test_hw_baseline_equivalence.py)이 증명하고, 아래도 몇 개 다시 확인한다.
"""
import pytest

from types import SimpleNamespace as NS

from hw_fakes import FakeTransport, make_robot
from maicon26.config import Config
from maicon26.hal.mil_api import FireModel, HitModel
from maicon26.mission.aim import LaserManager
from maicon26.mission.manager import Agent, Mode
from maicon26.types import Pose2D, RobotStatus, gate_ammo


def fire_n(robot, clock, n, gap=10.5):
    got = []
    for _ in range(n):
        clock.t += gap
        got.append(robot.laser_fire())
    return got


# ====================================================================== 탄약
def test_practice_twenty_consecutive_shots_are_all_allowed():
    robot, tr, clock = make_robot("practice", "idle")
    laser = LaserManager(robot.cfg, robot)
    for k in range(20):
        clock.t += 10.5                                     # 응답은 모두 fired, 10 s 간격
        assert laser.ready(), f"{k}번째 발사 전에 막혔다 (ammo={robot.status().ammo})"
        assert robot.laser_fire() is True
    assert len(tr.fire) == 20
    st = robot.status()
    assert st.ammo == 0 and st.ammo_free                     # 추정은 지금처럼 센다(mission 전환 때 초기화). 하지만 사격 조건은 아니다
    assert laser.ammo == robot.cfg.laser.ammo                # 사격 조건으로 쓰는 값은 가득
    clock.t += 10.5
    assert laser.ready()                                     # 추정 탄약 0 이어도 막히지 않는다


def test_mission_still_runs_out_after_the_magazine_exactly_as_before():
    robot, tr, clock = make_robot("mission", "start")
    laser = LaserManager(robot.cfg, robot)
    shots = 0
    for _ in range(20):
        clock.t += 10.5
        if laser.ready() and robot.laser_fire():
            shots += 1
    assert shots == robot.cfg.laser.ammo and robot.status().ammo == 0


def test_practice_empty_blocks_for_the_backoff_then_allows_again():
    robot, tr, clock = make_robot("practice", "idle", transport=FakeTransport([("empty", None)]))
    laser = LaserManager(robot.cfg, robot)
    clock.t += 1.0
    assert robot.laser_fire() is False                       # empty 응답
    assert robot.fire.ammo == 0 and laser.ammo == robot.cfg.laser.ammo   # 추정은 0 이지만 사격 조건은 가득 — 막는 것은 backoff 뿐이다
    assert robot.status().cooldown_left > 0
    for dt in (0.5, 4.0, 4.0):                               # 10 s 가 되기 전
        clock.t += dt
        assert not laser.ready() and robot.laser_fire() is False
    assert len(tr.fire) == 1                                 # 서비스도 다시 부르지 않았다
    clock.t += 2.0                                           # 총 10.5 s 지남
    assert laser.ready() and robot.laser_fire() is True
    assert len(tr.fire) == 2


def test_practice_empty_backoff_follows_the_config_key():
    cfg = Config()
    cfg.laser.practice_empty_backoff_s = 3.0
    robot, tr, clock = make_robot("practice", "idle", cfg=cfg, transport=FakeTransport([("empty", None)]))
    clock.t += 1.0
    robot.laser_fire()
    clock.t += 2.0
    assert robot.laser_fire() is False
    clock.t += 1.5
    assert robot.laser_fire() is True


def test_practice_still_follows_reloading_and_locked_with_left_ms():
    f = FireModel(8, 5.0, 10.0)
    f.set_practice(True)
    f.on_response(100.0, "reloading", 3000)
    assert f.cooldown_left(100.0) == pytest.approx(3.0)
    f.on_response(110.0, "locked", None)
    assert f.rejected["locked"] == 1


def test_practice_to_mission_resets_ammo_and_hits_but_starting_in_mission_does_not():
    robot, tr, clock = make_robot("practice", "idle")
    robot.fire.ammo = 2                                      # 연습 중에 어떤 이유로든 어긋난 값
    robot.on_state(3, 4, 90.0)
    robot.on_state(1, 4, 90.0)                               # 피격 2
    robot.on_state(4, 4, 90.0)                               # 연습: 회복 → 0
    robot.on_state(3, 4, 90.0)                               # 마지막 상태: hp 3/4
    robot.on_match("mission", "standby")                     # 전환을 실제로 본 순간
    assert robot.fire.ammo == robot.cfg.laser.ammo
    assert robot.hit.hits == 1                               # hp_max − hp
    # 처음부터 mission 이면 아무것도 초기화하지 않는다(경기 중 재시작 → 기존 복원 경로)
    robot2, _, _ = make_robot("mission", "start")
    robot2.fire.ammo = 3
    robot2.on_state(2, 4, 90.0)
    assert robot2.hit.hits == 2 and robot2.fire.ammo == 3
    robot2.on_match("mission", "pause")
    assert robot2.fire.ammo == 3 and robot2.hit.hits == 2


def test_unknown_mode_is_not_practice():
    robot, tr, clock = make_robot("practise", "idle")        # 철자가 다른 모르는 값
    assert not robot.fire.practice and not robot.hit.practice
    robot.on_match("practice", "idle")
    assert robot.fire.practice and robot.hit.practice


# ====================================================================== 피격
def test_practice_hit_and_recover_six_times_never_accumulates_or_pauses_driving():
    h = HitModel(10.0, 10.0)
    h.set_practice(True)
    h.on_state(0.0, 4, 4)
    t = 1.0
    for _ in range(6):
        h.on_state(t, 3, 4)                                  # 피격
        assert h.hits <= 1 and h.disabled_left(t) > 0         # 피격 잠금은 지금처럼 건다
        t += 2.0
        h.on_state(t, 4, 4)                                  # 자동 회복
        assert h.hits == 0
        t += 2.0
    assert h.drive_left(t) == 0.0


def test_without_practice_the_same_sequence_accumulates_as_before():
    h = HitModel(10.0, 10.0)
    h.on_state(0.0, 4, 4)
    t = 1.0
    for _ in range(6):
        h.on_state(t, 3, 4)
        t += 2.0
        h.on_state(t, 4, 4)
        t += 2.0
    assert h.hits == 6 and h.drive_left(t) > 0                           # 기존 규칙(회복해도 누적 → 가짜 주행 정지)이 그대로다


def test_practice_robot_hit_recover_cycle_has_no_fake_drive_stop():
    robot, tr, clock = make_robot("practice", "idle")
    robot.on_state(4, 4, 90.0)
    for _ in range(6):
        clock.t += 1.0
        robot.on_state(3, 4, 90.0)
        clock.t += 1.0
        robot.on_state(4, 4, 90.0)
    st = robot.status()
    assert st.hits_taken <= 1 and st.drive_paused_left == 0.0


# ====================================================================== 탄약을 사격 조건으로 보는 모든 지점
def test_gate_ammo_helper_only_changes_practice_and_tolerates_plain_objects():
    assert gate_ammo(RobotStatus(ammo=0), 8) == 0                                   # mission(기본): 그대로
    assert gate_ammo(RobotStatus(ammo=3), 8) == 3
    assert gate_ammo(RobotStatus(ammo=0, ammo_free=True), 8) == 8                   # 연습: 가득
    assert gate_ammo(NS(ammo=0), 8) == 0                                            # ammo_free 가 없는 가짜·시뮬 상태


def _agent_self(cfg, st_enemies=(), faces=(object(),), reserve=0):
    cfg.mission.engage_ugv = True
    return NS(cfg=cfg, locks=NS(weapon_locked=lambda st, now: False), stack=NS(pose=Pose2D(0.0, 0.0, 0.0)),
              _hunt_candidates=lambda now: list(st_enemies), _target_faces=lambda: list(faces), _reserve=lambda: reserve,
              _strike_block_until=0.0, _m2_defer_s=0.0)


def test_arbitrate_strike_and_hunt_ignore_the_estimate_in_practice_but_not_in_mission():
    cfg = Config()
    enemy = NS(x=0.5, y=0.0, last_cam_t=10.0)
    for faces, enemies, want in (((object(),), (), Mode.STRIKE), ((), (enemy,), Mode.HUNT)):
        me = _agent_self(cfg, enemies, faces, reserve=3)
        assert Agent._arbitrate(me, 10.0, RobotStatus(ammo=0, ammo_free=True)) == want      # 연습: 추정 0 이어도 허용
        assert Agent._arbitrate(me, 10.0, RobotStatus(ammo=0)) == Mode.EXPLORE              # mission: 지금처럼 막힌다
        assert Agent._arbitrate(me, 10.0, RobotStatus(ammo=8)) == want                      # mission 탄약이 있으면 허용(기존)


def test_arbitrate_reserve_comparison_uses_the_gate_value_too():
    cfg = Config()
    me = _agent_self(cfg, (NS(x=0.5, y=0.0, last_cam_t=10.0),), (), reserve=3)
    assert Agent._arbitrate(me, 10.0, RobotStatus(ammo=3)) == Mode.EXPLORE                   # mission: 예비(3) 이하면 안 쏜다
    assert Agent._arbitrate(me, 10.0, RobotStatus(ammo=3, ammo_free=True)) == Mode.HUNT      # 연습: 예비도 자동 회복


def test_laser_manager_ready_and_ammo_follow_the_gate_value():
    for mode, cmd, free in (("practice", "idle", True), ("mission", "start", False)):
        robot, tr, clock = make_robot(mode, cmd)
        robot.fire.ammo = 0                                                                 # 추정이 0 이 된 상황
        clock.t += 10.5
        laser = LaserManager(robot.cfg, robot)
        assert robot.status().ammo_free is free
        assert laser.ready() is free                                                        # 연습만 허용, mission 은 지금처럼 막힘
        assert laser.ammo == (robot.cfg.laser.ammo if free else 0)
