"""하드웨어 문서 반영 4장: 짐벌 정착 시간 — /mil/gimbal 의 ok 는 접수일 뿐 도착이 아니다(p72). 시뮬 없이 돈다.

기본값(aim.gimbal_settle_s = 0.0)은 발사 시점이 기존과 완전히 같고, 0 보다 크면 '크게 움직인 마지막 명령' 뒤 그 시간 동안 발사를 미룬다.
"""
from hw_fakes import FakeTransport, make_robot
from maicon26.config import Config
from maicon26.mission.aim import LaserManager


def _cfg(settle, move_deg=1.0):
    cfg = Config()
    cfg.aim.gimbal_settle_s = settle
    cfg.aim.settle_move_deg = move_deg
    return cfg


def _robot(settle, **kw):
    robot, tr, clock = make_robot("practice", "idle", cfg=_cfg(settle, **kw))
    clock.t += 1.0
    return robot, tr, clock


def _move(robot, clock, pan, tilt=0.0):
    clock.t += 0.5                                   # gimbal_min_interval_s 보다 길게
    robot.set_pan_tilt(pan, tilt)
    robot.pump()


def test_default_is_zero_and_fire_timing_is_unchanged():
    cfg = Config()
    assert cfg.aim.gimbal_settle_s == 0.0 and cfg.aim.settle_move_deg == 1.0
    robot, tr, clock = _robot(0.0)
    _move(robot, clock, 30.0)
    assert robot.gimbal_settle_left() == 0.0
    assert robot.laser_fire() is True                # 명령을 막 접수시킨 직후에도 기존처럼 바로 쏜다
    assert "fire_wait_settle" not in robot.stats


def test_big_move_blocks_fire_until_settle_time_has_passed():
    robot, tr, clock = _robot(0.4)
    _move(robot, clock, 30.0)
    assert tr.gimbal and robot.gimbal_settle_left() > 0
    assert robot.laser_fire() is False and tr.fire == []         # 서비스도 부르지 않는다
    clock.t += 0.3
    assert robot.laser_fire() is False
    clock.t += 0.15                                              # 접수 후 0.45 s > 0.4
    assert robot.gimbal_settle_left() == 0.0
    assert robot.laser_fire() is True and tr.fire == [1]
    assert robot.stats["fire_wait_settle"] == 2


def test_small_move_does_not_restart_the_settle_clock():
    robot, tr, clock = _robot(0.4)
    _move(robot, clock, 30.0)
    clock.t += 0.6
    assert robot.gimbal_settle_left() == 0.0
    _move(robot, clock, 30.5)                                    # 0.5° < settle_move_deg(1.0)
    assert len(tr.gimbal) == 2                                   # 명령은 나갔지만
    assert robot.gimbal_settle_left() == 0.0                     # 정착 시계는 다시 시작하지 않는다
    assert robot.laser_fire() is True


def test_each_new_big_move_restarts_the_clock():
    robot, tr, clock = _robot(0.4)
    _move(robot, clock, 30.0)
    clock.t += 0.6
    _move(robot, clock, -20.0)
    assert robot.gimbal_settle_left() > 0.3
    assert robot.laser_fire() is False


def test_settle_move_deg_threshold_is_configurable():
    robot, tr, clock = _robot(0.4, move_deg=10.0)
    _move(robot, clock, 30.0)
    clock.t += 0.6
    _move(robot, clock, 38.0)                                    # 8° < 10°
    assert robot.gimbal_settle_left() == 0.0
    _move(robot, clock, 55.0)                                    # 17° > 10°
    assert robot.gimbal_settle_left() > 0


def test_unanswered_big_move_counts_as_not_settled():
    tr = FakeTransport()
    tr.gimbal_pending = []                                       # 접수 응답이 아직 안 왔다
    robot, tr, clock = make_robot("practice", "idle", cfg=_cfg(0.4), transport=tr)
    clock.t += 1.0
    _move(robot, clock, 30.0)
    assert robot.gimbal_settle_left() == 0.4
    assert robot.laser_fire() is False
    clock.t += 0.1
    tr.gimbal_pending.pop(0)("ok")                               # 접수
    assert robot.gimbal_settle_left() > 0.3
    clock.t += 0.5
    assert robot.laser_fire() is True


def test_rejected_or_locked_gimbal_command_does_not_start_the_clock():
    tr = FakeTransport(gimbal_reply="locked")
    robot, tr, clock = make_robot("practice", "idle", cfg=_cfg(0.4), transport=tr)
    clock.t += 1.0
    _move(robot, clock, 30.0)
    assert robot.gimbal_settle_left() == 0.0                     # 접수되지 않았으니 움직이지 않았다


def test_laser_manager_goes_through_the_same_single_gate():
    robot, tr, clock = _robot(0.4)
    _move(robot, clock, 30.0)
    lm = LaserManager(robot.cfg, robot)
    assert lm.fire(clock.t) is False and lm.shots == 0           # 낭비로 세지 않는다
    clock.t += 0.5
    assert lm.fire(clock.t) is True and lm.shots == 1


def test_resume_resends_the_pose_and_restarts_the_settle_clock():
    robot, tr, clock = _robot(0.4)
    robot.on_match("mission", "start")
    _move(robot, clock, 30.0)
    clock.t += 0.6
    assert robot.laser_fire() is True                            # 정착 완료 후 정상 발사
    n = len(tr.gimbal)
    robot.on_match("mission", "pause")
    clock.t += 15.0                                              # 발광·재장전 대기도 끝난 뒤
    robot.on_match("mission", "resume")                          # 재개: 팬틸트 재전송 강제(p12) — 이전 접수 사실을 믿지 않는다
    robot.pump()
    assert len(tr.gimbal) == n + 1
    assert robot.gimbal_settle_left() > 0.3
    assert robot.laser_fire() is False
    clock.t += 0.5
    assert robot.laser_fire() is True
