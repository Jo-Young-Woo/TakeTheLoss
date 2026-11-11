"""depth.gate_drive=False: 엄격 깊이(OAK)는 피아식별·거리 측정에만 쓰는 선택 센서다 — 영상이 끊겨도 로봇을 세우지 않는다.
기본값(True)은 기존 동작(영상이 끊기면 구동·준비 판정을 막는다)을 그대로 둔다. ROS·시뮬을 쓰지 않는다."""
import numpy as np
import pytest

from test_depth_contract import _image, _info, _robot


def _live(r):
    r.on_depth_info(_info())
    r.on_depth(_image(), tf_validated=True)


def test_default_still_stops_the_robot_when_strict_depth_goes_stale():
    r, tr, clock = _robot()
    assert r.cfg.depth.gate_drive is True
    r.on_match("practice", "idle")
    _live(r)
    r._publish_cmd(.2, .3)
    assert tr.cmds[-1] == (.2, .3)
    clock[0] += 5.0                                    # 깊이가 5 s 끊겼다
    r._publish_cmd(.2, .3)
    assert tr.cmds[-1] == (0, 0) and r.stats["cmd_zero_depth"] >= 1
    assert "depth" in r.readiness()[1]


def test_gate_drive_false_keeps_driving_without_depth_frames():
    r, tr, clock = _robot()
    r.cfg.depth.gate_drive = False
    r.on_match("practice", "idle")
    _live(r)
    before = r.stats.get("cmd_zero_depth", 0)           # 이 스위치를 끄기 전 준비 단계의 정지는 세지 않는다
    clock[0] += 5.0
    r._received.update({k: clock[0] for k in ("scan", "odom", "image", "imu", "state")})
    r._publish_cmd(.2, .3)
    assert tr.cmds[-1] == (.2, .3) and r.stats.get("cmd_zero_depth", 0) == before
    assert r.readiness() == (True, [])                  # 깊이가 없어도 준비 완료 판정에서 빠진다


def test_gate_drive_false_never_started_depth_does_not_block_the_first_command():
    r, tr, _ = _robot()
    r.cfg.depth.gate_drive = False
    r.on_match("practice", "idle")
    r._publish_cmd(.2, .3)                              # 깊이 영상을 한 번도 못 받았다
    assert tr.cmds[-1] == (.2, .3)


def test_gate_drive_false_rejected_depth_does_not_publish_a_stop_but_frame_is_dropped():
    r, tr, _ = _robot()
    r.cfg.depth.gate_drive = False
    r.on_match("practice", "idle")
    _live(r)
    r._publish_cmd(.2, .3)
    n = len(tr.cmds)
    bad = _image(frame="other_frame")
    r.on_depth(bad, tf_validated=True)                  # 검증 실패: 영상은 버리되
    assert r.get_depth() is None and "depth_error" in r.stats
    assert len(tr.cmds) == n                            # 정지 명령을 새로 내지 않는다
    r._publish_cmd(.2, .3)
    assert tr.cmds[-1] == (.2, .3)


def test_gate_drive_false_does_not_loosen_validation_of_the_depth_frames_themselves():
    r, _, _ = _robot()
    r.cfg.depth.gate_drive = False
    r.on_depth(_image(), tf_validated=True)             # CameraInfo 없이 받은 영상
    assert r.get_depth() is None and "CameraInfo" in r.stats["depth_error"]
    r.on_depth_info(_info())
    r.on_depth(_image())                                 # TF 검증을 못 받은 영상
    assert r.get_depth() is None and "TF" in r.stats["depth_error"]
    r.on_depth(_image(), tf_validated=True)
    assert r.get_depth() is not None and np.all(r.get_depth().depth == 1)


def test_gate_drive_is_ignored_when_depth_is_disabled():
    r, tr, _ = _robot()
    r.cfg.depth.enable = False
    assert r._depth_gates_drive() is False
    r.on_match("practice", "idle")
    r._publish_cmd(.2, .3)
    assert tr.cmds[-1] == (.2, .3)


@pytest.mark.parametrize("gate,expect", [(True, True), (False, False)])
def test_gate_flag_only_matters_in_strict_mode(gate, expect):
    r, _, _ = _robot()
    r.cfg.depth.gate_drive = gate
    assert r._depth_gates_drive() is expect
    r.cfg.depth.require_calibration = False
    assert r._depth_gates_drive() is False              # 느슨한 모드는 애초에 구동을 막지 않는다
