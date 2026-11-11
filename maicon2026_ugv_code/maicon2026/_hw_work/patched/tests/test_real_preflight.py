from types import SimpleNamespace as NS
from pathlib import Path
import pytest
from maicon26.config import Config
from maicon26.preflight import wait_ready, validate_model_labels, validate_real_config, clear_standby_restore


class Clock:
    t = 0.0
    def __call__(self): return self.t
    def sleep(self, dt): self.t += dt


@pytest.mark.parametrize("missing", ["state", "match", "match_mode"])
def test_sensor_timeout_never_becomes_ready(missing):
    # 규칙 변경(프롬프트 1장): 'odom 이 끝까지 없으면 TimeoutError' 였던 이 시험은 필수 항목(대회 API: state·match·match_mode)에 대한
    # '시간이 지나도 ready 가 되지 않는다'로 좁혔다. odom·scan·imu 는 이제 저하 모드(no_drive)로 /ready 를 보낸다 → test_ready_degraded.py.
    c = Clock()
    with pytest.raises(TimeoutError, match=missing):
        wait_ready(NS(readiness=lambda: (False, [missing])), NS(), .4, clock=c, sleep=c.sleep)


def test_nav2_tf_missing_is_still_checked_but_degrades_to_no_drive():
    # 규칙 변경 3번째(프롬프트는 두 개로 적었지만 같은 표의 '주행 갈래'에 걸린다): map→base TF 는 여전히 확인하되,
    # 제한시간 뒤 TimeoutError 가 아니라 ReadyState(degraded, no_drive)를 돌려준다. 이유에 TF 가 남아야 운영자가 안다.
    c = Clock()
    stack = NS(client=NS(available=lambda: True, lookup_map_to_base=lambda: None))
    state = wait_ready(NS(readiness=lambda: (True, [])), stack, .3, clock=c, sleep=c.sleep)
    assert state.degraded and state.flags == {"no_drive"} and any("TF" in r for r in state.reasons)


def test_standby_ready_does_not_require_motion_permission():
    wait_ready(NS(readiness=lambda: (True, [])), NS(), .1)


def test_rejects_generic_yolo_and_accepts_mapped_labels():
    cfg = Config()
    with pytest.raises(ValueError, match="필수 클래스"):
        validate_model_labels(NS(model=NS(names={0: "person"}), names_map={}), cfg)
    labels = list(cfg.perception.object_classes + cfg.perception.target_classes) + [
        "UGV", "COMMAND_POST", "MAINT_DEPOT", "SUPPLY_BASE"]
    mapping = {"label" + str(i): v for i, v in enumerate(labels)}
    validate_model_labels(NS(model=NS(names=list(mapping)), names_map=mapping), cfg)


def test_real_profile_separates_cmd_and_disables_uncalibrated_depth():
    cfg = Config.load(str(Path(__file__).resolve().parents[1] / "config/waveshare_real.json"))
    validate_real_config(cfg)
    assert cfg.nav2.base_frame == "base_footprint"
    assert not cfg.depth.enable and not cfg.nav2.publish_depth_obstacles
    cfg.nav2.cmd_topic = cfg.hal.cmd_topic
    with pytest.raises(ValueError, match="분리"):
        validate_real_config(cfg)


def test_old_start_cleared_only_during_new_standby(tmp_path):
    p = tmp_path / "start.json"
    p.write_text('{}')
    robot = NS(status=lambda: NS(match_mode="mission", match_cmd="resume"))
    clear_standby_restore(robot, str(p))
    assert p.exists()
    robot.status = lambda: NS(match_mode="mission", match_cmd="standby")
    clear_standby_restore(robot, str(p))
    assert not p.exists()


def test_real_depth_misconfiguration_warns_and_turns_depth_off(caplog):
    # 규칙 변경(프롬프트 1장-6): 깊이는 선택 항목이다. 설정이 덜 됐으면 ValueError 로 시작을 막지 않고, 경고 후 depth.enable=False.
    # (다른 검사 — 토픽 이름·Nav2 출력 분리·API 한계 — 는 그대로 ValueError 다: 아래 두 시험이 계속 확인한다.)
    cfg = Config.load(str(Path(__file__).resolve().parents[1] / "config/waveshare_real.json"))
    cfg.depth.enable = True
    cfg.nav2.publish_depth_obstacles = True
    with caplog.at_level("WARNING"):
        validate_real_config(cfg)
    assert not cfg.depth.enable and not cfg.nav2.publish_depth_obstacles
    assert "calibration_verified" in caplog.text
    cfg.depth.enable, cfg.depth.calibration_verified = True, True
    caplog.clear()
    with caplog.at_level("WARNING"):
        validate_real_config(cfg)
    assert not cfg.depth.enable and "optical_frame" in caplog.text
    cfg.depth.enable, cfg.depth.optical_frame = True, "actual_depth_optical_frame"
    validate_real_config(cfg)
    assert cfg.depth.enable                                   # 제대로 설정되면 그대로 켜져 있다
    cfg.depth.tf_tolerance_m = float("nan")
    caplog.clear()
    with caplog.at_level("WARNING"):
        validate_real_config(cfg)
    assert not cfg.depth.enable and "허용값" in caplog.text


def test_relative_velocity_topic_cannot_alias_final_actuator_topic():
    cfg = Config.load(str(Path(__file__).resolve().parents[1] / "config/waveshare_real.json"))
    cfg.nav2.cmd_topic = "cmd_vel"
    with pytest.raises(ValueError, match="절대 이름"):
        validate_real_config(cfg)


def test_depth_preflight_error_identifies_the_bad_input():
    # 규칙 변경(프롬프트 1장): 깊이는 선택 항목 — 막지 않는다. 대신 잘못된 입력의 원인은 이유 목록에 그대로 남는다.
    c = Clock()
    robot = NS(readiness=lambda: (False, ["depth"]), stats={"depth_error": "CameraInfo frame mismatch"})
    state = wait_ready(robot, NS(), .1, clock=c, sleep=c.sleep)
    assert state.degraded and not state.flags
    assert any("CameraInfo frame mismatch" in r for r in state.reasons)


@pytest.mark.parametrize("timeout", [float("nan"), float("inf"), float("-inf"), 0])
def test_invalid_deadline_cannot_wait_forever(timeout):
    with pytest.raises(ValueError, match="finite and positive"):
        wait_ready(NS(), NS(), timeout)
