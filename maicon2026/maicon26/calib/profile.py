"""로봇 프로파일 — 모델·구동·센서·항법 값의 단일 출처(single source of truth) + 값마다 출처(provenance).

출처 표기:  pdf      1차 사전교육 자료에 적힌 값
            measured 보정 시험(calib/)에서 측정한 값
            tuned    시뮬/현장 튜닝으로 정한 값
            derived  다른 값에서 계산한 값
            assumed  자료에 없어서 둔 가정 → 실기에서 측정해 교체할 것 (calib_report.py 가 목록을 뽑는다)

파일: calib/robot_profile.yaml  (사람이 읽고 고칠 수 있는 YAML).  측정 결과는 calib_fit.py 가 measured 로 덮어쓴다.
"""
from __future__ import annotations

import copy
import datetime as _dt
import json
import math
from typing import Any, Dict, Iterator, List, Optional, Tuple

SRC_ORDER = ["assumed", "tuned", "derived", "pdf", "measured"]


def default_data() -> Dict[str, Any]:
    """WINGBOT-UGV 초기값. 확실한 것만 pdf, 나머지는 assumed."""
    return {
        "meta": {"robot": "WINGBOT-UGV", "schema": 1, "created": _dt.date.today().isoformat(),
                 "note": "PDF p20-21 사양 + 가정. 측정값은 calib_fit.py 가 채운다."},
        "model": {
            "size": {"x": 0.250, "y": 0.230, "z": 0.320},          # PDF: 250(가로) x 230(세로) x 320(높이) mm. 가로를 전진(x) 방향으로 가정
            "mass": 2.9,                                           # PDF: 약 2.9 kg
            "chassis_height": 0.110,                               # 섀시 몸통 높이(나머지는 카메라/안테나 마스트)  [assumed]
            "com": {"x": 0.0, "y": 0.0, "z": 0.090},               # 무게중심(base_link 기준)                          [assumed]
            "inertia_override": None,                              # {ixx,iyy,izz} 를 알면 넣는다. None → 직육면체 근사
            "wheels": {"count": 6, "radius": 0.040, "width": 0.030,   # PDF: 6WD. 반경·폭은                               [assumed]
                       "x": [0.090, 0.0, -0.090], "track": 0.200},    # 앞/중/뒤 x, 좌우 바퀴 중심 간격                     [assumed]
            # 충돌 외곽 = size(x,y) 직사각형 [pdf]. 원 근사가 필요하면 외접 반경(0.5·√(x²+y²))을 쓴다 — 더 작은 원으로 모서리 충돌을 가리지 않는다.
            "collision": {"shape": "box", "padding": 0.0},
            "pan_tilt": {"xyz": [0.0, 0.0, 0.200], "pan_limit_deg": 50.0, "tilt_limit_deg": 15.0},  # PDF: 수평 100°/수직 30° → ±50°/±15°
        },
        "drive": {
            "v_max": 0.45, "v_max_reverse": 0.45,                  # m/s                                              [assumed] → straight/speed_steps
            "a_accel": 1.2, "a_brake": 1.2,                        # m/s²                                             [assumed] → speed_steps/brake
            "w_max": 2.2, "alpha_max": 8.0,                        # rad/s, rad/s²                                    [assumed] → turn_steps
            "cmd_latency_s": 0.0, "rise_time_s": 0.0,              # 명령→실제 움직임 지연, 상승시간(10→90 %)             [assumed]
            "lr_bias_rad_per_m": 0.0,                              # 직진 시 좌우 편차 (+ = 왼쪽으로 휨)                  [assumed]
            "stop_distance": {},                                   # {"0.30": m}  시험으로 채움 (속도→정지거리)
            "pivot": {"yaw_efficiency": 1.0, "drift_m_per_rad": 0.0, "overshoot_deg": 0.0},
            "surfaces": {"default": {"slip": 0.0, "a_accel": None, "a_brake": None, "mu_lower_bound": None,
                                     "mu_pull_test_N": None}},
        },
        "sensors": {
            "lidar": {"model": "D500", "frame": "laser", "xyz": [0.0, 0.0, 0.200], "rpy": [0.0, 0.0, 0.0],   # TF base_link→laser [assumed]
                      "rate_hz": 10.0, "range_min": 0.05, "range_max": 8.0, "latency_s": 0.0,
                      # LiDAR 불확실도 — PDF 에 정확도 수치가 없다. 전부 [assumed]; 현장 정지 표적 반복 측정으로 measured 로 바꾼다.
                      # 6종: 거리 편향·잡음, 각도 오차, 장착 위치·yaw 오차, 전달 지연, (누락·이상치는 지속성 필터로 처리), + 자기 위치추정 오차(pose_sigma_m)
                      "error": {"range_bias_m": 0.005, "range_sigma_m": 0.008, "angle_sigma_deg": 0.3, "mount_xy_m": 0.005,
                                "mount_yaw_deg": 0.5, "latency_s": 0.10, "pose_sigma_m": 0.010, "dropout_p": 0.02, "outlier_p": 0.005,
                                "k_sigma": 2.0, "ref_range_m": 0.5}},
            "camera": {"frame": "camera_optical", "parent": "tilt_link", "xyz": [0.05, 0.0, 0.030], "rpy": [0.0, 0.0, 0.0],  # [assumed]
                       "width": 640, "height": 480, "hfov_deg": 110.0,    # PDF: 160°·5 MP 센서 중 사용 해상도/보정 후 유효 화각 [assumed]
                       "model": "plumb_bob", "K": None, "D": None, "calibrated": False, "latency_s": 0.0},
            # OAK Stereo: 2축 팬/틸트 '아래'에 붙은 고정 센서(사용자 확인) → base_link 기준, 팬/틸트와 같이 움직이지 않는다. 위치는 [assumed]
            "depth": {"frame": "depth_optical", "parent": "base_link", "xyz": [0.10, 0.0, 0.10], "rpy": [0.0, 0.0, 0.0],
                      # 깊이 영상(스트림 해상도 기준). K 가 None 이면 hfov 로 근사. 실기: depth camera_info 의 K 또는 scripts/calib_depth.py 결과
                      "width": 640, "height": 400, "hfov_deg": 72.0, "K": None, "calibrated": False,
                      "unit_scale": 0.001, "min_range": 0.30, "max_range": 3.5, "latency_s": 0.0},
            "imu": {"frame": "imu_link", "xyz": [0.0, 0.0, 0.050], "rpy": [0.0, 0.0, 0.0],                                 # [assumed]
                    "gyro_bias_z": 0.0, "gyro_noise_z": 0.0, "gyro_scale": 1.0, "latency_s": 0.0},
            "odom": {"frame": "odom", "child": "base_link", "lin_scale": 1.0, "yaw_scale": 1.0,
                     "gyro_bias_z": 0.0, "noise_v": 0.0, "noise_w": 0.0, "latency_s": 0.0,
                     "pos_error_p95_m_per_m": 0.05},
        },
        "nav": {
            "footprint": {"mode": "rect", "margin": None},         # Nav2 풋프린트 = size 직사각형 + 계획 여유. margin None → LiDAR 오차·위치추정 오차에서 유도(footprint.py)
            "clearance": {"corridor_min": 0.40, "inflation_radius": 0.30, "cost_scaling": 8.0},
            "follow": {"desired_linear_vel": None, "lookahead_dist": None, "rotate_to_heading_w": None},  # None → 보정표에서 유도
            "recovery": {"spin_dist": 1.57, "backup_dist": 0.25, "backup_speed": 0.05, "wait_s": 2.0},
            "goal": {"xy_tol": 0.08, "base_entry_tol": 0.10},
        },
        "provenance": {
            "model.size.x": "pdf", "model.size.y": "pdf", "model.size.z": "pdf", "model.mass": "pdf",
            "model.wheels.count": "pdf", "model.pan_tilt.pan_limit_deg": "pdf", "model.pan_tilt.tilt_limit_deg": "pdf",
            "model.collision.shape": "tuned", "model.collision.padding": "tuned",
            "sensors.lidar.error.range_bias_m": "assumed", "sensors.lidar.error.range_sigma_m": "assumed",
            "sensors.lidar.error.angle_sigma_deg": "assumed", "sensors.lidar.error.mount_xy_m": "assumed",
            "sensors.lidar.error.mount_yaw_deg": "assumed", "sensors.lidar.error.latency_s": "assumed",
            "sensors.lidar.error.pose_sigma_m": "assumed",
            "nav.follow.desired_linear_vel": "derived", "nav.follow.lookahead_dist": "derived", "nav.follow.rotate_to_heading_w": "derived",
            "sensors.lidar.model": "pdf", "sensors.lidar.range_max": "tuned",
            "sensors.camera.hfov_deg": "tuned", "sensors.camera.width": "tuned", "sensors.camera.height": "tuned",
            "sensors.odom.frame": "tuned", "sensors.odom.child": "tuned", "sensors.lidar.frame": "tuned",
            "nav.footprint.margin": "derived", "nav.footprint.mode": "tuned",
            "nav.clearance.corridor_min": "pdf",
            "nav.recovery.spin_dist": "tuned", "nav.recovery.backup_dist": "tuned",
            "nav.recovery.backup_speed": "tuned", "nav.recovery.wait_s": "tuned",
            "nav.clearance.inflation_radius": "tuned", "nav.clearance.cost_scaling": "tuned",
            "nav.goal.xy_tol": "tuned", "nav.goal.base_entry_tol": "tuned",
        },
        "measurements": [],
    }


class Profile:
    def __init__(self, data: Optional[Dict[str, Any]] = None) -> None:
        self.data: Dict[str, Any] = data if data is not None else default_data()

    # ------------------------------------------------------------------ 접근
    def get(self, path: str, default: Any = None) -> Any:
        cur: Any = self.data
        for k in path.split("."):
            if isinstance(cur, dict) and k in cur:
                cur = cur[k]
            else:
                return default
        return cur

    def set(self, path: str, value: Any, src: str = "measured", note: Optional[str] = None) -> None:
        ks = path.split(".")
        cur = self.data
        for k in ks[:-1]:
            cur = cur.setdefault(k, {})
        cur[ks[-1]] = value
        self.data.setdefault("provenance", {})[path] = src
        if note:
            self.data.setdefault("notes", {})[path] = note

    def src(self, path: str) -> str:
        return self.data.get("provenance", {}).get(path, "assumed")

    def leaves(self, root: str = "") -> Iterator[Tuple[str, Any]]:
        """스칼라/리스트 말단 (경로, 값). provenance/measurements/meta/notes 는 제외."""
        def walk(prefix: str, node: Any) -> Iterator[Tuple[str, Any]]:
            if isinstance(node, dict) and node and prefix.split(".")[0] not in ("provenance", "measurements", "meta", "notes"):
                for k, v in node.items():
                    yield from walk(f"{prefix}.{k}" if prefix else k, v)
            elif prefix and prefix.split(".")[0] not in ("provenance", "measurements", "meta", "notes"):
                yield prefix, node
        yield from walk("", self.data)

    def assumed(self) -> List[Tuple[str, Any]]:
        """아직 가정인 값들(실기에서 측정해 교체해야 할 목록). None/빈 dict 는 '미측정'으로 포함."""
        return [(p, v) for p, v in self.leaves() if self.src(p) == "assumed"]

    def add_measurement(self, rec: Dict[str, Any]) -> None:
        self.data.setdefault("measurements", []).append(rec)

    def copy(self) -> "Profile":
        return Profile(copy.deepcopy(self.data))

    # ------------------------------------------------------------------ 파생 기하
    def inscribed_radius(self) -> float:
        return 0.5 * min(self.get("model.size.x"), self.get("model.size.y"))

    def circumscribed_radius(self) -> float:
        return 0.5 * math.hypot(self.get("model.size.x"), self.get("model.size.y"))

    def inertia(self) -> Dict[str, float]:
        ov = self.get("model.inertia_override")
        if ov:
            return dict(ov)
        m, x, y = self.get("model.mass"), self.get("model.size.x"), self.get("model.size.y")
        h = self.get("model.size.z")
        return {"ixx": m * (y * y + h * h) / 12.0, "iyy": m * (x * x + h * h) / 12.0, "izz": m * (x * x + y * y) / 12.0}

    # ------------------------------------------------------------------ 입출력
    def save(self, path: str) -> None:
        import yaml  # type: ignore
        with open(path, "w", encoding="utf-8") as f:
            yaml.safe_dump(self.data, f, allow_unicode=True, sort_keys=False, default_flow_style=None)

    @classmethod
    def load(cls, path: str) -> "Profile":
        import yaml  # type: ignore
        with open(path, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f)
        base = default_data()
        _fill_missing(base, data)                              # 옛 파일에 새 키가 없어도 기본값으로 채운다
        return cls(data)

    def to_json(self) -> str:
        return json.dumps(self.data, ensure_ascii=False, indent=2)


def _fill_missing(base: Dict[str, Any], data: Dict[str, Any]) -> None:
    for k, v in base.items():
        if k not in data:
            data[k] = copy.deepcopy(v)
        elif isinstance(v, dict) and isinstance(data[k], dict) and k not in ("provenance", "surfaces", "stop_distance"):
            _fill_missing(v, data[k])
