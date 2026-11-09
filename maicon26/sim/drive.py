"""시뮬 구동 특성(보정 시험의 '정답' 노브). 기본값은 전부 중립 → 기존 시뮬 결과를 바꾸지 않는다.

실기에서 측정해 보정표에 넣는 항목과 1:1 대응한다(maicon26/calib/):
  lag_s        명령→반응 지연           a_brake      제동 감속도(가속 a_max 와 다름)
  lr_bias      직진 시 좌우 편차(rad/m)  pivot_loss   제자리 회전 효율 손실(슬립)
  pivot_drift  제자리 회전 중 위치 이동   surfaces     주행면별 (슬립, 가속배율, 제동배율)
  odom_*       오도메트리 선형/회전 스케일, 자이로 편향   imu_*  IMU 편향·잡음·지연
  lidar_xy/yaw LiDAR 장착 오프셋(차체 중심 기준)
"""
from __future__ import annotations

import dataclasses
import json
from dataclasses import dataclass, field
from typing import Dict, Optional, Tuple


@dataclass
class SimDrive:
    a_brake: Optional[float] = None                 # None → robot.a_max
    alpha: float = 8.0                              # rad/s² 각가속 제한(기존 시뮬 상수)
    lag_s: float = 0.0
    lr_bias: float = 0.0
    pivot_loss: float = 0.0
    pivot_drift: float = 0.0
    odom_scale: Optional[float] = None              # None → 기존처럼 시드 기반 무작위(±2 %)
    odom_yaw_scale: float = 1.0
    gyro_bias: Optional[float] = None               # 오도메트리 yaw 편향 rad/s. None → 기존 무작위
    imu_bias: float = 0.0
    imu_noise: float = 0.0
    imu_lag_s: float = 0.0
    lidar_xy: Tuple[float, float] = (0.0, 0.0)
    lidar_yaw: float = 0.0
    surface: str = "default"
    surfaces: Dict[str, Tuple[float, float, float]] = field(default_factory=lambda: {"default": (0.0, 1.0, 1.0)})

    def surf(self) -> Tuple[float, float, float]:
        return self.surfaces.get(self.surface, (0.0, 1.0, 1.0))

    def to_dict(self) -> dict:
        return dataclasses.asdict(self)

    @classmethod
    def from_dict(cls, d: dict) -> "SimDrive":
        d = dict(d)
        for k in ("lidar_xy",):
            if k in d:
                d[k] = tuple(d[k])
        if "surfaces" in d:
            d["surfaces"] = {k: tuple(v) for k, v in d["surfaces"].items()}
        return cls(**d)

    @classmethod
    def load(cls, path: str) -> "SimDrive":
        with open(path, "r", encoding="utf-8") as f:
            return cls.from_dict(json.load(f))


# 보정 파이프라인 시험용 '숨은 정답' — 분석 코드가 이 값을 복원하는지 본다(tests/test_calib.py).
def truth_for_tests() -> SimDrive:
    return SimDrive(a_brake=1.9, lag_s=0.12, lr_bias=0.025, pivot_loss=0.06, pivot_drift=0.02,
                    odom_scale=1.03, odom_yaw_scale=0.96, gyro_bias=0.004,
                    imu_bias=-0.006, imu_noise=0.002, imu_lag_s=0.03,
                    lidar_xy=(0.04, -0.01), lidar_yaw=0.07, surface="default",
                    surfaces={"default": (0.0, 1.0, 1.0), "tile": (0.01, 1.0, 1.0),
                              "carpet": (0.06, 0.8, 0.7)})
