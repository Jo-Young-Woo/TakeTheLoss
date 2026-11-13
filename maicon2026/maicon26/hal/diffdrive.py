"""차동(스키드 스티어) 구동 운동학. 차체 속도 (v, w) ↔ 좌우 바퀴 속도 (v_L, v_R).

  v_L = v − w·b/2        v_R = v + w·b/2        (b = 좌우 바퀴 간격, w 는 반시계 +)

6WD 는 좌우 3개씩이 같은 속도로 돈다고 본다. 이 모듈이 하는 일:
  1. wheels_from_twist / twist_from_wheels  — 변환. 바퀴 속도를 직접 받는 드라이버(hal/wingbot.py)용.
  2. compensate  — 보정으로 잰 슬립(yaw_eff)과 직진 편차(lr_bias)를 지령에서 미리 뺀다.
        실제 w = yaw_eff · w_cmd + lr_bias · v   →   w_cmd = (w_원하는 − lr_bias · v) / yaw_eff
  3. limit  — 바퀴 하나가 한계를 넘으면 v, w 를 **같은 비율로** 줄여 곡률(w/v)을 유지한다.
        (v, w 를 따로 자르면 빠르게 달리며 크게 도는 순간 호가 바깥으로 벌어진다)
  4. command  — compensate → limit. Agent 가 로봇에 보내기 직전에 한 번 거친다.

한계: yaw_eff 는 제자리 회전에서 잰 값이라 주행 중 호에도 같은 값을 쓴다(바닥·속도에 따라 다를 수 있다).
lr_bias 는 주행면마다 다를 수 있으나 한 값만 쓴다. 둘 다 실기 측정 전에는 중립(1.0, 0.0)이다.
"""
from __future__ import annotations

from typing import Tuple

from ..config import RobotCfg


class DiffDrive:
    def __init__(self, cfg: RobotCfg) -> None:
        self.cfg = cfg
        self.n_limited = 0               # 바퀴 한계로 지령을 줄인 횟수(진단용)
        self.n_calls = 0

    # ------------------------------------------------------------------ 변환
    @property
    def track(self) -> float:
        return max(self.cfg.track_width, 1e-3)

    @property
    def wheel_max(self) -> float:
        return self.cfg.wheel_v_max if self.cfg.wheel_v_max > 0 else self.cfg.v_max

    def wheels_from_twist(self, v: float, w: float) -> Tuple[float, float]:
        """(v, w) → 왼쪽·오른쪽 바퀴 선속도 [m/s]."""
        h = 0.5 * self.track * w
        return v - h, v + h

    def twist_from_wheels(self, v_left: float, v_right: float) -> Tuple[float, float]:
        return 0.5 * (v_left + v_right), (v_right - v_left) / self.track

    def wheel_rates(self, v: float, w: float) -> Tuple[float, float]:
        """(v, w) → 왼쪽·오른쪽 바퀴 각속도 [rad/s] (모터 드라이버가 rad/s 나 rpm 을 받을 때)."""
        vl, vr = self.wheels_from_twist(v, w)
        r = max(self.cfg.wheel_radius, 1e-3)
        return vl / r, vr / r

    # ------------------------------------------------------------------ 보상 / 제한
    def compensate(self, v: float, w: float) -> Tuple[float, float]:
        """원하는 실제 (v, w) → 슬립·편차를 미리 뺀 지령. yaw_eff=1, lr_bias=0 이면 항등."""
        eff = self.cfg.yaw_eff if self.cfg.yaw_eff > 0.2 else 1.0     # 비정상 값 방어
        return v, (w - self.cfg.lr_bias * v) / eff

    def limit(self, v: float, w: float) -> Tuple[float, float, bool]:
        """바퀴 한계를 넘으면 곡률을 유지한 채 비례 축소. → (v, w, 줄였는가)"""
        if not self.cfg.wheel_limit:
            return v, w, False
        vl, vr = self.wheels_from_twist(v, w)
        peak = max(abs(vl), abs(vr))
        lim = self.wheel_max
        if peak <= lim or peak < 1e-9:
            return v, w, False
        k = lim / peak
        return v * k, w * k, True

    def command(self, v: float, w: float) -> Tuple[float, float]:
        """Agent → 로봇 최종 지령: 보상 후 바퀴 한계."""
        self.n_calls += 1
        v, w = self.compensate(v, w)
        v, w, cut = self.limit(v, w)
        self.n_limited += int(cut)
        return v, w
