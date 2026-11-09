"""공용 수학 유틸: 각도 정규화, 클램프, PID, 저역통과.

좌표 규약(ROS REP-103): x 전방, y 왼쪽, θ 반시계(+). 방위각도 왼쪽이 +.
"""
from __future__ import annotations

import math
from typing import Optional


def wrap(a: float) -> float:
    """각도를 (-π, π]로 정규화."""
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def clamp(x: float, lo: float, hi: float) -> float:
    return lo if x < lo else hi if x > hi else x


def sign(x: float) -> float:
    return 1.0 if x >= 0 else -1.0


class PID:
    """위치형 PID. dt는 호출 때마다 실측값을 넘긴다.

    - 미분은 측정값이 아니라 '오차'가 아닌 d(error)/dt를 LPF 후 사용(setpoint kick 완화는
      호출측에서 setpoint를 자주 바꾸지 않는 설계로 해결).
    - anti-windup: 출력이 포화되면 적분을 더 쌓지 않는다(조건부 적분).
    """

    def __init__(self, kp: float, ki: float = 0.0, kd: float = 0.0,
                 out_min: float = -math.inf, out_max: float = math.inf,
                 d_alpha: float = 0.5, i_limit: Optional[float] = None) -> None:
        self.kp, self.ki, self.kd = kp, ki, kd
        self.out_min, self.out_max = out_min, out_max
        self.d_alpha = d_alpha
        self.i_limit = i_limit
        self.reset()

    def reset(self) -> None:
        self._i = 0.0
        self._prev: Optional[float] = None
        self._d = 0.0

    def update(self, error: float, dt: float) -> float:
        if dt <= 0:
            return clamp(self.kp * error, self.out_min, self.out_max)
        if self._prev is None:
            d_raw = 0.0
        else:
            d_raw = (error - self._prev) / dt
        self._prev = error
        self._d = self.d_alpha * d_raw + (1 - self.d_alpha) * self._d
        i_new = self._i + error * dt
        if self.i_limit is not None:
            i_new = clamp(i_new, -self.i_limit, self.i_limit)
        u = self.kp * error + self.ki * i_new + self.kd * self._d
        u_sat = clamp(u, self.out_min, self.out_max)
        # 포화 중에 오차가 포화 방향으로 계속 쌓이면 적분 정지
        if u == u_sat or (u > u_sat and error < 0) or (u < u_sat and error > 0):
            self._i = i_new
        return u_sat


class LowPass:
    def __init__(self, alpha: float, init: Optional[float] = None) -> None:
        self.alpha = alpha
        self.y = init

    def update(self, x: float) -> float:
        self.y = x if self.y is None else self.alpha * x + (1 - self.alpha) * self.y
        return self.y


def rot(x: float, y: float, th: float):
    """(x, y)를 θ만큼 반시계 회전."""
    c, s = math.cos(th), math.sin(th)
    return c * x - s * y, s * x + c * y
