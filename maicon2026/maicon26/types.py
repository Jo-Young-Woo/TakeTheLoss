"""시스템 전반에서 쓰는 데이터 타입. 로봇 종류(실기/시뮬)와 무관한 순수 데이터."""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Tuple

import numpy as np


@dataclass
class Pose2D:
    x: float = 0.0
    y: float = 0.0
    th: float = 0.0  # rad, 반시계 +

    def copy(self) -> "Pose2D":
        return Pose2D(self.x, self.y, self.th)


@dataclass
class Scan:
    """2D LiDAR 한 바퀴. ranges[i]의 각도 = angle_min + i*angle_inc (로봇 기준, 전방 0, 왼쪽 +).
    NaN/-inf는 무효 측정(관측 없음), +inf는 측정 범위 안에 반사 없음.
    valid()는 유한한 반사 끝점만 반환한다."""
    angle_min: float
    angle_inc: float
    ranges: np.ndarray
    stamp: float = 0.0
    range_max: float = 8.0
    # Diagnostic lineage only: local monotonic scan callback entry, never ROS
    # time or a control/sensor freshness input. None for uninstrumented sources.
    receipt_wall_s: Optional[float] = None
    receipt_seq: Optional[int] = None

    @property
    def angles(self) -> np.ndarray:
        return self.angle_min + self.angle_inc * np.arange(len(self.ranges))

    def valid(self) -> np.ndarray:
        r = self.ranges
        return np.isfinite(r) & (r > 0.05) & (r < self.range_max)

    def min_in_sector(self, center: float, half_width: float) -> float:
        """center±half_width(rad) 안의 최소 거리. 없으면 range_max."""
        a = np.angle(np.exp(1j * (self.angles - center)))
        m = (np.abs(a) <= half_width) & self.valid()
        return float(self.ranges[m].min()) if m.any() else self.range_max


@dataclass
class Frame:
    """카메라 한 장. image는 BGR uint8 (H,W,3) 또는 시뮬에서는 None.
    pan/tilt는 '촬영 시점'의 값(deg) — 방위 계산에 필요."""
    image: Optional[np.ndarray]
    stamp: float
    pan: float = 0.0
    tilt: float = 0.0
    width: int = 640
    height: int = 480
    meta: Dict[str, Any] = field(default_factory=dict)


@dataclass
class DepthFrame:
    """깊이 영상 한 장. depth 는 float32 (H, W), 단위 m, z 깊이(광축 방향 거리); 0/nan = 무효.
    stamp 는 Frame 과 같은 계약(robot.now() 시계의 촬영 시각). 깊이 카메라는 팬/틸트 아래 차체에 고정이라 pan/tilt 는 3D 등록에 쓰지 않는다
    (필드는 호환용으로 남겼다).
    K 는 이 영상 해상도 기준 (fx,0,cx,0,fy,cy,0,0,1). 없으면 config.depth 의 값을 쓴다."""
    depth: np.ndarray
    stamp: float
    pan: float = 0.0
    tilt: float = 0.0
    K: Optional[Tuple[float, ...]] = None
    meta: Dict[str, Any] = field(default_factory=dict)

    @property
    def width(self) -> int:
        return int(self.depth.shape[1])

    @property
    def height(self) -> int:
        return int(self.depth.shape[0])

    def valid_ratio(self) -> float:
        d = self.depth
        return float(np.mean(np.isfinite(d) & (d > 0)))


@dataclass
class Detection:
    cls: str
    conf: float
    box: Tuple[float, float, float, float]  # x1,y1,x2,y2 (px)
    stamp: float = 0.0
    team: Optional[str] = None  # UGV일 때 'RED'/'BLUE' (안테나 색)
    extra: Dict[str, Any] = field(default_factory=dict)

    @property
    def cx(self) -> float:
        return 0.5 * (self.box[0] + self.box[2])

    @property
    def cy(self) -> float:
        return 0.5 * (self.box[1] + self.box[3])

    @property
    def w(self) -> float:
        return self.box[2] - self.box[0]

    @property
    def h(self) -> float:
        return self.box[3] - self.box[1]

    def point(self, fx: float, fy: float) -> Tuple[float, float]:
        """bbox 안의 상대 위치(fx,fy ∈ [0,1])를 픽셀 좌표로."""
        return (self.box[0] + fx * self.w, self.box[1] + fy * self.h)


@dataclass
class RobotStatus:
    """로봇이 알려주는 자기 상태(서버/ESP32 상태값)."""
    ammo: int = 8
    laser_on: bool = False
    cooldown_left: float = 0.0      # 재발사 대기 남은 시간(s)
    disabled_left: float = 0.0      # 피격 후 레이저/틸트 정지 남은 시간(s)
    drive_paused_left: float = 0.0  # 4회 누적 피격 후 주행 정지 남은 시간(s)
    hits_taken: int = 0             # 유효 피격 누적
    battery_v: float = 12.0
    # ---- 실기 API(/mil/state, /mil/match, /imu/data; 2차 사전교육 PDF p10~12, p17)에서 오는 값. 시뮬은 기본값(= 제한 없음·모름)
    control: Optional[bool] = None  # 제어 허용(practice 이거나 cmd ∈ start/resume). False = 주행 무시·조준/발사 locked. None = 모름(시뮬)
    match_mode: str = ""            # practice | mission | "" (모름)
    match_cmd: str = ""             # idle standby start pause resume report stop | "" (모름)
    hp: Optional[int] = None
    hp_max: Optional[int] = None
    battery_pct: Optional[float] = None
    roll_deg: Optional[float] = None    # IMU 기울기 (/imu/data)
    pitch_deg: Optional[float] = None
    accel_g: Optional[float] = None     # IMU 가속도 크기 [g] — 충격 감지
    ammo_free: bool = False             # 연습 모드: 탄약이 자동 회복된다(p70) → ammo 는 추정값일 뿐 사격 조건이 아니다. 기본 False = 지금과 같다

    @property
    def paused(self) -> bool:
        return self.match_cmd == "pause"


def gate_ammo(status, capacity: int) -> int:
    """사격 '조건'으로 쓰는 탄약. 연습 모드(status.ammo_free)는 자동 회복되므로 가득으로 본다. mission 은 status.ammo 그대로.
    status 에 ammo_free 가 없는 객체(시뮬·가짜)도 그대로 status.ammo 를 돌려준다."""
    return int(capacity) if getattr(status, "ammo_free", False) else status.ammo


@dataclass
class Command:
    """M1 해석 결과. None은 '명령에 없었음'."""
    target: Optional[str] = None            # 지정표적 canonical key (예: RADAR_FACILITY)
    sectors: List[str] = field(default_factory=list)
    goal_base: Optional[str] = None
    raw: str = ""
    confidence: float = 0.0
    source: str = "rule"                    # rule | llm | merged
    notes: List[str] = field(default_factory=list)

    def to_payload(self) -> Dict[str, Any]:
        """M1 제출 JSON. 키 이름은 1차 사전교육 예시(target/sectors/goal_base)를 따른다."""
        return {"target": self.target, "sectors": list(self.sectors), "goal_base": self.goal_base}


def dist(a: Tuple[float, float], b: Tuple[float, float]) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])
