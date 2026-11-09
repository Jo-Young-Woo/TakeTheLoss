"""2D 교전 시뮬레이터(키네매틱). 인식 '모델'은 시뮬하지 않고, 카메라 투영 + 가림 + 잡음으로
'검출 결과'를 만든다. 목적은 제어·임무 로직(FSM, 조준, 항법, 규칙 준수)의 회귀 시험이다.

구현한 대회 규칙(1차 사전교육 자료 p14~16):
  - 레이저: 탄창 8발, 1발 5초 발광, 재발사 대기 5초, 수광부 2초 연속 수광 = 유효타격
  - 유효피격 후 10초간 피격자의 레이저·팬틸트 정지 + 10초 무적, 4회 누적 시 10초간 주행도 정지
  - 아군 오사도 유효피격
  - UAV 1 FPS, 경기 5분, 4:00 목표거점 명령, 5:00 정지 + 30초 보고
"""
from __future__ import annotations

import math
import random
import zlib
from collections import deque
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Tuple

import numpy as np

from ..config import Config
from ..footprint import Footprint, ray_rect, sat_push
from ..footprint import aabb_corners as aabb_corners
from ..geom3d import RigModel, Rt
from ..perception.camera import CameraModel
from ..types import Detection, Pose2D, RobotStatus, Scan
from ..util import clamp, wrap
from .drive import SimDrive

FACE_N = [(1.0, 0.0), (0.0, 1.0), (-1.0, 0.0), (0.0, -1.0)]  # E N W S
UGV_R = 0.13
UGV_H = 0.32
# 수광센서 크기·위치는 PDF 사진(p12 표적 면, p21 UGV)에서 짐작한 값이다 — 실측이 아니다.
RECEIVER_Z = (0.10, 0.18)    # UGV: 상판 위 하얀 돔(좌/우 앞 모서리). 높이는 사진 비율로 추정
RECEIVER_LAT = 0.09          # 중심에서 좌/우 수광센서까지
RECEIVER_RADIUS = 0.035      # 돔 반경(사진상 차체 폭 250 mm 대비 약 3.5 cm)
TARGET_WINDOW_Z = (0.30, 0.37)   # 지정표적: 면(400 mm) 위쪽 중앙의 흰 원판 — 높이 약 84 %, 지름 약 6 cm
TARGET_WINDOW_LAT = 0.035


@dataclass
class Item:
    kind: str                  # 'object' | 'target' | 'base'
    cls: str
    led_on: bool = False
    contact: float = 0.0
    last_contact_t: float = -1e9
    hit_by: Optional[str] = None


@dataclass
class Cube:
    cx: float
    cy: float
    items: Dict[int, Item] = field(default_factory=dict)
    half: float = 0.2
    sx: float = 0.4
    sy: float = 0.4

    def rect(self):
        return (self.cx - self.sx / 2, self.cy - self.sy / 2,
                self.cx + self.sx / 2, self.cy + self.sy / 2)


class SimUGV:
    def __init__(self, world: "World", uid: str, team: str, slot: int, pose: Pose2D, cfg: Config,
                 rng: random.Random) -> None:
        self.world, self.id, self.team, self.slot, self.cfg = world, uid, team, slot, cfg
        self.pose = pose
        self.v = self.w = 0.0
        self.cmd_v = self.cmd_w = 0.0
        self.pan = self.tilt = 0.0
        self.pan_cmd = self.tilt_cmd = 0.0
        L = cfg.laser
        self.ammo = L.ammo
        self.laser_on = False
        self.laser_left = 0.0
        self.cooldown_left = 0.0
        self.hits_taken = 0
        self.invuln_left = 0.0
        self.disabled_left = 0.0
        self.paused_left = 0.0
        self.contact: Dict[str, Tuple[float, float]] = {}   # 피격 대상 키 → (누적, 마지막 접촉 시각)
        # 통계
        self.shots = 0
        self.hits_dealt: List[str] = []
        self.friendly_hits = 0
        self.targets_hit: List[str] = []
        self.distance = 0.0
        self.collisions = 0
        self.reached_base: Optional[float] = None
        # 오도메트리(시작 기준, 잡음/드리프트 포함)
        self.odom = Pose2D(0, 0, 0)
        self._scale = 1.0 + rng.gauss(0, 0.02)
        self._gyro_bias = rng.gauss(0, 0.0012)
        self.rng = rng
        self.trail: List[Tuple[float, float]] = []
        self.goal_base: Optional[str] = None
        # 구동 특성(보정 시험용; 기본은 중립) / IMU
        self.drive = SimDrive()
        self._cmd_q: deque = deque()
        self._w_hist: deque = deque(maxlen=400)
        self.v_true = self.w_true = 0.0
        self.imu_rng = random.Random(zlib.crc32(uid.encode()))

    def imu_gyro_z(self) -> float:
        """IMU 자이로 z (rad/s): 지연된 실제 각속도 + 편향 + 잡음."""
        d, t = self.drive, self.world.t - self.drive.imu_lag_s
        w = 0.0
        for ts, wv in reversed(self._w_hist):
            if ts <= t:
                w = wv
                break
        return w + d.imu_bias + (self.imu_rng.gauss(0, d.imu_noise) if d.imu_noise > 0 else 0.0)

    # ------------------------------------------------------------------ 물리
    def step(self, dt: float) -> None:
        r = self.cfg.robot
        for name in ("laser_left", "cooldown_left", "invuln_left", "disabled_left", "paused_left"):
            setattr(self, name, max(0.0, getattr(self, name) - dt))
        if self.laser_on and self.laser_left <= 0:
            self.laser_on = False
            self.cooldown_left = self.cfg.laser.cooldown_s
        d = self.drive
        slip, acc_s, brk_s = d.surf()
        if self.paused_left > 0 or self.world.ended:
            cv, cw = 0.0, 0.0
        else:
            cv = clamp(self.cmd_v, -r.v_max, r.v_max)
            cw = clamp(self.cmd_w, -r.w_max, r.w_max)
            if d.lag_s > 0:                                  # 명령 지연: lag_s 이전의 가장 최신 명령을 적용
                self._cmd_q.append((self.world.t, cv, cw))
                while len(self._cmd_q) > 1 and self._cmd_q[1][0] <= self.world.t - d.lag_s:
                    self._cmd_q.popleft()
                cv, cw = self._cmd_q[0][1], self._cmd_q[0][2]
        a_dn = (d.a_brake if d.a_brake is not None else r.a_max) * brk_s
        lim = a_dn if abs(cv) < abs(self.v) else r.a_max * acc_s     # 제동은 가속과 다른 한계
        self.v += clamp(cv - self.v, -lim * dt, lim * dt)
        self.w += clamp(cw - self.w, -d.alpha * dt, d.alpha * dt)
        x0, y0 = self.pose.x, self.pose.y
        v_eff = self.v * (1.0 - slip)                        # 바닥 슬립: 바퀴가 돈 만큼 못 간다
        w_eff = self.w * (1.0 - d.pivot_loss) + d.lr_bias * v_eff    # 제자리 회전 손실 + 직진 시 좌우 편차
        self.v_true, self.w_true = v_eff, w_eff
        self.pose.th = wrap(self.pose.th + w_eff * dt)
        self.pose.x += v_eff * math.cos(self.pose.th) * dt
        self.pose.y += v_eff * math.sin(self.pose.th) * dt
        if d.pivot_drift:                                    # 스키드 조향: 회전 중 차체 앞뒤로 밀림
            self.pose.x += d.pivot_drift * abs(self.w) * math.cos(self.pose.th) * dt
            self.pose.y += d.pivot_drift * abs(self.w) * math.sin(self.pose.th) * dt
        self.world.resolve_collisions(self)
        self.distance += math.hypot(self.pose.x - x0, self.pose.y - y0)
        self._w_hist.append((self.world.t, w_eff))
        # 오도메트리: 스케일 오차 + 자이로 바이어스 + 잡음
        sc = d.odom_scale if d.odom_scale is not None else self._scale
        gb = d.gyro_bias if d.gyro_bias is not None else self._gyro_bias
        ov = self.v * sc + self.rng.gauss(0, 0.004)
        ow = self.w * d.odom_yaw_scale + gb + self.rng.gauss(0, 0.004)
        self.odom.th = wrap(self.odom.th + ow * dt)
        self.odom.x += ov * math.cos(self.odom.th) * dt
        self.odom.y += ov * math.sin(self.odom.th) * dt
        # 서보(피격 정지 중에는 팬/틸트 고정)
        if self.disabled_left <= 0:
            step = r.pan_rate * dt
            self.pan += clamp(self.pan_cmd - self.pan, -step, step)
            self.tilt += clamp(self.tilt_cmd - self.tilt, -step, step)
        self.pan = clamp(self.pan, -r.pan_limit, r.pan_limit)
        self.tilt = clamp(self.tilt, -r.tilt_limit, r.tilt_limit)
        if int(self.world.t * 5) != int((self.world.t - dt) * 5):
            self.trail.append((self.pose.x, self.pose.y))

    # ------------------------------------------------------------------ 명령(HAL이 호출)
    def fire(self) -> bool:
        if (self.ammo <= 0 or self.laser_on or self.cooldown_left > 0 or self.disabled_left > 0
                or self.world.ended):
            return False
        self.ammo -= 1
        self.shots += 1
        self.laser_on = True
        self.laser_left = self.cfg.laser.shot_s
        return True

    def abort(self) -> None:
        if self.laser_on:
            self.laser_on = False
            self.laser_left = 0.0
            self.cooldown_left = self.cfg.laser.cooldown_s

    def status(self) -> RobotStatus:
        return RobotStatus(ammo=self.ammo, laser_on=self.laser_on, cooldown_left=self.cooldown_left,
                           disabled_left=self.disabled_left, drive_paused_left=self.paused_left,
                           hits_taken=self.hits_taken)

    # ------------------------------------------------------------------ 기하
    def receivers(self) -> List[Tuple[float, float]]:
        c, s = math.cos(self.pose.th), math.sin(self.pose.th)
        out = []
        for lat in (RECEIVER_LAT, -RECEIVER_LAT):
            out.append((self.pose.x + 0.06 * c - lat * s, self.pose.y + 0.06 * s + lat * c))
        return out

    def cam_pos(self) -> Tuple[float, float, float]:
        return (self.pose.x, self.pose.y, self.cfg.robot.cam_height)


def _ray_aabb(ox, oy, dx, dy, rect, tmax=1e9):
    """레이-AABB. (t, face_idx) 또는 None. face_idx는 맞은 면의 바깥 법선(E,N,W,S)."""
    x0, y0, x1, y1 = rect
    tmin, tmaxi = 0.0, tmax
    face = None
    for axis, (o, d, lo, hi) in enumerate(((ox, dx, x0, x1), (oy, dy, y0, y1))):
        if abs(d) < 1e-12:
            if o < lo or o > hi:
                return None
            continue
        t1, t2 = (lo - o) / d, (hi - o) / d
        f1 = 2 if axis == 0 else 3     # lo 쪽 면: W 또는 S
        f2 = 0 if axis == 0 else 1     # hi 쪽 면: E 또는 N
        if t1 > t2:
            t1, t2 = t2, t1
            f1, f2 = f2, f1
        if t1 > tmin:
            tmin, face = t1, f1
        tmaxi = min(tmaxi, t2)
        if tmin > tmaxi:
            return None
    if face is None:        # 시작점이 박스 안
        return None
    return tmin, face


class World:
    def __init__(self, cfg: Optional[Config] = None, seed: int = 0, m5_ends_match: bool = True,
                 uav_period: float = 1.0, layout_seed: Optional[int] = None,
                 scenario: str = "random", fixed_m3_count: int = 4) -> None:
        self.cfg = cfg or Config()
        self.geo = self.cfg.arena
        self.scenario = scenario
        self.rng = random.Random(seed)
        self.sensor_rng = random.Random(seed * 7919 + 13)    # 수광센서 위치 노이즈 전용 — 공용 난수열을 건드리지 않는다
        self.t = 0.0
        self.ended = False
        self.finished = False
        self.end_time: Optional[float] = None
        self.m5_ends_match = m5_ends_match
        self.goal_base_types: Dict[str, str] = {}  # one M5 objective per alliance
        self.m5_winner: Optional[str] = None
        self._base_contacts: Dict[Tuple[str, str], float] = {}
        self.uav_period = uav_period
        # ---- 인지 잡음 노브(시험용): 기본값은 이전 하드코딩과 같다. 실기 값이 아니다 — 시나리오에서 바꿔 민감도를 본다
        self.ugv_det_range = 3.5          # m: 카메라 UGV 검출 최대 거리
        self.ugv_miss_base = 0.05         # 카메라가 UGV 를 놓칠 확률(거리 2 m 이하)
        self.ugv_miss_slope = 0.04        # 2 m 넘는 거리 1 m 당 추가 누락 확률
        self.team_flip = 0.04             # 안테나 색(팀) 오판 확률
        self.uav_sigma = 0.04             # m: UAV 위치 잡음
        self.uav_latency = 0.4            # s: UAV 영상 지연
        self.uav_enabled = True           # False 면 UAV 영상 없음(실기 get_uav 가 None 인 상황)
        self.ugvs: List[SimUGV] = []
        self.cubes: List[Cube] = []
        self.base_types: Dict[str, str] = {}
        self.events: List[Dict[str, Any]] = []
        # LiDAR 평면(약 0.24 m) 아래의 낮은 물체: (x0, y0, x1, y1, 높이). 깊이 카메라에만 보이고 로봇은 부딪힌다 — 3D 지도 시험용(규정에 있는 물체가 아니다)
        self.low_obstacles: List[Tuple[float, float, float, float, float]] = []
        self._depth_rays: Dict[Tuple[int, int, float], np.ndarray] = {}
        self.smog = False                                              # 스모그 발생장치(경기장 중앙 측면 1곳에서 분출)
        self.smog_center = (self.geo.W / 2, 0.0)
        self.smog_radius = 2.6
        self.submissions: Dict[str, Dict[str, Dict[str, Any]]] = {}   # team_slot → kind → payload
        self.assign: Dict[str, Dict[str, Any]] = {}                   # ugv.id → 명령 진실값
        self.layout_failures = []
        self.layout_meta: Dict[str, Any] = {}
        if scenario == "maicon_fixed":
            self._build_fixed_layout(layout_seed if layout_seed is not None else seed, fixed_m3_count)
        elif scenario == "random":
            self._build_valid_layout(layout_seed if layout_seed is not None else seed)
        else:
            raise ValueError(f"Unknown arena scenario: {scenario}")

    # ================================================================== 이벤트 장치(환경효과·드론)
    def set_smog(self, on: bool, announce: bool = True) -> None:
        """스모그 장치 ON/OFF. announce 면 서버가 모든 팀에 이벤트로 알린다(실제 API 가 알려 주는지는 모른다)."""
        self.smog = bool(on)
        if announce:
            self.events.append({"t": self.t, "type": "smog", "on": self.smog, "broadcast": True})

    def smog_strength(self, u: "SimUGV") -> float:
        """로봇 위치의 연기 농도(0~1). 분출구에서 멀수록 옅다."""
        if not self.smog:
            return 0.0
        d = math.hypot(u.pose.x - self.smog_center[0], u.pose.y - self.smog_center[1])
        return max(0.0, 1.0 - d / self.smog_radius)

    def set_uav_period(self, period: float) -> None:
        """드론 영상 제공 주기(s). 상대의 지정표적 유효타격 1개→2, 2개→5, 3개→중단(math.inf)."""
        self.uav_period = period

    def uav_frame_interval(self, team: str) -> Optional[float]:
        """Example server policy: opposing target hits slow this team's feed."""
        if not self.uav_enabled or not math.isfinite(self.uav_period):
            return None
        own_side = "B" if team == "BLUE" else "R"
        hits = sum(ev.get("type") == "target_hit" and
                   str(ev.get("by", "")).startswith("R" if own_side == "B" else "B")
                   for ev in self.events)
        if hits >= 3:
            return None
        return max(self.uav_period, (1.0, 2.0, 5.0)[hits])

    # ================================================================== 배치
    def _build_fixed_layout(self, seed: int = 0, m3_count: int = 4) -> None:
        """Keep photographed infrastructure, add seeded PDF-sized M3 cubes."""
        from .fixed_maicon import BASE_TYPES, STRUCTURES, generate_m3_structures

        if abs(self.geo.W - 6.12) > .01 or abs(self.geo.H - 5.0) > .01:
            raise ValueError("The fixed MAICON layout requires a 6.12 x 5.00 m arena")
        self.cubes = []
        m3 = generate_m3_structures(seed, self.geo, m3_count)
        structures = (*STRUCTURES, *m3)
        for structure in structures:
            cube = Cube(structure.x, structure.y, sx=structure.sx, sy=structure.sy)
            cube.items = {face: Item(kind, cls) for face, kind, cls in structure.faces}
            self.cubes.append(cube)
        self.base_types = dict(BASE_TYPES)
        self.layout_meta = {"used_layout_seed": seed, "scenario": "maicon_fixed",
                            "validated": True, "validation_scope": "new_m3_clearance_and_static_routes",
                            "fixed_m3_count": len(m3), "m3_face_count": len(m3)*4,
                            "structure_names": [s.name for s in structures]}

    def _build_valid_layout(self, seed0: int) -> None:
        """무작위 배치를 만들고 정적 유효성(sim/arena_check.py)을 검사한다. 통과 못 하면 *결정적인* 다음 시드로 다시 만든다(상한 arena.max_layout_tries).
        같은 seed 는 항상 같은 시도 순서·결과를 낸다. 원래 seed·시도 횟수·제외 사유는 self.layout_meta 에 남는다.
        끝내 못 만들면 ArenaGenerationError(제외 사유 포함) — 조용히 건너뛰지 않는다."""
        from .arena_check import ArenaGenerationError, check_arena
        geo = self.cfg.arena
        meta: Dict[str, Any] = {"seed": seed0, "attempts": 0, "used_layout_seed": seed0, "excluded": [], "validated": bool(geo.validate_layout)}
        tries = max(int(geo.max_layout_tries), 1) if geo.validate_layout else 1
        for k in range(tries):
            s = seed0 + 10007 * k
            self._build_layout(s)
            meta["attempts"] = k + 1
            meta["used_layout_seed"] = s
            if not geo.validate_layout:
                break
            rep = check_arena(self, self.cfg, full=False)    # 값싼 사유(배치 실패·내용물)가 있으면 경로 계획은 건너뛴다
            if rep.ok:
                break
            meta["excluded"].append({"attempt": k, "layout_seed": s, "reasons": rep.reasons})
        else:
            self.layout_meta = meta
            raise ArenaGenerationError(f"seed={seed0}: {tries}회 시도에도 유효한 경기장을 만들지 못했다. 마지막 사유: {meta['excluded'][-1]['reasons'][:4]}", meta)
        self.layout_meta = meta

    def _build_layout(self, seed: int) -> None:
        self.cubes = []
        self.base_types = {}
        self.layout_failures: List[str] = []
        rng = random.Random(seed)
        g = self.geo
        obj_cls = list(self.cfg.perception.object_classes)
        tgt_cls = list(self.cfg.perception.target_classes)
        placed: List[Tuple[float, float]] = []

        def ok(x, y):
            if not (g.base_depth + 0.4 <= x <= g.W - g.base_depth - 0.4):
                return False
            if not (0.6 <= y <= g.H - 0.6):
                return False
            return all(max(abs(x - px), abs(y - py)) >= 0.8 for px, py in placed)

        def place_in(sector: str) -> Optional[Tuple[float, float]]:
            x0, y0, x1, y1 = g.sector_rect(sector)
            for _ in range(600):
                x, y = rng.uniform(x0 + 0.1, x1 - 0.1), rng.uniform(y0 + 0.2, y1 - 0.2)
                x, y = round(x / 0.2) * 0.2, round(y / 0.2) * 0.2
                if ok(x, y):
                    return x, y
            return None

        for side in ("B", "R"):
            kinds = ["obj", "obj", "obj", "tgt", "tgt", "tgt"]
            rng.shuffle(kinds)
            tgts = list(tgt_cls)
            rng.shuffle(tgts)
            for i, sec in enumerate([f"{side}-S{k}" for k in (1, 2, 3)] * 2):
                p = place_in(sec)
                if p is None:
                    self.layout_failures.append(f"{side}:{sec}#{i}")           # 놓지 못했다 — 유효성 검사가 이 배치를 제외한다
                    continue
                placed.append(p)
                cube = Cube(*p)
                if kinds[i] == "obj":
                    for f in range(4):
                        cube.items[f] = Item("object", rng.choice(obj_cls))
                else:
                    cube.items[rng.randrange(4)] = Item("target", tgts.pop() if tgts else rng.choice(tgt_cls))
                self.cubes.append(cube)
        for _ in range(2):
            p = place_in("COMMON")
            if p:
                placed.append(p)
                c = Cube(*p)
                for f in range(4):
                    c.items[f] = Item("object", rng.choice(obj_cls))
                self.cubes.append(c)
        # 거점 표지: 벽 쪽 면 → 시뮬에서는 '거점 중앙 뒤 벽'에 붙은 가상 표지로 처리
        types = ["COMMAND_POST", "MAINT_DEPOT", "SUPPLY_BASE"]
        for side in ("B", "R"):
            t = types[:]
            rng.shuffle(t)
            for i in (1, 2, 3):
                self.base_types[f"{side}-{i}"] = t[i - 1]

    def base_of_type(self, side: str, btype: str) -> str:
        for k, v in self.base_types.items():
            if k.startswith("B" if side.upper() == "BLUE" else "R") and v == btype:
                return k
        raise KeyError(btype)

    # ================================================================== 로봇 추가
    def add_ugv(self, team: str, slot: int, cfg: Optional[Config] = None, face_center: bool = True) -> SimUGV:
        base = f"{'B' if team == 'BLUE' else 'R'}-{slot}"
        cx, cy = self.geo.base_center(base)
        cx += 0.15 if team == "BLUE" else -0.15
        th = 0.0 if team == "BLUE" else math.pi
        th += self.rng.uniform(-0.05, 0.05)
        u = SimUGV(self, f"{team[0]}-UGV-{slot}", team, slot, Pose2D(cx, cy, th), cfg or self.cfg,
                   random.Random(self.rng.random()))
        self.ugvs.append(u)
        return u

    # ================================================================== 구조물 실시간 변경(동적 경기장 시험)
    def move_cube(self, idx: int, x: float, y: float) -> None:
        """박스 하나를 (x, y)로 옮긴다(면의 객체/표적도 같이 이동). 경기 중에 호출해도 된다."""
        c = self.cubes[idx]
        c.cx, c.cy = x, y

    def add_blank_cube(self, x: float, y: float) -> Cube:
        """객체/표적이 없는 비인식용 박스를 새로 놓는다(비인식용 장애물 모사)."""
        c = Cube(x, y)
        self.cubes.append(c)
        return c

    def remove_cube(self, c: Cube) -> None:
        self.cubes.remove(c)

    # ================================================================== 장애물/충돌
    def obstacles(self) -> List[Tuple[float, float, float, float]]:
        """로봇이 부딪히는 바닥 위 사각형들(박스 + 낮은 물체)."""
        return [c.rect() for c in self.cubes] + [lo[:4] for lo in self.low_obstacles if lo[5] < 0.2]      # 공중에 뜬 것(z0 ≥ 0.2)은 부딪히지 않는다

    def add_low_obstacle(self, x0: float, y0: float, x1: float, y1: float, height: float = 0.12, z0: float = 0.0) -> None:
        """LiDAR 평면(약 0.24 m)에 안 걸리는 물체. 기본은 바닥의 낮은 블록(높이 height), z0 > 0 이면 공중에 뜬 선반(z0 ~ height).
        LiDAR·RGB 검출에는 안 보이고 깊이 카메라에만 보인다. 규정의 구조물이 아닌 3D 지도 시험용이다."""
        self.low_obstacles.append((min(x0, x1), min(y0, y1), max(x0, x1), max(y0, y1), float(height), float(z0)))

    def _fp(self) -> Footprint:
        rb = self.cfg.robot
        return Footprint(rb.length, rb.width, rb.height)

    def ugv_corners(self, u: SimUGV) -> np.ndarray:
        """UGV 의 직사각형 외곽(250 × 230 mm, PDF p21) 네 모서리."""
        return self._fp().corners(u.pose.x, u.pose.y, u.pose.th)

    def _resolve_rect(self, u: SimUGV) -> None:
        """직사각형 외곽 충돌: 외벽·박스(AABB)·다른 UGV(직사각형)와 SAT 로 겹침을 풀어낸다. 모서리가 닿는 것도 충돌로 센다."""
        g = self.geo
        fp = self._fp()
        p = u.pose
        moved = 0.0
        obs = [(o, aabb_corners(*o)) for o in self.obstacles()]
        for _ in range(4):                                     # 여러 접촉이 서로 밀어내는 경우를 몇 번 반복해 푼다
            cs = fp.corners(p.x, p.y, p.th)
            dx = max(0.0 - cs[:, 0].min(), 0.0) - max(cs[:, 0].max() - g.W, 0.0)
            dy = max(0.0 - cs[:, 1].min(), 0.0) - max(cs[:, 1].max() - g.H, 0.0)
            if dx or dy:
                p.x, p.y = p.x + dx, p.y + dy
                moved += math.hypot(dx, dy)
                cs = fp.corners(p.x, p.y, p.th)
            hit = False
            for bounds, ob in obs:
                # Broad phase must include each obstacle's complete extent.
                # A fixed centre-distance cutoff made the ends of long walls
                # nonphysical. The current rotated footprint is updated after
                # each push, so neighbouring contacts remain conservative.
                x0, y0, x1, y1 = bounds
                if (cs[:, 0].max() < x0 or cs[:, 0].min() > x1 or
                        cs[:, 1].max() < y0 or cs[:, 1].min() > y1):
                    continue
                push = sat_push(cs, ob)
                if push is not None:
                    p.x, p.y = p.x + push[0], p.y + push[1]
                    moved += float(np.hypot(*push))
                    cs = fp.corners(p.x, p.y, p.th)
                    hit = True
            for o in self.ugvs:
                if o is u or math.hypot(p.x - o.pose.x, p.y - o.pose.y) > 0.4:
                    continue
                push = sat_push(cs, self.ugv_corners(o))
                if push is not None:
                    p.x, p.y = p.x + 0.5 * push[0], p.y + 0.5 * push[1]       # 상대는 자기 스텝에서 처리
                    moved += 0.5 * float(np.hypot(*push))
                    cs = fp.corners(p.x, p.y, p.th)
                    hit = True
            if not hit:
                break
        if moved > 0.004:
            u.collisions += 1
            u.v *= 0.2

    def resolve_collisions(self, u: SimUGV) -> None:
        if self.cfg.robot.collision_model != "circle":
            self._resolve_rect(u)
            return
        g, r = self.geo, self.cfg.robot.radius                # 레거시 원 모델(변경 전 비교용)
        p = u.pose
        moved = 0.0
        # 외벽
        nx, ny = clamp(p.x, r, g.W - r), clamp(p.y, r, g.H - r)
        moved += math.hypot(nx - p.x, ny - p.y)
        p.x, p.y = nx, ny
        # 박스
        for (x0, y0, x1, y1) in self.obstacles():
            qx, qy = clamp(p.x, x0, x1), clamp(p.y, y0, y1)
            d = math.hypot(p.x - qx, p.y - qy)
            if d < r:
                if d < 1e-9:   # 박스 안으로 파고든 경우: 가장 가까운 변으로 밀어냄
                    cands = [(p.x - x0, -1, 0), (x1 - p.x, 1, 0), (p.y - y0, 0, -1), (y1 - p.y, 0, 1)]
                    _, ax, ay = min(cands)
                    p.x, p.y = qx + ax * r, qy + ay * r
                else:
                    p.x, p.y = qx + (p.x - qx) / d * r, qy + (p.y - qy) / d * r
                moved += r - d
        # 다른 로봇
        for o in self.ugvs:
            if o is u:
                continue
            d = math.hypot(p.x - o.pose.x, p.y - o.pose.y)
            if d < 2 * UGV_R:
                if d < 1e-9:
                    p.x += 1e-3
                    continue
                push = (2 * UGV_R - d)
                p.x += (p.x - o.pose.x) / d * push * 0.5
                p.y += (p.y - o.pose.y) / d * push * 0.5
                o.pose.x -= (p.x - o.pose.x) / d * push * 0.0  # 상대는 자기 스텝에서 처리
                moved += push * 0.5
        if moved > 0.004:
            u.collisions += 1
            u.v *= 0.2

    def _blocked(self, ax, ay, bx, by, ignore_cube: Optional[Cube] = None, ignore_ugv=(),
                 other_ugv_blocks: bool = True) -> bool:
        """선분 (a→b)가 박스(또는 로봇)에 가로막히는가."""
        dx, dy = bx - ax, by - ay
        L = math.hypot(dx, dy)
        if L < 1e-6:
            return False
        dx, dy = dx / L, dy / L
        for c in self.cubes:
            if c is ignore_cube:
                continue
            h = _ray_aabb(ax, ay, dx, dy, c.rect(), L)
            if h is not None and h[0] < L:
                return True
        if other_ugv_blocks:
            ign = ignore_ugv if isinstance(ignore_ugv, (list, tuple, set)) else (ignore_ugv,)
            for o in self.ugvs:
                if any(o is g for g in ign):
                    continue
                # 점-선분 거리
                t = clamp((o.pose.x - ax) * dx + (o.pose.y - ay) * dy, 0, L)
                px, py = ax + dx * t, ay + dy * t
                if math.hypot(o.pose.x - px, o.pose.y - py) < UGV_R * 0.9 and t < L - 0.05:
                    return True
        return False

    # ================================================================== LiDAR
    def lidar(self, u: SimUGV, n: int = 360, rmax: float = 8.0) -> Scan:
        dr = u.drive
        le = self.cfg.lidar_err
        ang = np.linspace(-math.pi, math.pi, n, endpoint=False) + u.pose.th + dr.lidar_yaw   # 스캔 각도는 LiDAR 프레임 기준
        if le.sim_angle_sigma_deg > 0:                                  # 빔 각도 오차(기하에 그대로 반영)
            arng = np.random.default_rng(int(self.t * 1000) % 2 ** 31 + 4243 + zlib.crc32(u.id.encode()) % 1000)
            ang = ang + arng.normal(0, math.radians(le.sim_angle_sigma_deg), n)
        c0, s0 = math.cos(u.pose.th), math.sin(u.pose.th)
        ox = u.pose.x + c0 * dr.lidar_xy[0] - s0 * dr.lidar_xy[1]
        oy = u.pose.y + s0 * dr.lidar_xy[0] + c0 * dr.lidar_xy[1]
        dx, dy = np.cos(ang), np.sin(ang)
        best = np.full(n, rmax)
        # 외벽(내부에서): 각 축 방향 교차
        with np.errstate(divide="ignore", invalid="ignore"):
            tx = np.where(dx > 0, (self.geo.W - ox) / dx, np.where(dx < 0, (0 - ox) / dx, np.inf))
            ty = np.where(dy > 0, (self.geo.H - oy) / dy, np.where(dy < 0, (0 - oy) / dy, np.inf))
        best = np.minimum(best, np.minimum(tx, ty))
        # 박스(슬랩)
        for c in self.cubes:
            x0, y0, x1, y1 = c.rect()
            with np.errstate(divide="ignore", invalid="ignore"):
                t1x, t2x = (x0 - ox) / dx, (x1 - ox) / dx
                t1y, t2y = (y0 - oy) / dy, (y1 - oy) / dy
            tmin = np.maximum(np.minimum(t1x, t2x), np.minimum(t1y, t2y))
            tmax = np.minimum(np.maximum(t1x, t2x), np.maximum(t1y, t2y))
            hit = (tmax >= np.maximum(tmin, 0)) & (tmin > 0)
            best = np.where(hit, np.minimum(best, tmin), best)
        # 다른 로봇: 직사각형 외곽(250 × 230 mm). 레거시 모드(collision_model="circle")에서는 예전 원(UGV_R)
        for o in self.ugvs:
            if o is u:
                continue
            if self.cfg.robot.collision_model == "circle":
                fx, fy = ox - o.pose.x, oy - o.pose.y
                b = fx * dx + fy * dy
                cc = fx * fx + fy * fy - UGV_R ** 2
                disc = b * b - cc
                with np.errstate(invalid="ignore"):
                    t = -b - np.sqrt(np.maximum(disc, 0))
                hit = (disc >= 0) & (t > 0)
                best = np.where(hit, np.minimum(best, t), best)
                continue
            if math.hypot(o.pose.x - ox, o.pose.y - oy) > rmax + 0.3:
                continue
            fp = self._fp()
            co, so = math.cos(o.pose.th), math.sin(o.pose.th)
            lx0 = (ox - o.pose.x) * co + (oy - o.pose.y) * so              # 광선 원점을 상대 차체 좌표계로
            ly0 = -(ox - o.pose.x) * so + (oy - o.pose.y) * co
            ldx = dx * co + dy * so
            ldy = -dx * so + dy * co
            with np.errstate(divide="ignore", invalid="ignore"):
                t1x, t2x = (-fp.half_l - lx0) / ldx, (fp.half_l - lx0) / ldx
                t1y, t2y = (-fp.half_w - ly0) / ldy, (fp.half_w - ly0) / ldy
            tmin = np.maximum(np.minimum(t1x, t2x), np.minimum(t1y, t2y))
            tmax = np.minimum(np.maximum(t1x, t2x), np.maximum(t1y, t2y))
            hit = (tmax >= np.maximum(tmin, 0)) & (tmin > 0)
            best = np.where(hit, np.minimum(best, tmin), best)
        rng = np.random.default_rng(int(self.t * 1000) % 2 ** 31 + zlib.crc32(u.id.encode()) % 1000)
        r = best + le.sim_range_bias_m + rng.normal(0, 1.0, n) * (le.sim_range_sigma_m + le.sim_range_sigma_per_m * best)
        # Missing measurements are unknown, not positive-infinity no-return
        # rays: the latter legitimately clear free space up to sensor range.
        dropout = rng.random(n) < le.sim_dropout_p
        if le.sim_outlier_p > 0:                                      # 이상치: 임의 거리의 가짜 반사
            orng = np.random.default_rng(int(self.t * 1000) % 2 ** 31 + 9173 + zlib.crc32(u.id.encode()) % 1000)
            k = orng.random(n) < le.sim_outlier_p
            r[k] = orng.uniform(0.15, 3.0, int(k.sum()))
        r[best >= rmax - 1e-6] = np.inf
        s = self.smog_strength(u)
        if s > 0:                                            # 연기 입자: 무작위 빔이 0.2~1.0 m 의 고립된 가짜 반사를 낸다
            srng = np.random.default_rng(int(self.t * 1000) % 2 ** 31 + 7717)
            k = srng.random(n) < 0.16 * s
            m = int(k.sum())
            # [시뮬 가정] 센서 바로 앞의 짙은 입자는 8~20 cm 의 반사도 만든다(30 %). 실제 분포는 모른다 — 현장 측정 필요
            r[k] = np.where(srng.random(m) < 0.3, srng.uniform(0.08, 0.2, m), srng.uniform(0.2, 1.0, m))
        r[dropout] = np.nan
        return Scan(angle_min=-math.pi, angle_inc=2 * math.pi / n, ranges=r, stamp=self.t, range_max=rmax)

    # ================================================================== 깊이 카메라
    def depth_image(self, u: SimUGV, dc=None) -> np.ndarray:
        """깊이 카메라가 찍을 z-깊이 영상(m, float32; 0 = 무효). 바닥·외벽·박스·낮은 물체·다른 UGV(원기둥)를 광선으로 맞힌다.
        센서 위치·자세는 차체→팬→틸트→깊이 링크의 기구학(geom3d.RigModel) + 숨은 설치 오차(dc.sim_mount_err_*)다.
        잡음(스테레오: σ = k·z²)·무효 픽셀·가장자리 번짐('날아다니는 점')·연기 속 열화는 [시뮬 가정]이다 — 난수는 전용 생성기를 쓴다."""
        dc = dc or self.cfg.depth
        W, H = dc.width, dc.height
        intr = dc.intrinsics(W, H)
        key = (W, H, round(intr.fx, 3))
        if key not in self._depth_rays:
            vv, uu = np.meshgrid(np.arange(H), np.arange(W), indexing="ij")
            self._depth_rays[key] = intr.rays(uu.astype(float), vv.astype(float)).reshape(-1, 3)
        rays = self._depth_rays[key]
        extra = Rt.from_xyz_rpy(dc.sim_mount_err_xyz, dc.sim_mount_err_rpy)
        T = RigModel(self.cfg.rig).optical_in_world(u.pose.x, u.pose.y, u.pose.th, u.pan, u.tilt, "depth", extra=extra)
        o = T.t
        d = rays @ T.R.T                                          # 월드 방향 (z_opt = 1 로 정규화 → 매개변수 t 가 곧 z-깊이)
        n = len(d)
        best = np.full(n, np.inf)
        dx, dy, dz = d[:, 0], d[:, 1], d[:, 2]
        g = self.geo
        with np.errstate(divide="ignore", invalid="ignore"):
            # 바닥
            tf = np.where(dz < -1e-9, -o[2] / dz, np.inf)
            best = np.minimum(best, np.where(tf > 0, tf, np.inf))
            # 외벽 4면 (높이 wall_h 까지)
            for axis, plane, lo, hi in ((0, 0.0, 0.0, g.H), (0, g.W, 0.0, g.H), (1, 0.0, 0.0, g.W), (1, g.H, 0.0, g.W)):
                da = d[:, axis]
                t = (plane - o[axis]) / da
                other = 1 - axis
                po = o[other] + t * d[:, other]
                pz = o[2] + t * dz
                ok = (t > 0) & (po >= lo - 1e-6) & (po <= hi + 1e-6) & (pz >= 0) & (pz <= g.wall_h)
                best = np.where(ok, np.minimum(best, t), best)
            # 박스(AABB)와 낮은 물체
            boxes = [(c.rect(), 0.0, g.box_size) for c in self.cubes] + \
                    [((lo[0], lo[1], lo[2], lo[3]), lo[5], lo[4]) for lo in self.low_obstacles]
            for (x0, y0, x1, y1), zb, hh in boxes:
                if math.hypot(0.5 * (x0 + x1) - o[0], 0.5 * (y0 + y1) - o[1]) > dc.max_range + 1.0:
                    continue
                t1x, t2x = (x0 - o[0]) / dx, (x1 - o[0]) / dx
                t1y, t2y = (y0 - o[1]) / dy, (y1 - o[1]) / dy
                t1z, t2z = (zb - o[2]) / dz, (hh - o[2]) / dz
                tmin = np.maximum(np.maximum(np.minimum(t1x, t2x), np.minimum(t1y, t2y)), np.minimum(t1z, t2z))
                tmax = np.minimum(np.minimum(np.maximum(t1x, t2x), np.maximum(t1y, t2y)), np.maximum(t1z, t2z))
                ok = (tmax >= tmin) & (tmin > 0)
                best = np.where(ok, np.minimum(best, tmin), best)
            # 다른 UGV: 수직 원기둥(반경 UGV_R, 높이 UGV_H) + 윗면
            for other_u in self.ugvs:
                if other_u is u:
                    continue
                cx, cy = other_u.pose.x, other_u.pose.y
                if math.hypot(cx - o[0], cy - o[1]) > dc.max_range + 1.0:
                    continue
                fx_, fy_ = o[0] - cx, o[1] - cy
                a = dx * dx + dy * dy
                b = fx_ * dx + fy_ * dy
                cq = fx_ * fx_ + fy_ * fy_ - UGV_R ** 2
                disc = b * b - a * cq
                tc = (-b - np.sqrt(np.maximum(disc, 0))) / a
                zc = o[2] + tc * dz
                ok = (disc >= 0) & (tc > 0) & (zc >= 0) & (zc <= UGV_H)
                best = np.where(ok, np.minimum(best, tc), best)
                ttop = (UGV_H - o[2]) / dz
                px, py = o[0] + ttop * dx - cx, o[1] + ttop * dy - cy
                ok = (dz < 0) & (ttop > 0) & (px * px + py * py <= UGV_R ** 2)
                best = np.where(ok, np.minimum(best, ttop), best)
        # ---- 센서 모델 (전용 난수)
        rng = np.random.default_rng((int(self.t * 1000) % 2 ** 31) + zlib.crc32(u.id.encode()) % 997 + 31)
        s = self.smog_strength(u)
        z = best.copy()
        z[~np.isfinite(z) | (z > 8.0)] = 0.0
        sig = dc.sim_noise_k * z * z * (1.0 + 2.0 * s)
        z = z + rng.normal(0.0, 1.0, n) * sig
        drop = rng.random(n) < dc.sim_dropout + 0.5 * s * (z > 1.2)           # 연기: 먼 픽셀부터 스테레오 매칭 실패
        z[drop] = 0.0
        img = z.reshape(H, W)
        if dc.sim_flying_pixels > 0:                                          # 깊이 불연속에서 앞/뒤 사이 값으로 번짐
            right = np.roll(img, -1, axis=1)
            jump = (np.abs(img - right) > 0.30) & (img > 0) & (right > 0)
            jump[:, -1] = False
            ys, xs = np.nonzero(jump & (rng.random(img.shape) < dc.sim_flying_pixels))
            if len(ys):
                k = rng.random(len(ys))
                img[ys, xs] = img[ys, xs] * (1 - k) + right[ys, xs] * k
        return img.astype(np.float32)

    # ================================================================== 카메라
    def camera_detections(self, u: SimUGV) -> List[Detection]:
        cam = self.cfg.camera
        W, H = cam.width, cam.height
        cm = CameraModel(cam)                                    # K(보정값)가 있으면 fx, fy, 주점을 그대로, 없으면 hfov
        f, fy_, cx_, cy_ = cm.f, cm.fy, cm.cx, cm.cy
        psi = u.pose.th + math.radians(u.pan)
        phi = math.radians(u.tilt)
        cx0, cy0, cz0 = u.cam_pos()
        rng = self.rng
        out: List[Detection] = []

        def project(px, py, pz):
            dx, dy, dz = px - cx0, py - cy0, pz - cz0
            fwd = dx * math.cos(psi) + dy * math.sin(psi)
            left = -dx * math.sin(psi) + dy * math.cos(psi)
            xc = fwd * math.cos(phi) + dz * math.sin(phi)
            up = -fwd * math.sin(phi) + dz * math.cos(phi)
            if xc < 0.08:
                return None
            return (cx_ - f * left / xc, cy_ - fy_ * up / xc, xc)

        def bbox_of(points):
            pr = [project(*p) for p in points]
            if any(q is None for q in pr):
                return None
            us = [q[0] for q in pr]
            vs = [q[1] for q in pr]
            x1, x2, y1, y2 = min(us), max(us), min(vs), max(vs)
            area = max((x2 - x1) * (y2 - y1), 1e-6)
            cx1, cx2, cy1, cy2 = max(x1, 0), min(x2, W), max(y1, 0), min(y2, H)
            if cx2 <= cx1 or cy2 <= cy1:
                return None
            if (cx2 - cx1) * (cy2 - cy1) / area < 0.65:
                return None
            return (x1, y1, x2, y2)

        def jitter(box, dist):
            x1, y1, x2, y2 = box
            w, h = x2 - x1, y2 - y1
            s = 0.03 + 0.01 * dist
            return (x1 + rng.gauss(0, s) * w, y1 + rng.gauss(0, s) * h,
                    x2 + rng.gauss(0, s) * w, y2 + rng.gauss(0, s) * h)

        def sensor_px(px, py, pz, radius):
            """수광센서 한 점을 영상에 투영한 (u, v, 반경 px, 신뢰도). 실기에서는 perception/receiver.py 가 영상에서 찾는 값이다."""
            q = project(px, py, pz)
            if q is None:
                return None
            return (q[0] + self.sensor_rng.gauss(0, 0.8), q[1] + self.sensor_rng.gauss(0, 0.8), f * radius / q[2],
                    clamp(0.95 - 0.05 * max(0.0, q[2] - 1.5), 0.4, 0.99))

        classes_o = list(self.cfg.perception.object_classes)
        classes_t = list(self.cfg.perception.target_classes)
        for c in self.cubes:
            for fi, item in c.items.items():
                nx, ny = FACE_N[fi]
                fcx, fcy = c.cx + nx * c.sx / 2, c.cy + ny * c.sy / 2
                if (cx0 - fcx) * nx + (cy0 - fcy) * ny <= 0.02:
                    continue                                   # 뒷면
                dist = math.hypot(fcx - cx0, fcy - cy0)
                if dist > 3.0 or dist < 0.25:
                    continue
                if self._blocked(cx0, cy0, fcx + nx * 0.02, fcy + ny * 0.02, ignore_ugv=(u,)):
                    continue
                tx, ty = -ny, nx
                if item.kind == "target" and self.cfg.perception.sim_target_box == "object":
                    hw, zs = 0.16, (0.03, 0.29)                  # 시설 물체만 덮는다: 위쪽 하늘의 수광 원판(z 0.30~0.37)은 bbox 밖
                else:
                    hw, zs = (c.sy / 2 if fi % 2 == 0 else c.sx / 2), (0.0, 0.4)
                pts = [(fcx + s * tx * hw, fcy + s * ty * hw, z) for s in (-1, 1) for z in zs]
                box = bbox_of(pts)
                if box is None:
                    continue
                size = max(box[2] - box[0], box[3] - box[1])
                if size < 14 or rng.random() < 0.04 + 0.05 * max(0, dist - 1.5):
                    continue
                conf = clamp(0.97 - 0.10 * max(0, dist - 0.7) + rng.gauss(0, 0.03), 0.05, 0.99)
                cls = item.cls
                if rng.random() < 0.02 + 0.03 * max(0, dist - 1.5):
                    cls = rng.choice(classes_o if item.kind == "object" else classes_t)
                    conf *= 0.7
                ex = {"kind": item.kind, "led_on": item.led_on, "_gt": (c.cx, c.cy, fi)}
                if item.kind == "target":                       # 표적 면의 수광창 중심(흰 부분의 원)
                    sp = sensor_px(fcx + nx * 0.01, fcy + ny * 0.01, 0.5 * sum(TARGET_WINDOW_Z), TARGET_WINDOW_LAT)
                    if sp is not None:
                        ex["sensors"] = [sp]
                out.append(Detection(cls=cls, conf=conf, box=jitter(box, dist), stamp=self.t, extra=ex))
        # 거점 표지: 거점 벽쪽 면의 판 (0.4x0.3)
        for bname, btype in self.base_types.items():
            x0, y0, x1, y1 = self.geo.base_rect(bname)
            cy_b = 0.5 * (y0 + y1)
            wall_x = 0.0 if bname.startswith("B") else self.geo.W
            nx = 1.0 if bname.startswith("B") else -1.0
            if (cx0 - wall_x) * nx <= 0.05:
                continue
            dist = math.hypot(wall_x - cx0, cy_b - cy0)
            if dist > 3.2 or self._blocked(cx0, cy0, wall_x + nx * 0.03, cy_b, ignore_ugv=(u,)):
                continue
            pts = [(wall_x, cy_b + s * 0.2, z) for s in (-1, 1) for z in (0.05, 0.35)]
            box = bbox_of(pts)
            if box is None or max(box[2] - box[0], box[3] - box[1]) < 14 or rng.random() < 0.1:
                continue
            out.append(Detection(cls=btype, conf=clamp(0.95 - 0.08 * max(0, dist - 1.0), 0.3, 0.99),
                                 box=jitter(box, dist), stamp=self.t,
                                 extra={"kind": "base", "_gt": bname}))
        # 다른 UGV
        for o in self.ugvs:
            if o is u:
                continue
            dist = math.hypot(o.pose.x - cx0, o.pose.y - cy0)
            if dist > self.ugv_det_range or dist < 0.2:
                continue
            if self._blocked(cx0, cy0, o.pose.x, o.pose.y, ignore_ugv=(o, u)):
                continue
            lx, ly = -math.sin(psi), math.cos(psi)
            pts = [(o.pose.x + s * lx * UGV_R, o.pose.y + s * ly * UGV_R, z) for s in (-1, 1) for z in (0.0, UGV_H)]
            box = bbox_of(pts)
            if box is None or box[3] - box[1] < 12 or rng.random() < self.ugv_miss_base + self.ugv_miss_slope * max(0, dist - 2.0):
                continue
            team = o.team if rng.random() >= self.team_flip else ("RED" if o.team == "BLUE" else "BLUE")
            conf = clamp(0.95 - 0.08 * max(0, dist - 1.2) + rng.gauss(0, 0.03), 0.1, 0.99)
            ex = {"kind": "ugv", "_gt": o.id}
            # E8 UGV 뒤쪽 LED(피격 횟수 색 + 8칸 탄창): 뒤쪽에서(±55°) 가까이(1.8 m) 볼 때만 읽힌다. 탄창은 가끔 한 칸 오독.
            if dist <= self.cfg.event.ugv_led_range_m and \
                    abs(wrap(math.atan2(cy0 - o.pose.y, cx0 - o.pose.x) - (o.pose.th + math.pi))) < math.radians(55):
                am = o.ammo + (self.sensor_rng.choice((-1, 1)) if self.sensor_rng.random() < 0.08 else 0)
                ex["led"] = {"hits": float(min(o.hits_taken, 4)), "ammo": float(max(0, min(am, 8))), "conf": 0.9}
            # 수광센서는 UGV 전방 반구에서만 보인다(뒤에서는 차체가 가린다). 보이는 것만 좌→우 순으로.
            if abs(wrap(math.atan2(cy0 - o.pose.y, cx0 - o.pose.x) - o.pose.th)) < math.radians(105):
                sens = [sensor_px(sx, sy, 0.5 * sum(RECEIVER_Z), RECEIVER_RADIUS) for sx, sy in o.receivers()]
                sens = sorted([s for s in sens if s is not None], key=lambda s: s[0])
                if sens:
                    ex["sensors"] = sens
            out.append(Detection(cls="UGV", conf=conf, box=jitter(box, dist), stamp=self.t, team=team, extra=ex))
        s = self.smog_strength(u)
        if s > 0:                                            # 연기 속 카메라: 검출이 빠지고 신뢰도가 낮아진다(전용 난수로 — 평시 난수열은 그대로)
            keep = []
            for d in out:
                if self.sensor_rng.random() < 0.35 * s:
                    continue
                d.conf *= (1.0 - 0.35 * s)
                keep.append(d)
            out = keep
        return out

    # ================================================================== 레이저
    def _laser_origin(self, u: SimUGV):
        """레이저 발광부 위치 (x, y, 높이). 카메라(차체 중심·cam_height)에서 옆·위로 떨어져 있을 수 있다(시차의 원인)."""
        psi = u.pose.th + math.radians(u.pan)
        lat = self.cfg.laser.sim_origin_lateral_m
        return (u.pose.x - math.sin(psi) * lat, u.pose.y + math.cos(psi) * lat,
                self.cfg.robot.cam_height + self.cfg.laser.sim_origin_vertical_m)

    def _beam_hit(self, u: SimUGV):
        """레이저 한 줄기가 맞는 첫 대상. (kind, obj, key, t) 또는 None."""
        psi = u.pose.th + math.radians(u.pan)
        dx, dy = math.cos(psi), math.sin(psi)
        ox, oy, h0 = self._laser_origin(u)
        tan_t = math.tan(math.radians(u.tilt))
        L = self.cfg.laser.max_range
        best = (L, None)
        for c in self.cubes:
            h = _ray_aabb(ox, oy, dx, dy, c.rect(), L)
            if h is not None and h[0] < best[0]:
                best = (h[0], ("cube", c, h[1]))
        for o in self.ugvs:
            if o is u:
                continue
            # 수광센서(수직 원기둥) 우선 검사
            for si, (sx, sy) in enumerate(o.receivers()):
                fx, fy = sx - ox, sy - oy
                tproj = fx * dx + fy * dy
                if tproj <= 0 or tproj > L:
                    continue
                perp = abs(fx * dy - fy * dx)
                z = h0 + tproj * tan_t
                if perp <= RECEIVER_RADIUS and RECEIVER_Z[0] <= z <= RECEIVER_Z[1]:
                    # 수광센서는 전방 반구에서만 유효(뒤에서 쏘면 차체가 가림)
                    to_shooter = math.atan2(oy - o.pose.y, ox - o.pose.x)
                    if abs(wrap(to_shooter - o.pose.th)) < math.radians(105) and tproj < best[0]:
                        best = (tproj, ("ugv", o, si))
            # 차체(원)에 가려짐
            if self.cfg.robot.collision_model != "circle":
                t = ray_rect(ox, oy, dx, dy, self.ugv_corners(o), L)
                if t is not None and 0 < t < best[0] - 0.12:
                    best = (t, ("block", o, 0))
                continue
            fx, fy = ox - o.pose.x, oy - o.pose.y
            b = fx * dx + fy * dy
            cc = fx * fx + fy * fy - UGV_R ** 2
            disc = b * b - cc
            if disc >= 0:
                t = -b - math.sqrt(disc)
                if 0 < t < best[0] - 0.12:
                    best = (t, ("block", o, 0))
        return best

    def _laser_step(self, u: SimUGV, dt: float) -> None:
        if not u.laser_on:
            u.contact.clear()
            return
        t_hit, what = self._beam_hit(u)
        key = None
        if what is not None:
            kind, obj, extra = what
            if kind == "cube":
                item = obj.items.get(extra)
                if item is not None and item.kind == "target":
                    # 수광창: 면 중심 기준 좌우/높이
                    psi = u.pose.th + math.radians(u.pan)
                    lx, ly, lz = self._laser_origin(u)
                    hx, hy = lx + math.cos(psi) * t_hit, ly + math.sin(psi) * t_hit
                    nx, ny = FACE_N[extra]
                    lat = (hx - obj.cx) * (-ny) + (hy - obj.cy) * nx
                    z = lz + t_hit * math.tan(math.radians(u.tilt))
                    if abs(lat) <= TARGET_WINDOW_LAT and TARGET_WINDOW_Z[0] <= z <= TARGET_WINDOW_Z[1]:
                        key = ("item", id(item))
                        self._contact(u, key, dt, ("item", obj, extra))
            elif kind == "ugv":
                key = ("ugv", obj.id)
                self._contact(u, key, dt, ("ugv", obj, extra))
        # S6.3/S8.3 require continuous illumination. A missed physics sample
        # ends the contact; no invented 0.3-second grace period joins bursts.
        for k in list(u.contact):
            if k != key:
                u.contact.pop(k)

    def _contact(self, u: SimUGV, key, dt: float, info) -> None:
        acc, last = u.contact.get(key, (0.0, self.t))
        if self.t - last > dt + 1e-9:
            acc = 0.0
        acc += dt
        u.contact[key] = (acc, self.t)
        if acc + 1e-9 < self.cfg.laser.dwell_s:
            return
        u.contact[key] = (0.0, self.t)
        kind, obj, extra = info
        if kind == "item":
            item = obj.items[extra]
            if not item.led_on:
                item.led_on = True
                item.hit_by = u.id
                u.targets_hit.append(item.cls)
                # Local referee evidence includes the struck camp, not a private
                # object id or pose. A same-class friendly target is not M2.
                camp = "RED" if obj.cx > self.geo.W/2 else "BLUE"
                self.events.append({"t": self.t, "type": "target_hit", "by": u.id,
                                    "cls": item.cls, "camp": camp})
        else:
            v: SimUGV = obj
            if v.invuln_left > 0:
                return
            v.hits_taken += 1
            v.invuln_left = self.cfg.laser.disabled_s
            v.disabled_left = self.cfg.laser.disabled_s
            v.abort()
            if v.hits_taken % 4 == 0:
                v.paused_left = self.cfg.laser.disabled_s
            if v.team == u.team:
                u.friendly_hits += 1
            else:
                u.hits_dealt.append(v.id)
            self.events.append({"t": self.t, "type": "ugv_hit", "by": u.id, "victim": v.id,
                                "friendly": v.team == u.team})

    # ================================================================== 진행
    def step(self, dt: float) -> None:
        for u in self.ugvs:
            u.step(dt)
        for u in self.ugvs:
            self._laser_step(u, dt)
        self._base_sensor_step()
        if not self.ended and self.t >= self.cfg.mission.t_total:
            self.end()
        self.t += dt
        if self.ended and self.end_time is not None and self.t >= self.end_time + self.cfg.mission.t_report:
            self.finished = True

    def _base_sensor_step(self) -> None:
        """Referee-only M5 sensor proxy; occupant identity cannot change credit.

        The simulator uses the existing base interior as its sensor footprint.
        Real sensor placement and any future dwell requirement need field
        confirmation; the provisional dwell defaults to zero, not a guessed
        physical duration. Same-tick events use simulated receipt/list order.
        """
        if self.ended or self.t < self.cfg.mission.t_base_cmd:
            return
        goals = {camp: self.base_of_type("RED" if camp == "BLUE" else "BLUE", kind)
                 for camp, kind in self.goal_base_types.items()}
        for u in self.ugvs:
            if u.goal_base:
                goals.setdefault(u.team, u.goal_base)
        active = set()
        dwell = max(0.0, getattr(self.cfg.mission, "m5_dwell_s", 0.0))
        for occupant in self.ugvs:
            for beneficiary, base in goals.items():
                if not self.geo.in_base(occupant.pose.x, occupant.pose.y, base, margin=-0.05):
                    continue
                key = (base, occupant.id)
                active.add(key)
                since = self._base_contacts.setdefault(key, self.t)
                if self.t - since + 1e-9 < dwell:
                    continue
                if occupant.reached_base is not None:
                    continue
                occupant.reached_base = self.t
                if self.m5_winner is None:
                    self.m5_winner = beneficiary
                self.events.append({"t": self.t, "type": "base_reached", "by": occupant.id, "base": base,
                                    "camp": beneficiary, "broadcast": True})
                if self.m5_ends_match:
                    self.end()
                    return
        self._base_contacts = {key: stamp for key, stamp in self._base_contacts.items() if key in active}

    def end(self) -> None:
        if not self.ended:
            self.ended = True
            self.end_time = self.t
            self.events.append({"t": self.t, "type": "match_end"})
            for u in self.ugvs:
                u.cmd_v = u.cmd_w = 0.0
                u.abort()

    # ================================================================== 지표
    def objects_truth(self, ) -> Dict[str, Dict[str, int]]:
        out: Dict[str, Dict[str, int]] = {}
        for c in self.cubes:
            sec = self.geo.sector_of(c.cx, c.cy)
            for it in c.items.values():
                if it.kind == "object":
                    out.setdefault(sec, {}).setdefault(it.cls, 0)
                    out[sec][it.cls] += 1
        return out

