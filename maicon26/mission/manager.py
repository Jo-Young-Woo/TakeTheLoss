"""임무 관리자(Agent): 센서 → 상태 추정 → 모드 결정 → 구동·조준·발사 → 서버 제출.

상태 전이표(MAICON2026_상태전이표_경쟁상태반영.html)의 구현이다. 한 틱의 처리 순서:
  1. 서버 메시지 수집(START/STOP 래치, match_id·순번 검증)   ← 센서·SLAM 처리보다 먼저 (R01, 순위 00)
  2. 제출 결과 수집·재시도 / 로컬 만료(시작 후 t_total) 확인
  3. 센서 처리(SLAM·검출·UAV)
  4. 순위대로 전이 평가 → 구동·팬틸트·레이저 출력은 한 틱에 한 번

상태(우선순위 순)
  HALT / REPORT / DONE  STOP 래치 뒤: 구동 0·목표 취소·소등 → M6 제출(ACK 또는 30 s) → 종료   [순위 00~02]
  SAFE_STOP  LiDAR/자세 노후화·주행 잠금·RECOVER 소진: 구동 0, 새 센서 표본 뒤에만 복귀      [순위 04·05]
  GOAL       목표거점 명령(4:00) 수신 후: 거점 종류 해석 → 거점 식별 → 진입                  [M5]
  STAGE      prestage_at 이후: 상대 진영 중앙으로 사전 이동                                  [M5 대비]
  RECOVER    항법이 recover_after_s 넘게 막히면: 제한된 후진·회전 뒤 재계획                   [순위 13·14]
  STRIKE     지정표적 접근(nav)·조준(aim)·발사. 세부 상태는 state_name (STRIKE_APPROACH/AIM)  [M2]
  HUNT       상대 UGV 교전 — 기본 OFF(mission.engage_ugv)                                    [M4]
  EXPLORE    시점 순회 + 360° 회전 관측으로 객체/표적/적 탐색                                [M3 + 탐색]
  WAIT       START 전.
피격은 두 잠금값으로 분리: weapon_locked(발사·틸트), drive_locked(주행; 4회째부터).
제출: M1(명령 해석 직후), M3(초기·주기적 전체 스냅샷, 마감 전 종료), M6(STOP 뒤). ACK 없는 제출을 완료로 기록하지 않는다.

모든 시각은 robot.now() 기준(시뮬/실기 공통). 블로킹 호출이 없다 — 한 틱(기본 20 Hz)에 끝난다.
"""
from __future__ import annotations

import dataclasses
import json
import math
import os
import threading
import time
from collections import deque
from enum import Enum
from typing import Any, Dict, List, Optional, Tuple

import numpy as np

from ..arena import ArenaGeometry
from ..config import Config
from ..hal.base import Robot
from ..hal.clock import StampSync
from ..hal.diffdrive import DiffDrive
from ..link import schema
from ..link.base import ServerLink, ServerMsg
from ..link.schema import normalize_event
from ..link.secrets import REDACT
from ..nav.factory import make_stack
from ..nav.selfmask import SelfMask
from ..nav.stall import StallDetector
from ..nav.stack import NavStack
from ..perception.camera import CameraModel, estimate_range
from ..perception.coverage import CoverageMap
from ..perception.depth_fusion import DepthFusion
from ..perception.detector import Detector, SimDetector
from ..perception.lidar_blobs import BlobDetector
from ..perception.threat_board import ThreatBoard
from ..perception.object_map import ObjectMap
from ..perception.tracks import Track, UgvTracker
from ..types import Command, Detection, Frame, Pose2D, Scan
from ..util import clamp, wrap
from . import threat as threat_mod
from ..nav import risk_astar
from .aim import AimController, LaserManager
from .avoid import AvoidController, AvoidCtx, Level, carrot_point
from .threat_field import ThreatField, build_field, path_exposure
from .command import CommandParser, LLMClient, build_prompt, merge, parse_base_command, parse_llm_json
from .env import SmogMonitor, TowerMonitor, UavMonitor, image_contrast, speckle_filter
from .explore import CubeFinder, FaceTour, ViewpointTour
from .explore_online import Cand, OnlineExplorer
from .guard import Locks, MatchLatch, SensorWatch
from .field_policy import OwnBasePolicy
from .deadlock import DeadlockMixin
from .submit import Submitter


class Mode(str, Enum):
    WAIT = "WAIT"
    EXPLORE = "EXPLORE"
    STRIKE = "STRIKE"
    HUNT = "HUNT"
    STAGE = "STAGE"
    GOAL = "GOAL"
    RECOVER = "RECOVER"
    SAFE_STOP = "SAFE_STOP"
    HALT = "HALT"
    REPORT = "REPORT"
    DONE = "DONE"


class Agent(DeadlockMixin):
    def __init__(self, robot: Robot, server: ServerLink, cfg: Optional[Config] = None,
                 detector: Optional[Detector] = None, llm: Optional[LLMClient] = None,
                 seed: int = 0, name: str = "agent", stack: Optional[NavStack] = None) -> None:
        self.robot, self.server = robot, server
        self.cfg = cfg or Config()
        self.geo: ArenaGeometry = self.cfg.arena
        self.name = name
        self.detector = detector or SimDetector()
        self.llm = llm
        self.cam = CameraModel(self.cfg.camera)
        self.parser = CommandParser()
        # 위치추정(SLAM)+지도+경로계획(Nav2) 한 묶음. config.nav.backend: builtin | nav2 | nav2-fake
        self.stack: NavStack = stack or make_stack(self.cfg, robot, seed=seed)
        self.omap = ObjectMap(self.cfg)
        self.coverage = CoverageMap(self.cfg, self.cam)             # 관측 지도·박스/면 장부(perception/coverage.py)
        self.explorer = OnlineExplorer(self.cfg, self.coverage)     # M3 온라인 탐색 시점 선택(mission/explore_online.py)
        self._ex_cand: Optional[Cand] = None
        self._ex_phase = "travel"                                   # travel | observe
        self._ex_sel_t = -1e9
        self._ex_goal_t0 = 0.0
        self._ex_obs_t0 = 0.0
        self._ex_aligned_since: Optional[float] = None
        self._ex_aligned_face: Optional[Tuple[int, int]] = None
        self._ex_views0: Dict[Tuple[int, int], int] = {}
        self._ex_spin_acc = 0.0
        self._ex_spin_prev = 0.0
        self.tracker: Optional[UgvTracker] = None
        self.aim = AimController(self.cfg)
        self.laser = LaserManager(self.cfg, robot)
        self.cubefinder = CubeFinder(self.geo)
        self.facetour = FaceTour(self.geo)
        self._look: Optional[Dict[str, Any]] = None
        self._cubes_cache: Tuple[float, List] = (-1e9, [])
        self.tour = ViewpointTour(self.geo, self.cfg.mission.tour_nx, self.cfg.mission.tour_ny)
        # 정체성
        self.team: Optional[str] = self.cfg.team
        self.slot: Optional[int] = self.cfg.slot
        # 상태
        self.mode = Mode.WAIT
        self._mode_since = 0.0
        self.t_start: Optional[float] = None
        self.cmd: Optional[Command] = None
        self._cmd_ver = 0                                # M1 명령 버전: 늦은 LLM 응답이 새 명령을 덮어쓰지 못하게 (R11)
        self.m5_type: Optional[str] = None
        self.m5_base: Optional[str] = None
        self.m5_look_since: Optional[float] = None
        self.m5_entered_local: Optional[float] = None    # 자체 위치로 거점 진입을 추정한 시각 — 경기 종료의 근거가 아니다 (R14)
        self.target_done = False
        self.target_failed = False
        self.log: List[Tuple[float, str]] = []
        self.stats: Dict[str, Any] = {"mode_time": {}, "state_time": {}, "strike_shots": 0, "hunt_shots": 0,
                                      "recoveries": 0, "multi_output": 0, "det_consumed": 0, "det_dropped": 0,
                                      "safe_stops": {}, "msg_rejected": {}, "detector_errors": 0, "link_errors": 0,
                                      "lidar_suspects": [], "pose_at_clamped": 0, "aim_resets": 0,
                                      # 이벤트 요소(상태전이표 6장)
                                      "smog_changes": 0, "smog_s": 0.0, "game_events": {}, "base_entries": [],
                                      "wrong_target_inhibit": 0, "led_confirmed": 0, "tower_obs": 0, "uav_state": "unknown",
                                      # 3D 지도(깊이 카메라) 사용
                                      "depth_frames": 0, "depth_stale_skipped": 0, "nav_layer_cells": 0, "depth_guard_ticks": 0,
                                      "range_from_depth": 0, "beam_blocked_ticks": 0, "beam_override": 0, "beam_checked": 0,
                                      "height_reject": 0, "depth_lidar_alarm": 0, "bump_events": 0,
                                      # 실기 API(PDF p10~18)
                                      "tick_ms_max": 0.0, "tick_over_budget": 0, "paused_s": 0.0, "match_locked_s": 0.0,
                                      "tilt_stops": 0, "restored": False}
        self._stall = StallDetector(self.cfg.map3d.bump_window_s, self.cfg.map3d.bump_v_min, self.cfg.map3d.bump_ratio)   # 접촉(정체) 감지
        self._bumps: List[Tuple[float, float, float, float]] = []    # (감지 시각, 장애물 x, y, 접촉 때 헤딩) — 접촉으로 알아낸 LiDAR·깊이에 안 보이는 장애물
        self._bump_dirty = False
        # 3D 지도: 깊이 영상 + 카메라 자세(TF) + 로봇 자세(오도메트리·SLAM) → 복셀 지도. 깊이가 없거나 꺼져 있으면 None (LiDAR 2D 만으로 동작)
        self.fusion: Optional[DepthFusion] = DepthFusion(self.cfg) if (self.cfg.depth.enable and self.cfg.map3d.enable) else None
        self._sync_depth = StampSync(floor_s=self.cfg.depth.min_latency_s)
        self._depth_stamp = -1.0
        self._layer_ver = -1
        self._det_ctx: Optional[Tuple[float, Pose2D, float, float]] = None      # 현재 검출이 나온 프레임의 (robot 시각, 자세, 팬, 틸트)
        self._beam_since: Optional[float] = None
        self._beam_logged = -1e9
        self.smog = SmogMonitor(self.cfg.event)            # E3 스모그: LiDAR 고립 반사·영상 대비·서버 이벤트로 감지 (mission/env.py)
        self.uavmon = UavMonitor(self.cfg.event)           # E4 드론 영상 주기(1 s→2 s→5 s→중단)
        self.tower = TowerMonitor(self.cfg.event)          # E2 3색 타워 LED(준비·진행·완료) — 서버 STOP 유실 보조
        self._link_ok = True
        self._contrast: Optional[float] = None             # 최신 프레임의 영상 대비(실영상일 때만)
        self._stop_via_event = False                       # STOP 이 '경기상태 finished' 이벤트로 왔는가
        self._stop_via_match = False                       # STOP 이 /mil/match 의 cmd=report/stop 으로 왔는가 (서버 STOP 유실 보조)
        self._saw_run = False                              # /mil/match 가 start/resume 이었던 적이 있는가 (이전 경기의 낡은 cmd=stop 오인 방지)
        self._pause_s = 0.0                                # cmd=pause 로 멈춘 경기 시계 누적
        self._tilt_latched = False                         # IMU 기울기 정지 래치 (히스테리시스)
        self._oled_t = -1e9
        self._batt_warned = False
        self._depth_off = False                            # 배터리 부족 시 깊이 영상 처리 중단
        self._restore_tried = False
        self._wrong_logged = 0.0
        self.m5_confirmed_event: Optional[float] = None    # 서버가 '내가 거점에 진입했다'고 알린 시각
        self._sync_frame = StampSync(floor_s=self.cfg.camera.min_latency_s)    # 카메라 stamp → robot.now() 시계 (hal/clock.py)
        self._sync_scan = StampSync(floor_s=self.cfg.lidar.min_latency_s)
        self._scan_t: Optional[float] = None               # 최신 스캔의 robot.now() 시계 시각
        self.carry_over: List[ServerMsg] = []              # 정지 뒤 도착한 '다음 경기' 메시지 — 러너가 새 Agent 에 넘긴다(⑨)
        self.mask = SelfMask(self.cfg.lidar)               # LiDAR 자기 가림 마스크 + 의심 구간 감시 (nav/selfmask.py)
        self._mask_cache: Tuple[float, Optional[Scan]] = (-1.0, None)
        self._health_seen: Dict[str, Tuple[str, float]] = {"link": ("", -1e9), "detector": ("", -1e9)}
        # ---- 상태 전이표 부품
        mc = self.cfg.mission
        self.latch = MatchLatch()
        self.sw = SensorWatch(mc.lidar_stale_s)
        self.locks = Locks(mc.hit_grace_s)
        self.diff = DiffDrive(self.cfg.robot)            # 차동 구동: 슬립·편차 보상 + 바퀴 속도 한계 (hal/diffdrive.py)
        self.field_policy = OwnBasePolicy(self.cfg)
        self.submitter = Submitter(server, mc.submit_retry_s, mc.submit_ack_timeout_s, mc.submit_max_tries)
        self.epoch = 0                                   # 목표·상태 세대 번호: 취소된 작업의 늦은 결과를 거르는 데 쓴다
        self.halt_t: Optional[float] = None
        self.stop_source: Optional[str] = None           # server | local_timeout
        self.m6_result: Optional[str] = None             # acked | rejected | unconfirmed
        self._m6: Optional[Dict[str, Any]] = None
        self._safe: Dict[str, Any] = {}
        self._safe_hold_until = -1e9
        self._rec: Dict[str, Any] = {}
        self._rec_hist: deque = deque(maxlen=16)
        self._rec_cooldown = -1e9
        self._blocked_since: Optional[float] = None
        self._pending_events: List[Dict[str, Any]] = []
        self._tick_no = 0
        self._out_tick = -1
        self._m3_last: Tuple[float, Optional[Dict]] = (-1e9, None)
        self._m3_final_requested = False
        # 센서 캐시
        self._last_t: Optional[float] = None
        self._scan: Optional[Scan] = None
        self._scan_stamp = -1.0
        self._frame = None
        self._frame_stamp = -1.0
        self._frames: deque = deque(maxlen=16)           # 최근 프레임(stamp로 검출 결과와 짝을 찾는다 — R03)
        self._dets: List[Detection] = []
        self._dets_t = -1e9                              # 마지막 검출 결과를 소비한 시각(신선도)
        self._det_consumed = -1.0                        # 이미 반영한 검출 결과의 프레임 stamp
        self._uav_stamp = -1.0
        self._pending_llm: Optional[Dict[str, Any]] = None
        # 모드별 내부 상태
        self._spin_acc = 0.0
        self._spin_active = False
        self._strike: Dict[str, Any] = {}
        self._hunt: Dict[str, Any] = {}
        self._invuln_until: Dict[str, float] = {}      # 상대 ident → 무적 종료 시각(우리가 맞힌 직후 10 s)
        self._hunt_block: Dict[int, float] = {}
        self._enemy_streak = 0
        self._v_cmd = self._w_cmd = 0.0
        self._prev_hits = 0
        self._prev_ugv_hit_by_us = 0
        self._cur_goal: Optional[Tuple[float, float]] = None
        self._goal_kind: Tuple = ("lattice",)
        self._goal_t0 = 0.0
        self._hist: deque = deque(maxlen=120)           # (t, x, y, th) 최근 자세 이력 — 센서 시각 동기화용
        self._dth = 0.0
        self._pose_frame: Optional[Pose2D] = None
        # ---- 적 UGV 회피(안전 계층): LiDAR 동적 군집 → 위협 보드 → 위협장(시간 층) → 회피 제어기. 임무 계층/A*/Nav2 는 그대로 둔다.
        self.blobs = BlobDetector(self.cfg)
        self.board: Optional[ThreatBoard] = None
        self.avoid = AvoidController(self.cfg)
        self._blobs: List = []
        self._av_tracks: List = []
        self._field: Optional[ThreatField] = None
        self._field_t = -1e9
        self._field_ver = 0
        self._av_maps: Tuple[float, Any, Any] = (-1e9, None, None)
        self._av_ctx: Optional[AvoidCtx] = None
        self._rc: Dict[str, Any] = {}                     # 위험 인지 경유지(carrot) 캐시
        self._strike_block_until = -1e9
        self._m2_defer_s = 0.0
        self._kappa = 1.0
        self._init_deadlock()
        self.stats.update({"avoid_ticks": 0, "avoid_acquired": 0, "carrot_sent": 0, "risk_plans": 0, "risk_plan_fail": 0,
                           "m2_deferred_s": 0.0, "field_builds": 0, "field_ms": 0.0})

    # ================================================================== 자세 이력
    def pose_at(self, t: float) -> Pose2D:
        """시각 t의 자세(선형 보간). 센서 데이터는 도착 시점이 아니라 '촬영 시각'의 자세와 맞춰야 한다."""
        h = self._hist
        if not h:
            return self.stack.pose.copy()
        if t > h[-1][0] + 0.3 or t < h[0][0] - 0.3:          # 이력 범위 밖: 센서 stamp 시계가 로봇 시계와 다르다는 신호(⑧). 조용히 넘기지 않는다
            self.stats["pose_at_clamped"] += 1
            n = self.stats["pose_at_clamped"]
            if n == 1 or n % 200 == 0:
                self._log(h[-1][0], f"WARN pose_at(t={t:.2f}) 가 자세 이력 [{h[0][0]:.2f}, {h[-1][0]:.2f}] 밖이라 가장자리 자세로 대체했다"
                                    f"({n}회). 센서 stamp 가 robot.now() 시계가 아닐 수 있다(hal/clock.py)")
        if t >= h[-1][0]:
            return Pose2D(h[-1][1], h[-1][2], h[-1][3])
        if t <= h[0][0]:
            return Pose2D(h[0][1], h[0][2], h[0][3])
        for i in range(len(h) - 1, 0, -1):
            if h[i - 1][0] <= t <= h[i][0]:
                a, b = h[i - 1], h[i]
                k = (t - a[0]) / max(b[0] - a[0], 1e-9)
                return Pose2D(a[1] + (b[1] - a[1]) * k, a[2] + (b[2] - a[2]) * k, wrap(a[3] + wrap(b[3] - a[3]) * k))
        return self.stack.pose.copy()

    def _pose_at_scan(self) -> Optional[Pose2D]:
        """최신 스캔을 받은 시각의 자세(스캔 stamp 가 아니라 robot.now() 시계로 변환한 시각)."""
        if self._scan is None:
            return None
        return self.pose_at(self._scan_t if self._scan_t is not None else self._scan.stamp)

    def _log_sync(self, what: str, sync: StampSync, now: float) -> None:
        ch = sync.consume_change()
        if ch is not None and (ch[0] == "offset" or sync.switches > 1):
            self._log(now, f"WARN {what} stamp 가 robot.now() 와 다른 시계로 보인다(mode={ch[0]}, offset={ch[1]:+.3f}s) — "
                           f"변환해 쓰지만 최소 지연만큼의 오차가 남는다. HAL 이 robot.now() 시계로 stamp 를 채우는 것이 맞다")

    # ================================================================== 로그
    def _log(self, now: float, msg: str) -> None:
        t = now - (self.t_start if self.t_start is not None else now)
        self.log.append((round(t, 2), REDACT(msg)))

    def _set_mode(self, now: float, mode: Mode, why: str = "") -> None:
        if mode != self.mode:
            self._log(now, f"mode {self.mode.value}->{mode.value} {why}")
            self.mode, self._mode_since = mode, now
            self.aim.reset()
            self._ex_cand, self._ex_phase = None, "travel"          # 탐색 중이던 시점·관측 단계는 다른 모드를 거치면 버린다(오래된 목표를 이어가지 않음)
            if mode in (Mode.STRIKE, Mode.HUNT):
                self.stack.clear_goal()
            if mode in (Mode.GOAL, Mode.STAGE, Mode.SAFE_STOP, Mode.RECOVER, Mode.HALT):
                # 이전 목표·탐색 상태를 새 상태로 끌고 가지 않는다(순위 06·07·13): 항법 목표 취소, 세대 증가
                self.epoch += 1
                self.stack.clear_goal()
                self._cur_goal = None
                self._look = None
                self._spin_active = False
                self._blocked_since = None

    # ------------------------------------------------------------------ 공개 상태
    @property
    def stopped(self) -> bool:
        """STOP(서버) 또는 로컬 만료로 정지 절차에 들어갔는가."""
        return self.latch.stopped or self.halt_t is not None

    @property
    def m1_submitted(self) -> bool:
        return self.submitter.started("m1")

    @property
    def m3_submitted(self) -> bool:
        return self.submitter.started("m3")

    @property
    def m6_submitted(self) -> bool:
        return self.submitter.started("m6")

    @property
    def state_name(self) -> str:
        """전이표의 세부 상태 이름. STRIKE 는 내부 단계에 따라 STRIKE_APPROACH / STRIKE_AIM."""
        if self.mode == Mode.STRIKE:
            return "STRIKE_AIM" if self._strike.get("phase") == "aim" else "STRIKE_APPROACH"
        return self.mode.value

    def _tower_stop_due(self, now: float) -> bool:
        """E2: 3색 타워 LED 가 '완료'를 충분히 오래·일관되게 보여 주고, 서버 연결이 끊겨 STOP 을 못 받는 상황이면 정지한다.
        오검출로 경기 중에 멈추면 치명적이므로 조건이 많다: 시작 후 tower_min_t 이상, 연속 프레임 확인, 신뢰도, 그리고 링크 이상."""
        ev = self.cfg.event
        if not ev.tower_stop_backup or self.latch.stopped or self.t_start is None or now - self.t_start < ev.tower_min_t:
            return False
        if ev.tower_requires_link_down and self._link_ok:
            return False
        if not self.tower.confirmed("finished", now):
            return False
        self._log(now, f"WARN 3색 타워가 '완료'({ev.tower_finished_color})를 {ev.tower_confirm_s:.1f}s 보여 주는데 서버 STOP 이 없다"
                       f"(링크 {'이상' if not self._link_ok else '정상'}) → 보조 정지")
        return True

    def weapon_locked(self, now: float) -> bool:
        return self.locks.weapon_locked(self.robot.status(), now)

    def drive_locked(self, now: float) -> bool:
        return self.locks.drive_locked(self.robot.status(), now)

    def vision_fresh(self, now: float) -> bool:
        if self._deadlock_enabled():
            # Inference completion is not the age of its evidence. The frame
            # context already carries capture time in the robot clock domain.
            if self._det_ctx is None:
                return False
            age = now - self._det_ctx[0]
            return math.isfinite(age) and -.05 <= age <= self.cfg.mission.vision_stale_s
        return now - self._dets_t <= self.cfg.mission.vision_stale_s

    # ================================================================== 메인 루프
    def tick(self) -> None:
        """한 틱. 실기 API 규칙(PDF p18): team_node 의 step() 은 0.1 s 이내 — 넘으면 센다(AI 추론은 비동기 검출기 스레드)."""
        t0 = time.perf_counter()
        try:
            self._tick()
        finally:
            ms = (time.perf_counter() - t0) * 1000.0
            self.stats["tick_ms_max"] = max(self.stats["tick_ms_max"], ms)
            if ms > self.cfg.hal.step_budget_s * 1000.0:
                self.stats["tick_over_budget"] += 1
                n = self.stats["tick_over_budget"]
                if n == 1 or n % 100 == 0:
                    self._log(self._last_t if self._last_t is not None else 0.0,
                              f"WARN 틱 {ms:.0f} ms > {self.cfg.hal.step_budget_s * 1000:.0f} ms (PDF p18 step 한도) — {n}회째. 무거운 처리가 제어 루프에 있다")

    def _tick(self) -> None:
        now = self.robot.now()
        dt = 0.05 if self._last_t is None else clamp(now - self._last_t, 1e-3, 0.25)
        self._last_t = now
        self._tick_no += 1
        if self.mode == Mode.DONE:
            return
        mc = self.cfg.mission
        # ---- (00) 서버 메시지·STOP 래치는 센서·SLAM 처리보다 먼저. 센서가 막혀도 STOP은 즉시 처리된다 (R01)
        self._serve(now)
        st_api = self.robot.status()
        self._match_api(now, dt, st_api)                    # /mil/match: 일시정지 시계 정지 · report/stop 보조 정지 · 재시작 복원
        self._note_health(now, "link", *self.server.health())          # 서버 연결 오류를 로그·통계로 드러낸다(조용한 대기 방지)
        self._guard_m3_submission(now)
        self.submitter.update(now)
        if not self.latch.stopped:                           # 한도를 둔 설정에서 포기한 M1/M3 는 STOP 전까지 다시 시도한다
            for kind in ("m1",):  # M3 must never be revived without its original receive-time deadline.
                if self.submitter.revive(kind, now):
                    self._log(now, f"{kind.upper()} submit revived after giving up")
        if self.mode not in (Mode.HALT, Mode.REPORT):
            if self.latch.stopped:
                self._enter_halt(now, "match_cmd" if self._stop_via_match else ("server_event" if self._stop_via_event else "server"))
            elif self._tower_stop_due(now):
                self._enter_halt(now, "tower")               # 서버 STOP 이 유실됐고 3색 타워가 '완료'를 보여 준다(E2)
            elif self.t_start is not None and now - self.t_start >= mc.t_total + mc.local_stop_grace_s:
                self._enter_halt(now, "local_timeout")       # 통신 유실 대비 로컬 안전 정지
        if self.mode in (Mode.HALT, Mode.REPORT):
            self._mode_report(now, dt)
            return
        if self.t_start is not None:
            # Previously acquired evidence must still be sent during a sensor
            # outage or localization stop; no new sensor frame is required.
            self._submit_m3_if_due(now, now - self.t_start)
            self._poll_llm(now)  # M1 replacement must not wait on a fresh SLAM/sensor sample either.
        # ---- 센서·SLAM
        if not self._sense(now, dt):
            if self.t_start is not None:                     # (03) START는 왔는데 SLAM/TF가 아직: SAFE_STOP
                self._enter_safe(now, "pose_not_ready")
            self._stop_output()
            return
        self._update_field_policy(now)
        self._drain_events(now)
        if self.t_start is None:
            self._apply(0.0, 0.0, 0.0, 0.0, dt)
            return
        t = now - self.t_start
        self._stall_check(now, dt)
        self._threat_update(now, dt, t)
        self._deadlock_update(now, dt, t)
        self._decide(now, dt, t)
        self.stats["aim_resets"] = self.aim.auto_resets
        if self.smog.active:
            self.stats["smog_s"] += dt
        ms, ss = self.stats["mode_time"], self.stats["state_time"]
        ms[self.mode.value] = ms.get(self.mode.value, 0.0) + dt
        ss[self.state_name] = ss.get(self.state_name, 0.0) + dt

    # ================================================================== 감지
    def _note_health(self, now: float, what: str, ok: bool, why: str) -> None:
        """링크·검출기의 문제를 같은 문구는 10 s 에 한 번만 로그하고, 회복도 남긴다."""
        last_why, last_t = self._health_seen[what]
        if what == "link":
            self._link_ok = ok
        if not ok:
            if why != last_why or now - last_t >= 10.0:
                self._health_seen[what] = (why, now)
                self.stats["link_errors" if what == "link" else "detector_errors"] += 1
                self._log(now, f"WARN {what} 문제: {why}")
        elif last_why:
            self._health_seen[what] = ("", -1e9)
            self._log(now, f"{what} 회복")

    def _masked_scan(self, sc: Optional[Scan], now: float) -> Optional[Scan]:
        """자기 가림 마스크 → 스모그 감시 → (스모그 중이면) 고립 반사 제거를 거친 스캔(스탬프별 1회).
        이후 격자·안전 제한·SLAM·거리추정이 모두 이 스캔을 쓴다."""
        if sc is None:
            return sc
        if self._mask_cache[0] == sc.stamp and self._mask_cache[1] is not None:
            return self._mask_cache[1]
        out = self.mask.apply(sc) if self.mask.active else sc
        ev = self.cfg.event
        ch = self.smog.update(out, now, self._contrast)
        if ch is not None:
            self._on_smog_change(now, ch, "sensor")
        if self.smog.active:                                  # 연기 입자의 고립 반사를 지운다(안 지우면 허상이 전방 정지·지도에 쌓인다)
            out = speckle_filter(out, ev.smog_near_m, ev.speckle_nbr_m)
        self._mask_cache = (sc.stamp, out)
        return out

    def _on_smog_change(self, now: float, on: bool, src: str) -> None:
        self.stats["smog_changes"] += 1
        self._log(now, f"SMOG {'ON' if on else 'OFF'} (src={src}, speckle={self.smog.index:.2f}) → "
                       f"{'고립 반사 제거·감속·발사 신뢰도 상향·객체 집계 거리 제한' if on else '정상 복귀'}")

    def _sense(self, now: float, dt: float) -> bool:
        raw = self.robot.get_scan()
        sc = self._masked_scan(raw, now)
        if not self.stack.initialized:
            if sc is None:
                return False
            if self.tracker is None:                        # 정체성은 한 번만 정한다(SLAM 준비 대기 중 재시도해도 로그 폭주 방지)
                ident = self.server.identity()
                if ident:
                    self.team, self.slot = ident
                if self.team is None or self.slot is None:
                    self._log(now, "WARN: 팀/슬롯 미확인 → BLUE/2 가정 (config.team/slot 설정 필요)")
                    self.team = self.team or "BLUE"
                    self.slot = self.slot or 2
                self.tracker = UgvTracker(self.cfg, self.team)
                self.board = ThreatBoard(self.cfg, self.team)
            base =f"{'B' if self.team == 'BLUE' else 'R'}-{self.slot}"
            cx, cy = self.geo.base_center(base)
            p0 = self.stack.start(sc, self.robot.get_odom(), (cx, cy))
            if p0 is None:                                  # SLAM/TF 준비 전: 정지 상태로 기다린다
                return False
            self._log(now, f"init team={self.team} slot={self.slot} backend={self.cfg.nav.backend} "
                           f"pose=({p0.x:.2f},{p0.y:.2f},{p0.th:.2f})")
        new_scan = sc if (sc is not None and sc.stamp != self._scan_stamp) else None
        if new_scan is not None:
            self._scan_stamp, self._scan = new_scan.stamp, new_scan
            self._scan_t = self._sync_scan.to_robot(new_scan.stamp, now)     # 같은 시계면 그대로, 다르면 변환
            self._log_sync("scan", self._sync_scan, now)
            self.sw.on_scan(new_scan.stamp, now)
        self.stack.update(self.robot.get_odom(), new_scan, now)     # SLAM: 위치추정 + 지도 갱신
        self._hist.append((now, self.stack.pose.x, self.stack.pose.y, self.stack.pose.th))
        if new_scan is not None:
            self.coverage.update_scan(new_scan, self.stack.pose)    # '실제로 훑은 곳' 지도(점유격자와 별개 — 프런티어·정보 가치용)
            if self.cfg.avoid.enable and self.cfg.avoid.use_blobs:
                self._blobs = self.blobs.update(new_scan, self.stack.pose, self.stack.grid, now, self._w_cmd,
                    static_boxes=self.coverage.confirmed_static_footprints(now, self.omap))
        if new_scan is not None and raw is not None:                # 설정 없이도 자기 가림을 알아채게: 원본 스캔을 감시
            for msg in self.mask.observe(raw, self.stack.pose.th, now):
                self.stats["lidar_suspects"].append(msg)
                self._log(now, "WARN " + msg)
        self._ingest_vision(now)
        self._ingest_depth(now)
        old_layout_state = self.coverage.uav_layout_state
        self.coverage.tick_uav_layout(now)
        uav = self.server.get_uav()
        if uav is not None and uav.stamp > self._uav_stamp:          # 단조 증가하는 촬영 시각만 (R05): 늦게 온 옛 프레임은 버린다
            self._uav_stamp = uav.stamp
            self.uavmon.observe(now)                                 # E4: 새 프레임 도착 간격으로 영상 주기를 잰다
            if uav.age_s is not None:                                # 실기: stamp 는 서버 벽시계 → 로봇 시계의 촬영 시각으로 바꿔 트랙에 넣는다
                waited = max(time.monotonic() - uav.recv_mono, 0.0) if uav.recv_mono is not None else 0.0
                uav = dataclasses.replace(uav, stamp=now - uav.age_s - waited)
            if getattr(uav, "layout_source", None) is not None:
                layout_t = getattr(uav, "layout_stamp", None)
                self.coverage.ingest_uav_boxes(getattr(uav, "boxes", ()),
                                               uav.stamp if layout_t is None else layout_t, self.stack.grid)
            capture_pose = self.pose_at(uav.stamp)
            alignment = self.coverage.uav_alignment_report
            self_pose_arena = Pose2D(capture_pose.x - alignment.get("dx", 0.0),
                                    capture_pose.y - alignment.get("dy", 0.0), capture_pose.th)
            self.tracker.update_uav(uav, self_pose_arena,
                                    self_ident=f"{self.team[0]}-UGV-{self.slot}" if self.team and self.slot else None)
        self.uavmon.tick(now)
        if self.uavmon.state != self.stats["uav_state"]:
            self._log(now, f"UAV 영상 상태 {self.stats['uav_state']} → {self.uavmon.state} (주기 ≈ {self.uavmon.period:.1f}s)"
                           f" — 상대의 지정표적 유효타격으로 영상이 줄거나 끊긴다(규정). 트랙 보존 {self.uavmon.track_max_age():.1f}s")
            self.stats["uav_state"] = self.uavmon.state
        self.coverage.refresh(self.stack.grid, self.omap, now)       # 박스·면 장부, 이동 감지, 분류 득표 동기화(내부에서 1 s 주기로만 실행)
        if old_layout_state != self.coverage.uav_layout_state:
            self._log(now, f"UAV 배치 영상 {old_layout_state} → {self.coverage.uav_layout_state}"
                           f" — 탐색 {self.coverage.layout_diagnostics()['mode']}")
            # Re-evaluate image-dependent goals immediately on outage/recovery.
            # Keep all ground observations and SLAM evidence; only the plan is reset.
            if self.mode == Mode.EXPLORE:
                self._ex_cand, self._ex_phase, self._ex_sel_t = None, "travel", -1e9
                self.stack.clear_goal()
        self.stats["uav_layout_state"] = self.coverage.uav_layout_state
        return True

    def _ingest_vision(self, now: float) -> None:
        """검출 결과를 '그 결과가 나온 프레임'의 시각·자세·팬틸트와 묶어 한 번만 반영한다 (R03).
        비동기 검출기가 이전 프레임의 결과를 줄 수 있으므로, 새 프레임의 자세를 입히지 않는다."""
        fr = self.robot.get_frame()
        fresh_frame = fr is not None and fr.stamp != self._frame_stamp
        if fresh_frame:
            self._frame_stamp, self._frame = fr.stamp, fr
            # 짝짓기용 보관본은 이미지를 뺀다(5MP BGR 한 장이 약 15 MB — 16장이면 240 MB). 방위 계산엔 stamp·팬·틸트·크기만 쓴다
            self._contrast = image_contrast(fr.image)               # E3: 영상 대비(연기가 끼면 낮아짐). 시뮬은 영상이 없어 None
            t_r = self._sync_frame.to_robot(fr.stamp, now)          # 도착 시점에 변환해 둔다(비동기 검출은 한참 뒤에 이 프레임을 쓴다)
            self._log_sync("frame", self._sync_frame, now)
            self._frames.append(dataclasses.replace(fr, image=None, meta={"t_robot": t_r}))
            # 실제로 볼 수 있었던 면(거리·입사각·시야·가림 통과)에 독립 관측을 기록 — 검출이 없었어도 '봤는데 없었다'가 된다
            self.coverage.record_view(self.pose_at(t_r), fr.pan, self.stack.grid, t_r)
        # 동기 검출기는 새 프레임에서만 돌린다. 비동기 검출기는 매 틱 '완료된 결과'만 가져온다.
        if fr is not None and (fresh_frame or self.detector.asynchronous):
            try:
                dets, dstamp = self.detector.detect_stamped(fr)
            except Exception as e:                                   # 동기 검출기(YOLO) 예외가 제어 루프를 죽이면 안 된다
                self._note_health(now, "detector", False, f"{type(e).__name__}: {e}")
                dets, dstamp = [], -1.0                              # 결과 없음 → 오래되면 영상 신선도 감시가 발사를 막는다
            else:
                self._note_health(now, "detector", *self.detector.health())
            if dstamp > self._det_consumed:
                src = next((f for f in reversed(self._frames) if f.stamp == dstamp), None)
                if src is None:                                      # 짝이 되는 프레임을 잃었다 → 섞어 쓰지 않고 폐기
                    self.stats["det_dropped"] += 1
                else:
                    self._det_consumed, self._dets, self._dets_t = dstamp, dets, now
                    self.stats["det_consumed"] += 1
                    self._pose_frame = self.pose_at(src.meta.get("t_robot", src.stamp))
                    self._det_ctx = (src.meta.get("t_robot", src.stamp), self._pose_frame, src.pan, src.tilt)
                    pose_scan = self._pose_at_scan()
                    self._dth = wrap(self._pose_frame.th - pose_scan.th) if pose_scan is not None else 0.0
                    self.omap.observe(dets, self._pose_frame, src, self._scan, self._dth, pose_scan,
                                      max_range=self.cfg.event.smog_obs_range_m if self.smog.active else None,
                                      boxes=self.coverage.boxes.values())
                    self.tracker.update_camera(dets, self._pose_frame, src, self._scan, self._dth)
                    for d in dets:                                    # E2: 3색 타워 LED 색 → 경기 상태 관측
                        if d.cls == "TOWER_LED" and d.conf >= self.cfg.event.tower_conf_min:
                            if self.tower.observe(d.extra.get("tower_color"), now) is not None:
                                self.stats["tower_obs"] += 1
        if self._dets and not self.vision_fresh(now):
            self._dets = []                                          # 오래된 검출은 조준·사격에 쓰지 않는다

    # ================================================================== 3D 지도 (깊이 카메라)
    def _ingest_depth(self, now: float) -> None:
        """깊이 영상 → 3D 복셀 지도. 촬영 시각의 차체 자세(pose_at: 오도메트리가 예측하고 LiDAR 스캔 정합이 교정한 자세의 이력)와
        촬영 시점의 팬/틸트로 깊이 카메라 자세(TF 체인)를 구해 점들을 월드에 등록한다. 깊이가 없으면 아무것도 안 한다(LiDAR 2D 로 동작)."""
        fu = self.fusion
        if fu is None or self._depth_off:
            return
        df = self.robot.get_depth()
        if df is None or df.stamp == self._depth_stamp:
            return
        self._depth_stamp = df.stamp
        self.stats["depth_frames"] += 1
        t_r = self._sync_depth.to_robot(df.stamp, now)
        self._log_sync("depth", self._sync_depth, now)
        pose = self.pose_at(t_r)
        mr = self.cfg.map3d.smog_max_range if self.smog.active else None      # 연기 속 스테레오는 가까운 것만 믿는다
        did = fu.ingest(df, t_r, pose, self._scan, self._pose_at_scan(), mr)
        if fu.stats["lidar_bias_alarm"] > self.stats["depth_lidar_alarm"]:
            self.stats["depth_lidar_alarm"] = fu.stats["lidar_bias_alarm"]
            self._log(now, f"WARN 깊이↔LiDAR 정합 잔차 {fu.stats['lidar_resid_m']:+.3f} m — 깊이 카메라 설치(rig.depth_xyz/rpy)·팬 영점·"
                           f"시각 동기를 점검할 것(3D 지도가 어긋나 쌓인다)")
        if did and fu.map.version != self._layer_ver:
            self._layer_ver = fu.map.version
            self._push_layer()

    def _push_layer(self) -> None:
        """항법에 'LiDAR 가 못 보는 장애물'(깊이 3D 층 + 접촉으로 알아낸 것)을 더한다(내장 A*·Nav2 폴백·Nav2 비용지도)."""
        layer = self.fusion.nav_layer() if self.fusion is not None else None
        bump = self._bump_mask()
        if bump is not None:
            layer = bump if layer is None else (layer | bump)
        self.stack.set_obstacle_layer(layer)
        self.stats["nav_layer_cells"] = int(layer.sum()) if layer is not None else 0

    def _update_field_policy(self, now: float) -> None:
        elapsed = None if self.t_start is None else now-self.t_start
        was_active = self.field_policy.active
        mask = self.field_policy.planning_mask(self.stack.grid, self.stack.pose, self.team, elapsed)
        changed = self.stack.set_keepout_layer(mask)
        self.stats["own_base_keepout_active"] = self.field_policy.active
        self.stats["own_base_keepout_cells"] = int(mask.sum()) if mask is not None else 0
        if changed:
            self._rc = {}
            self._av_maps = (-1e9, None, None)
            self._ex_cand = None
        if self.field_policy.active and not was_active:
            self._log(now, "Own-base keepout active; initial overlaps allow outward escape only")

    def _bump_mask(self) -> Optional[np.ndarray]:
        if not self._bumps:
            return None
        g = self.stack.grid
        mask = np.zeros(g.L.shape, dtype=bool)
        for _, bx, by, th in self._bumps:
            c, s = math.cos(th), math.sin(th)
            for lat in np.linspace(-0.15, 0.15, 7):
                for dep in (-0.05, 0.0, 0.05, 0.10):
                    iy, ix = g.to_cell(bx + c * dep - s * lat, by + s * dep + c * lat)
                    if g.in_bounds(iy, ix):
                        mask[iy, ix] = True
        return mask

    def _stall_check(self, now: float, dt: float) -> None:
        """접촉(정체) 감지. 전진하라고 계속 명령했는데 SLAM 자세가 거의 안 변하고 LiDAR 앞은 비었다 = LiDAR·깊이에 안 보이는 것에 막혔다.
        (바퀴 오도메트리는 헛도는 바퀴도 이동으로 재므로 쓰지 않고 SLAM/스캔 정합 자세를 쓴다.) 그 자리를 기억하고 RECOVER 한다."""
        mc = self.cfg.map3d
        if self._bumps and now - self._bumps[0][0] > mc.bump_ttl_s:
            self._bumps = [b for b in self._bumps if now - b[0] <= mc.bump_ttl_s]
            self._bump_dirty = True
        if self._bump_dirty:
            self._bump_dirty = False
            self._push_layer()
        if (not mc.bump_enable or self.mode not in (Mode.EXPLORE, Mode.STRIKE, Mode.HUNT, Mode.STAGE, Mode.GOAL)
                or self.drive_locked(now)):
            self._stall.reset()
            return
        p = self.stack.pose
        st = self._stall.update(now, dt, p.x, p.y, self._v_cmd)          # nav/stall.py (현장 실험 F07 과 같은 로직)
        if st is None:
            return
        expected = st["expected"]
        sc = self._scan
        if sc is not None and sc.min_in_sector(0.0, math.radians(25)) < mc.bump_lidar_clear_m:
            return                                                    # LiDAR 가 보는 장애물: 기존 안전 제한·RECOVER 가 맡는다
        self.stats["bump_events"] += 1
        self._bumps.append((now, p.x + mc.bump_ahead_m * math.cos(p.th), p.y + mc.bump_ahead_m * math.sin(p.th), p.th))
        self._push_layer()
        self._log(now, f"BUMP 전진 명령인데 {mc.bump_window_s:.1f}s 동안 거의 안 움직임(기대 {expected:.2f} m) + LiDAR 앞이 비었다 → "
                       f"({self._bumps[-1][1]:.2f}, {self._bumps[-1][2]:.2f}) 에 보이지 않는 장애물을 기록하고 RECOVER")
        if self._deadlock_enabled():
            self._progress_backoff = (now, self.progress.jittered(.7, "contact"))
            self.stack.clear_goal()
        else:
            self._enter_recover(now)

    def map3d_report(self) -> Dict[str, Any]:
        return self.fusion.report() if self.fusion is not None else {}

    def export_map3d(self, path: str) -> int:
        """3D 지도를 .ply(점유 복셀 중심, 높이별 색) 또는 .npz(로그오즈 전체)로 저장. 반환: 점유 복셀 수."""
        if self.fusion is None:
            return 0
        if path.endswith(".npz"):
            self.fusion.map.save_npz(path)
            return int(self.fusion.map.occupied().sum())
        return self.fusion.map.save_ply(path)

    def _locate_det(self, det: Detection, pixel: Optional[Tuple[float, float]] = None) -> Optional[np.ndarray]:
        """검출(또는 지정 픽셀)이 가리키는 월드 3D 점. 현재 검출이 나온 프레임의 시각·자세·팬틸트와 짝지은 깊이 프레임으로 구한다."""
        if self.fusion is None or self._det_ctx is None:
            return None
        u, v = pixel if pixel is not None else self.fusion.det_pixel(det)
        t, pose, pan, tilt = self._det_ctx
        return self.fusion.locate_pixel(u, v, t, pose, pan, tilt)

    def _range(self, det: Detection, real_h: float, pan: float) -> Tuple[Optional[float], str]:
        """검출까지 수평 거리. LiDAR 가 믿을 만한 값을 주면 그대로(mm 급), LiDAR 가 못 주거나(평면 밖·가림) 크기 추정으로 떨어질 때만
        깊이 영상으로 잰다 — 스테레오(cm 급)가 크기 추정(수십 %)보다 낫다."""
        r, how = estimate_range(self.cam, det, real_h, self._scan, pan, self._dth)
        if how == "lidar" or self.fusion is None or not self.cfg.map3d.use_range:
            return r, how
        pt = self._locate_det(det)
        if pt is None or self._det_ctx is None:
            return r, how
        p = self._det_ctx[1]
        self.stats["range_from_depth"] += 1
        return float(math.hypot(pt[0] - p.x, pt[1] - p.y)), "depth"

    def _depth_guard(self, v: float) -> float:
        """전방 안전: LiDAR 평면에 안 걸리는 장애물(낮은 물체 등)이 차체 앞 통로에 있으면 감속·정지. (LiDAR 가 보는 건 LiDAR 제한이 맡는다)"""
        mc = self.cfg.map3d
        c = self.fusion.front_clearance(self.stack.pose, mc.guard_slow + 0.05)
        if c >= mc.guard_slow:
            return v
        self.stats["depth_guard_ticks"] += 1
        if c <= mc.guard_dist:
            return 0.0
        return v * (c - mc.guard_dist) / (mc.guard_slow - mc.guard_dist)

    def _shot_geometry_ok(self, det: Detection, now: float, pan: float, tilt: float) -> bool:
        """발사 전 3D 확인. (1) 수광센서의 깊이 기반 높이가 그 종류의 기대 높이대 안인가(벽의 하얀 점·반사 같은 오검출 거름)
        (2) 레이저 발광부→수광센서 선분에 3D 지도상 장애물이 끼었나. 깊이로 못 재면 통과(모르는 것을 '나쁨'으로 보지 않는다)."""
        fu, mc = self.fusion, self.cfg.map3d
        if fu is None or self._det_ctx is None or not (mc.height_check or mc.beam_check):
            return True
        sens = det.extra.get("sensors") if det.extra else None
        px: Optional[Tuple[float, float]] = None
        if sens:
            ref = self.aim._last_pt if self.aim._last_pt is not None else (self.cam.cx, self.cam.cy)
            s = min(sens, key=lambda q: math.hypot(q[0] - ref[0], q[1] - ref[1]))
            px = (float(s[0]), float(s[1]))
        pt = self._locate_det(det, px)
        if pt is None:
            self._beam_since = None
            return True
        if mc.height_check and sens:
            lo, hi = mc.target_window_z if det.cls in self.cfg.perception.target_classes else mc.ugv_receiver_z
            if not (lo <= float(pt[2]) <= hi):
                self.stats["height_reject"] += 1
                if now - self._beam_logged > 5.0:
                    self._beam_logged = now
                    self._log(now, f"WARN 수광센서 높이 {pt[2]:.2f} m 가 {det.cls} 기대 높이대 [{lo:.2f}, {hi:.2f}] 밖 → 오검출로 보고 발사 보류")
                return False
        if mc.beam_check:
            org = fu.laser_origin(self.stack.pose, pan, tilt, self.cfg.aim.laser_lateral_m, self.cfg.aim.laser_vertical_m)
            self.stats["beam_checked"] += 1
            if fu.beam_blocked(org, pt) >= mc.beam_min_blocked:
                self._beam_since = now if self._beam_since is None else self._beam_since
                self.stats["beam_blocked_ticks"] += 1
                if now - self._beam_since < mc.beam_timeout_s:
                    return False
                if now - self._beam_logged > 5.0:                       # 오래 막혔다 = 허상일 가능성이 높다 → 통과(M2 를 통째로 잃지 않게)
                    self._beam_logged = now
                    self.stats["beam_override"] += 1
                    self._log(now, f"WARN 사선이 {now - self._beam_since:.1f}s 막힘으로 보이지만 허상으로 보고 발사 허용")
                return True
            self._beam_since = None
        return True

    # ================================================================== 실기 API 상태 (/mil/match, /mil/state, /imu/data; PDF p10~12, p17)
    def _match_api(self, now: float, dt: float, st) -> None:
        """로봇이 직접 알려 주는 경기 단계로 하는 일. 서버(HTTP) 메시지와 별개의 두 번째 증거라 STOP/START 판단은 서버가 우선이다.
          1. 재시작 복원: 팀 노드가 경기 중 다시 실행됐을 때(경기당 최대 3회, p18) 경기 시계를 되살린다.
          2. 일시정지(cmd=pause): 경기 시계를 멈춘다 [가정: freeze_clock_on_pause — 서버가 일시정지 시간을 빼는지 HTTP 문서로 확인].
          3. cmd=report/stop: 서버 STOP 이 유실됐을 때의 보조 정지. start/resume 을 본 적이 있을 때만(이전 경기의 낡은 값 오인 방지).
          4. OLED 상태 표시, 배터리 절전."""
        h = self.cfg.hal
        run = st.match_cmd in ("start", "resume")
        if run:
            self._saw_run = True
        if self.t_start is None and not self.latch.started and not self._restore_tried and run:
            self._try_restore(now)                            # 재시작이면 저장된 시각으로 — 로봇 신호(cmd=start 가 계속 유지됨)보다 먼저
        if (self.t_start is None and st.match_mode == "mission" and st.match_cmd == "start" and not self.latch.started
                and not self.latch.stopped):
            ok, _ = self.latch.accept(ServerMsg("start", data={"src": "robot"}))      # 로봇이 받은 시작 신호 = 팀 시계 0:00 (서버 API 3단계)
            if ok:
                self._on_start(now, "robot_match")
        if self.t_start is not None and not self.stopped:
            if st.paused and h.freeze_clock_on_pause:
                self.t_start += dt
                self._pause_s += dt
                self.stats["paused_s"] = self._pause_s
            if st.control is False:
                self.stats["match_locked_s"] += dt
        if (h.match_cmd_stop_backup and st.match_mode == "mission" and self._saw_run and self.t_start is not None
                and not self.latch.stopped and st.match_cmd in ("report", "stop")):
            ok, _ = self.latch.accept(ServerMsg("stop"))
            if ok:
                self._stop_via_match = True
                self._log(now, f"STOP latched via /mil/match cmd={st.match_cmd} (서버 STOP 없음)")
        if st.battery_pct is not None and st.battery_pct < h.battery_low_pct and not self._batt_warned:
            self._batt_warned = True
            self._log(now, f"WARN 배터리 {st.battery_pct:.0f}% < {h.battery_low_pct:.0f}% — 배터리 교체 불가(PDF p6): 깊이 영상 처리를 멈춘다(지금까지의 3D 지도는 유지)")
            self._depth_off = True
        if now - self._oled_t >= h.oled_period_s:
            self._oled_t = now
            self._show_status(now, st)

    def _show_status(self, now: float, st) -> None:
        """OLED 4줄(21자, 영문·숫자·기호; PDF p13). 현장 디버깅용: 팀·모드 / HP·배터리 / 경기 시각·상태 / 제출·정지 사유."""
        t = 0.0 if self.t_start is None else max(now - self.t_start, 0.0)
        hp = f"{st.hp}/{st.hp_max}" if st.hp is not None else "-/-"
        bat = f"{st.battery_pct:.0f}%" if st.battery_pct is not None else "--"
        l3 = f"SAFE {self._safe.get('reason', '')}" if self.mode == Mode.SAFE_STOP else \
            f"M1{'+' if self.m1_submitted else '-'} M3{'+' if self.m3_submitted else '-'} M6{'+' if self.m6_submitted else '-'}"
        self.robot.show("\n".join([f"{(self.team or '?')[0]}{self.slot or '?'} {self.mode.value}", f"HP {hp} BAT {bat}",
                                   f"{int(t) // 60:02d}:{int(t) % 60:02d} {self.state_name[:12]}", l3]))

    def _on_start(self, now: float, src: str, data: Optional[Dict[str, Any]] = None) -> None:
        """경기 0:00 래치(한 번). 서버 메시지(/me phase=running)와 로봇의 /mil/match cmd=start 중 먼저 오는 쪽이 이긴다.
        서버 문서: '시작 신호를 받은 때가 곧 경기의 0:00' — 로봇 신호가 정확하고 서버 phase 는 폴링 주기만큼 늦다."""
        if data and data.get("restarted"):
            elapsed = self._recovered_elapsed(data)
            if elapsed is None:
                self._log(now, "WARN invalid restart clock ignored; a new zero time was not invented")
                return
            self.t_start = now - elapsed
            self._restore_tried = True
            self.stats["restored"] = True
            self.stats["clock_restore_uncertainty_s"] = float(data.get("clock_uncertainty_s", 0.))
            self._log(now, f"RESTORE server-issued clock {elapsed:.1f}s; navigation pose/anchor are not restored")
            self._persist_start(now)
            return
        # A synthetic HTTP running notification can arrive before _match_api.
        # Recover the persisted clock before overwriting it with a new start.
        st = self.robot.status()
        if (not self._restore_tried and st.match_mode == "mission" and
                st.match_cmd in ("start", "resume")):
            self._try_restore(now)
            if self.stats["restored"]:
                return
        self.t_start = now                                       # 경기 시계는 START를 래치한 시각부터, 한 번만 (R09)
        self._log(now, f"START latched src={src} match_id={self.latch.match_id}")
        self._persist_start(now)

    def _recovered_elapsed(self, data: Dict[str, Any]) -> Optional[float]:
        elapsed, uncertainty = data.get("elapsed_s"), data.get("clock_uncertainty_s", 0.)
        if (data.get("clock_sync") is not True or isinstance(elapsed, bool)
                or not isinstance(elapsed, (int, float)) or not math.isfinite(elapsed)
                or not 0 <= elapsed <= min(self.cfg.mission.t_total, 300.)
                or isinstance(uncertainty, bool) or not isinstance(uncertainty, (int, float))
                or not math.isfinite(uncertainty) or uncertainty < 0):
            return None
        return float(elapsed)

    def _persist_start(self, now: float) -> None:
        p = self.cfg.hal.restart_state_path
        if not p:
            return
        try:
            with open(p, "w", encoding="utf-8") as f:
                elapsed = max(0., now-self.t_start) if self.t_start is not None else 0.
                json.dump({"match_id": self.latch.match_id, "wall_start": time.time()-elapsed,
                           "team": self.team, "slot": self.slot}, f)
        except OSError as e:
            self._log(now, f"WARN 재시작 복원 파일을 못 썼다: {e}")

    def _clear_persist(self) -> None:
        p = self.cfg.hal.restart_state_path
        if p and os.path.exists(p):
            try:
                os.remove(p)
            except OSError:
                pass

    def _try_restore(self, now: float) -> None:
        """START 메시지를 못 받았는데 로봇은 경기 중(start/resume)이다 = 팀 노드가 도중에 다시 실행됐다. 저장해 둔 경기 시작 시각(벽시계)으로
        경기 시계만 복원한다. **M1 명령·표적·객체 집계·SLAM 앵커는 복원하지 못한다** — 서버가 명령을 다시 주는지, SLAM/Nav2 가 따로 살아 있는지는 현장 확인 항목."""
        self._restore_tried = True
        p = self.cfg.hal.restart_state_path
        if not p or not os.path.exists(p):
            return
        try:
            with open(p, "r", encoding="utf-8") as f:
                d = json.load(f)
            age = time.time() - float(d["wall_start"])
        except (OSError, ValueError, KeyError):
            return
        mc = self.cfg.mission
        ident = self.server.identity()
        team = self.team or self.cfg.team or (ident[0] if ident else None)
        slot = self.slot or self.cfg.slot or (ident[1] if ident else None)
        if (d.get("team") is not None and team is not None and d["team"] != team) or (
                d.get("slot") is not None and slot is not None and d["slot"] != slot):
            return
        if not 0.0 <= age <= mc.t_total + mc.t_report:
            return
        self.latch.started = True
        self.latch.match_id = d.get("match_id")
        self.t_start = now - age
        self.stats["restored"] = True
        self._log(now, f"RESTORE 경기 도중 재시작: 경기 시계를 {age:.1f}s 로 복원 (match_id={self.latch.match_id}). 명령·지도는 복원되지 않는다")

    # ================================================================== 서버 메시지
    def _serve(self, now: float) -> None:
        for m in self.server.poll():
            if m.kind == "start" and (m.data or {}).get("restarted") and self._recovered_elapsed(m.data) is None:
                self.stats["msg_rejected"]["invalid_restart_clock"] = self.stats["msg_rejected"].get("invalid_restart_clock", 0) + 1
                self._log(now, "WARN rejected unsynchronized/out-of-range restart clock")
                continue  # Do not latch a bad recovery or prevent a later valid start.
            ok, why = self.latch.accept(m)
            if not ok:
                self.stats["msg_rejected"][why] = self.stats["msg_rejected"].get(why, 0) + 1
                mid = (m.data or {}).get("match_id")
                if self.latch.stopped and m.kind in ("start", "command", "base_command") and mid is not None \
                        and mid != self.latch.match_id:
                    # 정지·보고 중에 '다음 경기'의 메시지가 먼저 도착했다. 버리지 않고 러너가 새 Agent 에 넘긴다(⑨).
                    # (match_id 가 없으면 이전 경기의 늦은 중복과 구분할 수 없어 넘기지 않는다)
                    self.carry_over.append(m)
                    self._log(now, f"msg {m.kind} of match {mid} carried over to the next agent")
                    continue
                self._log(now, f"msg {m.kind} dropped ({why})")
                continue
            if m.kind == "start":
                self._on_start(now, (m.data or {}).get("src", "server"), m.data)
            elif m.kind == "command":
                self._on_command(now, m.text)
            elif m.kind == "base_command":
                obj = (m.data or {}).get("objective")                # 서버 보고값(RED_MAINTENANCE)이 있으면 그것이 정확하다. 없으면 이름 문장 해석
                self.m5_type = schema.parse_objective(obj) if obj else None
                if self.m5_type is None:
                    self.m5_type = parse_base_command(m.text)
                self._log(now, f"M5 command '{m.text[:30]}' objective={obj} -> {self.m5_type}")
                camp, mine = schema.objective_camp(obj) if obj else None, (self.team or self.cfg.team)
                if camp is not None and mine is not None and camp == schema.camp_of(mine):
                    self._log(now, f"WARN M5 목표거점이 우리 진영({camp})이다 — 서버는 상대 진영 거점을 줄 것이라고 했다. 거점 해석을 확인할 것")
            elif m.kind == "stop":
                self._log(now, "STOP latched")
            elif m.kind == "event":
                ev = normalize_event(m.data)
                if ev["type"] in ("game_state", "smog", "base_entered"):
                    self._on_env_event(now, ev)                      # 경기상태·환경 이벤트는 정체성·센서 준비를 기다리지 않는다(STOP 이 늦으면 안 된다)
                elif self.tracker is None or self.team is None:      # 아직 정체성·트래커가 없으면 센서 준비 뒤 처리
                    self._pending_events.append(m.data)
                else:
                    self._on_event(now, m.data)

    def _on_env_event(self, now: float, ev: Dict[str, Any]) -> None:
        """서버 이벤트 중 경기 진행·환경에 관한 것(대시보드 '주요 이벤트': 경기상태 변경·목표거점 진입, 스모그 장치)."""
        t = ev["type"]
        ge = self.stats["game_events"]
        ge[t] = ge.get(t, 0) + 1
        if t == "game_state":
            st = ev.get("state")
            self._log(now, f"EVENT game_state={st}")
            if st == "finished" and not self.latch.stopped:
                # '완료'는 STOP 과 같은 뜻이다(서버가 보낸 것이라 신뢰한다). STOP 메시지와 같은 규칙(match_id·종단 래치)으로 받는다.
                ok, why = self.latch.accept(ServerMsg("stop", data={k: ev[k] for k in ("match_id", "seq") if k in ev}))
                if ok:
                    self._stop_via_event = True
                    self._log(now, "STOP latched via game_state event")
        elif t == "smog":
            on = bool(ev.get("on"))
            if self.smog.force(on) is not None:
                self._on_smog_change(now, on, "server")
            elif not on:
                self.smog.force(None)                                  # 끄는 이벤트 뒤에는 다시 센서 판단에 맡긴다
        elif t == "base_entered":
            self.stats["base_entries"].append((round(now, 2), ev.get("by"), ev.get("base")))
            me = f"{self.team[0]}-UGV-{self.slot}" if self.team and self.slot else None
            self._log(now, f"EVENT base_entered by={ev.get('by')} base={ev.get('base')}")
            if me and ev.get("by") == me:
                self.m5_confirmed_event = now                          # 서버가 내 진입을 확인했다(곧 STOP 이 온다)

    def _drain_events(self, now: float) -> None:
        if self._pending_events and self.tracker is not None and self.team is not None:
            evs, self._pending_events = self._pending_events, []
            for ev in evs:
                self._on_event(now, ev)

    def _on_command(self, now: float, text: str) -> None:
        if self.latch.stopped or self.halt_t is not None:
            return
        ident = self.server.identity()
        self.parser.team = self.team or (ident[0] if ident else None)       # '나머지 상대진영 구역' 같은 표현을 펼치려면 팀이 필요하다
        self.cmd = self.parser.parse(text)
        self._cmd_ver += 1
        self._log(now, f"M1 rule -> {self.cmd.to_payload()} conf={self.cmd.confidence:.2f} {self.cmd.notes}")
        needs_llm = self.llm is not None  # PDF1 M1: when configured, run LLM even for an unambiguous rule parse
        if needs_llm:
            box: Dict[str, Any] = {"done": False, "res": None, "t0": now, "ver": self._cmd_ver}
            self._pending_llm = box

            def work() -> None:
                try:
                    box["res"] = parse_llm_json(self.llm.complete(build_prompt(text), self.cfg.mission.m1_llm_timeout))
                except Exception:
                    box["res"] = None  # bounded rule fallback; never leak provider credentials in logs
                finally:
                    box["done"] = True
            threading.Thread(target=work, daemon=True).start()
        else:
            self._submit_m1(now)

    def _poll_llm(self, now: float) -> None:
        b = self._pending_llm
        if b is None or self.latch.stopped or self.halt_t is not None:
            return
        if b.get("ver") != self._cmd_ver:                 # 그 사이 새 명령이 왔다 → 늦은 응답은 현재 행동을 바꾸지 않는다 (R11)
            self._pending_llm = None
            self._log(now, "M1 LLM result for an old command dropped")
            return
        if b.get("processed") or b.get("expired"):
            return
        if now - b["t0"] > self.cfg.mission.m1_llm_timeout:
            b["expired"] = True
            self._submit_m1(now)
            return
        if b["done"]:
            b["processed"] = True
            self.cmd = merge(self.cmd, b["res"])
            self._log(now, f"M1 merged -> {self.cmd.to_payload()} {self.cmd.notes}")
            self._submit_m1(now)
        elif now - b["t0"] >= self.cfg.mission.m1_llm_timeout:
            b["expired"] = True
            self._submit_m1(now)
        elif now - b["t0"] >= self.cfg.mission.m1_submit_after:
            self._submit_m1(now)  # Send rule evidence now; a timely LLM may replace the complete payload.

    def _submit_m1(self, now: float) -> None:
        if self.cmd is None or self.latch.stopped or self.halt_t is not None:
            return
        payload = schema.m1_payload(self.cmd)
        previous = self.submitter.jobs.get("m1")
        if previous is not None and previous.payload == payload:
            return  # Existing retry/ACK tracking owns an unchanged snapshot.
        if self.submitter.request("m1", payload, now, force=previous is not None, backoff=True):
            self._log(now, "M1 full snapshot requested")

    def _on_event(self, now: float, ev: Dict[str, Any]) -> None:
        me = f"{self.team[0]}-UGV-{self.slot}"
        if ev.get("type") == "target_hit" and self._target_hit_matches(ev):
            self.target_done = True
            self._log(now, "M2 target hit confirmed")
        elif ev.get("type") == "ugv_hit" and ev.get("by") == me:
            v = ev.get("victim")
            self._invuln_until[v] = now + self.cfg.laser.disabled_s
            for tr in self.tracker.tracks:
                if tr.ident == v:
                    tr.hits_by_us += 1
            self._log(now, f"M4 hit {v} friendly={ev.get('friendly')}")
        elif ev.get("type") == "ugv_hit" and ev.get("victim") == me:
            self.locks.on_hit_event(now)           # 하드웨어 상태가 늦어도 먼저 발사를 잠근다 (R08)
            self._log(now, f"I was hit by {ev.get('by')}")

    # ================================================================== 제출
    def _m3_deadline(self) -> Optional[float]:
        if self.t_start is None:
            return None
        mc = self.cfg.mission
        return self.t_start + max(0., min(mc.t_total, 300.) - max(0., mc.m3_deadline_margin_s))

    def _guard_m3_submission(self, now: float) -> None:
        if self.latch.stopped or getattr(self, "halt_t", None) is not None:
            # This runs BEFORE Submitter.update: no queued M1 or M3 may leave
            # on the STOP tick while waiting for _enter_halt to close them.
            self.submitter.close_for_stop(("m6",))
        deadline = self._m3_deadline()
        if self.latch.stopped or (deadline is not None and now >= deadline):
            self.submitter.close_kind("m3")
        elif deadline is not None and "m3" in self.submitter.jobs:
            # /mil/match pause shifts t_start; keep the same match-clock cutoff.
            self.submitter.jobs["m3"].deadline = deadline

    def _submit_m3_if_due(self, now: float, t: float) -> None:
        mc = self.cfg.mission
        self._guard_m3_submission(now)
        deadline = self._m3_deadline()
        if (deadline is None or now >= deadline or self.latch.stopped or not self.team
                or self.server.submission_allowed("m3") is False):
            return
        state = self.submitter.state("m3")
        if state == "inflight" or "m3" in self.submitter.closed:
            return  # Do not race full replacements while an earlier one is in flight.
        final_at = min(mc.t_total, 300.) - max(mc.m3_final_flush_s, mc.m3_deadline_margin_s + mc.submit_ack_timeout_s + 1.)
        final_due = t >= max(0., final_at) and not self._m3_final_requested
        first_due = not self.m3_submitted and t >= min(mc.m3_first_s, max(0., final_at))
        gap = mc.m3_late_gap_s if t >= mc.m3_late_at_s else mc.m3_resubmit_s
        refresh_due = gap > 0 and now - self._m3_last[0] >= gap
        if not (first_due or final_due or refresh_due and self.m3_submitted):
            return
        counts = self._m3_counts()
        changed = counts != self._m3_last[1]
        if not (first_due or final_due or changed or state == "gave_up"):
            return
        if self.submitter.request("m3", schema.m3_payload(counts), now, deadline=deadline,
                                  force=self.m3_submitted, backoff=True):
            self._m3_last = (now, counts)
            self._m3_final_requested |= final_due
            self._log(now, f"M3 full snapshot requested sectors={ {k: sum(v.values()) for k, v in counts.items()} }")

    def _m3_counts(self) -> Dict[str, Dict[str, int]]:
        """M3 제출용 섹터별 객체 수. explore.m3_counting: "ledger"(면 장부, 같은 면 1회) | "omap"(기존 위치 합치기)."""
        if self.cfg.explore.m3_counting == "ledger" and self.coverage.boxes:
            raw = self.coverage.m3_counts(self.omap)
        else:
            raw = self.omap.counts_by_sector()
        if not self.team:
            return {}
        # Replacement semantics: every snapshot contains the complete current
        # evidence for all four scoring sectors, including explicit zero buckets.
        # Counts may decrease when evidence is corrected; this is not max-ever
        # accumulation and never a delta containing only newly found objects.
        sectors = {s: ("COMMON" if s == "CM" else s) for s in schema.scoring_sectors(self.team)}
        out = {s: {cls: 0 for cls in schema.OBJECT_CODES} for s in sectors.values()}
        for sector, classes in raw.items():
            canonical = schema.server_sector(sector)
            if canonical not in sectors:
                continue
            for cls, count in classes.items():
                if cls not in schema.OBJECT_CODES or isinstance(count, bool):
                    continue
                try:
                    n = int(count)
                except (ValueError, TypeError, OverflowError):
                    continue
                if n == count and n > 0:
                    bucket = out[sectors[canonical]]
                    bucket[cls] = min(99, bucket[cls] + n)
        return out

    def _submit_m3(self, now: float) -> None:
        if self.t_start is not None:
            self._submit_m3_if_due(now, now - self.t_start)

    # ================================================================== 정지 절차: HALT → REPORT → DONE (R12)
    def _enter_halt(self, now: float, source: str) -> None:
        """순위 00: 구동 0 · 목표 취소 · 소등 · 세대 증가. 이후 START/목표 명령은 모두 무시된다."""
        self.halt_t, self.stop_source = now, source
        self._set_mode(now, Mode.HALT, source)
        self.stack.clear_goal()
        self.close_deadlock()
        self.avoid.reset()                                   # 회피 중이든 STOP 이 우선: 기동·경유지·위협 상태를 모두 버린다
        self._rc = {}
        self._av_ctx = None
        self.stats["avoid"] = dict(self.avoid.stats)
        self._stop_output()
        self.laser.abort()
        allowed = ["m6"]  # M3 accepts only IN_PROGRESS; legacy m3_at_stop cannot override the rule.
        self.submitter.close_for_stop(allowed)               # STOP 뒤 M1~M5 신규 제출 금지
        self._log(now, f"HALT source={source} m1={self.submitter.state('m1')} m3={self.submitter.state('m3')}")
        self.stats["coverage"] = self.coverage.summary()     # 관측 지도 요약(박스·면 상태·훑은 비율)
        if self.fusion is not None:
            self.stats["map3d"] = self.fusion.report()       # 3D 지도 요약(점유 복셀·시야 범위·깊이↔LiDAR 잔차). 파일은 export_map3d()
        self._set_mode(now, Mode.REPORT, "halt done")        # 순위 01: 정지 출력이 나갔으므로 보고 단계로

    def _prepare_m6(self, now: float) -> None:
        """STOP 관측 증거로 즉시 제출할 순위를 만들고, LLM 정정은 별도 스레드에서 제한 시간만 허용한다."""
        if self.tracker is None:                              # 센서 준비 전에 정지됐다: 증거 없음 → 중간값 순위
            self.tracker = UgvTracker(self.cfg, self.team or "BLUE")
        evs = threat_mod.score(threat_mod.evidence_from_tracks(self.tracker, self.team or "BLUE"))
        box: Dict[str, Any] = {"rank": [e.ident for e in evs], "evs": evs, "llm": None, "t0": now}
        if self.llm is not None:
            lb: Dict[str, Any] = {"done": False, "res": None}

            def work() -> None:
                try:
                    lb["res"] = threat_mod.llm_rank(evs, self.llm, self.cfg.mission.report_llm_wait_s)
                except Exception:
                    lb["res"] = None
                finally:
                    lb["done"] = True
            threading.Thread(target=work, daemon=True).start()
            box["llm"] = lb
        self._m6 = box

    def _mode_report(self, now: float, dt: float) -> None:
        """구동 0 유지 · M6 제출(ACK 또는 보고 기한까지 재시도) · 결과를 성공/거부/미확인으로 구분해 기록."""
        mc = self.cfg.mission
        if self._out_tick != self._tick_no:                   # 정지 진입 틱에는 _enter_halt 가 이미 0 을 냈다
            self._stop_output()
        if self._m6 is None:
            self._prepare_m6(now)
        m6 = self._m6
        assert m6 is not None and self.halt_t is not None
        lb = m6["llm"]
        if lb is not None and lb["done"] and lb["res"] and now - m6["t0"] <= mc.report_llm_wait_s:
            m6["rank"] = lb["res"]
        payload = schema.m6_payload(m6["rank"])
        previous = self.submitter.jobs.get("m6")
        replace = previous is not None and previous.state != "inflight" and previous.payload != payload
        if previous is None or replace:
            # Reporting may end as soon as all six teams submit. Send the
            # available observed-evidence ranking immediately, without waiting
            # eight seconds for LLM. A timely result may correct a pending job.
            evs = m6["evs"]
            self.submitter.request("m6", payload, now, force=replace,
                                   deadline=self.halt_t + mc.t_report, max_tries=10 ** 6)
            self._log(now, f"M6 submit requested ranking={m6['rank']} " +
                      " ".join(f"{e.ident}:{e.score:.2f}(seen={e.seen},hit={e.hits_est:.1f},sec={e.sectors},mv={e.travel:.1f})"
                               for e in evs))
        st = self.submitter.state("m6")
        if st == "acked":
            self.m6_result = "acked"
        elif st == "gave_up" or now - self.halt_t >= mc.t_report:
            self.m6_result = "rejected" if self.submitter.rejections("m6") > 0 else "unconfirmed"
        if self.m6_result is not None:                        # 순위 02: ACK 또는 30 s 마감
            self._log(now, f"M6 result={self.m6_result} tries={self.submitter.tries('m6')}")
            self._clear_persist()                              # 다음 경기의 노드가 이 경기를 '재시작 복원' 대상으로 오인하지 않게
            self._set_mode(now, Mode.DONE, "report finished")

    # ================================================================== 의사결정
    def _unsafe_reason(self, now: float, st) -> Optional[str]:
        """순위 04: 주행을 계속하면 안 되는 이유(없으면 None)."""
        if not self.stack.initialized:
            return "pose_not_ready"
        if not self.sw.fresh(now):
            return "lidar_stale"
        if st.control is False:
            return "match_locked"                             # /mil/match: 경기 모드에서 start/resume 이 아니다 — 주행은 무시되고 조준·발사는 locked (PDF p11·12)
        if self._tilt_unsafe(st):
            return "tilt"                                     # IMU 기울기 초과 (PDF p17: 기울기·충격 감지 시 Twist() 즉시 정지)
        if self.locks.drive_locked(st, now):
            return "drive_locked"
        if now < self._safe_hold_until:
            return "recover_exhausted"
        return None

    def _tilt_unsafe(self, st) -> bool:
        r, p = st.roll_deg, st.pitch_deg
        if r is None and p is None:
            return False
        m, h = max(abs(r or 0.0), abs(p or 0.0)), self.cfg.hal
        if self._tilt_latched:
            if m < h.tilt_clear_deg:
                self._tilt_latched = False
        elif m > h.tilt_stop_deg:
            self._tilt_latched = True
            self.stats["tilt_stops"] += 1
            self._log(self.robot.now(), f"WARN IMU 기울기 roll={r or 0.0:.1f}° pitch={p or 0.0:.1f}° > {h.tilt_stop_deg:.0f}° → 정지")
        return self._tilt_latched

    def _enter_safe(self, now: float, reason: str) -> None:
        if self.mode != Mode.SAFE_STOP:
            self.stats["safe_stops"][reason] = self.stats["safe_stops"].get(reason, 0) + 1
            self._safe = {"reason": reason, "n0": self.sw.n_new, "t0": now, "from": self.mode}
            self.avoid.reset()                                # 재개 시 CLEAR 부터 다시 평가(옛 회피 상태를 맹목적으로 이어가지 않는다)
            self._rc = {}
            if self.laser.firing():
                self.laser.abort()                            # 발사 금지(지원하지 않으면 무시)
            self._set_mode(now, Mode.SAFE_STOP, reason)
        else:
            if reason != self._safe.get("reason"):
                self._safe["reason"] = reason
            if reason in ("lidar_stale", "pose_not_ready"):   # 원인이 계속되는 동안은 '새 표본' 기준을 계속 밀어 둔다
                self._safe["n0"] = self.sw.n_new

    def _decide(self, now: float, dt: float, t: float) -> None:
        mc = self.cfg.mission
        st = self.robot.status()
        # 피격 감지: 잠금값 갱신 + 로그
        if self.locks.update(st, now) or st.hits_taken != self._prev_hits:
            self._log(now, f"hit taken total={st.hits_taken}")
            self._prev_hits = st.hits_taken
        # ---- (04·05) SAFE_STOP: 새 센서 표본이 들어오기 전에는 복귀하지 않고, 이전 상태를 맹목적으로 재개하지 않는다
        reason = self._unsafe_reason(now, st)
        if reason is not None:
            self._enter_safe(now, reason)
            self._stop_output()
            return
        if self.mode == Mode.SAFE_STOP:
            if self.sw.n_new - self._safe.get("n0", 0) < mc.safe_resume_scans:
                self._stop_output()
                return
            self._log(now, f"SAFE_STOP resume after {self._safe.get('reason')} ({now - self._safe.get('t0', now):.1f}s)")
            # Revalidate the mission phase instead of reviving an old aim/path.
            # Resetting GOAL/STAGE to EXPLORE lets avoidance take ownership
            # before M5 arbitration, defeating its existing in_goal contract.
            resume = (Mode.GOAL if self.m5_type is not None and t >= mc.t_base_cmd - 1.0
                      else Mode.STAGE if t >= mc.prestage_at else Mode.EXPLORE)
            self._set_mode(now, resume, "safe resume")
        elif self.mode == Mode.WAIT:
            self._set_mode(now, Mode.EXPLORE, "start")        # (03) START 래치 + 센서 준비됨
        # ---- (08) 발사/틸트 잠금 중에는 진행 중인 발광도 끈다(지원 시)
        if self.laser.firing() and self.locks.weapon_locked(st, now):
            self.laser.abort()
            self._log(now, "laser abort: weapon locked")
        # ---- 적 UGV 회피(안전 계층): EVADE/CLOSE 에서 소유권을 가져간다. 순위: STOP/SAFE_STOP 아래, 물리 안전 제한(마지막 필터) 위, 임무(M5 포함) 위.
        #      단 M5 시간 여유가 없거나 거점 직전 돌진 조건이면 회피는 정지/후퇴하지 않는다(임무 지연 상한) — avoid.step 이 None 을 준다.
        if self._progress_gate(now, dt):
            return
        if self._avoid_gate(now, dt, t):
            return
        escape = self.field_policy.escape_goal(self.stack.grid, self.stack.pose)
        if escape is not None:
            self.stats["own_base_escape_ticks"] = self.stats.get("own_base_escape_ticks", 0) + 1
            self._nav_to(now, dt, escape, self.stack.grid.res, raw=True)
            return
        # ---- (06) M5 최우선. 단, GOAL 에서 시작한 RECOVER 는 끝까지 한다
        if self.m5_type is not None and t >= mc.t_base_cmd - 1.0 and not (
                self.mode == Mode.RECOVER and self._rec.get("from") == Mode.GOAL):
            self._set_mode(now, Mode.GOAL)
            self._mode_goal(now, dt, t)
            return
        # ---- (13·14) RECOVER 진행
        if self.mode == Mode.RECOVER:
            self._mode_recover(now, dt, t)
            return
        # ---- 진행 중인 발사는 끝까지(조준 유지)
        firing = self.laser.firing()
        if self.mode in (Mode.STRIKE, Mode.HUNT) and firing:
            self._run_mode(now, dt, t)
            return
        # ---- 사전 이동
        if t >= mc.prestage_at:
            if self._m2_can_finish_before_stage(now, t):
                self._run_mode(now, dt, t)
                return
            self._set_mode(now, Mode.STAGE)
            self._mode_stage(now, dt, t)
            return
        # ---- 교전 모드 중재
        want = self._arbitrate(now, st)
        if want != self.mode and (now - self._mode_since > 1.2 or want in (Mode.HUNT,) and self.mode == Mode.EXPLORE):
            self._set_mode(now, want)
        self._run_mode(now, dt, t)

    def _run_mode(self, now: float, dt: float, t: float) -> None:
        {Mode.EXPLORE: self._mode_explore, Mode.STRIKE: self._mode_strike, Mode.HUNT: self._mode_hunt,
         Mode.STAGE: self._mode_stage, Mode.GOAL: self._mode_goal}.get(self.mode, self._mode_explore)(now, dt, t)

    # ------------------------------------------------------------------ 중재
    def _snap_face_normal(self, f) -> None:
        """영상으로 추정한 면 법선은 '처음 본 방향'에 끌려 틀리기 쉽다(옆에서 본 면을 정면으로 착각 → 스침각 사격, 접근 목표 점도 틀어짐).
        박스는 격자 정렬 직육면체이므로, LiDAR 군집에서 얻은 장부(CoverageMap)의 면과 위치가 맞으면 그 법선으로 바로잡는다."""
        cov = getattr(self, "coverage", None)
        if cov is None:
            return
        best, bd = None, 0.40
        for b in cov.boxes.values():
            for fr in b.faces:
                d = math.hypot(fr.cx - f.x, fr.cy - f.y)
                if d < bd:
                    best, bd = fr, d
        if best is None:
            return
        if f.vx * best.nx + f.vy * best.ny < math.cos(math.radians(20.0)):
            f.vx, f.vy = float(best.nx), float(best.ny)
            self.stats["normal_snaps"] = self.stats.get("normal_snaps", 0) + 1

    def _opponent_target_face(self, face) -> bool:
        """Class and camp from ground observations in the navigation frame.

        The measured arena is translated into this same nonnegative frame;
        neither a simulator target id nor an unobserved layout is consulted.
        """
        if (face is None or self.cmd is None or self.cmd.target is None
                or self.team not in ("BLUE", "RED") or face.kind != "target"
                or face.n < 2 or not face.votes or face.cls != self.cmd.target
                or not math.isfinite(face.cx) or not math.isfinite(face.cy)
                or not self.geo.inside(face.cx, face.cy)):
            return False
        return face.cx > self.geo.W/2 if self.team == "BLUE" else face.cx < self.geo.W/2

    def _target_hit_matches(self, event) -> bool:
        """Optional device evidence must refer to our selected opponent target.

        The official participant API has no target-hit receipt. Missing camp
        or kind, or an explicit device id we cannot match, cannot authorize
        completion; real deployments use the target's confirmed signal LED.
        """
        face = self._strike.get("face")
        if (not self._opponent_target_face(face)
                or schema.callsign(event.get("by", event.get("shooter"))) != f"{self.team[0]}-{self.slot}"
                or self.laser.shots <= self._strike.get("shots0", 1 << 30)):
            return False
        kind = str(event.get("cls", event.get("kind", ""))).upper()
        kind = {v: k for k, v in schema.TARGET_CODES.items()}.get(kind, kind)
        camp = {"B": "BLUE", "R": "RED", "BLUE": "BLUE", "RED": "RED"}.get(str(event.get("camp", "")).upper())
        if kind != self.cmd.target or camp != ("RED" if self.team == "BLUE" else "BLUE"):
            return False
        target_id = event.get("target_id", event.get("target"))
        if target_id is not None and target_id != getattr(face, "target_id", None):
            return False  # A referee's private layout index is not an observed device id.
        return True

    def _target_detection_matches(self, det, face) -> bool:
        """Reuse the map's measured projection instead of aiming by class alone."""
        if not self._opponent_target_face(face) or self._det_ctx is None:
            return False
        stamp, pose, pan, tilt = self._det_ctx
        located = self.omap.locate(det, pose, Frame(None, stamp, pan, tilt), self._scan, dth=self._dth)
        if located is None:
            return False
        x, y, _, _ = located
        return math.hypot(x-face.x, y-face.y) <= self.cfg.perception.object_merge_dist

    def _target_faces(self):
        if self.cmd is None or self.cmd.target is None or self.target_done or self.target_failed:
            return []
        allf = [f for f in self.omap.find_target(self.cmd.target) if self._opponent_target_face(f)]
        for f in allf:
            self._snap_face_normal(f)
        live = [f for f in allf if not f.led_on]
        if allf and not live and not self.target_done:
            # 상대 진영의 지정 표적 LED가 켜졌다 -> 팀 단위 목표이므로 재사격하지 않는다.
            # E5: 시그널 LED(유효 타격 시 빨간 점등)를 영상으로 연속 확인했다 — 서버 이벤트가 늦거나 없어도 M2 완료를 안다
            self.target_done = True
            self.stats["led_confirmed"] += 1
            self._log(self.robot.now(), "M2 target signal LED lit (visual confirm)")
        return live

    def _hunt_candidates(self, now: float) -> List[Track]:
        out = []
        for tr in self.tracker.enemies(now, max_age=self.uavmon.track_max_age(3.0)):   # E4: 영상이 2 s·5 s 에 한 장이면 그 사이 트랙이 사라지면 안 된다
            if self._hunt_block.get(tr.tid, 0.0) > now:
                continue
            if tr.ident and self._invuln_until.get(tr.ident, 0.0) > now:
                continue
            out.append(tr)
        return out

    def _reserve(self) -> int:
        return 0 if (self.target_done or self.target_failed or self.cmd is None or self.cmd.target is None) else 3

    def _arbitrate(self, now: float, st) -> Mode:
        mc = self.cfg.mission
        can_shoot = not self.locks.weapon_locked(st, now)
        px, py = self.stack.pose.x, self.stack.pose.y
        enemies = self._hunt_candidates(now)
        near_enemy = [e for e in enemies if math.hypot(e.x - px, e.y - py) < 1.6 and now - e.last_cam_t < 1.0]
        faces = self._target_faces()
        hunt_ok = (mc.engage_ugv and can_shoot and st.ammo > self._reserve() and enemies and
                   any(now - e.last_cam_t < 2.0 or math.hypot(e.x - px, e.y - py) < 2.5 for e in enemies))
        if hunt_ok and near_enemy:
            return Mode.HUNT
        if faces and can_shoot and st.ammo > 0:
            if now < self._strike_block_until and self._m2_defer_s < self.cfg.avoid.m2_defer_max_s:
                return Mode.EXPLORE                                  # 회피 직후 쿨다운(같은 적 앞으로 곧장 다시 가지 않는다). 연기 합계 상한을 넘으면 무시
            return Mode.STRIKE
        if hunt_ok:
            return Mode.HUNT
        return Mode.EXPLORE

    # ================================================================== 적 UGV 회피 (안전 계층)
    def _goal_xy(self) -> Optional[Tuple[float, float]]:
        """거점(M5)이 정해졌으면 그 중심, 아니면 스테이징 지점 — 시간 예산(slack) 계산용."""
        try:
            if self.m5_base:
                return self.geo.base_center(self.m5_base)
        except Exception:
            pass
        return self._stage_point()

    def _avoid_goal_flags(self, pose, gxy, dist_goal):
        """Existing mission/avoidance contract, using the current FSM mode."""
        ac = self.cfg.avoid
        in_goal = self.mode in (Mode.GOAL, Mode.STAGE)
        dash_ok = False
        if self.mode == Mode.GOAL and gxy and dist_goal <= ac.dash_dist_m:
            exp = 0.0
            if self._field is not None:
                _, exp = path_exposure([(pose.x, pose.y), gxy], self.cfg.nav.v_cruise,
                                       self._field, self.stack.grid, ac.risk_hit)
            dash_ok = exp <= ac.dash_exposure_s
        return in_goal, dash_ok

    def _threat_update(self, now: float, dt: float, t: float) -> None:
        """위협 보드·위협장·회피 수준을 갱신한다(출력은 `_avoid_gate` 에서). 위협이 없으면 거의 공짜."""
        ac = self.cfg.avoid
        if not ac.enable or self.board is None or self.tracker is None or not self.stack.initialized:
            self._av_ctx = None
            return
        pose = self.stack.pose
        vx, vy = self._v_cmd * math.cos(pose.th), self._v_cmd * math.sin(pose.th)
        self._av_tracks = self.board.update(self.tracker, self._blobs, now, pose,
                                            static_boxes=self.coverage.confirmed_static_footprints(now, self.omap),
                                            use_risk_weights=self._deadlock_enabled())
        if self._deadlock_enabled() and not self.cfg.risk.threaded:
            if self._field is None or now-self._field_t >= 1./max(.1,self.cfg.risk.update_hz):
                self._field_ver += 1
                self._field = build_field(self._av_tracks,self.stack.grid,ac,self.cfg.laser.max_range,now,
                    self.cfg.nav.inflate_hard,self._field_ver,use_visibility=ac.use_visibility,
                    risk_cfg=self.cfg.risk,min_clearance_m=self.cfg.engage.min_clearance_m,
                    body_radius_m=math.hypot(self.cfg.robot.length,self.cfg.robot.width)/2.,
                    hit_history=self._hit_history,static_boxes=self.coverage.confirmed_static_footprints(now,self.omap),
                    ego_velocity=(vx,vy))
                self._field_t=now
        elif self._deadlock_enabled():
            from .risk_worker import RiskFieldWorker
            if self._risk_worker is None:
                self._risk_worker = RiskFieldWorker(self.cfg)
            urgent = self._scan_stamp != self._risk_scan_stamp
            if urgent or now-self._field_t >= 1./max(.1,self.cfg.risk.update_hz):
                self._field_ver += 1
                self._risk_worker.submit(self._av_tracks,self.stack.grid,now,self._field_ver,
                    hit_history=self._hit_history,
                    static_boxes=self.coverage.confirmed_static_footprints(now,self.omap),
                    ego_velocity=(vx,vy),urgent=urgent)
                self._field_t=now
                self._risk_scan_stamp=self._scan_stamp
            version, field = self._risk_worker.latest()
            if version >= 0:
                self._field = field
            if not self._av_tracks and not self._hit_history:
                self._field = None
        elif not self._av_tracks:
            self._field = None
        elif self._field is None or now - self._field_t >= ac.field_period_s:
            import time as _time
            t0 = _time.perf_counter()
            self._field_ver += 1
            self._field = build_field(self._av_tracks, self.stack.grid, ac, self.cfg.laser.max_range, now, self.cfg.nav.inflate_hard,
                                      self._field_ver, use_visibility=ac.use_visibility)
            self._field_t = now
            self.stats["field_builds"] += 1
            self.stats["field_ms"] += 1000.0 * (_time.perf_counter() - t0)
        # 정적 하드 차단·비용: 위협이 있을 때만 0.5 s 주기로
        if self._av_tracks and (self._av_maps[1] is None or now - self._av_maps[0] >= 0.5):
            # 위험 인지 경로는 벽·박스에 바짝 붙지 않게 하드 인플레이션을 조금 더 크게 잡는다(위험을 피해 벽을 타고 가는 경로가 실제로는 LiDAR 제한에 걸려 멈추는 문제)
            extra = 0. if self._deadlock_enabled() else self.cfg.avoid.risk_extra_inflate_m
            b, c = self.stack.grid.cost_maps(self.cfg.nav.inflate_hard + extra, self.cfg.nav.inflate_soft, self.cfg.nav.soft_weight)
            self._av_maps = (now, b, c)
        gxy = self._goal_xy()
        dist_goal = math.hypot(gxy[0] - pose.x, gxy[1] - pose.y) if gxy else 0.0
        kappa, slack = self.avoid.budget(t, dist_goal)
        self._kappa = kappa
        in_goal, dash_ok = self._avoid_goal_flags(pose, gxy, dist_goal)
        blocked, cost = self._av_maps[1], self._av_maps[2]
        ctx = AvoidCtx(now, dt, t, pose, vx, vy, self._scan, self.stack.grid,
                       blocked if blocked is not None else np.zeros(self.stack.grid.L.shape, dtype=bool),
                       cost if cost is not None else np.ones(self.stack.grid.L.shape, dtype=np.float32),
                       self._field, self._av_tracks, self.cfg.nav.v_cruise * ac.aware_speed_scale, slack, in_goal, dash_ok, self._cur_goal)
        self._av_ctx = ctx
        lvl = self.avoid.assess(ctx)
        self.avoid.note_pose(pose, now, safe=(lvl == Level.CLEAR))
        # 복구 방향 힌트를 항법 스택에 알린다(Nav2/내장 모두): 적 쪽으로 돌지 않고, 적이 뒤에 있으면 후진하지 않는다
        tp, back = (None, True)
        if lvl >= Level.CLOSE and not (in_goal and slack <= 0):          # 근거리(CLOSE)에서만: 멀리서부터 회전 방향을 강제하면 거점 앞 적 곁을 지나는 임무와 충돌한다
            tp, back = self.avoid.recover_hint(pose, self._av_tracks, vx, vy, now)
        self.stack.set_hint(tp, back)

    def _avoid_gate(self, now: float, dt: float, t: float) -> bool:
        """EVADE/CLOSE 소유권. 반환 True 면 이번 틱의 구동 출력은 이미 나갔다(임무 모드는 실행하지 않는다)."""
        ctx = self._av_ctx
        if self._deadlock_enabled() and self.posture_state and not self.posture_state.allow_evade:
            if self.avoid._was_owner:
                self.avoid._was_owner = False
                self.avoid.cmd = None
                self.stack.clear_goal()
            return False
        if ctx is None or self.mode in (Mode.HALT, Mode.REPORT, Mode.DONE, Mode.SAFE_STOP, Mode.WAIT):
            return False
        # _threat_update runs before _decide. SAFE_STOP may have cleared in
        # between, so refresh only mission flags without re-consuming tracks
        # or assessing the threat twice in one tick.
        gxy = self._goal_xy()
        distance = math.hypot(gxy[0]-ctx.pose.x, gxy[1]-ctx.pose.y) if gxy else 0.0
        ctx.in_goal, ctx.dash_ok = self._avoid_goal_flags(ctx.pose, gxy, distance)
        cmd = self.avoid.step(ctx)
        if self.avoid.acquired:
            self.stats["avoid_acquired"] += 1
            self._on_avoid_acquire(now)
        if self.avoid.released:
            self._on_avoid_release(now)
        if self.mode in (Mode.STRIKE, Mode.HUNT) or self._strike.get("face") is not None:
            if cmd is not None or now < self._strike_block_until:
                self._m2_defer_s += dt
                self.stats["m2_deferred_s"] = round(self._m2_defer_s, 2)
        if cmd is None:
            return False
        self.stats["avoid_ticks"] += 1
        pose = self.stack.pose
        m = self.avoid.main[1] if self.avoid.main else None
        pan = 0.0
        if cmd.pan is not None:
            pan = clamp(cmd.pan, -self.cfg.robot.pan_limit, self.cfg.robot.pan_limit)
        elif m is not None:
            pan = clamp(math.degrees(m["beta"]), -self.cfg.robot.pan_limit, self.cfg.robot.pan_limit)    # 위협 쪽을 보며 팀을 확인
        if cmd.kind == "CARROT" and cmd.path:
            carrot = carrot_point(cmd.path, pose, self.cfg.avoid.carrot_m)
            self._nav_to(now, dt, carrot, 0.20, pan=pan, speed=self.cfg.avoid.aware_speed_scale, raw=True)
        else:
            self._apply(cmd.v, cmd.w, pan, 0.0, dt, limit=True)
        return True

    def _on_avoid_acquire(self, now: float) -> None:
        """임무 → 회피 소유권 이전: 옛 목표·속도를 정리하고(Nav2 목표 취소 + 늦게 오는 속도 폐기), 사격·관측 중이면 중단한다."""
        self.epoch += 1
        self.stack.clear_goal()
        self._rc = {}
        if self.laser.firing():
            self.laser.abort()
        self.aim.reset()
        self._ex_cand, self._ex_phase = None, "travel"
        self._look = None
        self._spin_active = False
        self._cur_goal = None
        if self._strike.get("phase") == "aim":
            self._strike["phase"] = "nav"
        self._log(now, f"AVOID acquire level={self.avoid.effective.name} main={None if self.avoid.main is None else (round(self.avoid.main[1]['dc'], 2), self.avoid.main[0].tid)}")

    def _on_avoid_release(self, now: float) -> None:
        """회피 → 임무 복귀: 옛 목표를 이어가지 않고 새로 정하게 한다. 지정표적은 쿨다운 뒤 재접근(조준부터 다시)."""
        self.epoch += 1
        self.stack.clear_goal()
        self._rc = {}
        self._cur_goal = None
        self._ex_cand, self._ex_phase = None, "travel"
        self.aim.reset()
        if self._strike.get("face") is not None:
            self._strike["phase"] = "nav"
            self._strike["goal"] = None
        if self._m2_defer_s < self.cfg.avoid.m2_defer_max_s:
            self._strike_block_until = now + self.cfg.avoid.strike_cooldown_s
        self._log(now, "AVOID release -> mission resume")

    @staticmethod
    def _path_remainder(path, pose) -> List[Tuple[float, float]]:
        """경로에서 현재 위치에 가장 가까운 점부터의 나머지(현재 위치를 앞에 붙인다)."""
        bi, bd = 0, 1e18
        for i, p in enumerate(path):
            d = (p[0] - pose.x) ** 2 + (p[1] - pose.y) ** 2
            if d < bd:
                bi, bd = i, d
        return [(pose.x, pose.y)] + list(path[bi + 1:]) if bi + 1 < len(path) else [(pose.x, pose.y), path[-1]]

    def _path_cost(self, path, w_risk: float, speed: float, route_cost=None) -> float:
        """현재 위협장 기준 경로 비용 = 길이 + w_risk·위험적분(risk·초). 적 예측 점유(body)를 지나면 무한대."""
        fld, grid = self._field, self.stack.grid
        v = self.cfg.nav.v_cruise * self.cfg.avoid.aware_speed_scale * speed
        if self._deadlock_enabled() and route_cost is not None:
            # Use the same finite grid costs and physical masks as risk A*.
            # Otherwise old unknown-space shortcuts can win route hysteresis.
            total, elapsed = 0., 0.
            for (ax, ay), (bx, by) in zip(path[:-1], path[1:]):
                seg = math.hypot(bx-ax, by-ay)
                elapsed += seg/max(v,1e-6)
                iy,ix = grid.to_cell(bx,by)
                if not grid.in_bounds(iy,ix) or self._av_ctx.blocked[iy,ix]:
                    return math.inf
                cost = float(route_cost[iy,ix])
                if fld is not None:
                    k=fld.layer(elapsed)
                    if fld.body_at(elapsed,iy,ix): return math.inf
                    cost += w_risk*fld.risk[k][iy,ix] + self.cfg.avoid.w_close*fld.close[k][iy,ix]
                total += seg*cost
            return total
        length, t = 0.0, 0.0
        for (ax, ay), (bx, by) in zip(path[:-1], path[1:]):
            seg = math.hypot(bx - ax, by - ay)
            length += seg
            t += seg / max(v, 1e-6)
            if fld is not None:
                iy, ix = grid.to_cell(bx, by)
                if grid.in_bounds(iy, ix) and fld.body_at(t,iy,ix):
                    return math.inf
        ri = 0.0
        if fld is not None:
            ri, _ = path_exposure(path, v, fld, grid, self.cfg.avoid.risk_hit)
        return length + w_risk * ri

    def _risk_active(self) -> bool:
        return (self.cfg.avoid.enable and self._field is not None and self._av_ctx is not None
                and (self._deadlock_enabled() or self.avoid.effective >= Level.AWARE))

    def _risk_carrot(self, now: float, goal: Tuple[float, float], speed: float) -> Optional[Tuple[float, float]]:
        """AWARE 이상에서 임무 목표까지의 *위험 인지* 경로를 추정하고 그 위 안전 경유지를 돌려준다. 실제 주행 경로는 Nav2/A* 가 계획한다.
        경유지는 가까워지기 전까지 고정(Nav2 목표 재전송 폭주 방지). 경로를 못 구하면 None(=원래 목표를 그대로 보낸다)."""
        ac, ctx = self.cfg.avoid, self._av_ctx
        pose = self.stack.pose
        rc = self._rc
        same = rc.get("goal") is not None and math.hypot(goal[0] - rc["goal"][0], goal[1] - rc["goal"][1]) < 0.10
        period = self.progress.jittered(ac.plan_period_s,"replan") if self._deadlock_enabled() else ac.plan_period_s
        if same and now - rc.get("t", -1e9) < period and rc.get("carrot") is not None:
            if math.hypot(rc["carrot"][0] - pose.x, rc["carrot"][1] - pose.y) > ac.carrot_hold_m:
                return rc["carrot"]
        # 시간 여유 κ: 여유가 없으면(slack<=0, 커밋) 위험 가중을 거의 끄고 곧장 간다 — 위험을 피해 돌아가다 M5 를 놓치는 것보다 낫다. 하드 안전(LiDAR·body)은 유지.
        slack = ctx.slack_s if ctx is not None else 1e9
        w_eff = ac.w_risk * (0.05 if slack <= 0 else (0.35 + 0.65 * self._kappa))
        if self._deadlock_enabled():
            w_eff = self._risk_weight()
        route_cost, passing = self._right_hand_cost(goal, self._observed_space_cost(ctx.cost), now)
        res = risk_astar.plan(ctx.grid, ctx.blocked, route_cost, (pose.x, pose.y), goal, self.cfg.nav.v_cruise * ac.aware_speed_scale * speed,
                              self._field, w_eff, ac.w_close, ac.risk_hit, 15000)
        if self._deadlock_enabled() and self.progress_state and self.progress_state.stage >= 1:
            direct = risk_astar.plan(ctx.grid,ctx.blocked,route_cost,(pose.x,pose.y),goal,
                    self.cfg.nav.v_cruise*speed,self._field,0.,0.,ac.risk_hit,15000)
            if direct is not None and (res is None or self._path_cost(direct.path,w_eff,speed,route_cost)<self._path_cost(res.path,w_eff,speed,route_cost)):
                res=direct
        self.stats["risk_plans"] += 1
        if res is None or len(res.path) < 2:
            self.stats["risk_plan_fail"] += 1
            self._rc = {}
            return None
        if res.length <= ac.carrot_m + 0.2:
            self._rc = {"goal": goal, "t": now, "carrot": None, "path": res.path}
            return None                                        # 가까우면 원래 목표(도착 판정이 의미를 가진다)
        # 경로 유지(히스테리시스): 새 경로가 옛 경로(남은 부분)보다 충분히 싸지 않으면 옛 경로를 쓴다 — 매 0.5 s 마다 경로가 뒤집혀 제자리에서 흔들리는 것을 막는다
        old_path = rc.get("path") if same else None
        if old_path and not passing:
            ox = self._path_remainder(old_path, pose)
            old_s = self._path_cost(ox, w_eff, speed, route_cost)
            new_s = self._path_cost(res.path, w_eff, speed, route_cost)
            if old_s < math.inf and old_s <= ac.route_keep_ratio * new_s + ac.route_keep_abs:
                res = risk_astar.PlanResult(ox, 0.0, 0.0, 0.0, 0.0)
                self.stats["route_kept"] = self.stats.get("route_kept", 0) + 1
        car = carrot_point(res.path, pose, ac.carrot_m)
        old = rc.get("carrot")
        if old is not None and same and math.hypot(old[0] - car[0], old[1] - car[1]) < 0.5 and math.hypot(old[0] - pose.x, old[1] - pose.y) > ac.carrot_hold_m:
            car = old
        else:
            self.stats["carrot_sent"] += 1
        self._rc = {"goal": goal, "t": now, "carrot": car, "path": res.path}
        return car

    # ================================================================== 공통 도우미
    def _apply(self, v: float, w: float, pan: float, tilt: float, dt: float, limit: bool = False) -> None:
        r = self.cfg.robot
        self._count_output()
        v, w = self._dynamic_clearance(v, w, dt)
        if limit and self._scan is not None and not self._deadlock_enabled():
            v, w = self.stack.limit(self._scan, v, w, self.robot.now())
        if v > 0 and self.fusion is not None and self.cfg.map3d.guard_enable and self.stack.initialized:
            v = self._depth_guard(v)
        v = self._v_cmd + clamp(v - self._v_cmd, -r.a_max * dt * 1.6, r.a_max * dt * 1.6)
        w = self._w_cmd + clamp(w - self._w_cmd, -8.0 * dt, 8.0 * dt)
        if self._deadlock_enabled() and self._scan is not None:
            # A deceleration ramp must never restore motion after a LiDAR
            # wall stop, including already-limited Navigator/Nav2 commands.
            v, w = self.stack.limit(self._scan, v, w, self.robot.now())
        v, w = self._dynamic_clearance(v, w, dt)
        self._v_cmd, self._w_cmd = v, w                      # 상태는 '원하는 실제 속도'로 유지(램프 기준)
        vc, wc = self.diff.command(v, w)                     # 로봇에는 슬립·편차를 보상하고 바퀴 한계를 지킨 지령
        unguarded = (vc, wc)
        vc, wc = self.field_policy.limit_velocity(self.stack.pose, vc, wc, dt)
        if (vc, wc) != unguarded:
            if vc == 0.:
                self._v_cmd = 0.
            if wc == 0.:
                self._w_cmd = 0.
        self.stats["own_base_blocked_commands"] = self.field_policy.blocked_commands
        self.robot.set_velocity(vc, wc)
        self.robot.set_pan_tilt(pan, tilt)
        self.stats["wheel_limited"] = self.diff.n_limited

    def _count_output(self) -> None:
        """한 틱에 구동·팬틸트 출력은 한 번이다. 두 번 나가면 두 상태가 동시에 명령한 것이므로 센다."""
        if self._out_tick == self._tick_no:
            self.stats["multi_output"] += 1
        self._out_tick = self._tick_no

    def _stop_output(self) -> None:
        """감속 램프 없이 즉시 0 속도(SAFE_STOP·HALT·REPORT). 팬틸트는 그대로 둔다."""
        self._count_output()
        self._v_cmd = self._w_cmd = 0.0
        self.robot.set_velocity(0.0, 0.0)

    def _nav_to(self, now: float, dt: float, goal: Tuple[float, float], tol: float, pan: float = 0.0,
                speed: float = 1.0, raw: bool = False):
        """임무 계층의 이동 요청. 경로계획은 항상 스택(Nav2/내장 A*)이 한다. 적이 가까우면(AWARE 이상) 위험 인지 경로 추정기로 안전 경유지를
        골라 *그 경유지*를 스택에 보낸다(raw=True 면 호출자가 이미 경유지를 준 것)."""
        if self.smog.active:
            speed *= self.cfg.event.smog_speed_scale                 # E3: 스모그 중 감속(LiDAR·카메라가 흐려진다)
        if self._deadlock_enabled() and not raw:
            self._progress_goal = tuple(goal)
        routed = False
        if not raw and self._risk_active():
            car = self._risk_carrot(now, goal, speed)
            if car is not None:
                goal, tol, routed = car, max(tol, 0.20), True
            if not self._deadlock_enabled() or not self.posture_state or self.posture_state.posture.value != 'ENGAGE_OK':
                speed *= self.cfg.avoid.aware_speed_scale
        self.stack.set_goal(goal, tol, speed)
        out = self.stack.step(self._scan, now, dt)
        if routed and out.status == "reached":
            out = dataclasses.replace(out, status="moving")          # 경유지에 닿은 것이지 임무 목표에 닿은 것이 아니다
        if self._deadlock_enabled() and not raw and out.status == "reached":
            self._progress_arrived = True
        self._apply(out.v, out.w, pan, 0.0, dt)
        self._track_block(now, out)
        return out

    # ------------------------------------------------------------------ RECOVER (순위 13·14)
    def _track_block(self, now: float, out) -> None:
        """항법이 오래 막히면 RECOVER. LiDAR 안전 제한은 그동안에도 계속 적용된다."""
        mc = self.cfg.mission
        if self._deadlock_enabled():
            return  # The single ProgressMonitor owns mission stall recovery.
        if out.status != "blocked" or now < self._rec_cooldown:
            self._blocked_since = None
            return
        if self._blocked_since is None:
            self._blocked_since = now
        elif now - self._blocked_since >= mc.recover_after_s and self.mode in (
                Mode.EXPLORE, Mode.STRIKE, Mode.HUNT, Mode.STAGE, Mode.GOAL):
            self._enter_recover(now)

    def _enter_recover(self, now: float) -> None:
        mc = self.cfg.mission
        recent = [x for x in self._rec_hist if now - x < mc.recover_window_s]
        self._rec_hist.clear()
        self._rec_hist.extend(recent)
        if len(recent) >= mc.recover_max:                     # 같은 막힘으로 무한 루프 금지 → SAFE_STOP 후 잠시 쉰다
            self._rec_hist.clear()
            self._safe_hold_until = now + mc.recover_hold_s
            self._log(now, f"RECOVER exhausted ({mc.recover_max} in {mc.recover_window_s:.0f}s)")
            self._enter_safe(now, "recover_exhausted")
            return
        self._rec_hist.append(now)
        self.stats["recoveries"] += 1
        sc = self._scan
        rear_ok = sc is None or sc.min_in_sector(math.pi, math.radians(40)) > 0.30
        left = sc.min_in_sector(math.radians(70), math.radians(35)) if sc else 1.0
        right = sc.min_in_sector(-math.radians(70), math.radians(35)) if sc else 1.0
        origin = self.mode
        turn = 1.0 if left >= right else -1.0
        if self.cfg.avoid.enable and self.board is not None:
            p = self.stack.pose
            tp, back = self.avoid.recover_hint(p, self._av_tracks, self._v_cmd * math.cos(p.th), self._v_cmd * math.sin(p.th), now)
            if tp is not None:
                turn = tp                                           # 적 쪽으로 돌지 않는다
            rear_ok = rear_ok and back                              # 적이 뒤에 있으면 후진하지 않는다
        self._rec = {"from": origin, "t0": now, "turn": turn, "rear_ok": rear_ok}
        self._set_mode(now, Mode.RECOVER, f"blocked {now - (self._blocked_since or now):.1f}s from {origin.value}")

    def _mode_recover(self, now: float, dt: float, t: float) -> None:
        mc, R = self.cfg.mission, self._rec
        back = mc.recover_back_s if R.get("rear_ok") else 0.0
        el = now - R["t0"]
        if el < back:
            self._apply(-0.10, 0.0, 0.0, 0.0, dt, limit=True)      # 후진도 매 틱 LiDAR 후방 안전 제한을 거친다(진입 때 한 번만 보지 않는다)
        elif el < back + mc.recover_turn_s:
            self._apply(0.0, 0.9 * R["turn"], 0.0, 0.0, dt, limit=True)
        else:                                                 # 새 경로로 다시 계획: 목표는 현재 단계 기준으로 재생성
            self._rec_cooldown = now + 2.0
            origin = R.get("from")
            self._log(now, f"RECOVER done ({el:.1f}s) -> {origin.value if origin else 'EXPLORE'}")
            self._set_mode(now, Mode.GOAL if origin == Mode.GOAL else Mode.EXPLORE, "recovered")
            self._apply(0.0, 0.0, 0.0, 0.0, dt)

    def _sweep_pan(self, now: float, amp: float = 35.0, rate: float = 0.9) -> float:
        return amp * math.sin(rate * now)

    def _best_det(self, pred) -> Optional[Detection]:
        c = [d for d in self._dets if pred(d)]
        if not c:
            return None
        return min(c, key=lambda d: abs(d.cx - self.cam.cx) - 20 * d.conf)

    def _fire_conf_ok(self, det: Detection) -> bool:
        """E7: 스모그 중에는 검출이 흐려 오인식이 늘므로 더 높은 신뢰도일 때만 쏜다(8발을 허공·오표적에 쓰지 않게)."""
        return (not self.smog.active) or det.conf >= self.cfg.event.smog_fire_conf

    def _wrong_target_in_line(self, aim_det: Detection, now: float, pan: float) -> bool:
        """E6: 사선에 '우리 것이 아닌' 지정표적(다른 팀에 지정된 표적)이 우리 표적보다 앞이거나 비슷한 거리에 있으면 True.
        다른 팀 표적을 맞히면 그 팀에 가점이 가고 우리는 탄만 쓴다(멘토링: '남 좋은 일')."""
        ev = self.cfg.event
        if not ev.wrong_target_guard or self.cmd is None or self.cmd.target is None:
            return False
        guard = math.radians(self.cfg.mission.friendly_fire_guard_deg)
        tb = self.cam.bearing(aim_det.cx)
        r_t, _ = self._range(aim_det, 0.40, pan)
        for d in self._dets:
            if d is aim_det or d.cls == self.cmd.target or d.cls not in self.cfg.perception.target_classes:
                continue
            if d.conf < self.cfg.perception.conf_min:
                continue
            if abs(self.cam.bearing(d.cx) - tb) > guard + math.atan2(d.w / 2, self.cam.f):
                continue
            r_d, _ = self._range(d, 0.40, pan)
            if r_t is None or r_d is None or r_d < r_t + 0.3:
                self.stats["wrong_target_inhibit"] += 1
                if now - self._wrong_logged > 5.0:
                    self._wrong_logged = now
                    self._log(now, f"WARN 사선에 다른 팀 표적({d.cls})이 있어 발사 보류")
                return True
        return False

    def _may_fire(self, now: float) -> bool:
        """(08) 발사 전 조건: 발사/틸트 잠금이 없고, 검출이 신선해야 한다."""
        return not self.weapon_locked(now) and self.vision_fresh(now)

    def _friend_in_line(self, aim_det: Detection, now: float, pan: float) -> bool:
        """사격선 근처에 (표적이 아닌) UGV가 있으면 True — 아군 오사 방지.
        소등 API 가 없으면(robot.can_abort_laser=False) 발사 뒤에 끌 수 없으므로 보호 각을 두 배로 넓힌다 (R07)."""
        guard = math.radians(self.cfg.mission.friendly_fire_guard_deg) * (1.0 if self.robot.can_abort_laser else 2.0)
        tb = self.cam.bearing(aim_det.cx)
        r_t, _ = self._range(aim_det, 0.32, pan)
        for d in self._dets:
            if d is aim_det or d.cls != "UGV":
                continue
            if abs(self.cam.bearing(d.cx) - tb) > guard + math.atan2(d.w / 2, self.cam.f):
                continue
            r_d, _ = self._range(d, 0.32, pan)
            if r_t is None or r_d is None or r_d < r_t + 0.3:
                return True
        return False

    # ================================================================== EXPLORE
    def _known_cubes(self, now: float):
        if now - self._cubes_cache[0] > 1.5:
            self._cubes_cache = (now, self.cubefinder.find(self.stack.grid))
        return self._cubes_cache[1]

    def _mode_explore(self, now: float, dt: float, t: float) -> None:
        if self.cfg.explore.strategy == "lattice":
            self._mode_explore_lattice(now, dt, t)                  # 기존 방식(비교 기준선으로 보존)
        else:
            self._mode_explore_online(now, dt, t)

    def _mode_explore_online(self, now: float, dt: float, t: float) -> None:
        """M3 온라인 탐색: 프런티어 + 미확인 면 NBV 후보를 정보/시간으로 비교해 다음 시점을 고르고, 거기까지는 기존 Nav2/A*(_nav_to)가 간다.
        관측 방식은 look(한 방향) 또는 spin(360°). '방문'이 아니라 CoverageMap 이 기록한 '실제로 볼 수 있었던 관측'이 완료 기준이다."""
        pose, ec, mc = self.stack.pose, self.cfg.explore, self.cfg.mission
        # ---- 관측 단계
        if self._ex_phase == "observe" and self._ex_cand is not None:
            c = self._ex_cand
            el = now - self._ex_obs_t0
            done = False
            pending_pictures = [f for k in c.faces if (f := self.coverage.get_face(k)) is not None
                                and self.coverage.picture_pending(f)]
            if c.mode == "spin" and mc.spin_w > 0:
                self._ex_spin_acc += abs(wrap(pose.th - self._ex_spin_prev))
                self._ex_spin_prev = pose.th
                self._apply(0.0, mc.spin_w, 0.0, 0.0, dt)
                done = self._ex_spin_acc >= 2 * math.pi - 0.25
            else:
                if pending_pictures:
                    # Arrival tolerance can make a different cube closer than
                    # the planned face. Keep the NBV's subject, not the nearest
                    # incidental picture behind the intended camera direction.
                    f = next((f for f in pending_pictures if f.key == c.primary_face), None)
                    if f is None:
                        f = min(pending_pictures, key=lambda f: abs(wrap(
                            math.atan2(f.cy-pose.y, f.cx-pose.x) - c.heading)))
                    c.primary_face = f.key
                    c.heading = math.atan2(f.cy-pose.y, f.cx-pose.x)
                want = wrap(c.heading - pose.th)
                pan = 0.0 if pending_pictures else self._sweep_pan(now, 20.0, 1.5)
                self._apply(0.0, clamp(2.2 * want, -1.2, 1.2), pan, 0.0, dt)
                if pending_pictures:
                    if self._ex_aligned_face != c.primary_face:
                        self._ex_aligned_face = c.primary_face
                        self._ex_aligned_since = None
                    if abs(want) >= 0.2:
                        self._ex_aligned_since = None
                    elif self._ex_aligned_since is None:
                        self._ex_aligned_since = now
                    # Turning and travel are not a stable photo observation.
                    done = (self._ex_aligned_since is not None
                            and now - self._ex_aligned_since >= ec.m3_observe_s)
                else:
                    done = el >= ec.observe_look_s and abs(want) < 0.2
            if not pending_pictures and not done and el >= 0.8 and c.faces and all(self._face_views(k) > self._ex_views0.get(k, 0) for k in c.faces) and c.gain_u <= 0:
                done = True                                           # 목표 면을 모두 봤다 — 회전을 끝까지 하지 않고 일찍 끝낸다
            capped = self._deadlock_enabled() and self._dwell_expired(now)
            if done or el > 12.0 or capped:
                self.coverage.mark_arrival([k for k in c.faces if self._face_views(k) <= self._ex_views0.get(k, 0)])
                if not capped:
                    self.explorer.note_observation(c, now)
                else:
                    self.explorer.mark_bad(c.vp, now)
                    self.stats["exposed_observation_deferred"] = self.stats.get("exposed_observation_deferred", 0) + 1
                self._ex_cand, self._ex_phase = None, "travel"
                self._ex_sel_t = -1e9
            return
        # ---- 후보 선택(2 s 주기 또는 목표가 없을 때)
        if now - self._ex_sel_t >= 2.0 and (self._ex_cand is None or now - self._ex_goal_t0 >= ec.min_goal_commit_s):
            self._ex_sel_t = now
            blocked, cost = self.coverage.planning_cost_maps(self.stack.grid)
            # M3 always covers the opponent's three sectors plus COMMON.
            # M1's target-search sectors are a separate mission instruction.
            cands = self.explorer.build(pose, self.stack.grid, blocked, now, current=self._ex_cand, team=self.team)
            if self._deadlock_enabled():
                cands = [c for c in cands if self.progress.eligible(self._goal_key(c.vp), now)]
            ac = self.cfg.avoid
            ranked = self.explorer.evaluate(cands, pose, self.stack.grid, blocked, cost, t, self._stage_point(),
                                            self._field, self._risk_weight(), ac.w_close)
            if self._deadlock_enabled():
                ranked.sort(key=lambda c: (-round(c.utility, 6), self.progress.tie_break(self._goal_key(c.vp))))
            choice = self.explorer.choose(ranked, self._ex_cand)
            if choice is not None and (self._ex_cand is None or choice is not self._ex_cand):      # choose() 는 같은 시점이면 현재 객체를 그대로 돌려준다
                self._ex_cand, self._ex_goal_t0 = choice, now
                self.stack.clear_goal()
                self._log(now, f"EXPLORE pick {choice.kind}/{choice.mode} vp=({choice.vp[0]:.2f},{choice.vp[1]:.2f}) gain={choice.gain:.2f} "
                               f"T={choice.t_move:.1f}+{choice.t_obs:.1f}s faces={len(choice.faces)}")
            elif choice is None:
                self._ex_cand = None
                self._log(now, f"EXPLORE no_candidate {self.explorer.diagnostics['last_evaluate']}")
        if self._ex_cand is None:                                    # 할 일이 없으면(또는 시간 안에 못 끝내면) 스테이징 지점으로 미리 간다
            sp = self._stage_point()
            if math.hypot(sp[0] - pose.x, sp[1] - pose.y) > 0.3:
                self._nav_to(now, dt, sp, 0.25, pan=self._sweep_pan(now, 35.0, 1.4))
            else:
                self._apply(0.0, 0.0, self._sweep_pan(now, 35.0, 1.0), 0.0, dt)
            return
        # ---- 이동 단계(경로는 기존 Nav2/A* 가 계산)
        c = self._ex_cand
        route_goal, intermediate = self.explorer.route_goal(c, pose, self.stack.grid)
        if intermediate and math.hypot(route_goal[0] - pose.x, route_goal[1] - pose.y) < 0.05:
            self.explorer.mark_bad(c.vp, now)
            self._ex_cand, self._ex_sel_t = None, -1e9
            self.stack.clear_goal()
            self._apply(0.0, 0.0, self._sweep_pan(now, 25.0, 1.3), 0.0, dt)
            return
        out = self._nav_to(now, dt, route_goal, 0.12 if intermediate else 0.18,
                           pan=self._sweep_pan(now, 25.0, 1.3), raw=intermediate)
        if intermediate and out.status == "reached":
            return
        if out.status == "reached":
            if c.mode == "spin" and not self._spin_clear(pose):
                # 좁은 곳(상자에 바짝 붙은 시점)에서 제자리 회전은 스키드 조향의 밀림 때문에 상자를 친다 → 한 방향 관측으로 바꾼다
                c.mode = "look"
                self.stats["spin_to_look"] = self.stats.get("spin_to_look", 0) + 1
                tgt = None
                keys = ([c.primary_face] if c.primary_face in c.faces else []) + c.faces
                for k in keys:
                    b = self.coverage.boxes.get(k[0])
                    if b is not None:
                        tgt = (b.faces[k[1]].cx, b.faces[k[1]].cy)
                        break
                if tgt is not None:
                    c.heading = math.atan2(tgt[1] - pose.y, tgt[0] - pose.x)
            self._ex_phase, self._ex_obs_t0 = "observe", now
            self._ex_aligned_since, self._ex_aligned_face = None, None
            self._ex_spin_acc, self._ex_spin_prev = 0.0, pose.th
            self._ex_views0 = {k: self._face_views(k) for k in c.faces}
            self.stack.clear_goal()
        elif not self._deadlock_enabled() and (out.status == "blocked" or now - self._ex_goal_t0 > ec.goal_timeout_s):
            self.explorer.mark_bad(c.vp, now)
            self._ex_cand = None
            self._ex_sel_t = -1e9

    def _spin_clear(self, pose, r: float = None) -> bool:
        """제자리 360° 회전을 해도 되는 여유가 있나(몸 둘레 r m 안에 점유 칸이 없나). 기본 r 은 explore.spin_clear_m."""
        r = self.cfg.explore.spin_clear_m if r is None else r
        grid = self.stack.grid
        try:
            occ = grid.occupied()
            iy, ix = grid.to_cell(pose.x, pose.y)
            k = int(math.ceil(r / grid.res))
            y0, y1, x0, x1 = max(iy - k, 0), min(iy + k + 1, occ.shape[0]), max(ix - k, 0), min(ix + k + 1, occ.shape[1])
            sub = occ[y0:y1, x0:x1]
            yy, xx = np.mgrid[y0:y1, x0:x1]
            return not bool(np.any(sub & (((yy - iy) ** 2 + (xx - ix) ** 2) * grid.res ** 2 <= r * r)))
        except Exception:
            return True

    def _face_views(self, key: Tuple[int, int]) -> int:
        b = self.coverage.boxes.get(key[0])
        return b.faces[key[1]].views if b is not None else 10 ** 6    # 박스가 사라졌으면(이동) 충족으로 본다

    def _mode_explore_lattice(self, now: float, dt: float, t: float) -> None:
        """박스 면 단위 탐색(FaceTour) + 박스가 아직 모를 때는 격자 시점 순회(ViewpointTour).
        face 시점에 도착하면 그 면을 향해 ~1 s 정지·관측한다. 격자 시점에서는 제자리 360° 회전."""
        pose = self.stack.pose
        cfgm = self.cfg.mission
        if self._spin_active:
            self._spin_acc += abs(wrap(pose.th - self._spin_prev))
            self._spin_prev = pose.th
            self._apply(0.0, cfgm.spin_w, 0.0, 0.0, dt)
            if self._spin_acc >= 2 * math.pi - 0.25:
                self._spin_active = False
                self.tour.mark_done()
            return
        if self._look is not None:                       # 면을 향해 정렬하고 잠시 관측
            L = self._look
            want = wrap(math.atan2(L["fc"][1] - pose.y, L["fc"][0] - pose.x) - pose.th)
            self._apply(0.0, clamp(2.2 * want, -1.2, 1.2), 0.0, 0.0, dt)
            if abs(want) < 0.12:
                L["hold"] = L.get("hold", now)
            if (now - L.get("hold", 1e18) > 0.9) or now - L["t0"] > 4.0:
                self.facetour.mark(L["key"])
                self._look = None
            return
        if self._cur_goal is None:
            blocked, _ = self.stack.grid.cost_maps(self.cfg.nav.inflate_hard, self.cfg.nav.inflate_soft, 0.0)
            pick = None
            cubes = self._known_cubes(now)
            if len(cubes) >= 4:
                seen = [(f.x, f.y, f.vx, f.vy) for f in self.omap.faces if f.n >= 3]
                pick = self.facetour.pick(cubes, seen, (pose.x, pose.y), blocked, self.stack.grid)
            if pick is not None:
                self._cur_goal, self._goal_kind = pick[0], ("face", pick[1], pick[2])
            else:
                g = self.tour.pick((pose.x, pose.y), blocked, self.stack.grid, None)
                if g is None:
                    self.tour.done.clear()
                    g = self.tour.pick((pose.x, pose.y), blocked, self.stack.grid, None)
                self._cur_goal, self._goal_kind = g, ("lattice",)
            self._goal_t0 = now
            if self._cur_goal is None:
                self._apply(0, 0, 0, 0, dt)
                return
        out = self._nav_to(now, dt, self._cur_goal, 0.18, pan=self._sweep_pan(now, 25.0, 1.3))
        timeout = now - self._goal_t0 > 30.0
        if out.status == "reached":
            kind = self._goal_kind
            self._cur_goal = None
            if kind[0] == "face":
                self._look = {"fc": kind[1], "key": kind[2], "t0": now}
            elif cfgm.spin_w > 0:
                self._spin_active, self._spin_acc, self._spin_prev = True, 0.0, pose.th
            else:
                self.tour.mark_done()
        elif out.status == "blocked" or timeout:
            if self._goal_kind[0] == "face":
                self.facetour.mark(self._goal_kind[2])
                self.facetour.mark(self._goal_kind[2])
            else:
                self.tour.mark_done()
            self._cur_goal = None

    # ================================================================== STRIKE (M2)
    def _strike_goal(self, f, pose) -> Tuple[Optional[Tuple[float, float]], float]:
        """표적 면을 쏠 접근 목표 → ((x, y), 허용 입사각 deg). 면 정면 strike_range 지점이 다른 박스 뒤·벽 속이거나 면이 보이지 않으면
        (박스 사이 좁은 틈의 면 등) 각도·거리를 바꾼 후보 중 '비어 있고 면 앞이 트인' 점을 고른다. 먼저 정면에 가까운 각(±35°)만 보고,
        없으면 비스듬한 각(±80°)까지 넓히되 그 경우 입사각 한계도 함께 넓힌다(대신 더 가까이에서 쏜다)."""
        mc = self.cfg.mission
        base = (clamp(f.x + f.vx * mc.strike_range, 0.35, self.geo.W - 0.35), clamp(f.y + f.vy * mc.strike_range, 0.35, self.geo.H - 0.35))
        try:
            from ..nav.grid import line_free
            grid = self.stack.grid
            blocked, _ = grid.cost_maps(self.cfg.nav.inflate_hard, self.cfg.nav.inflate_soft, 0.0)
        except Exception:
            return (None if self._deadlock_enabled() else base), mc.strike_max_incidence_deg
        ang0 = math.atan2(f.vy, f.vx)
        for tier, (lim, dists, tol) in enumerate(((35.0, (mc.strike_range, 0.85, 0.65, 0.5), mc.strike_max_incidence_deg),
                                                  (70.0, (0.85, 0.65, 0.5), 75.0))):
            best, bs = None, 1e9
            for da in range(-int(lim), int(lim) + 1, 10):
                for d in dists:
                    a = ang0 + math.radians(da)
                    p = (f.x + math.cos(a) * d, f.y + math.sin(a) * d)
                    if self._deadlock_enabled() and not self.progress.eligible(self._goal_key(p),self.robot.now()):
                        continue
                    iy, ix = grid.to_cell(*p)
                    if not (grid.in_bounds(iy, ix) and not blocked[iy, ix] and self.geo.inside(p[0], p[1], 0.30)):
                        continue
                    if not line_free(grid, blocked, p, (f.x + math.cos(a) * 0.22, f.y + math.sin(a) * 0.22)):
                        continue                                     # 면 바로 앞까지 가로막힘 없이 보여야 한다
                    s = math.hypot(p[0] - pose.x, p[1] - pose.y) + 0.8 * abs(math.radians(da)) + 0.6 * abs(d - mc.strike_range)
                    if s < bs:
                        best, bs = p, s
            if best is not None:
                return best, tol
        return (None if self._deadlock_enabled() else base), mc.strike_max_incidence_deg

    def _strike_incidence_ok(self, f, pose) -> bool:
        """표적 면을 거의 정면에서 보고 있나. 수광창은 ±3.5 cm 라 비스듬히(스침각) 쏘면 같은 각도 오차가 면 위에서 1/cos 배로 커진다
        (온라인 탐색 중 옆에서 표적을 처음 본 채 바로 쏘면 계속 빗나가던 문제). 한계는 mission.strike_max_incidence_deg."""
        dx, dy = pose.x - f.x, pose.y - f.y
        d = math.hypot(dx, dy)
        if d < 1e-6:
            return True
        cosang = (dx * f.vx + dy * f.vy) / d
        return cosang >= math.cos(math.radians(self._strike.get("incid_deg", self.cfg.mission.strike_max_incidence_deg)))

    def _m2_can_finish_before_stage(self, now: float, t: float) -> bool:
        """Finish an existing M2 only if its entire shot and stage return fit.

        This never starts a late target, changes avoidance, or postpones a
        published M5. Travel uses sensed occupancy and the same time model as
        exploration. A fixed departure deadline cannot slide on each tick.
        """
        mc, ec, state = self.cfg.mission, self.cfg.explore, self._strike
        face = state.get("face")
        if (self.mode != Mode.STRIKE or self.target_done or self.target_failed
                or self.m5_type is not None or self.stopped
                or not mc.prestage_at <= t < min(mc.t_base_cmd, mc.t_total)
                or state.get("t0", now) - (now-t) >= mc.prestage_at
                or state.get("phase") not in ("nav", "aim")
                or not self._opponent_target_face(face) or not self.laser.ready()
                or self.laser.shots-state.get("shots0", 0) >= mc.max_shots_per_target):
            return False
        pose, grid = self.stack.pose, self.stack.grid
        goal = state.get("goal") if state["phase"] == "nav" else (pose.x, pose.y)
        if goal is None:
            return False
        # Recompute measured routes at most once per second, and after a goal
        # change; an old route estimate never extends the original deadline.
        cached = state.get("stage_finish_budget")
        if cached is None or now-cached["checked_at"] >= 1. or math.dist(goal, cached["goal"]) > grid.res:
            blocked, cost = grid.cost_maps(self.cfg.nav.inflate_hard, self.cfg.nav.inflate_soft, 0.)
            speed = max(self.cfg.nav.v_cruise*ec.plan_speed_scale, .05)
            field = getattr(self, "_field", None)
            onward = risk_astar.plan(grid, blocked, cost, (pose.x, pose.y), goal, speed, field=field)
            returning = risk_astar.plan(grid, blocked, cost, goal, self._stage_point(), speed, field=field)
            if onward is None or returning is None:
                return False
            # A* may snap an obstructed endpoint. Do not budget an unreachable
            # shooting position as though it were the requested one.
            if math.dist(onward.path[-1], goal) > .15 or math.dist(returning.path[-1], self._stage_point()) > .2:
                return False
            travel = 0. if math.dist((pose.x, pose.y), goal) <= .15 else onward.time_s*ec.travel_factor
            back = returning.time_s*ec.travel_factor
            depart = now-t + min(mc.t_base_cmd, mc.t_total)-ec.stage_margin_s-back
            state["stage_finish_by"] = min(state.get("stage_finish_by", math.inf), depart)
            cached = {"checked_at": now, "goal": goal, "travel_s": travel,
                      "return_s": back, "departure_at": state["stage_finish_by"]}
            state["stage_finish_budget"] = cached
        det = self._best_det(lambda d: d.cls == self.cmd.target and d.conf >= self.cfg.perception.conf_min
                             and self._target_detection_matches(d, face))
        pan, _ = self.robot.get_pan_tilt()
        bearing = math.atan2(face.y-pose.y, face.x-pose.x)
        body_turn = abs(wrap(bearing-pose.th)) / max(self.cfg.robot.w_max, 1e-6)
        pan_turn = abs(math.degrees(wrap(bearing-pose.th))-pan) / max(self.cfg.robot.pan_rate, 1e-6)
        locate_sensor = (self.cfg.aim.sensor_fallback_s if self.cfg.aim.require_sensor
                         and (det is None or not det.extra.get("sensors")) else 0.)
        aiming = max(body_turn, pan_turn) + locate_sensor + self.cfg.aim.hold_before_fire
        # Reserve the full emission period, not just the minimum continuous
        # receiver dwell: hardware need not support aborting a five-second shot.
        shot = max(self.cfg.laser.shot_s, self.cfg.laser.dwell_s)
        cached["aim_s"], cached["shot_s"] = aiming, shot
        if not state.get("stage_finish_approved"):
            state["stage_finish_approved"] = now+cached["travel_s"]+aiming+shot <= state["stage_finish_by"]
        # The approved travel/aim budget is consumed, not reserved afresh on
        # every control tick. The fixed last-launch time still bounds failures.
        return state["stage_finish_approved"] and now+shot <= state["stage_finish_by"]

    def _mode_strike(self, now: float, dt: float, t: float) -> None:
        mc, S = self.cfg.mission, self._strike
        pose = self.stack.pose
        old = S.get("face")
        if self._opponent_target_face(old) and old.led_on and self.laser.shots > S.get("shots0", 1 << 30):
            # 우리가 쏘던 그 면의 시그널 LED 가 켜졌다 = 유효 타격. 같은 종류의 다른 면이 미점등이어도 M2 는 끝난 것이다
            # (온라인 탐색은 같은 종류 표적 면을 일찍 여러 개 알게 되므로, 점등 면이 live 목록에서 빠지며 '다른 면으로 재선정'되는 일을 막는다)
            self.target_done = True
            self._log(now, "M2 target signal LED lit on the face we engaged (visual confirm)")
            self.stats["led_confirmed"] += 1
            S.clear()
            self._set_mode(now, Mode.EXPLORE, "M2 done")
            return
        faces = self._target_faces()
        if not faces:
            self._set_mode(now, Mode.EXPLORE, "no target face")
            return
        if "phase" not in S or S.get("face") not in faces:
            f = min(faces, key=lambda f: math.hypot(f.x - pose.x, f.y - pose.y))
            S.clear()
            S.update(face=f, phase="nav", fails=0, shots0=self.laser.shots, lost_since=None, t0=now)
            self.aim.reset()                                         # 새 표적 면: 옛 표적의 필터·겨냥점 연속성을 끊는다
        f = S["face"]
        cls = self.cmd.target
        det = self._best_det(lambda d: d.cls == cls and d.conf >= self.cfg.perception.conf_min
                             and self._target_detection_matches(d, f))
        # 서버/LED 확인
        if f.led_on:
            self.target_done = True
        if self.target_done:
            self._log(now, "M2 done")
            S.clear()
            self._set_mode(now, Mode.EXPLORE, "M2 done")
            return
        used = self.laser.shots - S["shots0"]
        if used >= mc.max_shots_per_target and not self.laser.firing():
            self.target_failed = True
            self._log(now, f"M2 give up after {used} shots")
            S.clear()
            self._set_mode(now, Mode.EXPLORE, "M2 failed")
            return
        if S["phase"] == "nav":
            if (S.get("goal") is None or now - S.get("goal_t", -99.0) > 6.0 or
                    (self._deadlock_enabled() and not self.progress.eligible(self._goal_key(S["goal"]),now))):
                S["goal"], S["incid_deg"] = self._strike_goal(f, pose)
                S["goal_t"] = now
            if S["goal"] is None:
                self._strike_block_until=now+self.cfg.progress.retry_after_s
                self._set_mode(now,Mode.EXPLORE,"M2 has no eligible observed approach")
                self._stop_output()
                return
            gx, gy = S["goal"]
            out = self._nav_to(now, dt, (gx, gy), 0.15, pan=self._sweep_pan(now, 30.0, 1.5))
            r_vis = None
            if det is not None:
                r_vis, _ = self._range(det, 0.40, self.robot.get_pan_tilt()[0])
            if (out.status == "reached") or (r_vis is not None and r_vis <= mc.strike_range + 0.3 and self._strike_incidence_ok(f, pose)):
                S["phase"], S["lost_since"], S["t_aim0"] = "aim", None, now      # (11) 목표 취소 뒤 조준 시작
                self.stack.clear_goal()
                self.aim.reset()
            elif out.status == "blocked" and now - S["t0"] > 12.0:
                self.target_failed = True
                S.clear()
                self._set_mode(now, Mode.EXPLORE, "M2 unreachable")
            return
        # ---- aim 단계
        pan, tilt = self.robot.get_pan_tilt()
        over_budget = now - S.get("t_aim0", now) > mc.aim_budget_s and not self.laser.firing()   # (12) 시간 예산 초과
        if det is None or over_budget:
            S["lost_since"] = S["lost_since"] or now
            self.laser.track(False, now, dt)
            if over_budget or (now - S["lost_since"] > 1.5 and not self.laser.firing()):
                S["fails"] += 1
                S["phase"] = "nav"
                S["lost_since"] = None
                if over_budget:
                    self._log(now, f"M2 aim budget exceeded -> re-approach (fails={S['fails']})")
                if S["fails"] >= 3:
                    self.target_failed = True
                    S.clear()
                    self._set_mode(now, Mode.EXPLORE, "M2 lost")
                    return
            if det is None and not self.laser.firing():
                # 표적이 화면에서 벗어났다: 멈춰 서서 기다리지 말고 지도상 면 쪽으로 카메라·차체를 돌려 다시 찾는다
                brg = math.atan2(f.y - pose.y, f.x - pose.x)
                e = wrap(brg - pose.th)
                pc = clamp(math.degrees(e), -self.cfg.robot.pan_limit, self.cfg.robot.pan_limit)
                bw = clamp(self.cfg.aim.body_kp * math.radians(pan), -self.cfg.robot.w_max * 0.7, self.cfg.robot.w_max * 0.7)
                self._apply(0.0, bw, pc, tilt, dt, limit=True)
                return
            self._apply(0.0, 0.0, pan, tilt, dt)
            return
        S["lost_since"] = None
        if not self._strike_incidence_ok(f, pose) and not self.laser.firing():       # 스침각: 정면 접근 지점으로 다시 간다(실패 횟수에 넣지 않는다)
            S["phase"] = "nav"
            self.aim.reset()
            self._log(now, "M2 oblique view -> re-approach front")
            return
        r, how = self._range(det, 0.40, pan)
        pan_c, tilt_c, res = self.aim.update(det, pan, tilt, now, r)
        v = 0.0
        if r is not None and abs(res.err_pan) < 10.0 and not self.laser.firing():
            v = clamp(0.8 * (r - mc.strike_range), -0.10, 0.14)
        self._apply(v, res.body_w, pan_c, tilt_c, dt, limit=True)
        self.laser.track(res.aligned, now, dt)
        # 표적 창은 ±3.5 cm 라 허용 각도가 거리에 반비례한다 → 접근 목표 거리 근처까지 다가온 뒤에만 쏜다(멀리서 우연히 정렬된 순간에 쏘지 않는다)
        ready_r = (r is not None and self.cfg.laser.min_range <= r <= self.cfg.laser.max_range
                   and r <= mc.strike_range + mc.fire_range_slack)
        late_window = S.get("stage_finish_by", math.inf)
        shot_fits = now + max(self.cfg.laser.shot_s, self.cfg.laser.dwell_s) <= late_window
        if (res.aligned and res.sensor_ok and self.aim.held_for(now) >= self.cfg.aim.hold_before_fire and ready_r
                and shot_fits
                and self._may_fire(now) and self._fire_conf_ok(det) and not self._wrong_target_in_line(det, now, pan)
                and self.laser.ready() and not self.laser.firing() and self._shot_geometry_ok(det, now, pan, tilt)):
            if self.laser.fire(now):
                self.stats["strike_shots"] += 1
                self._log(now, f"M2 fire r={r:.2f} err=({res.err_pan:.2f},{res.err_tilt:.2f})")

    # ================================================================== HUNT (M4)
    def _mode_hunt(self, now: float, dt: float, t: float) -> None:
        mc, H = self.cfg.mission, self._hunt
        pose = self.stack.pose
        cands = self._hunt_candidates(now)
        if not cands:
            self._set_mode(now, Mode.EXPLORE, "no enemy")
            return
        tr = min(cands, key=lambda e: math.hypot(e.x - pose.x, e.y - pose.y))
        if H.get("tid") != tr.tid:
            H.clear()
            H.update(tid=tr.tid, t0=now, shots0=self.laser.shots, streak=0)
            self.aim.reset()                                         # 새 교전 대상(UGV): 직전 겨냥점·센서 선택을 비운다
        if now - H["t0"] > 25.0 and not self.laser.firing():          # 한 대에 너무 오래 매달리지 않는다
            self._hunt_block[tr.tid] = now + 15.0
            H.clear()
            self._set_mode(now, Mode.EXPLORE, "hunt timeout")
            return
        opp = self.geo.opponent(self.team)
        pan, tilt = self.robot.get_pan_tilt()
        det = self._best_det(lambda d: d.cls == "UGV" and d.conf >= self.cfg.perception.ugv_conf_min and d.team == opp)
        if det is None:
            H["streak"] = 0
            self.laser.track(False, now, dt)
            px, py = tr.predicted(now)
            d = math.hypot(px - pose.x, py - pose.y)
            if d > mc.ugv_range:
                ux, uy = (pose.x - px) / max(d, 1e-6), (pose.y - py) / max(d, 1e-6)
                goal = (clamp(px + ux * mc.ugv_range, 0.35, self.geo.W - 0.35),
                        clamp(py + uy * mc.ugv_range, 0.35, self.geo.H - 0.35))
                if self._cur_goal is None or math.hypot(goal[0] - self._cur_goal[0], goal[1] - self._cur_goal[1]) > 0.25:
                    self._cur_goal = goal
                self._nav_to(now, dt, self._cur_goal, 0.2, pan=self._sweep_pan(now, 40.0, 1.6))
            else:   # 가까운데 안 보임: 제자리에서 적 방향으로 돌아 찾는다
                want = wrap(math.atan2(py - pose.y, px - pose.x) - pose.th)
                self._apply(0.0, clamp(2.0 * want, -1.2, 1.2), 0.0, 0.0, dt)
            return
        H["streak"] += 1
        self.stats["hunt_seen"] = self.stats.get("hunt_seen", 0) + 1
        r, how = self._range(det, 0.32, pan)
        pan_c, tilt_c, res = self.aim.update(det, pan, tilt, now, r)
        v = 0.0
        if r is not None and not self.laser.firing():
            if r > mc.ugv_range + 0.2 and abs(res.err_pan) < 15:
                v = clamp(0.7 * (r - mc.ugv_range), 0.0, 0.25)
            elif r < 0.7:
                v = -0.1
        self._apply(v, res.body_w, pan_c, tilt_c, dt, limit=True)
        self.laser.track(res.aligned, now, dt)
        blocked = (self._friend_in_line(det, now, pan) or not self._may_fire(now) or not res.sensor_ok
                   or not self._fire_conf_ok(det))
        ready_r = r is not None and self.cfg.laser.min_range <= r <= self.cfg.laser.max_range
        why = ("nosensor" if not res.sensor_ok else "friend" if blocked else "streak" if H["streak"] < 3 else
               "unaligned" if not res.aligned else
               "hold" if self.aim.held_for(now) < self.cfg.aim.hold_before_fire else "range" if not ready_r else
               "notready" if not self.laser.ready() else "ok")
        self.stats.setdefault("hunt_why", {})
        self.stats["hunt_why"][why] = self.stats["hunt_why"].get(why, 0) + 1
        if blocked and self.laser.firing():
            self.laser.abort()             # 발광 중 사격선에 아군이 들어오면 즉시 끈다
            self._log(now, "abort: friend in line")
        if (not blocked and H["streak"] >= 3 and res.aligned and self.aim.held_for(now) >= self.cfg.aim.hold_before_fire
                and ready_r and self.laser.ready() and not self.laser.firing() and self.laser.ammo > self._reserve()
                and self._shot_geometry_ok(det, now, pan, tilt)):
            if self.laser.fire(now):
                self.stats["hunt_shots"] += 1
                self._log(now, f"M4 fire at {tr.ident} r={r:.2f}")

    # ================================================================== STAGE (M5 대비)
    def _stage_point(self) -> Tuple[float, float]:
        if self.cfg.mission.stage_point_override is not None:
            return self.cfg.mission.stage_point_override
        x = self.geo.W - 2.0 if self.team == "BLUE" else 2.0
        return (x, self.geo.H / 2)

    def _mode_stage(self, now: float, dt: float, t: float) -> None:
        pose = self.stack.pose
        sp = self._mandatory_navigation_goal(now,self._stage_point())
        if sp is None:
            self._apply(0.,0.,self._sweep_pan(now,35.,1.4),0.,dt,limit=True)
            return
        d = math.hypot(sp[0] - pose.x, sp[1] - pose.y)
        if d > 0.25:
            self._nav_to(now, dt, sp, 0.2, pan=self._sweep_pan(now, 35.0, 1.4))
            return
        # 도착: 상대 벽(거점)을 향해 정렬하고 팬 스윕으로 거점 표지를 읽는다
        face = 0.0 if self.team == "BLUE" else math.pi
        e = wrap(face - pose.th)
        self._apply(0.0, clamp(2.0 * e, -1.0, 1.0), self._sweep_pan(now, 40.0, 1.2), 0.0, dt)

    # ================================================================== GOAL (M5)
    def _enemy_bases(self) -> List[str]:
        return self.geo.bases(self.geo.opponent(self.team))

    def _resolve_goal_base(self) -> Optional[str]:
        seen = {b: c for b, c in self.omap.seen_bases().items() if b in self._enemy_bases()}
        for b, c in seen.items():
            if c == self.m5_type:
                return b
        unknown = [b for b in self._enemy_bases() if b not in seen]
        if len(unknown) == 1 and self.m5_type not in seen.values() and len(seen) >= 2:
            return unknown[0]                       # 소거법
        return None

    def _mode_goal(self, now: float, dt: float, t: float) -> None:
        pose = self.stack.pose
        if self.laser.firing():
            self.laser.abort()
        if self.m5_base is None:
            self.m5_base = self._resolve_goal_base()
            if self.m5_base:
                self._log(now, f"M5 base resolved -> {self.m5_base} ({self.m5_type})")
        if self.m5_base is None:
            if self.m5_look_since is None:
                self.m5_look_since = now
            if now - self.m5_look_since < 6.0:      # 표지를 읽을 시간을 준다(스테이징 위치로 가며 팬 스윕)
                sp = self._mandatory_navigation_goal(now,self._stage_point())
                if sp is None:
                    self._apply(0.,0.,self._sweep_pan(now,40.,1.6),0.,dt,limit=True)
                    return
                if math.hypot(sp[0] - pose.x, sp[1] - pose.y) > 0.3:
                    self._nav_to(now, dt, sp, 0.25, pan=self._sweep_pan(now, 40.0, 1.6), speed=1.15)
                else:
                    face = 0.0 if self.team == "BLUE" else math.pi
                    self._apply(0.0, clamp(2.0 * wrap(face - pose.th), -1.2, 1.2), self._sweep_pan(now, 45.0, 1.8), 0.0, dt)
                return
            # 시간 초과: 가장 가까운 후보를 추측(1/3 확률이라도 움직이지 않는 것보다 낫다)
            seen = self.omap.seen_bases()
            cand = [b for b in self._enemy_bases() if b not in seen] or self._enemy_bases()
            self.m5_base = min(cand, key=lambda b: math.hypot(self.geo.base_center(b)[0] - pose.x,
                                                              self.geo.base_center(b)[1] - pose.y))
            self._log(now, f"M5 base GUESS -> {self.m5_base}")
        goal = self.geo.base_center(self.m5_base)
        if (self.cfg.mission.goal_entry_creep_s > 0 and
                self.geo.in_base(pose.x, pose.y, self.m5_base, margin=-0.03)):
            if self.m5_entered_local is None:
                self.m5_entered_local = now
                self._log(now, f"M5 near {self.m5_base}; low-speed entry until server STOP")
            if now - self.m5_entered_local < self.cfg.mission.goal_entry_creep_s:
                face = 0.0 if self.team == "BLUE" else math.pi
                err = wrap(face - pose.th)
                v = 0.0 if abs(err) > 0.25 else 0.08
                self._apply(v, clamp(1.4 * err, -0.45, 0.45), 0.0, 0.0, dt, limit=True)
            else:
                self._apply(0.0, 0.0, 0.0, 0.0, dt)
            return
        if self.geo.in_base(pose.x, pose.y, self.m5_base, margin=-0.12):
            # (15) 자체 위치로 거점 진입을 추정했다. 광전센서 판정과 서버 STOP 이 공식 종료 기준이므로
            #      구동 0 으로 기다리기만 하고 M6·경기 종료를 확정하지 않는다. 로컬 만료(t_total)가 최후 안전장치.
            if self.m5_entered_local is None:
                self.m5_entered_local = now
                self._log(now, f"M5 local entry estimate in {self.m5_base}; holding for server STOP")
            self._apply(0.0, 0.0, 0.0, 0.0, dt)
            return
        self.m5_entered_local = None
        goal=self._mandatory_navigation_goal(now,goal)
        if goal is None:
            self._apply(0.,0.,self._sweep_pan(now,35.,1.4),0.,dt,limit=True)
            return
        self._nav_to(now, dt, goal, 0.10, pan=0.0, speed=1.2, raw=not self._deadlock_enabled())



