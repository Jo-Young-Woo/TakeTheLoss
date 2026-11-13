"""적 UGV 회피 제어기 — 임무 계층(탐색/지정표적/스테이징/거점)과 *직교*하는 안전 계층.

원칙 (사용자 제약)
  · Nav2 ROS 플러그인과 임무 경유지 선택의 역할을 분리한다. 이 모듈은 '어디로 갈지(안전 경유지)'와 '언제 멈출지'만 정하고, 실제 경로는
    Nav2(NavigateToPose)/내장 A* 가 그 경유지까지 계획한다. 위험 비용은 nav/risk_astar_realtime.py 의 *추정기*에만 쓰인다.
  · 안전 계층(LiDAR 근접 정지, SAFE_STOP, STOP/HALT)이 항상 이 계층보다 위다. 이 모듈의 출력도 `_apply(limit=True)` 의 안전 제한을 거친다.

수준(히스테리시스, 진입은 즉시·해제는 한 단계씩 유지시간 후)
  CLEAR  → AWARE(위험 비용으로 경로 선택, 속도 소폭 감속) → EVADE(소유권 이전, 안전 경유지로 이동) → CLOSE(근거리 기동)
후퇴 후보: 지나온 길(breadcrumb) + 주변 링 샘플 — *현재* 시점에 다시 검증한 것만 쓴다(무조건 후진 금지):
  ① 정적 자유 ② 위험 A* 경로 존재(적 예측 점유 body_k 회피) ③ 도착 위험이 현재보다 낮음 ④ 경로 위에서 적과의 예측 거리가 줄지 않음 ⑤ 연속 노출 시간 한도
시간 예산: 거점(M5)까지 여유(slack)가 없으면 EVADE/HOLD 를 하지 않고 위험이 낮은 경로로 돌파한다(하드 정지·충돌 방지는 유지).

수치는 시뮬 초깃값이다(config.AvoidCfg, field/tunables.py 에 가정/실측 필요 표시).
"""
from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass
from enum import IntEnum
from typing import Any, Deque, Dict, List, Optional, Sequence, Tuple

import numpy as np

from ..config import Config
from ..nav import risk_astar_realtime as risk_route
from ..perception.threat_board import ThreatBoard, ThreatTrack
from ..types import Pose2D, Scan
from ..util import clamp
from .threat_field import ThreatField, risk_at

Pt = Tuple[float, float]


class Level(IntEnum):
    CLEAR = 0
    AWARE = 1
    EVADE = 2
    CLOSE = 3


@dataclass
class AvoidCtx:
    now: float
    dt: float
    t: float                                   # 경기 시각(s)
    pose: Pose2D
    vx: float
    vy: float
    scan: Optional[Scan]
    grid: Any
    blocked: np.ndarray                        # 정적 하드 차단(인플레이션 포함)
    cost: np.ndarray
    field: Optional[ThreatField]
    tracks: List[ThreatTrack]
    v_plan: float
    slack_s: float = 1e9                       # 거점까지 시간 여유(<=0 이면 커밋)
    in_goal: bool = False                      # M5 거점 접근 중
    dash_ok: bool = False                      # 거점 직전 돌진 조건 성립
    mission_goal: Optional[Pt] = None


@dataclass
class AvoidCmd:
    kind: str                                  # CARROT | HOLD | REVERSE | TURN
    goal: Optional[Pt] = None
    path: Optional[List[Pt]] = None
    v: float = 0.0
    w: float = 0.0
    pan: Optional[float] = None                # 위협 쪽으로 카메라를 돌려 팀을 확인(HOLD)
    why: str = ""
    emergency: bool = False


class AvoidController:
    def __init__(self, cfg: Config) -> None:
        self.cfg, self.ac, self.geo = cfg, cfg.avoid, cfg.arena
        self.level = Level.CLEAR
        self._since = 0.0
        self._calm_since: Optional[float] = None
        self.main: Optional[Tuple[ThreatTrack, Dict[str, float]]] = None
        self.cmd: Optional[AvoidCmd] = None
        self._cmd_t0 = 0.0
        self._cmd_until = 0.0
        self._cmd_goal_t = -1e9
        self.breadcrumb: Deque[Tuple[float, float, float]] = deque(maxlen=cfg.avoid.breadcrumb_max)   # (x, y, t)
        self._bc_last: Optional[Pt] = None
        self._entries: Deque[float] = deque(maxlen=16)
        self._owned: Deque[Tuple[float, float]] = deque()          # (시각, dt) — 회피가 구동을 쥔 시간
        self._downgrade_until = -1e9
        self._hold_used = 0.0
        self._rev_dist = 0.0
        self.released = False                                       # 이번 틱에 소유권이 막 풀렸다(임무 목표를 새로 잡아야 한다)
        self.acquired = False                                       # 이번 틱에 막 소유권을 가져갔다
        self._was_owner = False
        self.stats: Dict[str, Any] = {"aware_s": 0.0, "evade_s": 0.0, "close_s": 0.0, "owner_s": 0.0, "evade_entries": 0,
                                       "close_entries": 0, "retreats": 0, "holds": 0, "reverses": 0, "downgrades": 0,
                                       "no_candidate": 0, "released": 0, "commit_skips": 0, "plans": 0, "plan_s": 0.0,
                                       "max_exposure_s": 0.0}

    # ------------------------------------------------------------------ 수명
    def reset(self) -> None:
        """HALT/SAFE_STOP/새 경기: 위협 상태·기동·breadcrumb 를 모두 버린다(재개 시 CLEAR 부터 다시 평가)."""
        self.level = Level.CLEAR
        self._calm_since = None
        self.main = None
        self.cmd = None
        self.breadcrumb.clear()
        self._bc_last = None
        self._hold_used = self._rev_dist = 0.0
        self._was_owner = False
        self.released = self.acquired = False

    def note_pose(self, pose: Pose2D, now: float, safe: bool) -> None:
        """안전했던 지점(위협이 멀었을 때)만 breadcrumb 로 남긴다."""
        if not safe:
            return
        if self._bc_last is None or math.hypot(pose.x - self._bc_last[0], pose.y - self._bc_last[1]) >= self.ac.breadcrumb_step_m:
            self.breadcrumb.append((pose.x, pose.y, now))
            self._bc_last = (pose.x, pose.y)

    @property
    def effective(self) -> Level:
        if self.level >= Level.EVADE and self._downgrade_until > self._now:
            return Level.AWARE
        return self.level

    _now = 0.0

    # ------------------------------------------------------------------ 수준 판정
    def assess(self, ctx: AvoidCtx) -> Level:
        ac, now = self.ac, ctx.now
        self._now = now
        ms: List[Tuple[ThreatTrack, Dict[str, float]]] = []
        for tt in ctx.tracks:
            if tt.friend or tt.weight < 0.3 or tt.age(now) > ac.hold_track_s + 1.0:
                continue
            ms.append((tt, ThreatBoard.metrics(tt, ctx.pose, ctx.vx, ctx.vy, now, ac)))
        cur = self.level
        if not ms:
            self.main = None
            raw, dmin, tmin = Level.CLEAR, 99.0, math.inf
        else:
            ms.sort(key=lambda a: a[1]["dc"])
            self.main = ms[0]
            dmin = min(m["dc"] for _, m in ms)
            tmin = min(m["ttc"] for _, m in ms)
            # 소유권(EVADE/CLOSE)은 *신선한* 정보에만 건다. 오래된(UAV 지연 등) 위치로 후퇴/정지를 하면 기준선보다 나빠졌다(시뮬: UAV 지연 2 s).
            fresh = [m for tt, m in ms if tt.age(now) <= ac.fresh_s]
            fd = min((m["dc"] for m in fresh), default=99.0)
            ft = min((m["ttc"] for m in fresh), default=math.inf)
            if fd < ac.r_close:
                raw = Level.CLOSE
            elif fd < ac.r_evade or (ft < ac.ttc_evade_s and fd < 3.0):
                raw = Level.EVADE
            elif dmin < ac.r_aware or (tmin < ac.ttc_aware_s and dmin < 4.5):
                raw = Level.AWARE
            else:
                raw = Level.CLEAR
        if raw > cur:
            if cur < Level.EVADE <= raw:
                self._entries.append(now)
                self.stats["evade_entries"] += 1
                recent = [x for x in self._entries if now - x <= ac.reentry_window_s]
                if len(recent) > ac.reentry_max and self._downgrade_until <= now:
                    self._downgrade_until = now + ac.downgrade_s            # 같은 위협에 EVADE 재진입이 잦다 → 위험 비용만 쓰는 AWARE 로 격하
                    self.stats["downgrades"] += 1
            if cur < Level.CLOSE <= raw:
                self.stats["close_entries"] += 1
            self.level, self._since, self._calm_since = raw, now, None
        elif cur > Level.CLEAR:
            if cur == Level.CLOSE:
                can_exit = dmin >= ac.r_close + ac.close_exit_margin_m
                hold = ac.exit_hold_close_s
            elif cur == Level.EVADE:
                can_exit = dmin >= ac.r_evade + ac.exit_margin_m and tmin >= 2 * ac.ttc_evade_s
                hold = ac.exit_hold_evade_s
            else:
                can_exit = dmin >= ac.r_aware + ac.exit_margin_m and tmin >= 1.5 * ac.ttc_aware_s
                hold = ac.exit_hold_s
            if can_exit:
                self._calm_since = self._calm_since if self._calm_since is not None else now
                if now - self._calm_since >= hold and now - self._since >= ac.min_hold_s:
                    self.level, self._since, self._calm_since = Level(int(cur) - 1), now, None      # 한 번에 한 단계만 내린다
            else:
                self._calm_since = None
        dt = ctx.dt
        eff = self.effective
        key = {Level.AWARE: "aware_s", Level.EVADE: "evade_s", Level.CLOSE: "close_s"}.get(eff)
        if key:
            self.stats[key] += dt
        return eff

    # ------------------------------------------------------------------ 시간 예산
    def budget(self, t: float, dist_to_goal: float) -> Tuple[float, float]:
        """(κ, slack_s). slack = 남은 시간 − 1.3·ETA − 여유. κ = clamp(slack/t_soft, 0, 1)."""
        eta = dist_to_goal * 1.3 / max(self.cfg.nav.v_cruise, 0.05)
        slack = (self.cfg.mission.t_total - t) - 1.3 * eta - self.ac.t_margin_s
        return clamp(slack / max(self.ac.t_soft_s, 1e-6), 0.0, 1.0), slack

    # ------------------------------------------------------------------ 한 틱
    def step(self, ctx: AvoidCtx) -> Optional[AvoidCmd]:
        """EVADE/CLOSE 일 때 구동 명령. None 이면 임무가 계속 진행(경로만 위험 인지)."""
        self.released = self.acquired = False
        now, ac = ctx.now, self.ac
        eff = self.effective
        cmd: Optional[AvoidCmd] = None
        if eff >= Level.EVADE and self.main is not None:
            # 거점 직전·시간 여유 없음: 정지/후퇴를 하지 않는다(임무 지연 상한). 물리 충돌 방지는 LiDAR 제한이 계속 맡는다.
            if ctx.in_goal:
                # M5 접근·사전 이동(STAGE/GOAL) 중에는 회피가 소유권을 가져가지 않는다(설계서 §5.1 4번: GOAL 이 소유권 유지, 경로계획만 위협 인지).
                # 시뮬에서 소유권을 주면 뒤에서 쫓는 적 앞에서 후퇴·정지를 반복해 M5 가 30~45 s 늦어졌다. 물리 충돌 방지는 LiDAR 제한이 계속 맡는다.
                self.stats["commit_skips"] += 1
                self.cmd = None
            else:
                cmd = self._maneuver(ctx, eff)
        else:
            self.cmd = None
            self._hold_used = 0.0
            self._rev_dist = 0.0
        owner = cmd is not None
        if owner:
            self._owned.append((now, ctx.dt))
            self.stats["owner_s"] += ctx.dt
        while self._owned and now - self._owned[0][0] > ac.cap_window_s:
            self._owned.popleft()
        if sum(d for _, d in self._owned) > ac.cap_s and self._downgrade_until <= now:      # 회피가 구동을 너무 오래 쥐었다 → 임무 지연 상한
            self._downgrade_until = now + ac.downgrade_s
            self.stats["downgrades"] += 1
            self._owned.clear()
        if owner and not self._was_owner:
            self.acquired = True
        if not owner and self._was_owner:
            self.released = True
            self.stats["released"] += 1
        self._was_owner = owner
        return cmd

    # ------------------------------------------------------------------ 기동 선택
    def _geom(self, m: Dict[str, float]) -> str:
        b = abs(m["beta"])
        return "FRONT" if b < math.radians(35) else ("LATERAL" if b < math.radians(110) else "REAR")

    def _maneuver(self, ctx: AvoidCtx, eff: Level) -> Optional[AvoidCmd]:
        ac, now = self.ac, ctx.now
        tt, m = self.main  # type: ignore[misc]
        cls = self._geom(m)
        # 진행 중 기동 유지(진동 방지): 최소 유지시간 동안, 그리고 경유지가 아직 유효할 때
        if self.cmd is not None and now < self._cmd_until and self._cmd_valid(ctx, self.cmd):
            return self.cmd
        # 정면에서 막고 있지만 접근하지 않는 적: 잠깐 정지하며 팀을 확인(돔 노출 최소화 위해 회전 안 함)
        r_here = risk_at(ctx.field, ctx.grid, ctx.pose.x, ctx.pose.y, 0.0)
        if (cls == "FRONT" and m["closing"] <= 0.05 and m["dc"] >= max(ac.hold_front_min_d, ac.r_close) and self._hold_used < ac.hold_max_s
                and r_here < ac.risk_hit):                      # 사정거리·시야 안(위험 높음)에서는 서 있지 않는다
            self._hold_used += ctx.dt
            self.stats["holds"] += 1
            return self._set(AvoidCmd("HOLD", v=0.0, w=0.0, pan=math.degrees(m["beta"]), why="front-hold"), now, 0.5)
        pick = self.pick_retreat(ctx, m)
        if pick is not None:
            goal, path = pick
            self.stats["retreats"] += 1
            return self._set(AvoidCmd("CARROT", goal=goal, path=path, why=f"retreat-{cls.lower()}"), now, ac.commit_min_s)
        self.stats["no_candidate"] += 1
        # 후보가 없다: 근거리(CLOSE)이고 뒤가 비어 있으며 적이 뒤에 없다면 짧게 후진(<=reverse_max_m), 아니면 정지하고 재관측
        if eff == Level.CLOSE and cls != "REAR" and self._rev_dist < ac.reverse_max_m and self._rear_ok(ctx):
            self._rev_dist += ac.reverse_speed * ctx.dt
            self.stats["reverses"] += 1
            return self._set(AvoidCmd("REVERSE", v=-ac.reverse_speed, w=0.0, why="short-reverse", emergency=True), now, 0.3)
        if cls == "REAR":
            return None                                    # 뒤에서 쫓길 때 정지/제자리 회전은 금지: 임무 경로(위험 인지)로 계속 전진
        if r_here >= ac.risk_hit:
            return None                                    # 맞을 수 있는 자리에서 얼어붙지 않는다: 후보가 없으면 임무가 위험 비용 경로로 계속 움직인다(하드 안전은 유지)
        if self._hold_used < ac.hold_max_s + 2.0:
            self._hold_used += ctx.dt
            self.stats["holds"] += 1
            return self._set(AvoidCmd("HOLD", v=0.0, w=0.0, pan=math.degrees(m["beta"]), why="freeze"), now, 0.5)
        return None                                        # 대기 예산 소진: 임무가 위험 비용 경로로 돌파(하드 안전은 유지)

    def _set(self, cmd: AvoidCmd, now: float, hold_s: float) -> AvoidCmd:
        if self.cmd is None or self.cmd.kind != cmd.kind or self.cmd.goal != cmd.goal:
            self._cmd_t0 = now
        self.cmd = cmd
        self._cmd_until = now + hold_s
        return cmd

    def _cmd_valid(self, ctx: AvoidCtx, cmd: AvoidCmd) -> bool:
        if cmd.kind != "CARROT" or cmd.goal is None:
            return True
        if math.hypot(cmd.goal[0] - ctx.pose.x, cmd.goal[1] - ctx.pose.y) < 0.25:
            return False
        iy, ix = ctx.grid.to_cell(*cmd.goal)
        if not ctx.grid.in_bounds(iy, ix) or ctx.blocked[iy, ix]:
            return False
        if ctx.field is not None and bool(ctx.field.body[ctx.field.K - 1][iy, ix]):
            return False
        return True

    def _rear_ok(self, ctx: AvoidCtx) -> bool:
        sc = ctx.scan
        if sc is not None and sc.min_in_sector(math.pi, math.radians(40)) < 0.45:
            return False
        for tt in ctx.tracks:                               # 적이 뒤 ±60° 에서 1.5 m 안이면 후진하지 않는다
            if tt.friend:
                continue
            m = ThreatBoard.metrics(tt, ctx.pose, ctx.vx, ctx.vy, ctx.now, self.ac)
            if abs(m["beta"]) > math.radians(120) and m["dc"] < 1.5:
                return False
        return True

    # ------------------------------------------------------------------ 후퇴 후보
    def pick_retreat(self, ctx: AvoidCtx, m: Dict[str, float]) -> Optional[Tuple[Pt, List[Pt]]]:
        ac, grid, fld = self.ac, ctx.grid, ctx.field
        if fld is None:
            return None
        pose = ctx.pose
        K = fld.K
        r_now = risk_at(fld, grid, pose.x, pose.y, 0.0)
        dc_now = m["dc"]
        # 후보 모으기: breadcrumb(최근 것부터) + 링 샘플
        cands: List[Tuple[Pt, str]] = []
        for (x, y, _t) in list(self.breadcrumb)[::-1]:
            d = math.hypot(x - pose.x, y - pose.y)
            if ac.retreat_min_m <= d <= ac.retreat_max_m:
                cands.append(((x, y), "crumb"))
        for r in ac.ring_radii:
            for i in range(ac.ring_n):
                a = 2 * math.pi * i / ac.ring_n
                cands.append(((pose.x + r * math.cos(a), pose.y + r * math.sin(a)), "ring"))
        scored: List[Tuple[float, Pt]] = []
        for (p, _kind) in cands:
            iy, ix = grid.to_cell(*p)
            if not grid.in_bounds(iy, ix) or ctx.blocked[iy, ix] or not self.geo.inside(p[0], p[1], 0.30):
                continue                                                    # ① 정적 자유
            if bool(fld.body[K - 1][iy, ix]) or bool(fld.body[0][iy, ix]):
                continue
            r_p = float(fld.risk[K - 1][iy, ix])
            if r_p > max(0.25, r_now - 0.10):                              # ③ 도착 위험이 현재보다 충분히 낮아야 한다
                continue
            md = float(fld.mind[K - 1][iy, ix])
            if md < dc_now + 0.15 and not (r_p <= 0.5 * r_now and md >= dc_now - 0.3):   # 적에게 접근하는 '후퇴'는 배제(단 박스 뒤로 숨어 위험이 절반 이하가 되면 조금 가까워도 허용)
                continue
            d = math.hypot(p[0] - pose.x, p[1] - pose.y)
            scored.append((r_p * 3.0 + 0.2 * d - 0.05 * float(fld.mind[K - 1][iy, ix]), p))
        scored.sort(key=lambda a: a[0])
        best: Optional[Tuple[float, Pt, List[Pt]]] = None
        for _s, p in scored[:ac.max_verify]:
            res = risk_route.plan(grid, ctx.blocked, ctx.cost, (pose.x, pose.y), p, ctx.v_plan, fld, ac.w_risk, ac.w_close, ac.risk_hit, 12000)
            self.stats["plans"] += 1
            if res is None or len(res.path) < 2:                            # ② 적 예측 점유를 피한 경로가 없다
                continue
            if self._dist_drops(res.path, fld, grid, ctx.v_plan, dc_now, slack=0.5):   # ④ 경로 위에서 적과의 예측 거리가 크게 줄어든다
                continue
            exp_pen = max(0.0, res.max_exposure_s - ac.exposure_limit_s)    # ⑤ 연속 노출 한도
            self.stats["max_exposure_s"] = max(self.stats["max_exposure_s"], res.max_exposure_s)
            score = res.risk_int + 0.3 * res.length + 4.0 * exp_pen
            if best is None or score < best[0]:
                best = (score, p, res.path)
        self.stats["plan_s"] += 0.0
        if best is None:
            return None
        return best[1], best[2]

    @staticmethod
    def _dist_drops(path: Sequence[Pt], fld: ThreatField, grid, v_plan: float, d_now: float, slack: float = 0.25) -> bool:
        t = 0.0
        for (ax, ay), (bx, by) in zip(path[:-1], path[1:]):
            seg = math.hypot(bx - ax, by - ay)
            t += seg / max(v_plan, 1e-6)
            iy, ix = grid.to_cell(bx, by)
            if grid.in_bounds(iy, ix) and float(fld.mind[fld.layer(t)][iy, ix]) < d_now - slack:
                return True
        return False

    # ------------------------------------------------------------------ 복구 방향 힌트
    def recover_hint(self, pose: Pose2D, tracks: Sequence[ThreatTrack], vx: float, vy: float, now: float) -> Tuple[Optional[float], bool]:
        """막힘 복구(RECOVER/Navigator/SafetyLimiter)의 회전 방향과 후진 허용. 위협이 없으면 (None, True)=기존 동작.
        적 쪽으로 돌지 않고, 적이 뒤 ±60° 에서 1.5 m 안이면 후진하지 않는다."""
        ac = self.ac
        best, bd = None, 1e9
        for tt in tracks:
            if tt.friend or tt.weight < 0.3 or tt.age(now) > ac.hold_track_s + 1.0:
                continue
            mm = ThreatBoard.metrics(tt, pose, vx, vy, now, ac)
            if mm["dc"] < bd:
                best, bd = mm, mm["dc"]
        if best is None or bd > ac.r_aware:
            return None, True
        beta = best["beta"]
        turn = -1.0 if beta > 0 else 1.0                    # 적이 왼쪽(+)이면 오른쪽(-)으로
        allow_back = not (abs(beta) > math.radians(120) and bd < 1.5)
        return turn, allow_back


def carrot_point(path: Sequence[Pt], pose: Pose2D, lead: float) -> Pt:
    """경로(꺾은선)에서 현재 위치에 가장 가까운 점(선분 위 사영)을 찾고 그 앞 lead m 지점을 돌려준다. 경로 끝을 넘지 않는다."""
    if not path:
        return (pose.x, pose.y)
    if len(path) == 1:
        return path[0]
    best_i, best_t, bd = 0, 0.0, 1e18
    for i in range(len(path) - 1):
        ax, ay = path[i]
        bx, by = path[i + 1]
        dx, dy = bx - ax, by - ay
        L2 = dx * dx + dy * dy
        t = 0.0 if L2 < 1e-12 else clamp(((pose.x - ax) * dx + (pose.y - ay) * dy) / L2, 0.0, 1.0)
        d = (ax + t * dx - pose.x) ** 2 + (ay + t * dy - pose.y) ** 2
        if d < bd:
            best_i, best_t, bd = i, t, d
    ax, ay = path[best_i]
    bx, by = path[best_i + 1]
    cur = (ax + best_t * (bx - ax), ay + best_t * (by - ay))
    acc = 0.0
    prev = cur
    for q in list(path[best_i + 1:]):
        seg = math.hypot(q[0] - prev[0], q[1] - prev[1])
        if acc + seg >= lead:
            k = (lead - acc) / max(seg, 1e-9)
            return (prev[0] + (q[0] - prev[0]) * k, prev[1] + (q[1] - prev[1]) * k)
        acc += seg
        prev = q
    return path[-1]
