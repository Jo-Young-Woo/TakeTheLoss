"""시험용 적대 상대(스크립트). 같은 World 의 SimUGV 를 직접 구동한다 — Agent 가 아니다.

!! 가정이다: 실제 상대 팀 코드는 모른다. 여기서는 '최악에 가까운' 단순 모델(완벽한 위치 정보, 사정거리 안이면 조준·발사)과
   '정지/순찰' 모델로 회피 알고리즘의 *반응*을 재는 데 쓴다. 실기 성능이 아니다.

종류
  Chaser     목표 UGV 를 쫓아 사정거리 안에서 멈추고 조준·발사
  Patroller  주어진 점들을 순서대로 순찰(횡단 시나리오). aim=True 면 지나가다 사정거리에 들면 조준·발사
  Blocker    정지한 채 목표 쪽으로 몸을 돌려 조준·발사(진로 차단)
공통: react_s(반응 지연), v_max(속도), 레이저는 UGV 규칙 그대로(5 s 발광, 5 s 재장전, 2 s 연속 수광 = 유효타격).
"""
from __future__ import annotations

import math
from typing import List, Optional, Sequence, Tuple

from ..types import Pose2D
from ..util import clamp, wrap
from .world import RECEIVER_Z, SimUGV, World


class Hostile:
    kind = "base"

    def __init__(self, world: World, team: str, slot: int, pose: Pose2D, target_id: str, v_max: float = 0.35,
                 react_s: float = 0.6, shoot: bool = True, start_t: float = 0.0,
                 spawn_rel: Optional[Tuple[float, float]] = None, trigger=None) -> None:
        self.world = world
        self.u: SimUGV = world.add_ugv(team, slot)
        self.u.pose = Pose2D(pose.x, pose.y, pose.th)
        self.target_id = target_id
        self.v_max, self.react_s, self.shoot, self.start_t = v_max, react_s, shoot, start_t
        self._in_range_since: Optional[float] = None
        self.fired = 0
        self.spawn_rel = spawn_rel                  # (거리 m, 우리 헤딩 기준 방위 deg): start_t 에 우리 로봇 기준 이 자리(빈 곳·시야 트임)로 나타난다
        self.spawned = spawn_rel is None
        if not self.spawned:
            # Delayed encounters do not exist in the sensed/physical world
            # until placement succeeds. Parking them in R-1 leaked the future
            # opponent into UAV/camera/LiDAR and changed avoidance before t_trig.
            world.ugvs.remove(self.u)
        self.spawn_xy: Optional[Tuple[float, float]] = None
        self.trigger = trigger                      # 선택: fn(match)->bool. True 가 되는 순간부터 start_t(초) 뒤에 등장한다(예: 지정표적 조준 중·거점 접근 중)
        self._armed = trigger is None
        self._match = None

    # ------------------------------------------------------------------ 기하
    def target(self) -> Optional[SimUGV]:
        return next((x for x in self.world.ugvs if x.id == self.target_id), None)

    def can_hit(self, tgt: SimUGV) -> bool:
        """지금 이 위치·자세에서 tgt 의 수광센서를 맞힐 수 있는 조건(거리·상대 전방 반구·가림 없음)."""
        if not self.spawned:
            return False
        w, u = self.world, self.u
        d = math.hypot(tgt.pose.x - u.pose.x, tgt.pose.y - u.pose.y)
        if d > w.cfg.laser.max_range or d < 0.15:
            return False
        if abs(wrap(math.atan2(u.pose.y - tgt.pose.y, u.pose.x - tgt.pose.x) - tgt.pose.th)) >= math.radians(105):
            return False
        return not w._blocked(u.pose.x, u.pose.y, tgt.pose.x, tgt.pose.y, ignore_ugv=(u, tgt))

    def _aim_and_fire(self, now: float, tgt: SimUGV) -> None:
        u = self.u
        if not self.shoot or u.disabled_left > 0:
            return
        ok = self.can_hit(tgt)
        if not ok:
            self._in_range_since = None
            return
        if self._in_range_since is None:
            self._in_range_since = now
        # 가장 가까운 수광센서를 겨눈다
        sx, sy = min(tgt.receivers(), key=lambda p: math.hypot(p[0] - u.pose.x, p[1] - u.pose.y))
        bearing = math.atan2(sy - u.pose.y, sx - u.pose.x)
        rel = wrap(bearing - u.pose.th)
        u.pan_cmd = clamp(math.degrees(rel), -self.world.cfg.robot.pan_limit, self.world.cfg.robot.pan_limit)
        dist = math.hypot(sx - u.pose.x, sy - u.pose.y)
        zt = 0.5 * sum(RECEIVER_Z)
        u.tilt_cmd = clamp(math.degrees(math.atan2(zt - self.world.cfg.robot.cam_height, max(dist, 0.1))), -15.0, 15.0)
        aligned = abs(wrap(math.radians(u.pan) - rel)) < math.radians(1.0)
        if aligned and now - self._in_range_since >= self.react_s and not u.laser_on:
            if u.fire():
                self.fired += 1

    def _drive_to(self, gx: float, gy: float, v: float) -> None:
        """단순 추종 + 박스 앞 회피(앞 0.45 m 가 막히면 트인 쪽으로 돈다)."""
        u, w = self.u, self.world
        want = math.atan2(gy - u.pose.y, gx - u.pose.x)
        err = wrap(want - u.pose.th)
        ahead = (u.pose.x + 0.45 * math.cos(u.pose.th), u.pose.y + 0.45 * math.sin(u.pose.th))
        if w._blocked(u.pose.x, u.pose.y, ahead[0], ahead[1], ignore_ugv=(u,), other_ugv_blocks=False):
            lft = w._blocked(u.pose.x, u.pose.y, u.pose.x + 0.45 * math.cos(u.pose.th + 1.0), u.pose.y + 0.45 * math.sin(u.pose.th + 1.0),
                             ignore_ugv=(u,), other_ugv_blocks=False)
            u.cmd_v, u.cmd_w = 0.0, (-1.0 if lft else 1.0)
            return
        u.cmd_w = clamp(2.0 * err, -1.2, 1.2)
        u.cmd_v = v * max(0.0, math.cos(err)) if abs(err) < 1.2 else 0.0

    def _maybe_spawn(self) -> bool:
        """spawn_rel 이 있으면 start_t 에 우리 로봇 기준 (거리, 방위)로 순간이동한다. 그 점이 박스에 막혔거나 가려지면 방위를 조금씩 바꿔 찾는다.
        반환 False 면 아직 물리/센서 월드에 등록되지 않았다."""
        w = self.world
        if not self._armed:
            if self._match is not None and self.trigger(self._match):
                self._armed = True
                self.start_t = w.t + self.start_t          # trigger 후 start_t 초 뒤(여기서 start_t 는 지연)
            else:
                self.u.cmd_v = self.u.cmd_w = 0.0
                return False
        if self.spawned:
            return True
        if w.t < self.start_t:
            self.u.cmd_v = self.u.cmd_w = 0.0
            return False
        tgt = self.target()
        if tgt is None:
            return False
        dist, brg = self.spawn_rel  # type: ignore[misc]
        for dd in (dist, dist - 0.4, dist + 0.4, dist - 0.8):
            for dbr in (0, 15, -15, 30, -30, 45, -45, 60, -60):
                a = tgt.pose.th + math.radians(brg + dbr)
                x, y = tgt.pose.x + dd * math.cos(a), tgt.pose.y + dd * math.sin(a)
                if not w.geo.inside(x, y, 0.30):
                    continue
                if any(abs(x - c.cx) < 0.45 and abs(y - c.cy) < 0.45 for c in w.cubes):
                    continue
                if w._blocked(tgt.pose.x, tgt.pose.y, x, y, ignore_ugv=(tgt, self.u)):
                    continue
                self.u.pose = Pose2D(x, y, math.atan2(tgt.pose.y - y, tgt.pose.x - x))
                self.spawn_xy = (x, y)
                self.world.ugvs.append(self.u)
                self.spawned = True
                return True
        return False                                   # 이번 틱엔 마땅한 자리가 없다 — 다음 틱에 다시

    def active(self, match) -> bool:
        """이번 틱에 행동해도 되는가(트리거·지연·등장 처리 포함)."""
        self._match = match
        if not self._maybe_spawn():
            return False
        return self.world.t >= self.start_t

    def step(self, match) -> None:
        raise NotImplementedError


class Chaser(Hostile):
    kind = "chaser"

    def __init__(self, *a, stop_dist: float = 1.3, **kw) -> None:
        super().__init__(*a, **kw)
        self.stop_dist = stop_dist

    def step(self, match) -> None:
        w, u = self.world, self.u
        tgt = self.target()
        if tgt is None or not self.active(match):
            u.cmd_v = u.cmd_w = 0.0
            return
        d = math.hypot(tgt.pose.x - u.pose.x, tgt.pose.y - u.pose.y)
        if d > self.stop_dist:
            self._drive_to(tgt.pose.x, tgt.pose.y, self.v_max)
        else:                                               # 사정거리 안: 몸을 돌려 정면으로 겨눈다
            want = math.atan2(tgt.pose.y - u.pose.y, tgt.pose.x - u.pose.x)
            u.cmd_v = 0.0
            u.cmd_w = clamp(2.0 * wrap(want - u.pose.th), -1.0, 1.0)
        self._aim_and_fire(w.t, tgt)


class Patroller(Hostile):
    kind = "patroller"

    def __init__(self, *a, points: Sequence[Tuple[float, float]] = (), aim: bool = True, cross: bool = False, **kw) -> None:
        super().__init__(*a, **kw)
        self.cross = cross
        self.points: List[Tuple[float, float]] = list(points)
        self.i = 0
        self.aim = aim

    def step(self, match) -> None:
        w, u = self.world, self.u
        tgt = self.target()
        if not self.active(match):
            u.cmd_v = u.cmd_w = 0.0
            return
        if self.cross and not self.points and self.spawn_xy is not None and tgt is not None:
            # 횡단: 나타난 자리에서 우리 로봇의 반대편(같은 거리)으로 가로지른다
            sx, sy = self.spawn_xy
            self.points = [(2 * tgt.pose.x - sx, 2 * tgt.pose.y - sy), (sx, sy)]
        if not self.points:
            u.cmd_v = u.cmd_w = 0.0
            return
        gx, gy = self.points[self.i]
        if math.hypot(gx - u.pose.x, gy - u.pose.y) < 0.2:
            self.i = (self.i + 1) % len(self.points)
            gx, gy = self.points[self.i]
        self._drive_to(gx, gy, self.v_max)
        if self.aim and tgt is not None:
            self._aim_and_fire(w.t, tgt)


class Blocker(Hostile):
    kind = "blocker"

    def step(self, match) -> None:
        w, u = self.world, self.u
        tgt = self.target()
        u.cmd_v = 0.0
        if tgt is None or not self.active(match):
            u.cmd_w = 0.0
            return
        want = math.atan2(tgt.pose.y - u.pose.y, tgt.pose.x - u.pose.x)
        e = wrap(want - u.pose.th)
        u.cmd_w = clamp(2.0 * e, -0.8, 0.8) if abs(e) > math.radians(50) else 0.0
        self._aim_and_fire(w.t, tgt)



