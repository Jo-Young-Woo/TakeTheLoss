"""월드 좌표 UGV 트랙(상대/아군). 카메라 검출 + UAV 검출을 같은 트랙에 합친다.

용도: (a) 교전 대상 선택·사격선 아군 확인, (b) M6 위험도 증거 수집(이동량, 방문 섹터, 정지 구간).
식별: 경기 시작 직후 UAV 영상에서 각 UGV는 자기 거점(R-1/2/3, B-1/2/3) 근처에 있으므로
거점 번호로 ident('R-UGV-2')를 부여하고, 이후 위치 연속성으로 이어간다(1 FPS라 ID 스위치 가능성은 있음).
"""
from __future__ import annotations

import math
from collections import defaultdict
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Set, Tuple

from ..config import Config
from ..link.base import UavFrame
from ..types import Detection, Frame, Pose2D, Scan
from .camera import CameraModel, estimate_range


@dataclass
class Track:
    tid: int
    x: float
    y: float
    t: float
    ident: Optional[str] = None
    team_votes: Dict[str, float] = field(default_factory=lambda: defaultdict(float))
    vx: float = 0.0
    vy: float = 0.0
    n_obs: int = 0
    travel: float = 0.0
    sectors: Set[str] = field(default_factory=set)
    hits_by_us: int = 0
    first_t: float = 0.0
    still_since: Optional[float] = None
    frozen_episodes: int = 0
    _in_frozen: bool = False
    last_cam_t: float = -1e9
    last_uav_t: float = -1e9
    # E8: 상대 UGV 뒤쪽 LED 에서 읽은 값(피격 횟수 색, 8칸 탄창). 못 읽었으면 None. 피격은 늘기만 하고 탄은 줄기만 한다.
    led_hits: Optional[int] = None
    led_ammo: Optional[int] = None
    led_reads: List[Tuple[int, int]] = field(default_factory=list)       # 최근 (피격, 탄) 판독(중앙값으로 잡음 완화)
    led_t: float = -1e9
    # Latest independently observed team colour per sensor. Identity inferred
    # from an old start location must not erase contradictory fresh pixels.
    source_team_labels: Dict[str, Tuple[str, float, float]] = field(default_factory=dict)
    identity_conflict_t: float = -1e9
    identity_disputed: bool = False

    @property
    def team(self) -> Optional[str]:
        if not self.team_votes:
            return None
        k, v = max(self.team_votes.items(), key=lambda kv: kv[1])
        tot = sum(self.team_votes.values())
        return k if v >= 0.65 * tot else None

    def predicted(self, now: float, max_dt: float = 3.0) -> Tuple[float, float]:
        dt = min(max(now - self.t, 0.0), max_dt)
        return self.x + self.vx * dt, self.y + self.vy * dt


class UgvTracker:
    GATE = 0.55

    def __init__(self, cfg: Config, my_team: str) -> None:
        self.cfg, self.geo = cfg, cfg.arena
        self.my_team = my_team
        self.cam = CameraModel(cfg.camera)
        self.tracks: List[Track] = []
        self._next = 1
        self._ident_done = False
        self._first_uav_t: Optional[float] = None
        self.my_pose: Optional[Pose2D] = None
        self._self_observation: Optional[Tuple[float, float, float, float, float]] = None
        self.self_filtered = 0
        self.self_pruned = 0
        self.self_reacquired = 0

    # ------------------------------------------------------------------ 연관
    def _match(self, x: float, y: float, now: float, ident_hint: Optional[str] = None) -> Optional[Track]:
        best, bd = None, 1e9
        for tr in self.tracks:
            if ident_hint is not None and tr.ident is not None and tr.ident != ident_hint:
                continue
            px, py = tr.predicted(now)
            d = math.hypot(px - x, py - y)
            age = now-tr.t
            if getattr(getattr(self.cfg,"progress",None),"enabled",False):
                age=max(age,0.)
            gate = self.GATE + 0.35 * min(age, 3.0)
            if d < gate and d < bd:
                best, bd = tr, d
        return best

    def _upsert(self, x: float, y: float, now: float, team: Optional[str], w: float, src: str,
                ident: Optional[str] = None) -> Track:
        improved=bool(getattr(getattr(self.cfg,"progress",None),"enabled",False))
        tr = self._match(x, y, now, ident)
        if tr is not None and improved and now<(tr.last_cam_t if src=="cam" else tr.last_uav_t):
            return tr  # An out-of-order source frame cannot renew evidence.
        if tr is None:
            tr = Track(self._next, x, y, now, ident=ident, first_t=now)
            self._next += 1
            self.tracks.append(tr)
        elif not (improved and now<tr.t):
            dt = now - tr.t
            step = math.hypot(x - tr.x, y - tr.y)
            if 0.0 < dt < 3.0:
                a = 0.5
                tr.vx = (1 - a) * tr.vx + a * (x - tr.x) / dt
                tr.vy = (1 - a) * tr.vy + a * (y - tr.y) / dt
            if step < 1.0:
                tr.travel += step if step > 0.03 else 0.0     # 잡음 수준의 떨림은 이동으로 치지 않음
            # 정지 구간(=피격 정지 의심) 집계
            if step < 0.08:
                if tr.still_since is None:
                    tr.still_since = tr.t
                if now - tr.still_since >= 8.0 and not tr._in_frozen:
                    tr._in_frozen = True
                    tr.frozen_episodes += 1
            else:
                tr.still_since, tr._in_frozen = None, False
            tr.x, tr.y, tr.t = x, y, max(now, tr.t)
            if ident and tr.ident is None:
                tr.ident = ident
        if team:
            tr.team_votes[team] += w
            previous = tr.source_team_labels.get(src)
            if team in ("RED", "BLUE") and (previous is None or now >= previous[1]):
                tr.source_team_labels[src] = (team, float(now), float(w))
        tr.n_obs += 1
        if src == "cam":
            tr.last_cam_t = max(tr.last_cam_t,now) if improved else now
        else:
            tr.last_uav_t = max(tr.last_uav_t,now) if improved else now
        sec = self.geo.sector_of(x, y)
        if sec:
            tr.sectors.add(sec)
        return tr

    # ------------------------------------------------------------------ 입력
    def update_camera(self, dets: List[Detection], pose: Pose2D, frame: Frame, scan: Optional[Scan],
                      dth: float = 0.0) -> None:
        self.my_pose = pose
        for d in dets:
            if d.cls != "UGV" or d.conf < self.cfg.perception.ugv_conf_min:
                continue
            r, how = estimate_range(self.cam, d, 0.32, scan, frame.pan, dth)
            if r is None or r > 4.0:
                continue
            psi = self.cam.world_ray(pose, frame.pan, d.cx)
            x, y = pose.x + (r + 0.06) * math.cos(psi), pose.y + (r + 0.06) * math.sin(psi)
            tr = self._upsert(x, y, frame.stamp, d.team, d.conf, "cam")
            led = d.extra.get("led") if d.extra else None
            if led and r <= self.cfg.event.ugv_led_range_m and float(led.get("conf", 0.0)) >= self.cfg.event.ugv_led_conf_min:
                self._add_led(tr, int(led["hits"]), int(led["ammo"]), frame.stamp)

    @staticmethod
    def _add_led(tr: Track, hits: int, ammo: int, t: float) -> None:
        """뒤쪽 LED 판독을 트랙에 쌓는다. 한 판독의 오독(가려진 칸·반사)을 피하려고 최근 5개의 중앙값을 쓰고,
        피격은 줄지 않고 탄은 늘지 않는다는 사실로 단조 보정한다."""
        tr.led_reads.append((max(0, min(hits, 4)), max(0, min(ammo, 8))))
        del tr.led_reads[:-5]
        hs = sorted(h for h, _ in tr.led_reads)
        ams = sorted(a for _, a in tr.led_reads)
        med_h, med_a = hs[len(hs) // 2], ams[len(ams) // 2]
        tr.led_hits = med_h if tr.led_hits is None else max(tr.led_hits, med_h)
        tr.led_ammo = med_a if tr.led_ammo is None else min(tr.led_ammo, med_a)
        tr.led_t = t

    def update_uav(self, uav: UavFrame, my_pose: Pose2D, *, self_ident: Optional[str] = None) -> None:
        """Ingest a frame against our pose at capture time, in the arena frame.

        Track the single self observation through relative pose motion. A
        distance check against the *arrival-time* pose leaks a delayed image of
        ourselves into the obstacle list when moving or when SLAM drifts.
        Other nearby teammates remain valid physical obstacles.
        """
        self.my_pose = my_pose
        now = uav.stamp
        if self._first_uav_t is None:
            self._first_uav_t = now
        expected = (my_pose.x, my_pose.y)
        gate = 0.20
        previous = self._self_observation
        if previous is not None and 0 < now - previous[4] <= 12.0:
            expected = (previous[0] + my_pose.x - previous[2], previous[1] + my_pose.y - previous[3])
            gate = 0.30
        candidates = []
        for i, d in enumerate(uav.detections):
            if self_ident is not None and d.ident == self_ident:
                candidates.append((-1.0, i))
                continue
            if d.team not in (None, self.my_team) or (d.ident is not None and d.ident != self_ident):
                continue
            distance = math.hypot(d.x - expected[0], d.y - expected[1])
            if distance <= gate:
                candidates.append((distance, i))
        if not candidates and previous is not None:
            # A newly validated arena/SLAM alignment can jump the stored pose
            # reference. Reacquire only one precise capture-pose match rather
            # than treating every nearby BLUE detection as ourselves.
            for i, d in enumerate(uav.detections):
                if d.team not in (None, self.my_team) or (d.ident is not None and d.ident != self_ident):
                    continue
                distance = math.hypot(d.x - my_pose.x, d.y - my_pose.y)
                if distance <= 0.20:
                    candidates.append((distance, i))
            if candidates:
                self.self_reacquired += 1
        own_index = min(candidates)[1] if candidates else None
        if own_index is not None:
            own = uav.detections[own_index]
            self._self_observation = (own.x, own.y, my_pose.x, my_pose.y, now)
            self.self_filtered += 1
            # Remove a previously leaked UAV-only self track after reacquiring
            # a precise self association. Preserve camera evidence/other IDs.
            kept = []
            for tr in self.tracks:
                px, py = tr.predicted(now)
                is_own = (self_ident is not None and tr.ident == self_ident)
                leaked = (tr.last_cam_t < 0 and tr.team in (None, self.my_team)
                          and tr.ident in (None, self_ident)
                          and math.hypot(px - own.x, py - own.y) < 0.15)
                if is_own or leaked:
                    self.self_pruned += 1
                else:
                    kept.append(tr)
            self.tracks = kept
        dets = [d for i, d in enumerate(uav.detections) if i != own_index]
        for d in dets:
            ident = d.ident
            if not self._ident_done and ident is None:
                for side in ("B", "R"):
                    for i in (1, 2, 3):
                        bx, by = self.geo.base_center(f"{side}-{i}")
                        if math.hypot(d.x - bx, d.y - by) < 0.75:
                            ident = f"{side}-UGV-{i}"
            self._upsert(d.x, d.y, now, d.team, 1.0, "uav", ident)
        if now - self._first_uav_t > 3.0:
            self._ident_done = True

    # ------------------------------------------------------------------ 질의
    def enemies(self, now: float, max_age: float = 6.0) -> List[Track]:
        out = []
        for tr in self.tracks:
            if now - tr.t > max_age:
                continue
            tm = tr.team
            if tm is not None and tm == self.my_team:
                continue
            if tm is None and tr.ident and tr.ident[0] == self.my_team[0]:
                continue
            out.append(tr)
        return out

    def friends(self, now: float, max_age: float = 6.0) -> List[Track]:
        return [tr for tr in self.tracks if now - tr.t <= max_age and
                (tr.team == self.my_team or (tr.team is None and tr.ident and tr.ident[0] == self.my_team[0]))]

    def opponent_tracks(self) -> List[Track]:
        """M6 증거용: 상대 진영으로 식별된 모든 트랙(오래된 것 포함)."""
        opp = "R" if self.my_team == "BLUE" else "B"
        return [tr for tr in self.tracks if (tr.ident and tr.ident[0] == opp) or tr.team == self.geo.opponent(self.my_team)]

    def nearest_track(self, x: float, y: float, now: float, max_d: float = 0.8) -> Optional[Track]:
        best, bd = None, max_d
        for tr in self.tracks:
            px, py = tr.predicted(now)
            d = math.hypot(px - x, py - y)
            if d < bd:
                best, bd = tr, d
        return best
