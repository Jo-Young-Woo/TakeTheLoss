"""월드 좌표 UGV 트랙(상대/아군). 카메라 검출 + UAV 검출을 같은 트랙에 합친다.

용도: (a) 교전 대상 선택·사격선 아군 확인, (b) M6 위험도 증거 수집(이동량, 방문 섹터, 정지 구간).
식별: 경기 시작 직후 UAV 영상에서 각 UGV는 자기 거점(R-1/2/3, B-1/2/3) 근처에 있으므로
거점 번호로 ident('R-UGV-2')를 부여하고, 이후 위치 연속성으로 이어간다(1 FPS라 ID 스위치 가능성은 있음).
"""
from __future__ import annotations

import math
from collections import defaultdict
from dataclasses import dataclass, field
from typing import Callable, Dict, List, Optional, Set, Tuple

from ..config import Config
from ..link.base import UavFrame
from ..types import Detection, Frame, Pose2D, Scan
from .camera import CameraModel, ObsCamera, estimate_range
from .oak_range import DepthRange
from .friend_map import FriendMap


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
    oak_obs: int = 0                                      # 이 트랙에 들어온 OAK 관측 수(6장). 발사 판단에는 쓰지 않는다
    ident_lost: Optional[str] = None                      # avoid.friend_track: 아군을 놓쳐 지운 신원(기록용). 이 트랙은 이제 '식별 안 됨'이다

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
        self.self_pose_overrides = 0
        self.self_identity_conflicts = 0
        self.last_self_association = {}
        self.depth_stats: Dict[str, int] = defaultdict(int)       # oak_det.depth_range: 깊이 거리 사유별 횟수(ok·none·size_disagree·width_reject·dropped·iff_far·iff_boost)
        self.last_depth_range: Optional[Tuple[float, float, Optional[str]]] = None   # (영상 시각, 거리 m, 팀)
        # 아군 추적(avoid.friend_track). 꺼져 있으면 None 이고 아래 모든 분기가 옛 동작과 같다.
        self.fmap: Optional[FriendMap] = FriendMap(cfg, cfg.arena, my_team) if getattr(cfg.avoid, "friend_track", False) else None

    # ------------------------------------------------------------------ 연관
    def _by_tid(self, tid: Optional[int]) -> Optional[Track]:
        if tid is None:
            return None
        return next((tr for tr in self.tracks if tr.tid == tid), None)

    def _match(self, x: float, y: float, now: float, ident_hint: Optional[str] = None, *,
               team: Optional[str] = None, is_cam: bool = False) -> Optional[Track]:
        best, bd = None, 1e9
        reserved = self.fmap.reserved() if self.fmap is not None else ()
        for tr in self.tracks:
            if ident_hint is not None and tr.ident is not None and tr.ident != ident_hint:
                continue
            if tr.ident in reserved:
                # 추적 중인 아군의 트랙은 아군 지도(UAV 프레임)만 이어 준다. 색 없는 검출·상대 색 검출이 위치만 가깝다고 이 트랙을 가져가면 신원이 새어 나간다.
                # 예외: 카메라가 '우리 팀 색'으로 읽었고 다른 아군보다 분명히 이 아군에 가까운 검출.
                if not (is_cam and team == self.my_team and self.fmap.camera_ok(tr.ident, x, y)):
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
                ident: Optional[str] = None, *, force_tid: Optional[int] = None) -> Track:
        improved=bool(getattr(getattr(self.cfg,"progress",None),"enabled",False))
        is_cam = src in ("cam", "oak")                  # OAK 도 카메라 증거다(UAV 증거와 구분) — 출처 이름은 source_team_labels 에 따로 남는다
        tr = self._by_tid(force_tid)                    # 아군 지도가 이미 정한 짝(허용 거리·모호성 검사를 마쳤다)
        if tr is None:
            tr = self._match(x, y, now, ident, team=team, is_cam=is_cam)
        if tr is not None and improved and now<(tr.last_cam_t if is_cam else tr.last_uav_t):
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
        if is_cam and team == self.my_team and self.fmap is not None and tr.ident in self.fmap.reserved():
            self.fmap.note_camera(tr.ident, tr.x, tr.y, now)        # 카메라가 아군 색으로 읽은 검출이 이 아군에 이어졌다 — 마지막 확인 시각 갱신
        if src == "oak":
            tr.oak_obs += 1
        if is_cam:
            tr.last_cam_t = max(tr.last_cam_t,now) if improved else now
        else:
            tr.last_uav_t = max(tr.last_uav_t,now) if improved else now
        sec = self.geo.sector_of(x, y)
        if sec:
            tr.sectors.add(sec)
        return tr

    # ------------------------------------------------------------------ 입력
    def update_camera(self, dets: List[Detection], pose: Pose2D, frame: Frame, scan: Optional[Scan],
                      dth: float = 0.0, cam: Optional[ObsCamera] = None, weight: float = 1.0, source: str = "cam",
                      depth: Optional[Callable[[Detection, Optional[float]], Tuple[Optional[DepthRange], str]]] = None) -> None:
        """cam / weight / source (6장): 팬틸트가 아닌 카메라(OAK)의 모델·장착, 팀 득표 가중치, 출처("oak"). 안 넘기면 기존과 같다.
        depth (oak_det.depth_range): 검출 → (DepthRange | None, 사유) 를 돌려주는 함수(perception/oak_range.UgvDepthRanger.measure 를 프레임에 묶은 것).
        사유가 'ok' 면 깊이 거리가 LiDAR·크기 추정보다 우선하고(how='depth'), 그 거리로 피아식별을 거른다: iff_max_range_m 보다 멀면 팀 색을 쓰지 않고, 가까우면 득표 가중을 올린다.
        깊이로 잰 폭이 UGV 가 아니면(근거가 충분할 때만) 그 검출을 버린다. 그 밖의 사유(깊이 없음·불일치)는 깊이를 무시하고 기존대로 간다."""
        self.my_pose = pose
        cm, pan = (self.cam, frame.pan) if cam is None else (cam.model, frame.pan + cam.pan_equiv_deg)
        for d in dets:
            if d.cls != "UGV" or d.conf < self.cfg.perception.ugv_conf_min:
                continue
            r, how = estimate_range(cm, d, 0.32, scan, pan, dth)
            team, wgt = d.team, d.conf * weight
            if depth is not None:
                dr, why = depth(d, r)
                self.depth_stats[why] += 1
                if why == "width_reject" and dr is not None and dr.quality >= 0.5 and dr.n >= 30:
                    self.depth_stats["dropped"] += 1                  # 깊이로 잰 폭이 UGV 가 아니다(그림·반사·상자 면) — 트랙을 만들지 않는다
                    continue
                if why == "ok" and dr is not None:
                    r, how = dr.r, "depth"
                    if team is not None and dr.r > self.cfg.oak_det.iff_max_range_m:
                        team = None                                   # 먼 UGV 의 안테나 색은 오독이 많다 — 위치 증거만 쓰고 피아는 보류
                        self.depth_stats["iff_far"] += 1
                    elif team is not None:
                        wgt *= self.cfg.oak_det.iff_weight_boost
                        self.depth_stats["iff_boost"] += 1
                    self.last_depth_range = (frame.stamp, float(dr.r), team)
            if r is None or r > 4.0:
                continue
            psi = cm.world_ray(pose, pan, d.cx, d.cy)
            ox, oy = (pose.x, pose.y) if cam is None else cam.origin(pose, how)
            x, y = ox + (r + 0.06) * math.cos(psi), oy + (r + 0.06) * math.sin(psi)
            tr = self._upsert(x, y, frame.stamp, team, wgt, source)
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
        # Continuity can latch onto a different BLUE robot during a crossing
        # or registration correction. A precise, unique CAPTURE-TIME pose
        # match must be allowed to recover even while that wrong track remains
        # inside the continuity gate. Never use arrival-time proximity here.
        # A unique capture-time observation inside our own body is stronger
        # than team colour. Explicit other IDs and ground-camera tracks stay.
        direct = sorted((math.hypot(d.x-my_pose.x,d.y-my_pose.y),i)
                        for i,d in enumerate(uav.detections)
                        if d.ident in (None,self_ident))
        precise_index = (direct[0][1] if direct and direct[0][0] <= .12 and
                         (len(direct)==1 or direct[1][0]-direct[0][0] >= .12) else None)
        explicit_self = any(distance < 0 for distance,_ in candidates)
        birth_index = None
        if own_index is None and previous is None and self_ident and now-self._first_uav_t <= 3.:
            # Known instructed start, not simulator truth. Startup only,
            # bounded pose uncertainty, unique detection near our own base.
            parts = self_ident.split('-')
            if len(parts) == 3 and parts[0] == self.my_team[0] and parts[2] in ('1','2','3'):
                bx, by = self.geo.base_center(parts[0]+'-'+parts[2])
                near = sorted((math.hypot(d.x-bx,d.y-by),i) for i,d in enumerate(uav.detections)
                              if d.ident in (None,self_ident))
                if (math.hypot(my_pose.x-bx,my_pose.y-by) <= .35 and near and near[0][0] <= .12
                        and (len(near)==1 or near[1][0]-near[0][0] >= .20)):
                    own_index = birth_index = near[0][1]
        if precise_index is not None and not explicit_self and own_index != precise_index:
            own_index = precise_index
            self.self_reacquired += 1
            self.self_pose_overrides += 1
        precise_self = own_index is not None and own_index == precise_index
        geometry_self = precise_self or birth_index is not None
        explicit_others = {d.ident for d in uav.detections if d.ident not in (None,self_ident)}
        self.last_self_association = dict(stamp=now,capture_pose=[my_pose.x,my_pose.y],
            continuity_expected=list(expected),chosen=own_index,precise=precise_self,
            start_base_match=birth_index is not None,
            colour_override=geometry_self and uav.detections[own_index].team not in (None,self.my_team),
            previous=None if previous is None else list(previous),
            observed=None if own_index is None else [uav.detections[own_index].x,uav.detections[own_index].y],
            detections=[dict(x=d.x,y=d.y,team=d.team,ident=d.ident) for d in uav.detections])
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
                leaked = (tr.last_cam_t < 0 and (geometry_self or tr.team in (None, self.my_team))
                          and tr.ident in (None, self_ident)
                          and math.hypot(px - own.x, py - own.y) < 0.15)
                wrong_birth_identity = (precise_self and tr.last_cam_t < 0 and tr.team == self.my_team
                                        and tr.ident not in explicit_others
                                        and math.hypot(px-own.x,py-own.y) < .12)
                if wrong_birth_identity and tr.ident not in (None,self_ident):
                    self.self_identity_conflicts += 1
                    if self.fmap is not None:
                        self.fmap.lose(tr.ident,now,'self_association_conflict')
                if is_own or leaked or wrong_birth_identity:
                    self.self_pruned += 1
                else:
                    kept.append(tr)
            self.tracks = kept
        dets = [d for i, d in enumerate(uav.detections) if i != own_index]
        fm = self.fmap
        assign: Dict[int, str] = {}
        if fm is not None:
            # 자기 검출을 못 찾은 프레임에서는 자기 자세 근처(0.35 m) 검출을 아군 후보에서 뺀다
            assign, lost = fm.update(dets, now, period_s=getattr(uav, "interval_s", None), self_ident=self_ident,
                                     self_xy=None if own_index is not None else (my_pose.x, my_pose.y))
            self._friend_lost(lost)
        for k, d in enumerate(dets):
            ident, force = d.ident, None
            if k in assign:                                     # 아군 지도가 신원을 이은 검출
                ident = assign[k]
                force = next((r.tid for r in fm.recs.values() if r.ident == ident), None)
            elif not self._ident_done and ident is None:
                for side in ("B", "R"):
                    if fm is not None and side == fm.side:
                        continue                                # 우리 진영 거점 신원은 아군 지도가 정한다
                    for i in (1, 2, 3):
                        bx, by = self.geo.base_center(f"{side}-{i}")
                        if math.hypot(d.x - bx, d.y - by) < 0.75:
                            ident = f"{side}-UGV-{i}"
            tr = self._upsert(d.x, d.y, now, d.team, 1.0, "uav", ident, force_tid=force)
            if k in assign:
                fm.bind(ident, tr.tid)
        if now - self._first_uav_t > 3.0:
            self._ident_done = True

    # ------------------------------------------------------------------ 아군 지도(avoid.friend_track)
    def _friend_lost(self, idents: List[str]) -> None:
        """놓친 아군의 트랙에서 신원을 지운다 — 이후 위치만 남은 '식별 안 됨' 트랙이다(팀 색 득표가 쌓이면 그 색으로만 판단)."""
        if not idents:
            return
        gone = set(idents)
        for tr in self.tracks:
            if tr.ident in gone:
                tr.ident_lost, tr.ident = tr.ident, None
                tr.identity_disputed = False

    def tick_friends(self, now: float, period_s: Optional[float] = None) -> List[str]:
        """UAV 영상이 끊겨도 시간이 흐르면 놓친 것으로 본다(Agent 가 매 틱 부른다). 새로 놓친 신원 목록."""
        if self.fmap is None:
            return []
        lost = self.fmap.tick(now, period_s)
        self._friend_lost(lost)
        return lost

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
