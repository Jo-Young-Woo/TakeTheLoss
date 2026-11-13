"""위협 보드: 카메라(YOLO+안테나 색) · UAV · LiDAR 동적 군집을 한 목록으로 합쳐 '일반 장애물 / 팀 미상 UGV / 적 UGV / 아군 UGV'를 가른다.

증거 사다리(설계서 §2.1)
  OBJECT  LiDAR 로만 본 움직이는 작은 물체(팀·종류 미상)          → 적과 같은 보수 가중(p_cons)
  UGV     카메라/UAV 로 UGV 임은 확인, 팀은 미상·불일치               → 같은 보수 가중
  ENEMY   UGV 이고 p_enemy ≥ enemy_p                                  → 가중 p_enemy
  FRIEND  UGV 이고 p_enemy ≤ friend_p, 라벨이 충분히 쌓임             → 위험 가중만 friend_w(물리 점유는 그대로)
팀 믿음 p_enemy: 라벨 표(안테나 색 투표, ident 가 확정이면 고정)를 로그오즈로 합치고, 마지막 라벨이 오래되면 0.5 쪽으로 감쇠.
오분류 비용이 비대칭이다 — 아군이라 믿고 접근했는데 적이면 피격, 적이라 믿고 피했는데 아군이면 시간 손실뿐 — 그래서 아군 확정은 높은 문턱.

!! 수치(r_cam, σ, 반감기 …)는 시뮬 초깃값/가정이다. 실측 필요 항목은 config.AvoidCfg 와 field/tunables.py 에 표시한다.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field, replace
from typing import Dict, List, Optional, Sequence, Tuple

from ..config import Config
from ..types import Pose2D
from .lidar_blobs import Blob
from .tracks import Track, UgvTracker

EV_NONE, EV_OBJECT, EV_UGV, EV_ENEMY = 0, 1, 2, 3
EV_NAME = {0: "NONE", 1: "OBJECT", 2: "UGV", 3: "ENEMY"}
R_CAM = 0.80          # [가정] 카메라 안테나 색 라벨의 정확도 — 현장 사진으로 측정(F11). 시뮬은 0.96
T_HALF_LABEL = 6.0    # s: 라벨 신뢰의 반감기


@dataclass
class ThreatTrack:
    tid: int
    x: float
    y: float
    vx: float
    vy: float
    sigma: float
    evidence: int
    friend: bool
    p_enemy: float
    t_last: float                      # 마지막 관측(어느 출처든) 시각 — UAV 는 촬영 시각(지연 포함)
    t_src: Dict[str, float] = field(default_factory=dict)      # cam/uav/lidar 마지막 시각
    ident: Optional[str] = None
    from_blob: bool = False
    weight: float = 1.0                # 위험 가중(팀·ghost 감쇠 반영)
    iff_conflict: bool = False

    def age(self, now: float) -> float:
        return max(0.0, now - self.t_last)

    def speed(self) -> float:
        return math.hypot(self.vx, self.vy)

    def predict(self, now: float, tau: float, ac) -> Tuple[float, float, float]:
        """(x, y, 반경 ρ) — 현재 시각 + tau 의 예측. 관측이 끊기면 ghost 로 반경을 키운다. 속도 외삽은 3 s 로 제한."""
        age = self.age(now) + tau
        dt = min(age, 3.0)
        sig = math.sqrt(self.sigma ** 2 + (0.15 * age) ** 2)          # σ_v = 0.15 m/s [가정]
        rho = ac.r_body + ac.k_sigma * sig
        over = self.age(now) - ac.hold_track_s
        if over > 0:
            rho += min(ac.v_enemy_max * over, 1.2)
        return self.x + self.vx * dt, self.y + self.vy * dt, rho


def _sigmoid(z: float) -> float:
    return 1.0 / (1.0 + math.exp(-max(-30.0, min(30.0, z))))


def deduplicate_tracks(tracks, max_distance=.15):
    """Fuse physically coincident, consistently identified sensor tracks.

    Unknown/conflicting affiliation and two explicit distinct IDs are retained.
    Complete-link clustering prevents a chain of intermediate observations from
    collapsing two real .3m-separated bodies. This never creates extra votes.
    """
    def side(track):
        if track.friend and track.p_enemy<=.20:
            return "ally"
        if not track.friend and track.evidence==EV_ENEMY and track.p_enemy>=.80:
            return "enemy"
        return None
    groups=[]
    for track in sorted(tracks,key=lambda tr:(-tr.t_last,tr.sigma,tr.tid)):
        label=side(track)
        group=None
        if label is not None:
            for candidates in groups:
                if all(side(other)==label and
                       not (track.ident and other.ident and track.ident!=other.ident) and
                       math.hypot(track.x-other.x,track.y-other.y)<max_distance
                       for other in candidates):
                    group=candidates;break
        if group is None:
            groups.append([track])
        else:
            group.append(track)
    out=[]
    for group in groups:
        if len(group)==1:
            out.append(group[0]);continue
        # Group order selects newest observed position, then smaller covariance.
        newest=group[0]
        best=max(group,key=lambda tr:1.-tr.p_enemy if tr.friend else tr.p_enemy)
        sources={name:max(tr.t_src.get(name,-1e9) for tr in group)
                 for name in {name for tr in group for name in tr.t_src}}
        out.append(replace(newest,p_enemy=best.p_enemy,t_src=sources,
            ident=newest.ident or next((tr.ident for tr in group if tr.ident),None),
            weight=max(tr.weight for tr in group)))
    return out


class ThreatBoard:
    def __init__(self, cfg: Config, my_team: str) -> None:
        self.cfg, self.ac, self.geo = cfg, cfg.avoid, cfg.arena
        self.my_team = my_team
        self.tracks: List[ThreatTrack] = []
        self._blob_tracks: Dict[int, ThreatTrack] = {}
        self.ident_conflicts = []
        self._conflicted_ids = set()
        self.stats: Dict[str, int] = {"updates": 0, "enemy": 0, "friend": 0, "object_only": 0,
                                      "static_blobs_suppressed": 0, "static_ghosts_cleared": 0}

    def reset(self) -> None:
        self.tracks = []
        self._blob_tracks.clear()
        self.ident_conflicts.clear()
        self._conflicted_ids.clear()

    # ------------------------------------------------------------------
    def _belief(self, tr: Track, now: float) -> Tuple[float, int]:
        """(p_enemy, 라벨 득표 합). ident 가 있으면 확정."""
        my = self.my_team
        opp = self.geo.opponent(my)
        improved = bool(getattr(getattr(self.cfg, "progress", None), "enabled", False))
        label_age = max(0., now-max(tr.last_cam_t, tr.last_uav_t))
        if tr.ident:
            p = 0.995 if tr.ident[0] != my[0] else 0.005
            if improved:
                identity_team = opp if tr.ident[0] != my[0] else my
                labels = [{"source":source,"team":label,"stamp":stamp,"age_s":max(0.,now-stamp),"confidence":confidence}
                          for source,(label,stamp,confidence) in getattr(tr,"source_team_labels",{}).items()
                          if 0. <= now-stamp <= 5. and confidence >= .65]
                # Compatibility with restored/older tracks which predate the
                # per-sensor label field: a fresh contradictory majority also
                # invalidates identity confidence, never silently relabels it.
                conflict = any(item["team"] != identity_team for item in labels)
                conflict |= not labels and label_age <= 5. and tr.team not in (None,identity_team)
                if conflict:
                    tr.identity_conflict_t = max(getattr(tr,"identity_conflict_t",-1e9),
                        max((item["stamp"] for item in labels if item["team"]!=identity_team),
                            default=max(tr.last_cam_t,tr.last_uav_t)))
                disputed_at=getattr(tr,"identity_conflict_t",-1e9)
                if disputed_at>-1e8 and not any(item["team"]==identity_team and
                        item["stamp"]>disputed_at for item in labels):
                    # Silence or LiDAR-only position renewal cannot magically
                    # restore a disputed team ID when the bad label ages out.
                    conflict=True
                tr.identity_disputed=bool(conflict)
                if conflict:
                    self._conflicted_ids.add(tr.tid)
                    self.ident_conflicts.append({"tid":tr.tid,"ident":tr.ident,
                                                 "reason":"fresh_colour_conflicts_with_identity","labels":labels})
                    self.stats["iff_conflicts"] = self.stats.get("iff_conflicts",0)+1
                    return .5, int(sum(tr.team_votes.values()))
            # A historical birth-base identity is not permanent live IFF.
            # LiDAR can renew position, but cannot renew an old colour label.
            if improved and label_age > 5.:
                p = .5 + (p-.5)*2.**(-(label_age-5.)/2.)
            return p, 99
        v_opp = float(tr.team_votes.get(opp, 0.0))
        v_my = float(tr.team_votes.get(my, 0.0))
        llr = math.log(R_CAM / (1.0 - R_CAM))
        z = (v_opp - v_my) * llr
        last = max(tr.last_cam_t, tr.last_uav_t)
        z *= 0.5 ** (max(0.0, now - last) / T_HALF_LABEL)           # 오래된 라벨은 0.5 쪽으로
        return _sigmoid(z), int(v_opp + v_my)

    def update(self, tracker: Optional[UgvTracker], blobs: Sequence[Blob], now: float, pose: Pose2D,
               uav_period: float = 1.0, static_boxes: Sequence = (), use_risk_weights: bool = False) -> List[ThreatTrack]:
        """Fuse robot evidence and dynamic blobs.

        static_boxes must be supplied only for recent UAV footprints confirmed
        by repeated full LiDAR observations, in the current navigation frame.
        They can contradict a LiDAR-only phantom inside a solid structure;
        camera/UAV UGV tracks and boundary/outside blobs are always retained.
        """
        ac = self.ac
        out: List[ThreatTrack] = []
        self.ident_conflicts = []
        self._conflicted_ids.clear()
        self.stats["updates"] += 1
        static_interiors = []
        for box in static_boxes:
            try:
                cx, cy, sx, sy = (float(getattr(box, name)) for name in ("cx", "cy", "sx", "sy"))
            except (AttributeError, TypeError, ValueError):
                continue
            if all(math.isfinite(v) for v in (cx, cy, sx, sy)) and sx > .10 and sy > .10:
                # A real UGV beside a box must not disappear: only the strict
                # interior, inset 5 cm from every edge, contradicts a blob.
                static_interiors.append((cx - sx / 2 + .05, cy - sy / 2 + .05,
                                         cx + sx / 2 - .05, cy + sy / 2 - .05))

        def inside_structure(x, y):
            return any(x0 < x < x1 and y0 < y < y1 for x0, y0, x1, y1 in static_interiors)

        # Expanding stale uncertainty cannot revive a blob contradicted by a
        # confirmed solid interior. Purge its stored measurement before the
        # ghost-retention loop below, not merely the current scan's candidate.
        contradicted = [bid for bid, tt in self._blob_tracks.items()
                        if tt.from_blob and inside_structure(tt.x, tt.y)]
        for bid in contradicted:
            del self._blob_tracks[bid]
        self.stats["static_ghosts_cleared"] += len(contradicted)
        if tracker is not None:
            for tr in tracker.tracks:
                age = now - tr.t
                if age > ac.hold_track_s + ac.ghost_s:
                    continue
                # UgvTracker associates exactly one self observation using
                # capture-time motion. Do not discard real nearby teammates
                # or enemies with a broad arrival-time distance heuristic.
                p, votes = self._belief(tr, now)
                src = "cam" if tr.last_cam_t >= tr.last_uav_t else "uav"
                sig0 = ac.sigma_cam if src == "cam" else ac.sigma_uav
                v = math.hypot(tr.vx, tr.vy)
                vx, vy = (tr.vx, tr.vy) if v >= 0.05 else (0.0, 0.0)
                if v > ac.v_enemy_max:
                    vx, vy = vx * ac.v_enemy_max / v, vy * ac.v_enemy_max / v
                if p >= ac.enemy_p:
                    ev, friend = EV_ENEMY, False
                elif p <= ac.friend_p and votes >= 2:
                    ev, friend = EV_UGV, True
                else:
                    ev, friend = EV_UGV, False
                tt = ThreatTrack(tr.tid, tr.x, tr.y, vx, vy, sig0, ev, friend, p, max(tr.t, tr.last_cam_t, tr.last_uav_t),
                                 {"cam": tr.last_cam_t, "uav": tr.last_uav_t}, tr.ident)
                tt.iff_conflict = tr.tid in self._conflicted_ids
                out.append(tt)
        # LiDAR 군집 연관: 가까운 트랙의 위치를 LiDAR 위치로 갱신(더 정확·더 최신), 없으면 OBJECT 새 트랙
        for b in blobs:
            best, bd = None, 0.6
            for tt in out:
                px, py, _ = tt.predict(now, 0.0, ac)
                d = math.hypot(px - b.x, py - b.y)
                if d < bd:
                    best, bd = tt, d
            if inside_structure(b.x, b.y) and (best is None or best.from_blob):
                # Another unclassified blob is not corroborating UGV evidence.
                # Otherwise input order could let a nearby outside blob absorb
                # an inside phantom and move its retained track into the box.
                self.stats["static_blobs_suppressed"] += 1
                continue
            if best is not None:
                best.x, best.y, best.t_last = b.x, b.y, max(best.t_last, b.t)
                best.t_src["lidar"] = b.t
                best.sigma = min(best.sigma, ac.sigma_lidar)
                if b.speed_est >= 0.05:
                    best.vx, best.vy = b.vx, b.vy
                continue
            bt = self._blob_tracks.get(b.bid)
            if bt is None:
                bt = ThreatTrack(100000 + b.bid, b.x, b.y, b.vx, b.vy, ac.sigma_lidar, EV_OBJECT, False, 0.5, b.t, {"lidar": b.t}, None, True)
                self._blob_tracks[b.bid] = bt
            bt.x, bt.y, bt.vx, bt.vy, bt.t_last = b.x, b.y, b.vx, b.vy, b.t
            bt.t_src["lidar"] = b.t
            out.append(bt)
        alive = {id(t) for t in out}
        self._blob_tracks = {k: v for k, v in self._blob_tracks.items() if now - v.t_last <= ac.hold_track_s + ac.ghost_s}
        for k, v in self._blob_tracks.items():
            if id(v) not in alive and now - v.t_last <= ac.hold_track_s + ac.ghost_s:
                out.append(v)
        if use_risk_weights and bool(getattr(getattr(self.cfg,"progress",None),"enabled",False)):
            before=len(out)
            out=deduplicate_tracks(out)
            self.stats["duplicates_fused"]=self.stats.get("duplicates_fused",0)+before-len(out)
        # 위험 가중
        for tt in out:
            improved = use_risk_weights and bool(getattr(getattr(self.cfg, "progress", None), "enabled", False))
            if improved:
                weights = getattr(self.cfg.risk, "team_weight", {"enemy":1., "unknown":.7, "ally":.3})
                if tt.friend:
                    w = float(weights["ally"])*(1.-tt.p_enemy)
                elif tt.evidence == EV_ENEMY:
                    w = float(weights["enemy"])*tt.p_enemy
                else:
                    w = float(weights["unknown"])
            elif tt.friend:
                w = ac.friend_w
            elif tt.evidence == EV_ENEMY:
                w = max(tt.p_enemy, ac.p_cons)
            else:
                w = ac.p_cons
            age = tt.age(now)
            if age > ac.hold_track_s:                              # ghost: 선형 감쇠
                w *= max(0.0, 1.0 - (age - ac.hold_track_s) / max(ac.ghost_s, 1e-6))
            tt.weight = w
        out = [t for t in out if t.weight > 0.02]
        self.tracks = out
        self.stats["enemy"] = sum(1 for t in out if t.evidence == EV_ENEMY)
        self.stats["friend"] = sum(1 for t in out if t.friend)
        self.stats["object_only"] = sum(1 for t in out if t.evidence == EV_OBJECT)
        return out

    # ------------------------------------------------------------------
    @staticmethod
    def metrics(tt: ThreatTrack, pose: Pose2D, vx: float, vy: float, now: float, ac) -> Dict[str, float]:
        """우리 기준 거리 d(차체 사이), 방위 β(우리 헤딩 기준, + 왼쪽), 접근 속도 closing, TTC."""
        px, py, rho = tt.predict(now, 0.0, ac)
        dx, dy = px - pose.x, py - pose.y
        dc = math.hypot(dx, dy)
        d = max(dc - ac.r_body - 0.17, 0.0)
        beta = math.atan2(dy, dx) - pose.th
        beta = math.atan2(math.sin(beta), math.cos(beta))
        if dc > 1e-6:
            closing = -((dx * (tt.vx - vx) + dy * (tt.vy - vy)) / dc)
        else:
            closing = 0.0
        ttc = d / closing if closing > 0.05 else math.inf
        return {"d": d, "dc": dc, "beta": beta, "closing": closing, "ttc": ttc, "rho": rho, "x": px, "y": py}
