"""아군 UGV 추적 지도(UAV 영상) — 출발 때 자기 거점 위치로 신원을 붙이고, 이후 UAV 프레임마다 그 아군을 이어서 추적한다.

규칙(avoid.friend_track):
  * 태어남: 첫 UAV 프레임 뒤 friend_birth_s 안에, 자기 진영 거점(자기 슬롯 제외) friend_birth_radius_m 안에 '한 대만' 보이면 그 거점 번호로 신원('B-UGV-1')을 붙인다.
    한 거점에 둘 이상이 보이거나 상대 색으로 읽힌 것은 붙이지 않는다. 창이 닫힌 뒤에는 새 신원을 붙이지 않는다.
  * 이어짐: 프레임마다 추적 중인 아군과 후보(상대 색이 아닌 검출) 사이를 '서로 가장 가까운 쌍'으로만 잇는다. 허용 거리 = friend_gate_m + friend_max_speed × 경과 시간.
    가장 가까운 후보와 두 번째 후보의 거리 차가 friend_ambiguity_m 미만이거나(어느 쪽인지 모름) 한 검출을 두 아군이 다투면 잇지 않는다.
  * 놓침: 모호하거나, friend_lost_s(UAV 주기의 2.5 배 이상) 동안 확인되지 않거나, 상대 색이 나타나 모순이면 그 순간부터 '식별 안 됨'이다.
    한 번 놓친 아군은 위치가 비슷하다는 이유로 다시 이어 붙이지 않는다(자리를 바꾼 적·아군을 흉내 낸 적이 신원을 가져가지 못하게) — 이후에는 카메라가 읽은 팀 색만이 '아군(번호 모름)'을 만든다.
이 모듈은 위치·신원만 다루며 발사·회피 판단에는 직접 관여하지 않는다: 신원을 잃은 트랙은 UgvTracker 가 ident 를 지워 위협판(ThreatBoard)이 '미확인 UGV'로 다룬다.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Sequence, Set, Tuple

TRACKED, WAITING, LOST, NEVER = "tracked", "waiting", "lost", "never"
TRAIL_MAX = 40


@dataclass
class FriendRec:
    slot: int
    ident: str
    state: str = WAITING                       # waiting(출생 창 열림) | tracked | lost | never(창이 닫혔는데 못 만남)
    x: Optional[float] = None
    y: Optional[float] = None
    t: float = -1e9                            # 마지막으로 확인된 촬영 시각
    t_born: Optional[float] = None
    t_lost: Optional[float] = None
    tid: Optional[int] = None                  # 묶인 UgvTracker 트랙 번호
    n_match: int = 0
    reason: str = ""                           # 놓친/못 만난 사유
    trail: List[List[float]] = field(default_factory=list)       # [t, x, y] 최근 몇 개(보고서용 — JSON 으로 쓸 수 있는 리스트만)


class FriendMap:
    def __init__(self, cfg: Any, geo: Any, my_team: str) -> None:
        self.ac = cfg.avoid
        self.geo, self.my_team = geo, my_team
        self.side = "B" if str(my_team).upper().startswith("B") else "R"
        self.recs: Dict[int, FriendRec] = {i: FriendRec(i, f"{self.side}-UGV-{i}") for i in (1, 2, 3)}
        self.self_slot: Optional[int] = None
        self.first_t: Optional[float] = None
        self.window_end: Optional[float] = None
        self.events: List[Dict[str, Any]] = []
        self._unread: List[Dict[str, Any]] = []                 # drain() 으로 한 번만 꺼내 가는 새 사건(Agent 로그용)
        self.stats: Dict[str, int] = {"born": 0, "matched": 0, "lost_ambiguous": 0, "lost_timeout": 0, "lost_colour": 0,
                                      "lost_other": 0, "birth_ambiguous": 0}

    # ------------------------------------------------------------------ 설정
    def note_self(self, self_ident: Optional[str]) -> None:
        """자기 슬롯은 추적 대상이 아니다('B-UGV-2' 형식). 모르면 세 슬롯 모두 후보로 둔다(자기 검출은 호출 전에 걸러진다)."""
        if self.self_slot is not None or not self_ident:
            return
        try:
            side, _, num = str(self_ident).split("-")
            if side == self.side:
                self.self_slot = int(num)
                self.recs[self.self_slot].state = "self"
        except (ValueError, KeyError):
            pass

    def _is_friend_ident(self, ident: Optional[str]) -> Optional[int]:
        if not ident:
            return None
        for r in self.recs.values():
            if r.ident == ident and r.slot != self.self_slot:
                return r.slot
        return None

    def reserved(self) -> Set[str]:
        """추적 중인 아군의 신원 — UgvTracker 가 다른 검출과 이 트랙을 섞지 않게 막는다."""
        return {r.ident for r in self.recs.values() if r.state == TRACKED}

    def _lost_s(self, period_s: Optional[float]) -> float:
        return max(float(self.ac.friend_lost_s), 2.5 * float(period_s) if period_s else 0.0)

    def _event(self, now: float, kind: str, rec: FriendRec, **extra: Any) -> None:
        ev = {"t": round(float(now), 3), "kind": kind, "ident": rec.ident, **extra}
        self.events.append(ev)
        self._unread.append(ev)
        del self.events[:-60], self._unread[:-60]

    def drain(self) -> List[Dict[str, Any]]:
        out, self._unread = self._unread, []
        return out

    # ------------------------------------------------------------------ 상태 전이
    def lose(self, ident: str, now: float, reason: str) -> bool:
        """아군을 놓쳤다 — 이후 '식별 안 됨'. 이미 놓쳤거나 추적 중이 아니면 False."""
        for r in self.recs.values():
            if r.ident == ident and r.state in (TRACKED, WAITING):
                was = r.state
                r.state, r.t_lost, r.reason = (LOST if was == TRACKED else NEVER), float(now), reason
                key = {"ambiguous": "lost_ambiguous", "timeout": "lost_timeout", "colour": "lost_colour"}.get(reason, "lost_other")
                if was == TRACKED:
                    self.stats[key] += 1
                self._event(now, "lost" if was == TRACKED else "never", r, reason=reason)
                return True
        return False

    def _confirm(self, r: FriendRec, x: float, y: float, now: float, source: str) -> None:
        if r.state == TRACKED and r.x is not None:
            self.stats["matched"] += 1
        r.x, r.y, r.t = float(x), float(y), max(float(now), r.t)
        r.n_match += 1
        r.trail.append([round(float(now), 2), round(float(x), 3), round(float(y), 3)])
        del r.trail[:-TRAIL_MAX]

    def tick(self, now: float, period_s: Optional[float] = None) -> List[str]:
        """영상이 끊겨도 시간이 흐르면 놓친 것으로 본다. 새로 놓친 신원 목록."""
        out = []
        lim = self._lost_s(period_s)
        for r in self.recs.values():
            if r.state == TRACKED and now - r.t > lim:
                if self.lose(r.ident, now, "timeout"):
                    out.append(r.ident)
            if r.state == WAITING and self.window_end is not None and now > self.window_end:
                r.state, r.reason = NEVER, "not_seen_at_start"
                self._event(now, "never", r, reason=r.reason)
        return out

    # ------------------------------------------------------------------ UAV 프레임
    def update(self, dets: Sequence[Any], now: float, *, period_s: Optional[float] = None,
               self_ident: Optional[str] = None, self_xy: Optional[Tuple[float, float]] = None
               ) -> Tuple[Dict[int, str], List[str]]:
        """dets: 자기 자신을 뺀 UAV 검출(UavDet). self_xy 를 주면(자기 검출을 못 찾은 프레임) 그 0.35 m 안의 검출은 자기로 보고 후보에서 뺀다.
        돌려주는 값: ({검출 번호: 신원}, 이번 프레임에 새로 놓친 신원들). 아무 것도 잇지 못한 검출은 매핑에 없다."""
        ac = self.ac
        self.note_self(self_ident)
        if self.first_t is None:
            self.first_t = float(now)
            self.window_end = self.first_t + float(ac.friend_birth_s)
        if period_s:                                            # 영상이 느릴수록 출생 창도 넓힌다(처음엔 주기를 모를 수 있어 프레임마다 다시 계산)
            self.window_end = max(self.window_end, self.first_t + 1.5 * float(period_s))
        assign: Dict[int, str] = {}
        newly_lost: List[str] = []
        cand: List[int] = []                                   # 신원을 붙일 수 있는 후보 검출 번호
        for k, d in enumerate(dets):
            if self_xy is not None and math.hypot(d.x - self_xy[0], d.y - self_xy[1]) < 0.35:
                continue
            slot = self._is_friend_ident(getattr(d, "ident", None))
            if slot is not None:                                # 서버·시뮬이 아군 신원을 직접 알려 준 검출은 그대로 믿는다
                r = self.recs[slot]
                if r.state in (TRACKED, WAITING):
                    if r.state == WAITING:
                        r.state, r.t_born = TRACKED, float(now)
                        self.stats["born"] += 1
                        self._event(now, "born", r, x=round(d.x, 2), y=round(d.y, 2), source="uav_ident")
                    self._confirm(r, d.x, d.y, now, "uav_ident")
                    assign[k] = r.ident
                continue
            if getattr(d, "ident", None) is not None:
                continue                                        # 상대 신원이 붙은 검출
            if d.team not in (None, self.my_team):
                continue                                        # 상대 색으로 읽힌 검출은 아군 후보가 아니다
            cand.append(k)

        # ---- 이어짐: 가까운 쌍부터 하나씩 잇는다(탐욕 배정). 그 쌍 대신 같은 값어치로 설명되는 다른 짝이 있으면(모호) 잇지 않고 놓친 것으로 본다
        amb = float(ac.friend_ambiguity_m)
        trk = [r for r in self.recs.values() if r.state == TRACKED and r.ident not in assign.values() and now > r.t]
        dist: Dict[Tuple[int, int], float] = {}                # (슬롯, 검출 번호) → 거리, 허용 거리 안인 쌍만
        for r in trk:
            gate = ac.friend_gate_m + ac.friend_max_speed * min(now - r.t, 5.0)
            for k in cand:
                if k in assign:
                    continue
                dd = math.hypot(dets[k].x - r.x, dets[k].y - r.y)
                if dd <= gate:
                    dist[(r.slot, k)] = dd
        owner_rec: Dict[int, Tuple[int, float]] = {}           # 슬롯 → (검출, 거리)
        owner_det: Dict[int, Tuple[int, float]] = {}           # 검출 → (슬롯, 거리)
        for dd, slot, k in sorted((dd, slot, k) for (slot, k), dd in dist.items()):
            if slot not in owner_rec and k not in owner_det:
                owner_rec[slot], owner_det[k] = (k, dd), (slot, dd)

        def det_is_owned_by_other(k_alt: int, slot: int) -> bool:
            """검출 k_alt 는 다른 아군이 훨씬 가깝게(amb 이상 차이로) 이미 차지했다 — 이 아군의 대안 후보가 아니다."""
            o = owner_det.get(k_alt)
            return o is not None and o[0] != slot and o[1] + amb <= dist[(slot, k_alt)]

        def rec_is_bound_elsewhere(slot_alt: int, k: int) -> bool:
            """아군 slot_alt 는 다른 검출을 훨씬 가깝게 이미 잡고 있다 — 이 검출의 대안 주인이 아니다."""
            o = owner_rec.get(slot_alt)
            return o is not None and o[0] != k and o[1] + amb <= dist[(slot_alt, k)]

        doomed: Set[int] = set()
        for slot, (k, dd) in owner_rec.items():
            for (s2, k2), d2 in dist.items():
                if s2 == slot and k2 != k and d2 < dd + amb and not det_is_owned_by_other(k2, slot):
                    doomed.add(slot)                           # 같은 아군일 수 있는 다른 검출이 있다(적·다른 아군일 수도)
                if k2 == k and s2 != slot and d2 < dd + amb and not rec_is_bound_elsewhere(s2, k):
                    doomed.update((slot, s2))                  # 같은 검출을 다른 아군도 똑같이 주장한다
        for slot, (k, dd) in owner_rec.items():
            r = self.recs[slot]
            if slot not in doomed:
                self._confirm(r, dets[k].x, dets[k].y, now, "uav")
                assign[k] = r.ident
        for slot in sorted(doomed):
            if self.lose(self.recs[slot].ident, now, "ambiguous"):
                newly_lost.append(self.recs[slot].ident)

        # ---- 태어남: 창이 열려 있는 동안 자기 거점 근처에 한 대만 보일 때
        if now <= (self.window_end if self.window_end is not None else -1e9):
            for r in self.recs.values():
                if r.state != WAITING:
                    continue
                bx, by = self.geo.base_center(f"{self.side}-{r.slot}")
                near = [k for k in cand if k not in assign and math.hypot(dets[k].x - bx, dets[k].y - by) < ac.friend_birth_radius_m]
                if len(near) == 1:
                    k = near[0]
                    r.state, r.t_born = TRACKED, float(now)
                    self.stats["born"] += 1
                    self._event(now, "born", r, x=round(dets[k].x, 2), y=round(dets[k].y, 2), source="base")
                    self._confirm(r, dets[k].x, dets[k].y, now, "base")
                    assign[k] = r.ident
                elif len(near) > 1:
                    self.stats["birth_ambiguous"] += 1
                    r.state, r.reason = NEVER, "birth_ambiguous"
                    self._event(now, "never", r, reason="birth_ambiguous", n=len(near))
        newly_lost += self.tick(now, period_s)
        return assign, newly_lost

    # ------------------------------------------------------------------ 카메라
    def camera_ok(self, ident: str, x: float, y: float) -> bool:
        """우리 카메라가 읽은 '아군 색' 검출을 이 아군에 이어도 되나: 다른 아군 추적점이 이 검출에 더 가깝거나 비슷하게 가까우면 안 된다(자리바꿈 방지)."""
        mine = next((r for r in self.recs.values() if r.ident == ident and r.state == TRACKED and r.x is not None), None)
        if mine is None:
            return False
        d0 = math.hypot(x - mine.x, y - mine.y)
        for r in self.recs.values():
            if r is mine or r.state != TRACKED or r.x is None:
                continue
            if math.hypot(x - r.x, y - r.y) - d0 < self.ac.friend_ambiguity_m:
                return False
        return True

    def note_camera(self, ident: str, x: float, y: float, now: float) -> None:
        """카메라가 아군 색으로 읽은 검출이 이 아군에 이어졌다 — 마지막 확인 위치·시각을 갱신(영상이 느려도 놓치지 않게)."""
        r = next((r for r in self.recs.values() if r.ident == ident and r.state == TRACKED), None)
        if r is not None and now >= r.t:
            self._confirm(r, x, y, now, "cam")

    def bind(self, ident: str, tid: int) -> None:
        for r in self.recs.values():
            if r.ident == ident:
                r.tid = int(tid)

    # ------------------------------------------------------------------ 질의
    def snapshot(self, now: float) -> List[Dict[str, Any]]:
        """아군 지도: [{ident, state: tracked|unidentified|self, x, y, age_s, tid, reason}] — 자기 슬롯 제외. 위치는 마지막으로 확인된 곳이다."""
        out = []
        for r in self.recs.values():
            if r.state == "self":
                continue
            ok = r.state == TRACKED
            out.append({"ident": r.ident, "state": "tracked" if ok else "unidentified",
                        "x": None if r.x is None else round(r.x, 3), "y": None if r.y is None else round(r.y, 3),
                        "age_s": None if r.x is None else round(max(0.0, now - r.t), 2),
                        "tid": r.tid, "reason": r.reason, "n_match": r.n_match})
        return out

    def summary(self, now: float) -> Dict[str, Any]:
        return {"slots": self.snapshot(now), "stats": dict(self.stats), "events": list(self.events[-20:]),
                "window_end": self.window_end}
