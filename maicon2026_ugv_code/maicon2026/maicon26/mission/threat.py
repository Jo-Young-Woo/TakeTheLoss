"""M6 전장 위험도 분석.

서버의 정답은 상대 UGV의 '생존점수, 잔여탄약, 객체인식점수'로 정해진다(비공개). 우리는 그 셋을
관측 증거로 추정한다:
   생존   ← 우리 유효타격 횟수 + (UAV에서 본 8초 이상 정지 구간 = 피격 정지 의심) 으로 깎음
   탄약   ← 직접 관측 불가: 중립 상수(차별력 없음) — 발광 관측이 가능해지면 여기에 반영
   인식   ← 방문 섹터 수 + 이동 거리(활발히 돌아다니며 정찰했을수록 높다고 추정)
   위협도 = 생존 + 탄약 + 인식 (각 항은 0~1 추정값, 공식은 목숨/4 + 탄/8 + M3점수/30)
인식 추정에는 실제 M3 점수나 숨은 정답을 읽지 않는다.
LLM이 있으면 같은 증거 요약을 주고 순위를 받되, 순열 검증에 실패하면 이 휴리스틱을 쓴다.
"""
from __future__ import annotations

import json
import re
from dataclasses import dataclass
from typing import Dict, List, Optional

from ..perception.tracks import UgvTracker
from .command import LLMClient


@dataclass
class Weights:
    # Equal weight in S11.2. Explicit overrides are experimental estimator
    # settings; the referee always uses the published formula.
    surv: float = 1.0
    ammo: float = 1.0
    recog: float = 1.0


@dataclass
class Evidence:
    ident: str
    seen: bool
    hits_est: float = 0.0
    sectors: int = 0
    travel: float = 0.0
    n_obs: int = 0
    surv: float = 0.5
    ammo: float = 0.6
    recog: float = 0.0
    score: float = 0.0
    hits_known: bool = False        # 뒤쪽 LED 로 피격 횟수를 읽었다(우리 추정보다 정확)
    ammo_known: bool = False        # 뒤쪽 LED 로 남은 탄을 읽었다(없으면 0.6 중간값)
    identity_uncertain: bool = False


def evidence_from_tracks(tracker: UgvTracker, my_team: str) -> Dict[str, Evidence]:
    opp = "R" if my_team == "BLUE" else "B"
    out: Dict[str, Evidence] = {f"{opp}-UGV-{i}": Evidence(f"{opp}-UGV-{i}", False) for i in (1, 2, 3)}
    candidate = bool(getattr(getattr(tracker.cfg,"progress",None),"enabled",False))
    disputed = {tr.ident for tr in tracker.tracks if candidate and getattr(tr,"identity_disputed",False)}
    for ident in disputed:
        if ident in out:
            out[ident].identity_uncertain=True
    for tr in tracker.opponent_tracks():
        if not tr.ident or tr.ident not in out:
            continue
        if tr.ident in disputed:
            continue  # Keep the required ID slot, but do not claim observed evidence for it.
        ev = out[tr.ident]
        # 같은 ident가 여러 트랙으로 갈라졌으면 증거를 합친다
        ev.seen = True
        ev.hits_est += tr.hits_by_us + 0.5 * tr.frozen_episodes
        ev.sectors = max(ev.sectors, len(tr.sectors))
        ev.travel += tr.travel
        ev.n_obs += tr.n_obs
        # E8: 상대 UGV 뒤쪽 LED(피격 색·8칸 탄창)를 읽었으면 추정 대신 그 값을 쓴다 — M6 판정 기준이 바로 생존·잔탄이다
        if tr.led_hits is not None:
            ev.hits_est, ev.hits_known = float(max(tr.led_hits, 0)), True
        if tr.led_ammo is not None:
            ev.ammo, ev.ammo_known = tr.led_ammo / 8.0, True
    return out


def score(evs: Dict[str, Evidence], w: Optional[Weights] = None) -> List[Evidence]:
    w = w or Weights()
    for ev in evs.values():
        if not ev.seen:                      # 한 번도 못 봤다면 정보가 없다: 중간값
            ev.surv, ev.ammo, ev.recog = 0.5, 0.6, 0.3
        else:
            ev.surv = max(0.0, 1.0 - ev.hits_est / 4.0)
            if not ev.ammo_known:
                ev.ammo = 0.6
            ev.recog = 0.5 * min(1.0, ev.sectors / 5.0) + 0.5 * min(1.0, ev.travel / 12.0)
        ev.score = w.surv * max(0.0, ev.surv) + w.ammo * max(0.0, ev.ammo) + w.recog * max(0.0, ev.recog)
    return sorted(evs.values(), key=lambda e: (-e.score, e.ident))


def llm_rank(evs: List[Evidence], llm: LLMClient, timeout: float) -> Optional[List[str]]:
    lines = [f"- {e.ident}: 관측={'예' if e.seen else '아니오'}, 추정 피격 {e.hits_est:.1f}회, "
             f"방문 섹터 {e.sectors}개, 이동 {e.travel:.1f}m" for e in evs]
    prompt = ("상대 UGV 3대 중 가장 위험한 순서(1순위가 가장 위험)를 정하라. 위험도는 생존 상태가 좋고, "
              "잔여 탄약이 많고, 객체 인식 성과가 높을수록 크다. 관측이 없으면 정보 부족으로 중간 취급.\n"
              + "\n".join(lines) + '\nJSON만 출력: {"ranking": ["id1","id2","id3"]}')
    try:
        s = llm.complete(prompt, timeout)
        m = re.search(r"\{.*\}", s, flags=re.S)
        r = json.loads(m.group(0))["ranking"] if m else None
    except Exception:
        return None
    ids = {e.ident for e in evs}
    if isinstance(r, list) and len(r) == len(ids) and set(r) == ids:
        return [str(x) for x in r]
    return None
