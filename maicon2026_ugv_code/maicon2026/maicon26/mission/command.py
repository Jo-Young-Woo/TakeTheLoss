"""M1 전술명령 해석: 규칙 기반(즉시·결정적) + 선택적 LLM 보정.

설계 원칙
  - 규칙 파서가 먼저, 항상 결과를 낸다(수 ms). LLM이 없어도/느려도/틀려도 제출은 된다.
  - LLM은 '규칙 파서가 모호하다고 표시한 필드'에서만 규칙 결과를 덮어쓴다.
  - 출력 값은 vocab 의 canonical key 만 허용한다(LLM이 엉뚱한 문자열을 내도 걸러짐).
  - 명령문 한 줄에 표적/거점/섹터가 섞여 나오므로 '역할 키워드'(타격·지정 / 거점·이동)와의
    거리로 역할을 판별한다.
"""
from __future__ import annotations

import difflib
import json
import re
from typing import Callable, Dict, List, Optional, Tuple

from ..types import Command
from ..vocab import BASES, NATO, NATO_KR, TARGETS

TARGET_ROLE = ["타격", "표적", "지정", "공격", "파괴", "격파", "제거", "목표물"]
BASE_ROLE = ["거점", "이동", "진입", "점령", "집결", "향하", "도착", "복귀", "목표"]
NEGATION = ["제외", "말고", "아닌", "하지 말", "금지", "빼고"]


def _norm(s: str) -> str:
    return re.sub(r"\s+", " ", s.strip())


def _mentions(text: str, table: Dict[str, List[str]]) -> List[Tuple[int, int, str, str]]:
    """(시작, 끝, canonical key, 별칭) — 겹치면 긴 별칭 우선."""
    found: List[Tuple[int, int, str, str]] = []
    for key, aliases in table.items():
        for a in sorted(aliases, key=len, reverse=True):
            for m in re.finditer(re.escape(a), text):
                found.append((m.start(), m.end(), key, a))
    found.sort(key=lambda t: (t[0], -(t[1] - t[0])))
    out: List[Tuple[int, int, str, str]] = []
    last_end = -1
    for f in found:
        if f[0] >= last_end:
            out.append(f)
            last_end = f[1]
    return out


def _role_distance(text: str, pos: int, roles: List[str]) -> int:
    best = 10 ** 6
    for r in roles:
        for m in re.finditer(re.escape(r), text):
            best = min(best, abs(m.start() - pos))
    return best


def _negated(text: str, end: int) -> bool:
    tail = text[end:end + 12]
    return any(n in tail for n in NEGATION)


def _fuzzy(text: str, table: Dict[str, List[str]], cutoff: float = 0.72) -> Optional[str]:
    """오타 대비: 어절 단위로 가장 비슷한 별칭."""
    alias_to_key = {a.replace(" ", ""): k for k, al in table.items() for a in al}
    best, bs = None, 0.0
    for tok in re.findall(r"[가-힣A-Za-z0-9]+", text):
        for a, k in alias_to_key.items():
            if len(a) < 3:
                continue
            sc = difflib.SequenceMatcher(None, tok, a).ratio()
            if sc > bs:
                best, bs = k, sc
    return best if bs >= cutoff else None


def _sectors(text: str, team: Optional[str] = None) -> List[str]:
    """등장 순서대로, 내부 이름('B-S1' / 'COMMON' / 예전 NATO 이름)으로. 서버 이름('CM' 등)으로의 변환은 link/schema.py.

    서버 문서의 예시 문장은 「상대 중앙진지를 제외하고 나머지 상대진영 탐색 구역과 …」처럼 *제외*와 *나머지*를 쓴다.
    그래서 ① 부정 표현('제외·말고'…)이 뒤따르는 섹터는 목록에서 빼고, ② 명시한 섹터가 하나도 없는데 '나머지·모든·전체·상대진영'이 있으면
    (팀을 알 때) 채점 구역 4개(상대 3구역 + 공통)에서 제외된 것을 뺀 나머지로 본다. [가정 — 실제 문장은 리허설 /order 로 확인할 것]"""
    hits: List[Tuple[int, int, str]] = []
    for m in re.finditer(r"(?<![A-Za-z])([BR])\s*-?\s*S\s*([1-3])(?![A-Za-z0-9])", text, flags=re.I):
        hits.append((m.start(), m.end(), f"{m.group(1).upper()}-S{m.group(2)}"))
    for m in re.finditer(r"(?<![A-Za-z])COMMON(?![A-Za-z])|(?<![A-Za-z])CM(?![A-Za-z])|공통\s*(?:구역|섹터|구간)?|중앙\s*(?:진지|구역|섹터)", text, flags=re.I):
        hits.append((m.start(), m.end(), "COMMON"))
    for w in NATO:
        for m in re.finditer(rf"(?<![A-Za-z]){w}(?![A-Za-z])", text, flags=re.I):
            hits.append((m.start(), m.end(), w))
    for kr, w in NATO_KR.items():
        for m in re.finditer(kr, text):
            hits.append((m.start(), m.end(), w))
    # 'B섹터', 'D 구역' 같은 한 글자 표기 → NATO 첫 글자 대응(A=ALPHA…)
    for m in re.finditer(r"(?<![A-Za-z-])([A-F])\s*(?:섹터|구역|구간)", text):
        hits.append((m.start(), m.end(), NATO["ABCDEF".index(m.group(1))]))
    hits.sort()
    out: List[str] = []
    neg: List[str] = []
    for _, end, s in hits:
        bucket = neg if _negated(text, end) else out
        if s not in bucket:
            bucket.append(s)
    if not out and team and re.search(r"나머지|모든|전체|상대\s*진영", text):
        p = "R" if team.upper().startswith("B") else "B"
        out = [s for s in (f"{p}-S1", f"{p}-S2", f"{p}-S3", "COMMON") if s not in neg]
    return out


class CommandParser:
    def __init__(self, team: Optional[str] = None) -> None:
        self.team = team                                  # 알면 '나머지/상대진영' 표현을 구역 목록으로 펼친다

    def parse(self, text: str) -> Command:
        t = _norm(text)
        cmd = Command(raw=text, source="rule")
        notes = cmd.notes
        score = 0.0

        # ---- 지정표적
        tm = [m for m in _mentions(t, TARGETS) if not _negated(t, m[1])]
        if tm:
            if len(tm) > 1:
                notes.append("ambiguous:target")
            tm.sort(key=lambda m: _role_distance(t, m[0], TARGET_ROLE))
            cmd.target = tm[0][2]
            score += 1
        else:
            fz = _fuzzy(t, TARGETS)
            if fz:
                cmd.target = fz
                notes.append("ambiguous:target")
                score += 0.5

        # ---- 작전거점
        bm = [m for m in _mentions(t, BASES) if not _negated(t, m[1])]
        if bm:
            if len(bm) > 1:
                notes.append("ambiguous:goal_base")
            bm.sort(key=lambda m: _role_distance(t, m[0], BASE_ROLE))
            cmd.goal_base = bm[0][2]
            score += 1
        else:
            fz = _fuzzy(t, BASES)
            if fz:
                cmd.goal_base = fz
                notes.append("ambiguous:goal_base")
                score += 0.5

        # ---- 섹터
        cmd.sectors = _sectors(t, self.team)
        if cmd.sectors:
            score += 1
        cmd.confidence = score / 3.0
        return cmd


# ====================================================================== LLM 보정
class LLMClient:
    """주최측 LLM Agent API가 공개되면 이 인터페이스로 감싼다(예: HTTP POST → 문자열)."""

    def complete(self, prompt: str, timeout: float) -> str:  # pragma: no cover
        raise NotImplementedError


class CallableLLM(LLMClient):
    def __init__(self, fn: Callable[[str, float], str]) -> None:
        self.fn = fn

    def complete(self, prompt: str, timeout: float) -> str:
        return self.fn(prompt, timeout)


def build_prompt(text: str) -> str:
    return (
        "다음 군사 임무 명령문에서 정보를 추출해 JSON 한 개만 출력하라. 설명·코드블록 금지.\n"
        f"허용 target: {list(TARGETS)} (감시초소/레이더시설/통신시설)\n"
        f"허용 goal_base: {list(BASES)} (작전지휘소/정비소/군수보급소)\n"
        "sectors: 명령문에 나온 탐색 구역 이름 문자열 배열(예: [\"CHARLIE\",\"DELTA\"] 또는 [\"B-S1\"]).\n"
        "타격 대상(target)과 이후 이동 거점(goal_base)을 혼동하지 말 것. 부정(제외/말고) 문구 주의.\n"
        '형식: {"target": ..., "sectors": [...], "goal_base": ...}\n'
        f"명령문: {text}"
    )


def parse_llm_json(s: str) -> Optional[Dict]:
    m = re.search(r"\{.*\}", s, flags=re.S)
    if not m:
        return None
    try:
        d = json.loads(m.group(0))
    except Exception:
        return None
    if not isinstance(d, dict):
        return None
    tgt, base = d.get("target"), d.get("goal_base")
    if tgt not in TARGETS:
        tgt = None
    if base not in BASES:
        base = None
    secs = d.get("sectors")
    secs = [str(x).strip() for x in secs] if isinstance(secs, list) else []
    return {"target": tgt, "goal_base": base, "sectors": secs}


def merge(rule: Command, llm: Optional[Dict]) -> Command:
    """규칙 결과를 기본으로, '모호' 표시 필드나 빈 필드에서만 LLM 값을 채택."""
    if not llm:
        return rule
    out = Command(rule.target, list(rule.sectors), rule.goal_base, rule.raw, rule.confidence,
                  "merged", list(rule.notes))
    for fld in ("target", "goal_base"):
        rv, lv = getattr(rule, fld), llm.get(fld)
        if lv and (rv is None or f"ambiguous:{fld}" in rule.notes):
            if rv != lv:
                out.notes.append(f"llm_override:{fld}")
            setattr(out, fld, lv)
    if not rule.sectors and llm.get("sectors"):
        out.sectors = llm["sectors"]
        out.notes.append("llm_fill:sectors")
    return out


def parse_base_command(text: str) -> Optional[str]:
    """4:00 목표거점 명령 → 거점 종류 key. 못 찾으면 None."""
    t = _norm(text)
    bm = [m for m in _mentions(t, BASES) if not _negated(t, m[1])]
    if bm:
        bm.sort(key=lambda m: _role_distance(t, m[0], BASE_ROLE))
        return bm[0][2]
    return _fuzzy(t, BASES)
