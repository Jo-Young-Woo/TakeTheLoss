"""서버 제출 JSON의 임시 형식. 근거: 「참가팀 단계별 API 안내」
(MAICON2026_game-server-api_p0-p12.html, 2026-11-04 스냅숏;
원본은 경기 룰북·SERVER↔UGV 통신 규약). 안내서에 공개 URL은 없으며
docs/API_SOURCE_EXTRACT.txt 에 같은 스냅숏의 추출문을 둔다.

Agent 는 내부 표기(vocab.py 의 canonical key, 'COMMON', 'R-UGV-2')로 payload 를 만들고, 서버가 요구하는 모양으로의 변환은 이 파일의
`*_to_server` 함수가 한 곳에서 한다(HttpServerLink.submit 이 부른다). 시뮬은 내부 표기를 그대로 쓴다.

서버 규칙 요약 (api 5·7·11단계, 부록)
  M1 POST /interpretation  {mission_code, target: "RED_RADAR", search_sectors: ["R-S1","CM"], base: "RED_SUPPLY_DEPOT"}
       target·base 의 앞 글자 진영은 *그 시설이 놓인 진영* — BLUE 팀이 치는 표적·가는 거점은 RED_…. 명칭(「레이더시설」)은 422.
       다시 보내면 **통째로 바뀐다**(빠뜨린 항목은 지난 값이 안 남는다) → 매번 아는 항목을 모두 보낸다. 항목이 하나도 없으면 422.
  M3 POST /detection       {mission_code, sectors: [{sector, objects: [{type, count}]}]}  type: military_truck mortar tank missile ammo_box drum
       섹터는 이름(B-S1~R-S3, CM), 숫자는 422. 같은 섹터 두 번·같은 종류 두 번 422. count 0~99. 채점 구역: BLUE 팀 R-S1·R-S2·R-S3·CM / RED 팀 B-S1·B-S2·B-S3·CM.
  M6 POST /situation       {mission_code, ranking: ["R-2","R-1","R-3"]}  상대 진영 호출부호만(자기 진영이면 422).
"""
from __future__ import annotations

import re
import json
from typing import Any, Dict, List, Optional

from ..vocab import BASE_WIRE, OBJECT_WIRE, TARGET_WIRE, WIRE_SECTORS

SCHEMA_VERIFIED = True        # 룰북 기반 문서로 확인. 단 JSON 형식 확정 전 항목(M1·M3·M6 '임시')은 현장에서 한 번 더 확인한다
SCHEMA_PROVISIONAL = True
SCHEMA_SNAPSHOT = "2026-11-04"


def mission_code_valid(value: Any) -> bool:
    """Eight ASCII alphanumeric characters; validate without altering/echoing the value."""
    return isinstance(value, str) and re.fullmatch(r"[A-Za-z0-9]{8}", value) is not None

# ---- 내부 key → 서버 보고값 -----------------------------------------------------------------
TARGET_CODES, BASE_CODES, OBJECT_CODES = TARGET_WIRE, BASE_WIRE, OBJECT_WIRE
SERVER_SECTORS = WIRE_SECTORS
_REV_BASE = {v: k for k, v in BASE_CODES.items()}
_REV_TARGET = {v: k for k, v in TARGET_CODES.items()}


def m1_payload(cmd) -> Dict[str, Any]:
    return cmd.to_payload()


def m3_payload(counts_by_sector: Dict[str, Dict[str, int]]) -> Dict[str, Any]:
    """내부 형식: {"sectors": {"B-S1": {"TANK": 2}}} (시뮬·Submitter 가 쓴다). 서버 형식은 m3_to_server."""
    return {"sectors": {s: dict(c) for s, c in sorted(counts_by_sector.items())}}


def m6_payload(ranking: List[str]) -> Dict[str, Any]:
    """내부 형식: {"ranking": ["R-UGV-2", ...]}. 서버 형식은 m6_to_server."""
    return {"ranking": list(ranking)}


# ---- 내부 → 서버 -----------------------------------------------------------------------------
def camp_of(team: str) -> str:
    return "BLUE" if str(team).upper().startswith("B") else "RED"


def opponent_camp(team: str) -> str:
    return "RED" if camp_of(team) == "BLUE" else "BLUE"


def server_sector(name: str) -> Optional[str]:
    """내부 섹터 이름 → 서버 이름. 'COMMON'→'CM'. NATO 이름(ALPHA…)·숫자 등 서버에 없는 이름은 None."""
    s = str(name).strip().upper().replace(" ", "")
    if s in ("COMMON", "CM"):
        return "CM"
    m = re.fullmatch(r"([BR])-?S([1-3])", s)
    return f"{m.group(1)}-S{m.group(2)}" if m else None


def scoring_sectors(team: str) -> List[str]:
    """M3 채점 구역: 상대 진영 3구역 + CM."""
    p = "R" if camp_of(team) == "BLUE" else "B"
    return [f"{p}-S1", f"{p}-S2", f"{p}-S3", "CM"]


def m1_to_server(payload: Dict[str, Any], team: str) -> Dict[str, Any]:
    """M1: 아는 항목만 담는다(모르는 것을 틀린 값으로 채우지 않는다). 표적·거점은 상대 진영 접두어."""
    opp = opponent_camp(team)
    out: Dict[str, Any] = {}
    t = payload.get("target")
    if t in TARGET_CODES:
        out["target"] = f"{opp}_{TARGET_CODES[t]}"
    b = payload.get("goal_base")
    if b in BASE_CODES:
        out["base"] = f"{opp}_{BASE_CODES[b]}"
    secs: List[str] = []
    for s in payload.get("sectors") or []:
        ss = server_sector(s)
        if ss and ss not in secs:
            secs.append(ss)
    if secs:
        out["search_sectors"] = secs
    return out


def m3_to_server(payload: Dict[str, Any], team: str) -> Dict[str, Any]:
    """M3: 채점 구역 4개는 항상 넣는다(없으면 objects: []). 그 밖의 섹터는 받지만 채점하지 않으므로 보내지 않는다."""
    cnt: Dict[str, Dict[str, int]] = {}
    for sec, objs in (payload.get("sectors") or {}).items():
        ss = server_sector(sec)
        if ss is None:
            continue
        d = cnt.setdefault(ss, {})
        for typ, n in (objs or {}).items():
            code = OBJECT_CODES.get(str(typ).upper())
            if code:
                # JSON booleans are Python ints, but never valid object counts.
                # Silently truncating 1.9 or parsing "2" would conceal bad data
                # and disagree with the simulator/server validation contract.
                if isinstance(n, bool) or not isinstance(n, int):
                    raise ValueError(f"M3 {ss}/{typ} count must be an integer, not {type(n).__name__}")
                if n > 0:
                    d[code] = min(d.get(code, 0) + n, 99)
    return {"sectors": [{"sector": s, "objects": [{"type": k, "count": v} for k, v in sorted(cnt.get(s, {}).items())]}
                        for s in scoring_sectors(team)]}


def callsign(ident: str) -> Optional[str]:
    """'R-UGV-2' / 'R-2' / 'r2' → 'R-2'."""
    m = re.fullmatch(r"([BR])\s*-?\s*(?:UGV)?\s*-?\s*([1-3])", str(ident).strip().upper())
    return f"{m.group(1)}-{m.group(2)}" if m else None


def m6_to_server(payload: Dict[str, Any], team: str) -> Dict[str, Any]:
    """M6: 상대 진영 호출부호만, 중복 없이, 위험한 순서. 자기 진영·형식 오류는 버린다(서버가 422 로 거부한다)."""
    p = "R" if camp_of(team) == "BLUE" else "B"
    out: List[str] = []
    for r in payload.get("ranking") or []:
        c = callsign(r)
        if c and c.startswith(p) and c not in out:
            out.append(c)
    return {"ranking": out}


def to_server(kind: str, payload: Dict[str, Any], team: str) -> Dict[str, Any]:
    return {"m1": m1_to_server, "m3": m3_to_server, "m6": m6_to_server}[kind](payload, team)


ENDPOINTS = {"m1": "interpretation", "m3": "detection", "m6": "situation"}

# ---- Snapshot error vocabulary. Only quoted, complete strings are exact. ----
ERR_INVALID_CODE = "임무코드가 올바르지 않습니다"
REASON_NOT_ASSIGNED = "이 경기에 편성되지 않은 팀입니다"
REASON_NO_POST = "게시된 임무가 없습니다"
REASON_NO_REHEARSAL = "진행 중인 리허설이 없습니다"
REASON_ORDER_NOT_YET = "임무는 경기 진행이 시작되면 게시됩니다"
REASON_TARGET_NOT_YET = "M5 목표거점은 종료 60초 전에 게시됩니다"
PHASE_LABELS = {"idle": "대기 전", "waiting": "대기", "ready": "준비", "running": "진행",
                "reporting": "보고", "finished": "종료"}
ERR_READY_UNCONFIRMED = "UGV 경기 설정이 아직 확인되지 않았습니다 — 잠시 뒤 다시 보내세요"
ERR_EXCLUDED_PREFIX = "이번 경기에서 빠진 팀입니다"
ERR_NOT_STARTED_PREFIX = "이번 경기에서 UGV가 출발하지 못해 0점 처리된 팀입니다"
ERROR_DETAILS_UNSPECIFIED = frozenset({
    "ready.phase", "ready.before_start", "submission.excluded", "submission.not_started",
    "drone.not_assigned", "drone.destroyed", "drone.no_frame", "drone.disabled",
})  # [확인필요] HTML gives a reason/prefix only; do not fabricate its omitted suffix.


def _phase_label(phase: Any) -> str:
    key = str(phase or "").lower()
    return PHASE_LABELS.get("running" if key == "in_progress" else key, "알 수 없음")


def ready_error(reason: str, phase: str) -> str:
    """Known ready-error text; incomplete source wording is marked above."""
    del phase  # The source does not quote ready's full phase-mismatch sentence.
    messages = {"phase": "준비 단계가 아님", "before_start": "경기 시작 직전",
                "not_assigned": REASON_NOT_ASSIGNED, "unconfirmed": ERR_READY_UNCONFIRMED}
    if reason not in messages:
        raise ValueError("Unknown ready-error reason")
    return messages[reason]


def submission_error(kind: str, reason: str, phase: str) -> str:
    if reason == "not_assigned":
        return REASON_NOT_ASSIGNED
    if reason == "excluded":
        return ERR_EXCLUDED_PREFIX
    if reason == "not_started":
        return ERR_NOT_STARTED_PREFIX
    names = {"m1": ("명령 해석 보고(M1)", "진행"), "m3": ("객체 인식 보고(M3)", "진행"),
             "m6": ("상황보고", "보고")}
    if reason != "phase" or kind not in names:
        raise ValueError("Unknown submission-error reason or mission")
    name, required = names[kind]
    return f"{name}는 '{required}' 단계에서만 받습니다 (지금은 '{_phase_label(phase)}')"


def drone_error(reason: str, phase: str, test_mode_requested: bool = False) -> str:
    messages = {"not_assigned": "지금 경기에 편성된 팀만",
                "phase": f"드론 화면은 경기 진행 중에만 받을 수 있습니다 (지금: {_phase_label(phase)})",
                "destroyed": "우리 진영 시설물 3개 파괴", "no_frame": "아직 화면 없음",
                "disabled": "카메라 수신 꺼짐"}
    if reason not in messages:
        raise ValueError("Unknown drone-error reason")
    suffix = " — 지금은 테스트 모드가 아닙니다" if test_mode_requested and reason in ("not_assigned", "phase") else ""
    return messages[reason]+suffix


def _reason_text(value: Any) -> str:
    if isinstance(value, dict):
        if "detail" in value:
            return _reason_text(value["detail"])
        if "msg" in value:
            return _reason_text(value["msg"])
        return " ".join(_reason_text(v) for v in value.values())
    if isinstance(value, (list, tuple)):
        return " ".join(_reason_text(v) for v in value)
    return str(value or "")


def _compact_reason(value: Any) -> str:
    return re.sub(r"[^\w가-힣]", "", _reason_text(value).casefold())


def classify_reason(reason: Any) -> str:
    """Classify accepted:false reasons without depending on spaces/punctuation."""
    text = _compact_reason(reason)
    for known, category in ((REASON_NOT_ASSIGNED, "NOT_ASSIGNED"), (REASON_NO_REHEARSAL, "NO_REHEARSAL"),
                            (REASON_NO_POST, "NO_POST"), (REASON_ORDER_NOT_YET, "NOT_YET"),
                            (REASON_TARGET_NOT_YET, "NOT_YET")):
        if _compact_reason(known) in text:
            return category
    return "UNKNOWN"


def classify_submission_409(detail: Any) -> str:
    text = _compact_reason(detail)
    for known, category in ((REASON_NOT_ASSIGNED, "NOT_ASSIGNED"), (ERR_EXCLUDED_PREFIX, "EXCLUDED"),
                            (ERR_NOT_STARTED_PREFIX, "UGV_NOT_STARTED")):
        if _compact_reason(known) in text:
            return category
    if any(part in text for part in ("단계에서만받습니다", "준비단계가아님", "경기시작직전", "경기진행중에만받을수있습니다")):
        return "PHASE_MISMATCH"
    return "UNKNOWN"


classify_409 = classify_submission_409


class ValidationError(ValueError):
    """Wire-format 422 detail, containing locations/reasons and no input echoes."""
    def __init__(self, detail):
        self.detail = detail
        super().__init__(detail if isinstance(detail, str) else json.dumps(detail, ensure_ascii=False))


def _invalid(loc, message):
    raise ValidationError([{"loc": ["body", *loc], "msg": message}])


def _body(payload):
    if not isinstance(payload, dict):
        _invalid((), "본문은 JSON 객체여야 합니다")
    return payload


def _wire_sector(value, loc):
    if not isinstance(value, str) or value.strip().upper() not in WIRE_SECTORS:
        _invalid(loc, "섹터는 이름으로 적습니다: " + ", ".join(WIRE_SECTORS))
    return value.strip().upper()


def validate_m1(payload: Dict[str, Any]) -> Dict[str, Any]:
    """Validate literal M1 wire fields; absent/null fields replace prior values with null.

    Authentication (mission_code) is the transport/server's separate concern.
    Source allows reports with a subset; an entirely unreported M1 is invalid.
    """
    payload = _body(payload)
    if "sector" in payload:
        _invalid(("sector",), "옛 sector 필드는 받지 않습니다; search_sectors를 사용합니다")
    out = {"target": None, "search_sectors": None, "base": None}
    for field, suffixes, description in (("target", TARGET_WIRE.values(), "지정표적"),
                                         ("base", BASE_WIRE.values(), "목표거점")):
        value = payload.get(field)
        if value is None:
            continue
        allowed = {f"{camp}_{suffix}" for camp in ("BLUE", "RED") for suffix in suffixes}
        if not isinstance(value, str) or value.strip().upper() not in allowed:
            _invalid((field,), f"{field}는 {description} 보고값으로 적습니다: " + ", ".join(sorted(allowed)))
        out[field] = value.strip().upper()
    sectors = payload.get("search_sectors")
    if sectors is not None:
        if not isinstance(sectors, list):
            _invalid(("search_sectors",), "search_sectors는 섹터 이름 목록이어야 합니다")
        names = [_wire_sector(v, ("search_sectors", i)) for i, v in enumerate(sectors)]
        if len(set(names)) != len(names):
            _invalid(("search_sectors",), "같은 섹터를 두 번 적지 않습니다")
        out["search_sectors"] = sorted(names)
    if all(value is None for value in out.values()):
        _invalid((), "target, search_sectors, base 중 보고할 항목이 하나 이상 있어야 합니다")
    return out


def validate_m3(payload: Dict[str, Any]) -> Dict[str, Any]:
    """Validate 1–7 literal wire sectors, unique six-class objects and integer 0–99 counts."""
    payload = _body(payload)
    sectors = payload.get("sectors")
    if not isinstance(sectors, list) or not 1 <= len(sectors) <= 7:
        _invalid(("sectors",), "sectors는 섹터 1~7개 목록이어야 합니다")
    seen, normalised = set(), []
    kinds = {value.upper() for value in OBJECT_WIRE.values()}
    for i, item in enumerate(sectors):
        if not isinstance(item, dict):
            _invalid(("sectors", i), "각 섹터는 JSON 객체여야 합니다")
        name = _wire_sector(item.get("sector"), ("sectors", i, "sector"))
        if name in seen:
            _invalid(("sectors", i, "sector"), "같은 섹터를 두 번 적지 않습니다")
        seen.add(name)
        objects = item.get("objects", [])
        if not isinstance(objects, list):
            _invalid(("sectors", i, "objects"), "objects는 목록이어야 하며 객체가 없으면 []입니다")
        found, canonical = set(), []
        for j, obj in enumerate(objects):
            path = ("sectors", i, "objects", j)
            if not isinstance(obj, dict):
                _invalid(path, "각 객체는 JSON 객체여야 합니다")
            kind = obj.get("type")
            if not isinstance(kind, str) or kind.strip().upper() not in kinds:
                _invalid((*path, "type"), "객체 종류는 " + ", ".join(sorted(kinds)) + " 중 하나입니다")
            kind = kind.strip().upper()
            if kind in found:
                _invalid((*path, "type"), "한 섹터에 같은 종류는 한 번만 적습니다")
            found.add(kind)
            count = obj.get("count")
            if isinstance(count, bool) or not isinstance(count, int) or not 0 <= count <= 99:
                _invalid((*path, "count"), "count는 0~99 범위의 정수여야 합니다")
            canonical.append({"type": kind, "count": count})
        normalised.append({"sector": name, "objects": sorted(canonical, key=lambda obj: obj["type"])})
    return {"sectors": sorted(normalised, key=lambda item: WIRE_SECTORS.index(item["sector"]))}


def validate_m6(payload: Dict[str, Any], team: str) -> Dict[str, Any]:
    """M6 rank order is significant; only case is normalised."""
    payload = _body(payload)
    ranking = payload.get("ranking")
    if not isinstance(ranking, list) or not 1 <= len(ranking) <= 3:
        _invalid(("ranking",), "ranking은 상대 진영 호출부호 1~3개 목록이어야 합니다")
    if str(team).upper() not in ("B", "BLUE", "R", "RED"):
        _invalid((), "위험도 순위를 검증할 우리 진영이 확인되지 않았습니다")
    prefix = "R" if camp_of(team) == "BLUE" else "B"
    out = []
    for i, value in enumerate(ranking):
        if not isinstance(value, str) or not re.fullmatch(r"[BR]-[1-3]", value.strip().upper()):
            _invalid(("ranking", i), "호출부호는 B-1~B-3 또는 R-1~R-3 형식입니다; 섹터 이름이 아닙니다")
        value = value.strip().upper()
        if not value.startswith(prefix):
            # The source includes the offending callsign. It is safe to echo
            # only after the strict [BR]-[1-3] validation above, never raw input.
            _invalid(("ranking", i), f"위험도 순위는 상대 진영({prefix}-1~{prefix}-3)만 적습니다: {value}")
        if value in out:
            _invalid(("ranking", i), "같은 UGV를 두 번 적지 않습니다")
        out.append(value)
    return {"ranking": out}


def equal_received(kind: str, submitted: Dict[str, Any], received: Dict[str, Any], team: str) -> bool:
    """Compare wire content without depending on cosmetic ordering/case.

    M1 sector order and M3 sector/object order are irrelevant; M6 threat rank
    order remains meaningful. Missing/extra sectors or types (even zero-count
    types) differ. Missing M1 fields and explicit null are the same omission.
    The sent side also accepts the existing internal to_server input format.
    """
    try:
        sent = _body(submitted)
        if kind == "m1":
            if "goal_base" in sent or "sectors" in sent:
                sent = m1_to_server(sent, team)
            return validate_m1(sent) == validate_m1(received)
        if kind == "m3":
            if isinstance(sent.get("sectors"), dict):
                sent = m3_to_server(sent, team)
            return validate_m3(sent) == validate_m3(received)
        if kind == "m6":
            # Internal call signs are a compatibility input on the SENT side;
            # received wire data is validated strictly and never silently pruned.
            if any(isinstance(x, str) and "UGV" in x.upper() for x in sent.get("ranking", [])):
                sent = m6_to_server(sent, team)
            return validate_m6(sent, team) == validate_m6(received, team)
    except (ValidationError, ValueError, TypeError, AttributeError):
        return False
    return False


# ---- 서버 → 내부 -----------------------------------------------------------------------------
def parse_objective(code: str) -> Optional[str]:
    """M5 order.objective ('RED_MAINTENANCE') → 내부 거점 key ('MAINT_DEPOT'). 모르면 None."""
    s = str(code or "").strip().upper()
    for camp in ("BLUE_", "RED_"):
        if s.startswith(camp):
            return _REV_BASE.get(s[len(camp):])
    return _REV_BASE.get(s)


def objective_camp(code: str) -> Optional[str]:
    s = str(code or "").strip().upper()
    return "BLUE" if s.startswith("BLUE_") else "RED" if s.startswith("RED_") else None


def parse_slot(slot: Optional[str]):
    """/me 의 slot 'B-1' → ('BLUE', 1). 편성 밖(null)이면 None."""
    m = re.fullmatch(r"([BR])-?([1-3])", str(slot or "").strip().upper())
    return (("BLUE" if m.group(1) == "B" else "RED"), int(m.group(2))) if m else None


# ---- 서버 이벤트(대시보드 '주요 이벤트'), 시뮬/예전 가정 ---------------------------------------------
# 서버가 참가팀 API 로 이벤트를 주지 않는 것이 확인됐다(M2·M4·M5 는 장치가 판정하고 알려 주지 않는다). 시뮬의 이벤트 경로를 시험하려고 남겨 둔다.
_TYPE_ALIASES = {
    "game_state": "game_state", "state": "game_state", "state_change": "game_state", "match_state": "game_state", "경기상태": "game_state",
    "smog": "smog", "smoke": "smog", "fog": "smog", "스모그": "smog",
    "base_entered": "base_entered", "base_entry": "base_entered", "base_enter": "base_entered", "base_reached": "base_entered",
    "거점진입": "base_entered",
    "target_hit": "target_hit", "ugv_hit": "ugv_hit",
}
_STATE_ALIASES = {
    "ready": "ready", "prepare": "ready", "standby": "ready", "준비": "ready",
    "running": "running", "run": "running", "playing": "running", "in_progress": "running", "start": "running", "진행": "running",
    "finished": "finished", "finish": "finished", "end": "finished", "ended": "finished", "stop": "finished",
    "stopped": "finished", "complete": "finished", "completed": "finished", "done": "finished", "완료": "finished", "종료": "finished",
}


def normalize_event(data: Dict[str, Any]) -> Dict[str, Any]:
    """원본 이벤트 dict → 같은 dict 에 정규화한 type/state/on 을 덮어쓴 복사본. 모르는 type 은 그대로 둔다."""
    d = dict(data or {})
    t = str(d.get("type", d.get("event", ""))).strip().lower()
    d["type"] = _TYPE_ALIASES.get(t, _TYPE_ALIASES.get(str(d.get("type", "")).strip(), t))
    if d["type"] == "game_state":
        s = str(d.get("state", d.get("status", d.get("value", "")))).strip().lower()
        d["state"] = _STATE_ALIASES.get(s, _STATE_ALIASES.get(str(d.get("state", "")).strip(), s))
    elif d["type"] == "smog":
        v = d.get("on", d.get("active", d.get("state", d.get("value"))))
        d["on"] = v if isinstance(v, bool) else str(v).strip().lower() in ("1", "true", "on", "start", "active", "시작", "켜짐")
    return d
