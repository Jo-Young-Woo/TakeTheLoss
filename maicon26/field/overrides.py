"""현장 실험 결과를 Config 에 덮어쓰는 파일(config/field_overrides.json)과 그 입출력.

원본 기본값(config.py)은 건드리지 않는다. 실험이 값을 제안하면 사람이 확인하고 `scripts/field_exp.py apply` 로 이 파일에 쓴다.
각 값에는 어느 실험·언제·메모가 같이 남고(provenance), 되돌릴 수 있다(history).
실행 쪽은 `load_config(base, overrides)` 또는 scripts 의 `--overrides` 로 적용한다.

파일 형식:
{"values": {"map3d.nav_thr": 2.2, ...},
 "provenance": {"map3d.nav_thr": {"exp": "F06", "date": "2026-11-07", "note": "...", "record": "field/records/.../F06_*.json"}},
 "history": [{"path": ..., "old": ..., "new": ..., "date": ...}, ...]}
"""
from __future__ import annotations

import copy
import datetime as _dt
import json
import os
from typing import Any, Dict, List, Optional

from ..config import Config

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
DEFAULT_PATH = os.path.join(ROOT, "config", "field_overrides.json")


def empty() -> Dict[str, Any]:
    return {"values": {}, "provenance": {}, "history": []}


def load(path: Optional[str] = None) -> Dict[str, Any]:
    path = path or DEFAULT_PATH
    if not os.path.exists(path):
        return empty()
    with open(path, "r", encoding="utf-8") as f:
        d = json.load(f)
    for k, v in empty().items():
        d.setdefault(k, copy.deepcopy(v))
    return d


def save(data: Dict[str, Any], path: Optional[str] = None) -> str:
    path = path or DEFAULT_PATH
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", encoding="utf-8", newline="\n") as f:
        f.write(json.dumps(data, ensure_ascii=False, indent=2) + "\n")
    return path


def _coerce(cur: Any, value: Any) -> Any:
    if isinstance(cur, tuple) and isinstance(value, (list, tuple)):
        return tuple(value)
    if isinstance(cur, bool):
        return bool(value)
    if isinstance(cur, float) and isinstance(value, int) and not isinstance(value, bool):
        return float(value)
    return value


def set_by_path(cfg: Any, path: str, value: Any) -> Any:
    """Config 의 점 경로에 값을 넣는다(없는 경로면 KeyError). 이전 값을 돌려준다."""
    ks = path.split(".")
    cur = cfg
    for k in ks[:-1]:
        if not hasattr(cur, k):
            raise KeyError(f"알 수 없는 설정 경로: {path}")
        cur = getattr(cur, k)
    if not hasattr(cur, ks[-1]):
        raise KeyError(f"알 수 없는 설정 경로: {path}")
    old = getattr(cur, ks[-1])
    setattr(cur, ks[-1], _coerce(old, value))
    return old


def apply_to_config(cfg: Config, data: Dict[str, Any]) -> List[str]:
    """오버라이드 값을 cfg 에 적용. 적용한 경로 목록."""
    done = []
    for p, v in data.get("values", {}).items():
        set_by_path(cfg, p, v)
        done.append(p)
    return done


def load_config(base: Optional[str] = None, overrides: Optional[str] = DEFAULT_PATH) -> Config:
    """base(config.json/yaml, 없으면 기본값) 위에 현장 오버라이드를 얹은 Config."""
    cfg = Config.load(base)
    if overrides and os.path.exists(overrides):
        apply_to_config(cfg, load(overrides))
    return cfg


def record(data: Dict[str, Any], cfg: Config, path: str, value: Any, exp: str, note: str = "",
           record_file: str = "", date: Optional[str] = None) -> None:
    """값 하나를 오버라이드에 기록한다(이전 값을 history 에). path 가 Config 에 없으면 KeyError."""
    probe = copy.deepcopy(cfg)
    old = set_by_path(probe, path, value)                 # 경로·형식 검증
    new = getattr(_walk(probe, path.split(".")[:-1]), path.split(".")[-1])
    d = date or _dt.date.today().isoformat()
    jv = list(new) if isinstance(new, tuple) else new
    jo = list(old) if isinstance(old, tuple) else old
    prev = data["values"].get(path, jo)
    data["values"][path] = jv
    data["provenance"][path] = {"exp": exp, "date": d, "note": note, "record": record_file}
    data["history"].append({"path": path, "old": prev, "new": jv, "date": d, "exp": exp})


def revert(data: Dict[str, Any], path: str) -> bool:
    """오버라이드를 지워 기본값으로 되돌린다."""
    had = path in data["values"]
    data["values"].pop(path, None)
    data["provenance"].pop(path, None)
    if had:
        data["history"].append({"path": path, "old": None, "new": "(기본값 복귀)", "date": _dt.date.today().isoformat(), "exp": "revert"})
    return had


def _walk(cfg: Any, ks: List[str]) -> Any:
    cur = cfg
    for k in ks:
        cur = getattr(cur, k)
    return cur
