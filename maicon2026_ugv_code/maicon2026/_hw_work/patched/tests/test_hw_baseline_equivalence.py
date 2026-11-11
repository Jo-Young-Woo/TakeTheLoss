"""하드웨어 반영 수정 후에도 기본 동작이 하나도 바뀌지 않았는지 — 수정 전에 저장한 기준값과 비교한다.

기준값: tests/fixtures/hw_baseline/*.json (scripts/capture_hw_baseline.py 가 수정 전에 한 번 만들었다. 덮어쓰지 않는다).
시뮬레이션을 쓰지 않는다. 실수는 1e-12 안에서, 그 밖은 정확히 같아야 한다.
"""
import json
import math
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
import capture_hw_baseline as cap  # noqa: E402

FIX = ROOT / "tests" / "fixtures" / "hw_baseline"
TOL = 1e-12


def assert_same(a, b, path="$"):
    if isinstance(a, bool) or isinstance(b, bool) or a is None or b is None:
        assert a == b, f"{path}: {a!r} != {b!r}"
    elif isinstance(a, (int, float)) and isinstance(b, (int, float)):
        if isinstance(a, float) or isinstance(b, float):
            assert math.isclose(a, b, rel_tol=TOL, abs_tol=TOL), f"{path}: {a!r} != {b!r}"
        else:
            assert a == b, f"{path}: {a!r} != {b!r}"
    elif isinstance(a, dict) and isinstance(b, dict):
        assert set(a) == set(b), f"{path}: keys {sorted(set(a) ^ set(b))}"
        for k in a:
            assert_same(a[k], b[k], f"{path}.{k}")
    elif isinstance(a, (list, tuple)) and isinstance(b, (list, tuple)):
        assert len(a) == len(b), f"{path}: len {len(a)} != {len(b)}"
        for i, (x, y) in enumerate(zip(a, b)):
            assert_same(x, y, f"{path}[{i}]")
    else:
        assert a == b, f"{path}: {a!r} != {b!r}"


@pytest.mark.parametrize("name", sorted(cap.COLLECTORS))
def test_default_behaviour_matches_saved_baseline(name):
    saved = json.loads((FIX / f"{name}.json").read_text(encoding="utf-8"))
    now = json.loads(json.dumps(cap._jsonable(cap.COLLECTORS[name]())))
    assert_same(now, saved)


def test_frozen_runtime_hashes_unchanged_when_runtime_is_present():
    saved = json.loads((FIX / "frozen_runtime_sha256.json").read_text(encoding="utf-8"))
    now = cap.frozen_runtime_hashes()
    if not now["present"]:
        pytest.skip("runtime/selected_candidate02 가 이 PC 에 없다 — 동결 런타임이 있는 PC 에서 --check-frozen 으로 확인")
    assert now["files"] == saved["files"]


def test_baseline_files_are_complete():
    expected = set(cap.COLLECTORS) | {"frozen_runtime_sha256"}
    assert expected == {p.stem for p in FIX.glob("*.json")}
