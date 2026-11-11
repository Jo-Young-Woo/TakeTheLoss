from pathlib import Path

from scripts.check_traceability import check, code_reference_exists


def test_all_specification_rules_have_real_code_and_marked_tests():
    errors, stats = check()
    assert errors == []
    assert stats["rows"] == stats["expected"] == 97


def test_traceability_rejects_nonexistent_symbols_and_files():
    root = Path(__file__).resolve().parents[1]
    assert code_reference_exists(root, "maicon26/link/schema.py:equal_received")
    assert not code_reference_exists(root, "maicon26/link/schema.py:invented_function")
    assert not code_reference_exists(root, "missing.py:equal_received")
