#!/usr/bin/env python3
"""Validate specification rows against real code and pytest spec decorators."""
from __future__ import annotations

import argparse
import ast
from pathlib import Path
import re


ROOT = Path(__file__).resolve().parents[1]
SID = re.compile(r"^S(?:\d+|A|R)\.\d+$")


def test_markers(root: Path):
    found = {}
    for path in sorted((root/"tests").rglob("test_*.py")):
        tree = ast.parse(path.read_text(encoding="utf-8-sig"), filename=str(path))
        for node in ast.walk(tree):
            if not isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)):
                continue
            ids = set()
            for dec in node.decorator_list:
                if isinstance(dec, ast.Call) and isinstance(dec.func, ast.Attribute) and dec.func.attr == "spec":
                    for arg in dec.args:
                        if isinstance(arg, ast.Constant) and isinstance(arg.value, str):
                            ids.add(arg.value)
            if ids:
                found[f"{path.relative_to(root).as_posix()}::{node.name}"] = ids
    return found


def code_reference_exists(root, reference):
    match = re.fullmatch(r"([^\s`:]+\.py):([A-Za-z_][A-Za-z_0-9.]*)", reference)
    if not match:
        return False
    path = root/match[1]
    if not path.is_file():
        return False
    tree = ast.parse(path.read_text(encoding="utf-8-sig"))
    names = {n.name for n in ast.walk(tree) if isinstance(n, (ast.ClassDef, ast.FunctionDef, ast.AsyncFunctionDef))}
    names.update(n.id for n in ast.walk(tree) if isinstance(n, ast.Name) and isinstance(n.ctx, ast.Store))
    return all(part in names for part in match[2].split("."))


def check(root=ROOT):
    expected = [s.strip() for s in (root/"docs/spec_ids.txt").read_text(encoding="utf-8-sig").splitlines() if s.strip()]
    errors = []
    if len(expected) != len(set(expected)) or any(not SID.fullmatch(s) for s in expected):
        errors.append("spec_ids.txt contains duplicate or invalid identifiers")
    table = root/"docs/API_TRACEABILITY.md"
    if not table.is_file():
        return ["docs/API_TRACEABILITY.md is missing"], {"expected": len(expected), "rows": 0}
    body = table.read_text(encoding="utf-8-sig")
    rows = {}
    for line in body.splitlines():
        cells = [v.strip().strip("`") for v in line.strip().strip("|").split("|")]
        if cells and SID.fullmatch(cells[0]):
            if cells[0] in rows:
                errors.append(f"Duplicate traceability row: {cells[0]}")
            rows[cells[0]] = cells
    if set(rows) != set(expected):
        errors.append(f"Traceability identifiers differ: missing={sorted(set(expected)-set(rows))}, extra={sorted(set(rows)-set(expected))}")
    marked = test_markers(root)
    for reference, ids in marked.items():
        if ids-set(expected):
            errors.append(f"Unknown marker identifiers in {reference}: {sorted(ids-set(expected))}")
    for sid, cells in rows.items():
        if len(cells) != 6:
            errors.append(f"{sid}: expected six table columns")
            continue
        if cells[2] not in ("구현", "규칙", "해당없음"):
            errors.append(f"{sid}: invalid implementation status {cells[2]}")
        if cells[2] == "해당없음":
            continue
        refs = re.findall(r"[\w./-]+\.py:[A-Za-z_][A-Za-z_0-9.]*", cells[3])
        if not refs or not all(code_reference_exists(root, ref) for ref in refs):
            errors.append(f"{sid}: missing/invalid code reference")
        tests = re.findall(r"tests/[\w./-]+\.py::[A-Za-z_][A-Za-z_0-9]*", cells[4])
        if not tests or any(sid not in marked.get(ref, set()) for ref in tests):
            errors.append(f"{sid}: test reference missing or its spec marker does not match")
    classified = body.split("## 미분류 문장", 1)
    if len(classified) != 2 or classified[1].strip() != "없음.":
        errors.append("Unclassified source sentences remain or their section is missing")
    return errors, {"expected": len(expected), "rows": len(rows),
                    "marked_ids": len(set().union(*marked.values())) if marked else 0,
                    "marked_tests": len(marked)}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, default=ROOT)
    args = parser.parse_args(argv)
    errors, stats = check(args.root)
    for error in errors:
        print(error)
    print("Traceability:", stats, "PASS" if not errors else "FAIL")
    return 1 if errors else 0


if __name__ == "__main__":
    raise SystemExit(main())
