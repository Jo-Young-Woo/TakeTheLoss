#!/usr/bin/env python3
"""Verify the exported candidate 02 Python sources without ROS or Gazebo."""
from pathlib import Path
import hashlib
import json
import sys

root = Path(__file__).resolve().parent
expected = json.loads((root / "source_hashes.json").read_text(encoding="utf-8"))
actual = {
    path.relative_to(root).as_posix(): hashlib.sha256(path.read_bytes()).hexdigest()
    for path in sorted((root / "maicon26").rglob("*.py"))
}
changed = sorted(
    key for key in actual.keys() | expected.keys()
    if actual.get(key) != expected.get(key)
)
if changed:
    print("Source mismatch: " + ", ".join(changed[:12]), file=sys.stderr)
    raise SystemExit(1)
print(f"Candidate 02 source verified: {len(actual)} Python files")
