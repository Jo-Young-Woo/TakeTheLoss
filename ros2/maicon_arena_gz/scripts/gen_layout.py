#!/usr/bin/env python3
"""Deterministic, constrained layouts derived from a user-supplied template.

The template contributes cluster topology, not a second set of arena dimensions.
Generation never rewrites the delivered ``layout_default.yaml``. ``--count`` is
the number of separate layouts, not the number of physical cubes.
"""
from __future__ import annotations

import argparse
from copy import deepcopy
import json
from pathlib import Path
import random

import yaml

from arena_common import (DECO_TYPES, FACE_DIRS, M3_TYPES, TARGET_TYPES,
                          accessible_faces, expand_boxes, load_layout, load_spec,
                          recompute_truth, targets_summary, validate_layout)

PACKAGE = Path(__file__).resolve().parents[1]


def _geometry_ok(spec: dict, layout: dict) -> bool:
    return validate_layout(spec, layout, require_targets=False, check_truth=False,
                           check_accessibility=False)["ok"]


def _clear_faces(layout: dict) -> None:
    for box in layout["boxes"]:
        box["faces"] = dict.fromkeys(FACE_DIRS, "blank")


def _normalise_hidden(spec: dict, layout: dict) -> None:
    expanded = expand_boxes(spec, layout)
    for physical in expanded:
        for face, value in physical["faces"].items():
            if value == "hidden" and all(b["faces"][face] == "hidden" for b in expanded
                                         if b["index"] == physical["index"]):
                layout["boxes"][physical["index"]]["faces"][face] = "hidden"


def _variant(rng: random.Random, value: str, faces_root: Path | None) -> str:
    if faces_root is None or ":" not in value:
        return value
    folder = faces_root / value.replace(":", "_")
    candidates = sorted(path.name for path in folder.glob("*")
                        if path.is_file() and path.suffix.lower() in (".jpg", ".jpeg", ".png"))
    return value + "@" + rng.choice(candidates) if candidates else value


def generate_layout(spec: dict, template: dict, seed: int = 2026,
                    *, faces_root: Path | None = None) -> dict:
    """Randomise valid grid translations, paired targets and accessible faces.

    Random translations preserve the supplied cluster shapes and cube count.
    Rejected moves retain the previous valid placement: there is no lowered
    clearance threshold or fallback to an unchecked random arrangement.
    """
    rng = random.Random(seed)
    layout = deepcopy(template)
    layout.pop("m3_ground_truth", None)
    layout.pop("targets_summary", None)
    _clear_faces(layout)
    _normalise_hidden(spec, layout)
    if not _geometry_ok(spec, layout):
        problems = validate_layout(spec, layout, require_targets=False,
                                   check_truth=False, check_accessibility=False)["errors"]
        raise ValueError("invalid layout template: " + "; ".join(problems))
    groups = {}
    for index, box in enumerate(layout["boxes"]):
        groups.setdefault(box["cluster"], []).append(index)
    # Pair geometry by mirrored coordinates, rather than assuming cluster names.
    pairs, used = [], set()
    for name, indices in groups.items():
        if name in used:
            continue
        points = {tuple(layout["boxes"][i]["center"]) for i in indices}
        mirrored = {(-x, y) for x, y in points}
        partner = next((other for other, js in groups.items() if other != name and other not in used
                        and {tuple(layout["boxes"][j]["center"]) for j in js} == mirrored), None)
        pairs.append((indices, groups[partner] if partner is not None else []))
        used.add(name)
        if partner is not None:
            used.add(partner)
    step = float(spec["floor"]["grid_cell"])
    accepted_moves = 0
    for _ in range(4):
        rng.shuffle(pairs)
        for indices, mirrored_indices in pairs:
            proposals = [(dx, dy) for dx in (-1, 0, 1) for dy in (-1, 0, 1) if dx or dy]
            rng.shuffle(proposals)
            for dx, dy in proposals:
                proposed = deepcopy(layout)
                for i in indices:
                    x, y = layout["boxes"][i]["center"]
                    proposed["boxes"][i]["center"] = [round(x + dx * step, 6), round(y + dy * step, 6)]
                for i in mirrored_indices:
                    x, y = layout["boxes"][i]["center"]
                    proposed["boxes"][i]["center"] = [round(x - dx * step, 6), round(y + dy * step, 6)]
                # A whole cluster must remain in its original labelled sector.
                _clear_faces(proposed)
                _normalise_hidden(spec, proposed)
                if _geometry_ok(spec, proposed):
                    layout = proposed
                    accepted_moves += 1
                    break
    access = accessible_faces(spec, layout)
    physical = expand_boxes(spec, layout)
    if access["blocked_spawns"]:
        raise ValueError("generated layout blocks a spawn")
    # One inward-facing target in each BLUE row; the RED counterpart is exact.
    kinds = list(TARGET_TYPES)
    rng.shuffle(kinds)
    for row, kind in enumerate(kinds, 1):
        candidates = []
        for box in physical:
            if box["sector"] != f"B-S{row}" or box["level"] != 0:
                continue
            x, y, _ = box["center"]
            mate = next((b for b in physical if b["sector"] == f"R-S{row}" and b["level"] == 0
                         and abs(b["center"][0] + x) < 1e-8 and abs(b["center"][1] - y) < 1e-8), None)
            if mate and access["views"][box["name"] + ":E"] and access["views"][mate["name"] + ":W"]:
                if layout["boxes"][box["index"]].get("stack", 1) == 1 and layout["boxes"][mate["index"]].get("stack", 1) == 1:
                    candidates.append((box, mate))
        if not candidates:
            raise ValueError(f"B-S{row}/R-S{row}: no mirrored accessible target faces")
        a, b = rng.choice(candidates)
        for box, face in ((a, "E"), (b, "W")):
            layout["boxes"][box["index"]]["faces"][face] = _variant(rng, "target:" + kind, faces_root)
    for index, box in enumerate(layout["boxes"]):
        layers = [b for b in physical if b["index"] == index]
        for face, value in box["faces"].items():
            if value == "hidden" or value.startswith("target:"):
                continue
            # All physical layers of a shared face assignment must be reachable.
            if not all(access["views"][b["name"] + ":" + face] for b in layers):
                box["faces"][face] = "blank"
                continue
            draw = rng.random()
            value = ("m3:" + rng.choice(M3_TYPES) if draw < .55 else
                     "deco:" + rng.choice(DECO_TYPES) if draw < .80 else "blank")
            box["faces"][face] = _variant(rng, value, faces_root)
    # At least one actual M3 image per populated sector, not only an empty cube.
    truth = recompute_truth(spec, layout)
    for sector, counts in truth.items():
        if counts:
            continue
        choices = [(b, f) for b in physical if b["sector"] == sector for f in FACE_DIRS
                   if access["views"][b["name"] + ":" + f]
                   and not layout["boxes"][b["index"]]["faces"][f].startswith("target:")]
        if not choices:
            raise ValueError(f"{sector}: no accessible M3 face")
        box, face = rng.choice(choices)
        layout["boxes"][box["index"]]["faces"][face] = _variant(rng, "m3:" + rng.choice(M3_TYPES), faces_root)
    layout["meta"] = {
        "seed": int(seed), "source": "supplied template topology + constrained seeded grid translations and face assignment",
        "confidence": "assumed", "accepted_translation_moves": accepted_moves,
        "face_keys": "N=+Y E=+X S=-Y W=-X; hidden/blank are black; type@filename selects a class-folder image",
        "assumptions": {"mirrored_target_types": True, "default_stack_layers": 1,
                        "reachability_disc": "UGV circumscribed radius + grid_cell/20",
                        "reachability_resolution": "grid_cell/8",
                        "optical_visibility": "planar access only; verify camera height/pitch for stacks"},
    }
    layout["targets_summary"] = targets_summary(spec, layout)
    layout["m3_ground_truth"] = recompute_truth(spec, layout)
    report = validate_layout(spec, layout)
    if not report["ok"]:
        raise ValueError("generated layout failed validation: " + "; ".join(report["errors"]))
    layout["meta"]["clearance_check"] = "OK: geometry, hidden faces, target mirrors, truth and robot access verified"
    return layout


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--spec", type=Path, default=PACKAGE / "config/arena_spec.yaml")
    parser.add_argument("--template", type=Path, default=PACKAGE / "config/layouts/layout_default.yaml")
    parser.add_argument("--output", "-o", type=Path, required=True)
    parser.add_argument("--seed", type=int, default=2026)
    parser.add_argument("--count", type=int, default=1, help="number of independent layout YAML files")
    parser.add_argument("--faces-root", type=Path, default=PACKAGE / "media/textures/faces")
    args = parser.parse_args(argv)
    if args.count < 1:
        parser.error("--count must be positive")
    if args.count > 1 and args.output.suffix.lower() in (".yaml", ".yml"):
        parser.error("--count > 1 requires an output directory")
    spec, template = load_spec(args.spec), load_layout(args.template)
    results = []
    for offset in range(args.count):
        seed = args.seed + offset
        output = (args.output if args.count == 1 and args.output.suffix.lower() in (".yaml", ".yml")
                  else args.output / f"layout_seed_{seed}.yaml")
        if output.resolve() == args.template.resolve() or output.name == "layout_default.yaml":
            parser.error("the supplied layout_default/template is immutable; choose a new output name")
        generated = generate_layout(spec, template, seed, faces_root=args.faces_root)
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(yaml.safe_dump(generated, allow_unicode=True, sort_keys=False, width=120), encoding="utf-8")
        results.append({"file": str(output), "seed": seed, "cubes": len(expand_boxes(spec, generated))})
    print(json.dumps({"ok": True, "layouts": results}, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
