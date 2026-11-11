#!/usr/bin/env python3
"""Plot the measured Gazebo trajectory and referee layout from a trial report."""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path


def is_cube_candidate(item: dict) -> bool:
    """Select by footprint only; a top view cannot identify side pictures.

    The plot uses the report's arena geometry for reference, not as planner
    input. Never use its referee-only ``m3`` label to select or colour cubes.
    """
    return all(math.isclose(float(item[axis]), .4, abs_tol=.065)
               for axis in ("sx", "sy"))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("report", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    data = json.loads(args.report.read_text(encoding="utf-8"))
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Rectangle
    from matplotlib.collections import LineCollection
    import numpy as np

    arena = data["arena"]
    fig, ax = plt.subplots(figsize=(10, 8))
    fig.patch.set_facecolor("#f5f7fa")
    ax.set_facecolor("#edf0f4")
    count = 0
    for item in arena["structures"]:
        candidate = is_cube_candidate(item)
        color = "#e59a34" if candidate else "#4c596b"
        ax.add_patch(Rectangle((item["cx"] - item["sx"] / 2,
                                item["cy"] - item["sy"] / 2),
                               item["sx"], item["sy"], facecolor=color,
                               edgecolor="white", lw=1.3, zorder=3))
        if candidate:
            count += 1
            ax.text(item["cx"], item["cy"], f"C{count}", ha="center", va="center",
                    color="#17212f", fontsize=8, weight="bold", zorder=4)
    path = data["B2_trajectory"]
    samples = [p for p in path if p["t"] < data["base_command_at_s"] - 18]
    if len(samples) > 1:
        points = np.array([(p["x"], p["y"]) for p in samples])
        lines = np.stack([points[:-1], points[1:]], axis=1)
        lc = LineCollection(lines, cmap="viridis", linewidth=2.2, zorder=5)
        lc.set_array(np.array([p["t"] for p in samples[:-1]]))
        ax.add_collection(lc)
        fig.colorbar(lc, ax=ax, label="Match time (s)", shrink=.75)
        ax.plot(points[0, 0], points[0, 1], "o", color="#2076bf", ms=9, zorder=6)
        ax.plot(points[-1, 0], points[-1, 1], "s", color="#d63c42", ms=7, zorder=6)
    score = data["agents"]["B2"].get("m3") or {}
    ax.set_title(f"MAICON M3 / seed {data['seed']} / {count} cube inspection candidates\n"
                 f"Path before M5 staging | final count recall {score.get('recall', 0):.0%}, "
                 f"precision {score.get('precision', 0):.0%}", pad=15)
    ax.set(xlim=(0, arena["width"]), ylim=(0, arena["height"]),
           xlabel="Arena x (m)", ylabel="Arena y (m)", aspect="equal")
    ax.grid(alpha=.25)
    ax.text(0, -.16, "Orange: all 0.4 m cubes (side contents unknown from above)  |  Grey: rectangles\n"
            "Layout: simulation reference  |  Path: measured Gazebo positions\n"
            "M3 classifications are simulated UGV side detections; this is not a real-camera accuracy test.",
            transform=ax.transAxes, fontsize=9, color="#536071")
    fig.subplots_adjust(bottom=.18, top=.82)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.output, dpi=160)
    plt.close(fig)


if __name__ == "__main__":
    main()
