#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
from pathlib import Path

import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle


def read_xy(path: Path) -> tuple[list[float], list[float]]:
    xs: list[float] = []
    ys: list[float] = []
    with path.open(newline="", encoding="utf-8") as handle:
        for row in csv.DictReader(handle):
            xs.append(float(row["x_m"]))
            ys.append(float(row["y_m"]))
    return xs, ys


def main() -> None:
    parser = argparse.ArgumentParser(description="Plot the deterministic differential-drive demo.")
    parser.add_argument("trajectory", type=Path)
    parser.add_argument("planned_path", type=Path)
    parser.add_argument("map", type=Path)
    parser.add_argument("--output", type=Path, default=Path("trajectory.png"))
    args = parser.parse_args()

    traj_x, traj_y = read_xy(args.trajectory)
    path_x, path_y = read_xy(args.planned_path)

    occupied: list[tuple[int, int, float]] = []
    with args.map.open(newline="", encoding="utf-8") as handle:
        for row in csv.DictReader(handle):
            if int(row["occupied"]) == 1:
                occupied.append((int(row["x_cell"]), int(row["y_cell"]), float(row["resolution_m"])))

    fig, ax = plt.subplots(figsize=(9, 6))
    for x_cell, y_cell, resolution in occupied:
        ax.add_patch(Rectangle((x_cell * resolution, y_cell * resolution), resolution, resolution, alpha=0.25))

    ax.plot(path_x, path_y, linestyle="--", linewidth=1.5, label="A* planned path")
    ax.plot(traj_x, traj_y, linewidth=2.2, label="closed-loop trajectory")
    ax.scatter([traj_x[0]], [traj_y[0]], marker="o", s=70, label="start")
    ax.scatter([path_x[-1]], [path_y[-1]], marker="x", s=90, label="goal")
    ax.set_title("robotics-control-core: plan → track → goal")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.axis("equal")
    ax.grid(True, alpha=0.25)
    ax.legend()
    fig.tight_layout()
    fig.savefig(args.output, dpi=170)


if __name__ == "__main__":
    main()
