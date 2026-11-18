#!/usr/bin/env python3
"""Plot run.csv from the rate loop simulation.

Explained in Stage1-c.md in the repo root.

Usage:
    cargo run > run.csv
    python3 plot.py [run.csv] [picture.png]

Needs matplotlib: pip install --user matplotlib
"""

import csv
import sys

import matplotlib.pyplot as plt

COLUMNS = ["time", "target", "rate", "angle", "output", "left", "right"]


def read_run(path):
    """Returns a dict of column name to list of floats."""
    run = {name: [] for name in COLUMNS}
    with open(path, newline="") as f:
        for row in csv.DictReader(f):
            for name in COLUMNS:
                run[name].append(float(row[name]))
    return run


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else "run.csv"
    run = read_run(path)
    time = run["time"]

    # Two panels, one above the other, sharing the time axis.
    fig, (top, bottom) = plt.subplots(2, 1, sharex=True, figsize=(9, 6))

    # Top: target and rate.
    top.plot(time, run["target"], label="target", color="gray", linestyle="--")
    top.plot(time, run["rate"], label="rate")
    top.set_ylabel("rate (°/s)")
    top.set_title(f"Rate loop on the seesaw: {path}")
    top.legend()
    top.grid(True)

    # Bottom: the two motor commands.
    bottom.plot(time, run["left"], label="left")
    bottom.plot(time, run["right"], label="right")
    bottom.set_ylabel("motor command (0 to 1)")
    bottom.set_xlabel("time (s)")
    bottom.legend()
    bottom.grid(True)

    fig.tight_layout()

    # With a second argument, save a picture instead of opening a window.
    if len(sys.argv) > 2:
        fig.savefig(sys.argv[2], dpi=120)
    else:
        plt.show()


if __name__ == "__main__":
    main()
