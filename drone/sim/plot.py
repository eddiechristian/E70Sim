#!/usr/bin/env python3
"""Plot run.csv from the rate loop simulation.

Explained in Stage1-c.md in the repo root.

Usage:
    cargo run > run.csv
    python3 plot.py [run.csv] [picture.png]

To compare pictures, give them the same axes:
    python3 plot.py run.csv run.png --rate-range -15 70 --command-range 0.45 0.95

Needs matplotlib: sudo apt install python3-matplotlib
"""

import argparse
import csv
import os

import matplotlib.pyplot as plt

COLUMNS = ["time", "target", "rate", "angle", "output", "left", "right"]

# What the PID was told: the rate with noise and delay. Older files lack it.
OPTIONAL_COLUMNS = ["measured"]


def read_run(path):
    """Returns a dict of column name to list of floats."""
    with open(path, newline="") as f:
        reader = csv.DictReader(f)
        names = COLUMNS + [n for n in OPTIONAL_COLUMNS if n in reader.fieldnames]
        run = {name: [] for name in names}
        for row in reader:
            for name in names:
                run[name].append(float(row[name]))
    return run


def main():
    parser = argparse.ArgumentParser(description="Plot a CSV from the rate loop simulation.")
    parser.add_argument("csv", nargs="?", default="run.csv", help="file to plot")
    parser.add_argument("picture", nargs="?", help="save a picture here instead of opening a window")
    parser.add_argument("--rate-range", nargs=2, type=float, metavar=("MIN", "MAX"),
                        help="fixed vertical range of the top panel, in °/s")
    parser.add_argument("--command-range", nargs=2, type=float, metavar=("MIN", "MAX"),
                        help="fixed vertical range of the bottom panel")
    parser.add_argument("--title", help="title of the graph; the file name if not given")
    args = parser.parse_args()

    path = args.csv
    run = read_run(path)
    time = run["time"]

    # Two panels, one above the other, sharing the time axis.
    fig, (top, bottom) = plt.subplots(2, 1, sharex=True, figsize=(9, 6))

    # Top: target and rate, with the measured rate as a pale line behind.
    if "measured" in run:
        top.plot(time, run["measured"], label="measured", color="lightblue", linewidth=0.5)
    top.plot(time, run["target"], label="target", color="gray", linestyle="--")
    top.plot(time, run["rate"], label="rate")
    top.set_ylabel("rate (°/s)")
    top.set_title(args.title or f"Rate loop on the seesaw: {os.path.basename(path)}")
    if args.rate_range:
        top.set_ylim(*args.rate_range)
    top.legend(loc="upper right")
    top.grid(True)

    # Bottom: the two motor commands.
    bottom.plot(time, run["left"], label="left")
    bottom.plot(time, run["right"], label="right")
    bottom.set_ylabel("motor command (0 to 1)")
    bottom.set_xlabel("time (s)")
    if args.command_range:
        bottom.set_ylim(*args.command_range)
    bottom.legend(loc="upper right")
    bottom.grid(True)

    fig.tight_layout()

    # With a second argument, save a picture instead of opening a window.
    if args.picture:
        fig.savefig(args.picture, dpi=120)
    else:
        plt.show()


if __name__ == "__main__":
    main()
