#!/usr/bin/env python3
"""Plot quad.csv from the quad rate loop simulation.

Usage:
    cargo run --bin quad > quad.csv
    python3 plot_quad.py [quad.csv] [picture.png]

Needs matplotlib: sudo apt install python3-matplotlib
"""

import argparse
import csv
import os

AXES = ["roll", "pitch", "yaw"]
MOTORS = ["fl", "fr", "br", "bl"]


def read_run(path):
    """Returns a dict of column name to list of floats."""
    with open(path, newline="") as f:
        reader = csv.DictReader(f)
        run = {name: [] for name in reader.fieldnames}
        for row in reader:
            for name, value in row.items():
                run[name].append(float(value))
    return run


def main():
    parser = argparse.ArgumentParser(description="Plot a CSV from the quad rate loop simulation.")
    parser.add_argument("csv", nargs="?", default="quad.csv", help="file to plot")
    parser.add_argument("picture", nargs="?", help="save a picture here instead of opening a window")
    parser.add_argument("--title", help="title of the graph; the file name if not given")
    args = parser.parse_args()

    # Imported here so that a missing matplotlib does not stop --help.
    import matplotlib.pyplot as plt

    path = args.csv
    run = read_run(path)
    time = run["time"]

    # Four panels, one above the other, sharing the time axis: a rate panel
    # per axis, then the four motor commands.
    fig, panels = plt.subplots(4, 1, sharex=True, figsize=(9, 10))

    for panel, axis in zip(panels, AXES):
        panel.plot(time, run[f"{axis}_measured"], label="measured", color="lightblue", linewidth=0.5)
        panel.plot(time, run[f"{axis}_target"], label="target", color="gray", linestyle="--")
        panel.plot(time, run[f"{axis}_rate"], label="rate")
        panel.set_ylabel(f"{axis} rate (°/s)")
        panel.legend(loc="upper right")
        panel.grid(True)
    panels[0].set_title(args.title or f"Rate loops on the quad: {os.path.basename(path)}")

    bottom = panels[3]
    for motor in MOTORS:
        bottom.plot(time, run[motor], label=motor)
    bottom.set_ylabel("motor command (0 to 1)")
    bottom.set_xlabel("time (s)")
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
