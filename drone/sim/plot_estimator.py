#!/usr/bin/env python3
"""Plot the estimator's angles against the true ones, from quad.csv.

Usage:
    cargo run --bin quad > quad.csv
    python3 plot_estimator.py [quad.csv] [picture.png]

Needs matplotlib: sudo apt install python3-matplotlib
"""

import argparse
import csv
import os

AXES = ["roll", "pitch"]


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
    parser = argparse.ArgumentParser(description="Plot estimated against true angles.")
    parser.add_argument("csv", nargs="?", default="quad.csv", help="file to plot")
    parser.add_argument("picture", nargs="?", help="save a picture here instead of opening a window")
    parser.add_argument("--title", help="title of the graph; the file name if not given")
    parser.add_argument("--error-range", nargs=2, type=float, metavar=("MIN", "MAX"),
                        help="fixed vertical range of the error panel, in degrees")
    args = parser.parse_args()

    # Imported here so that a missing matplotlib does not stop --help.
    import matplotlib.pyplot as plt

    path = args.csv
    run = read_run(path)
    time = run["time"]

    # Three panels sharing the time axis: roll, pitch, then the error of both.
    fig, panels = plt.subplots(3, 1, sharex=True, figsize=(9, 8))

    for panel, axis in zip(panels, AXES):
        panel.plot(time, run[f"{axis}_angle"], label="true", color="gray", linewidth=2.5)
        panel.plot(time, run[f"{axis}_estimate"], label="estimate", linewidth=1)
        panel.set_ylabel(f"{axis} (°)")
        panel.legend(loc="upper left")
        panel.grid(True)
    panels[0].set_title(args.title or f"Estimator on the quad: {os.path.basename(path)}")

    bottom = panels[2]
    for axis in AXES:
        error = [e - t for e, t in zip(run[f"{axis}_estimate"], run[f"{axis}_angle"])]
        bottom.plot(time, error, label=f"{axis} error", linewidth=1)
    bottom.set_ylabel("estimate − true (°)")
    bottom.set_xlabel("time (s)")
    if args.error_range:
        bottom.set_ylim(*args.error_range)
    bottom.legend(loc="upper left")
    bottom.grid(True)

    fig.tight_layout()

    # With a second argument, save a picture instead of opening a window.
    if args.picture:
        fig.savefig(args.picture, dpi=120)
    else:
        plt.show()


if __name__ == "__main__":
    main()
