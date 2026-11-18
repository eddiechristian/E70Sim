#!/usr/bin/env python3
"""Print how far the estimator's angles are from the true ones in a quad run.

Usage:
    python3 measure_estimator.py quad.csv [more.csv ...]

For roll and for pitch, with "error" meaning the estimate minus the true angle,
in degrees:

    start   the error of the very first estimate, which is one accelerometer reading
    worst   the largest error, either way, over the last second of the run
    end     the average error over the last second
    wobble  standard deviation of the error over the last second
"""

import csv
import math
import sys

AXES = ["roll", "pitch"]


def read_run(path):
    with open(path, newline="") as f:
        return [{k: float(v) for k, v in row.items()} for row in csv.DictReader(f)]


def std_dev(values):
    mean = sum(values) / len(values)
    return math.sqrt(sum((v - mean) ** 2 for v in values) / len(values))


def measure(run, axis):
    errors = [r[f"{axis}_estimate"] - r[f"{axis}_angle"] for r in run]
    end_time = run[-1]["time"]
    last_second = [e for r, e in zip(run, errors) if r["time"] > end_time - 1.0]
    average = sum(last_second) / len(last_second)
    return errors[0], max(abs(e) for e in last_second), average, std_dev(last_second)


def main():
    columns = f"{'start':>7s} {'worst':>7s} {'end':>7s} {'wobble':>7s}"
    print(f"{'file':30s} roll: {columns}   pitch: {columns}")
    for path in sys.argv[1:]:
        run = read_run(path)
        roll = measure(run, "roll")
        pitch = measure(run, "pitch")
        figures = lambda m: " ".join(f"{v:6.3f}°" for v in m)
        print(f"{path:30s}       {figures(roll)}          {figures(pitch)}")


if __name__ == "__main__":
    main()
