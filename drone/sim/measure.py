#!/usr/bin/env python3
"""Print the figures used to judge a run of the rate loop simulation.

Usage:
    python3 measure.py run.csv [more.csv ...]

All figures are for the step up to the target, on the true rate:

    rise      seconds from the step until the rate first reaches 90% of the target
    overshoot how far the rate passes the target, as a percentage of it
    settle    seconds from the step until the rate stays within 5% of the target
    jitter    standard deviation of the right motor command over the last 0.3 s
              of the step, when it should be still
    swing     largest minus smallest right motor command over the same 0.3 s
"""

import csv
import math
import sys


def read_run(path):
    with open(path, newline="") as f:
        return [{k: float(v) for k, v in row.items()} for row in csv.DictReader(f)]


def std_dev(values):
    mean = sum(values) / len(values)
    return math.sqrt(sum((v - mean) ** 2 for v in values) / len(values))


def measure(run):
    # The step is where the target is not zero.
    step = [r for r in run if r["target"] != 0.0]
    start = step[0]["time"]
    end = step[-1]["time"]
    target = step[0]["target"]

    rise = next((r["time"] - start for r in step if r["rate"] >= 0.9 * target), float("nan"))
    overshoot = (max(r["rate"] for r in step) - target) / target * 100.0

    outside = [r["time"] for r in step if abs(r["rate"] - target) > 0.05 * target]
    settle = max(outside) - start if outside else 0.0

    hold = [r["right"] for r in step if r["time"] > end - 0.3]
    return rise, overshoot, settle, std_dev(hold), max(hold) - min(hold)


def main():
    print(f"{'file':40s} {'rise':>7s} {'overshoot':>9s} {'settle':>7s} {'jitter':>7s} {'swing':>6s}")
    for path in sys.argv[1:]:
        rise, overshoot, settle, jitter, swing = measure(read_run(path))
        print(f"{path:40s} {rise:6.3f}s {overshoot:8.1f}% {settle:6.3f}s {jitter:7.4f} {swing:6.3f}")


if __name__ == "__main__":
    main()
