#!/usr/bin/env python3
"""Print the figures used to judge a run of the quad rate loop simulation.

Usage:
    python3 measure_quad.py quad.csv [more.csv ...]

The axis that was stepped is found from the targets. The figures are for the
step up to the target, on the true rate, as in measure.py:

    rise      seconds from the step until the rate first reaches 90% of the target
    overshoot how far the rate passes the target, as a percentage of it
    settle    seconds from the step until the rate stays within 5% of the target
    others    largest rate reached by either of the two axes held at 0, in °/s
    jitter    standard deviation of the front-left motor command over the last
              0.3 s of the step, when it should be still
    motors    smallest and largest motor command of the whole run
"""

import csv
import math
import sys

AXES = ["roll", "pitch", "yaw"]
MOTORS = ["fl", "fr", "br", "bl"]


def read_run(path):
    with open(path, newline="") as f:
        return [{k: float(v) for k, v in row.items()} for row in csv.DictReader(f)]


def std_dev(values):
    mean = sum(values) / len(values)
    return math.sqrt(sum((v - mean) ** 2 for v in values) / len(values))


def measure(run):
    # The stepped axis is the one whose target is not always zero.
    axis = next(a for a in AXES if any(r[f"{a}_target"] != 0.0 for r in run))
    step = [r for r in run if r[f"{axis}_target"] != 0.0]
    start = step[0]["time"]
    end = step[-1]["time"]
    target = step[0][f"{axis}_target"]
    rate = f"{axis}_rate"

    rise = next((r["time"] - start for r in step if r[rate] >= 0.9 * target), float("nan"))
    overshoot = (max(r[rate] for r in step) - target) / target * 100.0

    outside = [r["time"] for r in step if abs(r[rate] - target) > 0.05 * target]
    settle = max(outside) - start if outside else 0.0

    others = max(abs(r[f"{a}_rate"]) for r in run for a in AXES if a != axis)

    hold = [r["fl"] for r in step if r["time"] > end - 0.3]
    lowest = min(r[m] for r in run for m in MOTORS)
    highest = max(r[m] for r in run for m in MOTORS)
    return axis, rise, overshoot, settle, others, std_dev(hold), lowest, highest


def main():
    print(f"{'file':32s} {'axis':5s} {'rise':>7s} {'overshoot':>9s} {'settle':>7s} "
          f"{'others':>7s} {'jitter':>7s} {'motors':>12s}")
    for path in sys.argv[1:]:
        axis, rise, overshoot, settle, others, jitter, lowest, highest = measure(read_run(path))
        print(f"{path:32s} {axis:5s} {rise:6.3f}s {overshoot:8.1f}% {settle:6.3f}s "
              f"{others:7.2f} {jitter:7.4f} {lowest:5.2f}..{highest:4.2f}")


if __name__ == "__main__":
    main()
