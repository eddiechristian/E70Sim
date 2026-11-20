#!/usr/bin/env python3
"""Print the figures used to judge an angle step in the quad simulation.

Usage:
    python3 measure_angles.py quad.csv [more.csv ...]

For a run with a step in roll or pitch; not for --axis yaw or --rate-only.
The axis that was stepped is found from the angle targets. The first four
figures are for the step up to the target, on the true angle:

    rise      seconds from the step until the angle first reaches 90% of the target
    overshoot how far the angle passes the target, as a percentage of it
    settle    seconds from the step until the angle stays within 5% of the target
    asked     largest rate the angle loop asked for, in °/s
    hold      largest distance of the true angle from the target over the last
              0.5 s of the step, in degrees
    level     the same over the last 0.5 s of the run, when the target is 0
    estimate  largest distance of the estimate from the true angle while flying,
              in degrees, on the stepped axis
    recover   seconds from the start of flying until the angle stays within 1°
              of its target, before the step; 0 for a start at the target
    motors    smallest and largest motor command while flying

A run with no step (--target 0) has no first four figures; roll is measured.
"""

import csv
import sys

AXES = ["roll", "pitch"]
MOTORS = ["fl", "fr", "br", "bl"]
NAN = float("nan")


def read_run(path):
    with open(path, newline="") as f:
        return [{k: float(v) for k, v in row.items()} for row in csv.DictReader(f)]


def measure(run):
    flying = [r for r in run if r["state"] == 1]

    # The stepped axis is the one whose angle target is not always zero.
    axis = next((a for a in AXES if any(r[f"{a}_angle_target"] != 0.0 for r in run)), "roll")
    angle = f"{axis}_angle"
    step = [r for r in run if r[f"{axis}_angle_target"] != 0.0]

    rise = overshoot = settle = asked = hold = NAN
    if step:
        start = step[0]["time"]
        end = step[-1]["time"]
        target = step[0][f"{axis}_angle_target"]
        # Dividing by the target makes every step a step from 0 to 1, also a
        # negative one.
        progress = [(r["time"], r[angle] / target) for r in step]

        rise = next((t - start for t, p in progress if p >= 0.9), NAN)
        overshoot = (max(p for _, p in progress) - 1.0) * 100.0
        outside = [t for t, p in progress if abs(p - 1.0) > 0.05]
        settle = max(outside) - start if outside else 0.0
        asked = max(abs(r[f"{axis}_target"]) for r in step)
        hold = max(abs(r[angle] - target) for r in step if r["time"] > end - 0.5)

    last = run[-1]["time"]
    level = max(abs(r[angle] - r[f"{axis}_angle_target"]) for r in flying if r["time"] > last - 0.5)
    estimate = max(abs(r[f"{axis}_estimate"] - r[angle]) for r in flying)
    # Before the step the target is 0, or the whole run when there is no step.
    before = [r for r in flying if not step or r["time"] < step[0]["time"]]
    off = [r["time"] for r in before if abs(r[angle] - r[f"{axis}_angle_target"]) > 1.0]
    recover = max(off) + 0.001 - flying[0]["time"] if off else 0.0

    lowest = min(r[m] for r in flying for m in MOTORS)
    highest = max(r[m] for r in flying for m in MOTORS)
    return axis, rise, overshoot, settle, asked, hold, level, estimate, recover, lowest, highest


def main():
    print(f"{'file':32s} {'axis':5s} {'rise':>7s} {'overshoot':>9s} {'settle':>7s} "
          f"{'asked':>6s} {'hold':>6s} {'level':>6s} {'estimate':>8s} {'recover':>7s} {'motors':>12s}")
    for path in sys.argv[1:]:
        axis, rise, overshoot, settle, asked, hold, level, estimate, recover, lowest, highest = measure(read_run(path))
        print(f"{path:32s} {axis:5s} {rise:6.3f}s {overshoot:8.1f}% {settle:6.3f}s "
              f"{asked:6.1f} {hold:6.2f} {level:6.2f} {estimate:8.2f} {recover:6.3f}s {lowest:5.2f}..{highest:4.2f}")


if __name__ == "__main__":
    main()
