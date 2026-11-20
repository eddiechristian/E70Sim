#!/usr/bin/env python3
"""Print the figures used to judge a free run of the quad simulation.

Usage:
    python3 measure_flight.py quad.csv [more.csv ...]

For a run of Stage 3a or later, with the body free; not for a run on the
joint. Explained in Stage3-a.md in the repo root.

    lift      when the drone leaves the ground, in seconds from the start
    top       height when the step starts, or at the end of a run with no step
    end       height at the end of the run
    climb     how fast it is still climbing at the end, in m/s
    drift     how far it is from where it took off, over the ground, when the
              step starts, or at the end of a run with no step, in metres
    landed    the time it came back to the ground, or - if it did not

The rest are for the step in roll or pitch, and empty for a run without one:

    axis      the axis that was stepped
    asked     the angle asked for, in degrees
    reached   the true angle furthest from level during the step
    error     largest distance of the estimate from the true angle during the
              step, in degrees
    after     the true angle furthest from level from 0.5 s after the step to
              the end of the run, when the target is level again
    speed     speed over the ground at the end of the step, in m/s
    height    change of height over the step, in metres
    rate      how fast it is climbing at the end of the step, in m/s; negative
              is sinking
"""

import csv
import math
import sys

AXES = ["roll", "pitch"]


def read_run(path):
    with open(path, newline="") as f:
        return [{k: float(v) for k, v in row.items()} for row in csv.DictReader(f)]


def largest(values):
    """The value furthest from zero, with its sign."""
    return max(values, key=abs)


def measure(run):
    flying = [r for r in run if r["state"] == 1]
    in_air = [r for r in flying if r["on_ground"] == 0]
    lift = in_air[0]["time"] if in_air else math.nan

    # It landed if it is on the ground again after having left it.
    back = [r["time"] for r in flying if r["on_ground"] == 1 and r["time"] > lift]
    landed = back[0] if back else math.nan

    # The stepped axis is the one whose angle target is not always zero.
    axis = next((a for a in AXES if any(r[f"{a}_angle_target"] != 0.0 for r in run)), None)
    step = [r for r in run if axis and r[f"{axis}_angle_target"] != 0.0]

    # "Before the step" is the whole run when there is none.
    before = [r for r in run if not step or r["time"] < step[0]["time"]]
    top = before[-1]["height"]
    drift = math.hypot(before[-1]["north"], before[-1]["east"])

    figures = {
        "lift": lift,
        "top": top,
        "end": run[-1]["height"],
        "climb": run[-1]["climb"],
        "drift": drift,
        "landed": landed,
        "axis": axis,
    }
    if step:
        angle = f"{axis}_angle"
        end = step[-1]["time"]
        after = [r for r in run if r["time"] >= end + 0.5]
        figures.update({
            "asked": step[0][f"{axis}_angle_target"],
            "reached": largest(r[angle] for r in step),
            "error": max(abs(r[f"{axis}_estimate"] - r[angle]) for r in step),
            "after": largest(r[angle] for r in after),
            "speed": math.hypot(step[-1]["north_speed"], step[-1]["east_speed"]),
            "height": step[-1]["height"] - step[0]["height"],
            "rate": step[-1]["climb"],
        })
    return figures


def main():
    print(f"{'file':26s} {'lift':>6s} {'top':>6s} {'end':>6s} {'climb':>6s} {'drift':>6s} {'landed':>7s}"
          f" {'axis':>5s} {'asked':>6s} {'reached':>7s} {'error':>6s} {'after':>6s} {'speed':>6s} {'height':>7s} {'rate':>6s}")
    for path in sys.argv[1:]:
        f = measure(read_run(path))
        landed = "-" if math.isnan(f["landed"]) else f"{f['landed']:.2f}s"
        line = (f"{path:26s} {f['lift']:5.2f}s {f['top']:6.2f} {f['end']:6.2f} {f['climb']:6.2f} "
                f"{f['drift']:6.2f} {landed:>7s}")
        if "asked" in f:
            line += (f" {f['axis']:>5s} {f['asked']:6.1f} {f['reached']:7.2f} {f['error']:6.2f} "
                     f"{f['after']:6.2f} {f['speed']:6.2f} {f['height']:7.2f} {f['rate']:6.2f}")
        print(line)


if __name__ == "__main__":
    main()
