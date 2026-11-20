#!/usr/bin/env python3
"""Print the figures used to judge a disturbance in the quad simulation.

Usage:
    python3 measure_disturbance.py quad.csv [more.csv ...]

For a run with --target 0 and a kick, a tilted start or a weak motor; not
for --rate-only. Explained in Stage2-d.md in the repo root.

Times are counted from the start of the kick, or from the moment the motors
start when the run has no kick. "Off level" is the distance of the true
angle from its target, which is 0 in these runs.

    from      when the count starts, in seconds from the start of the run
    roll      largest roll off level from then on, in degrees, with its sign
    pitch     the same for pitch
    back      seconds until roll and pitch both stay within 1° of level
    roll end  roll off level at the end of the run, in degrees
    pitch end the same for pitch
    yaw peak  largest yaw rate from then on, in °/s, with its sign
    yaw end   yaw rate at the end of the run, in °/s
    heading   how far the nose has turned by the end of the run, in degrees
    motors    smallest and largest motor command while flying
"""

import csv
import sys

AXES = ["roll", "pitch"]
PUSHES = ["push_roll", "push_pitch", "push_yaw"]
MOTORS = ["fl", "fr", "br", "bl"]


def read_run(path):
    with open(path, newline="") as f:
        return [{k: float(v) for k, v in row.items()} for row in csv.DictReader(f)]


def largest(values):
    """The value furthest from zero, with its sign."""
    return max(values, key=abs)


def measure(run):
    flying = [r for r in run if r["state"] == 1]
    kicked = [r for r in flying if any(r[p] != 0.0 for p in PUSHES)]
    start = kicked[0]["time"] if kicked else flying[0]["time"]
    after = [r for r in flying if r["time"] >= start]

    def off_level(r, axis):
        return r[f"{axis}_angle"] - r[f"{axis}_angle_target"]

    peaks = [largest(off_level(r, axis) for r in after) for axis in AXES]
    outside = [r["time"] for r in after if any(abs(off_level(r, axis)) > 1.0 for axis in AXES)]
    back = max(outside) + 0.001 - start if outside else 0.0
    ends = [off_level(run[-1], axis) for axis in AXES]

    yaw_peak = largest(r["yaw_rate"] for r in after)
    yaw_end = run[-1]["yaw_rate"]
    heading = run[-1]["yaw_angle"] - after[0]["yaw_angle"]

    lowest = min(r[m] for r in flying for m in MOTORS)
    highest = max(r[m] for r in flying for m in MOTORS)
    return start, peaks, back, ends, yaw_peak, yaw_end, heading, lowest, highest


def main():
    print(f"{'file':28s} {'from':>6s} {'roll':>7s} {'pitch':>7s} {'back':>7s} "
          f"{'roll end':>8s} {'pitch end':>9s} {'yaw peak':>8s} {'yaw end':>8s} {'heading':>7s} {'motors':>12s}")
    for path in sys.argv[1:]:
        start, peaks, back, ends, yaw_peak, yaw_end, heading, lowest, highest = measure(read_run(path))
        print(f"{path:28s} {start:5.2f}s {peaks[0]:7.2f} {peaks[1]:7.2f} {back:6.3f}s "
              f"{ends[0]:8.2f} {ends[1]:9.2f} {yaw_peak:8.2f} {yaw_end:8.2f} {heading:7.2f} {lowest:5.2f}..{highest:4.2f}")


if __name__ == "__main__":
    main()
