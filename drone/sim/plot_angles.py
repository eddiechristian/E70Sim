#!/usr/bin/env python3
"""Plot quad.csv from the quad simulation: angles, rates and motors.

Usage:
    cargo run --bin quad > quad.csv
    python3 plot_angles.py [quad.csv] [picture.png]

Not for a run made with --rate-only; plot that with plot_quad.py.

Needs matplotlib: sudo apt install python3-matplotlib
"""

import argparse
import csv
import os

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
    parser = argparse.ArgumentParser(description="Plot a CSV from the quad simulation.")
    parser.add_argument("csv", nargs="?", default="quad.csv", help="file to plot")
    parser.add_argument("picture", nargs="?", help="save a picture here instead of opening a window")
    parser.add_argument("--title", help="title of the graph; the file name if not given")
    args = parser.parse_args()

    # Imported here so that a missing matplotlib does not stop --help.
    import matplotlib.pyplot as plt

    path = args.csv
    run = read_run(path)
    time = run["time"]

    # The calibration is every step with state 0. It is shaded in each panel.
    calibrating = [t for t, state in zip(time, run["state"]) if state == 0]

    # Six panels, one above the other, sharing the time axis: the angle and
    # the rate of roll, the same for pitch, the yaw rate, then the motors.
    fig, panels = plt.subplots(6, 1, sharex=True, figsize=(9, 14))
    roll_angle, roll_rate, pitch_angle, pitch_rate, yaw_rate, motors = panels

    for axis, panel in [("roll", roll_angle), ("pitch", pitch_angle)]:
        panel.plot(time, run[f"{axis}_angle_target"], label="target", color="gray", linestyle="--")
        panel.plot(time, run[f"{axis}_estimate"], label="estimate", color="tab:orange", linewidth=0.8)
        panel.plot(time, run[f"{axis}_angle"], label="true angle", color="tab:blue")
        panel.set_ylabel(f"{axis} angle (°)")

    # On roll and pitch the rate target is what the angle loop asks for. On
    # yaw it is the target of the run.
    for axis, panel, target in [
        ("roll", roll_rate, "asked for by the angle loop"),
        ("pitch", pitch_rate, "asked for by the angle loop"),
        ("yaw", yaw_rate, "target"),
    ]:
        panel.plot(time, run[f"{axis}_measured"], label="measured", color="lightblue", linewidth=0.5)
        panel.plot(time, run[f"{axis}_target"], label=target, color="gray", linestyle="--")
        panel.plot(time, run[f"{axis}_rate"], label="true rate", color="tab:blue")
        panel.set_ylabel(f"{axis} rate (°/s)")

    for motor in MOTORS:
        motors.plot(time, run[motor], label=motor)
    motors.set_ylabel("motor command (0 to 1)")
    motors.set_xlabel("time (s)")

    for panel in panels:
        if calibrating:
            # Named in the top panel's legend only.
            label = "calibrating" if panel is panels[0] else None
            panel.axvspan(calibrating[0], calibrating[-1], color="0.9", label=label)
        panel.legend(loc="upper right", fontsize="small")
        panel.grid(True)
    panels[0].set_title(args.title or f"The quad: {os.path.basename(path)}", fontsize="medium")

    fig.tight_layout()

    # With a second argument, save a picture instead of opening a window.
    if args.picture:
        fig.savefig(args.picture, dpi=120)
    else:
        plt.show()


if __name__ == "__main__":
    main()
