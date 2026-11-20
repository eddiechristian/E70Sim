#!/usr/bin/env python3
"""Plot quad.csv from a free run of the quad simulation: where the drone
goes, its angles and its motors.

Usage:
    cargo run --bin quad > quad.csv
    python3 plot_flight.py [quad.csv] [picture.png]

For a run of Stage 3a or later, with the body free. A run on the joint goes
nowhere; plot that with plot_angles.py. Explained in Stage3-a.md in the repo
root.

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
    parser = argparse.ArgumentParser(description="Plot a CSV from a free run of the quad simulation.")
    parser.add_argument("csv", nargs="?", default="quad.csv", help="file to plot")
    parser.add_argument("picture", nargs="?", help="save a picture here instead of opening a window")
    parser.add_argument("--title", help="title of the graph; the file name if not given")
    args = parser.parse_args()

    # Imported here so that a missing matplotlib does not stop --help.
    import matplotlib.pyplot as plt

    path = args.csv
    run = read_run(path)
    time = run["time"]

    # The calibration is every step with state 0, and a kick is every step
    # with a push on any axis. Both are shaded in each panel.
    calibrating = [t for t, state in zip(time, run["state"]) if state == 0]
    pushes = [run[f"push_{axis}"] for axis in ["roll", "pitch", "yaw"]]
    kicking = [t for t, *push in zip(time, *pushes) if any(push)]

    # Six panels, one above the other, sharing the time axis: the height,
    # the place over the ground, the three speeds, the two angles, then the
    # motors.
    fig, panels = plt.subplots(6, 1, sharex=True, figsize=(9, 14))
    height, place, speed, roll_angle, pitch_angle, motors = panels

    height.plot(time, run["height"], label="height", color="tab:blue")
    height.set_ylabel("height (m)")

    place.plot(time, run["north"], label="north", color="tab:blue")
    place.plot(time, run["east"], label="east", color="tab:orange")
    place.set_ylabel("from the start (m)")

    speed.plot(time, run["north_speed"], label="north", color="tab:blue")
    speed.plot(time, run["east_speed"], label="east", color="tab:orange")
    speed.plot(time, run["climb"], label="climb", color="tab:green")
    speed.set_ylabel("speed (m/s)")

    for axis, panel in [("roll", roll_angle), ("pitch", pitch_angle)]:
        panel.plot(time, run[f"{axis}_angle_target"], label="target", color="gray", linestyle="--")
        panel.plot(time, run[f"{axis}_estimate"], label="estimate", color="tab:orange", linewidth=0.8)
        panel.plot(time, run[f"{axis}_angle"], label="true angle", color="tab:blue")
        panel.set_ylabel(f"{axis} angle (°)")

    for motor in MOTORS:
        motors.plot(time, run[motor], label=motor)
    motors.plot(time, run["throttle"], label="throttle", color="gray", linestyle="--")
    motors.set_ylabel("motor command (0 to 1)")
    motors.set_xlabel("time (s)")

    for panel in panels:
        # Each is named in the top panel's legend only.
        top = panel is panels[0]
        if calibrating:
            label = "calibrating" if top else None
            panel.axvspan(calibrating[0], calibrating[-1], color="0.9", label=label)
        if kicking:
            label = "kick" if top else None
            panel.axvspan(kicking[0], kicking[-1], color="tab:red", alpha=0.25, label=label)
        panel.legend(loc="upper left", fontsize="small")
        panel.grid(True)
    panels[0].set_title(args.title or f"The quad, free: {os.path.basename(path)}", fontsize="medium")

    fig.tight_layout()

    # With a second argument, save a picture instead of opening a window.
    if args.picture:
        fig.savefig(args.picture, dpi=120)
    else:
        plt.show()


if __name__ == "__main__":
    main()
