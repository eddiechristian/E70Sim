#!/bin/sh
# The whole loop on the quad: calibration, angle loops, rate loops and mixer.
# Runs each case, prints its figures and saves a picture of it in stage2-c/.
# Explained in Stage2-c.md in the repo root, and in stage2-c/README.md.
#
# Usage: ./stage2-c.sh

set -e
cd "$(dirname "$0")"

mkdir -p stage2-c
tmp=$(mktemp -d)
trap 'rm -rf "$tmp"' EXIT
cargo build --quiet --bin quad

# These runs are from before the rate loops had an I term (Stage2-d.md).
NO_I="--ki 0 --yaw-ki 0"

# name, then the arguments of the run
run_case() {
    name=$1
    shift
    cargo run --quiet --bin quad -- $NO_I "$@" > "$tmp/$name.csv"
    python3 plot_angles.py "$tmp/$name.csv" "stage2-c/$name.png" --title "$name:  $*"
}

# Gyro noise, a loop delay, accelerometer noise and a gyro bias.
REAL="--noise 2 --delay 2 --accel-noise 0.5 --gyro-bias 1"

run_case 01_roll_step                --axis roll
run_case 02_pitch_step               --axis pitch
run_case 03_yaw_step                 --axis yaw
run_case 04_roll_step_real           --axis roll $REAL
run_case 05_roll_step_no_calibration --axis roll $REAL --no-calibration
run_case 06_levels_from_20           --start-roll 20 --target 0 $REAL
run_case 07_angle_kp_3               --axis roll --angle-kp 3
run_case 08_angle_kp_12              --axis roll --angle-kp 12
run_case 09_step_45                  --axis roll --target 45
run_case 10_step_45_limit_100        --axis roll --target 45 --rate-limit 100

# The yaw step is a step in rate, so the angle figures have nothing to say
# about it. measure_quad.py gives the figures of a rate step; its motor
# range takes in the calibration, when the motors are off.
(cd "$tmp" && python3 "$OLDPWD/measure_angles.py" *.csv && echo &&
    python3 "$OLDPWD/measure_quad.py" 03_yaw_step.csv)
