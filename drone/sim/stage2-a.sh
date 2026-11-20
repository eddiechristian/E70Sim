#!/bin/sh
# The rate loops on the quad: runs a step on each axis, prints its figures
# and saves a picture of it in stage2-a/. Explained in "In the simulation" in
# Stage2-a.md in the repo root, and in stage2-a/README.md.
#
# Usage: ./stage2-a.sh

set -e
cd "$(dirname "$0")"

mkdir -p stage2-a
tmp=$(mktemp -d)
trap 'rm -rf "$tmp"' EXIT
cargo build --quiet --bin quad

# name, then the arguments of the run. Every run has --rate-only as well:
# these cases are from before the angle loops and the calibration.
run_case() {
    name=$1
    shift
    cargo run --quiet --bin quad -- --rate-only "$@" > "$tmp/$name.csv"
    python3 plot_quad.py "$tmp/$name.csv" "stage2-a/$name.png" --title "$name:  $*"
}

NOISY="--noise 2 --delay 2"

run_case 01_roll_step               --axis roll
run_case 02_pitch_step              --axis pitch
run_case 03_yaw_step                --axis yaw
run_case 04_roll_step_noise_delay   --axis roll $NOISY
run_case 05_yaw_step_noise_delay    --axis yaw $NOISY

(cd "$tmp" && python3 "$OLDPWD/measure_quad.py" *.csv)
