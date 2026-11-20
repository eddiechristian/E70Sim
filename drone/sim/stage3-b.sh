#!/bin/sh
# The throttle: a hover throttle that is a little wrong, and more throttle
# in a tilt.
# Runs each case, prints its figures and saves a picture of it in stage3-b/.
# Explained in Stage3-b.md in the repo root, and in stage3-b/README.md.
#
# Usage: ./stage3-b.sh

set -e
cd "$(dirname "$0")"

mkdir -p stage3-b
tmp=$(mktemp -d)
trap 'rm -rf "$tmp"' EXIT
cargo build --quiet --bin quad

# name, then the arguments of the run. Every run has --stage 3b as well, so
# that a later stage that changes a default does not change these runs.
run_case() {
    name=$1
    shift
    cargo run --quiet --bin quad -- --stage 3b "$@" > "$tmp/$name.csv"
    python3 plot_flight.py "$tmp/$name.csv" "stage3-b/$name.png" --title "$name:  $*"
}

# Gyro noise, a loop delay, accelerometer noise and a gyro bias.
REAL="--noise 2 --delay 2 --accel-noise 0.5 --gyro-bias 1"

# No step: take off, then the hover command to the end.
HOVER="--target 0"

# The throttle left alone in a tilt, as in Stage 3a.
OFF="--no-tilt-compensation"

run_case 01_hover                  $HOVER
run_case 02_hover_throttle_high    $HOVER --throttle-error 0.014
run_case 03_hover_throttle_low     $HOVER --throttle-error -0.014
run_case 04_tilt_10_off            --axis pitch --target -10 $OFF
run_case 05_tilt_10                --axis pitch --target -10
run_case 06_tilt_20_off            --axis pitch --target -20 $OFF
run_case 07_tilt_20                --axis pitch --target -20
run_case 08_tilt_30_off            --axis pitch --target -30 $OFF
run_case 09_tilt_30                --axis pitch --target -30
run_case 10_tilt_roll_20           --axis roll --target 20
run_case 11_tilt_10_real           --axis pitch --target -10 $REAL
run_case 12_tilt_30_tau_1          --axis pitch --target -30 --estimator-tau 1

(cd "$tmp" && python3 "$OLDPWD/measure_flight.py" *.csv)
