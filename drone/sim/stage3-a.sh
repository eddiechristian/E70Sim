#!/bin/sh
# The free body: the quad with gravity, movement and a ground.
# Runs each case, prints its figures and saves a picture of it in stage3-a/.
# Explained in Stage3-a.md in the repo root, and in stage3-a/README.md.
#
# Usage: ./stage3-a.sh

set -e
cd "$(dirname "$0")"

mkdir -p stage3-a
tmp=$(mktemp -d)
trap 'rm -rf "$tmp"' EXIT
cargo build --quiet --bin quad

# name, then the arguments of the run. Every run has --stage 3a as well, so
# that a later stage that changes a default does not change these runs.
run_case() {
    name=$1
    shift
    cargo run --quiet --bin quad -- --stage 3a "$@" > "$tmp/$name.csv"
    python3 plot_flight.py "$tmp/$name.csv" "stage3-a/$name.png" --title "$name:  $*"
}

# Gyro noise, a loop delay, accelerometer noise and a gyro bias.
REAL="--noise 2 --delay 2 --accel-noise 0.5 --gyro-bias 1"

# No step: take off, then the hover command to the end.
HOVER="--target 0"

# Nose down by 10°, which flies forward.
TILT="--axis pitch --target -10"

# A gyro bias that the calibration has not removed.
BIAS_LEFT="--no-calibration --gyro-bias 0.1"

run_case 01_hover                  $HOVER
run_case 02_tilt_pitch             $TILT
run_case 03_tilt_roll              --axis roll --target 10
run_case 04_tilt_tau_1             $TILT --estimator-tau 1
run_case 05_tilt_tau_3             $TILT --estimator-tau 3
run_case 06_tilt_tau_30            $TILT --estimator-tau 30
run_case 07_tilt_no_drag           $TILT --drag 0
run_case 08_tilt_drag_0.8          $TILT --drag 0.8
run_case 09_tilt_real              $TILT $REAL
run_case 10_tilt_20_tau_1          --axis pitch --target -20 --estimator-tau 1
run_case 11_tilt_20                --axis pitch --target -20
run_case 12_hover_real             $HOVER $REAL
run_case 13_hover_bias_left        $HOVER $BIAS_LEFT
run_case 14_hover_bias_left_tau_1  $HOVER $BIAS_LEFT --estimator-tau 1
run_case 15_hover_bias_left_tau_30 $HOVER $BIAS_LEFT --estimator-tau 30

(cd "$tmp" && python3 "$OLDPWD/measure_flight.py" *.csv)
