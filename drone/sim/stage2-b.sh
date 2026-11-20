#!/bin/sh
# The estimator on the quad: runs each case, prints how far the estimate is
# from the true angles and saves a picture of it in stage2-b/. Explained in
# Stage2-b.md in the repo root, and in stage2-b/README.md.
#
# Usage: ./stage2-b.sh

set -e
cd "$(dirname "$0")"

mkdir -p stage2-b
tmp=$(mktemp -d)
trap 'rm -rf "$tmp"' EXIT
cargo build --quiet --bin quad

# name, then the arguments of the run. Every run has --stage 2b as well, which
# runs the program as it was at this stage: rate loops only, with no I term,
# from before the angle loops and the calibration.
run_case() {
    name=$1
    shift
    cargo run --quiet --bin quad -- --stage 2b "$@" > "$tmp/$name.csv"
    python3 plot_estimator.py "$tmp/$name.csv" "stage2-b/$name.png" --title "$name:  $*"
}

NOISY="--noise 2 --delay 2 --accel-noise 0.5"

run_case 01_roll_step_clean          --axis roll
run_case 02_roll_step_noisy          --axis roll $NOISY
run_case 03_pitch_step_noisy         --axis pitch $NOISY
run_case 04_gyro_bias_1              --axis roll --target 0 --gyro-bias 1 $NOISY
run_case 05_gyro_bias_1_tau_0.5      --axis roll --target 0 --gyro-bias 1 --estimator-tau 0.5 $NOISY
run_case 06_noisy_tau_0.05           --axis roll --estimator-tau 0.05 $NOISY
run_case 07_noisy_tau_5              --axis roll --estimator-tau 5 $NOISY

(cd "$tmp" && python3 "$OLDPWD/measure_estimator.py" *.csv)
