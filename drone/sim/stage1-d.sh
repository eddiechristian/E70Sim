#!/bin/sh
# The retune with sensor noise and loop delay on: runs each case, prints its
# figures and saves a picture of it in stage1-d/. The pictures are numbered in
# the order of the retune and share the same axes, so they can be compared.
# Explained in Stage1-d.md in the repo root, and in stage1-d/README.md.
#
# Usage: ./stage1-d.sh

set -e
cd "$(dirname "$0")"

mkdir -p stage1-d
tmp=$(mktemp -d)
trap 'rm -rf "$tmp"' EXIT
cargo build --quiet

# name, then the arguments of the run
run_case() {
    name=$1
    shift
    cargo run --quiet -- "$@" > "$tmp/$name.csv"
    python3 plot.py "$tmp/$name.csv" "stage1-d/$name.png" \
        --title "$name:  $*" --rate-range -15 70 --command-range 0.45 0.95
}

NOISY="--noise 2 --delay 2"

run_case 01_before_kp_0.001             --kp 0.001
run_case 02_noise_and_delay_kp_0.001    --kp 0.001 $NOISY
run_case 03_lower_kp_0.0005             --kp 0.0005 $NOISY
run_case 04_higher_kp_0.002             --kp 0.002 $NOISY
run_case 05_much_higher_kp_0.004        --kp 0.004 $NOISY
run_case 06_add_d_kp_0.002_kd_0.00002   --kp 0.002 --kd 0.00002 $NOISY
run_case 07_small_d_kp_0.00125_kd_0.000005 --kp 0.00125 --kd 0.000005 $NOISY
run_case 08_add_i_kp_0.001_ki_0.001     --kp 0.001 --ki 0.001 $NOISY

# With the D-term filter
run_case 09_filter_the_d_of_06          --kp 0.002 --kd 0.00002 --d-filter 0.005 $NOISY
run_case 10_more_d_kd_0.00004           --kp 0.002 --kd 0.00004 --d-filter 0.005 $NOISY
run_case 11_chosen_longer_filter_0.01   --kp 0.002 --kd 0.00004 --d-filter 0.01 $NOISY

(cd "$tmp" && python3 "$OLDPWD/measure.py" *.csv)
