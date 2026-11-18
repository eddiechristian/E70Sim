#!/bin/sh
# Runs the simulation once per Kp value and saves a CSV and a picture of each
# in runs/. These are the four cases in the "What to expect" table of
# Stage1-c.md.
#
# Usage: ./run_cases.sh
# Extra arguments are passed to every run, for example: ./run_cases.sh --ki 0.003

set -e
cd "$(dirname "$0")"

mkdir -p runs
cargo build --quiet

for kp in 0.0005 0.001 0.002 0.004; do
    name="runs/kp_$kp"
    cargo run --quiet -- --kp "$kp" "$@" > "$name.csv"
    python3 plot.py "$name.csv" "$name.png"
    echo "$name.csv  $name.png"
done
