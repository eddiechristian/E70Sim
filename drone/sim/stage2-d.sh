#!/bin/sh
# The disturbance tests on the quad: a kick, a 30° start and one weak motor.
# Runs each case, prints its figures and saves a picture of it in stage2-d/.
# Explained in Stage2-d.md in the repo root, and in stage2-d/README.md.
#
# Usage: ./stage2-d.sh

set -e
cd "$(dirname "$0")"

mkdir -p stage2-d
tmp=$(mktemp -d)
trap 'rm -rf "$tmp"' EXIT
cargo build --quiet --bin quad

# name, then the arguments of the run
run_case() {
    name=$1
    shift
    cargo run --quiet --bin quad -- "$@" > "$tmp/$name.csv"
    python3 plot_angles.py "$tmp/$name.csv" "stage2-d/$name.png" --title "$name:  $*"
}

# Gyro noise, a loop delay, accelerometer noise and a gyro bias.
REAL="--noise 2 --delay 2 --accel-noise 0.5 --gyro-bias 1"

# The controller as it was after Stage 2c: no I term in the rate loops.
NO_I="--ki 0 --yaw-ki 0"

# No step, so that the only thing happening is the disturbance.
LEVEL="--target 0"

run_case 01_kick_roll              $LEVEL --kick-roll 0.3
run_case 02_kick_pitch             $LEVEL --kick-pitch 0.3
run_case 03_kick_compound          $LEVEL --kick-roll 0.3 --kick-pitch 0.3
run_case 04_kick_compound_hard     $LEVEL --kick-roll 1.2 --kick-pitch 1.2
run_case 05_kick_yaw               $LEVEL --kick-yaw 0.1
run_case 06_kick_hard              $LEVEL --kick-roll 2.4
run_case 07_kick_roll_real         $LEVEL --kick-roll 0.3 $REAL
run_case 08_start_30               $LEVEL --start-roll 30
run_case 09_start_30_both          $LEVEL --start-roll 30 --start-pitch 30
run_case 10_weak_motor_no_i        $LEVEL --weak-motor fr $NO_I
run_case 11_weak_motor             $LEVEL --weak-motor fr
run_case 12_weak_motor_real        $LEVEL --weak-motor fr $REAL
run_case 13_step_no_i              --axis roll $NO_I
run_case 14_step                   --axis roll
run_case 15_yaw_step_no_i          --axis yaw $NO_I
run_case 16_yaw_step               --axis yaw

# The disturbances, then what the I terms cost a step. measure_quad.py gives
# the figures of the yaw steps, which are steps in rate; its motor range
# takes in the calibration, when the motors are off.
(cd "$tmp" && python3 "$OLDPWD/measure_disturbance.py" 0*.csv 10*.csv 11*.csv 12*.csv && echo &&
    python3 "$OLDPWD/measure_angles.py" 13*.csv 14*.csv && echo &&
    python3 "$OLDPWD/measure_quad.py" 15*.csv 16*.csv)
