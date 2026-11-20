# sim

Runs the rate PID from `../flight` against the seesaw from `../physics` and writes the result as CSV. `plot.py` turns the CSV into a graph. A second program runs the three-axis quad with the whole controller; see [The quad](#the-quad).

The design is explained in [`Stage1-c.md`](../../Stage1-c.md) in the repo root.

## Quick start

Run everything from this directory (`drone/sim/`):

```
cargo run > run.csv
python3 plot.py
```

The first command runs 3 simulated seconds and saves the output in `run.csv`. The second opens a window with the graph.

## One-time setup

The plot scripts need the `matplotlib` library. On Ubuntu:

```
sudo apt install python3-matplotlib
```

Where `pip` is available, `pip install --user matplotlib` works too.

## Running the simulation

```
cargo run > run.csv
```

- The program prints CSV to standard output, and `> run.csv` saves it to a file. Without the `>` the 3001 lines scroll past in the terminal.
- It does not run in real time; it finishes in a moment.
- `run.csv` is overwritten every time. To keep a run for comparison, give it another name: `cargo run > kp002.csv`.

## Plotting

| Command | What it does |
|---|---|
| `python3 plot.py` | opens a window showing `run.csv` |
| `python3 plot.py kp002.csv` | opens a window showing another file |
| `python3 plot.py run.csv run.png` | saves a picture instead of opening a window |
| `python3 plot.py run.csv run.png --rate-range -15 70 --command-range 0.45 0.95` | fixes the vertical axes, so that pictures can be compared |
| `python3 measure.py run.csv` | prints the rise time, overshoot, settling time and motor jitter of a run |

The graph has two panels:

- **Top:** the target rate and the true rate, in °/s, with the measured rate as a pale line behind.
- **Bottom:** the left and right motor commands, from 0 to 1.

[`Stage1-c.md`](../../Stage1-c.md#the-plot-script) has a table for reading the top panel: sluggish, good, ringing or steady-state error.

## Changing a run

The gains and the seesaw's size are command-line arguments. Anything not given keeps its default. The `--` separates cargo's own arguments from the simulation's:

```
cargo run -- --kp 0.002 > run.csv
cargo run -- --kp 0.001 --ki 0.003 --inertia 0.03 > run.csv
cargo run -- --help
```

| Argument | Meaning | Default |
|---|---|---|
| `--kp` | proportional gain, in throttle per °/s | 0.001 |
| `--ki` | integral gain | 0 |
| `--kd` | derivative gain | 0 |
| `--d-filter` | time constant of the D-term low-pass filter, in s; 0 for none | 0 |
| `--arm-length` | pivot to motor, in m | 0.2 |
| `--inertia` | how hard the bar is to spin, in kg·m² | 0.015 |
| `--thrust-coeff` | thrust of each motor at full speed, in N | 6.0 |
| `--noise` | gyro noise: standard deviation in °/s | 0 |
| `--delay` | loop delay: the PID sees the reading from this many steps ago | 0 |
| `--seed` | seed for the gyro noise; the same seed gives the same run | 1 |

`--noise`, `--delay` and `--seed` are explained in [`Stage1-d.md`](../../Stage1-d.md). With the defaults of no noise and no delay the PID sees the true rate.

The plot only reads the file, so it does not update until the simulation is rerun.

The other settings are constants at the top of `src/main.rs`:

| Constant | Meaning | Value |
|---|---|---|
| `OUT_LIMIT` | the PID output stays within ± this | 0.3 |
| `I_LIMIT` | limit on the I term | 0.1 |
| `BASE` | throttle both motors share | 0.7 |
| `STEP_START`, `STEP_END` | when the target step begins and ends, in s | 0.5, 1.5 |
| `STEP_TARGET` | target rate during the step, in °/s | 50 |
| `STEPS` | number of 1 ms steps | 3000 |
| `MOTOR_TIME_CONSTANT` | the motors' lag | 0.04 s |

For the weak-motor experiment in `Stage1-c.md`, give the right motor a thrust coefficient of 5.4 where the seesaw is built in `main`; `--thrust-coeff` sets both motors.

## Running the four Kp cases

```
./run_cases.sh
```

This runs the simulation once for each `Kp` in the "What to expect" table of `Stage1-c.md` (0.0005, 0.001, 0.002 and 0.004) and writes a CSV and a picture of each into `runs/`:

```
runs/kp_0.0005.csv  runs/kp_0.0005.png
runs/kp_0.001.csv   runs/kp_0.001.png
runs/kp_0.002.csv   runs/kp_0.002.png
runs/kp_0.004.csv   runs/kp_0.004.png
```

Extra arguments are passed to every run, so `./run_cases.sh --inertia 0.03` repeats the four cases on a heavier seesaw. The files in `runs/` are overwritten each time.

## The retune with noise and delay

```
./stage1-d.sh
```

This runs the eleven cases of the retune in [`Stage1-d.md`](../../Stage1-d.md#the-retune), prints the figures for each and saves numbered pictures with matching axes into `stage1-d/`. [`stage1-d/README.md`](stage1-d/README.md) lists them.

## The quad

A second program runs the three-axis body in the whole loop of [`Stage2-c.md`](../../Stage2-c.md): sensors, the `Controller` from `../flight`, four motors and the body. It leaves the seesaw program and its scripts as they are.

```
cargo run --bin quad > quad.csv
python3 plot_angles.py
python3 measure_angles.py quad.csv
```

Plain `cargo run` is still the seesaw. The quad's run is 5 s:

- **0 to 1 s:** the calibration, with the motors off.
- **1 to 1.5 s:** flying, with every target 0.
- **1.5 to 3.5 s:** the step. Roll or pitch is asked for an angle of 20°, or yaw for a rate of 50 °/s.
- **3.5 to 5 s:** every target 0 again.

```
cargo run --bin quad -- --axis pitch --target -20 > quad.csv
cargo run --bin quad -- --noise 2 --delay 2 --accel-noise 0.5 --gyro-bias 1 > quad.csv
cargo run --bin quad -- --start-roll 20 --target 0 > quad.csv
cargo run --bin quad -- --help
```

| Argument | Meaning | Default |
|---|---|---|
| `--axis` | which axis gets the step: `roll`, `pitch` or `yaw` | `roll` |
| `--target` | the step: an angle in degrees for roll and pitch, a rate in °/s for yaw | 20, or 50 for yaw |
| `--angle-kp` | angle loop gain for roll and pitch, in °/s per degree | 6 |
| `--rate-limit` | the angle loops ask for no more than this rate, in °/s | 200 |
| `--start-roll`, `--start-pitch` | the angle the body starts at, in degrees | 0, 0 |
| `--no-calibration` | fly without what the calibration finds; see below | off |
| `--rate-only` | run as before Stage 2c; see [Rate loops only](#rate-loops-only) | off |
| `--kp`, `--ki`, `--kd` | roll and pitch rate loop gains | 0.0014, 0.003, 0.00003 |
| `--yaw-kp`, `--yaw-ki`, `--yaw-kd` | yaw rate loop gains | 0.01, 0.02, 0 |
| `--d-filter` | time constant of the D-term low-pass filter on all three axes, in s | 0.01 |
| `--thrust-coeff` | thrust of each motor at full speed, in N | 6.0 |
| `--kick-roll`, `--kick-pitch`, `--kick-yaw` | a kick: a torque about that axis, in N·m; see [Disturbances](#disturbances) | 0, 0, 0 |
| `--kick-at`, `--kick-for` | when the kick starts and how long it lasts, in s | 2.0, 0.05 |
| `--weak-motor` | one motor that is weaker than the others: `fl`, `fr`, `br` or `bl` | none |
| `--weak-by` | how much weaker, as a percentage of its thrust | 10 |
| `--noise`, `--delay`, `--seed` | as for the seesaw; each axis has its own gyro | 0, 0, 1 |
| `--gyro-bias` | a constant error on all three gyro axes, in °/s | 0 |
| `--accel-noise` | accelerometer noise on each axis: standard deviation in m/s² | 0 |
| `--estimator-tau` | time constant of the estimator, in s | 1 |

The body's size (arm length, the three inertias, the yaw coefficient) and the limits are constants at the top of `src/bin/quad.rs`. The gain defaults are the same as `Gains::default()` in the flight crate, and a test checks that they stay so.

With `--no-calibration` the motors still stay off for the first second, but the controller then starts as it would with no calibration: the gyro bias is taken as zero, and the starting angle comes from one accelerometer reading. Compare a run with and without it, with `--gyro-bias 1 --accel-noise 0.5`, to see what the calibration is worth.

If a calibration window fails, the program says why on standard error and the controller tries again; the motors stay off until one passes.

Where the default rate gains come from:

- **Roll and pitch** use the gains chosen in `Stage1-d.md` (`--kp 0.002 --kd 0.00004`), divided by √2. One unit of PID output turns the quad √2 times harder than it turned the seesaw: four motors at 0.14 m from the axis, against two motors at 0.2 m.
- **Yaw** needs a much larger gain, because the same command turns the body about 16 times more slowly. Even so, a 50 °/s yaw step drives the output to its limit for a moment.
- **The two `Ki` values** were chosen in [`Stage2-d.md`](../../Stage2-d.md#results), to hold the drone level with one weak motor. The Stage 2a, 2b and 2c scripts pass `--ki 0 --yaw-ki 0`, so that they still make the pictures of those stages.

`plot_angles.py` draws six panels: the angle and the rate of roll, the same for pitch, the yaw rate, then the four motor commands. The calibration is shaded. It takes the same two optional arguments as `plot.py`: the file to read and a picture to save.

`measure_angles.py` prints the figures of an angle step: rise time, overshoot, settling time, the largest rate the angle loop asked for, how far the angle is from its target at the end of the step and at the end of the run, the largest error of the estimate, and how long a tilted start takes to come within 1° of its target.

```
./stage2-c.sh
```

This runs ten cases (a step on each axis, the real sensors with and without calibration, a tilted start, two other angle gains and two 45° steps), prints their figures and saves numbered pictures into `stage2-c/`. [`stage2-c/README.md`](stage2-c/README.md) lists them.

### Disturbances

The run can be disturbed in three ways, explained in [`Stage2-d.md`](../../Stage2-d.md). Use `--target 0` with them, so that the only thing happening is the disturbance.

```
cargo run --bin quad -- --target 0 --kick-roll 0.3 > quad.csv
cargo run --bin quad -- --target 0 --kick-roll 0.3 --kick-pitch 0.3 > quad.csv
cargo run --bin quad -- --target 0 --start-roll 30 > quad.csv
cargo run --bin quad -- --target 0 --weak-motor fr > quad.csv
python3 measure_disturbance.py quad.csv
```

- **A kick** is a torque on the body for a short time. The three kick arguments can be given together, which makes a push about an axis between them. It is not available with `--rate-only`. `plot_angles.py` shades it red.
- **A tilted start** is `--start-roll` and `--start-pitch`, as before.
- **A weak motor** is weak from the start of the run. Add `--ki 0 --yaw-ki 0` to see the drone hang about 1° off level without the I terms.

`measure_disturbance.py` prints the largest roll and pitch off level, how long until both stay within 1° of level, the angles and the yaw rate at the end, and how far the nose has turned. Times are counted from the start of the kick, or from the moment the motors start when there is no kick.

```
./stage2-d.sh
```

This runs sixteen cases (seven kicks, two tilted starts, a weak motor with and without the I terms, and steps with and without them), prints their figures and saves numbered pictures into `stage2-d/`. [`stage2-d/README.md`](stage2-d/README.md) lists them.

### The quad's CSV columns

| Columns | Unit | Meaning |
|---|---|---|
| `time` | s | step number × 0.001 |
| `state` | | 0 while calibrating, 1 while flying |
| `roll_angle_target`, `pitch_angle_target` | ° | the angles asked for |
| `roll_angle`, `pitch_angle`, `yaw_angle` | ° | the true attitude |
| `roll_estimate`, `pitch_estimate` | ° | the controller's estimate; `NaN` while calibrating |
| `roll_target`, `pitch_target` | °/s | the rates the angle loops ask for |
| `yaw_target` | °/s | the yaw rate asked for |
| `roll_rate`, `pitch_rate`, `yaw_rate` | °/s | the true rates |
| `roll_measured`, `pitch_measured`, `yaw_measured` | °/s | what the gyro says, bias included |
| `fl`, `fr`, `br`, `bl` | 0–1 | the four motor commands |
| `push_roll`, `push_pitch`, `push_yaw` | N·m | the kick on the body during that step; 0 when there is none |

### Rate loops only

With `--rate-only` the program runs as it did in [`Stage2-a.md`](../../Stage2-a.md) and [`Stage2-b.md`](../../Stage2-b.md): three rate PIDs and the mixer, no angle loops and no calibration. The estimator is given the readings every step and its angles are logged, but it is not used for control.

```
cargo run --bin quad -- --rate-only > quad.csv
python3 plot_quad.py
python3 measure_quad.py quad.csv
```

The run is 3 s: every target is 0, then one axis is asked for a rate of 50 °/s from 0.5 s to 1.5 s, then 0 again. `--target` is a rate on any axis. `--angle-kp`, `--rate-limit` and `--no-calibration` do nothing here.

`plot_quad.py` draws four panels: the target, true and measured rate of each axis, then the four motor commands.

```
./stage2-a.sh
```

This runs five cases (a step on roll, pitch and yaw, then roll and yaw again with noise and delay), prints the figures for each with `measure_quad.py` and saves numbered pictures into `stage2-a/`. [`stage2-a/README.md`](stage2-a/README.md) lists them.

```
cargo run --bin quad -- --rate-only --noise 2 --delay 2 --accel-noise 0.5 > quad.csv
python3 plot_estimator.py
python3 measure_estimator.py quad.csv
```

`plot_estimator.py` draws three panels: roll and pitch, each as the true angle and the estimate, then the error of both. `measure_estimator.py` prints the error at the start and over the last second.

```
./stage2-b.sh
```

This runs seven cases (clean, noisy, a gyro bias, and three other time constants), prints their figures and saves numbered pictures into `stage2-b/`. [`stage2-b/README.md`](stage2-b/README.md) lists them.

The CSV has other columns in this mode:

| Columns | Unit | Meaning |
|---|---|---|
| `time` | s | step number × 0.001 |
| `roll_target`, `pitch_target`, `yaw_target` | °/s | the setpoints |
| `roll_rate`, `pitch_rate`, `yaw_rate` | °/s | the true rates |
| `roll_measured`, `pitch_measured`, `yaw_measured` | °/s | what the PIDs were told |
| `roll_angle`, `pitch_angle`, `yaw_angle` | ° | the true attitude |
| `roll_estimate`, `pitch_estimate` | ° | the estimator's roll and pitch |
| `roll_output`, `pitch_output`, `yaw_output` | throttle | what the PIDs returned |
| `fl`, `fr`, `br`, `bl` | 0–1 | the four motor commands |

## The seesaw's CSV columns

| Column | Unit | Meaning |
|---|---|---|
| `time` | s | step number × 0.001 |
| `target` | °/s | the setpoint |
| `rate` | °/s | the true rate |
| `measured` | °/s | what the PID was told: the rate with noise and delay |
| `angle` | ° | the seesaw's angle |
| `output` | throttle | what the PID returned |
| `left` | 0–1 | left motor command |
| `right` | 0–1 | right motor command |

## Tests

```
cargo test
```

The tests cover the seesaw program's `target_at` and `mix`, the quad program's targets and default gains, the mixer and the body together (`tests/mixer_and_body.rs`), the estimator and the body together (`tests/estimator_and_body.rs`), and the controller and the body together (`tests/controller_and_body.rs`). The PID, the mixer, the estimator, the calibration, the controller, the motor, the seesaw, the gyro, the quaternion and the body have their own tests in `../flight` and `../physics`.
