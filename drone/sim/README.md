# sim

Runs the rate PID from `../flight` against the seesaw from `../physics` and writes the result as CSV. `plot.py` turns the CSV into a graph.

The design is explained in [`Stage1-c.md`](../../Stage1-c.md) in the repo root.

## Quick start

Run everything from this directory (`drone/sim/`):

```
cargo run > run.csv
python3 plot.py
```

The first command runs 3 simulated seconds and saves the output in `run.csv`. The second opens a window with the graph.

## One-time setup

The plot script needs the `matplotlib` library:

```
pip install --user matplotlib
```

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

## The CSV columns

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

The tests cover `target_at` and `mix`. The PID, the motor, the seesaw and the gyro have their own tests in `../flight` and `../physics`.
