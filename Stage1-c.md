# Stage 1c — The rate loop simulation

This explains the third item of Stage 1 in [`drone.md`](drone.md#stage-1--host-simulation-one-axis):

> `drone/sim/`: rate loop holding a target rate; CSV output and a plot script

The first two items built the parts: the PID in [`Stage1-a.md`](Stage1-a.md) and the seesaw in [`Stage1-b.md`](Stage1-b.md). This item connects them. It is the first time the controller steers the physics model, and the first time there is something to plot and tune.

## What it is

`drone/sim/` is a small program that runs on the PC. It:

1. creates one `Pid` and one `Seesaw`
2. steps them together 1000 times per simulated second
3. prints one line of numbers per step, as CSV
4. exits

A separate plot script turns the CSV into a graph of rate against time. The graph shows whether the loop is sluggish, oscillating or good.

It does not run in real time. Three simulated seconds take a few milliseconds to compute.

## What "rate loop" means

The loop holds a target **rotation rate**, not a target angle. "Target 50 °/s" means "keep rotating at 50 °/s", and "target 0" means "stop rotating". This is the inner loop from [Inner and outer loops](drone.md#inner-and-outer-loops):

```
target rate ─►(−)─► rate PID ─► output ─► mixer ─► two motor commands ─► seesaw
              ▲                                                            │
              └──────────────────── gyro rate ─────────────────────────────┘
```

Holding an angle is the outer loop's job, which arrives in Stage 2. Here the angle is recorded but not controlled.

## The three crates

| Crate | Kind | Role in this item |
|---|---|---|
| `drone/flight/` | library, `no_std` | the `Pid` |
| `drone/physics/` | library | the `Seesaw` and its two `Motor`s |
| `drone/sim/` | program (has a `main`) | owns the loop, the mixer, the unit conversion and the CSV |

`sim` depends on the other two by path, in its `Cargo.toml`:

```toml
[dependencies]
flight = { path = "../flight" }
physics = { path = "../physics" }
```

The mixer and the unit conversion live in `sim` for now because they are a few lines each. The four-motor mixer in Stage 2 moves into `flight`, since the firmware needs it too.

## One step of the loop

Each step does five things, in this order:

1. **Read the gyro.** Take the seesaw's rate and convert it from rad/s to °/s.
2. **Run the PID.** Give it the target rate, the measured rate and `dt`. It returns one number, the output.
3. **Mix.** Turn the output into a left and a right motor command.
4. **Step the physics.** Give the seesaw the two commands and `dt`.
5. **Log.** Print one CSV line.

As a sketch:

```
for each step:
    measured = seesaw.rate() converted to °/s
    output   = pid.update(target, measured, dt)
    left     = clamp(base − output, 0, 1)
    right    = clamp(base + output, 0, 1)
    seesaw.step(left, right, dt)
    print the CSV line
```

This is the same shape as the closed-loop test in [`Stage1-a.md`](Stage1-a.md#one-closed-loop-test). The one-line plant from that test is replaced by the mixer and the seesaw.

The order matters. The PID sees the rate from the end of the previous step, and its output only affects the next one. That is [how the loop closes](drone.md#how-the-loop-closes): the flight code only ever sees the result of its previous outputs.

## The mixer

The PID returns one number, and the seesaw has two motors. The mixer splits the number between them:

```
left  = base − output
right = base + output
```

- **`base`** is the throttle both motors get before the PID adds or subtracts anything. Use 0.7; [What `base` is](#what-base-is) explains it.
- **`output`** is added to one side and taken from the other. The total thrust stays about the same and only the difference changes.
- **The signs** follow from the seesaw: a stronger right motor gives a positive rate. When the measured rate is below the target the PID output is positive, so the right motor must get the `+`. With the signs swapped the loop pushes the wrong way and the rate runs off.
- **Clamp each command to 0.0–1.0** after mixing, because a motor cannot do less than stop or more than full speed.

The PID output is therefore in **throttle units**: an output of 0.05 means "right motor 5% of full throttle faster, left motor 5% slower".

Set the PID's output limits so the commands cannot leave their range. With `base = 0.7` the room above is 0.3, so `out_min = −0.3` and `out_max = 0.3` keep both commands between 0.4 and 1.0.

### What `base` is

The two numbers in the mixer do separate jobs:

- **`base` holds the drone up.** It is the same on both motors, so it produces lift but no rotation.
- **`output` rotates it.** It is added to one motor and taken from the other, so it produces rotation but almost no change in lift.

With `base = 0.7` and a PID output of 0.05:

```
left  = 0.7 − 0.05 = 0.65
right = 0.7 + 0.05 = 0.75
```

Why 0.7:

- **It matches a hover.** With 6 N of maximum thrust per motor, 0.7 gives about 3 N each (6 × 0.7²), a quarter of a 1.2 kg drone's weight.
- **It leaves room on both sides.** The PID must be able to speed one motor up and slow the other down. At a base of 1.0 there would be no room to speed up, and at 0.0 none to slow down.
- **It sets how strongly the seesaw responds.** The thrust difference is `6 × 4 × base × output`, so a higher base makes the same output push harder. The 2.8 in the arithmetic under [Starting gains](#starting-gains) is 4 × 0.7.

In the code it is a constant in `drone/sim/`, not a field of the seesaw or the PID.

Hover throttle does not lift a drone off the ground. At exactly 0.7 the thrust equals the weight, so the net force is zero: a drone on the ground stays there, light on its feet, and a drone already in the air keeps whatever vertical speed it has. Taking off needs more than 0.7, and stopping the climb means coming back to 0.7. None of this shows on the seesaw, which is held by its pivot and cannot rise or fall. In Stage 3, when the drone can move up and down, `base` stops being a constant and becomes the throttle stick or the output of a height controller.

## Units at the boundary

The two crates use different units on purpose (see [Units](drone.md#units-what-the-gyro-measures)):

| Value | In `physics` | In `flight` and the CSV |
|---|---|---|
| Rate | rad/s | °/s |
| Angle | rad | ° |
| Motor command | 0.0 to 1.0 | 0.0 to 1.0 |

The conversion happens once, in step 1 of the loop, and again for the angle when logging. 1 rad is about 57.3°, and Rust's `f32` has `to_degrees()` for it. A PID fed rad/s while its gains were chosen for °/s behaves as if every gain were 57 times too small.

## The run

A run needs a target that changes, since a loop that sits at zero shows nothing. Use a **step**: a target that jumps from one value to another.

| Time | Target | What it shows |
|---|---|---|
| 0.0 to 0.5 s | 0 °/s | the motors spin up; a balanced seesaw should not rotate |
| 0.5 to 1.5 s | 50 °/s | how quickly and cleanly the rate reaches a new target |
| 1.5 to 3.0 s | 0 °/s | the same on the way down, then holding still |

With `dt = 0.001` that is 3000 steps. The time step is fixed: step number × `dt` is the time. Nothing reads a clock.

The seesaw model has no end stops, so the angle is free to keep growing. After this run it should be near 50°, because 50 °/s for 1 s is 50°. That is a useful check on the whole chain.

## The CSV

CSV is plain text with one row per step, values separated by commas and column names on the first line. Any spreadsheet or plotting tool reads it.

| Column | Unit | Meaning |
|---|---|---|
| `time` | s | step number × `dt` |
| `target` | °/s | the setpoint |
| `rate` | °/s | the measured rate |
| `angle` | ° | the seesaw's angle |
| `output` | throttle | what the PID returned |
| `left` | 0–1 | left motor command, after clamping |
| `right` | 0–1 | right motor command, after clamping |

The first lines look like this:

```
time,target,rate,angle,output,left,right
0.000,0.0,0.000,0.000,0.00000,0.70000,0.70000
0.001,0.0,0.000,0.000,0.00000,0.70000,0.70000
```

The simplest way to write it is to print to standard output and redirect that to a file:

```
cd drone/sim
cargo run > run.csv
```

Anything that is not data (a note, a warning) should go to standard error with `eprintln!`, so it does not end up in the file.

`output`, `left` and `right` are logged as well as the rate because they explain it. A rate that climbs too slowly usually has an output stuck at its limit, and only the log shows that.

## The plot script

The script reads `run.csv` and draws the graph. It is a separate small file, `drone/sim/plot.py`, in Python with the `matplotlib` library, which needs installing once (`sudo apt install python3-matplotlib` on Ubuntu). A spreadsheet works too for a first look.

Two panels, one above the other with a shared time axis, are enough:

- **Top: `target` and `rate`.** This is the main result. The rate should follow the target.
- **Bottom: `left` and `right`.** These show what the controller did to achieve it, and whether it hit a limit.

How to read the top panel after the step at 0.5 s:

| What the rate does | Meaning | Usual fix |
|---|---|---|
| climbs slowly, no overshoot | sluggish | raise `Kp` |
| passes the target once, then settles | good; a little overshoot is normal | none |
| passes the target and swings several times | too aggressive | lower `Kp`, or add a little `Kd` |
| settles beside the target, not on it | steady-state error | add `Ki` |
| grows without limit | wrong sign in the mixer, or far too much gain | check the signs first |

## Starting gains

Start with P only: `Kp = 0.001`, `Ki = 0`, `Kd = 0`, output limits ±0.3 and `i_limit = 0.1`.

`Kp` is small because of its units. It turns an error in °/s into a throttle fraction, and one is in the tens or hundreds while the other is below 1. With `Kp = 0.001`, an error of 50 °/s gives an output of 0.05: the right motor 5% faster, the left 5% slower.

How much that moves the seesaw, using the starting values from the `physics` crate:

```
thrust difference = 6 × [(0.7 + output)² − (0.7 − output)²]
                  = 6 × 2.8 × output                = 16.8 × output   N
torque            = 16.8 × output × 0.2             = 3.36 × output   N·m
acceleration      = 3.36 × output / 0.015           = 224 × output    rad/s²
                                                    ≈ 12,800 × output °/s²
```

So an output of 0.05 accelerates the seesaw at about 640 °/s². The squares cancel in the first line, which makes the seesaw respond in proportion to the output even though thrust goes with speed squared.

Each line, step by step:

**Thrust difference.** Once the motors have settled, each one's speed equals its command, and thrust is `thrust_coeff × speed²` with `thrust_coeff = 6`:

```
right thrust = 6 × (0.7 + output)²
left thrust  = 6 × (0.7 − output)²
```

Multiply out the two squares, using `(a + b)² = a² + 2ab + b²` and `(a − b)² = a² − 2ab + b²`:

```
(0.7 + output)² = 0.49 + 1.4 × output + output²
(0.7 − output)² = 0.49 − 1.4 × output + output²
```

Subtract the second from the first. The 0.49s cancel and the `output²`s cancel, and the two middle terms add:

```
difference = 1.4 × output + 1.4 × output = 2.8 × output
```

So the thrust difference is `6 × 2.8 × output = 16.8 × output` newtons. Check with `output = 0.05`: the thrusts are `6 × 0.75² = 3.375` N and `6 × 0.65² = 2.535` N, which differ by 0.84 N, and `16.8 × 0.05` is also 0.84.

**Torque.** Torque is the thrust difference times the arm length, 0.2 m: `16.8 × output × 0.2 = 3.36 × output` N·m. The 3.36 belongs to the torque, not the acceleration.

**Acceleration.** Rotation follows the same rule as `force = mass × acceleration`, with torque in place of force and inertia in place of mass, so `acceleration = torque / inertia`. The inertia is 0.015 kg·m²: `3.36 × output / 0.015 = 224 × output` rad/s². This is the `(torque / inertia)` in the seesaw's `step`.

**Degrees.** One radian is about 57.3°, so `224 × 57.3 ≈ 12,800`. With `output = 0.05` that is `12,800 × 0.05 = 640` °/s².

## What to expect

A trial of these equations, with the run above and P only, gave:

| `Kp` | Time to reach 45 °/s | Overshoot | Verdict |
|---|---|---|---|
| 0.0005 | 0.30 s | none | sluggish |
| 0.001 | 0.15 s | 5% | good |
| 0.002 | 0.08 s | 17% | fast, some ringing |
| 0.004 | 0.05 s | 31% | too much ringing |

Your numbers should be close to these. Three things stand out:

- **P alone reaches the target exactly.** The seesaw has no drag, so holding a steady rate needs no torque at all, and the P term can be zero at the target. The one-line plant in the PID test had drag, which is why it settled at −0.333.
- **The overshoot comes from the motor lag.** The PID asks for less thrust as the rate nears the target, but the motors are 40 ms behind, so they keep pushing a little too long.
- **Raising `Kp` adds ringing here, not instability.** That changes in the last Stage 1 item: with loop delay added, a high `Kp` makes the swings grow instead of dying away.

## An experiment: one weak motor

The I term has nothing to do on a perfectly balanced seesaw. To see it work, make the right motor 10% weaker by giving it a `thrust_coeff` of 5.4 and leaving the left at 6.0.

With P only (`Kp = 0.001`), the trial gave:

- **Target 0:** the rate settles near −18 °/s, not 0.
- **Target 50:** the rate settles near 32 °/s, not 50.

The weak motor needs a constant extra command to match the other one, and P can only supply it by keeping a constant error. This is the steady-state error from [`Stage1-a.md`](Stage1-a.md#p--proportional-react-to-the-error-now), now on the seesaw.

Adding `Ki = 0.003` removes it: by the end of the 50 °/s second the rate is within 0.5 °/s of the target. There are two costs. The I term takes about a second to find the right value, so the first half second of the run is still several °/s off. And the overshoot on the step rises to about 25%.

Too much `Ki` is worse than too little. With `Ki = 0.02` the trial overshot by nearly 100% and was still swinging a second later.

## Before starting

`Pid` has no constructor yet. Its two state fields are private, so code outside the `flight` crate cannot build one with a struct literal the way the unit tests do. `sim` needs a `Pid::new(kp, ki, kd)` that sets wide default limits and leaves the limits as public fields to adjust. [`Stage1-a.md`](Stage1-a.md#practical-notes) lists it under practical notes.

## What is not in yet

- **Sensor noise and loop delay.** The gyro reading is the exact rate with no delay, which makes the loop easier to tune than a real one. Both arrive in the last Stage 1 item, with the ±30% margin check.
- **The D term.** It is not needed for a good response here. It earns its place once delay and noise are in, together with its low-pass filter.
- **A disturbance kick.** A sudden push, to test recovery, belongs to Stage 2.

## Where this goes next

- The last Stage 1 item adds sensor noise and loop delay to this simulation, then retunes. The two additions are explained in [`Stage1-d.md`](Stage1-d.md).
- Stage 2 puts an angle loop around this rate loop. The rate loop's target then comes from the angle loop instead of a table of times.
- The CSV columns and the plot script carry on through every later stage, with more columns for more axes.
