# Stage 1d — Sensor noise and loop delay

This explains the last item of Stage 1 in [`drone.md`](drone.md#stage-1--host-simulation-one-axis):

> Add sensor noise and loop delay (motor lag is already in the seesaw); retune, and check the gains still work with inertia and thrust off by ±30%

The simulation in [`Stage1-c.md`](Stage1-c.md) is too kind to the controller in two ways. The gyro reading is the exact rate, and the PID sees it at the instant it is true. A real drone gives neither. This item takes both away, so that gains chosen in the simulation have a chance of working on hardware.

Motor lag, the third imperfection in the plan, is already in: it is the first-order lag in `Motor::step`.

## The loop, before and after

Before, the PID read the seesaw's rate directly:

```
seesaw rate ───────────────────────────────► PID
```

After, a sensor model sits in between:

```
seesaw rate ─► add noise ─► hold for N steps ─► PID
 (true rate)                                    (measured rate)
```

From here on there are two rates, and it matters which one is meant:

- **True rate:** what the seesaw is really doing. Only the simulation knows it.
- **Measured rate:** what the PID is told. It is the true rate from a moment ago, with an error added.

The PID only ever sees the measured rate. Whether the loop is good is judged on the true rate.

## Sensor noise

### What it is

Every reading from a gyro is slightly wrong, by a different amount each time. With the seesaw perfectly still, a real gyro does not report 0, 0, 0. It reports something like 0.3, −1.1, 0.8, −0.2: values scattered around the truth.

It comes from two places:

| Source | Size | Notes |
|---|---|---|
| The chip itself | about 0.05 °/s | electrical noise inside the sensor; the datasheet figure, quoted here from memory |
| Vibration | several °/s | the motors and propellers shake the frame, and the gyro measures the shaking as real rotation |

Vibration is far larger, and it is the reason [`drone.md`](drone.md#how-accurate-the-model-is) asks for "noise well above the datasheet figure". A rigid-body model has no vibration at all, so the noise has to be added by hand.

### How to model it

Each step, add a random number to the true rate:

```
measured = true rate + random error
```

The random error follows a **normal distribution**, the bell curve: small errors are common, large ones rare, and positive and negative are equally likely. One number describes it, the **standard deviation**, written σ (sigma). It sets how wide the bell is:

- about 68% of the errors are within ±σ
- about 95% are within ±2σ
- almost all are within ±3σ

So with σ = 2 °/s, most readings are within 2 °/s of the truth and nearly all within 6 °/s.

Start with **σ = 2 °/s**. It is a guess standing in for vibration, about 40 times the chip's own noise. It is the right order of size for a small quad with ordinary mounting, and the real figure has to be measured from a log once there is hardware.

Two details:

- **Use a fixed seed.** A random number generator started from the same seed gives the same sequence every time. With a fixed seed, two runs with the same settings give identical CSV files, so a change in the plot is caused by the change in the gains and not by different luck.
- **A new error every step, unrelated to the last.** This is called white noise. Real vibration is not quite like that, since it is concentrated at the motors' rotation speed, but white noise is the usual first model.

### What it does to the loop

The P term passes the noise straight through, scaled by `Kp`. With `Kp = 0.001` and σ = 2 °/s, the output jitters by about 0.002 of throttle. That is small.

A trial of these equations, with `Kp = 0.001` and no delay:

| σ (°/s) | Jitter on a motor command | Wobble in the true rate | Overshoot |
|---|---|---|---|
| 0 | none | none | 4.5% |
| 1 | 0.001 | 0.03 °/s | 4.7% |
| 2 | 0.002 | 0.06 °/s | 4.9% |
| 5 | 0.005 | 0.16 °/s | 5.5% |

The jitter and wobble columns are standard deviations while holding 50 °/s.

Two things stand out:

- **The seesaw hardly notices.** The measured rate is off by 2 °/s, but the true rate wobbles by only 0.06 °/s. The motors cannot follow changes that fast, because of their lag, and the bar's inertia smooths out what is left.
- **The motors do notice.** The noise ends up in the motor commands. A little is harmless. A lot heats the motors, wastes battery and uses up output range that the controller needs for real corrections.

With P only, then, noise is a nuisance. The D term is where it becomes a problem.

### Noise and the D term

The D term looks at how much the measurement changed since the last step, divided by `dt`:

```
D term = −Kd × (measurement − previous measurement) / dt
```

Two noisy readings in a row can easily differ by 3 °/s when the true rate has not changed at all. Dividing by `dt = 0.001` turns that into an apparent 3000 °/s². The D term cannot tell this from a real change, and reacts to it at full strength.

In the trial, `Kp = 0.002` with `Kd = 0.00002` looks excellent without noise: the D term cuts the overshoot from 17% to 8%. With σ = 2 °/s added:

| | No noise | σ = 2 °/s |
|---|---|---|
| Overshoot | 8% | 9% |
| Jitter on a motor command | none | 0.065 |
| Largest swing of a motor command | none | 0.40 |

A motor command that should sit at 0.70 is being thrown between about 0.5 and 0.9, a thousand times a second. The rate still looks fine on the plot, which is why the motor commands are plotted too. Raising `Kd` to 0.00005 pushes the output into its ±0.3 limits on noise alone.

### The D-term filter

The fix is a **low-pass filter** on the D term: something that lets slow changes through and smooths away fast ones. The real changes in rate are slow, taking tens of milliseconds. The noise is fast, changing every millisecond. A filter can separate them.

The simplest low-pass filter is the first-order lag already used for the motor:

```
filtered += (raw − filtered) × dt / time_constant
```

`raw` is the unfiltered derivative, and the D term uses `filtered` in its place. The motor's lag smooths out fast changes in its command in exactly this way; here the same formula is put to use on purpose.

In the trial, with `Kp = 0.002`, `Kd = 0.00002`, σ = 2 °/s and 2 steps of delay:

| Filter time constant | Jitter on a motor command | Overshoot |
|---|---|---|
| no filter | 0.065 | 9.5% |
| 0.002 s | 0.029 | 9.4% |
| 0.005 s | 0.013 | 9.1% |
| 0.016 s | 0.007 | 8.4% |

A time constant of 0.005 s cuts the jitter to a fifth and leaves the overshoot alone. For comparison, P only at the same `Kp` has jitter of 0.004 and an overshoot of 19%.

The filter is not free. It is a lag, and a lag is a kind of delay: the D term now reacts a little late. On this model that costs nothing visible. On a real drone, too much filtering makes the D term too late to do its job, so the filter is set as light as the noise allows.

The filter is in the `Pid` in `drone/flight/`, because the firmware needs it too. It is one tuning field, `d_filter`, holding the time constant in seconds, and one piece of state, the filtered rate of change. A `d_filter` of 0 means "no filter", which is what `Pid::new` sets. The fraction `dt / d_filter` is capped at 1, so a filter shorter than `dt` passes the raw value through instead of overshooting it. `reset()` clears the filter's state along with the I term.

The filter has four tests in `pid.rs`, each with `Kd = 0.01` and `dt = 0.001`:

| Test | Setup | Expected |
|---|---|---|
| Smooths a jump | filter 0.005 s; the measurement jumps by 0.5 in one step | D term −1, a fifth of the unfiltered −5 |
| Reaches a steady slope | filter 0.005 s; the measurement rises by 0.5 every step | D term between −2.5 and −4 after one time constant, −5 after 100 steps |
| Shorter than `dt` is no filter | filter 0.0001 s; the same jump | D term −5, not more |
| Reset clears the filter | a jump, `reset()`, then one step | output 0 |

## Loop delay

### What it is

Time passes between the moment the seesaw has a certain rate and the moment the motors start to respond to it. On a real drone the pieces are:

| Where | Typical size |
|---|---|
| The gyro's own internal filtering | 1 ms or more |
| Reading the gyro over I2C or SPI | a fraction of a millisecond |
| The flight code running | a fraction of a millisecond |
| The ESC accepting the new command and acting on it | 1 ms or more |

[`drone.md`](drone.md#how-accurate-the-model-is) asks for one or two samples, which at 1 kHz is 1 to 2 ms.

Motor lag is a different thing, although both make the response late:

- **Lag:** the motor starts to respond at once, but gradually.
- **Delay:** nothing happens at all until the time is up, and then the whole change arrives.

### How to model it

Hold each reading back for N steps. The PID is given the reading from N steps ago:

```
at step 100, with N = 2:   the PID is given the reading taken at step 98
```

In code this is a queue. Each step, the new reading goes in at the back and the oldest comes out at the front. If the queue always holds N readings, the one that comes out is N steps old. Rust's `VecDeque` does this.

Details:

- **Fill the queue with zeros at the start.** The seesaw starts at rest, so "0" is the correct reading for the steps before the run began.
- **N = 0 must mean no delay,** with the new reading passed straight through. Then the results of `Stage1-c.md` can still be reproduced.
- **Noise first, then delay.** The noise is added when the reading is taken, and the noisy reading is what gets held back.
- **All the delay is modelled in one place.** On a real drone part of it is before the PID (the sensor) and part after (the ESC). For one loop the effect is the same wherever it sits, so the model puts all of it on the sensor side.

### What it does to the loop

The controller acts on old information. When the rate reaches the target, the PID is still looking at a reading from before it got there, so it keeps pushing for a moment longer. The result is more overshoot.

A trial of these equations, P only, no noise. Each entry is the overshoot:

| Delay | `Kp` = 0.001 | `Kp` = 0.002 | `Kp` = 0.004 |
|---|---|---|---|
| 0 steps | 4.5% | 17% | 31% |
| 1 step | 4.9% | 18% | 33% |
| 2 steps | 5.3% | 19% | 35% |
| 5 steps | 6.5% | 22% | 43% |
| 10 steps | 9.1% | 29% | 58% |
| 20 steps | 16% | 47% | swings do not die away |

At the gains that looked good, one or two steps of delay change very little: 4.5% becomes 5.3%. That is the expected result and a good sign, not a reason to skip the delay. The motors' 40 ms lag is 20 times larger than a 2 ms delay, so the lag dominates.

What the delay changes is the **ceiling**:

- **Without delay, no `Kp` makes this loop unstable.** A higher `Kp` only adds ringing, which always dies away in the end.
- **With delay, there is a `Kp` above which the swings grow instead of dying away.** The more delay, the lower that ceiling.

In the trial, the ceiling was near:

| Delay | `Kp` where the swings stop dying away |
|---|---|
| 0 steps | none |
| 2 steps | about 0.03 |
| 5 steps | about 0.012 |
| 10 steps | about 0.008 |

With 2 steps of delay the ceiling is 30 times the good gain of 0.001, which is a comfortable margin. With 20 steps it would be only about 4 times.

Why delay causes this: the correction arrives late. If it arrives late enough, the seesaw has already passed the target and is coming back, and the "correction" now pushes it the way it is already going. Each swing is then larger than the last. A playground swing pushed at the wrong moment behaves the same way.

A growing swing does not grow for ever in this simulation. The output limit of ±0.3 caps it, and the rate ends up swinging back and forth at a steady size. On the plot that shows as a rate that never settles and motor commands banging between 0.4 and 1.0.

## Where the code goes

The noise and the delay are both properties of the sensor. [`drone.md`](drone.md#flight-code-one-no_std-crate-two-hosts) puts the sensor model in `drone/physics/`, so it is a new module there, next to `motor` and `seesaw`. Stage 2 reuses it for all three axes.

A sketch of its shape:

```
Gyro
    parameters:  noise standard deviation, delay in steps
    state:       the random number generator, the queue of held readings

    read(true rate) -> measured rate
```

In the loop in `drone/sim/`, step 1 changes from "read the seesaw's rate" to "read the seesaw's rate and pass it through the gyro". Nothing else in the loop moves.

Other changes that come with it:

| Where | Change |
|---|---|
| `drone/physics/` | needs random numbers. The usual crates are `rand` and `rand_distr`; the second supplies the normal distribution |
| `drone/sim/` arguments | `--noise` (σ in °/s), `--delay` (steps) and `--seed`, all defaulting to 0 so that plain `cargo run` gives the same result as now |
| `drone/sim/` arguments | `--thrust-coeff`, for the ±30% margin check; `--inertia` is already there |
| CSV | one new column, `measured`: what the PID was given. `rate` stays the true rate |
| `plot.py` | draw `measured` as a thin, pale line behind `rate` in the top panel |
| `drone/flight/` | the D-term filter in `Pid`, and a `--d-filter` argument in `drone/sim/` to set it |

The `physics` crate works in rad/s, so the gyro's σ is in rad/s inside the crate. `sim` takes `--noise` in °/s, as that is how it is talked about, and converts it once.

## Tests for the sensor model

| Test | Setup | Expected |
|---|---|---|
| No noise, no delay | σ = 0, N = 0 | the reading equals the true rate exactly |
| Delay only | σ = 0, N = 3; feed 1, 2, 3, 4, 5 | readings 0, 0, 0, 1, 2 |
| Noise averages out | σ = 2, N = 0; 10,000 readings of a true rate of 10 | the average is within about 0.1 of 10 |
| Noise has the right size | the same 10,000 readings | their standard deviation is within about 5% of 2 |
| Same seed, same readings | two gyros with the same seed | identical readings |
| Different seed, different readings | two gyros with different seeds | the readings differ |

The two noise tests need a tolerance, because the result is random. With a fixed seed they still give the same answer on every run.

## What to expect on the plot

With `--noise 2 --delay 2` and `Kp = 0.001`, the trial gave an overshoot of 5.7%, against 4.5% before. So:

- **The true rate** looks almost the same as in `Stage1-c.md`.
- **The measured rate** is a fuzzy band about ±4 °/s wide around it.
- **The motor commands** have a fine fuzz on them, about ±0.004.

The interesting plots are the ones that go wrong: a `Kd` with no filter, where the bottom panel fills with noise, and a large `--delay`, where the rate stops settling.

## The retune

With noise and delay on, the aim is gains that respond quickly, overshoot little and keep the motor commands quiet. These are the Rust simulation's own figures, with `--noise 2 --delay 2`. The pictures are in [`drone/sim/stage1-d/`](drone/sim/stage1-d/README.md), numbered in this order, and `./stage1-d.sh` in `drone/sim/` regenerates them.

| | Gains | Rise | Overshoot | Settle | Motor jitter |
|---|---|---|---|---|---|
| 01 | `Kp` 0.001, no noise or delay | 0.146 s | 4.5% | 0.160 s | none |
| 02 | `Kp` 0.001 | 0.144 s | 4.6% | 0.158 s | 0.002 |
| 03 | `Kp` 0.0005 | 0.300 s | none | 0.363 s | 0.001 |
| 04 | `Kp` 0.002 | 0.081 s | 18% | 0.201 s | 0.004 |
| 05 | `Kp` 0.004 | 0.050 s | 35% | 0.285 s | 0.008 |
| 06 | `Kp` 0.002, `Kd` 0.00002 | 0.092 s | 8.5% | 0.194 s | 0.062 |
| 07 | `Kp` 0.00125, `Kd` 0.000005 | 0.123 s | 6.0% | 0.237 s | 0.017 |
| 08 | `Kp` 0.001, `Ki` 0.001 | 0.134 s | 12.5% | 0.545 s | 0.002 |
| 09 | `Kp` 0.002, `Kd` 0.00002, filter 0.005 s | 0.091 s | 8.1% | 0.190 s | 0.013 |
| 10 | `Kp` 0.002, `Kd` 0.00004, filter 0.005 s | 0.106 s | 1.7% | 0.118 s | 0.021 |
| 11 | `Kp` 0.002, `Kd` 0.00004, filter 0.01 s | 0.105 s | 0.8% | 0.118 s | 0.013 |

Rise is the time to reach 45 °/s, settle the time until the rate stays within 2.5 °/s of the target, and motor jitter the standard deviation of a motor command while holding 50 °/s.

**The result: `Kp = 0.002`, `Ki = 0`, `Kd = 0.00004`, with a D-term filter of 0.01 s** (picture 11). Against the Stage 1c gains with the same noise and delay (02), it reaches the target sooner (0.105 s against 0.144 s), overshoots less (0.8% against 4.6%) and settles sooner (0.118 s against 0.158 s). The price is motor jitter of 0.013 against 0.002: the commands move by about ±0.04 of throttle while holding still.

How the retune got there:

- **Noise and delay at this size hardly touch a P-only loop** (01 against 02).
- **`Kp` alone trades speed for overshoot** (03, 04, 05). Doubling it to 0.002 gets to the target sooner but settles later, because of the ringing. With P only, 0.001 is the best there is.
- **D without a filter works, and cannot be afforded** (06). It cuts the overshoot of `Kp = 0.002` from 18% to 8.5%, and multiplies the motor jitter by 15. Even a quarter of that `Kd` gives 8 times the jitter of P only (07).
- **I only hurts here** (08). The seesaw is balanced, so there is no steady error for it to remove.
- **The filter makes D affordable** (09). The same gains as 06 give the same response with a fifth of the jitter.
- **That leaves room for more D** (10). Twice the `Kd` takes the overshoot from 8% to under 2%, and the higher `Kp` no longer rings.
- **A longer filter costs nothing here** (11). Going from 0.005 s to 0.01 s leaves the response alone and nearly halves the jitter. The motors' own 40 ms lag is so much longer than either filter that the extra lateness does not show.

The chosen gains are also less sensitive to the model being wrong, which is the subject of the margin check. A quick look, as an average over three seeds:

| Change | P only, `Kp` 0.001 | Chosen gains |
|---|---|---|
| none | overshoot 5.1% | 1.7% |
| delay of 5 steps instead of 2 | 6.4% | 2.5% |
| inertia 30% low and thrust 30% high | 17% | 1.8% |
| inertia 30% high and thrust 30% low | 0.2%, rise 0.27 s | 0.6%, rise 0.17 s |
| noise of 5 °/s instead of 2 | motor jitter 0.005 | motor jitter 0.031 |

The last row is the weak point: the jitter grows in step with the noise, and real vibration may be worse than 2 °/s. If it is, the filter gets longer or `Kd` comes down.

Two things are still provisional:

- **`Ki` is 0.** It becomes necessary with a weak motor, as in [`Stage1-c.md`](Stage1-c.md#an-experiment-one-weak-motor), and has to be chosen against that case.
- **The filter length depends on this model's slow motors.** A drone with quicker motors has less room for a 0.01 s filter.

The simulation's defaults are still the Stage 1c gains, so that plain `cargo run` and the pictures in `drone/sim/runs/` stay reproducible. The chosen gains are `--kp 0.002 --kd 0.00004 --d-filter 0.01`.

## The margin check

The last part of the Stage 1 item: do the chosen gains still work when the model is wrong? Each run uses the chosen gains with `--noise 2 --delay 2`, and changes the inertia, the thrust coefficient or both by 30%.

| Inertia | Thrust | Rise | Overshoot | Settle | Motor jitter |
|---|---|---|---|---|---|
| as modelled | as modelled | 0.105 s | 0.8% | 0.118 s | 0.013 |
| 30% low | as modelled | 0.082 s | 1.6% | 0.092 s | 0.013 |
| 30% high | as modelled | 0.128 s | 0.9% | 0.147 s | 0.013 |
| as modelled | 30% low | 0.138 s | 0.9% | 0.160 s | 0.013 |
| as modelled | 30% high | 0.088 s | 1.5% | 0.098 s | 0.013 |
| 30% low | 30% high | 0.069 s | 1.2% | 0.076 s | 0.013 |
| 30% high | 30% low | 0.172 s | 0.6% | 0.203 s | 0.013 |
| 30% low | 30% low | 0.105 s | 0.8% | 0.118 s | 0.013 |
| 30% high | 30% high | 0.105 s | 0.8% | 0.118 s | 0.013 |

The gains pass. The overshoot stays under 2% in every case and nothing rings. What changes is the speed: the rise time runs from 0.07 s on the liveliest seesaw to 0.17 s on the most sluggish.

Two things the table shows:

- **Only the ratio matters.** The seesaw's response depends on thrust divided by inertia, so when both are off by the same amount the result is identical to the modelled one. The two worst cases are the ones where they are off in opposite directions.
- **The jitter does not change.** It is set by the noise and the gains, not by the seesaw.

To repeat a row, add for example `--inertia 0.0105 --thrust-coeff 7.8` to the chosen gains and pass the CSV to `measure.py`.

## What comes after

Stage 1 is complete. Stage 2 gives the same sensor model two more axes and an accelerometer.
