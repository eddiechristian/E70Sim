# Stage 2b — The estimator

This explains the second item of Stage 2 in [`drone.md`](drone.md#stage-2--host-simulation-self-levelling-attitude):

> Complementary filter; compare its angles with the true angles from the physics

The rate loops of [`Stage2-a.md`](Stage2-a.md) hold a rotation rate. Self-levelling needs more: the drone has to know its roll and pitch angle, so that the next item can steer them back to zero. No sensor measures those angles. The estimator works them out from the gyro and the accelerometer.

[The estimator](drone.md#the-estimator) in `drone.md` gives the idea in a page. This document turns it into formulas, code and tests.

## What is in and what is not

| In this item | Not yet |
|---|---|
| an accelerometer reading from the body | the angle loop that uses the estimate (the next item) |
| roll and pitch angles from the accelerometer | a yaw angle (no sensor on this drone can correct it) |
| the complementary filter | movement and thrust in the accelerometer reading (Stage 3) |
| the estimate plotted against the true angle | the Mahony filter |

## The two sources

| | Gyro | Accelerometer |
|---|---|---|
| Measures | rotation rate | which way gravity pulls |
| Gives the angle by | adding up rate × time | trigonometry on one reading |
| Good at | fast changes; smooth | the long run; never drifts |
| Bad at | the long run: every small error is added up | fast changes: noisy, and wrong when the drone accelerates |

Each is good exactly where the other is bad. The filter takes the fast part from the gyro and the slow part from the accelerometer.

## The accelerometer

### What it measures

An accelerometer does not measure gravity. It measures the push that stops it falling.

Hold one still on a table. Gravity pulls it down, the table pushes it up, and it does not move. The sensor feels the table's push, so it reads 1 g **upwards**. Drop it, and while it falls it reads zero, although gravity is still pulling.

The drone in this stage is held by a ball joint at its centre, as in Stage 2a. The joint is the table: it pushes up with exactly the drone's weight. So the reading is always 1 g, pointing straight up, and what changes as the drone tilts is how that upward push looks from the drone's own axes.

The accelerometer has three sensing axes, lined up with the drone's body axes, and gives one number for each. This document calls them `ax`, `ay` and `az`:

| Name | The push felt along | Positive when the push is towards |
|---|---|---|
| `ax` | the body's x axis | the nose |
| `ay` | the body's y axis | the right side |
| `az` | the body's z axis | the belly (down) |

The `a` is for acceleration. Together the three numbers are one arrow: its direction is where the push comes from, and its length is how strong the push is. They are in m/s² here, and 9.81 m/s² is 1 g.

In the body frame of [Stage 2a](Stage2-a.md#the-body-frame), z points down. Level and still, the reading is therefore:

```
ax =  0
ay =  0
az = −9.81 m/s²        (up is the negative z direction)
```

### The reading at any attitude

Tilt the drone and the same upward push is shared between the three axes. With roll angle φ and pitch angle θ:

```
ax =  g × sin θ
ay = −g × sin φ × cos θ
az = −g × cos φ × cos θ
```

Checking the signs against the body frame:

- **Nose up (positive pitch):** the x axis now points forward and a little up. The upward push has a part along it, so `ax` is positive.
- **Right side down (positive roll):** the y axis now points right and a little down. The upward push is against it, so `ay` is negative.
- **Yaw does not appear.** Turning about the vertical changes nothing the accelerometer can feel. This is why it cannot correct a yaw angle.

Three values, which the tests use:

| Attitude | ax | ay | az |
|---|---|---|---|
| level | 0 | 0 | −9.807 |
| pitch +30° | 4.903 | 0 | −8.493 |
| roll +30° | 0 | −4.903 | −8.493 |

### From the quaternion

The physics holds the attitude as a quaternion, not as angles, so the model computes the reading from that. With `g = 9.80665`:

```
ax = −g × 2 × (x×z − w×y)
ay = −g × 2 × (y×z + w×x)
az = −g × (1 − 2 × (x² + y²))
```

Here `w`, `x`, `y` and `z` are the quaternion's four numbers. This gives the same values as the angle formulas above and has no trouble at 90° of pitch.

### Angles from a reading

Going the other way, from a reading `(ax, ay, az)` to the two angles:

```
roll  = atan2(−ay, −az)
pitch = atan2( ax, √(ay² + az²))
```

- **Only the direction matters, not the size.** Doubling all three numbers gives the same angles. So the unit is free: m/s², g or raw counts all work.
- **`atan2`, not `atan`.** It takes the two parts separately, so it handles every quadrant and does not divide by zero.

### Noise

The accelerometer is far noisier than the gyro in practice. The chip's own noise is small; the motors' vibration is not, and the sensor reports the shaking as real acceleration.

An error of 0.5 m/s² on one axis tilts the computed direction by about 3°. On a real frame the vibration can be several times that. Used by itself, the accelerometer's angle would be useless for control. Averaged over a second, it is very good, and that is what the filter does.

## The gyro's side

Adding up the gyro rate gives the angle:

```
roll  = roll  + roll rate  × dt
pitch = pitch + pitch rate × dt
```

This is smooth and has no lag. Its weakness is **bias**: a real gyro at rest reads a small rate that is not zero, typically a fraction of a degree per second. Adding up a bias of 1 °/s gives an error of 1° after a second and 60° after a minute.

The simulated gyro has no bias yet. One is added in this item, as an argument, so that the drift can be seen and the correction tested.

## The complementary filter

Each step, for roll and for pitch:

```
angle = α × (angle + gyro rate × dt) + (1 − α) × accelerometer angle
```

- **The first part** moves the estimate by what the gyro saw this step.
- **The second part** pulls it a small fraction of the way towards the accelerometer's answer.
- **The two weights add up to 1,** which gives the filter its name.

With α close to 1, each accelerometer reading barely moves the estimate, so its noise is averaged away. But the pull never stops, so over time the estimate cannot drift from where the accelerometer says it should be.

### Use a time constant, not α

`drone.md` gave 0.98 as a starting value for α. That number means nothing without the loop rate, because the blend happens once per step. At 1 kHz, α = 0.98 pulls towards the accelerometer a thousand times a second and trusts it far too much.

The honest setting is a **time constant** τ, in seconds: roughly how long the estimate takes to follow the accelerometer. The code works α out from it:

```
α = τ / (τ + dt)
```

| τ | α at 1 kHz | α at 250 Hz |
|---|---|---|
| 0.05 s | 0.980 | 0.926 |
| 0.5 s | 0.9980 | 0.9921 |
| 1 s | 0.9990 | 0.9960 |
| 2 s | 0.9995 | 0.9980 |

So α = 0.98 at 1 kHz is a time constant of 0.05 s. Start with **τ = 1 s**.

This is the same choice as `d_filter` in the PID: a time in seconds keeps its meaning when the loop rate changes, and a per-step fraction does not.

### What τ trades

| | Short τ (0.1 s) | Long τ (2 s) |
|---|---|---|
| Trusts | the accelerometer more | the gyro more |
| Accelerometer noise in the estimate | more | less |
| Error left by a gyro bias | small | large |
| Disturbed by sideways acceleration | quickly | slowly |

Two of these can be put in numbers:

- **A gyro bias leaves a constant error of bias × τ.** It no longer grows. A bias of 1 °/s with τ = 1 s leaves the estimate 1° off for ever, where the gyro alone would be 20° off after 20 seconds.
- **After a time τ the estimate has covered 63% of any gap** between itself and the accelerometer's answer, 86% after 2τ and 95% after 3τ.

### The first reading

At power-on the estimate is not known. Starting it at zero is wrong if the drone is sitting on a slope, and the filter would then take several seconds to pull it right.

So the first call sets the estimate straight to the accelerometer's angle and returns. This is the same idea as the PID's first call, which has no previous measurement to take a derivative from.

## Turning while tilted

The gyro's side above treated roll and pitch as separate: roll rate changes roll, pitch rate changes pitch. That is exact while the drone turns about one axis at a time. It is not exact in general.

Tilt the drone 30° to the right and then yaw it a quarter turn. Nothing rolled or pitched as far as the gyro is concerned; only the yaw gyro read anything. But the tilt that was to the right is now towards the nose or the tail: the roll angle has become a pitch angle. The simple filter does not know. Its gyro side saw no roll or pitch rate, and only the slow pull of the accelerometer brings the estimate round.

A trial with τ = 1 s:

| Case | Simple version, worst error | With the extra terms |
|---|---|---|
| roll step of 50 °/s for 1 s | 0° | 0° |
| tilted 10°, yawing at 20 °/s | 2.7° | 0° |
| tilted 30°, yawing at 90 °/s | 22° | 0° |

The fix is to use the full relation between the body rates and the change of the two angles. With `p`, `q` and `r` as the roll, pitch and yaw rates from the gyro, and φ and θ the current estimates:

```
change of roll  = p + (q × sin φ + r × cos φ) × tan θ
change of pitch = q × cos φ − r × sin φ
```

At level, with φ and θ both zero, these reduce to `p` and `q`, which is the simple version.

- **Write the simple version first** and get its tests passing.
- **Then add these terms.** They cost three lines and remove the largest error of the simple version. A gentle hover hardly yaws, so the simple version would fly; the angle loop of the next item will be better with the full one.
- **`tan θ` grows without limit near 90° of pitch.** That is the same breakdown of three angles as in Stage 2a. A self-levelling hover stays far from it. Limiting the estimate's pitch to ±85° before taking the tangent keeps the arithmetic finite.

## Limits

- **No yaw angle.** The accelerometer cannot see yaw, so nothing corrects the yaw gyro's drift. Yaw stays under rate control only.
- **Sustained acceleration fools it.** The filter assumes that, on average, the only push on the drone is the one holding it up. A long sideways acceleration tilts the accelerometer's answer, and the estimate slowly follows. The longer τ is, the slower it follows.
- **Free flight is harder than the ball joint.** In this stage the joint supplies the upward push, so the reading is exactly gravity's opposite. A flying drone is held up by its own thrust, which always points along the body's own z axis however the drone is tilted. With thrust alone, the accelerometer would read "level" at every angle. Real drones are saved by air drag on the propellers, which adds the missing sideways part. Stage 3 has to model that drag for the estimator to keep working, and it is the main thing to check there.
- **The angles wrap.** Near ±180° of roll the blend of two angles goes wrong, because 179° and −179° are 2° apart and their average is not 0°. A hover is nowhere near.

## Shape of the code

A sketch, to make the above concrete:

```
drone/physics/
    quad        accel() -> Accel            what the accelerometer reads, m/s²
    gyro        a bias, added to every reading

drone/flight/
    estimator   Estimator
                    update(GyroRates, Accel, dt) -> Attitude
                    reset()
                angles_from_accel(Accel) -> Attitude

drone/sim/
    bin/quad    three more sensors, the estimator, and two new CSV columns
```

The types, following the rule of [Stage 2a](Stage2-a.md#shape-of-the-code) that nothing crosses a function boundary as a bare tuple:

| Type | Crate | Fields | Unit |
|---|---|---|---|
| `Accel` | `physics` | `x`, `y`, `z` | m/s² |
| `Accel` | `flight` | `x`, `y`, `z` | g |
| `GyroRates` | `flight` | `roll`, `pitch`, `yaw` | °/s |
| `Attitude` | `flight` | `roll`, `pitch` | ° |

`Accel` exists twice, as `MotorCommands` does, because neither crate depends on the other. The simulation converts between them, dividing by 9.80665.

A sketch of the estimator:

```rust
pub struct Estimator {
    // Tuning
    pub time_constant: f32, // s; how long the estimate takes to follow the accelerometer. Start with 1.0

    // State
    attitude: Option<Attitude>, // None until the first update
}

impl Estimator {
    pub fn update(&mut self, gyro: GyroRates, accel: Accel, dt: f32) -> Attitude {
        let from_accel = angles_from_accel(accel);

        // The first reading sets the estimate.
        let Some(previous) = self.attitude else {
            self.attitude = Some(from_accel);
            return from_accel;
        };

        let alpha = self.time_constant / (self.time_constant + dt);
        let attitude = Attitude {
            roll: alpha * (previous.roll + gyro.roll * dt) + (1.0 - alpha) * from_accel.roll,
            pitch: alpha * (previous.pitch + gyro.pitch * dt) + (1.0 - alpha) * from_accel.pitch,
        };
        self.attitude = Some(attitude);
        attitude
    }
}
```

This is the simple version, without the terms of [Turning while tilted](#turning-while-tilted). The real code is in `drone/flight/src/estimator.rs`; it has those terms and the check on `dt`.

### Choices to make in the code

- **The flight crate is `no_std`, and `atan2`, `sqrt`, `sin`, `cos` and `tan` are not in `core`.** They are methods of `f32` only when `std` is present. The usual answer is the `libm` crate, which gives `libm::atan2f`, `libm::sqrtf` and the rest and works on the Cortex-M7 and the host alike. This will be the flight crate's first dependency. The tests build with `std`, so a plain `cargo build` in `drone/flight/` is what catches a function used by mistake.
- **A bad `dt`.** As in the PID: with `dt` not a positive number, return the current estimate unchanged, with a `debug_assert!`.
- **A reading of all zeros.** `atan2(0, 0)` is 0, so the code does not fail, but the answer is meaningless. A broken or absent sensor must be caught before the estimator, as a NaN measurement must be caught before the PID.
- **Degrees and g in the flight crate.** Angles in degrees match the PIDs, and g is the unit the driver gets from the chip (2048 counts per g at ±16 g). The trigonometric functions need radians, so the estimator converts inside.
- **`reset()`** sets the state back to `None`, so the next reading sets the estimate again.

### The sensors in the physics

- **`Quad::accel()`** returns the reading from the quaternion, with the formulas above. It is exact; noise is added outside it.
- **Noise and delay** can come from the existing `Gyro`, which is a noise-and-delay stage for one number and does not care what the number is. Three more of them serve the accelerometer's axes.
- **A gyro bias** is one new field, added to every reading. Zero by default, so every earlier run stays the same.
- **A delay starts with zeros, which is wrong for the accelerometer.** The `Gyro` fills its queue with zeros for the readings "from before the run", which is right for a rate at rest. For the accelerometer's z axis the reading at rest is −9.807, not 0. With a delay of two steps, the estimator's first two readings would be (0, 0, 0), and its first-reading rule would start from a meaningless angle. Either run the accelerometer with no delay, or give the queue a starting value.

## Tests

### The accelerometer reading (`physics`)

| Test | Setup | Expected |
|---|---|---|
| Level | a new quad | (0, 0, −9.807) |
| Pitched | turn 30° about pitch | (4.903, 0, −8.493) |
| Rolled | turn 30° about roll | (0, −4.903, −8.493) |
| Level, any yaw | turn any amount about yaw | still (0, 0, −9.807) |
| Always 1 g | any attitude | the length is 9.807 |

The yaw test starts level on purpose. Yawing a drone that is already tilted does change `ax` and `ay`, because the tilt moves round the body; [Turning while tilted](#turning-while-tilted) is about exactly that.

### Angles from a reading (`flight`)

| Test | Reading, in g | Expected |
|---|---|---|
| Level | (0, 0, −1) | roll 0°, pitch 0° |
| Nose up | (0.5, 0, −0.866) | pitch 30°, roll 0° |
| Right side down | (0, −0.5, −0.866) | roll 30°, pitch 0° |
| Nose down, left side down | signs reversed | −30° on each |
| Size does not matter | the nose-up reading × 2 | the same angles |

### The filter (`flight`)

All with τ = 1 s and `dt` = 0.001 s unless stated.

| Test | Setup | Expected |
|---|---|---|
| First reading | one update with a reading for 20° of roll | roll 20° at once |
| Gyro alone | τ very large; level reading; roll rate 10 °/s for 1 s | roll 10° |
| Stays still | level reading, zero rates, 5 s | roll and pitch stay 0 |
| Follows the accelerometer | start at 0; then a reading for 10° of roll, zero rates | 6.3° after 1 s, 8.6° after 2 s, 9.5° after 3 s |
| Bias is bounded | level reading; roll rate 1 °/s (a bias) for 20 s | roll settles at 1.0°, not 20° |
| Bias scales with τ | the same with τ = 0.5 s and τ = 2 s | 0.5° and 2.0° |
| Noise is averaged | level reading with noise of 0.05 g on each axis, 20 s | the estimate stays within about 0.3° of 0 |
| `dt` independence | the "follows" test at 1 kHz and at 500 Hz | the same angles, within 0.05° |
| Reset | run, `reset()`, then one update | the same as a new estimator |
| Bad `dt` | `dt` of 0, negative or NaN | panics in debug; the estimate is unchanged |

After the extra terms are added:

| Test | Setup | Expected |
|---|---|---|
| Level, the terms vanish | zero roll and pitch; yaw rate 90 °/s | roll and pitch stay 0 |
| Yaw moves a tilt round | τ very large; start at roll 30°; yaw rate 90 °/s for 1 s | roll 0°, pitch −30° |

The second one has no accelerometer correction, so it checks the terms alone. It fails on the simple version, which leaves roll at 30°.

### The estimator on the body (`sim`)

These run the real body and feed its gyro and accelerometer readings to the estimator, as the "two together" tests of Stage 2a did for the mixer. They catch a sign that differs between the body's accelerometer and the estimator's formulas.

| Test | Motion of the body | Expected |
|---|---|---|
| Roll agrees | a roll rate step to +30° | estimated roll within 0.5° of the true roll throughout |
| Pitch agrees | the same about pitch | estimated pitch within 0.5° of the true pitch |
| Signs agree | −30° on each | the same, negative |
| Tilt then yaw | roll to 30°, then yaw at 90 °/s for 1 s | within 1° with the extra terms |

## In the simulation

The quad program of [Stage 2a](Stage2-a.md#in-the-simulation) gains:

- **Three accelerometer axes,** each with its own noise.
- **The estimator,** run every step after the sensors are read.
- **Arguments:** `--accel-noise` (m/s²), `--gyro-bias` (°/s) and `--estimator-tau` (s).
- **CSV columns:** `roll_estimate` and `pitch_estimate`, next to the true `roll_angle` and `pitch_angle` that are already there.
- **A plot** of estimate against truth for roll and pitch. This is the "compare its angles with the true angles" of the checklist item.

The estimate is not used for control yet. The rate loops run as before, and the estimator only watches. That keeps this item's question simple: is the estimate right?

## Results

The estimator, the accelerometer reading and the gyro bias are written, with the tests above, and the quad program runs them. `./stage2-b.sh` in `drone/sim/` saves the pictures in [`drone/sim/stage2-b/`](drone/sim/stage2-b/README.md). Each run is 3 s, with the rate loops turning the body to 50° of roll unless stated.

| Run | Start | End | Wobble |
|---|---|---|---|
| no noise | 0° | 0° | none |
| gyro noise 2 °/s, delay 2 steps, accelerometer noise 0.5 m/s² | 1.41° | 0.09° | 0.05° |
| the same, target rate 0, gyro bias 1 °/s | 1.41° | 1.01° | 0.03° |
| the same bias, τ = 0.5 s | 1.41° | 0.52° | 0.07° |
| noise, τ = 0.05 s (α = 0.98 at 1 kHz) | 1.41° | 0.01° | 0.28° |
| noise, τ = 5 s | 1.41° | 0.82° | 0.08° |

"Start" is the error of the first estimate, "end" the average error over the last second and "wobble" its standard deviation there, all for roll.

- **The formulas hold.** With no noise the estimate is exact through a turn to 50°. A bias of 1 °/s leaves 1° with τ = 1 s and half that with τ = 0.5 s.
- **τ = 1 s is a reasonable start.** The 0.05 s that α = 0.98 would have meant gives about six times the wobble. At 5 s the estimate is quiet but has hardly begun to correct after 3 s.
- **On the real body, turning while tilted is handled.** In the test that tilts the body about 25° and spins it up to 87 °/s of yaw, the worst error is 0.02°. With the simple version in its place the same test gives 23°.

Two things that the runs showed and the plan above did not foresee:

- **The first estimate is one noisy reading.** With accelerometer noise of 0.5 m/s², it starts 1.4° off, and the filter then needs a few time constants to recover. That is acceptable for these runs. Before the angle loop relies on it, the start should be the average of a number of readings taken while the drone sits still. [Calibration before flight](drone.md#calibration-before-flight) in `drone.md` describes that step, which comes with the angle loop.
- **A gyro bias moves the drone, not only the estimate.** The rate loops trust the gyro. With a gyro reading 1 °/s too high they hold the body at −1 °/s to see zero, and the drone slowly turns. The estimator cannot fix that; a real flight controller measures the bias while still on the ground and subtracts it, in the same [calibration](drone.md#calibration-before-flight) step. The angle loop of the next item will also counter it, since it steers by the angle.

## What to expect

This section is the forecast made before the code was written, from a trial of the formulas. [Results](#results) has what the real code gave.

With τ = 1 s:

| Run | Result |
|---|---|
| a roll step of 50 °/s for 1 s, no noise | the estimate lies on top of the true angle, ending at 50° |
| the same with accelerometer noise of 0.5 m/s² | the estimate wobbles by about 0.1°, where the accelerometer's own angle jumps by 3° |
| a gyro bias of 1 °/s, drone still | the estimate settles 1° off and stays there |
| the same bias with τ = 0.5 s | 0.5° off, with more wobble from accelerometer noise |
| tilted 30°, then yaw at 90 °/s, simple version | up to 22° wrong while it turns |
| the same with the extra terms | no visible error |

The picture to look for is the estimate hugging the true angle through fast moves (the gyro's doing) and not creeping away over a long run (the accelerometer's doing).

## Where this goes next

- The next Stage 2 item puts an angle loop around the rate loops on roll and pitch, and starts each run with a short [calibration](drone.md#calibration-before-flight). It is explained in [`Stage2-c.md`](Stage2-c.md). Its measurement is this estimate, and from then on an estimator error is a flight error.
- The disturbance tests after that include a start from 30°, which the first-reading rule above makes possible.
- Stage 3 adds movement and thrust. The accelerometer then reads thrust and drag instead of the ball joint's push, which is the hard case described under [Limits](#limits).
- If the per-angle arithmetic proves inadequate, the Mahony filter does the same job on a quaternion, with no trouble at 90° of pitch or at ±180° of roll.
