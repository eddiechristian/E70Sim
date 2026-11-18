# Stage 1a — The PID controller

This explains the first item of Stage 1 in [`drone.md`](drone.md#stage-1--host-simulation-one-axis):

> `drone/flight/`: PID with output limits and integrator anti-windup; unit tests

It covers what a PID is, why its output must be limited, what integrator windup is and how to prevent it, and what the code and its tests should look like.

The examples use the inner rate loop of one axis (see [Inner and outer loops](drone.md#inner-and-outer-loops)): the target is a rotation rate in °/s, the measurement is the gyro rate, and the output is a torque command. The gyro rate is also in °/s, converted from the chip's raw counts before the PID sees it (see [Units: what the gyro measures](drone.md#units-what-the-gyro-measures)).

## What a controller does

A controller is given two numbers and produces one:

- **Setpoint:** what I want (target rate, for example 0 °/s).
- **Measurement:** what I have (gyro rate, for example 20 °/s).
- **Output:** what to do about it (torque command).

The difference between the first two is the **error**:

```
error = setpoint − measurement
```

A PID computes the output from the error in three ways and adds the results:

```
output = P term + I term + D term
```

## The three terms

### P — proportional: react to the error now

```
P term = Kp × error
```

The bigger the error, the harder the push. With `Kp = 0.5` and an error of −20 °/s, the P term is −10.

P alone has two problems:

- **Too low** and the response is sluggish. **Too high** and the drone overshoots and oscillates.
- **It cannot cancel a steady disturbance.** P only pushes when there is an error. If one motor is weaker than the others, the drone needs a constant corrective torque to stay still. P can only produce that torque by tolerating a constant error. This leftover is called *steady-state error*.

### I — integral: react to error that persists

```
integral = integral + error × dt
I term   = Ki × integral
```

The integral is a running total of the error over time. While any error remains, the total keeps growing, so the I term keeps pushing harder until the error is gone. It then holds whatever value was needed to get there.

This is what removes steady-state error. With the weak motor, the integral settles at exactly the corrective torque required, and the error returns to zero.

The cost is that the integral is a memory of the past. It reacts slowly, and too much `Ki` causes slow, swinging oscillations.

### D — derivative: react to how fast things are changing

```
D term = Kd × (rate of change of error)
```

D looks at the trend. If the error is shrinking fast, D pushes the other way to slow the approach. It acts as a brake that damps the overshoot P would cause, which allows a higher `Kp`.

D has two practical problems, and both shape the code:

- **Derivative kick.** If the setpoint jumps (a stick is moved), the error jumps in one step, and its rate of change is enormous for that step. The output spikes. The fix is to take the derivative of the *measurement* instead of the error:

  ```
  D term = −Kd × (measurement − previous measurement) / dt
  ```

  While the setpoint is constant the two are identical, because the error changes only when the measurement does. When the setpoint jumps, the measurement does not, so there is no spike. The worked example below shows the numbers.

- **Noise.** Differentiating amplifies fast changes, and gyro noise is nothing but fast changes. The D term needs a low-pass filter. This is not part of Stage 1's first item; it arrives with the item "Add motor lag, sensor noise and loop delay", where the noise first appears.

### Derivative kick, worked through

"A stick is moved" means the pilot changes what they are asking for. The setpoint is the target the pilot (or the outer loop) sets, and it can change instantly: one loop step it says "rotate at 0 °/s", the next it says "rotate at 100 °/s". The drone cannot change instantly, because it has mass and the motors take time.

A derivative is "how much did the value change since the last step, divided by how long that took":

```
rate of change = (value this step − value last step) / dt
```

Take `Kd = 0.01` and `dt = 0.001` s (1 kHz). The drone sits still, and at step 2 the setpoint jumps to 100 °/s:

| Step | Setpoint | Measurement | Error = setpoint − measurement |
|---|---|---|---|
| 1 | 0 | 0 | 0 − 0 = 0 |
| 2 | 100 | 0 | 100 − 0 = 100 |
| 3 | 100 | 0.5 | 100 − 0.5 = 99.5 |

At step 2 the drone has not moved yet, so the measurement is still 0. By step 3 the motors have started to turn it, and the gyro reads 0.5 °/s.

**Derivative of the error.** The values come from the Error column:

```
step 2:  D = Kd × (error at step 2 − error at step 1) / dt
           = 0.01 × (100 − 0) / 0.001
           = 0.01 × 100,000
           = 1000

step 3:  D = Kd × (error at step 3 − error at step 2) / dt
           = 0.01 × (99.5 − 100) / 0.001
           = 0.01 × (−500)
           = −5
```

The error changed by 100 in one millisecond, which the arithmetic reads as "changing at 100,000 °/s per second". The D term is 1000 for a single step and then −5. That one-step spike is the derivative kick. It slams the motors to their limit for a millisecond although nothing physical happened; a number in software changed.

**Derivative of the measurement.** The values come from the Measurement column:

```
step 2:  D = −Kd × (measurement at step 2 − measurement at step 1) / dt
           = −0.01 × (0 − 0) / 0.001
           = 0

step 3:  D = −Kd × (measurement at step 3 − measurement at step 2) / dt
           = −0.01 × (0.5 − 0) / 0.001
           = −0.01 × 500
           = −5
```

There is no spike at step 2, because the measurement did not change. Step 3 gives the same −5 as before: a gentle brake as the drone starts to rotate.

Why this is allowed: the error can change for two reasons.

- **The measurement changed.** The drone actually moved. This is real motion, and it is what D exists to damp.
- **The setpoint changed.** Someone asked for something different. This is not motion, and braking against it makes no sense.

Taking the derivative of the measurement keeps the first and discards the second. The minus sign is there because the error and the measurement move in opposite directions: if the measurement rises by 1, the error falls by 1. Step 3 shows it: the measurement rose by 0.5 and the error fell by 0.5.

The response to the stick is not lost. It comes from the P term, which sees the full error of 100 and pushes hard, without a spike.

In this project the inner loop's setpoint comes from the outer loop, not directly from a stick. The outer loop runs at 250 Hz, so the inner loop's setpoint changes in small steps every fourth cycle. With the derivative on the error, each of those steps would give a small kick and a steady 250 Hz buzz in the motors. The derivative on the measurement avoids that too.

### The three together

| Term | Looks at | Fixes | Too much causes |
|---|---|---|---|
| P | the error now | most of the error, quickly | overshoot, fast oscillation |
| I | the error accumulated | steady-state error | slow oscillation, windup |
| D | the error's trend | overshoot | amplified noise, hot motors |

## The time step `dt`

The controller runs once per loop, every `dt` seconds (0.001 s for the 1 kHz inner loop). Both the integral and the derivative use `dt`:

- The integral adds `error × dt`, so it is in units of error-seconds.
- The derivative divides by `dt`, so it is in units of error per second.

Passing `dt` in, instead of assuming it, means the gains keep their meaning if the loop rate changes. The same `Kp`, `Ki` and `Kd` then work at 1 kHz on the firmware and at any rate in the host simulation.

## Output limits

The controller's arithmetic can produce any number. The motors cannot follow any number:

- A motor command has a floor (stopped, or idle) and a ceiling (full power).
- The rate loop only gets a share of that range, because throttle and the other two axes need room too.

So the output is clamped:

```
output = clamp(P + I + D, out_min, out_max)
```

The limit belongs inside the PID, not just at the motors, for two reasons:

- The PID needs to know when it is being limited. That is the trigger for anti-windup below.
- For the outer angle loop, the limit is a feature: clamping its output caps the rotation rate it may ask for (for example ±200 °/s).

When the output is at a limit, the controller is **saturated**. It is asking for more than it can have, and the loop is no longer correcting as intended.

## Integrator windup

Saturation and the integral term interact badly.

Take the drone held firmly in a hand, tilted, with the controller running:

1. There is an error, so the output goes to its limit. The motors can do no more.
2. The hand does not give way, so the error stays.
3. The integral keeps adding `error × dt`, every step, for as long as the hand holds. It grows far past any value that could ever be useful. This is **windup**.
4. The hand lets go. The drone rotates back and reaches the target.
5. The error is now zero, but the integral is still huge, so the output is still at its limit. The drone shoots past the target.
6. Only the opposite error can drain the integral, so the drone has to stay on the wrong side until it has. The result is a large overshoot, often several swings.

The same thing happens without a hand: on the ground before take-off, during a big manoeuvre, or with a motor at full power. Any time the output is saturated and an error remains, the integral winds up.

The integral's job is to hold the modest steady correction the drone needs. Windup is the integral storing a value that has nothing to do with that job.

## Anti-windup

Anti-windup means stopping the integral from growing when growing cannot help. Two simple methods, used together:

### 1. Clamp the I term

Give the I term its own limit, smaller than the output limit:

```
I term = clamp(I term, −i_limit, +i_limit)
```

The I term only ever needs to cancel steady disturbances, which are small. Capping it at, for example, 30% of the output range puts a hard bound on how much can be stored, whatever happens.

The limit is on the I term (`Ki × integral`), not on the raw integral, so it is in output units: `i_limit = 0.3` means the I term contributes at most 0.3, whatever `Ki` is. [Choices made in the code](#choices-made-in-the-code) shows how the code does this.

### 2. Conditional integration

Do not integrate when the output is saturated and the error would push it further into saturation:

```
if output is at out_max and error > 0:  do not integrate
if output is at out_min and error < 0:  do not integrate
otherwise:                              integral += error × dt
```

The sign check matters. When the output is stuck at the maximum and the error turns negative, integrating helps to leave saturation, so it must stay allowed.

There is a third method, *back-calculation*, which feeds the amount clipped by the output limit back into the integral. It is smoother but has an extra gain to tune. The two methods above are enough here.

### One more case: reset

Before take-off the drone sits on the ground with the motors off or idling, and any small sensor error would wind the integral up for as long as it sits there. The controller needs a `reset()` that zeroes the integral and the stored previous measurement. The flight code calls it while disarmed.

## Shape of the code

A sketch of the interface, to make the above concrete. It is `no_std`, uses `f32` and touches no hardware.

```rust
pub struct Pid {
    // Tuning
    pub kp: f32,
    pub ki: f32,
    pub kd: f32,
    pub out_min: f32,
    pub out_max: f32,
    pub i_limit: f32,

    // State
    i_term: f32,                      // Ki × integral, in output units
    prev_measurement: Option<f32>,    // None until the first update
}

impl Pid {
    pub fn update(&mut self, setpoint: f32, measurement: f32, dt: f32) -> f32 {
        let error = setpoint - measurement;
        let p = self.kp * error;

        // A bad dt is treated as "no time passed". Also catches NaN.
        if !(dt > 0.0) {
            debug_assert!(false, "Pid::update called with dt = {dt}");
            return (p + self.i_term).clamp(self.out_min, self.out_max);
        }

        // Derivative on the measurement; zero on the first call.
        let d = match self.prev_measurement {
            Some(prev) => -self.kd * (measurement - prev) / dt,
            None => 0.0,
        };
        self.prev_measurement = Some(measurement);

        // Would the output saturate with the I term as it stands?
        let unclamped = p + self.i_term + d;
        let pushing_further = (unclamped >= self.out_max && error > 0.0)
            || (unclamped <= self.out_min && error < 0.0);

        if !pushing_further {
            self.i_term += self.ki * error * dt;
            self.i_term = self.i_term.clamp(-self.i_limit, self.i_limit);
        }

        (p + self.i_term + d).clamp(self.out_min, self.out_max)
    }

    pub fn reset(&mut self) {
        self.i_term = 0.0;
        self.prev_measurement = None;
    }
}
```

### Choices made in the code

Three cases need a deliberate choice.

#### 1. `i_limit` and `Ki = 0`: store the I term, not the raw integral

The code accumulates the integral already multiplied by `Ki`, and clamps that:

```rust
self.i_term += self.ki * error * dt;
self.i_term = self.i_term.clamp(-self.i_limit, self.i_limit);
```

The alternative is to store the raw integral and limit it to `i_limit / Ki`, which divides by zero when `Ki` is 0. Storing the I term has no division, so there is nothing to guard:

- **`Ki = 0` just works.** The term adds zero each step and stays where it is.
- **The limit is directly in output units.** `i_limit = 0.3` means the I term contributes at most 0.3, whatever `Ki` is.
- **Retuning is smooth.** With a stored raw integral, changing `Ki` rescales everything stored so far and the output jumps. Here a new `Ki` only affects what is added from then on.

The one catch: setting `Ki` to 0 freezes the I term at its current value instead of removing it. When turning I off while tuning, call `reset()` too.

#### 2. First call: `prev_measurement` is an `Option`

`None` means "no history yet", and the D term is zero in that case. `reset()` sets it back to `None`, so the first step after arming follows the same rule.

The alternative is to start `prev_measurement` at 0.0. That is wrong whenever the first reading is not 0, and gives the one-step spike that taking the derivative of the measurement was meant to remove.

#### 3. Bad `dt`: treat the step as "no time passed"

A flight controller must not panic mid-flight, or return an error that nothing can act on. So when `dt` is not a positive number, the code returns early with P and the existing I term only:

```rust
if !(dt > 0.0) {
    debug_assert!(false, "Pid::update called with dt = {dt}");
    return (p + self.i_term).clamp(self.out_min, self.out_max);
}
```

- **No time passed means nothing to integrate and no rate to compute.** The I term is left alone, D is zero, and the stored previous measurement is not touched. P still responds to the current error.
- **`!(dt > 0.0)` instead of `dt <= 0.0`.** It also catches NaN, which fails every comparison.
- **The `debug_assert!` makes it loud where it should be.** A bad `dt` is a bug in the caller. In host tests and debug builds it stops at once; in the release firmware it degrades to the safe behaviour above.

#### Not handled here: a NaN measurement

A NaN measurement (from a failed sensor read, for example) would poison the I term permanently, because NaN plus anything is NaN. The PID does not guard against this. The driver or the flight code must reject a bad reading before it gets this far, which keeps the PID simple. That check belongs to the flight code in `drone/flight/` and the driver in `firmware/drone-rs/`.

## Unit tests

Each test checks one behaviour from this document, with concrete numbers so the expected value is unambiguous. Unless a test says otherwise, it uses wide limits (output ±100, `i_limit` 100) and zero for the gains not mentioned, so only one behaviour is under test.

### Each term on its own

| Test | Setup | Expect |
|---|---|---|
| P only | `Kp = 2`, setpoint 10, measurement 4 | output 12 |
| P sign | same, measurement 16 | output −12 |
| I accumulates | `Ki = 2`, error 5, `dt = 0.01`, 10 steps | output 1.0 (2 × 5 × 0.01 × 10) |
| D zero on first call | `Kd = 0.01`, measurement 7 | output 0 |
| D opposes motion | `Kd = 0.01`, `dt = 0.001`, setpoint 0, measurement 0 then 0.5 | second output −5 |
| No derivative kick | `Kd = 0.01`, measurement stays 0, setpoint 0 then 100 | both outputs 0 |

The last two are the worked example from [Derivative kick, worked through](#derivative-kick-worked-through), so the document and the tests agree.

### Limits and anti-windup

| Test | Setup | Expect |
|---|---|---|
| Output clamp | `Kp = 1`, limits ±10, error +1000, then −1000 | exactly 10, then exactly −10 |
| I term clamp | `Ki = 1`, `i_limit = 0.3`, limits ±1, error 1, `dt = 0.01`, 1000 steps | output 0.3 (would be 10 unclamped) |
| No windup while saturated | `Kp = 1`, `Ki = 1`, limits ±1, `i_limit = 0.5`; error 5 for 1000 steps, then error 0 | final output exactly 0 |
| Integrates out of saturation | `Ki = 1`, limits ±0.2, `i_limit = 0.5`, `dt = 0.01`; error 1 for 100 steps, then error −1 for one step | output 0.2 after the 100 steps, then below 0.2 |

How the last two work:

- **No windup while saturated.** During the 1000 steps the output is pinned at 1 and the error pushes further, so the I term must stay at 0. When the error goes to 0, P is 0 and the output shows the I term alone. Without conditional integration it would read 0.5.
- **Integrates out of saturation.** The I term climbs until the output reaches its limit of 0.2 and then stops. When the error turns negative, integrating helps to leave saturation, so the I term must shrink at once. A version that froze the I term whenever the output was saturated, without the sign check, would stay stuck at 0.2.

### The three choices

These cover [Choices made in the code](#choices-made-in-the-code).

| Test | Setup | Expect |
|---|---|---|
| `Ki = 0` | `Kp = 1`, `Ki = 0`, any error, many steps | output equals `Kp × error` every step, never NaN |
| Retune `Ki` | run PI for 100 steps, double `Ki`, run one more step with the same inputs | output changes by only one step's worth, not by a factor of 2 |
| Bad `dt` | `dt` of 0, −0.001 and NaN | panics in debug (`#[should_panic]`), one test per value |
| Reset | run 100 steps, `reset()`, then one step | same output as a brand-new `Pid` given that step |

### One closed-loop test

The tests above check the arithmetic: given these inputs, is the output the right number? None of them shows that the controller can actually steer something to a target. For that, the test needs a **plant**: control engineers' word for the thing being controlled. Here it is a few lines of code, inside the test, that pretend to be a drone.

The plant has one variable, `rate`. It is the pretend drone's current rotation rate in °/s: the number a gyro would report. It starts at 0 and is updated every step by:

```
rate += (output + disturbance − rate) × dt
```

Each part:

- **`output`** is what the PID just returned: the torque command. A positive output makes the rate rise.
- **`disturbance`** is a constant unwanted push, standing in for a weak motor or an off-centre battery. It is −1 in this test.
- **`− rate`** is drag. The faster the plant spins, the harder it is slowed. Without this term a constant push would speed the plant up for ever; with it, a constant push leads to a steady rate.
- **`× dt`** turns "change per second" into "change in this one step".

The test then runs the same loop the real drone will:

```rust
let mut rate = 0.0;                              // the plant's state
for _ in 0..20_000 {                             // 20 seconds at 1 kHz
    let output = pid.update(0.0, rate, dt);      // setpoint 0; rate is the measurement
    rate += (output + disturbance - rate) * dt;  // the plant responds
}
```

The PID reads `rate` as its measurement, its output changes `rate`, and the new `rate` is the next measurement. This is the closed loop from [`drone.md`](drone.md#how-the-loop-closes) at its smallest. The seesaw in the next Stage 1 item is a more realistic plant in the same position.

With setpoint 0, disturbance −1 and `dt = 0.001`, after 20,000 steps:

- **P only (`Kp = 2`):** the rate settles near −0.333, not 0.
- **PI (`Kp = 2`, `Ki = 5`):** the rate settles within 0.01 of 0.

Where −0.333 comes from: the rate stops changing when the bracket is zero, and with P only the output is `Kp × (0 − rate) = −2 × rate`:

```
output + disturbance − rate = 0
(−2 × rate) + (−1) − rate   = 0
−3 × rate                   = 1
rate                        = −0.333
```

P needs an error to produce any output, so it settles where its push, the disturbance and the drag balance. That is the steady-state error from [P — proportional](#p--proportional-react-to-the-error-now).

With PI, the I term keeps growing while any error remains. It stops at +1, which exactly cancels the disturbance of −1, and the rate is 0. This test shows the reason the I term exists.

### Practical notes

- **Compare floats with a tolerance:** `assert!((out - 1.0).abs() < 1e-5)`, not `assert_eq!`. The exceptions are the output clamp tests, where the result must be exactly the limit.
- **Private state:** the I term is private, so tests read it through the output by setting `Kp = 0` and `Kd = 0`, or through a small getter.
- **Constructor:** the tests are much shorter with a `Pid::new(kp, ki, kd)` that sets wide default limits, plus setters or public fields for the limits.
- **Order:** write the six single-term tests first, since they pin down the basic formula, then add the others with each feature.

## Where this goes next

- The seesaw model and the rate loop that uses this PID are the next two items of [Stage 1](drone.md#stage-1--host-simulation-one-axis).
- The D-term low-pass filter arrives with sensor noise, in the last item of Stage 1.
- The outer angle loop in Stage 2 reuses the same `Pid` with only `Kp` set, and its output limit caps the commanded rate.
