# Stage 3b — The throttle: hover, and making up for a tilt

This explains the second item of Stage 3 in [`drone.md`](drone.md#stage-3--host-simulation-hover):

> Fixed hover throttle, then throttle compensated for tilt

After [Stage 3a](Stage3-a.md) the drone has a weight, and the throttle decides whether it climbs or sinks. Until now the throttle was a number the simulation chose, 0.7, that did nothing but set how hard the motors worked. This item looks at what a fixed throttle can and cannot do, gives the flight code the one number it needs to know about the airframe, and adds the first thing the controller does to the throttle on its own: more of it in a tilt.

Neither part holds a height. Nothing in this item looks at the height. That needs a sensor, and is the option described in [`altitude.md`](altitude.md).

## What is in and what is not

| In this item | Not yet |
|---|---|
| the hover throttle, as a number the flight code knows | holding a height (needs a height sensor) |
| what happens when that number is a little wrong | finding the hover throttle in flight |
| more throttle in a tilt, worked out from the estimated angle | motor limits when the throttle runs out of room (the next item) |
| a limit on how much is added | arming and the tilt cut-off (the next item) |

## A fixed hover throttle

### The number

[Stage 3a](Stage3-a.md#what-the-three-add-up-to) worked it out for the simulated drone:

```
hover command = √(mass × g / (4 × thrust_coeff)) = √(11.77 / 24) = 0.7002
```

At that command the four motors together push up with exactly the drone's weight.

The flight code gets this as a setting, `hover_throttle`. It is a fact about the airframe, like the gains, and three later things lean on it:

| Used by | For |
|---|---|
| the gamepad, in the last item of Stage 3 | the centre of the throttle stick: the stick adds to it or takes from it |
| arming, in the next item | telling "on the ground" from "taking off": the I terms are held until the throttle is most of the way to hover |
| a height loop, if one is added | the starting point that the loop corrects |

### What a fixed throttle does

It balances the drone on a knife edge. The throttle sets a force, not a height and not a speed. At exactly the hover command the force is zero, and the drone keeps whatever vertical speed it already has, until drag wears it away.

A command that is a little off gives a small force that never stops:

| Command | Thrust − weight | Acceleration at first | Settles at |
|---|---|---|---|
| 0.69 | −0.34 N | 0.28 m/s² down | sinking at 0.85 m/s |
| 0.7002 | 0 | 0 | staying put |
| 0.71 | +0.33 N | 0.28 m/s² up | climbing at 0.83 m/s |

So a command 1.4% off puts the drone a metre away within three seconds, and it keeps going. Drag is the only thing that stops the speed growing without limit.

On the real drone the hover command is not even a constant:

- **The battery sags.** A full battery gives more thrust at the same command than a nearly empty one, so the hover command creeps up through a flight.
- **The mass changes** with whatever is carried.
- **Near the ground** the propellers give a little more thrust.

This is why a pilot's thumb never rests on the throttle. Holding a height with a fixed throttle is the pilot's job, done by eye, all the time.

### What the simulation does with it

The free runs of Stage 3a climb at a higher throttle and then go to `hover_throttle`. This item adds a way to make it wrong on purpose, to see the drift, and nothing else: the simulated drone's hover command is known exactly, so there is nothing to tune.

## Throttle in a tilt

### The problem

Thrust points out of the top of the drone. Tilted by an angle, only part of it still points up:

```
lift = thrust × cos(tilt)
```

At the hover command, [a tilted drone sinks](Stage3-a.md#what-the-three-add-up-to): at 0.15 m/s² when tilted 10°, and at 1.3 m/s² when tilted 30°. Every time the pilot moves the right stick, the drone drops.

### The fix

Ask for more thrust, by exactly as much as the tilt takes away:

```
thrust wanted = thrust asked for / cos(tilt)
```

Two details turn that into a formula for the command.

**The tilt, from roll and pitch.** The cosine of the tilt is how much of the body's z axis still points down. For a roll of φ and a pitch of θ it is

```
cos(tilt) = cos φ × cos θ
```

**Thrust goes with the square of the command.** So to get 1 / cos(tilt) times the thrust, the command needs the square root of that:

```
throttle out = throttle in / √(cos φ × cos θ)
```

| Tilt | Lift left without it | Command multiplied by | Hover command becomes |
|---|---|---|---|
| 0° | 100% | 1.000 | 0.700 |
| 10° | 98.5% | 1.008 | 0.706 |
| 20° | 94.0% | 1.032 | 0.722 |
| 30° | 86.6% | 1.075 | 0.752 |
| 45° | 70.7% | 1.189 | 0.833 |
| 60° | 50.0% | 1.414 | 0.990 |

The angles are the controller's **estimate**, since that is all it has.

### A limit

The table shows why this cannot go on for ever. At 60° the hover command is already at full throttle, with nothing left for the loops to steer with. **Stop adding at 45°.** Past that the throttle is multiplied by 1.189 and no more, and the drone is allowed to sink.

45° is beyond anything this controller is asked to hold. The sticks will reach about 30°.

### What it does not do

- **It does not hold a height.** It removes one cause of sinking. A drone that was climbing still climbs.
- **It is only as right as the estimate.** [Stage 3a](Stage3-a.md#what-that-does-to-the-estimator) found that the estimate reads low for the first seconds of a tilt, while the drone speeds up. The compensation is then worked out for a smaller angle than the true one, and is too small. With the longer τ chosen there, the shortfall is small. With τ = 1 s it is not: see [What to expect](#what-to-expect).
- **It assumes thrust goes with the square of the command.** That is true of the model. On the real drone it is roughly true, and the thrust stand will say how roughly. If thrust went in step with the command, the square root would not belong, and the multiplier at 30° would be 1.155 and not 1.075.

### A side effect: it flies faster

More thrust in a tilt means more push sideways as well as up. Without the compensation the sideways push is weight × sin θ. With it, the push is weight × tan θ:

| Tilt | Settles at, without | Settles at, with |
|---|---|---|
| 10° | 5.1 m/s | 5.2 m/s |
| 30° | 14.7 m/s | 17.0 m/s |

## Shape of the code

```
drone/
  flight/src/
    controller.rs   `hover_throttle` and the tilt compensation, between the targets and the mixer
  sim/
    src/bin/quad.rs a throttle that is wrong on purpose; compensation on and off; `--stage 3b`
    tests/free_flight.rs   more tests
    stage3-b.sh
```

### In the flight code

Three new settings, next to the gains:

```rust
pub struct Gains {
    // ... as before, and:

    // The throttle.
    pub hover_throttle: f32,         // 0.0 to 1.0; the command that holds the drone's weight
    pub tilt_compensation: bool,     // more throttle in a tilt
    pub tilt_compensation_limit: f32, // °; no more is added past this tilt. Start with 45
}
```

And one function, which `update` calls on the throttle just before the mixer:

```rust
/// `throttle` raised to make up for the lift lost at this attitude:
/// throttle / √(cos roll × cos pitch), with the tilt taken as no more
/// than `limit` degrees.
fn compensate_for_tilt(throttle: f32, attitude: Attitude, limit: f32) -> f32
```

`update` then reads, in its last two steps:

```
4. Run the three rate PIDs.
5. Raise the throttle for the tilt, if `tilt_compensation` is on.
6. Mix the throttle with the three outputs and return the result.
```

### Choices to make in the code

- **It lives in the controller, not in the simulation.** The real firmware needs it, and it uses the estimate, which only the controller has.
- **It uses the estimate of this step,** not the target angle. The target says where the drone should be; the lift depends on where it is.
- **`hover_throttle` goes in with the gains.** `Gains` then holds every number the controller is given about the airframe. If the name stops fitting, rename the struct; do not start a second one.
- **The simulation fills `hover_throttle` from the body,** with `Quad::hover_command()`. The real firmware will have a measured constant.
- **The cosine comes from `libm`,** as the estimator's `atan2` does. The flight crate is `no_std`.
- **A command above 1.0 is the mixer's business.** The compensation does not clamp its answer. The next item gives the mixer proper limits.
- **Earlier stages keep it off.** `--stage 3a` and before set `tilt_compensation` to false, so their runs do not change.

## Tests

### The compensation (`flight`)

| Test | Setup | Expected |
|---|---|---|
| Level changes nothing | roll 0°, pitch 0° | the throttle that went in |
| Roll | roll 30° | × 1.0746 |
| Pitch | pitch 30° | the same |
| The sign does not matter | roll −30° | the same |
| Both | roll 20° and pitch 20° | × 1.0642, which is 1 / √(cos 20° × cos 20°) |
| The limit | roll 60°, limit 45° | × 1.1892, as at 45° |
| Zero stays zero | throttle 0, any tilt | 0 |
| Off | `tilt_compensation` false, roll 30° | the motor commands of Stage 3a |
| On, in the controller | calibrated at 30° of roll, throttle 0.5, zero rates | the four commands average 0.537 |

### On the free body (`sim`)

Each starts hovering, well above the ground, and tilts for 4 s.

| Test | Setup | Expected |
|---|---|---|
| Hover is not changed | level, compensation on, 10 s | height within 1 cm, as without it |
| A wrong hover throttle drifts | command 0.71, level | climbing at about 0.8 m/s after 10 s |
| Without it, a tilt sinks | pitch −10°, compensation off | more than 0.7 m lost |
| With it, much less | the same, on | less than 0.3 m lost |
| A steep tilt | pitch −30°, on and off | less than 2.5 m lost with it; more than 6 m without |
| Both axes | roll 15° and pitch −15°, a tilt of 21° | less than 1 m lost |
| Motors stay in range | pitch −10°, −20° and −25°, on | no command reaches 1.0 |
| Motors reach the limit at 30° | pitch −30°, on | a command sits at 1.0 for a few milliseconds as the drone levels off; without the compensation the highest is 0.95 |
| It goes back | level again after the tilt | the commands average the hover command again |

## In the simulation

- **`--stage 3b`** is new, and becomes what the program does when no stage is given: Stage 3a with the compensation on.
- **`--no-tilt-compensation`** turns it off, for the comparison.
- **`--throttle-error`** adds a fraction to the hover throttle the run uses: `--throttle-error 0.01` flies 1% too high.
- **CSV columns:** `throttle`, what the run asked for, was there already; `throttle_mixed` is new, and is what went to the mixer.
- **`measure_flight.py`** gains the rate of climb at the end of the tilt, next to the change in height over it.
- **`plot_flight.py`** draws the throttle that went to the mixer as a dotted line over the motor commands.
- **`./stage3-b.sh`** runs the cases and saves the pictures in `drone/sim/stage3-b/`.

## What to expect

From a trial with the real controller and the Stage 2 body, with gravity, drag and movement added outside it and the compensation added to a copy of the controller, made before any of this item was written. [Results](#results) has the figures of the finished code. Mass 1.2 kg, drag 0.4 N per m/s, τ = 10 s. The drone starts hovering and tilts nose down for 4 s.

| Target pitch | Compensation | Height lost after 2 s | After 4 s | Sinking at, after 4 s | True pitch reached |
|---|---|---|---|---|---|
| 10° | off | 0.21 m | 0.87 m | 0.41 m/s | 11.9° |
| 10° | on | 0.03 m | 0.17 m | 0.10 m/s | 12.0° |
| 20° | off | 0.85 m | 3.44 m | 1.64 m/s | 23.9° |
| 20° | on | 0.11 m | 0.69 m | 0.43 m/s | 24.0° |
| 30° | off | 1.88 m | 7.63 m | 3.63 m/s | 35.8° |
| 30° | on | 0.27 m | 1.68 m | 1.05 m/s | 36.2° |

- **It removes about four fifths of the sinking,** at every angle.
- **What is left comes from the estimate.** The true pitch is 2° to 6° more than the target, as Stage 3a found, and the compensation is worked out for the estimate, which sits on the target. The drone is tilted further than the controller thinks, and gets too little.
- **A level hover is not changed at all.**
- **Going into the tilt, the largest motor command is the same with and without:** 0.78, 0.87 and 0.95 for the three angles. It comes in the first moment, from the rate loop, before the angle has changed.
- **Coming out of it, the compensation adds to that.** The trial stopped while the tilt was still held. The tests, written afterwards, level off again, and at that moment the raised throttle and the rate loop's push arrive together: 0.79, 0.89 and, at 30°, 0.752 + 0.252 = 1.004. One command is held at 1.0 for six milliseconds. It does no harm at that size, and it is the case the motor limits of [Stage 3c](Stage3-c.md#running-out-of-room) are for.

The same 30° tilt with τ = 1 s, the value before Stage 3a: the estimate falls far behind, the true pitch reaches 68°, and the drone loses 16 m in 4 s with the compensation on. The compensation does not rescue a bad estimate.

The picture to look for: the height panel of a tilt, with and without. Without, the line bends down as soon as the angle changes and keeps steepening. With, it stays nearly flat for the first second and then slopes gently.

## Results

The compensation, the arguments of the quad program and the tests above are written, and `Gains::default()` has it turned on, with a limit of 45°. `./stage3-b.sh` in `drone/sim/` saves the pictures in [`drone/sim/stage3-b/`](drone/sim/stage3-b/README.md).

Each run is the 12 s of Stage 3a, with the tilt from 5 s to 9 s. The drone is still climbing when the tilt starts, so the figure is the height gained over those four seconds. Level, it gains 2.82 m.

### In a tilt

| Target pitch | Compensation | Height gained in the tilt | Short of a level flight by | What happens |
|---|---|---|---|---|
| 10° | off | 1.95 m | 0.87 m | |
| 10° | on | 2.65 m | 0.17 m | |
| 20° | off | −0.64 m | 3.46 m | ends the tilt lower than it began |
| 20° | on | 2.10 m | 0.72 m | |
| 30° | off | −4.85 m | 7.67 m | reaches the ground at 9.4 s, at 3.1 m/s |
| 30° | on | 1.09 m | 1.73 m | flies on |
| 10°, with noise, delay and bias | on | 2.60 m | 0.22 m | |

The "short by" column is the height lost of the trial, and the figures agree with it: 0.17 m, 0.72 m and 1.73 m, against 0.17 m, 0.69 m and 1.68 m. The tests, which start from a hover at rest, give 0.17 m, 0.69 m and 1.68 m exactly.

- **Four fifths of the loss is gone,** at every angle.
- **At 30° it is the difference between flying and not.** Without it the drone is on the ground half a second after the tilt ends.
- **A tilt about roll gives the same figures as one about pitch.**

### The hover throttle

| Throttle after the climb | Height at 12 s | Climbing at |
|---|---|---|
| 0.6904, which is 1.4% too little | 4.6 m | −0.65 m/s |
| 0.7002, the hover command | 9.6 m | 0.12 m/s, and fading |
| 0.7100, which is 1.4% too much | 14.6 m | 0.91 m/s |

Ten metres apart after nine seconds, from a difference of 0.02 in the command. The drift is the 0.8 m/s worked out above, on top of what is left of the climb.

### What it cannot do

- **It does not rescue a bad estimate.** A 30° tilt with τ = 1 s: the estimate falls far behind, the true pitch reaches 70°, and the drone reaches the ground at 8.1 s at 7.2 m/s, with the compensation on.
- **At 30° a motor reaches its limit.** As the drone levels off, the raised throttle and the rate loop's push add up to 0.752 + 0.252 = 1.004, and one command is held at 1.0 for six milliseconds. Up to 25° the highest is 0.95.

### What changed from the plan

- **The new CSV column is `throttle_mixed`,** and `throttle` keeps its meaning of what was asked for. Stage 3a already wrote a `throttle` column, and changing what it holds would have changed the output of `--stage 3a`.
- **`Controller::throttle()`** is new: the throttle that last went to the mixer, for the log.
- **The tests start from a hover at rest,** in their own file, `drone/sim/tests/throttle.rs`. The plan put them in `free_flight.rs`, whose flights are still climbing when they tilt.
- **Four earlier tests now turn the compensation off.** Three in the controller check motor commands by their arithmetic at a tilt, and one in `free_flight.rs` checks that a tilt costs lift. All four are about the controller before this item.
- **Every `--stage` before `3b` has the compensation off.** Eleven runs from `2a` to `3a` were compared with the program as it was: the same, column for column.

## Where this goes next

- The next item, [Stage 3c](Stage3-c.md), adds arming, the tilt cut-off, and motor limits that keep the loops working when the throttle is high.
- The gamepad item makes `hover_throttle` the centre of the throttle stick.
- Holding a height is an option, in [`altitude.md`](altitude.md). Its loop would add to `hover_throttle`, and the tilt compensation would act on the sum.
- The real airframe in [`quad10.md`](quad10.md) hovers at a command of about 0.45, so it has far more room above the hover than the simulated drone.
