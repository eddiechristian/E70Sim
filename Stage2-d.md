# Stage 2d — Disturbance tests

This explains the fourth and last item of Stage 2 in [`drone.md`](drone.md#stage-2--host-simulation-self-levelling-attitude):

> Disturbance tests: recovers to level after a kick, from a 30° start, and with one weak motor

After [Stage 2c](Stage2-c.md) the drone levels itself and follows an angle target. Every run so far has been kind to it: nothing pushed it, and all four motors were the same. This item is unkind on purpose. It pushes the drone, starts it far from level and weakens a motor, and checks that it still comes back to level.

Two of the three tests are expected to pass with the controller as it is. The third, the weak motor, is expected to fail, and fixing it is the real work of this item: it is where the I term of the rate loops, which has been 0 since Stage 1, gets a value.

## What is in and what is not

| In this item | Not yet |
|---|---|
| a push on the body in the physics: a torque for a set time, about one axis or several | wind that pushes the drone sideways (Stage 3, with movement) |
| the three tests: a kick, a 30° start, one weak motor | a motor that stops altogether |
| choosing `Ki` for the roll and pitch rate loops, and for yaw | gravity and hover (Stage 3) |
| new arguments in the quad program, a script and pictures | arming and the safety cut-off (Stage 3) |

## Three kinds of trouble

The three tests are three different shapes of disturbance. Each one asks a different question of the controller.

| Test | Shape | What it asks |
|---|---|---|
| A kick | a short push, then nothing, about one axis or about two at once | how far is the drone knocked, and how fast is it back? |
| A 30° start | a large error from the first moment | does a big correction stay under control? |
| One weak motor | a push that never stops | does the drone end up level, or only near it? |

## The kick

### What it is

A kick is a torque on the body for a short time: a gust, a bump, a hand. Its size is the torque times how long it lasts. That product is what changes the rotation rate:

```
change in rate = torque × time / inertia
```

For the standard kick of this item, 0.3 N·m about roll for 0.05 s, on a body with `Ixx` = 0.015 kg·m²:

```
change in rate = 0.3 × 0.05 / 0.015 = 1 rad/s = 57 °/s
```

So with no controller this kick would leave the drone rolling at 57 °/s for ever, since the model has no drag.

To compare it with the motors: at a throttle of 0.7 the largest roll torque the mixer can ask for, with the PID output at its limit of 0.3, is about 1.4 N·m. The standard kick is a fifth of that.

### What the controller does with it

1. The kick starts the body turning. The gyro sees it within a step.
2. The **rate loop** wants a rate of zero. It pushes back at once, with its P and D terms. This is what limits how fast the body turns, and most of the fight happens here.
3. While the kick lasted, the body moved a few degrees. The **angle loop** sees that angle and asks the rate loop for a rate back towards level.
4. The body returns in the same way as after a small step in Stage 2c.

The kick is over in 50 ms, so nothing is left to hold against. The controller of Stage 2c needs no change for this test.

### A compound kick: roll and pitch together

A real gust rarely lines up with one axis. A compound kick is a torque about roll and a torque about pitch at the same moment: 0.3 N·m on each.

Two torques at right angles add up to one torque about an axis between them. Equal parts of roll and pitch make a single push about a **diagonal** of the drone, of size

```
√(0.3² + 0.3²) = 0.42 N·m
```

Positive roll with positive pitch is a push about the diagonal that runs through the front-right and back-left motors. That changes which motors can fight it:

| Kick | Axis it turns the body about | Motors that push back |
|---|---|---|
| roll only | front to back | all four: the two on the left against the two on the right |
| pitch only | side to side | all four: front against back |
| roll and pitch, equal | the diagonal through front-right and back-left | only two: front-left and back-right |

The front-right and back-left motors sit on the axis itself. They have no lever about it, and the mixer shows it: for each of them the roll part and the pitch part of the command cancel, so their commands do not move. The whole job falls to the other two, front-left and back-right, which get the roll part and the pitch part added together.

Two things follow:

- **Each axis behaves as if it were kicked alone.** The roll loop and the pitch loop do not know about each other, and at these angles the body's roll and pitch barely interact. Roll and pitch each go where the single-axis kick sent them.
- **The motors run out sooner.** Two motors do the work of four, so they reach their limits at half the kick per axis. A compound kick is the harder test of the motor range, and the reason it is worth running.

There is a small side effect on yaw. Front-left and back-right both turn clockwise. One speeds up and the other slows down by the same command, but thrust goes with the square of the speed, so the two changes in reaction torque do not quite cancel. The drone gets a small nudge in yaw, which the yaw rate loop removes.

## The 30° start

The body starts at 30° of roll and sits there through the calibration, with the motors off. The calibration finds the angle. When the motors start, the target is level, so the angle loop sees an error of 30° at once.

Stage 2c already ran this at 20°. Two things make 30° worth its own test:

- **The rate asked for is larger.** 6 × 30 = 180 °/s, just under the limit of 200 °/s. Beyond 33° the limit takes over.
- **The motors work harder.** The commands leave the middle of their range, and a command that reaches 0 or 1 cannot give what the mixer asked for.

The test is also run with 30° of roll and 30° of pitch together, which makes all four motor commands differ.

A warning about what this test does not show. The simulation has no gravity yet, so a drone at 30° does not fall or slide while it rights itself. In the trial below the drone recovers even from 170°, nearly upside down. A real drone would have hit the ground. Stage 3 adds gravity, and a cut-off that stops the motors when the tilt is past a safe angle.

## The weak motor

### What it does to the body

Make the front-right motor 10% weak: a `thrust_coeff` of 5.4 N against 6.0 N for the other three. With all four at a command of 0.7:

```
thrust of a good motor = 6.0 × 0.7² = 2.94 N
thrust of the weak one = 5.4 × 0.7² = 2.65 N
missing                = 0.29 N
```

One missing push at one corner turns the body about all three axes at once:

| Axis | Why | Torque | Direction |
|---|---|---|---|
| Roll | the right side has less lift | 0.141 × 0.29 = 0.042 N·m | positive: right side down |
| Pitch | the front has less lift | 0.042 N·m | negative: nose down |
| Yaw | front-right turns counter-clockwise; less of its reaction torque | 0.015 × 0.29 = 0.0044 N·m | negative: nose left |

The 0.141 is `d`, the motor's distance from the axis, and the 0.015 is the yaw coefficient `c`, both from [`Stage2-a.md`](Stage2-a.md).

Unlike the kick, this never stops. To hold level the controller must give the weak motor a larger command than the others, for as long as it flies.

### Why the Stage 2c controller cannot hold level

Follow the roll chain. To cancel 0.042 N·m the rate loop must keep its output at a steady value, which works out at about 0.0088 in throttle units. But the rate loop is P and D only:

```
output = Kp × (target rate − rate)
```

Hovering still, the rate is 0. So a steady output needs a steady target rate:

```
target rate = 0.0088 / 0.0014 = 6.3 °/s
```

The target rate comes from the angle loop, which is P only as well:

```
target rate = 6 × (target angle − angle)
angle error = 6.3 / 6 = 1.04°
```

So the drone hangs about 1° from level, for ever. The error is what pays for the output. Take the error away and the output goes with it, and the weak corner drops again.

This is the steady-state error of [`Stage1-a.md`](Stage1-a.md#p--proportional-react-to-the-error-now), and the same experiment as the weak motor on the seesaw in [`Stage1-c.md`](Stage1-c.md#an-experiment-one-weak-motor). In the trial the error is 1.11° on roll and −1.11° on pitch.

Yaw is worse off. It has no angle loop, so its steady error is in the **rate**:

```
rate error = 0.0088 / 0.01 = 0.9 °/s
```

The drone turns to the left at about 0.9 °/s and never stops. Stage 2c found the same slow yaw from an uncalibrated gyro; here the cause is in the airframe, and no calibration removes it.

### The I term

The fix is the I term of the rate PIDs. It adds up the rate error over time:

```
I term += Ki × (target rate − rate) × dt
output  = P term + I term + D term
```

While any error is left, the I term keeps growing. It stops growing only when the error is zero, and by then it holds exactly the 0.0088 the weak motor needs. The output is now paid for by the I term's memory and not by an error, so the drone can be level and still hold the weak corner up.

The PID has had this term since Stage 1, with its limits and anti-windup ([`Stage1-a.md`](Stage1-a.md)). Only the gain has been 0.

**Why in the rate loops, and not the angle loops?** The disturbance is a torque, and the rate loop's output is a torque. Putting the memory there cancels the disturbance where it enters. It also covers yaw, which has no angle loop. This is where flight controllers keep it.

### What it costs

The I term cannot tell a weak motor from an ordinary move. During a 20° step the body lags behind the rate asked for, that lag is a rate error, and the I term collects it. When the body arrives, the I term holds a value nobody needs. It has to give it back, and until it has, it pushes the body off its target.

- **A small `Ki`** collects little and gives it back slowly: a small error that fades over a second or so.
- **A large `Ki`** collects a lot. In the trial, `Ki` = 0.01 brings the body to its target and then pulls it 2° back off it before it settles. `Ki` = 0.03 swings back and forth for more than a second.

So `Ki` is chosen between two cases: large enough to deal with the weak motor in about a second, small enough to leave a step alone. [What to expect](#what-to-expect) has the figures.

### Yaw's I term

Yaw gets its own `Ki`. It removes the steady turning. It does not undo what happened before it took hold: the drone ends up pointing a little to the left of where it started and stays there. Nothing in this controller knows which way the nose points, so nothing brings it back. That needs a compass or another reference, as [`Stage2-c.md`](Stage2-c.md#results) noted.

### Three places the I term can go wrong

- **The limit.** `i_limit` is 0.1. The 10% weak motor needs 0.0088, so there is room for a much weaker one. If the limit is ever reached, the drone goes back to hanging off level.
- **Motors at their limit.** If the output is held at its limit, the PID stops adding to the I term in that direction. That anti-windup is already in the PID.
- **On the ground.** A drone standing on the ground cannot turn, so an I term would grow without doing anything, and then throw the drone over at take-off. Here the PIDs do not run until the calibration is done, and the simulation has no ground. Stage 3 must hold the I terms at zero until the drone is flying.

## Shape of the code

Nothing changes in how the flight crate works. The changes are in the physics, which gains a push, and in the simulation, which gains arguments to use it.

```
drone/
  physics/src/
    quad.rs       a `push` field: a torque on each axis, added to the motors' torques
  flight/src/
    controller.rs `Gains::default()` gets the chosen `rate_ki` and `yaw_ki`
  sim/
    src/bin/quad.rs       arguments for a kick and a weak motor
    tests/disturbances.rs the tests below
    measure_disturbance.py
    stage2-d.sh
```

### In the physics

```rust
/// A torque on each axis from outside the drone, in N·m.
pub struct Torques {
    pub roll: f32,
    pub pitch: f32,
    pub yaw: f32,
}

pub struct Quad {
    // ...
    /// Added to the motors' torques on every step, until it is changed.
    /// Zero unless a test or the simulation sets it.
    pub push: Torques,
}
```

In `step`, each of the three torques gets its part of `push` added before the rates are updated. The caller sets the push at the start of a kick and sets it back to zero at the end.

A weak motor needs nothing new: each of the four motors already has its own `thrust_coeff`.

### Choices to make in the code

- **A push is a torque, not a force.** The model only rotates, so a force has nothing to act on. Wind that moves the drone sideways comes with Stage 3.
- **The caller times the kick.** The body knows nothing about time; it applies whatever `push` holds. That keeps `Quad` simple and lets a test hold a steady push as easily as a short one.
- **The weak motor is weak from the start.** It is a property of the airframe, not an event. The drone takes off with it, so the I term starts collecting the moment the controller starts flying.
- **`reset` clears the push.** It is part of the body's state for a run.

## Tests

### The push (`physics`)

| Test | Setup | Expected |
|---|---|---|
| No push, no change | push zero, equal commands | rates stay 0, as before |
| Roll push | 0.3 N·m about roll for 0.05 s, equal commands | roll rate 1 rad/s; pitch and yaw 0 |
| Pitch and yaw push | the same about each | that axis alone, at torque × time / inertia |
| Push stops | the push set back to zero | the rate stays where it was |
| Reset | a push set, then `reset()` | the push is zero |

### The disturbances on the whole loop (`sim`)

Each runs the loop of `tests/controller_and_body.rs`: the calibration, then flying with level targets.

| Test | Setup | Expected |
|---|---|---|
| Kick, roll | 0.3 N·m for 0.05 s | peak under 3°; within 1° of level in under 0.3 s; ends within 0.1° |
| Kick, pitch | the same about pitch | the same |
| Compound kick | 0.3 N·m about roll and 0.3 N·m about pitch, together | each axis as if kicked alone: peak under 3°, within 1° in under 0.3 s; the front-right and back-left commands stay within 0.01 of the throttle |
| Compound kick, other diagonal | 0.3 N·m about roll and −0.3 N·m about pitch | the same, with front-left and back-right the ones that stay still |
| Hard compound kick | 1.2 N·m about each, enough to hold two motors at their limits | recovers; both axes end within 0.1° |
| Kick, yaw | 0.1 N·m for 0.05 s about yaw | the yaw rate returns to within 1 °/s; roll and pitch stay within 0.1° |
| Hard kick | 2.4 N·m for 0.05 s, enough to hold the motors at their limit | recovers; ends within 0.1° |
| 30° start | roll 30° | within 1° of level in under 0.5 s; never more than 1° past level |
| 30° on both | roll 30° and pitch 30° | both within 1° in under 0.5 s |
| Weak motor, no I | front-right 10% weak, `Ki` = 0 | ends about 1.1° off on roll and on pitch: the test that shows the problem |
| Weak motor | the same, with the chosen `Ki` | roll and pitch within 0.2° of level 1.5 s after the motors start |
| Weak motor, yaw | the same | the yaw rate ends within 0.1 °/s of 0 |
| Any motor | each of the four weak in turn | the same result, with the signs to match |
| The step survives | no weak motor, the chosen `Ki`, a 20° step | the Stage 2c limits still hold: 90% within 0.4 s, overshoot under 2% |

The last test is the guard on the other side: a `Ki` that fixes the weak motor by spoiling the step would fail it.

## In the simulation

The quad program gains:

- **`--kick-roll`, `--kick-pitch` and `--kick-yaw`**, each a torque in N·m. Any of the three can be given together, which is how a compound kick is made. `--kick-at` (seconds, 2.0 unless given) and `--kick-for` (seconds, 0.05 unless given) say when and for how long.
- **`--weak-motor`** (`fl`, `fr`, `br` or `bl`) and **`--weak-by`**, a percentage.
- **`--start-roll` and `--start-pitch`** are there already.

Disturbance runs use `--target 0`, so that the only thing happening is the disturbance. `measure_disturbance.py` prints the largest angle reached, the time until the angle stays within 1° of level, the angle at the end and the yaw rate at the end. `./stage2-d.sh` runs the cases and saves the pictures in `drone/sim/stage2-d/`, as the earlier stages do.

## What to expect

From a trial with the real controller and body, and a push added to a copy of the physics for the trial. Nothing is in the repo yet.

### The kick

A kick about roll for 0.05 s at 2.0 s, with `Ki` = 0:

| Kick | Largest roll angle | Within 1° of level after | Motor commands |
|---|---|---|---|
| 0.3 N·m | 2.1° | 0.22 s | 0.62 to 0.78 |
| 0.6 N·m | 4.2° | 0.28 s | 0.54 to 0.86 |
| 1.2 N·m | 8.4° | 0.32 s | 0.40 to 1.00 |
| 2.4 N·m | 24.3° | 0.43 s | 0.40 to 1.00 |

- **Up to 1.2 N·m the angle grows in step with the kick.** Twice the kick, twice the angle.
- **At 2.4 N·m it does not.** The motors are at their limit and cannot push back any harder, so the angle is three times what the pattern says. The drone still recovers.
- **Noise makes no difference.** With gyro noise, delay, accelerometer noise and bias, the 0.3 N·m kick gives 2.1° and 0.21 s.
- **A kick about pitch gives the same figures.** A 0.1 N·m kick about yaw gives a yaw rate of 10 °/s for a moment and leaves the nose 1° from where it was.

The largest angle comes about 0.1 s after the kick starts, well after the kick is over: the body is still turning while the rate loop brings it to a stop.

### The compound kick

A kick about roll and about pitch together, for 0.05 s, with `Ki` = 0:

| Kick about each axis | Largest roll angle | Largest pitch angle | Within 1° of level after | Motor commands |
|---|---|---|---|---|
| 0.3 N·m | 2.1° | 2.1° | 0.22 s | 0.54 to 0.86 |
| 0.3 N·m roll, −0.3 N·m pitch | 2.1° | −2.1° | 0.22 s | 0.54 to 0.86 |
| 0.3 N·m roll, 0.6 N·m pitch | 2.1° | 4.2° | 0.22 s and 0.28 s | 0.47 to 0.93 |
| 0.6 N·m | 4.2° | 4.2° | 0.28 s | 0.39 to 1.00 |
| 1.2 N·m | 10.7° | 10.6° | 0.37 s | 0.10 to 1.00 |

- **Each axis gives the single-axis figures.** 0.3 N·m gives 2.1° on roll and 2.1° on pitch, as each did alone. With unequal kicks each axis follows its own.
- **Two motors do all of it.** In the first row the front-left command runs from 0.54 to 0.71 and the back-right from 0.69 to 0.86, while front-right and back-left stay at 0.70 throughout.
- **The limit comes at half the kick.** At 0.6 N·m about each axis a motor already reaches 1.0; a kick about roll alone needs 1.2 N·m for that. At 1.2 N·m about each, the angle is 10.7° against 8.4° for the same kick about roll alone, and one motor falls to 0.10.
- **Yaw is nudged a little.** The yaw rate reaches 0.2 °/s for the 0.3 N·m kick and 0.6 °/s at 0.6 N·m, then returns to zero. The heading ends within 0.1° of where it was.
- **Noise makes no difference here either:** 2.1° on both axes and 0.21 s.

### The 30° start

| Start | Within 1° of level after | Motor commands |
|---|---|---|
| roll 30° | 0.36 s | 0.45 to 0.95 |
| pitch −30° | 0.36 s | 0.45 to 0.95 |
| roll 30°, with noise, delay and bias | 0.36 s | 0.40 to 1.00 |
| roll 30° and pitch 30° | 0.34 s and 0.38 s | 0.20 to 1.00 |
| roll 60° | 0.50 s | 0.42 to 0.98 |
| roll 120° | 0.80 s | 0.42 to 0.98 |
| roll 170° | 1.05 s | 0.42 to 0.98 |

- **From 60° up the rate limit is in charge.** The drone turns at 200 °/s for most of the way, so the time grows with the angle.
- **With both axes tilted the yaw angle ends 7.7° from where it began,** though the body's yaw rate never passes 1 °/s. Roll, pitch and yaw as angles are not turns about the body's own axes, as [`Stage2-c.md`](Stage2-c.md#a-limit-the-angles-are-not-the-bodys-axes) explains: undoing a roll and a pitch together leaves a small change of heading.
- **The last three rows are not real,** for the reason given above: there is no gravity.

### The weak motor

Front-right 10% weak, targets level. The roll angle, with time counted from the moment the motors start:

| `Ki` | After 0.5 s | After 1 s | After 2 s | After 4 s |
|---|---|---|---|---|
| 0 | 1.12° | 1.11° | 1.11° | 1.11° |
| 0.001 | 0.89° | 0.62° | 0.30° | 0.07° |
| **0.003** | **0.54°** | **0.19°** | **0.03°** | **0.00°** |
| 0.01 | 0.04° | 0.01° | 0.00° | 0.00° |
| 0.03 | 0.18° | 0.03° | 0.04° | 0.02° |

Pitch is the same with the sign turned round. With `Ki` = 0 the error matches the 1.04° worked out above. A motor 5% weak gives 0.54° and one 20% weak gives 2.4°: the error grows in step with the weakness.

The same `Ki` on a 20° roll step with four good motors:

| `Ki` | Rise to 90% | Overshoot | Settled within 5% | Off target 0.5 s after the step | What happens |
|---|---|---|---|---|---|
| 0 | 0.284 s | 0.1% | 0.326 s | 0.01° | the Stage 2c result |
| 0.001 | 0.270 s | 0.3% | 0.305 s | 0.04° | no real change |
| **0.003** | **0.250 s** | **0.5%** | **0.277 s** | **0.30°** | a small error that fades over a second |
| 0.01 | 0.212 s | 0.3% | 0.582 s | 2.07° | arrives, then falls 2° back |
| 0.03 | 0.201 s | 8.1% | 1.379 s | 1.28° | swings for more than a second |

**Start with `Ki` = 0.003 for roll and pitch.** It clears the weak motor to 0.2° in a second. On a step it leaves 0.3° half a second after arriving, which fades. `Ki` = 0.01 is three times quicker on the weak motor but falls back 2° after every move, and that is the worse fault.

`Ki` = 0.003 does not make the kick worse in size: 2.0° against 2.1°. The return to level has the same slow tail as the step.

### Yaw

Front-right 10% weak, roll and pitch `Ki` = 0.003:

| Yaw `Ki` | Yaw rate after 0.5 s | After 1 s | At the end | Heading at the end | Overshoot of a 50 °/s yaw step |
|---|---|---|---|---|---|
| 0 | −0.93 °/s | −0.93 °/s | −0.93 °/s | −3.6° and still turning | 2.6% |
| **0.02** | **−0.45 °/s** | **−0.13 °/s** | **0.00 °/s** | **−0.5°** | **8.4%** |
| 0.05 | −0.01 °/s | 0.00 °/s | 0.00 °/s | −0.2° | 15.7% |
| 0.1 | 0.20 °/s | 0.04 °/s | 0.00 °/s | −0.1° | 19.2% |

**Start with a yaw `Ki` of 0.02.** It stops the turning within about a second, at the price of a yaw step that overshoots 8%. The larger values stop it sooner and overshoot more. The yaw step holds two motors at their limit, as Stage 2a found, which is part of why the overshoot grows so quickly; the yaw gains as a whole are still the first guess, and are worth another look in this item.

The picture to look for in the weak-motor run: with `Ki` = 0, three flat lines that sit beside zero, on roll, pitch and yaw rate. With the chosen gains, the same three lines bending back onto zero within a second or two, and the front-right motor's command settling a little above the other three.

## Where this goes next

- This is the last item of Stage 2. With it done, the drone holds level against a push, a bad start and an uneven airframe, in a simulation that only rotates.
- Stage 3 adds gravity and movement. A tilt then makes the drone slide, the throttle decides whether it climbs, and the large-angle recoveries of this item stop being free.
- Stage 3 also has to keep the I terms at zero until the drone has left the ground, and add the cut-off for a tilt past a safe angle.
- The real airframe in [`quad10.md`](quad10.md) has about 2.3 times the control authority of the simulated quad, so every gain chosen in Stage 2, `Ki` included, will need scaling or retuning for it.
