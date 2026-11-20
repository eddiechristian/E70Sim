# Stage 3a — Gravity, movement and the ground

This explains the first item of Stage 3 in [`drone.md`](drone.md#stage-3--host-simulation-hover):

> Full 6DOF: gravity, translation, ground

Through Stage 2 the drone was held by a ball joint at its centre. It could turn any way, but it could not go anywhere. This item takes the joint away. The drone now has a mass and a position, gravity pulls it down, its own thrust holds it up, and there is a ground to stand on and to hit.

"6DOF" means six degrees of freedom: three of rotation, which Stage 2 has, and three of movement, which are new.

Nothing in the flight code changes in this item. The controller of [Stage 2d](Stage2-d.md) flies the new body as it is. What changes is what its sensors tell it, and that is the part to watch: the accelerometer no longer reads what the estimator of [Stage 2b](Stage2-b.md) was built on.

## What is in and what is not

| In this item | Not yet |
|---|---|
| a mass, a position and a velocity | holding a height: the throttle is still set by hand (the next item) |
| gravity, and thrust that points where the drone points | making up for the lift lost in a tilt (the next item) |
| air drag, as one simple force | arming, the tilt cut-off and holding the I terms on the ground (the item after) |
| a flat ground: standing on it, leaving it, landing on it | wind, bouncing, tipping over on the ground |
| the accelerometer reading of a flying drone | a height sensor ([`altitude.md`](altitude.md)) |
| a new choice of the estimator's time constant | flying by hand (the last item) |

## The world frame

The body frame of [Stage 2a](Stage2-a.md#the-body-frame) is fixed to the drone and turns with it. A position needs axes that stay still. These are fixed to the ground:

| Axis | Points | Called |
|---|---|---|
| x | north | north |
| y | east | east |
| z | down, into the ground | down |

This is north-east-down, the frame aircraft use. It matches the body frame: a level drone with its nose to the north has its three body axes along the three world axes.

Two things follow from z pointing down:

- **Height is −z.** The ground is at z = 0, and a drone 5 m up is at z = −5. The code keeps z, and gives the log a `height` that is `−z`.
- **Gravity is positive.** It pulls along +z.

The drone's **position** is where its centre is, in metres. Its **velocity** is how fast that changes, in m/s, also along the world axes.

"North" is only a name here. Nothing in the simulation knows where north is; it is the way the nose points at the start.

## The forces

Three forces act on a flying drone.

### Gravity

```
gravity = mass × g, downwards
```

The mass is new. **Start with 1.2 kg.** That is the mass the throttle of 0.7 was chosen for in [Stage 1c](Stage1-c.md): four motors at 0.7 give 4 × 6 × 0.7² = 11.76 N, and 1.2 kg weighs 1.2 × 9.807 = 11.77 N.

### Thrust

Each motor pushes along the body's own z axis, upwards out of the top of the drone. The four add up to one force:

```
thrust = T_FL + T_FR + T_BR + T_BL, along the body's −z axis
```

The body's z axis is not the world's. When the drone tilts, the thrust tilts with it. The quaternion says where the body's z axis points in the world. With w, x, y and z the four numbers of the attitude:

```
body z axis, in the world = ( 2 × (x×z + w×y),
                              2 × (y×z − w×x),
                              1 − 2 × (x² + y²) )
```

The thrust force is the thrust times that, with a minus sign, since thrust pushes along −z.

Two checks of the signs:

| Attitude | Body z axis in the world | Thrust pushes the drone |
|---|---|---|
| level | (0, 0, 1) | straight up |
| nose up by θ | (sin θ, 0, cos θ) | up and backwards |
| right side down by φ | (0, −sin φ, cos φ) | up and to the right |

A drone moves towards its low side. Nose down, it flies forward.

### Drag

Air resists movement. The model uses the simplest form:

```
drag = −k × velocity
```

It points against the movement and grows in step with the speed. **Start with `k` = 0.4 N per m/s.** That is a guess. On the real drone it comes from a flight log: tilt, let the speed settle, and read it off.

Real drag is more complicated. Most of it at these speeds comes from the propellers, it is larger across them than along their axis, and the body adds a part that goes with the square of the speed. One number is enough for a hover, and it is enough to show the effect that matters most here, on the accelerometer.

Without drag the model would have nothing to stop a moving drone, and a tilted one would speed up for ever.

### What the three add up to

**Hovering.** Level and still, thrust equals weight and there is no drag. The command that does it:

```
hover command = √(mass × g / (4 × thrust_coeff)) = √(11.77 / 24) = 0.7002
```

A throttle above that climbs, and one below it sinks. Nothing brings the drone back to a height: there is no "level" for height as there is for roll.

**Climbing.** At a throttle of 0.76 the thrust is 13.86 N, which is 2.09 N more than the weight. The drone starts up at 2.09 / 1.2 = 1.7 m/s². Drag grows as it speeds up, and the climb settles at 2.09 / 0.4 = 5.2 m/s.

**Tilted.** At an angle θ, with the throttle at hover:

| | Formula | At 10° | At 30° |
|---|---|---|---|
| sideways push | weight × sin θ | 2.0 N | 5.9 N |
| sideways acceleration at first | g × sin θ | 1.7 m/s² | 4.9 m/s² |
| lift left | weight × cos θ | 98.5% | 86.6% |
| sinks at first at | g × (1 − cos θ) | 0.15 m/s² | 1.3 m/s² |
| speed it settles at | push / k | 5.1 m/s | 14.7 m/s |

So a tilt does two things. It sends the drone sideways, which is how a quadcopter moves at all. And it costs lift, so a tilted drone sinks unless the throttle goes up. Making up for that is the next item.

**How long it takes.** Speed closes on its final value with a time constant of

```
mass / k = 1.2 / 0.4 = 3 s
```

After 3 s a drone is at 63% of its final speed. The same 3 s is how long a moving drone takes to coast to 37% of its speed once it is level again. A drone flying at 5 m/s travels about 15 m more before it stops.

## From forces to movement

Each step, after the rotation:

```
force        = thrust force + gravity + drag
acceleration = force / mass
velocity    += acceleration × dt
position    += velocity × dt
```

This is `acceleration = torque / inertia` from [Stage 1b](Stage1-b.md), with force for torque and mass for inertia, done three times over for the three world axes.

Rotation does not depend on movement. The torques, the rates and the quaternion are worked out exactly as before. Movement only reads the attitude, to know where the thrust points.

## The ground

The ground is flat, at z = 0. The model of it is deliberately simple.

**Standing on it.** A drone on the ground does not move and does not turn: its velocity and its three rates are zero, and it keeps the attitude it has. A drone set down at 20° of roll stays at 20°, as if on a slope. The motors still run and still follow their commands.

**Leaving it.** The drone leaves when the upward part of its thrust is more than its weight. From then on it is flying.

**Landing on it.** When a flying drone reaches z = 0 it stops there, at once. The model records how it arrived: the speed downwards, the speed sideways and the tilt. A test or the simulation can then say whether that was a landing or a crash.

What this leaves out:

- **Bouncing.** The drone sticks where it lands.
- **Tipping over.** A drone that lands tilted stays tilted, and one that arrives sideways does not roll.
- **A drone light on its feet.** With the thrust just under the weight, a real drone skates about. This one stands still.
- **Ground effect.** Propellers near the ground give a little more thrust.

**The calibration now has a real place to happen.** Until now the drone hung on its joint through the calibration, motors off. Now it stands on the ground through it. That is what the real drone will do.

### Standing on the ground with the loops running

A warning for the item after next. After the calibration the controller's state is `Flying`, even when the throttle is too low to lift off. Its loops run, and on the ground they can push without anything moving. That is harmless for the P and D terms and not for the I terms.

Take a drone standing on a slope of 10°, right side low, with a level target. The figures are worked out from `Gains::default()`; the case has not been run.

| Step | Calculation | Result |
|---|---|---|
| angle loop | 6 × (0 − 10) | asks for −60 °/s: turn left |
| rate error | −60 − 0, since the ground holds the drone still | −60 °/s |
| P term | 0.0014 × −60 | −0.084 |
| mixer | | right motors up, left motors down |
| the drone | | does not move |

In the air the last line would be different. The drone would turn, the error would shrink, and the loops would settle. On the ground the error stays at −60 °/s for as long as the drone stands there.

The I term adds up the error over time, and stops growing only when the error is zero:

```
I term grows at Ki × error = 0.003 × −60 = −0.18 per second
```

Its limit is 0.1, so it is full after about half a second. From then on:

```
output = P term + I term = −0.084 + −0.1 = −0.184
```

At a throttle of 0.5 that is the right motors at about 0.68 and the left ones at about 0.32, pushing against a slope that does not give.

**At take-off the output acts.**

- **The P part, −0.084, is right.** The drone is 10° off level, and that is the push it should get.
- **The I part, −0.1, is not.** It is larger than the P part, so the drone is pushed more than twice as hard as it should be, at the moment it is closest to the ground.
- **The I term lets go slowly.** It drains at `Ki × error`, the same pace it filled at, and only once the error has changed sign. The drone turns left, reaches level, and is still being pushed left. It goes past level and tilts the other way.

A tilt now means movement: a tilted drone is pushed towards its low side at `g × sin θ`. So the swing past level becomes a lurch to the left, a few centimetres off the ground.

Stage 2d showed a small version of this. A step asked for as the motors start [overshoots 2.6% against 0.5%](Stage2-d.md#a-step-at-the-moment-the-motors-start), and there the I term had 0.1 s to collect. Here it is at its limit.

**Level ground is not safe either, only slower.** The angle error on level ground is never exactly zero. An estimate that is 0.1° off gives a rate target of 0.6 °/s, and the I term then fills in about a minute where the slope took half a second. Yaw does the same with any gyro bias the calibration left behind.

**What fixes it** is arming, two items on:

- **Disarmed:** the motors are off and the loops do not run.
- **Armed but not yet flying:** the I terms are held at zero until the drone has left the ground, usually judged by the throttle passing a threshold.

The P and D terms need no such care. They have no memory, so they can run on the ground without harm. Only the I term stores up what happened while nothing could move.

**In this item** the runs start level and take off as soon as the calibration is done. The I terms then have a tenth of a second on the ground and almost no error to collect.

## The accelerometer, in flight

### What it reads now

[Stage 2b](Stage2-b.md#what-it-measures) settled what an accelerometer measures: the push on the drone from everything except gravity, seen from the body's own axes. On the ball joint that push was the joint's, straight up, always 1 g.

A flying drone has no joint. The pushes on it are its own thrust and the drag of the air:

```
reading = (thrust force + drag) / mass, turned into the body frame
```

On the ground, the ground pushes too, with whatever it takes to keep the drone still. So a drone standing on the ground reads what the joint read: 1 g, away from the ground.

### Thrust alone says "level"

The thrust always points along the body's own z axis. Seen from the body, it has no x part and no y part, whatever the tilt.

So with thrust alone, a drone at 30° at the hover command reads (0, 0, −1 g): nothing sideways, the same as a level one. The estimator turns a reading into angles from its sideways parts, and it would say level.

That sounds wrong and is right. The accelerometer cannot feel gravity. A drone at 30° with nothing but thrust is accelerating sideways, exactly as a level drone in a lift accelerates upwards. Inside it, both feel a push straight through the floor.

### Drag puts the tilt back

Drag pushes against the movement, which is sideways. Seen from the tilted body, that push has a sideways part, and the accelerometer feels it.

When the speed has settled, thrust and drag together are exactly the opposite of gravity. The drone is no longer accelerating. The reading is then what the joint gave, and the angles from it are true.

So the reading is wrong at the start of a tilt and right at the end, and it goes from one to the other with the same 3 s time constant as the speed:

| Time after tilting to 10° | Speed | Angle the accelerometer gives |
|---|---|---|
| 0 s | 0 | 0° |
| 1 s | 28% of final | about 3° |
| 3 s | 63% | about 6° |
| 6 s | 86% | about 9° |
| settled | 100% | 10° |

The same happens the other way. A drone that levels off at speed is slowing down, and the accelerometer says it is still tilted until it has stopped.

### What that does to the estimator

The estimator of Stage 2b follows the gyro quickly and leans towards the accelerometer slowly, with its time constant τ. The gyro is right about the tilt. The accelerometer says level. So after a tilt the estimate drifts from the true angle towards zero, at a pace set by τ, and comes back as the accelerometer does.

With τ = 1 s, the value chosen in Stage 2b, and a speed time constant of 3 s, the estimate falls to about 40% of the true angle at its worst, 1.6 s after the tilt.

The controller makes it worse. It holds the **estimate** at the target. If the estimate reads 4° when the truth is 10°, the controller tilts further until the estimate reads 10°. The true angle ends up at more than twice the target.

In the [trial](#what-to-expect), a drone asked for 10° reaches 21.6°. Asked for 20°, it reaches 43.6° and flies into the ground.

Hovering is not affected. With a level target and no movement, the accelerometer and the gyro agree.

### A longer τ

The fix that needs no new sensor is a longer τ, so that the estimate leans on the accelerometer more slowly than the speed changes.

[Stage 2b](Stage2-b.md#what-τ-trades) chose 1 s when the gyro bias was not yet removed: a bias of 1 °/s costs about 1° of error for each second of τ. Since [Stage 2c](Stage2-c.md) the calibration removes the bias before flight, and what is left over is small. That makes a much longer τ affordable.

| τ | Largest true angle, asked for 10° | Error from 0.05 °/s of bias left over |
|---|---|---|
| 1 s | 21.6° | 0.05° |
| 3 s | 15.6° | 0.15° |
| 10 s | 12.0° | 0.5° |

**Start with τ = 10 s.** The work of this item is to check that choice on the free body, with noise and with a bias that the calibration has not quite removed, and to settle on a value.

Two other things a longer τ changes:

- **Accelerometer noise matters less.** The estimate averages it over a longer time.
- **A wrong starting angle lasts longer.** The calibration gives the starting angle, so this only matters if the calibration is skipped.

A longer τ does not remove the effect. A drone that holds a tilt for a minute is misled in the end. What removes it is a second opinion on the speed, such as GPS, which is not in this plan. Angle mode on a small drone lives with it: tilts are held for seconds, not minutes.

## Shape of the code

Only the physics crate changes, and the simulation that uses it.

```
drone/
  physics/src/
    vec3.rs    three numbers along x, y and z
    quat.rs    turning a vector from the body frame to the world, and back
    quad.rs    mass, drag, position, velocity, the ground; a new `accel`
  sim/
    src/bin/quad.rs   a free run: take off, hover, tilt
    tests/free_flight.rs
    plot_flight.py, measure_flight.py
    stage3-a.sh
```

### In the physics

```rust
/// Three numbers along x, y and z: a position, a velocity or a force.
pub struct Vec3 {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

impl Quat {
    /// A vector given along the body's axes, along the world's.
    pub fn to_world(&self, v: Vec3) -> Vec3
    /// The other way.
    pub fn to_body(&self, v: Vec3) -> Vec3
}

/// What holds the drone up when its motors do not.
pub enum Support {
    /// A ball joint at the centre: it turns but goes nowhere. Stage 2.
    Joint,
    /// Nothing: it flies, falls, and stands on the ground.
    Free,
}

pub struct Quad {
    // ... as before, and:
    pub mass: f32,    // kg. Start with 1.2
    pub drag: f32,    // N per m/s. Start with 0.4
    pub support: Support,

    position: Vec3,   // m, world frame; z is down, so height is −z
    velocity: Vec3,   // m/s, world frame
    on_ground: bool,
}
```

New on `Quad`:

| Function | Gives |
|---|---|
| `position()`, `velocity()` | the two new states |
| `height()` | `−position.z` |
| `on_ground()` | whether the ground is holding it |
| `touchdown()` | how it last arrived on the ground: speed down, speed sideways, tilt. `None` if it has not landed |
| `set_position(..)` | where a run starts, for a test that begins in the air |
| `hover_command()` | `√(mass × g / (4 × thrust_coeff))`, for four equal motors |

`step` gains the three steps of [From forces to movement](#from-forces-to-movement) and the ground, after the rotation. `accel` returns the new reading.

### Choices to make in the code

- **The joint stays, as `Support::Joint`.** Every test and script of Stage 2 was made on it, and their figures are the record of that stage. A `Quad` is on the joint unless it is told otherwise, so nothing old changes. The free body is what Stage 3 uses.
- **One `Quad`, not a second type around it.** The ground has to stop the rotation, and the accelerometer needs the forces, so the two halves are not separate enough to split. It also keeps the parameters in one struct, as [`drone.md`](drone.md#physics-model) asks.
- **A free `Quad` starts on the ground,** level, at the origin. That is where a real flight starts.
- **Drag is in the world frame and the same in every direction.** One number, easy to reason about. A body-frame drag with different values along and across the propellers can replace it later without changing anything else.
- **The reading on the ground is computed, not copied.** It is the same formula as in flight, with the ground's push added. It then comes out as 1 g away from the ground without a special case.
- **The physics does not judge a landing.** It records the speed and the tilt. Whether 1 m/s is a landing or a crash is for the caller to say.
- **The body frame's `Accel` and the world frame's `Vec3` stay different types.** Mixing a body vector with a world vector is the mistake this item makes easy.

## Tests

### Turning a vector (`physics`)

| Test | Setup | Expected |
|---|---|---|
| Level changes nothing | a new quaternion, any vector | the same vector both ways |
| Nose up | 30° of pitch, the body's z axis | (0.5, 0, 0.866) in the world |
| Right side down | 30° of roll, the body's z axis | (0, −0.5, 0.866) |
| Yaw turns north to east | 90° of yaw, the body's x axis | (0, 1, 0) |
| There and back | any attitude, any vector, to the world and back | the vector it started as |
| Length is kept | any attitude, any vector | the same length |

### The free body (`physics`)

| Test | Setup | Expected |
|---|---|---|
| The joint is unchanged | `Support::Joint`, any commands | position and velocity stay zero; `accel` as in Stage 2b |
| Stands on the ground | free, motors off, 1 s | on the ground, height 0, velocity 0 |
| Falls | free, starting 10 m up, motors off, no drag, 1 s | 4.9 m lower, falling at 9.8 m/s |
| Hovers | starting in the air, all four at the hover command | height changes by less than 1 cm in 5 s |
| Climbs | all four at 0.76, no drag | upwards at 1.7 m/s² |
| Climb settles | the same with drag, 20 s | climbing at 5.2 m/s |
| Leaves the ground | on the ground, the command raised from 0.6 to 0.76 | on the ground at 0.6; off it once the thrust passes the weight |
| Nose down goes forward | held at −10° of pitch, hover command | x grows, y stays 0 |
| Right side down goes right | held at 10° of roll | y grows, x stays 0 |
| A tilt costs lift | held at 30°, hover command | sinks at 1.3 m/s² at first |
| Speed settles | held at 10° for 20 s | 5.1 m/s sideways |
| Coasts to a stop | level again at 5 m/s | 37% of the speed left after 3 s |
| Lands | falling at 2 m/s onto the ground | stops at height 0; `touchdown()` gives 2 m/s |
| Held on the ground | on the ground, uneven motor commands below the weight | the rates stay 0 and the attitude does not change |
| Reset | flown, then `reset()` | on the ground at the origin, as a new one |

### The accelerometer (`physics`)

| Test | Setup | Expected |
|---|---|---|
| On the ground, level | free, motors off | (0, 0, −9.807) |
| On the ground, tilted | set down at 20° of roll | as the joint read at 20° |
| Hovering | in the air, level, hover command | (0, 0, −9.807) |
| Falling | in the air, motors off, no drag | (0, 0, 0) |
| Thrust alone says level | held at 30°, hover command, no drag | x and y parts 0 |
| Drag puts the tilt back | held at 10° with drag, 20 s | the angles from the reading give 10° |

### The whole loop, free (`sim`)

Each runs the loop of `tests/controller_and_body.rs` on a free body: calibration on the ground, then a throttle that takes off.

| Test | Setup | Expected |
|---|---|---|
| Calibrates on the ground | motors off for the first second | the calibration passes; the drone has not moved |
| Takes off level | throttle 0.76 | off the ground within 0.2 s; roll and pitch within 0.5° of level all the way up |
| Hover holds its place | climb, then the hover command, level targets | sideways drift under 0.1 m in 10 s |
| A tilt moves it | target pitch −10° for 4 s | goes forward, not sideways |
| The estimate holds in a tilt | the same, with the chosen τ | the true angle stays within 3° of the target |
| The estimate holds when it stops | level again after the tilt | the true angle stays within 3° of level |
| Bias and noise | the same with noise, delay and a bias the calibration removes | the same limits |
| Short τ shows the problem | the same with τ = 1 s | the true angle passes 20° |

The last test is the one that shows why τ changed, as the weak motor with no I term did in Stage 2d.

## In the simulation

The quad program gains a free run:

- **`--support`** says what holds the body up: `free`, which is now the default, or `joint`.
- **`--stage 3a`** is new, and becomes what the program does when no stage is given: the free body and the new τ. `--stage 2d` and the earlier ones gain the joint and τ = 1 s in their presets, so the scripts of Stage 2 keep giving their figures. [`drone/sim/README.md`](drone/sim/README.md#running-an-earlier-stage) explains `--stage`.
- **The run** is 12 s: calibration on the ground until 1 s, a climb at `--takeoff-throttle` (0.76 unless given) until 3 s, then the hover command. The step in angle runs from 5 s to 9 s. On the joint the run is the 5 s of Stage 2.
- **`--mass`** and **`--drag`** set the two new parameters. `--estimator-tau` is there already.
- **CSV columns:** `north`, `east` and `height` in metres, `north_speed`, `east_speed` and `climb` in m/s, the three accelerometer readings, `on_ground`, and the `throttle` asked for.
- **`plot_flight.py`** draws the height, the distance north and east, the three speeds, the two angles with their estimates, and the motors.
- **`measure_flight.py`** prints the time of take-off, the height and the drift over the ground, the largest true angle and the largest error of the estimate during the tilt, the angle left over after it, and the speed reached.
- **`./stage3-a.sh`** runs the cases and saves the pictures in `drone/sim/stage3-a/`.

## What to expect

From a trial with the real controller and the Stage 2 body, with gravity, drag, movement and a ground added outside it for the trial, made before any of this item was written. [Results](#results) has the figures of the finished code. Mass 1.2 kg, drag 0.4 N per m/s.

### Taking off and hovering

Throttle 0.76 from 1 s to 3 s, then the hover command, level targets:

| Time | Height | Climbing at |
|---|---|---|
| 1.1 s | 0 m, leaving the ground | 0 m/s |
| 2.0 s | 0.6 m | 1.3 m/s |
| 3.0 s | 2.5 m | 2.4 m/s |
| 5.0 s | 6.1 m | 1.3 m/s |
| 8.0 s | 8.5 m | 0.5 m/s |
| 12.0 s | 9.6 m | 0.1 m/s |

- **It leaves the ground 0.1 s after the motors start.** The motors need that long to reach the hover speed.
- **Going back to the hover command does not stop the climb.** It only stops the drone speeding up. Drag then slows it over the next several seconds, and it gains another 7 m. Holding a height needs something that looks at the height.
- **Roll, pitch and the drift sideways stay at zero.** The estimate lies on the true angle throughout: a climb pushes only along the body's z axis.

### A tilt

The same run, with a target pitch of −10°, nose down, from 5 s to 9 s. τ = 1 s:

| Time | True pitch | Estimate | Forward speed | Height |
|---|---|---|---|---|
| 5.5 s | −11.7° | −8.6° | 0.6 m/s | 6.7 m |
| 6.0 s | −15.4° | −8.9° | 1.6 m/s | 7.1 m |
| 7.0 s | −20.4° | −9.5° | 3.8 m/s | 7.6 m |
| 8.0 s | −21.6° | −10.1° | 5.7 m/s | 7.4 m |
| 9.0 s | −19.6° | −10.5° | 7.1 m/s | 6.7 m |
| 10.0 s | −0.9° | −1.7° | 6.1 m/s | 5.8 m |
| 11.0 s | 7.6° | −1.1° | 3.8 m/s | 5.2 m |
| 12.0 s | 12.0° | −0.4° | 1.2 m/s | 4.6 m |

- **The estimate stays near the target and the truth does not.** The controller is doing its job on the number it is given.
- **Levelling off goes wrong the other way.** From 9 s the target is level. The drone is slowing down, the accelerometer still says "nose down", and the controller answers by putting the nose 12° up, while the estimate reads level.
- **It loses height from the moment it tilts,** and is 3 m lower by the end.

The same tilt for other values of drag and τ. The figure is the largest true pitch, for a target of 10°:

| Drag `k` | τ = 1 s | τ = 3 s | τ = 10 s |
|---|---|---|---|
| 0 | 41.7° and still growing | 21.6° and still growing | 13.6° and still growing |
| 0.2 | 28.4° | 18.1° | 12.7° |
| 0.4 | 21.6° | 15.6° | 12.0° |
| 0.8 | 17.0° | 13.2° | 11.2° |

- **A longer τ helps at every drag.** At τ = 10 s the angle is within about 2° of the target.
- **More drag helps too,** because the accelerometer comes right sooner.
- **With no drag nothing brings the estimate back.** The true angle grows for as long as the tilt is held. This is why the model needs drag at all.
- **The real `k` is not known.** The table is the reason to choose a τ that works across its range, and not one tuned to 0.4.

A target of 20° with τ = 1 s: the true pitch reaches 43.6°, and the drone reaches the ground at 4.5 m/s.

The picture to look for: the height panel climbing and then easing off without ever turning level; and in the angle panel of a tilt, with τ = 1 s, the estimate sitting on the target while the true angle swings to twice as far. With the longer τ the two lines lie together.

## Results

The free body, the arguments of the quad program and the tests above are written, and `Gains::default()` has the new time constant: **τ = 10 s**. `./stage3-a.sh` in `drone/sim/` saves the pictures in [`drone/sim/stage3-a/`](drone/sim/stage3-a/README.md). Each run is 12 s: calibration on the ground until 1 s, a climb until 3 s, then the hover command, with a tilt from 5 s to 9 s.

### Flying

| Run | Leaves the ground | Height at 5 s | At 12 s | Over the ground at 12 s |
|---|---|---|---|---|
| hover | 1.10 s | 6.1 m | 9.6 m, still climbing at 0.12 m/s | where it took off |
| hover, with noise, delay and a bias the calibration removed | 1.10 s | 6.1 m | 9.6 m | 1.4 m away |

The figures match the trial. The drone takes off level, the hover command does not stop the climb, and with true sensors it stays exactly over the place it left.

### The tilt, and the choice of τ

A target pitch of −10°, with drag at 0.4:

| τ | True pitch reached | Largest error of the estimate | Pitch left after the tilt | With real sensors |
|---|---|---|---|---|
| 1 s | −21.6° | 11.7° | 12.0° the other way | −21.9° |
| 3 s | −15.6° | 5.6° | −5.0° | −15.9° |
| 5 s | −13.7° | 3.7° | −3.6° | −14.1° |
| **10 s** | **−12.0°** | **2.0°** | **−2.1°** | **−12.4°** |
| 20 s | −11.0° | 1.1° | −1.2° | −11.5° |
| 30 s | −10.7° | 0.7° | −0.9° | −11.2° |

A longer τ is better here without end. What it costs shows in a hover with a gyro bias of 0.1 °/s that the calibration has not removed:

| τ | Tilt after 11 s of flying | Where that tilt is heading | Over the ground after 11 s |
|---|---|---|---|
| 1 s | 0.08° | 0.1° | 0.9 m away, at 0.07 m/s |
| 3 s | 0.37° | 0.3° | 1.7 m, at 0.27 m/s |
| 5 s | 0.58° | 0.5° | 2.1 m, at 0.36 m/s |
| **10 s** | **0.81°** | **1°** | **2.3 m, at 0.47 m/s** |
| 20 s | 0.95° | 2° | 2.5 m, at 0.52 m/s |
| 30 s | 1.00° | 3° | 2.6 m, at 0.54 m/s |

The tilt heads for bias × τ, as [Stage 2b](Stage2-b.md#what-τ-trades) found. On the joint a degree of error was only a degree. Flying, a degree of tilt is a push of `g × sin 1°`, which settles into a slide of half a metre a second.

A bias of 0.1 °/s is not a pessimistic figure. The calibration averages 1000 readings with noise of 2 °/s, which leaves it about 0.06 °/s out, and the bias itself moves as the chip warms. The run with real sensors shows it: the calibration has removed a bias of 1 °/s, and the drone still ends tilted 0.6° and 0.8° and 1.4 m from where it took off.

**τ = 10 s is the choice.** It holds a 10° tilt to within 2.5° with real sensors, and limits the error from a left-over bias of 0.1 °/s to 1°. At 30 s the tilt is nearly true, but the same bias would lean the drone 3°. At 3 s the hover is steadier, but the drone tilts half as far again as it is asked to.

Neither error goes away at any τ. A hover that stays over one place needs to know where it is, from a camera looking down or from GPS, and that is outside this plan. A pilot corrects a slide of half a metre a second without thinking about it.

### Other runs

| Run | What happens |
|---|---|
| the same tilt about roll | the same figures, to the east |
| no drag | the true pitch reaches −13.6° and is still growing; the speed never settles; the climb never fades, and the drone ends 29 m up |
| drag 0.8 | −11.2°, and half the speed |
| pitch −20° | −24.0°; it flies on |
| pitch −20° with τ = 1 s | −43.6°; it loses its lift and reaches the ground at 9.0 s, coming down at 4.5 m/s |

### What changed from the plan

- **`--support free` or `joint`,** where the plan had `--free`. Free is now what the program does unless told otherwise, so a switch that could only turn it on had nothing to do.
- **The plot shows the distance north and east against time,** not a path seen from above. Every run in this item moves along one line, and a map of a straight line says nothing.
- **`Quad::new_free`** makes a free body, standing on the ground. `Quad::new` still makes one on the joint.
- **`Quad` keeps what the accelerometer feels,** in the world frame, as a private field. The movement step fills it and `accel` turns it into the body frame.
- **No earlier test had to change for the new τ.** The tests of Stage 2 that depend on it set it themselves, and the quad program's earlier stages get τ = 1 s from `--stage`.

## Where this goes next

- The next item, [Stage 3b](Stage3-b.md), deals with the throttle: a fixed hover throttle, then more throttle in a tilt to make up for the lift lost.
- After that, [Stage 3c](Stage3-c.md) adds arming, the tilt cut-off, and holding the I terms at zero until the drone has left the ground. The ground of this item is what makes those testable.
- A height sensor and a loop that holds the height are an option, described in [`altitude.md`](altitude.md). The climb that never stops, above, is the case for it.
- The real airframe in [`quad10.md`](quad10.md) is 1.5 kg and hovers at a command of about 0.45, not 0.7. The mass and the drag go into the same parameters as the rest.
