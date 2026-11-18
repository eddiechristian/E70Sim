# Stage 2a — The three-axis rigid body and the X mixer

This explains the first item of Stage 2 in [`drone.md`](drone.md#stage-2--host-simulation-self-levelling-attitude):

> Three-axis rigid body with four motors and the X mixer

Stage 1 controlled one axis with two motors. This item replaces the seesaw with a whole quadcopter body that can rotate about all three axes, and replaces the two-motor mixer with the real four-motor one.

Two parts of Stage 1 carry over unchanged: the `Motor` and the `Gyro` in `drone/physics/`. There are now four motors and three gyro axes.

## What is in and what is not

| In this item | Not yet |
|---|---|
| rotation about all three axes | gravity |
| four motors and their thrusts | moving up, down or sideways |
| torques about roll, pitch and yaw | the accelerometer and the estimator (the next item) |
| the mixer | the angle loop (the item after) |

The picture to keep in mind is a drone held by a ball joint at its centre. It can tip and turn any way, but it cannot go anywhere. That is the seesaw's pivot, extended from one axis to three.

## Axes and signs

Everything in this stage depends on agreeing which way is positive. Most first-flight crashes are sign errors, and the simulation is the cheap place to find them.

### The body frame

Three axes are fixed to the drone and turn with it:

| Axis | Points | Rotation about it | Positive rotation |
|---|---|---|---|
| x | forward, out of the nose | roll | right side goes down |
| y | out of the right side | pitch | nose goes up |
| z | down, out of the belly | yaw | nose turns right (clockwise seen from above) |

This is the convention aircraft and most flight controllers use. Its advantage is that the positive directions are the ones a pilot expects: roll right, nose up, turn right.

The positive rotations are not three separate choices. They follow from the axes by the **right-hand rule**: point the right thumb along the axis, and the fingers curl in the positive direction.

Two consequences worth knowing in advance:

- **z points down.** A motor's thrust pushes the drone up, so it is a force in the negative z direction. This looks odd at first and is normal for this convention.
- **The seesaw's sign is the other way round.** In Stage 1 a stronger right motor gave a positive rate, with the right side rising. Here positive roll is the right side going down, so stronger *left* motors give positive roll. The seesaw stays as it is, as the Stage 1 model. The new body uses the convention above throughout.

This is a decision, and the alternative is a frame with z pointing up. Either works as long as every part agrees. The reason to settle it now is that the mixer, the physics, the gyro and later the estimator all depend on it.

### The motors

Seen from above, with the nose at the top:

```
              front
       FL (0)        FR (1)
        CW  ↻        ↺  CCW
              ╲    ╱
               ╲  ╱
                ╳
               ╱  ╲
              ╱    ╲
       CCW  ↺        ↻  CW
       BL (3)        BR (2)
              back
```

| Field | Name | Position | Spins (seen from above) |
|---|---|---|---|
| `fl` | FL | front left | clockwise |
| `fr` | FR | front right | counter-clockwise |
| `br` | BR | back right | clockwise |
| `bl` | BL | back left | counter-clockwise |

- **The X layout** means the arms sit at 45° to the nose, so there are two motors at the front and two at the back. The alternative, a plus layout, has one motor at the front.
- **The motors are named, not numbered.** In the code each is a field (`fl`, `fr`, `br`, `bl`) of a struct. The physics and the mixer are in separate crates that do not know about each other, and matching motors by name means no list order has to agree between them. The numbers in the drawing only show the order going clockwise from the front left.
- **Diagonal pairs spin the same way.** The reason is under [Yaw](#yaw) below.

## The physics, step by step

Each step, the model does five things. The first is the seesaw's step 1, done four times; the rest are the seesaw's steps 2 to 4 with three axes in place of one.

1. **Motor commands to thrusts.** Each `Motor` follows its command with a lag and gives a thrust.
2. **Thrusts to three torques.** Roll, pitch and yaw.
3. **Torques to three rotation rates.**
4. **Rates to attitude.**
5. **Rates to gyro readings,** through three `Gyro`s.

### Step 2: thrusts to torques

Write the four thrusts as `T_FL`, `T_FR`, `T_BR` and `T_BL`.

#### Roll and pitch

The arm length `L` is the distance from the centre to a motor, as on the seesaw. Because the arms sit at 45°, each motor is a distance `d` to the side of the centre line and the same distance ahead of or behind it:

```
d = L / √2        with L = 0.2 m, d ≈ 0.141 m
```

Roll is the left side against the right side. Pitch is the front against the back:

```
roll torque  = d × (T_FL + T_BL − T_FR − T_BR)      left minus right
pitch torque = d × (T_FL + T_FR − T_BR − T_BL)      front minus back
```

- **Stronger left motors** lift the left side, so the right side goes down: positive roll.
- **Stronger front motors** lift the nose: positive pitch.

This is the seesaw's `torque = (thrust R − thrust L) × arm length`, with two motors on each side and `d` as the lever.

#### Yaw

No motor points sideways, so yaw cannot come from thrust. It comes from the propellers' **reaction torque**.

A motor turns its propeller one way, and the air resists. By Newton's third law the motor, and the frame it is bolted to, is pushed the other way. A propeller spinning clockwise tries to turn the drone counter-clockwise. This is why a helicopter needs a tail rotor.

- **With all four at the same speed,** the two clockwise propellers cancel the two counter-clockwise ones. That is why half spin each way.
- **To yaw,** speed up one pair and slow the other. The reaction torques no longer cancel, and the drone turns.
- **Why the pairs are diagonal:** FL and BR are on opposite sides of both centre lines. Speeding both up adds equal thrust to the left and the right, and to the front and the back, so it causes no roll and no pitch. Only the yaw changes.

The reaction torque of a propeller is proportional to its thrust:

```
reaction torque = c × thrust
```

`c` is a property of the propeller, in metres. Start with `c = 0.015` m; like the other parameters it is a guess until there are parts.

Positive yaw is clockwise seen from above. The counter-clockwise propellers (FR and BL) push the frame clockwise, so they count as positive:

```
yaw torque = c × (T_FR + T_BL − T_FL − T_BR)
```

#### How strong each axis is

Using the Stage 1 values (`thrust_coeff = 6` N, base throttle 0.7), a command of `u` added to one pair and taken from the other gives:

```
roll or pitch torque = 8 × 6 × 0.7 × d × u = 4.75 × u   N·m
yaw torque           = 8 × 6 × 0.7 × c × u = 0.50 × u   N·m
```

The 8 is the seesaw's 4 (see [`Stage1-c.md`](Stage1-c.md#starting-gains)) doubled, because two motors go up and two go down.

Yaw is about nine times weaker than roll and pitch, because `c` is nine times smaller than `d`. This is true of every quadcopter: tilting is done with thrust on a long lever, and turning is done with a side effect. Yaw is slow, and it is the first axis to run out of motor range.

### Step 3: torques to rates

There are three inertias, one per axis:

| | Meaning | Starting value |
|---|---|---|
| `Ixx` | how hard it is to roll | 0.015 kg·m², the seesaw's value |
| `Iyy` | how hard it is to pitch | 0.015 kg·m²; a symmetric X frame rolls and pitches alike |
| `Izz` | how hard it is to yaw | 0.025 kg·m² |

`Izz` is larger because all four motors are far from the yaw axis, while for roll and pitch each motor is only `d` from the axis.

To a first approximation each axis is its own seesaw:

```
roll rate  += (roll torque  / Ixx) × dt
pitch rate += (pitch torque / Iyy) × dt
yaw rate   += (yaw torque   / Izz) × dt
```

With a command of 0.05, as in Stage 1:

| Axis | Torque | Acceleration |
|---|---|---|
| roll or pitch | 0.238 N·m | about 910 °/s² |
| yaw | 0.025 N·m | about 58 °/s² |

Roll and pitch respond √2 (about 1.4) times faster than the seesaw's 640 °/s², because four motors on levers of `d` beat two motors on levers of `L`. The Stage 1 gains were chosen for the seesaw, so they will be about 40% too strong here and need a second look. Yaw responds 16 times more slowly than roll and will need its own, much larger, gains.

#### The cross term

The three axes are not quite independent. A body spinning about two axes at once feels a torque about the third. The full equations, called Euler's rotation equations, are:

```
roll rate  += (roll torque  − (Izz − Iyy) × pitch rate × yaw rate)   / Ixx × dt
pitch rate += (pitch torque − (Ixx − Izz) × yaw rate   × roll rate)  / Iyy × dt
yaw rate   += (yaw torque   − (Iyy − Ixx) × roll rate  × pitch rate) / Izz × dt
```

Each line is the simple version with one extra term. Three things about that term:

- **It is a product of two rates,** so it is tiny when the drone is rotating slowly or about one axis only. Near a hover it does almost nothing.
- **It matters in fast combined moves.** Rolling and yawing at 200 °/s each produces a pitch acceleration of about 465 °/s², which is half of what a 0.05 command gives.
- **The yaw line's term is zero here,** because `Iyy = Ixx`.

All three rates on the right-hand side must be the values from the start of the step. Work out the three accelerations first, then update the three rates.

It costs three lines to include, so the model includes it. The controller is not told about it: the rate PIDs treat it as one more disturbance to correct.

### Step 4: rates to attitude

On the seesaw this step was `angle += rate × dt`. With three axes that no longer works, and this is the one new idea in the item.

#### Why three angles are not enough

The obvious approach is to keep a roll, a pitch and a yaw angle and add each rate to its own angle. It fails for two reasons:

- **The rates are measured in the drone's own frame, and the angles are not.** With the nose pointing straight up, turning about the drone's own z axis no longer changes its compass heading; it rolls it over. Which angle a rate changes depends on the current attitude.
- **Three angles break down at 90° of pitch.** With the nose straight up, roll and yaw describe the same motion, and the angles jump. In a trial of this model, pitching smoothly past vertical made the reported roll and yaw flip from 0° to 180° in one step. This is called gimbal lock.

Near level, adding the rates to the angles is nearly right, and the estimator in the next item does exactly that. The physics model has to stay correct at any attitude, so it uses something better.

#### The quaternion

A **quaternion** is four numbers, `(w, x, y, z)`, that together describe an orientation. It has no bad attitudes and no jumps. The four numbers are not angles and are not meant to be read directly: they are the model's internal record of which way the drone is facing, and angles are worked out from them for the log.

- **Level, nose forward** is `(1, 0, 0, 0)`. That is the starting value.
- **The length is always 1:** `w² + x² + y² + z² = 1`.

The model needs three operations.

**1. Update from the rates.** Each step, with the three rates in rad/s:

```
dw = 0.5 × (−x × roll rate − y × pitch rate − z × yaw rate)
dx = 0.5 × ( w × roll rate + y × yaw rate   − z × pitch rate)
dy = 0.5 × ( w × pitch rate + z × roll rate − x × yaw rate)
dz = 0.5 × ( w × yaw rate  + x × pitch rate − y × roll rate)

w += dw × dt        x += dx × dt        y += dy × dt        z += dz × dt
```

This is the three-axis version of `angle += rate × dt`. Work out all four changes from the old values before applying any of them.

**2. Normalise.** The update leaves the length very slightly off 1, and the error builds up: in a trial, ten seconds of tumbling without correction left the length at 1.02. After each update, divide all four numbers by the length:

```
length = √(w² + x² + y² + z²)
w /= length        x /= length        y /= length        z /= length
```

**3. Convert to angles, for the log and the plots.**

```
roll  = atan2(2 × (w×x + y×z), 1 − 2 × (x² + y²))
pitch = asin (2 × (w×y − z×x))
yaw   = atan2(2 × (w×z + x×y), 1 − 2 × (y² + z²))
```

The results are in radians. Clamp the value given to `asin` to the range −1 to 1 first, since rounding can push it a hair outside and `asin` then returns NaN. These angles do jump near 90° of pitch, as described above, but that now affects only the plot and not the model.

These angles are also the **true attitude** that the estimator will be judged against in the next item, in the same way the true rate stood beside the measured rate in Stage 1.

A check that the three operations are right: rotating at a steady 90 °/s about one axis for half a second must give 45° on that angle and 0° on the other two. The trial gave exactly that for each axis.

### Step 5: rates to gyro readings

The three rates are already in the drone's own frame, which is what a gyro bolted to the drone measures. Each one goes through its own `Gyro` from [`Stage1-d.md`](Stage1-d.md), with its own seed so that the three axes do not get identical noise.

## The X mixer

The mixer is the physics of step 2 run backwards. The physics answers "given four thrusts, what are the torques?" The mixer answers "given the torques I want, what should the four motors do?"

### Inputs and outputs

| Input | From | Meaning |
|---|---|---|
| throttle | a constant for now | the `base` of Stage 1: the same on all four motors |
| roll | the roll rate PID | positive to roll right |
| pitch | the pitch rate PID | positive to raise the nose |
| yaw | the yaw rate PID | positive to turn right |

The output is four motor commands, each 0.0 to 1.0, named by position: `fl`, `fr`, `br` and `bl`.

### The formula

```
FL = throttle + roll + pitch − yaw
FR = throttle − roll + pitch + yaw
BR = throttle − roll − pitch − yaw
BL = throttle + roll − pitch + yaw
```

As a table of signs:

| Motor | throttle | roll | pitch | yaw |
|---|---|---|---|---|
| FL | + | + | + | − |
| FR | + | − | + | + |
| BR | + | − | − | − |
| BL | + | + | − | + |

Each column can be read off the physics:

- **Roll:** `+` on the left motors, `−` on the right. Stronger left motors give positive roll.
- **Pitch:** `+` on the front motors, `−` on the back.
- **Yaw:** `+` on the counter-clockwise motors, `−` on the clockwise ones.

Each of the roll, pitch and yaw columns has two `+` and two `−`. Whatever one of them adds to two motors it takes from the other two, so the total thrust stays about the same. Each column also leaves the other two axes alone: the roll column, for example, adds equally to the front and the back, so it causes no pitch.

The Stage 1 mixer, `left = base − output` and `right = base + output`, is one column of this table.

### A worked example

Throttle 0.7 and a roll command of 0.05, with pitch and yaw at 0:

```
FL = 0.7 + 0.05 = 0.75
FR = 0.7 − 0.05 = 0.65
BR = 0.7 − 0.05 = 0.65
BL = 0.7 + 0.05 = 0.75
```

The left motors speed up and the right ones slow down. The same command on pitch gives 0.75 at the front and 0.65 at the back, and on yaw gives 0.75 on FR and BL and 0.65 on FL and BR.

### Limits

Each command is clamped to 0.0–1.0 after mixing, as in Stage 1. With three inputs adding up, a limit is easier to hit than before: throttle 0.7 with roll and pitch at +0.15 and yaw at −0.15 asks FL for 0.7 + 0.15 + 0.15 + 0.15 = 1.15, which is more than full speed.

When a command is clamped, the motor does less than the mixer asked, and the torques are no longer the ones requested. Clamping one motor disturbs all three axes at once. Flight controllers deal with this by lowering all four commands together, or by giving up yaw first, and this project may need that later. To start with, the fix is to keep the three PID output limits small enough that the sum stays in range:

| Output | Limit | Reason |
|---|---|---|
| roll | ±0.1 | at worst one motor gets 0.7 + 0.1 + 0.1 + 0.1 = 1.0 and another 0.7 − 0.3 = 0.4, so no command leaves its range |
| pitch | ±0.1 | the same |
| yaw | ±0.1 | the same |

These are tighter than Stage 1's ±0.3, and the retune has to respect them.

### Where the mixer lives

In `drone/flight/`, because the firmware needs the identical function. It is a pure function with no state: four numbers in, four out, each set held in a named struct. The Stage 1 `mix` in `drone/sim/` stays with the seesaw.

## Shape of the code

A sketch, to make the above concrete:

```
drone/physics/
    quat        a quaternion: update from rates, normalise, convert to angles
    quad        the body: four Motors, three inertias, arm length, c,
                the three rates and the attitude
                    step(MotorCommands, dt)
                    rates()   -> Rates
                    angles()  -> Angles

drone/flight/
    mixer       mix(MixerInput) -> MotorCommands
```

Every value that crosses a function boundary has a named type, so nothing is passed as a bare tuple or array:

| Type | Crate | Fields | Unit |
|---|---|---|---|
| `MixerInput` | `flight` | `throttle`, `roll`, `pitch`, `yaw` | fractions of full motor command |
| `MotorCommands` | `flight` and `physics` | `fl`, `fr`, `br`, `bl` | 0.0 to 1.0 |
| `Motors` | `physics` | `fl`, `fr`, `br`, `bl` | four `Motor`s |
| `Inertia` | `physics` | `ixx`, `iyy`, `izz` | kg·m² |
| `Rates` | `physics` | `roll`, `pitch`, `yaw` | rad/s |
| `Angles` | `physics` | `roll`, `pitch`, `yaw` | rad |

`MotorCommands` exists twice because neither crate depends on the other. The simulation copies one into the other field by field, so a motor is always matched by name and never by its place in a list.

Three notes:

- **Named fields prevent swapped arguments.** `mix(0.7, 0.05, 0.0, 0.0)` compiles whichever way round the numbers are; `MixerInput { throttle: 0.7, roll: 0.05, pitch: 0.0, yaw: 0.0 }` cannot be misread.

- **The quaternion is small enough to write by hand:** a struct of four numbers and the three operations above. The estimator may want it later, so it is worth understanding. A maths crate such as `glam` is the alternative.
- **The simulation's loop keeps its shape.** Step 1 reads three rates, step 2 runs three PIDs, step 3 calls the new mixer and step 4 steps the new body. The CSV gains columns for the extra axes and motors. This loop is a second program, `drone/sim/src/bin/quad.rs`, described under [In the simulation](#in-the-simulation).

## Tests

### The mixer

| Test | Inputs | Expected |
|---|---|---|
| Throttle only | throttle 0.7, the rest 0 | all four 0.7 |
| Roll | roll 0.05 | FL and BL 0.75, FR and BR 0.65 |
| Pitch | pitch 0.05 | FL and FR 0.75, BR and BL 0.65 |
| Yaw | yaw 0.05 | FR and BL 0.75, FL and BR 0.65 |
| Sum unchanged | any roll, pitch and yaw that do not clamp | the four commands add up to 4 × throttle |
| Clamp | throttle 0.9, roll 0.3 | no command above 1.0 or below 0.0 |

### The quaternion

| Test | Setup | Expected |
|---|---|---|
| Starts level | a new quaternion | all three angles 0 |
| One axis at a time | 90 °/s about one axis for 0.5 s | 45° on that angle, 0° on the others; one test per axis |
| Stays at length 1 | tumble for 10 s with all three rates non-zero | length within 0.0001 of 1 |
| There and back | 90 °/s for 0.5 s, then −90 °/s for 0.5 s | back to all three angles 0 |

### The body

| Test | Setup | Expected |
|---|---|---|
| Equal commands | all four at 0.7 for 1 s | all three rates exactly 0 |
| Left stronger | FL and BL at 0.75, FR and BR at 0.65 | roll rate positive, pitch and yaw rates 0 |
| Front stronger | FL and FR at 0.75, BR and BL at 0.65 | pitch rate positive, the others 0 |
| Counter-clockwise stronger | FR and BL at 0.75, FL and BR at 0.65 | yaw rate positive, the others 0 |
| Yaw is weak | the same size of command on roll and on yaw | the yaw rate is far smaller |
| No drag | push, then equal commands | the rates hold |
| Cross term | roll and yaw rates together, equal commands | a pitch rate appears |

### The two together

These are the tests that catch a sign error or a spin direction that differs between the two crates. Each one copies the mixer's output into the body's `MotorCommands` by name and steps the body:

| Test | Mixer input | Expected from the body |
|---|---|---|
| Roll agrees | roll +0.05 | roll rate positive, the others 0 |
| Pitch agrees | pitch +0.05 | pitch rate positive, the others 0 |
| Yaw agrees | yaw +0.05 | yaw rate positive, the others 0 |
| Negative commands agree | roll, pitch and yaw −0.05, one at a time | that rate negative |

If a sign or a spin direction is wrong in one place, one of these fails. On a real drone the same mistake is a flip on take-off.

## What to expect

A trial of these equations, with throttle 0.7 and a command of 0.05 held for half a second:

| Command | Rates after 0.5 s | Angles after 0.5 s |
|---|---|---|
| roll 0.05 | roll 418 °/s, the others 0 | roll 97° |
| pitch 0.05 | pitch 418 °/s, the others 0 | past vertical |
| yaw 0.05 | yaw 27 °/s, the others 0 | yaw 6° |

With no controller a small command tumbles the drone within half a second, as the seesaw did. Yaw moves at a sixteenth of the pace.

## In the simulation

The body and the mixer are wired into the simulation as a second program, next to the seesaw's:

```
cd drone/sim
cargo run --bin quad > quad.csv
python3 plot_quad.py
```

It is the Stage 1 loop with three of everything: three gyros, three rate PIDs, the mixer and the body. One axis, chosen with `--axis`, is asked for 50 °/s for a second, and the other two are held at 0. [`drone/sim/README.md`](drone/sim/README.md#the-quad) lists the arguments and the CSV columns.

There is no angle loop yet, so this is rate control only: the drone turns at the rate asked for and stays at whatever angle it ends up at. The angle loop is a later Stage 2 item.

With the default gains, a 50 °/s step gives:

| Axis | Rise to 90% | Overshoot | Settled within 5% | Other two axes | Motor commands |
|---|---|---|---|---|---|
| Roll | 0.11 s | 0.9% | 0.13 s | stay at 0 | 0.63 to 0.77 |
| Pitch | 0.11 s | 0.9% | 0.13 s | stay at 0 | 0.63 to 0.77 |
| Yaw | 0.19 s | 2.6% | 0.21 s | stay at 0 | 0.40 to 1.00 |

- **Roll and pitch are identical,** as they should be for a symmetric body.
- **Yaw is slower and works the motors far harder.** Its output reaches the ±0.3 limit, and two motors reach full power for a moment. That is the weak yaw of this layout showing up in closed loop.
- **With `--noise 2 --delay 2`** the figures barely change, and the two axes that are held at 0 wander by about 0.6 °/s.

Pictures of these runs are in [`drone/sim/stage2-a/`](drone/sim/stage2-a/README.md), and `./stage2-a.sh` in `drone/sim/` regenerates them.

The roll and pitch gains are the ones chosen in [`Stage1-d.md`](Stage1-d.md), divided by √2, because one unit of PID output turns the quad √2 times harder than it turned the seesaw. The yaw gain is a first guess, not a tuned value.

## Where this goes next

- The next Stage 2 item is the estimator, explained in [`Stage2-b.md`](Stage2-b.md). It needs an accelerometer reading, which the body can supply once it knows which way is down: the direction of gravity seen from the drone's frame comes from the same quaternion.
- The item after puts three rate PIDs and two angle loops around this body.
- Stage 3 adds gravity and movement. The total thrust, which this item ignores, then decides whether the drone climbs or falls.
