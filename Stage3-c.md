# Stage 3c — Arming, motor limits and the tilt cut-off

This explains the third item of Stage 3 in [`drone.md`](drone.md#stage-3--host-simulation-hover):

> Arming, motor limits, and a cut-off when tilted past a safe angle; arming is refused until calibration has succeeded

Until now the controller has had one switch in it: calibrating, then flying. The motors started by themselves the moment the calibration was done. That was fine for a body on a ball joint. It is not fine for something with propellers, and [Stage 3a](Stage3-a.md#standing-on-the-ground-with-the-loops-running) found a second reason: a controller that runs its loops while the drone stands on the ground stores up a push that throws the drone sideways at take-off.

This item is about the edges of a flight: before take-off, at take-off, at the limits of the motors, and when it has gone wrong. Nothing in it changes how the drone flies in the middle.

## What is in and what is not

| In this item | Not yet |
|---|---|
| an arm switch: the motors stay off until the pilot asks | the gamepad buttons that work the switch (the next item) |
| arming refused until the calibration has passed, and with the throttle up | losing the radio signal, a low battery, a loop watchdog (the firmware, later) |
| the I terms held at zero until the throttle is nearly at hover | detecting the ground with a sensor |
| an idle speed, and a mixer that keeps the loops working at the motors' limits | a landing that the controller flies itself |
| a cut-off that stops the motors past 60° of tilt | recovering from upside down |

## The states

```
            power on, or disarmed
                     │
                     ▼
   ┌──────────► Calibrating ◄─────────────┐
   │   moved         │ a clean window      │
   └─────────────────┤                     │ arm switch off,
                     ▼                     │ or tilt past the cut-off
                 Disarmed                  │
                     │ arm switch turned on,
                     │ throttle low, not tilted
                     ▼                     │
                   Armed ──────────────────┘
```

| State | Motors | Loops | Leaves when |
|---|---|---|---|
| `Calibrating` | off | not running | a clean window of readings: to `Disarmed` |
| `Disarmed` | off | not running; the estimator keeps going | the pilot arms, and the checks pass: to `Armed` |
| `Armed` | at idle or above | running | the pilot disarms, or the tilt passes the cut-off: to `Calibrating` |

`Armed` replaces the `Flying` of Stage 2c. The flight code cannot know whether the drone is in the air. It knows that the motors are allowed to turn.

### Arming

The pilot asks with a switch, which arrives in `Targets` as `arm`. The controller arms when **all** of these hold:

| Check | Why |
|---|---|
| the state is `Disarmed` | the calibration has passed, so the gyro bias and the starting angle are known |
| the switch has just gone from off to on | a switch left on at power-up must not start the motors. The pilot has to move it |
| the throttle is below 0.05 | the motors start at idle, not at whatever the stick happens to say |
| the tilt is under the cut-off angle | no arming on its side or upside down |

If a check fails, nothing happens, and the controller records which one, for the log and later for a beep. The pilot fixes it and moves the switch again.

### Disarming

Switch off: the motors stop on that step. No conditions. This is the kill switch, and it must work in every state and at every throttle.

After a disarm the controller goes back to `Calibrating`, with everything cleared. That is the `reset()` of [Stage 2c](Stage2-c.md#choices-to-make-in-the-code), put to the use it was written for. A second later it is `Disarmed` again and can be armed.

Calibrating again is deliberate. A disarm often follows a crash, and after a crash the estimate may be wrong and the drone may be lying at an angle. A fresh window finds the angle it is at now. If the drone is still sliding or being picked up, the stillness check fails and the window starts again, so it cannot be armed in someone's hand.

## The I terms on the ground

### The problem, measured

[Stage 3a](Stage3-a.md#standing-on-the-ground-with-the-loops-running) worked the case through: on a slope the angle loop asks for a turn, the ground allows none, and the I term of the rate loop fills to its limit. The trial for this item ran it. The drone stands armed for 2 s at a throttle of 0.5 and then takes off:

| Slope | Front-left and front-right before take-off | Furthest past level | Where it is 4 s later |
|---|---|---|---|
| 0° | 0.50 and 0.50 | 0° | where it started |
| 1° | 0.46 and 0.54 | 3.1° | 0.9 m uphill, and moving |
| 5° | 0.36 and 0.64 | 8.2° | 2.1 m uphill, and moving |
| 10° | 0.32 and 0.68 | 7.6° | 1.7 m uphill, and moving |

The drone was tilted right side down, so the push to the left throws it uphill.

- **A slope of 1° is enough.** After 2 s the I term is about a third full, and the drone swings 3° past level.
- **A steeper slope is not worse.** The I term is at its limit in all of them, and on a steep slope more of the push is the P term's, which is wanted.
- **Time on the ground matters as much as the slope.** After 30 s on a 1° slope the I term is full: 8.6° past level and 2.4 m away.

No ground is level to within 1°, and no pilot takes off within a second of arming. Without a fix, every take-off does this.

### The fix

While the throttle is below a threshold, the three I terms of the rate loops are held at zero:

```
take-off throttle = 0.85 × hover_throttle
```

For the simulated drone that is 0.85 × 0.70 = 0.60. At a command of 0.60 the thrust is 73% of the weight, so the drone is certainly still on the ground. Above it the I terms run as before.

The P and D terms run all the time. They have no memory, so they cannot store anything up.

- **It needs no extra state.** The rule looks only at the throttle, on every step.
- **It works on landing too.** When the pilot lowers the throttle to settle, the I terms are cleared, and the next take-off starts clean.
- **In the air it costs little.** A pilot who drops the throttle below 85% of hover is descending fast. The I terms then let go of what they held, which for a weak motor is [about 1° of level](Stage2-d.md#why-the-stage-2c-controller-cannot-hold-level), and collect it again within a second of the throttle coming back.

This is why `hover_throttle` had to be a setting in [Stage 3b](Stage3-b.md#the-number).

### What is left

Taking off from a slope still moves the drone, with the fix. It leaves the ground tilted, and a tilted drone is pushed towards its low side until it has levelled, which takes a quarter of a second. From a 10° slope it ends 0.5 m downhill. That part is physics and not a fault.

## Motor limits

### An idle speed

While armed, no motor command goes below `idle`. **Start with 0.05.**

- **A stopped propeller steers nothing.** A motor at zero gives no thrust to take away, so the loops lose half their range on that side. It also takes longer to start from rest than to speed up.
- **The pilot can see and hear that it is armed.** All four turn slowly at zero throttle.
- **It lifts nothing.** Four motors at 0.05 give 4 × 6 × 0.05² = 0.06 N.

While not armed, the commands are exactly 0.

### Running out of room

The mixer adds the throttle and the three loop outputs, and until now each command was simply clamped to its range. [Stage 2a](Stage2-a.md#limits) kept the outputs small enough for that never to matter, at a throttle of 0.7. With the throttle in the pilot's hand, and raised further in a tilt by Stage 3b, it matters.

Take a throttle of 0.9 and a roll output of +0.2:

| | Left motors | Right motors | Difference |
|---|---|---|---|
| asked for | 1.1 | 0.7 | 0.4 |
| clamped, as now | 1.0 | 0.7 | 0.3: a quarter of the roll push is lost |
| shifted down together | 1.0 | 0.6 | 0.4: all of it kept |

Clamping one motor changes all three torques at once, and it does so exactly when the drone is working hardest. The rule from now on is that **steering comes before throttle**. After mixing:

1. **If the highest command is above 1,** lower all four by the excess.
2. **If the lowest is below idle,** raise all four by the shortfall.
3. **If both,** the four no longer fit in the range. Scale the differences between them down until they do, about their average.

Steps 1 and 2 keep every difference between the motors, and so all three torques, and give up a little total thrust. Step 3 is the only one that loses any steering, and it keeps the three torques in proportion.

### A ceiling on the throttle

The loops need room above the throttle to push with. The throttle that goes to the mixer, after the tilt compensation, is held to **0.85** at most. With the roll and pitch outputs limited to ±0.3, that leaves the shift of step 1 to do little.

On the simulated drone, which hovers at 0.70, the ceiling still allows a thrust of 1.47 times the weight, enough to start a climb at 4.6 m/s². It is also just above the 0.83 that the tilt compensation asks for at its limit of 45°. The real airframe hovers at about 0.45 and has far more room.

## The tilt cut-off

If the estimated tilt passes **60°**, the motors stop and the controller disarms, on that step.

```
tilted too far when cos(roll) × cos(pitch) < cos(60°) = 0.5
```

This is the same cosine that [Stage 3b](Stage3-b.md#the-fix) uses, so it costs nothing more to work out.

**What it is for.** A drone that passes 60° near the ground is flipping over: a propeller hit something, a motor is wired to the wrong place, a sign is wrong. The loops then see a huge error and drive the motors as hard as they can, into the ground. The cut-off stops the propellers before that. It is the protection for the first take-offs, when a wrong sign is likeliest.

**Why 60°.** It has to be past anything a good flight reaches, and short of a flip:

| What | Tilt reached |
|---|---|
| the largest angle the sticks will ask for | 30° |
| a 30° target, with the estimate's error from Stage 3a | 36° |
| the hard kick of Stage 2d, 2.4 N·m | 24° |
| where the tilt compensation stops adding | 45° |
| **the cut-off** | **60°** |
| on its side | 90° |

**What it costs.** High in the air, a cut-off means a fall. That is the trade: this controller does not recover from beyond 60°, so past that angle the choice is between falling with the propellers stopped and falling with them at full power. It is not for aerobatics, and a controller for those would not have it.

**It acts on the estimate.** Stage 3a showed the estimate reading low while the drone speeds up, so the true tilt at the cut-off can be more than 60°. With the longer τ the difference is a few degrees.

**No delay.** The estimate is smooth, since it follows the gyro and leans slowly on the accelerometer. One noisy reading cannot push it past 60°.

## Shape of the code

```
drone/
  flight/src/
    controller.rs  the states, the arming checks, the I hold, the cut-off
    mixer.rs       idle, the ceiling, and keeping the differences at a limit
    pid.rs         a way to clear the I term alone
  sim/
    src/bin/quad.rs  an arm time, a wait on the ground, a slope; `--stage 3c`
    tests/arming.rs
    stage3-c.sh
```

### In the flight code

```rust
pub struct Targets {
    // ... as before, and:
    pub arm: bool, // the arm switch: true asks for the motors to be allowed to turn
}

pub enum State {
    /// Collecting readings; the motors are off.
    Calibrating,
    /// Calibrated; the motors are off until the pilot arms.
    Disarmed,
    /// The motors turn and the loops run.
    Armed,
}

/// Why the last attempt to arm was refused.
pub enum ArmRefused {
    NotCalibrated,
    ThrottleNotLow,
    Tilted,
}

/// Why the controller last disarmed.
pub enum DisarmReason {
    Pilot,
    TiltCutOff,
}

/// How the controller comes to be armed.
pub enum Arming {
    /// By the switch, with the checks. The firmware uses this.
    Switch,
    /// By itself, as soon as the calibration is done, and with no cut-off.
    /// Stage 2 and the first two items of Stage 3 ran like this.
    Automatic,
}
```

New settings next to the gains: `arming`, `idle` (0.05), `throttle_ceiling` (0.85), `takeoff_fraction` (0.85) and `cut_off_tilt` (60°).

New on `Controller`: `last_refusal()` and `last_disarm()`, for the log.

`Pid` gains `clear_i()`, which zeroes the I term and leaves the D term's memory alone.

`mix` gains the two limits:

```rust
pub struct MixerLimits {
    pub idle: f32,    // no command below this
    pub ceiling: f32, // the throttle is held to this before mixing
}

pub fn mix(input: MixerInput, limits: MixerLimits) -> MotorCommands
```

### What `update` does each step

```
1. Calibrating: as before. On a clean window, go to Disarmed.
2. Disarmed: update the estimator; check the arm switch; motors off.
3. Armed:
     a. disarm if the switch is off, or the tilt is past the cut-off
     b. estimator, angle loops, rate loops, as before
     c. clear the three I terms if the throttle is below the take-off throttle
     d. raise the throttle for the tilt (Stage 3b)
     e. mix, with the limits
```

### Choices to make in the code

- **The estimator runs while disarmed.** A drone that is carried to its take-off spot after calibrating then still knows its angle when it is armed.
- **The cut-off and the disarm are checked first,** before anything that could turn a motor.
- **The I hold looks at the throttle the pilot asked for,** not the one raised for the tilt.
- **`Arming::Automatic` keeps the earlier stages running.** Their tests and scripts give a constant throttle from the first step, which the arming checks would refuse. `--stage 3b` and before use it. The firmware never will.
- **The mixer takes its limits as an argument,** and holds no state. The Stage 2 tests pass limits of 0 and 1 and get what they got before, as long as no command leaves its range.
- **One switch position, not two buttons.** The controller sees `arm` as true or false. The gamepad item turns two buttons into that.

## Tests

### Arming (`flight`)

| Test | Setup | Expected |
|---|---|---|
| Not while calibrating | switch turned on during the first window | stays `Calibrating`; refused as `NotCalibrated` |
| Disarmed after calibrating | a clean window, switch off | `Disarmed`; all four commands 0 |
| Arms | `Disarmed`, throttle 0, switch off then on | `Armed`; all four commands at idle |
| Not with the throttle up | the same with throttle 0.3 | stays `Disarmed`; refused as `ThrottleNotLow` |
| Then it does | throttle lowered, switch off and on again | `Armed` |
| Not from a switch left on | switch on from the first step | stays `Disarmed` until it is turned off and on |
| Not when tilted | calibrated at 70° of roll | refused as `Tilted` |
| Disarms at once | `Armed` at throttle 0.7, switch off | all four commands 0 on that step; `Calibrating` |
| Arms again | a second clean window, switch on | `Armed` |
| Automatic | `Arming::Automatic` | the commands of Stage 3b, step for step |

### The I hold (`flight`)

| Test | Setup | Expected |
|---|---|---|
| Held on the ground | calibrated at 10° of roll, armed, throttle 0.5, zero rates, 2 s | the commands do not change after the first step: 0.416 and 0.584 |
| Without it | the same, with the take-off fraction 0 | the commands move to 0.316 and 0.684 |
| Released above the threshold | the same at throttle 0.65 | the commands move apart |
| Cleared on the way down | throttle 0.65 for 1 s, then 0.5 | back to 0.416 and 0.584 on that step |

### The mixer's limits (`flight`)

| Test | Setup | Expected |
|---|---|---|
| In range, unchanged | throttle 0.7, roll 0.1 | as Stage 2a |
| Too high, shifted down | throttle 0.85, roll 0.2 | 1.0 and 0.6: the difference is still 0.4 |
| Too low, shifted up | throttle 0.1, roll 0.2, idle 0.05 | 0.45 and 0.05 |
| Too wide, scaled | differences that span more than 0.95 | lowest at idle, highest at 1.0, in the same proportions |
| Yaw is kept | throttle 0.85 with roll, pitch and yaw together | `fl + br − fr − bl` as asked for |
| The ceiling | throttle 0.95, no outputs | all four 0.85 |
| Idle | throttle 0, no outputs, idle 0.05 | all four 0.05 |

### The cut-off (`flight`)

| Test | Setup | Expected |
|---|---|---|
| Under it | armed, the estimate at 59° of roll | still `Armed` |
| Over it | the estimate at 61° | commands 0; `Calibrating`; the reason is `TiltCutOff` |
| Both axes count | roll 50° and pitch 50°: each under 60°, but a tilt of 66° together | cut |
| Not when automatic | `Arming::Automatic`, 170° | still running, as in Stage 2d |

### The whole loop, on the ground and off it (`sim`)

| Test | Setup | Expected |
|---|---|---|
| Stays down until armed | throttle 0.76, never armed | on the ground, motors off |
| Takes off level | arm, wait 2 s, throttle 0.76 | roll and pitch within 0.5° on the way up |
| From a 10° slope | arm, wait 2 s at throttle 0.5, take off | never more than 1° past level; within 1 m of the start after 4 s |
| From a 1° slope after a long wait | the same, waiting 30 s | never more than 0.5° past level |
| The slope without the hold | the 10° case with the take-off fraction 0 | more than 5° past level: the test that shows the problem |
| A second take-off | fly, land, wait 5 s on the ground, take off | as clean as the first |
| A flip is cut | a kick large enough to pass 60° | motors off on the step the estimate passes 60°; the reason is `TiltCutOff` |
| A hard kick is not | the 2.4 N·m kick of Stage 2d | stays armed and recovers |
| Disarm in the air | switch off at 5 m | motors off; it falls and lands |

## In the simulation

- **`--stage 3c`** is new, and becomes what the program does when no stage is given: arming by the switch, with the I hold, the limits and the cut-off.
- **The run:** calibration until 1 s; armed at `--arm-at` (1.2 s unless given); on the ground at `--ground-throttle` (0 unless given) until `--takeoff-at` (3 s unless given); then the climb and the hover as in Stage 3a.
- **`--start-roll` and `--start-pitch`** now mean a slope, since the drone starts on the ground.
- **`--no-i-hold`** sets the take-off fraction to 0, for the comparison.
- **`--disarm-at`** turns the switch off at that time.
- **CSV columns:** `state` now runs 0 to 2; `arm` is the switch; `i_held` is 1 while the I terms are held.
- **`plot_flight.py`** shades the time disarmed, and marks the moment of a cut-off.
- **`./stage3-c.sh`** runs the cases and saves the pictures in `drone/sim/stage3-c/`.

## What to expect

From a trial with the real controller and the Stage 2 body, with gravity, drag, movement and a ground added outside it, and the I hold added to a copy of the controller. Nothing is in the repo yet. τ = 10 s. The drone stands on a slope, right side down, at a throttle of 0.5, and then takes off at 0.76.

| Slope | Wait | I hold | Furthest past level | Level again after | Sideways after 4 s |
|---|---|---|---|---|---|
| 10° | none | off or on | 0.8° | 0.21 s | 0.3 m downhill |
| 10° | 0.2 s | off | 3.1° | 0.15 s | 0.4 m uphill |
| 10° | 2 s | off | 7.6° | 0.11 s | 1.7 m uphill, still moving at 0.3 m/s |
| 10° | 2 s | on | 0.2° | 0.25 s | 0.5 m downhill |
| 5° | 2 s | off | 8.2° | 0.08 s | 2.1 m uphill, still moving |
| 5° | 2 s | on | 0.1° | 0.25 s | 0.3 m downhill |
| 20° | 2 s | off | 6.5° | 0.14 s | 0.9 m uphill |
| 20° | 2 s | on | 0.4° | 0.25 s | 1.0 m downhill |
| 1° | 30 s | off | 8.6° | 0.03 s | 2.4 m uphill, still moving |
| 1° | 30 s | on | 0.02° | 0.25 s | 0.05 m downhill |

- **With the hold, the wait makes no difference.** Every take-off levels in a quarter of a second and swings less than half a degree past.
- **Without it, 0.2 s on the ground is already too long.**
- **The direction changes.** Without the hold the drone is thrown uphill, past level and beyond. With it, the drone drifts downhill, the way it was leaning, by an amount that goes with the slope.
- **A take-off with no wait is the same either way,** which is why Stage 3a's runs could do without the hold.

The motor limits and the cut-off were not part of the trial. Their figures come with the code.

The picture to look for: the roll angle at take-off from a slope. Without the hold it dives through zero and far out the other side. With it, the line comes down to zero and stays there.

## Where this goes next

- The next item is the gamepad: two buttons become the `arm` switch, and the sticks fill the rest of `Targets`.
- Stage 5's firmware adds the protections that need hardware: losing the receiver's signal, a low battery, and a watchdog on the loop. Each of them ends in the same place, a disarm.
- The first bench test in [`octocopter.md`](octocopter.md) is with the propellers off and the frame tilted by hand. The cut-off should be seen to work there before a propeller goes on.
