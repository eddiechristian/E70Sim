# Quadcopter Simulation Plan

## Goal

Simulate a quadcopter from scratch, before buying any parts.

- I write the flight code myself: the attitude estimator, the PID controllers and the mixer that sets the speed of each of the four motors.
- The target is a **stable hover with self-levelling**: with the sticks centred the drone returns to level and holds it.
- The same code runs in two places:
  - a fast simulation on the host PC, for tuning and plots
  - the real firmware on the SAME70 emulator, to prove the drivers and timing

## How the loop closes

The gyro readings depend on the PID output, and the PID output depends on the gyro readings. Stepping time in small increments resolves this. Each step:

1. The flight code reads the gyro and accelerometer.
2. The estimator works out the current angle (roll, pitch) from them.
3. The PIDs compute a correction, and the mixer turns it into four motor commands.
4. The physics model turns the motor commands into thrusts and body torques.
5. The model integrates those into new angular rates and angles.
6. The new rates and accelerations become the next sensor readings.

The flight code only ever sees the result of its previous outputs, as on a real drone.

## Decisions

### Airframe: Mark4 V2 10 inch quad

Decided 2026-11-18: the first real aircraft is a small quad on the Mark4 V2 10 inch carbon frame, flown on 4S to start. The parts list, prices and estimated physics parameters are in [`quad10.md`](quad10.md). The larger octocopter and 30 inch quad options in [`octocopter.md`](octocopter.md) are later builds.

### Sensor: ICM-42688-P instead of the BNO055

The BNO055 is a poor fit for a quadcopter:

- Its fusion output updates at 100 Hz. Flight controllers run the inner rate loop at 1 kHz or more.
- Its fusion runs inside the chip, so it hides the estimator, which is one of the parts I want to write.
- It is I2C only.

The **TDK InvenSense ICM-42688-P** is the usual choice on current flight controllers:

| | BNO055 | ICM-42688-P |
|---|---|---|
| Axes | 9 (gyro, accel, mag) | 6 (gyro, accel) |
| Gyro output rate | 100 Hz fused | up to 32 kHz raw |
| On-chip fusion | yes | no, the firmware does it |
| Bus | I2C | SPI (up to 24 MHz) or I2C (up to 1 MHz) |
| Gyro noise | higher | low |

Consequences:

- The firmware needs its own attitude estimator (a complementary filter first, Mahony later if needed). Self-levelling depends on it.
- There is no magnetometer, so yaw heading drifts slowly. That does not matter for a level hover; the yaw loop controls rate only.
- Start on **I2C**, because the emulator already has TWIHS. A 1 kHz loop reading 12 bytes fits easily at 400 kHz or 1 MHz. SPI can come later and needs a new SPI peripheral model.
- The BNO055 model stays. Its Euler angles, filled from the true physics state, make a useful check on my estimator.

Datasheet: `reference/icm42688/DS-000347-ICM-42688-P-v1.6.pdf` (110 pages). It is revision 1.6 from a third-party mirror; TDK's current revision is 1.8, which their site only serves through a browser. The register map is chapter 13 and the register descriptions are chapter 14.

The board bought for it is TDK's own evaluation board, the EV_ICM-42688-P. Its user guide is `reference/icm42688/AN-000488-EV_ICM-42688-P-User-Guide-v1.2.pdf`, and [`quad10.md`](quad10.md#the-gyro-board) has its pinout, jumper settings and the wiring to the Xplained board.

The register details below are checked against the datasheet:

- I2C address `0x68` (`0x69` if AP_AD0 is high)
- `WHO_AM_I` (`0x75`) = `0x47`
- `PWR_MGMT0` (`0x4E`, resets to `0x00` with both sensors off): GYRO_MODE and ACCEL_MODE turn the sensors on
- `GYRO_CONFIG0` (`0x4F`) / `ACCEL_CONFIG0` (`0x50`): full-scale range and output rate
- `ACCEL_DATA_X1` (`0x1F`) through `GYRO_DATA_Z0` (`0x2A`): 12 bytes, 16-bit, big-endian by default; `TEMP_DATA1` (`0x1D`) sits just before them
- `REG_BANK_SEL` (`0x76`): selects the register bank; everything above is in bank 0
- Default ranges: gyro ±2000 °/s at 16.4 LSB per °/s; accelerometer ±16 g at 2048 LSB per g

### Units: what the gyro measures

A gyro measures rotation rate, and the datasheet gives it in degrees per second (°/s). The chip does not hand over that number directly.

- **What the chip gives:** a raw signed 16-bit count per axis, from −32768 to +32767.
- **How to convert:** divide by the sensitivity for the selected range. At the default ±2000 °/s range that is 16.4 counts per °/s, so a raw reading of 328 means 328 / 16.4 = 20 °/s.
- **What it is not:** an angle. To get an angle, add up rate × time. That is the estimator's job, with the accelerometer correcting the drift.

Where the numbers come from:

- **±2000 °/s** is the chip's full-scale range: the fastest rotation it can report. It is the power-on default, set by the GYRO_FS_SEL field (bits 7–5) of `GYRO_CONFIG0` (`0x4F`); datasheet section 14.37, page 77.
- **16.4 counts per °/s** is the sensitivity the datasheet lists for that range, in Table 1 (section 3.1, page 11). The datasheet calls a count an LSB.
- **The two are linked.** A signed 16-bit number has 32768 counts on each side of zero, and those are spread over 2000 °/s: 32768 / 2000 = 16.384, which the datasheet rounds to 16.4.
- **328** is not from the datasheet. It is a made-up example reading, picked because 328 / 16.4 comes out as a round 20 °/s.

The sensitivity changes with the range, by the same division:

| GYRO_FS_SEL | Range | Sensitivity (counts per °/s) | 32768 / range |
|---|---|---|---|
| 0 (default) | ±2000 °/s | 16.4 | 16.384 |
| 1 | ±1000 °/s | 32.8 | 32.768 |
| 2 | ±500 °/s | 65.5 | 65.536 |
| 3 | ±250 °/s | 131 | 131.072 |

A smaller range gives finer steps but clips at faster rotation. The chip has four more ranges below these, down to ±15.625 °/s, which are too small for a drone. ±2000 °/s is the usual choice for a quad.

Units in the code:

- **The driver converts once.** Everything after it sees a physical unit, never raw counts.
- **The `flight` crate uses degrees and °/s** in its interface and gains. Those are the numbers read in logs and changed when tuning ("target 200 °/s").
- **The `physics` crate uses radians and rad/s**, which the equations and trigonometric functions need. The conversion happens at the boundary between the two.

### Flight code: one `no_std` crate, two hosts

The flight code is a `no_std` Rust crate with no hardware access. It takes sensor values and a time step, and returns four motor commands. Everything hardware-specific stays outside it.

| Crate | Runs on | Contents |
|---|---|---|
| `drone/flight/` | both (`no_std`) | estimator, PIDs, mixer, arming and safety logic |
| `drone/physics/` | host | rigid-body quadcopter model, motors, sensor noise |
| `drone/sim/` | host | `flight` + `physics` in a loop; writes CSV for plots |
| `firmware/drone-rs/` | emulator, later the board | `flight` + ICM-42688 driver over TWIHS + PWM output |

The emulator uses the same `physics` crate as its motion source, so both hosts fly the same model.

Open point: `f32` throughout the `flight` crate. The Cortex-M7 FPU handles it and the emulator already supports hard-float.

## Control design

A cascaded controller, one per axis:

- **Outer angle loop** (roll, pitch): angle error → target rate. P only to start.
- **Inner rate loop** (roll, pitch, yaw): rate error → torque command. PID.
- **Mixer**: throttle plus the three torque commands → four motor commands, X layout.
- **Estimator**: complementary filter. The gyro gives the angle short-term, the accelerometer's gravity direction corrects the drift.

Rates: inner loop and estimator at 1 kHz, outer loop at 250 Hz.

### Inner and outer loops

The two loops are controllers stacked on top of each other. The output of the outer one is the target of the inner one.

- **Outer loop (angle):** asks "how far am I from the angle I want?" It compares the target angle (level, so 0°) with the estimated angle and outputs a *target rotation rate*. For example, tilted 10° to the right gives "rotate left at 40 °/s".
- **Inner loop (rate):** asks "how far am I from the rotation rate I want?" It compares that target rate with the gyro's measured rate and outputs a *torque command*, which the mixer turns into motor speed differences.

The chain for one axis:

```
target angle ─►(−)─► × Kp ─► target rate ─►(−)─► rate PID ─► torque ─► mixer ─► motors
               ▲                           ▲
        estimated angle                gyro rate
        (estimator)                    (direct)
```

`× Kp` is the outer loop's whole controller: a PID with only the P term. `(−)` is a subtraction, target minus measurement.

Driving a car works the same way. The outer loop is the driver deciding "I'm left of the lane centre, so I should be turning right a little". The inner loop is the driver's hands moving the wheel until the car is actually turning at that rate.

Why split it in two:

- **The motors control rate, not angle.** A thrust difference produces torque, which changes how fast the drone rotates. The angle only follows later. The inner loop controls the thing the motors directly affect, so it can react to a gust before the angle has moved much.
- **The gyro measures rate directly.** The inner loop uses a fast, clean measurement with no estimator in the way. The angle is a derived quantity, slower and noisier.
- **Tuning is separable.** Tune the rate loop first until the drone follows rate commands crisply, then add the angle loop on top. With the inner loop doing the hard work, the outer loop usually needs only a P term.

Why the rates differ: the inner loop must be faster than the thing it controls, which is why it runs at 1 kHz. The outer loop only has to be slower than the inner loop, so the inner loop has time to reach each new target before the target changes again. 250 Hz is plenty for that.

Yaw has only the inner loop. Without a magnetometer there is no trustworthy heading angle to hold, so the drone just controls how fast it yaws.

### The estimator

The outer angle loop needs the drone's current roll and pitch angle, and no sensor measures that directly. The estimator works the angle out from the two sensors that are available, each of which is flawed in a different way. It is built in Stage 2; Stage 1 does not need it, because the rate loop uses the gyro rate directly.

The two sources:

- **Gyro:** adding up rate × time gives the angle. This is smooth and responds instantly, but every small error in the rate is added up too, so the angle slowly drifts away from the truth. After a minute it can be off by several degrees.
- **Accelerometer:** when the drone is not accelerating, the only thing it feels is gravity. The direction of gravity shows which way is down, and from that come roll and pitch. This never drifts, but it is noisy (motor vibration) and wrong whenever the drone accelerates sideways.

The gyro is good over short times and bad over long ones, and the accelerometer is the reverse.

The complementary filter blends the two, trusting each where it is good. Every step, for each of roll and pitch:

```
gyro_angle  = angle + gyro_rate × dt          (predict from the gyro)
accel_angle = angle computed from gravity's direction
angle       = 0.98 × gyro_angle + 0.02 × accel_angle
```

The gyro drives the angle from step to step, and the accelerometer pulls it a small amount towards its own answer each time. The drift is corrected continuously, and the accelerometer's noise is diluted to 2% per step. The name comes from the two weights adding up to 1.

The 0.98 is the tuning knob, and only an illustration: its effect depends on the loop rate. [`Stage2-b.md`](Stage2-b.md#use-a-time-constant-not-α) replaces it with a time constant in seconds, starting at 1 s. Higher trusts the gyro more: smoother, but slower to correct drift. Lower trusts the accelerometer more: quicker correction, but more noise and more disturbance from sideways acceleration.

Limits:

- **Yaw is not covered.** Gravity points straight down, so rotating about the vertical axis does not change what the accelerometer sees. Yaw angle would need a magnetometer.
- **Sustained acceleration fools it.** During a long sideways acceleration the accelerometer's "down" is tilted, and the estimate slowly follows. For a hover this is small.
- **The simple per-axis maths breaks at large tilt.** Treating roll and pitch separately is fine near level. If it proves inadequate, the Mahony filter does the same job with a quaternion.

The simulation helps here because the physics model knows the true angle, which a real drone never does. Plotting the estimator's angle against the true one shows exactly how much it lags, drifts or is disturbed, before the angle loop depends on it.

### Calibration before flight

For a second or two after power-on the controls are locked out. The motors are off, the drone sits still, and the flight code collects sensor readings. Real flight controllers do the same; it is usually called calibration or the pre-arm check.

The readings are used for three things:

- **Starting angle.** The accelerometer readings are averaged and the estimator is set from the average. Without this the estimator starts from a single noisy reading, which in the Stage 2b runs was 1.4° off and took a few seconds to recover. Averaging 1000 readings cuts the noise by about 30 times, to about 0.05°.
- **Gyro bias.** A still drone has a true rate of zero, so whatever the gyro reads on average is its bias. It is stored and subtracted from every later reading. Without this, a bias of 1 °/s makes the rate loops slowly turn the drone, because they trust the gyro.
- **A stillness check.** If the gyro readings vary too much during the window, someone is moving the drone. The readings are discarded and the window starts again. Arming is not allowed until one clean window is in.

Three points about it:

- **The drone has to be still, not level.** On a slope, the averaged accelerometer correctly reports the slope as the starting angle.
- **The motors are off, so there is no vibration.** These are the cleanest readings the sensors ever give.
- **The bias drifts as the chip warms up.** After a few minutes the value measured at start-up is no longer exact. The estimator's slow pull towards the accelerometer covers that for roll and pitch, which is a reason to keep both mechanisms.

It is built in two steps:

1. **A simple version with the angle loop,** the third Stage 2 item. That is the first point where the estimate steers the drone, so a starting error would show as a lurch at the start of every run. Each simulated run begins with one second of motors off, calibrating, and then the controls are released.
2. **The full arming logic in Stage 3.** Disarmed means motors off and calibration allowed; arming is refused until calibration has succeeded.

In the code:

- **`flight` crate:** a `Calibration` type that is fed gyro and accelerometer readings and reports "still collecting", "failed, the drone moved" or "done", with the starting attitude and the three gyro biases.
- **Estimator:** a way to set the starting attitude directly. The first-reading rule stays as the fallback.
- **Simulation:** the body sits still with the motors off during the window. It is held at its centre, so it does not fall.

## Physics model

- Motor: command → rotor speed with a first-order lag (time constant around 30–50 ms).
- Thrust proportional to rotor speed squared. Yaw torque from rotor drag, with alternate rotors spinning opposite ways.
- Roll and pitch torque from thrust differences times the arm length.
- Rigid body: Euler's rotation equations, attitude held as a quaternion, gravity, and (in stage 3) translation.
- Sensor model: gyro and accelerometer readings with noise, bias and one sample of delay.
- Disturbances: a torque kick or steady wind, to test recovery.

The parameters (mass, inertia, arm length, thrust coefficient, motor lag) are guesses until parts are chosen. Keep them in one struct so a real airframe can be dropped in.

### How accurate the model is

The model is accurate enough to get the code right and a first set of gains in the right range. It is not accurate enough to skip tuning on the real drone. That is the normal result for flight-controller simulation.

What transfers to the real drone:

- **Code correctness:** signs, axis conventions, the mixer, unit conversions, anti-windup, arming logic and timing. Most first-flight crashes come from these, and the simulation tests them exactly.
- **Controller structure:** the cascaded loops, loop rates and estimator design stay the same.
- **Behaviour under stress:** saturation, a weak motor, recovery from a large tilt and sensor dropouts can all be tested safely.
- **Rough gains:** with the physical parameters measured (see the table below), simulated gains are usually within a factor of two or so of the final ones, which is a flyable starting point.

What does not:

- **Final gains.** These always need adjusting in flight.
- **Vibration.** A real frame shakes, and propeller vibration reaches the gyro as noise much larger than the sensor's own. This is the biggest gap. It drives the D-term filtering and limits how high the gains can go, and a rigid-body model has none of it.
- **Aerodynamics.** The model has thrust proportional to speed squared and nothing else. Real propellers lose thrust in forward flight, interact with each other and behave differently near the ground. For a hover this matters little.
- **Motor and ESC detail.** A first-order lag is a decent approximation, but real motors speed up faster than they slow down, respond differently at low and high throttle, and sag as the battery drains.

The model's equations are fine for a hover. Its accuracy depends almost entirely on four numbers:

| Parameter | How to get it |
|---|---|
| Mass | weigh it |
| Inertia | hang the drone from two strings and time its swing (a bifilar pendulum), or calculate from component masses and positions |
| Thrust coefficient | motor manufacturer's thrust table, or a kitchen scale and a test stand |
| Motor time constant | log the motor speed response to a step command, or use a typical 30–50 ms |

With those measured, a hover simulation is good. Without them, it is only qualitatively right.

Three additions make the simulation more honest. All belong to the last item of Stage 1:

- **Noise well above the datasheet figure,** to stand in for vibration.
- **Loop delay:** one or two samples of sensor and computation delay.
- **Margin testing:** check that the gains still work with inertia and thrust each off by ±30%. Gains that only work on the exact model will not survive contact with hardware; gains that tolerate that spread usually do.

At first flight, expect the drone to be controllable but not well tuned, with a few sessions of adjustment to bring it in. A real seesaw rig (see [`Stage1-b.md`](Stage1-b.md)) is the bridge: it allows the simulated and real response to be compared on one axis, the model's parameters corrected and the gains retuned before anything leaves the ground.

## Hardware options and budget

The first aircraft has since been chosen and most of its parts bought: a Mark4 V2 10 inch quad, described in [`quad10.md`](quad10.md). This section is the earlier survey that led there. It records what a budget of $2000 could buy, and what a drone able to lift 20 lb (about 9 kg) would involve. The prices and sizes are rough figures from memory, not quotes; check current listings before buying.

One option is worked out in detail, with a parts list and looked-up prices: the Tarot T15 and T18 folding octocopters, in [`octocopter.md`](octocopter.md).

### What the budget must also cover

These are needed whatever the drone's size:

| Item | Rough cost |
|---|---|
| Radio transmitter and receiver | $100–250 |
| Battery charger and power supply | $100–200 |
| Spare propellers, wiring, connectors, mounting hardware | $100–150 |

That leaves roughly $1400–1700 for the aircraft. The flight controller is the SAME70 board plus a sensor breakout, which is cheap next to everything else.

### Three options

| | Small trainer | Mid-size lifter | Bare-bones 20 lb lifter |
|---|---|---|---|
| Payload | under 0.5 kg | 2–4 kg (4–9 lb) | about 9 kg (20 lb) |
| Take-off weight | 1–1.5 kg | 5–8 kg | 20–25 kg |
| Layout | quad, 7–10 inch props | quad or hexa, 15–18 inch props | quad, about 30 inch props |
| Battery | 4S | 6S | 12S |
| Aircraft cost | $250–400 | $700–1200 | $1500–2000 or more |
| Fits $2000 | easily, with spares | yes, with spares | barely, with none |

### The 20 lb lifter

Rough sizing:

- **Take-off weight:** the frame, motors and batteries needed to carry 9 kg weigh 10–15 kg themselves, so about 20–25 kg in total.
- **Thrust:** about twice the weight to stay controllable, so 40–50 kg. On a quad that is 10–12 kg per motor, with propellers around 28–32 inches.
- **Battery:** 12S (about 44 V) or more, for perhaps 10–20 minutes with the load.

It is possible near $2000 because agricultural drone parts have become cheap. Integrated motor, ESC and propeller units rated around 10–12 kg of thrust sell for somewhere near $150–200 each, so four cost $600–800. A frame adds a few hundred, and one large 12S battery adds $400–500 or more.

That lands at the budget limit, with compromises that matter at this weight:

- **Four motors, not six.** A quad cannot stay up if one motor fails; a hexacopter or octocopter can.
- **One battery.** A single flight per charge, and a charger for a pack that size is itself expensive.
- **No spares.** One crash of a 20 kg machine can break more than the remaining budget covers.
- **Thin thrust margin.** At full load the motors run near their limit, which leaves the controller little room to correct.

Two more things change at this size:

- **Safety.** Propellers of that size can cause severe injury. Testing needs tethers, distance and a kill switch.
- **Regulation.** In the US, the FAA's limit for small unmanned aircraft is 55 lb (25 kg) total take-off weight including payload, so this design sits at the edge. Check the current rules before buying anything.

### What size means for the flight code

The control code is the same at every size. Only the physics parameters and the gains change.

- **A large drone is slower to rotate.** More inertia means slower dynamics, which is gentler on loop rate and sensor speed than a small quad.
- **Motor lag is larger.** Big propellers take longer to change speed, which limits how aggressive the gains can be. The motor time constant goes from 30–50 ms to perhaps 100 ms or more.
- **Six or eight motors need a larger mixer table,** built on the same idea as the X mixer.
- **The parameters struct covers all three options.** Swapping it shows how the same controller behaves on each before any hardware is bought.

### Recommended order

1. **Small trainer first,** about $350. Crashes while tuning a first flight controller are close to certain, and at this size they cost a few dollars in propellers.
2. **Mid-size lifter next,** with the remaining $1200–1400. A 3–4 kg payload covers most practical uses, the parts are common, and there is money left for spares and a second battery.
3. **The 20 lb lifter as a third build,** with its own budget of $3000 or more, after the code has proven itself on the smaller two.

## Phases

### Stage 1 — Host simulation, one axis

- [x] `drone/flight/`: PID with output limits and integrator anti-windup; unit tests — explained in [`Stage1-a.md`](Stage1-a.md)
- [x] `drone/physics/`: one axis, two motors (a seesaw) — explained in [`Stage1-b.md`](Stage1-b.md)
- [x] `drone/sim/`: rate loop holding a target rate; CSV output and a plot script — explained in [`Stage1-c.md`](Stage1-c.md)
- [x] Add sensor noise and loop delay (motor lag is already in the seesaw); retune, and check the gains still work with inertia and thrust off by ±30% (see [How accurate the model is](#how-accurate-the-model-is)) — explained in [`Stage1-d.md`](Stage1-d.md)

### Stage 2 — Host simulation, self-levelling attitude

- [x] Three-axis rigid body with four motors and the X mixer — explained in [`Stage2-a.md`](Stage2-a.md)
- [x] Complementary filter; compare its angles with the true angles from the physics — introduced in [The estimator](#the-estimator), explained in full in [`Stage2-b.md`](Stage2-b.md)
- [x] Angle loop around the rate loop on roll and pitch; rate loop on yaw; a simple start-up calibration first (see [Calibration before flight](#calibration-before-flight)) — explained in [`Stage2-c.md`](Stage2-c.md)
- [x] Disturbance tests: recovers to level after a kick, from a 30° start, and with one weak motor — explained in [`Stage2-d.md`](Stage2-d.md)

### Stage 3 — Host simulation, hover

- [x] Full 6DOF: gravity, translation, ground — explained in [`Stage3-a.md`](Stage3-a.md)
- [x] Fixed hover throttle, then throttle compensated for tilt — explained in [`Stage3-b.md`](Stage3-b.md)
- [ ] Arming, motor limits, and a cut-off when tilted past a safe angle; arming is refused until [calibration](#calibration-before-flight) has succeeded — explained in [`Stage3-c.md`](Stage3-c.md)
- [ ] Fly it by hand with a gamepad (a Logitech F310, already owned): a `drone/sim/` program that runs in real time, reads the sticks and fills `Targets` from them
  - Sticks: left stick up and down is throttle, left and right is yaw rate; right stick is pitch and roll angle. Scale full stick to a chosen limit, about ±30° and ±120 °/s, with a small dead zone round the centre
  - Buttons: one to arm, one to disarm, which is also the motor kill
  - The left stick springs back to the centre, unlike the throttle stick of a radio. So the centre is hover throttle, and the stick adds to or takes from it
  - Reading it: the `gilrs` crate, in the `sim` crate only. The `flight` crate stays as it is and sees only `Targets`. Set the switch on the back of the pad to X
  - Needs a run that keeps pace with the clock, and a live view in place of the CSV and plot: a window that draws the drone in 3D and turns it with the body's quaternion
  - Pacing: the window draws 60 pictures a second, and the simulation runs the steps that fit in between, about 17 of 1 ms each. No sleeping for 1 ms at a time
  - Drawing: the `macroquad` crate, in the `sim` crate only. Start with a drone made of boxes: two arms, four motors, and a mark for the nose. A model file of the real frame can replace the boxes later
  - The body's axes are x forward, y right, z down; the window's are y up. One conversion, in one place
  - On screen as text: the state, the four targets and the four motor commands
  - `--stage` works here as in the quad program, so any earlier stage can be flown by hand and watched. The presets move out of `src/bin/quad.rs` into a place both programs share
    - `--stage 2a` and `2b`: the drone on its ball joint, rate loops only. The right stick asks for a roll and a pitch **rate**, not an angle, and letting go does not level it. For `2b`, draw the estimator's attitude as a second, see-through drone over the true one
    - `--stage 2c` and `2d`: still on the joint, with the angle loops. Letting go of the stick levels it. A button gives it a kick
    - `--stage 3a` on: it flies, falls and lands

### ✅ Milestone A: stable self-levelling hover in the host simulation

### Stage 4 — Emulator peripherals

- [ ] ICM-42688-P model as an `I2cDevice` (`src/icm42688.rs`), alongside the BNO055
- [ ] PWM peripheral model (PWM0 at `0x40020000`, check the SVD): period and duty registers for four channels, readable by the motion source
- [ ] Physics motion source: reads the four duties, steps `drone/physics/` from the cycle counter, fills the ICM-42688 registers (and the BNO055's Euler registers)
- [ ] `--drone-log=` CSV of true attitude and motor commands; a switch to inject a disturbance

### Stage 5 — Firmware in the loop

- [ ] `firmware/drone-rs/`: ICM-42688 driver over TWIHS0, 1 kHz loop from a TC interrupt, PWM output
- [ ] Run the `flight` crate unchanged; compare the emulator's log with the host simulation's
- [ ] Measure the loop's execution time with `DWT_CYCCNT`

### ✅ Milestone B: the real firmware hovers the simulated drone on the emulator

### Later

- SPI peripheral model and an SPI path for the ICM-42688
- Altitude hold, with a lidar rangefinder and a barometer fused together. [`altitude.md`](altitude.md) has the design and suggested parts
- Receiver input for stick commands on the real drone. The gamepad is wired USB and stays with the simulation; the firmware fills the same `Targets` from the receiver's channels
- Choosing real parts (see [Hardware options and budget](#hardware-options-and-budget)) and replacing the guessed physics parameters
