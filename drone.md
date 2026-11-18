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

Check that a breakout board is in stock before committing to the part; the Bosch BMI270 is the fallback.

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
target angle ─►(−)─► angle P ─► target rate ─►(−)─► rate PID ─► torque ─► mixer ─► motors
               ▲                              ▲
        estimated angle                   gyro rate
        (estimator)                       (direct)
```

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

The 0.98 is the tuning knob, and a starting value, not a tuned one. Higher trusts the gyro more: smoother, but slower to correct drift. Lower trusts the accelerometer more: quicker correction, but more noise and more disturbance from sideways acceleration.

Limits:

- **Yaw is not covered.** Gravity points straight down, so rotating about the vertical axis does not change what the accelerometer sees. Yaw angle would need a magnetometer.
- **Sustained acceleration fools it.** During a long sideways acceleration the accelerometer's "down" is tilted, and the estimate slowly follows. For a hover this is small.
- **The simple per-axis maths breaks at large tilt.** Treating roll and pitch separately is fine near level. If it proves inadequate, the Mahony filter does the same job with a quaternion.

The simulation helps here because the physics model knows the true angle, which a real drone never does. Plotting the estimator's angle against the true one shows exactly how much it lags, drifts or is disturbed, before the angle loop depends on it.

## Physics model

- Motor: command → rotor speed with a first-order lag (time constant around 30–50 ms).
- Thrust proportional to rotor speed squared. Yaw torque from rotor drag, with alternate rotors spinning opposite ways.
- Roll and pitch torque from thrust differences times the arm length.
- Rigid body: Euler's rotation equations, attitude held as a quaternion, gravity, and (in stage 3) translation.
- Sensor model: gyro and accelerometer readings with noise, bias and one sample of delay.
- Disturbances: a torque kick or steady wind, to test recovery.

The parameters (mass, inertia, arm length, thrust coefficient, motor lag) are guesses until parts are chosen. Keep them in one struct so a real airframe can be dropped in. The gains will need retuning on hardware; the simulation proves the structure, signs, mixer and timing.

## Phases

### Stage 1 — Host simulation, one axis

- [ ] `drone/flight/`: PID with output limits and integrator anti-windup; unit tests — explained in [`Stage1-a.md`](Stage1-a.md)
- [ ] `drone/physics/`: one axis, two motors (a seesaw)
- [ ] `drone/sim/`: rate loop holding a target rate; CSV output and a plot script
- [ ] Add motor lag, sensor noise and loop delay; retune

### Stage 2 — Host simulation, self-levelling attitude

- [ ] Three-axis rigid body with four motors and the X mixer
- [ ] Complementary filter; compare its angles with the true angles from the physics — explained in [The estimator](#the-estimator)
- [ ] Angle loop around the rate loop on roll and pitch; rate loop on yaw
- [ ] Disturbance tests: recovers to level after a kick, from a 30° start, and with one weak motor

### Stage 3 — Host simulation, hover

- [ ] Full 6DOF: gravity, translation, ground
- [ ] Fixed hover throttle, then throttle compensated for tilt
- [ ] Arming, motor limits, and a cut-off when tilted past a safe angle

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
- Altitude hold (needs a barometer or range sensor)
- Receiver input for stick commands
- Choosing real parts and replacing the guessed physics parameters
