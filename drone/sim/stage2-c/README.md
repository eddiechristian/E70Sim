# Stage 2c the whole loop on the quad: the pictures

The quad flown by the `Controller`: a second of calibration with the motors off, then the angle loops, the rate loops and the mixer. Explained in [`Stage2-c.md`](../../../Stage2-c.md#results).

Regenerate with `./stage2-c.sh` from `drone/sim/`. Every run is 5 s: calibration until 1 s, then level, a step from 1.5 s to 3.5 s, and level again. The step is 20° of roll unless the name says otherwise. The gains are the defaults: angle `Kp` 6, rate limit 200 °/s, and the Stage 2a rate gains.

Three of the runs are "real": `--noise 2 --delay 2 --accel-noise 0.5 --gyro-bias 1`. That is gyro noise of 2 °/s, a delay of two steps, accelerometer noise of 0.5 m/s² and a gyro bias of 1 °/s on every axis, with the default seed.

| Picture | Rise | Overshoot | Settle | Largest rate asked for | Off target at the end of the step | What it shows |
|---|---|---|---|---|---|---|
| `01_roll_step` | 0.284 s | 0.1% | 0.326 s | 120 °/s | 0° | the clean step: the angle rises to 20° with no bump, and the rate draws one hump under it |
| `02_pitch_step` | 0.284 s | 0.1% | 0.326 s | 120 °/s | 0° | the same about pitch, figure for figure |
| `03_yaw_step` | 0.193 s | 2.6% | 0.212 s | | | yaw has no angle loop: a step of 50 °/s in rate, as in Stage 2a. Two motors reach 1.0 for a moment |
| `04_roll_step_real` | 0.283 s | 0.5% | 0.328 s | 120 °/s | 0.10° | noise, delay and bias, calibrated: almost the clean figures |
| `05_roll_step_no_calibration` | 0.353 s | never arrives | never | 121 °/s | 1.23° | the same without calibration: the drone flies about 1° off its target the whole time, and yaws at −1 °/s |
| `06_levels_from_20` | | | | 120 °/s | | real, starting at 20° of roll with a level target: within 1° of level 0.33 s after the motors start |
| `07_angle_kp_3` | 0.675 s | none | 0.856 s | 60 °/s | 0.09° | half the angle gain: more than twice as slow |
| `08_angle_kp_12` | 0.161 s | 18% | 0.344 s | 200 °/s | 0° | twice the angle gain: quicker to rise, but it overshoots and settles no sooner |
| `09_step_45` | 0.315 s | 0.1% | 0.359 s | 200 °/s | 0° | a step of 45°: the rate asked for sits at the 200 °/s limit for part of the way |
| `10_step_45_limit_100` | 0.488 s | none | 0.540 s | 100 °/s | 0° | the same with the limit at 100 °/s: the angle climbs in a straight line, more slowly |

Each picture has six panels, with the calibration shaded:

1. **Roll angle:** the target, the true angle and the controller's estimate.
2. **Roll rate:** what the angle loop asks for, the true rate, and the gyro reading as a pale line.
3. **Pitch angle** and 4. **pitch rate:** the same for pitch.
5. **Yaw rate:** the target, the true rate and the gyro reading.
6. **The four motor commands.**

The columns (`measure_angles.py` prints them, with a few more; the yaw row is from `measure_quad.py`):

- **Rise:** from the step until the true angle first reaches 90% of the target.
- **Overshoot:** how far the angle passes the target, as a percentage of it.
- **Settle:** from the step until the angle stays within 5% of the target.
- **Largest rate asked for:** the largest target the angle loop gave the rate loop during the step.
- **Off target at the end of the step:** the largest distance of the true angle from the target over the step's last 0.5 s.

Things to notice:

- **The motors are off in the shaded second,** and the estimate has no line there: the controller has no estimate until the calibration is done.
- **With calibration the estimate lies on the true angle** from the first moment of flight (`04`). Without it (`05`) the estimate starts 1.4° off, from one noisy accelerometer reading, and the bias then holds it about 1° from the truth. The controller steers the estimate to the target, so it is the true angle that ends up wrong.
- **In `05` the yaw rate sits at −1 °/s.** Yaw has no angle loop to notice. The rate loop sees the biased gyro read zero and is content, so the drone turns steadily. In `04` the calibration has removed the bias and the true yaw rate stays at 0.
- **In `06` the drone sits at 20° through the calibration,** which finds that angle. When the motors start it turns straight back to level, with no jump first.
- **In `09` the dashed line in the rate panel has a flat top** at 200 °/s. In `10` the flat top is at 100 °/s and lasts longer.
- **The pitch panels of the roll runs have a small scale.** In the real runs the true pitch wanders by about 0.1°, from gyro noise reaching the motors.
