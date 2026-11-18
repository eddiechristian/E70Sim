# Stage 2a rate loops on the quad: the pictures

A 50 °/s step on each axis of the quad, with the other two axes held at 0. Explained in [`Stage2-a.md`](../../../Stage2-a.md#in-the-simulation).

Regenerate with `./stage2-a.sh` from `drone/sim/`. All five use the default gains: roll and pitch `Kp` 0.0014 and `Kd` 0.00003, yaw `Kp` 0.01, and a D-term filter of 0.01 s. The last two add `--noise 2 --delay 2` with the default seed.

| Picture | Rise | Overshoot | Settle | Other axes | Motor jitter | Motor commands | What it shows |
|---|---|---|---|---|---|---|---|
| `01_roll_step` | 0.112 s | 0.9% | 0.128 s | 0 | none | 0.63 to 0.77 | a clean step; pitch and yaw do not move |
| `02_pitch_step` | 0.112 s | 0.9% | 0.128 s | 0 | none | 0.63 to 0.77 | the same as roll, as a symmetric body should give |
| `03_yaw_step` | 0.193 s | 2.6% | 0.212 s | 0 | none | 0.40 to 1.00 | yaw is slower, and two motors reach full power at each edge of the step |
| `04_roll_step_noise_delay` | 0.109 s | 0.8% | 0.123 s | 0.58 °/s | 0.024 | 0.56 to 0.84 | the response is unchanged; the motor commands are noisy |
| `05_yaw_step_noise_delay` | 0.190 s | 2.7% | 0.209 s | 0.60 °/s | 0.024 | 0.36 to 1.00 | the same for yaw |

Each picture has four panels: the roll, pitch and yaw rates (target, true rate, and the measured rate as a pale line), then the four motor commands. In the yaw pictures without noise only two motor lines show, because each diagonal pair gets the same command and one line hides the other.

The columns, measured on the true rate for the step up to 50 °/s (`measure_quad.py` prints them):

- **Rise:** time from the step until the rate first reaches 45 °/s.
- **Overshoot:** how far the rate passes 50 °/s, as a percentage.
- **Settle:** time from the step until the rate stays within 2.5 °/s of 50.
- **Other axes:** the largest rate reached by either of the two axes held at 0.
- **Motor jitter:** standard deviation of the front-left motor command over the last 0.3 s of the step, when it should be still.
- **Motor commands:** the smallest and largest command of the whole run.

The motor jitter of 0.024 is about twice the 0.013 of the chosen gains on the seesaw in Stage 1d. The yaw gain is the likely cause: it is seven times the roll gain, so it passes seven times as much gyro noise to the motors. It is a first guess and has not been tuned.
