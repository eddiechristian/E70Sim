# Stage 2d the disturbance tests: the pictures

The quad pushed, started far from level and flown with one weak motor. Explained in [`Stage2-d.md`](../../../Stage2-d.md#results).

Regenerate with `./stage2-d.sh` from `drone/sim/`. Every run is 5 s: calibration until 1 s, then flying. The first twelve have a level target throughout (`--target 0`), so that the only thing happening is the disturbance. The gains are the defaults, which now have an I term in the rate loops: `Ki` 0.003 on roll and pitch and 0.02 on yaw. The runs named `no_i` have both set to 0, which is the controller as it was after Stage 2c.

Two of the runs are "real": `--noise 2 --delay 2 --accel-noise 0.5 --gyro-bias 1`, as in Stage 2c.

## The kick

A torque on the body for 0.05 s, starting at 2.0 s. It is shaded red in the pictures.

| Picture | Kick | Largest roll | Largest pitch | Within 1° of level after | Motor commands | What it shows |
|---|---|---|---|---|---|---|
| `01_kick_roll` | 0.3 N·m about roll | 2.03° | 0° | 0.18 s | 0.62 to 0.78 | the standard kick: knocked 2°, back in a fifth of a second |
| `02_kick_pitch` | 0.3 N·m about pitch | 0° | 2.03° | 0.18 s | 0.62 to 0.78 | the same about pitch, figure for figure |
| `03_kick_compound` | 0.3 N·m about each | 2.03° | 2.03° | 0.19 s | 0.54 to 0.86 | each axis as if kicked alone. Only front-left and back-right move; the other two stay at 0.70 |
| `04_kick_compound_hard` | 1.2 N·m about each | 10.6° | 10.4° | 0.65 s | 0.10 to 1.00 | two motors do the work of four and reach their limits. It still recovers |
| `05_kick_yaw` | 0.1 N·m about yaw | 0° | 0° | | 0.59 to 0.81 | the yaw rate reaches 10.4 °/s and returns; roll and pitch are not disturbed |
| `06_kick_hard` | 2.4 N·m about roll | 24.3° | 0° | 0.52 s | 0.40 to 1.00 | the motors are held at their limit, so the angle is far more than eight times that of `01` |
| `07_kick_roll_real` | 0.3 N·m about roll, real | 2.05° | 0.06° | 0.18 s | 0.56 to 0.83 | noise, delay and bias make no difference to the recovery |

## The 30° start

The body sits tilted through the calibration, and the target is level when the motors start at 1.0 s.

| Picture | Start | Within 1° of level after | Motor commands | What it shows |
|---|---|---|---|---|
| `08_start_30` | roll 30° | 0.29 s | 0.45 to 0.95 | straight back to level; it goes 0.8° past before it settles |
| `09_start_30_both` | roll 30° and pitch 30° | 0.44 s | 0.19 to 1.00 | all four commands differ, and one reaches its limit. The nose ends 7.5° from where it began |

## The weak motor

Front-right gives 10% less thrust, from the start.

| Picture | Largest roll and pitch | Roll and pitch at the end | Yaw rate at the end | Nose turned by | What it shows |
|---|---|---|---|---|---|
| `10_weak_motor_no_i` | 1.12°, −1.11° | 1.11°, −1.11° | −0.93 °/s | −3.7° and still turning | without an I term: three flat lines beside zero, on roll, pitch and yaw rate |
| `11_weak_motor` | 0.81°, −0.81° | 0°, 0° | 0 °/s | −0.5° | with it: the same three lines bend back onto zero, and the front-right command settles above the other three |
| `12_weak_motor_real` | 0.74°, −0.90° | 0.07°, 0° | −0.10 °/s | −0.3° | the same with noise, delay and bias |

## What the I terms cost a step

A step with four good motors, from 1.5 s to 3.5 s, with and without the I terms.

| Picture | Step | Rise | Overshoot | Settle | Off target at the end of the step | What it shows |
|---|---|---|---|---|---|---|
| `13_step_no_i` | roll 20°, no I | 0.284 s | 0.1% | 0.326 s | 0° | the Stage 2c step |
| `14_step` | roll 20° | 0.250 s | 0.5% | 0.277 s | 0.07° | a little quicker to arrive, then about 0.4° short of the target for a while, creeping back |
| `15_yaw_step_no_i` | yaw 50 °/s, no I | 0.193 s | 2.6% | 0.212 s | | the Stage 2a yaw step |
| `16_yaw_step` | yaw 50 °/s | 0.187 s | 8.4% | 0.457 s | | the yaw I term makes the overshoot three times as large |

The pictures have the six panels of [Stage 2c](../stage2-c/README.md), with the calibration shaded grey.

The columns of the first three tables are printed by `measure_disturbance.py`, those of the roll steps by `measure_angles.py` and those of the yaw steps by `measure_quad.py`:

- **Largest roll, largest pitch:** the furthest the true angle gets from level, counted from the start of the kick, or from the moment the motors start when there is no kick.
- **Within 1° of level after:** from the same moment until roll and pitch both stay within 1° of level.
- **Nose turned by:** the change in the true yaw angle from that moment to the end of the run.

Things to notice:

- **The largest angle comes after the kick is over.** In `01` the red band ends at 2.05 s and the roll peaks at about 2.1 s: the body is still turning while the rate loop brings it to a stop.
- **After a kick the angle dips about 0.5° the other way** and then creeps back to level over a second. That slow tail is the I term giving back what it collected during the kick.
- **In `03` two of the motor lines do not move.** Front-right and back-left sit on the axis of the push and can do nothing about it.
- **In `06` the motor lines have flat tops** at 1.0 and flat bottoms at 0.4, the output limit of 0.3 either side of the throttle.
- **In `10` the motor commands are not equal** even without an I term: the 1.1° of error is what pays for the larger command on the weak motor.
- **In `11` the nose still ends 0.5° to the left.** The yaw I term stops the turning but does not undo what happened before it took hold.
