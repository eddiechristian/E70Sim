# Stage 3a the free body: the pictures

The quad with a weight, a position and a ground under it. Explained in [`Stage3-a.md`](../../../Stage3-a.md#results).

Regenerate with `./stage3-a.sh` from `drone/sim/`. Every run is 12 s:

- **0 to 1 s:** the calibration, standing on the ground with the motors off.
- **1 to 3 s:** the climb, at a throttle of 0.76.
- **3 s on:** the hover command, 0.7002, which holds the drone's weight.
- **5 to 9 s:** the tilt, in the runs that have one: nose down by 10° unless the name says otherwise.

The body has a mass of 1.2 kg and a drag of 0.4 N per m/s, and the estimator's time constant τ is 10 s, unless the name says otherwise.

"Real" means `--noise 2 --delay 2 --accel-noise 0.5 --gyro-bias 1`, as in Stage 2. "Bias left" means a gyro bias of 0.1 °/s that the calibration has not removed (`--no-calibration --gyro-bias 0.1`).

## Hovering

| Picture | Height at 3 s, 5 s and 12 s | Over the ground at 12 s | What it shows |
|---|---|---|---|
| `01_hover` | 2.5 m, 6.1 m, 9.6 m | where it took off | it leaves the ground at 1.10 s and climbs. The hover command at 3 s does not stop the climb, it only stops it speeding up: the drone is still rising at 0.12 m/s at the end |
| `12_hover_real` | the same | 1.4 m away, moving at 0.34 m/s | noise, delay and a bias the calibration removed. It ends tilted 0.6° and 0.8°, and slides |
| `13_hover_bias_left` | the same | 2.3 m away, moving at 0.47 m/s | a bias of 0.1 °/s left in: tilted 0.8° on both axes and still tilting further |
| `14_hover_bias_left_tau_1` | the same | 0.9 m away, moving at 0.07 m/s | the same with τ = 1 s: tilted 0.08° |
| `15_hover_bias_left_tau_30` | the same | 2.6 m away, moving at 0.54 m/s | the same with τ = 30 s: tilted 1.0° and still tilting further |

## Tilting

| Picture | Asked for | True angle reached | Largest error of the estimate | Angle left after | Speed after 4 s | What it shows |
|---|---|---|---|---|---|---|
| `02_tilt_pitch` | pitch −10° | −12.0° | 2.0° | −2.1° | 4.2 m/s | the chosen τ: the drone flies north, 2° steeper than asked |
| `03_tilt_roll` | roll 10° | 12.0° | 2.0° | 2.1° | 4.2 m/s | the same to the east, figure for figure |
| `04_tilt_tau_1` | pitch −10° | −21.6° | 11.7° | 12.0° the other way | 7.1 m/s | τ = 1 s, as in Stage 2: the estimate sits on the target while the drone tilts twice as far. Levelling off, the nose comes 12° up |
| `05_tilt_tau_3` | pitch −10° | −15.6° | 5.6° | −5.0° | 5.2 m/s | τ = 3 s: half-way between |
| `06_tilt_tau_30` | pitch −10° | −10.7° | 0.7° | −0.9° | 3.9 m/s | τ = 30 s: nearly true |
| `07_tilt_no_drag` | pitch −10° | −13.6°, still growing | 3.8° | −4.1° | 7.7 m/s, still growing | no drag: nothing brings the estimate back, and nothing stops the climb either. It ends 29 m up |
| `08_tilt_drag_0.8` | pitch −10° | −11.2° | 1.2° | −1.2° | 2.6 m/s | twice the drag: the accelerometer comes right sooner |
| `09_tilt_real` | pitch −10° | −12.4° | 2.5° | −2.6° | 4.3 m/s | real sensors: nearly the clean figures |
| `10_tilt_20_tau_1` | pitch −20° | −43.6° | 23.7° | | | τ = 1 s and a steeper tilt: it loses its lift and reaches the ground at 9.0 s, coming down at 4.5 m/s and moving forward at 13.3 m/s |
| `11_tilt_20` | pitch −20° | −24.0° | 4.1° | −4.2° | 8.2 m/s | the same with the chosen τ: it flies on, 0.6 m lower after the tilt than before it |

Each picture has six panels, with the calibration shaded:

1. **Height.**
2. **How far north and east** of where it started.
3. **Speed:** north, east and the climb.
4. **Roll angle** and 5. **pitch angle:** the target, the true angle and the controller's estimate.
6. **The four motor commands,** with the throttle asked for as a dashed line.

The columns are printed by `measure_flight.py`:

- **True angle reached:** the true angle furthest from level during the tilt.
- **Largest error of the estimate:** the largest distance of the estimate from the true angle during the tilt.
- **Angle left after:** the true angle furthest from level from 0.5 s after the tilt to the end of the run, when the target is level.
- **Speed after 4 s:** the speed over the ground at the end of the tilt.

Things to notice:

- **The height never turns level.** In every picture the height line eases off after 3 s but keeps rising. The throttle sets a force, not a height.
- **In `04` the orange line and the blue line part company.** The estimate stays on −10° and the true pitch swings out to −21.6°. In `02` the two stay within 2° of each other.
- **After the tilt in `04` the true pitch rises above zero** while the estimate reads level. The drone is slowing down, and the accelerometer says it is still nose down.
- **A tilt costs height.** In `02` the drone gains 1.95 m during the tilt; level, in `01`, it gains 2.82 m over the same four seconds.
- **In the two `bias_left` pictures with a long τ the angle lines drift away from zero** and the north and east lines curve. With τ = 1 s they stay flat.
- **In `10` the height line comes down to zero** and every line stops moving: the ground holds a drone that has landed on it.
