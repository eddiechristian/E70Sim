# Stage 3b the throttle: the pictures

A hover throttle that is a little wrong, and more throttle in a tilt. Explained in [`Stage3-b.md`](../../../Stage3-b.md#results).

Regenerate with `./stage3-b.sh` from `drone/sim/`. Every run is the 12 s of [Stage 3a](../stage3-a/README.md): calibration on the ground until 1 s, a climb until 3 s, then the hover command, with a tilt from 5 s to 9 s in the runs that have one. The tilt compensation is on unless the name ends in `off`.

The drone is still climbing when the tilt starts, at 1.3 m/s and slowing. So the figure to compare is the height **gained** over the four seconds of the tilt. Level, it gains 2.82 m.

## The hover throttle

| Picture | Throttle after the climb | Height at 12 s | Climbing at | What it shows |
|---|---|---|---|---|
| `01_hover` | 0.7002, the hover command | 9.6 m | 0.12 m/s, and fading | the Stage 3a hover: the compensation changes nothing when level |
| `02_hover_throttle_high` | 0.7100, which is 1.4% too much | 14.6 m | 0.91 m/s | it never stops climbing |
| `03_hover_throttle_low` | 0.6904, which is 1.4% too little | 4.6 m | −0.65 m/s | the climb turns into a sink at about 7 s |

## The tilt

| Picture | Asked for | Height gained in the tilt | Short of a level flight by | Climbing at the end of the tilt | What it shows |
|---|---|---|---|---|---|
| `04_tilt_10_off` | pitch −10° | 1.95 m | 0.87 m | −0.08 m/s | no compensation: the Stage 3a run |
| `05_tilt_10` | pitch −10° | 2.65 m | 0.17 m | 0.23 m/s | with it: four fifths of the loss is gone |
| `06_tilt_20_off` | pitch −20° | −0.64 m | 3.46 m | −1.32 m/s | no compensation: it ends the tilt lower than it began |
| `07_tilt_20` | pitch −20° | 2.10 m | 0.72 m | −0.11 m/s | with it |
| `08_tilt_30_off` | pitch −30° | −4.85 m | 7.67 m | −3.32 m/s | no compensation: it comes down to 1.3 m during the tilt and reaches the ground at 9.4 s, at 3.1 m/s |
| `09_tilt_30` | pitch −30° | 1.09 m | 1.73 m | −0.74 m/s | with it: it flies on. One motor command touches 1.0 for 6 ms as it levels off |
| `10_tilt_roll_20` | roll 20° | 2.10 m | 0.72 m | −0.11 m/s | the same about roll as `07` about pitch |
| `11_tilt_10_real` | pitch −10°, real | 2.60 m | 0.22 m | 0.20 m/s | noise, delay and bias: nearly the clean figures |
| `12_tilt_30_tau_1` | pitch −30°, τ = 1 s | | | | the estimate of Stage 2: the true pitch reaches 70°, and the drone reaches the ground at 8.1 s at 7.2 m/s. The compensation does not rescue a bad estimate |

The pictures have the six panels of [Stage 3a](../stage3-a/README.md). The last one now has two throttle lines:

- **Dashed grey:** the throttle asked for.
- **Dotted black:** the throttle that went to the mixer, raised for the tilt.

The columns are printed by `measure_flight.py`, as `height` and `rate`. "Short of a level flight" is 2.82 m minus the height gained.

Things to notice:

- **In `05`, `07` and `09` the dotted line steps up** when the tilt starts and down when it ends: to 0.706, 0.722 and 0.752. In the `off` pictures it lies on the dashed line.
- **The step is flat-topped.** The compensation follows the estimate, and the estimate sits on the target. The true angle is 2° to 6° steeper, which is why some height is still lost.
- **In `02` and `03` the height line never levels off.** A throttle 1.4% out is a small force that never stops.
- **In `08` the height line reaches zero** and everything stops.
- **In `09` the motor lines spike to the top of the panel at 9 s.** The raised throttle and the rate loop's push arrive together as the drone levels off: 0.752 + 0.252.
