# Stage 1d retune: the pictures

The retune of the rate loop with sensor noise and loop delay on, in the order it was done. Explained in [`Stage1-d.md`](../../../Stage1-d.md#the-retune).

Regenerate with `./stage1-d.sh` from `drone/sim/`. Every picture has the same axes, so they can be compared directly. All but the first use `--noise 2 --delay 2` and the default seed.

| Picture | Gains | Rise | Overshoot | Settle | Motor jitter | What it shows |
|---|---|---|---|---|---|---|
| `01_before_kp_0.001` | `Kp` 0.001, no noise or delay | 0.146 s | 4.5% | 0.160 s | none | the starting point from Stage 1c |
| `02_noise_and_delay_kp_0.001` | `Kp` 0.001 | 0.144 s | 4.6% | 0.158 s | 0.002 | noise and delay barely change it. The best P-only tuning |
| `03_lower_kp_0.0005` | `Kp` 0.0005 | 0.300 s | none | 0.363 s | 0.001 | quieter, but twice as slow |
| `04_higher_kp_0.002` | `Kp` 0.002 | 0.081 s | 18% | 0.201 s | 0.004 | faster, but overshoots and settles later |
| `05_much_higher_kp_0.004` | `Kp` 0.004 | 0.050 s | 35% | 0.285 s | 0.008 | rings |
| `06_add_d_kp_0.002_kd_0.00002` | `Kp` 0.002, `Kd` 0.00002 | 0.092 s | 8.5% | 0.194 s | 0.062 | D halves the overshoot, but fills the motor commands with noise |
| `07_small_d_kp_0.00125_kd_0.000005` | `Kp` 0.00125, `Kd` 0.000005 | 0.123 s | 6.0% | 0.237 s | 0.017 | a little D: little gain, still 8 times the jitter |
| `08_add_i_kp_0.001_ki_0.001` | `Kp` 0.001, `Ki` 0.001 | 0.134 s | 12.5% | 0.545 s | 0.002 | I adds overshoot and has nothing to correct on a balanced seesaw |
| `09_filter_the_d_of_06` | `Kp` 0.002, `Kd` 0.00002, filter 0.005 s | 0.091 s | 8.1% | 0.190 s | 0.013 | the gains of 06 with the D-term filter: same response, a fifth of the jitter |
| `10_more_d_kd_0.00004` | `Kp` 0.002, `Kd` 0.00004, filter 0.005 s | 0.106 s | 1.7% | 0.118 s | 0.021 | with the filter there is room for twice the D, and the overshoot goes |
| `11_chosen_longer_filter_0.01` | `Kp` 0.002, `Kd` 0.00004, filter 0.01 s | 0.105 s | 0.8% | 0.118 s | 0.013 | a longer filter: same response, quieter motors. **The chosen gains** |

The columns, all measured on the true rate for the step up to 50 °/s (`measure.py` prints them):

- **Rise:** time from the step until the rate first reaches 45 °/s.
- **Overshoot:** how far the rate passes 50 °/s, as a percentage.
- **Settle:** time from the step until the rate stays within 2.5 °/s of 50.
- **Motor jitter:** standard deviation of the right motor command while holding 50 °/s, when it should be still.
