# Stage 2b estimator on the quad: the pictures

The estimator's roll and pitch against the true angles of the body, while the rate loops run. Explained in [`Stage2-b.md`](../../../Stage2-b.md#results).

Regenerate with `./stage2-b.sh` from `drone/sim/`. Every run is 3 s. All but the first use `--noise 2 --delay 2 --accel-noise 0.5` (gyro noise of 2 °/s, a delay of two steps, accelerometer noise of 0.5 m/s²) and the default seed. The time constant is 1 s unless the name says otherwise.

| Picture | Roll: start | Roll: end | Roll: wobble | Pitch: end | What it shows |
|---|---|---|---|---|---|
| `01_roll_step_clean` | 0° | 0° | none | 0° | no noise: the estimate lies on the true angle through a turn to 50° |
| `02_roll_step_noisy` | 1.41° | 0.09° | 0.05° | −0.03° | noise on: it starts 1.4° off, from one noisy reading, and closes the gap over a few seconds |
| `03_pitch_step_noisy` | 1.41° | 0.18° | 0.05° | −0.14° | the same with the turn about pitch |
| `04_gyro_bias_1` | 1.41° | 1.01° | 0.03° | 0.88° | a gyro bias of 1 °/s: the estimate settles 1° from the truth and stays there |
| `05_gyro_bias_1_tau_0.5` | 1.41° | 0.52° | 0.07° | 0.44° | half the time constant: half the error from the bias, twice the wobble |
| `06_noisy_tau_0.05` | 1.41° | 0.01° | 0.28° | −0.13° | a time constant of 0.05 s, which is α = 0.98 at 1 kHz: accelerometer noise comes straight through |
| `07_noisy_tau_5` | 1.41° | 0.82° | 0.08° | 0.02° | a time constant of 5 s: after 3 s it has barely begun to correct its starting error |

Each picture has three panels: roll (true and estimated), pitch (true and estimated), and the error of both, which is the estimate minus the true angle.

The columns (`measure_estimator.py` prints them, with a few more):

- **Start:** the error of the very first estimate, which is a single accelerometer reading.
- **End:** the average error over the last second.
- **Wobble:** the standard deviation of the error over the last second.

Three things to notice:

- **The starting error is the same 1.41° in every noisy run.** The seed is the same, so the first accelerometer reading is too. The estimate begins as that one reading, noise included.
- **In the bias pictures the true angle itself slopes down.** The rate loops trust the gyro. A gyro that reads 1 °/s too high makes them hold the body at −1 °/s to see zero, so the drone really turns. The estimator's part is the constant 1° gap between the two lines.
- **Pitch moves a little in the roll runs.** Noise in the pitch gyro reaches the motors through the pitch rate loop, so the body's true pitch wanders by about a tenth of a degree.
