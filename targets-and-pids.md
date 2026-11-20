# Why every target has a PID

A target on its own does nothing: it is only a number saying what you want. A PID is the part that compares that number with what a sensor says and turns the difference into a push. So every target that has a measurement to check it against gets one PID.

The code is in `drone/flight/src/controller.rs`; the angle loops are explained in [Stage 2c](Stage2-c.md).

| Target | Measured by | PID | The PID's output |
|---|---|---|---|
| `targets.roll` (°) | estimator's roll angle | `roll_angle_pid` | `roll_rate_target` (°/s) |
| `targets.pitch` (°) | estimator's pitch angle | `pitch_angle_pid` | `pitch_rate_target` (°/s) |
| `roll_rate_target` (°/s) | gyro roll rate | `roll_rate_pid` | roll push to the mixer |
| `pitch_rate_target` (°/s) | gyro pitch rate | `pitch_rate_pid` | pitch push to the mixer |
| `targets.yaw_rate` (°/s) | gyro yaw rate | `yaw_rate_pid` | yaw push to the mixer |
| `targets.throttle` | nothing | none | goes straight to the mixer |

Throttle shows the rule from the other side: nothing on the drone measures height or climb, so there is nothing to compare it with, and it has no PID.

## Why roll and pitch have two PIDs each

The motors cannot set an angle directly. They set a torque, torque changes the rotation rate, and the rate changes the angle over time. So the work is split into two questions, each answered by its own PID:

1. **Angle PID:** "I am 10° from where I should be. How fast should I be turning?" With `angle_kp` = 6, the answer is 60 °/s.
2. **Rate PID:** "I should be turning at 60 °/s and the gyro says 0. How hard should the motors push?" With `rate_kp` = 0.0014, the answer is 0.084 of throttle.

The output of the first is the target of the second. That is why `roll_rate_target` is a target that no pilot ever set: the angle PID produces it and the rate PID consumes it.

Yaw has only the rate PID because no sensor tells the drone which way the nose points, so there is no yaw angle to compare a target with.

## Why not one PID from angle error straight to motors

- **The gyro is fast and clean; the angle estimate is slower.** The rate loop runs every step on the gyro and catches a gust before it has built up into an angle error. A single angle PID would only react once the angle had already moved.
- **The rate limit needs a rate to limit.** The clamp at ±200 °/s between the two PIDs is what stops a large angle error from asking for a violent flip.
- **Each loop is tuned on its own.** The rate gains came from [Stage 2a](Stage2-a.md) without any angle loop existing; Stage 2c then added one number, `angle_kp`, on top.
