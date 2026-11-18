# Stage 1b — The seesaw model

This explains the second item of Stage 1 in [`drone.md`](drone.md#stage-1--host-simulation-one-axis):

> `drone/physics/`: one axis, two motors (a seesaw)

The seesaw is the first real physics model in the plan. It is a quadcopter reduced to the simplest thing that still behaves like one. The PID from [`Stage1-a.md`](Stage1-a.md) is the controller; this is the thing it controls.

## The picture

A rigid bar is balanced on a pivot at its centre, with a motor and propeller at each end, both pushing up:

```
   thrust L                    thrust R
      ▲                           ▲
    [motor]─────────┬─────────[motor]
                    △
                  pivot
```

- **Equal thrusts:** the bar stays as it is.
- **Unequal thrusts:** the stronger side rises and the bar rotates about the pivot.

That is how a quad rolls: it speeds up the motors on one side and slows the other. The seesaw is one axis of the drone (roll, for example) with everything else removed. It cannot fly away, yaw or pitch, so there is one angle and one rotation rate to think about.

## The physics

Each step, the model does four things:

1. **Motor commands to thrusts.** Each motor's speed follows its command with a short lag, and thrust is proportional to speed squared.
2. **Thrusts to torque.** `torque = (thrust R − thrust L) × arm length`, where the arm length is the distance from the pivot to a motor.
3. **Torque to rotation rate.** `rate += (torque / inertia) × dt`. Inertia is how hard the bar is to spin: a heavier or longer bar responds more slowly.
4. **Rate to angle.** `angle += rate × dt`.

The gyro reading is the `rate` from step 3.

The model works in radians and rad/s, and the flight code in degrees and °/s; the conversion happens between the two (see [Units](drone.md#units-what-the-gyro-measures)).

## How it differs from the tiny plant in the PID test

The closed-loop test in [`Stage1-a.md`](Stage1-a.md#one-closed-loop-test) used the plant `rate += (output + disturbance − rate) × dt`. The seesaw is the same idea made realistic:

| | Tiny test plant | Seesaw |
|---|---|---|
| Purpose | prove the PID arithmetic controls something | behave like a real drone axis |
| Input | one number | two motor commands |
| Drag | built in, so it settles by itself | almost none, so a push keeps it spinning |
| Motors | instant | lag behind the command |
| Parameters | none | arm length, inertia, thrust coefficient, motor lag |
| Tracks angle | no | yes |

The lack of drag is the important difference. With no controller, any imbalance makes the seesaw spin faster and faster, as a real quad would. The controller has to do all the stabilising, which makes it a fair test.

## Why start with it

- **It needs a mixer in miniature.** The PID gives one torque command, and the code must split it into two motor commands (`base + command` and `base − command`). The four-motor X mixer in Stage 2 is the same idea.
- **It is the real control problem.** The rate PID tuned here has the same structure as the one that will fly the drone.
- **It is easy to check.** With one axis, a plot of rate against time shows at a glance whether the loop is sluggish, oscillating or good.
- **It can be built.** A real two-motor seesaw rig is a common, safe first hardware test, so the simulation and a later bench test line up.

## Where this goes next

- The next Stage 1 item, `drone/sim/`, runs the PID and the seesaw together in a loop and writes a CSV to plot. It is explained in [`Stage1-c.md`](Stage1-c.md).
- The last Stage 1 item adds sensor noise and loop delay to this model. Motor lag is already in step 1 above.
- Stage 2 replaces the bar with a full three-axis body and four motors, reusing the motor and thrust parts unchanged.
