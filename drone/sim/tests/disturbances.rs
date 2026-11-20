//! The disturbance tests: a kick, a 30° start and one weak motor, on the
//! whole loop.
//!
//! Explained in `Stage2-d.md` in the repo root; the tests are named after
//! its table "The disturbances on the whole loop". Each runs the loop of
//! `controller_and_body.rs`: the calibration with the motors off, then
//! flying with level targets.
//!
//! One step of the loop, in order:
//!
//! 1. `to_gyro_rates(quad.rates())` and `to_flight_accel(quad.accel())`
//! 2. `controller.update(gyro, accel, targets, DT)`
//! 3. Set `quad.push`: the kick's torques while the kick lasts, `NO_PUSH`
//!    at every other time.
//! 4. `quad.step(to_body(commands), DT)`
//!
//! The true angles to compare with are `quad.angles()`, in radians.

use flight::controller::{Controller, Gains, Targets};
use flight::estimator::GyroRates;
use physics::motor::Motor;
use physics::quad::{G, Inertia, MotorCommands, Motors, NO_PUSH, Quad, Torques};
use physics::quat::{Angles, Rates};

const DT: f32 = 0.001;

// The calibration takes this long, with the motors off. Times in the
// comments below are counted from its end.
const CALIBRATION: f32 = 1.0; // s

const THROTTLE: f32 = 0.7;

// The body: the starting values, as in `controller_and_body.rs`.
const MOTOR_TIME_CONSTANT: f32 = 0.04; // s
const THRUST_COEFF: f32 = 6.0; // N; a good motor at full speed

// The standard kick lasts this long and starts this long after the motors
// start, when the drone has been level for a while.
const KICK_FOR: f32 = 0.05; // s
const KICK_AT: f32 = 1.0; // s

const LEVEL: Targets = Targets {
    roll: 0.0,
    pitch: 0.0,
    yaw_rate: 0.0,
    throttle: THROTTLE,
};

// The body's rates are in rad/s and the controller's in °/s.
fn to_gyro_rates(rates: Rates) -> GyroRates {
    GyroRates {
        roll: rates.roll.to_degrees(),
        pitch: rates.pitch.to_degrees(),
        yaw: rates.yaw.to_degrees(),
    }
}

// The body's accelerometer reading is in m/s² and the controller's in g.
fn to_flight_accel(accel: physics::quad::Accel) -> flight::estimator::Accel {
    flight::estimator::Accel {
        x: accel.x / G,
        y: accel.y / G,
        z: accel.z / G,
    }
}

// The mixer and the body each have their own `MotorCommands`. This copies
// one into the other by name.
fn to_body(commands: flight::mixer::MotorCommands) -> MotorCommands {
    MotorCommands {
        fl: commands.fl,
        fr: commands.fr,
        br: commands.br,
        bl: commands.bl,
    }
}

// A quad with the starting values and four good motors.
fn quad() -> Quad {
    let motor = || Motor::new(MOTOR_TIME_CONSTANT, THRUST_COEFF);
    Quad::new(
        0.2,
        0.015,
        Inertia {
            ixx: 0.015,
            iyy: 0.015,
            izz: 0.025,
        },
        Motors {
            fl: motor(),
            fr: motor(),
            br: motor(),
            bl: motor(),
        },
    )
}

// One of the four motors, to pick the weak one.
#[derive(Clone, Copy, Debug, PartialEq)]
enum Corner {
    Fl,
    Fr,
    Br,
    Bl,
}

// A quad with the motor at `corner` weak by `percent`: its `thrust_coeff`
// is `THRUST_COEFF × (1 − percent / 100)`, which is 5.4 N for 10%. The
// other three are good. It is weak from the start.
fn quad_with_weak_motor(corner: Corner, percent: f32) -> Quad {
    let weak = Motor::new(MOTOR_TIME_CONSTANT, THRUST_COEFF * (1.0 - percent / 100.0));
    let mut quad = quad();
    match corner {
        Corner::Fl => quad.motors.fl = weak,
        Corner::Fr => quad.motors.fr = weak,
        Corner::Br => quad.motors.br = weak,
        Corner::Bl => quad.motors.bl = weak,
    }
    quad
}

// A quad that starts at the given roll and pitch, in degrees
// (`Quad::set_attitude`).
fn quad_tilted(roll: f32, pitch: f32) -> Quad {
    let mut quad = quad();
    quad.set_attitude(Angles {
        roll: roll.to_radians(),
        pitch: pitch.to_radians(),
        yaw: 0.0,
    });
    quad
}

// A push on the body for a short time.
#[derive(Clone, Copy, Debug, PartialEq)]
struct Kick {
    push: Torques, // N·m
    at: f32,       // s, counted from the end of the calibration
    lasts: f32,    // s
}

// A kick of the standard length at the standard time, with the given
// torques in N·m.
fn kick(roll: f32, pitch: f32, yaw: f32) -> Kick {
    Kick {
        push: Torques { roll, pitch, yaw },
        at: KICK_AT,
        lasts: KICK_FOR,
    }
}

// The true attitude and the motor commands at one moment of a flight, in
// degrees and °/s.
#[derive(Clone, Copy, Debug)]
struct Sample {
    time: f32, // s, counted from the end of the calibration
    roll: f32,
    pitch: f32,
    yaw_rate: f32,
    commands: flight::mixer::MotorCommands,
}

// Runs the whole loop: the calibration, then `seconds` of flying with
// `targets`, with `kick` applied when its time comes. Returns one sample
// for each step flown; the calibration is not in it.
//
// The loop is the one at the top of this file. `fly` in
// `controller_and_body.rs` is the same without the gains, the kick and the
// commands in the sample.
fn fly(
    quad: &mut Quad,
    gains: Gains,
    targets: Targets,
    kick: Option<Kick>,
    seconds: f32,
) -> Vec<Sample> {
    fly_with(quad, gains, |_| targets, kick, seconds)
}

// The same, with targets that change: `targets_at` is given the time in
// seconds, counted from the end of the calibration, and returns the targets
// for that step. During the calibration it is given 0.
fn fly_with(
    quad: &mut Quad,
    gains: Gains,
    targets_at: impl Fn(f32) -> Targets,
    kick: Option<Kick>,
    seconds: f32,
) -> Vec<Sample> {
    let mut controller = Controller::new(gains);
    let calibration_steps = (CALIBRATION / DT).round() as usize;
    let flying_steps = (seconds / DT).round() as usize;
    let mut samples = Vec::with_capacity(flying_steps);

    // The steps flown on which the kick pushes, counted from the end of the
    // calibration. Whole steps, so that rounding cannot make a kick one
    // step longer or shorter.
    let kick_steps = match kick {
        Some(kick) => {
            let first = (kick.at / DT).round() as usize;
            first..first + (kick.lasts / DT).round() as usize
        }
        None => 0..0,
    };

    for step in 0..calibration_steps + flying_steps {
        let gyro = to_gyro_rates(quad.rates());
        let accel = to_flight_accel(quad.accel());
        let flying = step >= calibration_steps;
        let time = step.saturating_sub(calibration_steps) as f32 * DT;
        let commands = controller.update(gyro, accel, targets_at(time), DT);

        quad.push = match kick {
            Some(kick) if flying && kick_steps.contains(&(step - calibration_steps)) => kick.push,
            _ => NO_PUSH,
        };
        quad.step(to_body(commands), DT);

        if flying {
            let angles = quad.angles();
            samples.push(Sample {
                time: (step + 1 - calibration_steps) as f32 * DT,
                roll: angles.roll.to_degrees(),
                pitch: angles.pitch.to_degrees(),
                yaw_rate: quad.rates().yaw.to_degrees(),
                commands,
            });
        }
    }
    samples
}

// The gains chosen in this item, with the I terms taken out again: the
// controller as it was after Stage 2c.
fn gains_without_i() -> Gains {
    Gains {
        rate_ki: 0.0,
        yaw_ki: 0.0,
        ..Gains::default()
    }
}

// The largest distance from level, in degrees, of the angle that `angle`
// picks out of a sample.
fn peak(samples: &[Sample], angle: fn(&Sample) -> f32) -> f32 {
    samples.iter().map(|s| angle(s).abs()).fold(0.0, f32::max)
}

// Seconds from `from` until the angle stays within 1° of level for the rest
// of the run. 0 when it never leaves.
fn back_within_1_degree(samples: &[Sample], from: f32, angle: fn(&Sample) -> f32) -> f32 {
    samples
        .iter()
        .rfind(|s| s.time >= from && angle(s).abs() > 1.0)
        .map_or(0.0, |s| s.time - from)
}

fn roll(s: &Sample) -> f32 {
    s.roll
}

fn pitch(s: &Sample) -> f32 {
    s.pitch
}

// How long the kick tests fly: the kick comes at 1 s, which leaves 2 s to
// recover in.
const KICK_RUN: f32 = 3.0; // s

// Flies level with the controller as it is and the given kick.
fn fly_kicked(kick: Kick) -> Vec<Sample> {
    fly(&mut quad(), Gains::default(), LEVEL, Some(kick), KICK_RUN)
}

// Checks the recovery from a standard kick on the axis that `angle` picks:
// the peak under 3°, back within 1° of level in under 0.3 s from the start
// of the kick, and within 0.1° at the end.
fn assert_recovers(samples: &[Sample], name: &str, angle: fn(&Sample) -> f32) {
    let peak = peak(samples, angle);
    // The kick did do something.
    assert!(peak > 1.0, "{name} peaked at only {peak}°");
    assert!(peak < 3.0, "{name} peaked at {peak}°");

    let back = back_within_1_degree(samples, KICK_AT, angle);
    assert!(back < 0.3, "{name} back within 1° after {back} s");

    let end = angle(samples.last().unwrap());
    assert!(end.abs() < 0.1, "{name} ended at {end}°");
}

// How long the weak-motor tests fly.
const WEAK_RUN: f32 = 4.0; // s

// Flies level for `WEAK_RUN` with the motor at `corner` 10% weak.
fn fly_weak(corner: Corner, gains: Gains) -> Vec<Sample> {
    let mut quad = quad_with_weak_motor(corner, 10.0);
    fly(&mut quad, gains, LEVEL, None, WEAK_RUN)
}

// Checks that roll and pitch are within 0.2° of level from 1.5 s after the
// motors start.
fn assert_level_after_1_5_s(samples: &[Sample], corner: Corner) {
    for s in samples.iter().filter(|s| s.time >= 1.5) {
        assert!(
            s.roll.abs() < 0.2 && s.pitch.abs() < 0.2,
            "{corner:?} weak: {s:?}"
        );
    }
}

// ---- The kick

#[test]
fn test_kick_roll() {
    // 0.3 N·m about roll for 0.05 s: the roll peaks under 3°, is back
    // within 1° of level in under 0.3 s from the start of the kick, and ends
    // within 0.1°.
    let samples = fly_kicked(kick(0.3, 0.0, 0.0));
    assert_recovers(&samples, "roll", roll);
    // A positive push tips the right side down.
    assert!(samples.iter().map(roll).fold(0.0, f32::max) > 1.0);
    // The pitch is not disturbed.
    assert!(peak(&samples, pitch) < 0.1);
}

#[test]
fn test_kick_pitch() {
    // The same about pitch.
    let samples = fly_kicked(kick(0.0, 0.3, 0.0));
    assert_recovers(&samples, "pitch", pitch);
    assert!(samples.iter().map(pitch).fold(0.0, f32::max) > 1.0);
    assert!(peak(&samples, roll) < 0.1);
}

#[test]
fn test_compound_kick() {
    // 0.3 N·m about roll and 0.3 N·m about pitch, together: each axis as if
    // kicked alone, with a peak under 3° and back within 1° in under 0.3 s.
    // The push is about the diagonal through front-right and back-left, so
    // those two commands stay within 0.01 of the throttle.
    let samples = fly_kicked(kick(0.3, 0.3, 0.0));
    assert_recovers(&samples, "roll", roll);
    assert_recovers(&samples, "pitch", pitch);

    let mut most = 0.0_f32; // how far front-left or back-right moved
    for s in &samples {
        assert!((s.commands.fr - THROTTLE).abs() < 0.01, "{s:?}");
        assert!((s.commands.bl - THROTTLE).abs() < 0.01, "{s:?}");
        most = most.max((s.commands.fl - THROTTLE).abs());
        most = most.max((s.commands.br - THROTTLE).abs());
    }
    // The other two did the work.
    assert!(most > 0.1, "front-left and back-right moved only {most}");
}

#[test]
fn test_compound_kick_other_diagonal() {
    // 0.3 N·m about roll and −0.3 N·m about pitch: the same, with
    // front-left and back-right the commands that stay still.
    let samples = fly_kicked(kick(0.3, -0.3, 0.0));
    assert_recovers(&samples, "roll", roll);
    assert_recovers(&samples, "pitch", pitch);

    let mut most = 0.0_f32; // how far front-right or back-left moved
    for s in &samples {
        assert!((s.commands.fl - THROTTLE).abs() < 0.01, "{s:?}");
        assert!((s.commands.br - THROTTLE).abs() < 0.01, "{s:?}");
        most = most.max((s.commands.fr - THROTTLE).abs());
        most = most.max((s.commands.bl - THROTTLE).abs());
    }
    assert!(most > 0.1, "front-right and back-left moved only {most}");
}

#[test]
fn test_hard_compound_kick() {
    // 1.2 N·m about each, enough to hold two motors at their limits:
    // recovers, and both axes end within 0.1°.
    let samples = fly_kicked(kick(1.2, 1.2, 0.0));
    // A motor did reach its limit.
    let top = samples.iter().map(|s| s.commands.br).fold(0.0, f32::max);
    assert!(top > 0.99, "back-right reached only {top}");

    let end = samples.last().unwrap();
    assert!(end.roll.abs() < 0.1 && end.pitch.abs() < 0.1, "{end:?}");
}

#[test]
fn test_kick_yaw() {
    // 0.1 N·m about yaw for 0.05 s: the yaw rate returns to within 1 °/s,
    // and roll and pitch stay within 0.1° throughout.
    //
    // With a yaw `Ki` the rate swings a little past zero on the way back,
    // which takes about half a second, so it is checked from 1 s after the
    // kick.
    let samples = fly_kicked(kick(0.0, 0.0, 0.1));
    // The kick did turn the nose.
    let fastest = samples.iter().map(|s| s.yaw_rate).fold(0.0, f32::max);
    assert!(fastest > 5.0, "the yaw rate reached only {fastest} °/s");

    for s in samples.iter().filter(|s| s.time >= KICK_AT + 1.0) {
        assert!(s.yaw_rate.abs() < 1.0, "{s:?}");
    }
    for s in &samples {
        assert!(s.roll.abs() < 0.1 && s.pitch.abs() < 0.1, "{s:?}");
    }
}

#[test]
fn test_hard_kick() {
    // 2.4 N·m about roll for 0.05 s, enough to hold the motors at their
    // limit: recovers, and ends within 0.1°.
    let samples = fly_kicked(kick(2.4, 0.0, 0.0));
    // The motors did reach their limit, and the kick was more than the
    // controller could hold to a few degrees.
    let top = samples.iter().map(|s| s.commands.fr).fold(0.0, f32::max);
    assert!(top > 0.99, "front-right reached only {top}");
    assert!(peak(&samples, roll) > 10.0);

    let end = samples.last().unwrap();
    assert!(end.roll.abs() < 0.1, "{end:?}");
}

// ---- The 30° start

#[test]
fn test_start_30() {
    // The body starts at 30° of roll: within 1° of level in under 0.5 s,
    // and never more than 1° past level.
    let mut quad = quad_tilted(30.0, 0.0);
    let samples = fly(&mut quad, Gains::default(), LEVEL, None, 2.0);
    // It did start tilted: the calibration, with the motors off, left it be.
    assert!(samples[0].roll > 29.9, "{:?}", samples[0]);

    let back = back_within_1_degree(&samples, 0.0, roll);
    assert!(back < 0.5, "back within 1° after {back} s");
    for s in &samples {
        assert!(s.roll > -1.0, "{s:?}");
    }
}

#[test]
fn test_start_30_on_both() {
    // Roll 30° and pitch 30°: both within 1° of level in under 0.5 s.
    let mut quad = quad_tilted(30.0, 30.0);
    let samples = fly(&mut quad, Gains::default(), LEVEL, None, 2.0);
    assert!(samples[0].roll > 29.9, "{:?}", samples[0]);
    assert!(samples[0].pitch > 29.9, "{:?}", samples[0]);

    let roll_back = back_within_1_degree(&samples, 0.0, roll);
    let pitch_back = back_within_1_degree(&samples, 0.0, pitch);
    assert!(roll_back < 0.5, "roll back within 1° after {roll_back} s");
    assert!(
        pitch_back < 0.5,
        "pitch back within 1° after {pitch_back} s"
    );
}

// ---- The weak motor

#[test]
fn test_weak_motor_no_i() {
    // Front-right 10% weak, `gains_without_i()`: ends about 1.1° off on
    // roll and −1.1° off on pitch. The test that shows the problem.
    let samples = fly_weak(Corner::Fr, gains_without_i());
    let end = samples.last().unwrap();
    assert!((end.roll - 1.1).abs() < 0.15, "{end:?}");
    assert!((end.pitch + 1.1).abs() < 0.15, "{end:?}");

    // It is not on its way back: the error half-way is the same.
    let half_way = samples[samples.len() / 2];
    assert!((half_way.roll - end.roll).abs() < 0.05, "{half_way:?}");
}

#[test]
fn test_weak_motor() {
    // The same, with the chosen `Ki` (`Gains::default()`): roll and pitch
    // within 0.2° of level 1.5 s after the motors start.
    let samples = fly_weak(Corner::Fr, Gains::default());
    assert_level_after_1_5_s(&samples, Corner::Fr);

    // The weak motor is held up by a larger command than the other three.
    let end = samples.last().unwrap().commands;
    assert!(
        end.fr > end.fl && end.fr > end.br && end.fr > end.bl,
        "{end:?}"
    );
}

#[test]
fn test_weak_motor_yaw() {
    // The same: the yaw rate ends within 0.1 °/s of 0.
    let samples = fly_weak(Corner::Fr, Gains::default());
    let end = samples.last().unwrap();
    assert!(end.yaw_rate.abs() < 0.1, "{end:?}");
}

#[test]
fn test_any_motor() {
    // Each of the four weak in turn: the same result as `test_weak_motor`.
    // With `gains_without_i()` the errors have the signs to match: the
    // weak corner hangs low.
    //
    // Positive roll is right side down and positive pitch is nose up, so a
    // weak motor on the right gives a positive roll and one at the back a
    // positive pitch.
    let corners = [
        (Corner::Fl, -1.0, -1.0),
        (Corner::Fr, 1.0, -1.0),
        (Corner::Br, 1.0, 1.0),
        (Corner::Bl, -1.0, 1.0),
    ];
    for (corner, roll_sign, pitch_sign) in corners {
        let samples = fly_weak(corner, Gains::default());
        assert_level_after_1_5_s(&samples, corner);

        let samples = fly_weak(corner, gains_without_i());
        let end = samples.last().unwrap();
        assert!(
            (end.roll - roll_sign * 1.1).abs() < 0.15,
            "{corner:?} weak: {end:?}"
        );
        assert!(
            (end.pitch - pitch_sign * 1.1).abs() < 0.15,
            "{corner:?} weak: {end:?}"
        );
    }
}

#[test]
fn test_the_step_survives() {
    // No weak motor, the chosen `Ki`, a step to 20° of roll: the Stage 2c
    // limits still hold, 90% of the way within 0.4 s and overshoot under 2%.
    // A `Ki` that fixes the weak motor by spoiling the step fails here.
    //
    // The step comes 0.5 s after the motors start, as it does in the
    // simulation, so that the motors are up to speed when it is asked for.
    const STEP_AT: f32 = 0.5; // s
    let targets_at = |time: f32| {
        if time >= STEP_AT {
            Targets {
                roll: 20.0,
                ..LEVEL
            }
        } else {
            LEVEL
        }
    };
    let samples = fly_with(&mut quad(), Gains::default(), targets_at, None, 2.5);

    let rise = samples
        .iter()
        .find(|s| s.roll >= 18.0)
        .map(|s| s.time - STEP_AT)
        .expect("never reached 90% of the target");
    assert!(rise <= 0.4, "reached 90% after {rise} s");

    let top = samples.iter().map(roll).fold(0.0, f32::max);
    assert!(top <= 20.4, "the roll reached {top}°");
}
