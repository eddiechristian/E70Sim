//! The throttle on the free body: a fixed hover throttle, and more of it in
//! a tilt.
//!
//! Explained in `Stage3-b.md` in the repo root; the tests are named after
//! its table "On the free body". Each starts hovering at rest, well above
//! the ground, so that the only thing that changes the height is what the
//! test does.
//!
//! Getting there, in `fly_from_hover`:
//!
//! 1. The calibration, with the drone standing on the ground and the motors
//!    off: 1 s of the loop below, with the throttle at 0.
//! 2. Lift it to `START_HEIGHT` with `quad.set_position`, and bring the four
//!    motors to the hover command by stepping `quad.motors` on their own
//!    for a second, as `spin_up` does in the physics crate's tests. It is
//!    then in the air, at rest, with its thrust equal to its weight.
//! 3. Fly the plan.
//!
//! One step of the loop, in order:
//!
//! 1. `to_gyro_rates(quad.rates())` and `to_flight_accel(quad.accel())`
//! 2. `controller.update(gyro, accel, targets, DT)`
//! 3. `quad.step(to_body(commands), DT)`

use flight::calibration::CALIBRATION_STEPS;
use flight::controller::{Controller, Gains, State, Targets};
use flight::estimator::GyroRates;
use physics::motor::Motor;
use physics::quad::{G, Inertia, MotorCommands, Motors, Quad};
use physics::quat::Rates;
use physics::vec3::Vec3;

const DT: f32 = 0.001;

// The body: the starting values of `Stage3-a.md`.
const MASS: f32 = 1.2; // kg
const DRAG: f32 = 0.4; // N per m/s

// Where every flight starts, at rest. High enough that a 30° tilt with no
// compensation, which loses nearly 8 m in 4 s, stays clear of the ground.
const START_HEIGHT: f32 = 50.0; // m

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

// A free quad with the starting values, standing on the ground.
fn quad() -> Quad {
    let motor = || Motor::new(0.04, 6.0);
    Quad::new_free(
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
        MASS,
        DRAG,
    )
}

// The gains chosen so far, with the tilt compensation on or off.
fn gains(tilt_compensation: bool) -> Gains {
    Gains {
        tilt_compensation,
        ..Gains::default()
    }
}

// What a flight is asked to do once it is hovering. Times are counted from
// the moment it starts to fly the plan.
#[derive(Clone, Copy, Debug)]
struct Plan {
    tilt_compensation: bool,
    throttle_error: f32, // a fraction: the throttle is the hover command × (1 + this)
    roll_target: f32,    // °; asked for from `tilt_from` to `tilt_until`
    pitch_target: f32,   // °; the same
    tilt_from: f32,      // s
    tilt_until: f32,     // s
    seconds: f32,        // how long the plan is flown
}

// Hovering level for `seconds`, with the right throttle.
fn hover(tilt_compensation: bool, seconds: f32) -> Plan {
    Plan {
        tilt_compensation,
        throttle_error: 0.0,
        roll_target: 0.0,
        pitch_target: 0.0,
        tilt_from: 0.0,
        tilt_until: 0.0,
        seconds,
    }
}

// A second of hovering, then the given tilt for 4 s, then 2 s level again.
fn tilt(tilt_compensation: bool, roll: f32, pitch: f32) -> Plan {
    Plan {
        roll_target: roll,
        pitch_target: pitch,
        tilt_from: 1.0,
        tilt_until: 5.0,
        ..hover(tilt_compensation, 7.0)
    }
}

// The true state at one moment of a flight, and what went to the motors.
#[derive(Clone, Copy, Debug)]
struct Sample {
    time: f32,   // s, counted from the start of the plan
    height: f32, // m
    climb: f32,  // m/s; positive upwards, which is `−velocity().z`
    roll: f32,   // °, true
    pitch: f32,  // °, true
    commands: flight::mixer::MotorCommands,
}

// Brings the drone to a hover at `START_HEIGHT` as described at the top of
// this file, flies `plan`, and returns one sample for each step of the
// plan.
fn fly_from_hover(plan: Plan) -> Vec<Sample> {
    let mut quad = quad();
    let mut controller = Controller::new(gains(plan.tilt_compensation));
    let hover = quad.hover_command();

    // One step of the loop, with the given targets.
    let mut step = |quad: &mut Quad, targets: Targets| {
        let gyro = to_gyro_rates(quad.rates());
        let accel = to_flight_accel(quad.accel());
        let commands = controller.update(gyro, accel, targets, DT);
        quad.step(to_body(commands), DT);
        (commands, controller.state())
    };

    // 1. The calibration, standing on the ground.
    let on_the_ground = Targets {
        roll: 0.0,
        pitch: 0.0,
        yaw_rate: 0.0,
        throttle: 0.0,
    };
    let mut state = State::Calibrating;
    for _ in 0..CALIBRATION_STEPS {
        (_, state) = step(&mut quad, on_the_ground);
    }
    assert_eq!(state, State::Flying);
    assert!(quad.on_ground());

    // 2. Into the air, at rest, with the motors already at the hover
    //    command: a second of stepping them alone is 25 time constants.
    quad.set_position(Vec3 {
        x: 0.0,
        y: 0.0,
        z: -START_HEIGHT,
    });
    for _ in 0..1000 {
        quad.motors.fl.step(hover, DT);
        quad.motors.fr.step(hover, DT);
        quad.motors.br.step(hover, DT);
        quad.motors.bl.step(hover, DT);
    }

    // 3. The plan.
    let steps = (plan.seconds / DT).round() as usize;
    let mut samples = Vec::with_capacity(steps);
    for index in 0..steps {
        let time = index as f32 * DT;
        let tilting = (plan.tilt_from..plan.tilt_until).contains(&time);
        let targets = Targets {
            roll: if tilting { plan.roll_target } else { 0.0 },
            pitch: if tilting { plan.pitch_target } else { 0.0 },
            yaw_rate: 0.0,
            throttle: hover * (1.0 + plan.throttle_error),
        };
        let (commands, _) = step(&mut quad, targets);

        let angles = quad.angles();
        samples.push(Sample {
            time: (index + 1) as f32 * DT,
            height: quad.height(),
            climb: -quad.velocity().z,
            roll: angles.roll.to_degrees(),
            pitch: angles.pitch.to_degrees(),
            commands,
        });
    }
    samples
}

// The hover command of the body the tests fly.
fn hover_command() -> f32 {
    quad().hover_command()
}

// The sample nearest to `time`, which must be after the start.
fn at(samples: &[Sample], time: f32) -> Sample {
    samples[(time / DT).round() as usize - 1]
}

// The average of the four motor commands of a sample.
fn average(commands: flight::mixer::MotorCommands) -> f32 {
    (commands.fl + commands.fr + commands.br + commands.bl) / 4.0
}

// How much height the flight lost between two times, in metres. Positive
// when it ended lower.
fn height_lost(samples: &[Sample], from: f32, until: f32) -> f32 {
    at(samples, from).height - at(samples, until).height
}

// How much height a flight of `tilt` lost over the four seconds of its
// tilt, with the compensation on or off.
fn lost_in_tilt(tilt_compensation: bool, roll: f32, pitch: f32) -> f32 {
    let plan = tilt(tilt_compensation, roll, pitch);
    let samples = fly_from_hover(plan);
    // It did tilt.
    let middle = at(&samples, 3.0);
    assert!((middle.roll - roll).abs() < 8.0, "{middle:?}");
    assert!((middle.pitch - pitch).abs() < 8.0, "{middle:?}");
    height_lost(&samples, plan.tilt_from, plan.tilt_until)
}

#[test]
fn test_hover_is_not_changed() {
    // Level, compensation on, 10 s: the height stays within 1 cm of
    // `START_HEIGHT`, as it does with it off.
    let on = fly_from_hover(hover(true, 10.0));
    let off = fly_from_hover(hover(false, 10.0));
    for (on, off) in on.iter().zip(&off) {
        assert!((on.height - START_HEIGHT).abs() < 0.01, "{on:?}");
        assert!((off.height - START_HEIGHT).abs() < 0.01, "{off:?}");
        // Level, the compensation multiplies by 1: the same commands.
        assert_eq!(on.commands, off.commands);
    }
}

#[test]
fn test_wrong_hover_throttle_drifts() {
    // A throttle 1.4% too high, which is a command of 0.71, level: climbing
    // at about 0.8 m/s after 10 s. The same too low: sinking as fast.
    let too_high = Plan {
        throttle_error: 0.014,
        ..hover(true, 10.0)
    };
    let samples = fly_from_hover(too_high);
    let end = samples.last().unwrap();
    assert!((average(end.commands) - 0.71).abs() < 1e-3, "{end:?}");
    assert!((end.climb - 0.8).abs() < 0.1, "{end:?}");
    // It has gone several metres, and is not coming back.
    assert!(end.height > START_HEIGHT + 4.0, "{end:?}");

    let too_low = Plan {
        throttle_error: -0.014,
        ..hover(true, 10.0)
    };
    let samples = fly_from_hover(too_low);
    let end = samples.last().unwrap();
    assert!((end.climb + 0.8).abs() < 0.1, "{end:?}");
    assert!(end.height < START_HEIGHT - 4.0, "{end:?}");
}

#[test]
fn test_without_it_a_tilt_sinks() {
    // Pitch −10° for 4 s, compensation off: more than 0.7 m lost.
    let lost = lost_in_tilt(false, 0.0, -10.0);
    assert!(lost > 0.7, "lost {lost} m");
}

#[test]
fn test_with_it_much_less() {
    // The same with compensation on: less than 0.3 m lost.
    let lost = lost_in_tilt(true, 0.0, -10.0);
    assert!(lost < 0.3, "lost {lost} m");
    // It is not a height hold: some is still lost, because the true tilt
    // is a little more than the estimate that the compensation uses.
    assert!(lost > 0.0, "lost {lost} m");
}

#[test]
fn test_a_steep_tilt() {
    // Pitch −30° for 4 s: less than 2.5 m lost with compensation, and more
    // than 6 m without.
    let with = lost_in_tilt(true, 0.0, -30.0);
    let without = lost_in_tilt(false, 0.0, -30.0);
    assert!(with < 2.5, "lost {with} m with it");
    assert!(without > 6.0, "lost {without} m without it");
}

#[test]
fn test_both_axes() {
    // Roll 15° and pitch −15°, which is a tilt of 21°, compensation on:
    // less than 1 m lost.
    let with = lost_in_tilt(true, 15.0, -15.0);
    let without = lost_in_tilt(false, 15.0, -15.0);
    assert!(with < 1.0, "lost {with} m with it");
    assert!(without > 3.0 * with, "lost {without} m without it");
}

// The highest of the four commands of a sample.
fn highest(commands: flight::mixer::MotorCommands) -> f32 {
    commands
        .fl
        .max(commands.fr)
        .max(commands.br)
        .max(commands.bl)
}

#[test]
fn test_motors_stay_in_range() {
    // The mixer clamps every command to 0..1, so a command that reaches 1.0
    // is one that asked for more and did not get it.
    //
    // Up to a pitch of −25°, compensation on: no command reaches 1.0. The
    // highest comes when the drone levels off again, where the raised
    // throttle and the rate loop's push add up.
    for pitch in [-10.0, -20.0, -25.0] {
        let samples = fly_from_hover(tilt(true, 0.0, pitch));
        let top = samples
            .iter()
            .map(|s| highest(s.commands))
            .fold(0.0, f32::max);
        assert!(top < 0.96, "pitch {pitch}°: the highest command was {top}");
    }
}

#[test]
fn test_motors_reach_the_limit_at_30() {
    // Pitch −30°, compensation on: levelling off asks for
    // 0.752 + 0.252 = 1.004, so a command sits at 1.0 for a few
    // milliseconds. Without the compensation the highest is 0.953.
    //
    // This is where the throttle runs out of room. The motor limits of
    // `Stage3-c.md` are what deal with it.
    let plan = tilt(true, 0.0, -30.0);
    let samples = fly_from_hover(plan);
    let clamped: Vec<&Sample> = samples
        .iter()
        .filter(|s| highest(s.commands) >= 1.0)
        .collect();
    assert!(!clamped.is_empty(), "no command reached 1.0");
    assert!(clamped.len() < 20, "{} steps at 1.0", clamped.len());
    // All of them as it levels off, not as it tilts or while it holds.
    for s in &clamped {
        assert!(s.time > plan.tilt_until, "{s:?}");
    }

    let samples = fly_from_hover(tilt(false, 0.0, -30.0));
    let top = samples
        .iter()
        .map(|s| highest(s.commands))
        .fold(0.0, f32::max);
    assert!(top < 0.96, "without it the highest command was {top}");
}

#[test]
fn test_it_goes_back() {
    // Level again after the tilt: during the tilt the four commands
    // average more than the hover command, and by the end of the run they
    // average the hover command again.
    let plan = tilt(true, 0.0, -10.0);
    let samples = fly_from_hover(plan);
    let hover = hover_command();

    // Before the tilt: the hover command.
    let before = at(&samples, plan.tilt_from - 0.1);
    assert!(
        (average(before.commands) - hover).abs() < 1e-4,
        "{before:?}"
    );

    // In the tilt, once the drone has turned: about 0.8% more.
    for s in samples
        .iter()
        .filter(|s| s.time > 2.0 && s.time < plan.tilt_until)
    {
        let raised = average(s.commands) / hover;
        assert!(raised > 1.005 && raised < 1.012, "× {raised}: {s:?}");
    }

    // At the end, level again: the hover command, to within what the last
    // degree or two of tilt asks for.
    let end = samples.last().unwrap();
    assert!((average(end.commands) - hover).abs() < 1e-3, "{end:?}");
}
