//! The controller and the free body together: taking off, hovering and
//! tilting, with gravity and a ground.
//!
//! Explained in `Stage3-a.md` in the repo root; the tests are named after
//! its table "The whole loop, free". Each runs the loop of
//! `controller_and_body.rs` on a free body: the calibration with the drone
//! standing on the ground, then a throttle that takes off.
//!
//! One step of the loop, in order:
//!
//! 1. `to_gyro_rates(quad.rates())` and `to_flight_accel(quad.accel())`
//! 2. `controller.update(gyro, accel, targets, DT)`
//! 3. `quad.step(to_body(commands), DT)`
//!
//! The true angles to compare with are `quad.angles()`, in radians, and the
//! true place is `quad.position()`, in metres, with `quad.height()`.

use flight::controller::{Controller, Gains, State, Targets};
use flight::estimator::GyroRates;
use physics::motor::Motor;
use physics::quad::{G, Inertia, MotorCommands, Motors, Quad};
use physics::quat::Rates;

const DT: f32 = 0.001;

// The body: the starting values of `Stage3-a.md`.
const MASS: f32 = 1.2; // kg
const DRAG: f32 = 0.4; // N per m/s

// The run every test flies, with times counted from the start, calibration
// included. The drone stands on the ground through the calibration, climbs,
// and then gets the hover command.
const CALIBRATION: f32 = 1.0; // s; the motors are off until here
const CLIMB_THROTTLE: f32 = 0.76;
const CLIMB_UNTIL: f32 = 3.0; // s; the hover command from here on

// A tilt, when the test has one, is asked for between these two times.
const TILT_FROM: f32 = 5.0; // s
const TILT_UNTIL: f32 = 9.0; // s

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

// The gains chosen so far, with the estimator's time constant as it was
// before this item: 1 s. The test that shows the problem uses it.
fn gains_with_short_tau() -> Gains {
    Gains {
        estimator_time_constant: 1.0,
        ..Gains::default()
    }
}

// What a flight is asked to do, and what its sensors are like.
#[derive(Clone, Copy, Debug)]
struct Flight {
    gains: Gains,
    pitch_target: f32, // °; asked for from `TILT_FROM` to `TILT_UNTIL`. 0 for none
    gyro_bias: f32,    // °/s; added to every gyro reading, on all three axes
    seconds: f32,      // how long the run is, calibration included
}

// A flight with the chosen gains, no tilt and true sensors.
fn flight(seconds: f32) -> Flight {
    Flight {
        gains: Gains::default(),
        pitch_target: 0.0,
        gyro_bias: 0.0,
        seconds,
    }
}

// The true state at one moment of a flight, and what the controller made
// of it.
#[derive(Clone, Copy, Debug)]
struct Sample {
    time: f32,           // s, counted from the start of the run
    on_ground: bool,     //
    height: f32,         // m
    north: f32,          // m; `position().x`
    east: f32,           // m; `position().y`
    roll: f32,           // °, true
    pitch: f32,          // °, true
    pitch_estimate: f32, // °; 0 while calibrating
    state: State,
}

// Runs the whole loop for `flight.seconds` and returns one sample for each
// step.
//
// The targets at a time `t`: level and no yaw throughout; the throttle is
// `CLIMB_THROTTLE` until `CLIMB_UNTIL` and `quad.hover_command()` after;
// the pitch is `flight.pitch_target` from `TILT_FROM` to `TILT_UNTIL`.
// While it calibrates the controller keeps the motors off whatever the
// throttle says.
fn fly(quad: &mut Quad, flight: Flight) -> Vec<Sample> {
    let mut controller = Controller::new(flight.gains);
    let hover = quad.hover_command();
    let steps = (flight.seconds / DT).round() as usize;
    let mut samples = Vec::with_capacity(steps);

    for step in 0..steps {
        let time = step as f32 * DT;
        let tilting = (TILT_FROM..TILT_UNTIL).contains(&time);
        let targets = Targets {
            roll: 0.0,
            pitch: if tilting { flight.pitch_target } else { 0.0 },
            yaw_rate: 0.0,
            throttle: if time < CLIMB_UNTIL {
                CLIMB_THROTTLE
            } else {
                hover
            },
        };

        let mut gyro = to_gyro_rates(quad.rates());
        gyro.roll += flight.gyro_bias;
        gyro.pitch += flight.gyro_bias;
        gyro.yaw += flight.gyro_bias;
        let accel = to_flight_accel(quad.accel());

        let commands = controller.update(gyro, accel, targets, DT);
        quad.step(to_body(commands), DT);

        let angles = quad.angles();
        let position = quad.position();
        samples.push(Sample {
            time: (step + 1) as f32 * DT,
            on_ground: quad.on_ground(),
            height: quad.height(),
            north: position.x,
            east: position.y,
            roll: angles.roll.to_degrees(),
            pitch: angles.pitch.to_degrees(),
            pitch_estimate: controller.attitude().map_or(0.0, |attitude| attitude.pitch),
            state: controller.state(),
        });
    }
    samples
}

// A flight with the chosen gains and a tilt of −10° of pitch, nose down,
// long enough to see the drone slow down again afterwards.
fn tilted_flight() -> Flight {
    Flight {
        pitch_target: -10.0,
        ..flight(12.0)
    }
}

// The sample nearest to `time`.
fn at(samples: &[Sample], time: f32) -> Sample {
    samples[(time / DT).round() as usize - 1]
}

// Checks the two limits on the estimate: the true pitch within 3° of the
// −10° asked for while the tilt is held, and within 3° of level once it is
// over. Each is checked from half a second after the target changes, which
// is the time the drone takes to turn.
fn assert_true_pitch_follows(samples: &[Sample]) {
    for s in between(samples, TILT_FROM + 0.5, TILT_UNTIL) {
        assert!((s.pitch + 10.0).abs() < 3.0, "in the tilt: {s:?}");
    }
    let end = samples.last().unwrap().time;
    for s in between(samples, TILT_UNTIL + 0.5, end) {
        assert!(s.pitch.abs() < 3.0, "after the tilt: {s:?}");
    }
}

// The samples from `from` seconds up to `until`.
fn between(samples: &[Sample], from: f32, until: f32) -> impl Iterator<Item = &Sample> {
    samples
        .iter()
        .filter(move |s| s.time >= from && s.time < until)
}

#[test]
fn test_calibrates_on_the_ground() {
    // Motors off for the first second: the calibration passes, so the state
    // is `Flying` from 1.0 s, and until then the drone is on the ground and
    // has not moved.
    let samples = fly(&mut quad(), flight(1.5));
    for s in between(&samples, 0.0, CALIBRATION) {
        assert_eq!(s.state, State::Calibrating, "{s:?}");
        assert!(s.on_ground, "{s:?}");
        assert_eq!(s.height, 0.0, "{s:?}");
        assert_eq!(s.north, 0.0, "{s:?}");
        assert_eq!(s.east, 0.0, "{s:?}");
    }
    for s in between(&samples, CALIBRATION, 1.5) {
        assert_eq!(s.state, State::Flying, "{s:?}");
    }
}

#[test]
fn test_takes_off_level() {
    // Throttle 0.76 from 1.0 s: off the ground within 0.2 s, and roll and
    // pitch stay within 0.5° of level all the way up.
    let samples = fly(&mut quad(), flight(CLIMB_UNTIL));
    let lift_off = samples
        .iter()
        .find(|s| !s.on_ground)
        .map(|s| s.time)
        .expect("never left the ground");
    assert!(
        lift_off > CALIBRATION && lift_off <= CALIBRATION + 0.2,
        "left the ground at {lift_off} s"
    );
    for s in &samples {
        assert!(s.roll.abs() < 0.5 && s.pitch.abs() < 0.5, "{s:?}");
    }
    // It did climb: about 2.5 m in the two seconds.
    let top = samples.last().unwrap();
    assert!(top.height > 2.0, "{top:?}");
    assert!(!top.on_ground);
}

#[test]
fn test_hover_holds_its_place() {
    // Climb, then the hover command, level targets, 13 s: north and east
    // each change by less than 0.1 m over the 10 s from 3.0 s.
    let samples = fly(&mut quad(), flight(13.0));
    for s in between(&samples, CLIMB_UNTIL, 13.0) {
        assert!(s.north.abs() < 0.1 && s.east.abs() < 0.1, "{s:?}");
        assert!(s.roll.abs() < 0.5 && s.pitch.abs() < 0.5, "{s:?}");
    }
    // It is still in the air. The hover command does not hold a height: the
    // climb only fades, so it ends higher than it was at 3 s.
    let end = samples.last().unwrap();
    assert!(end.height > at(&samples, CLIMB_UNTIL).height, "{end:?}");
}

#[test]
fn test_a_tilt_moves_it() {
    // Target pitch −10°, nose down, for 4 s: it goes north by metres, and
    // east by less than 0.1 m.
    let samples = fly(&mut quad(), tilted_flight());
    let before = at(&samples, TILT_FROM);
    let after = at(&samples, TILT_UNTIL);
    let gone = after.north - before.north;
    assert!(gone > 5.0, "went {gone} m north");
    for s in &samples {
        assert!(s.east.abs() < 0.1, "{s:?}");
        assert!(s.roll.abs() < 0.5, "{s:?}");
    }
    // A tilt costs lift: it climbs less over those four seconds than a
    // level flight does.
    let level = fly(&mut quad(), flight(12.0));
    assert!(after.height < at(&level, TILT_UNTIL).height - 0.3);
}

#[test]
fn test_estimate_holds_in_a_tilt() {
    // The same, with the chosen τ (`Gains::default()`): the true pitch
    // stays within 3° of −10° from 0.5 s after the tilt is asked for until
    // it ends.
    let samples = fly(&mut quad(), tilted_flight());
    for s in between(&samples, TILT_FROM + 0.5, TILT_UNTIL) {
        assert!((s.pitch + 10.0).abs() < 3.0, "{s:?}");
    }
}

#[test]
fn test_estimate_holds_when_it_stops() {
    // Level again after the tilt, with the chosen τ: from 0.5 s after
    // `TILT_UNTIL` the true pitch stays within 3° of level, while the drone
    // slows down.
    let samples = fly(&mut quad(), tilted_flight());
    // It is moving when the tilt ends, and slower by the end of the run.
    let speed = |time: f32| (at(&samples, time).north - at(&samples, time - 0.1).north) / 0.1;
    assert!(speed(TILT_UNTIL) > 2.0);
    assert!(speed(12.0) < speed(TILT_UNTIL) - 1.0);

    for s in between(&samples, TILT_UNTIL + 0.5, 12.0) {
        assert!(s.pitch.abs() < 3.0, "{s:?}");
    }
}

#[test]
fn test_bias_and_noise() {
    // The same two limits with a gyro bias of 1 °/s, which the calibration
    // removes.
    //
    // Noise needs the `Gyro` of the physics crate, one for each axis, as in
    // `src/bin/quad.rs`; add it here if the bias alone does not say enough.
    let biased = Flight {
        gyro_bias: 1.0,
        ..tilted_flight()
    };
    let samples = fly(&mut quad(), biased);
    assert_true_pitch_follows(&samples);

    // The flight is the one with no bias: the calibration took it out.
    let clean = fly(&mut quad(), tilted_flight());
    assert_true_pitch_follows(&clean);
    let end = samples.last().unwrap();
    let clean_end = clean.last().unwrap();
    assert!((end.pitch - clean_end.pitch).abs() < 0.2, "{end:?}");
    assert!((end.north - clean_end.north).abs() < 0.2, "{end:?}");
}

#[test]
fn test_short_tau_shows_the_problem() {
    // The tilt with `gains_with_short_tau()`: the true pitch passes −20°
    // while the estimate stays within 1.5° of the −10° asked for. The test
    // that shows why τ changed.
    let short = Flight {
        gains: gains_with_short_tau(),
        ..tilted_flight()
    };
    let samples = fly(&mut quad(), short);

    let steepest = between(&samples, TILT_FROM, TILT_UNTIL)
        .map(|s| s.pitch)
        .fold(0.0, f32::min);
    assert!(steepest < -20.0, "the true pitch reached {steepest}°");

    // The controller did its job on the number it was given.
    for s in between(&samples, TILT_FROM + 1.0, TILT_UNTIL) {
        assert!((s.pitch_estimate + 10.0).abs() < 1.5, "{s:?}");
    }

    // Levelling off goes wrong the other way: the nose comes well up while
    // the estimate reads level.
    let highest = between(&samples, TILT_UNTIL, 12.0)
        .map(|s| s.pitch)
        .fold(0.0, f32::max);
    assert!(
        highest > 8.0,
        "the true pitch reached {highest}° afterwards"
    );
}
