//! The estimator and the body together.
//!
//! The estimator is in the `flight` crate and the body in the `physics`
//! crate, and neither knows about the other. Each test turns the real body,
//! feeds its gyro and accelerometer readings to the estimator and compares
//! the estimate with the body's true angles. That catches a sign that
//! differs between the body's accelerometer and the estimator's formulas.
//! See "The estimator on the body" in `Stage2-b.md` in the repo root.
//!
//! The body is turned through the motors, as in `mixer_and_body.rs`: unequal
//! commands to start it turning, the opposite to stop it.

use flight::estimator::{Estimator, GyroRates};
use physics::motor::Motor;
use physics::quad::{G, Inertia, MotorCommands, Motors, Quad};
use physics::quat::Rates;

const DT: f32 = 0.001;

// Motor commands, in the order fl, fr, br, bl.
const EQUAL: [f32; 4] = [0.7, 0.7, 0.7, 0.7];
const ROLL_RIGHT: [f32; 4] = [0.75, 0.65, 0.65, 0.75]; // left motors stronger
const ROLL_LEFT: [f32; 4] = [0.65, 0.75, 0.75, 0.65];
const NOSE_UP: [f32; 4] = [0.75, 0.75, 0.65, 0.65]; // front motors stronger
const NOSE_DOWN: [f32; 4] = [0.65, 0.65, 0.75, 0.75];
const YAW_RIGHT: [f32; 4] = [0.65, 0.75, 0.65, 0.75]; // counter-clockwise motors stronger

// A push of this long, then the opposite push for as long, then a moment
// with equal commands, leaves the body tilted by about 30°. It is not quite
// still afterwards: thrust goes with the square of motor speed, so the push
// and the stop do not cancel exactly, and the body keeps turning slowly.
const PUSH: f32 = 0.2; // s
const SETTLE: f32 = 0.3; // s

// The body's rates are in rad/s and the estimator's in °/s.
fn to_gyro_rates(rates: Rates) -> GyroRates {
    GyroRates {
        roll: rates.roll.to_degrees(),
        pitch: rates.pitch.to_degrees(),
        yaw: rates.yaw.to_degrees(),
    }
}

// The body's accelerometer reading is in m/s² and the estimator's in g.
fn to_flight_accel(accel: physics::quad::Accel) -> flight::estimator::Accel {
    flight::estimator::Accel {
        x: accel.x / G,
        y: accel.y / G,
        z: accel.z / G,
    }
}

// A quad with the starting values.
fn quad() -> Quad {
    Quad::new(
        0.2,
        0.015,
        Inertia {
            ixx: 0.015,
            iyy: 0.015,
            izz: 0.025,
        },
        Motors {
            fl: Motor::new(0.04, 6.0),
            fr: Motor::new(0.04, 6.0),
            br: Motor::new(0.04, 6.0),
            bl: Motor::new(0.04, 6.0),
        },
    )
}

// The body and the estimator watching it, with the largest difference seen
// so far between the estimate and the true angles, in degrees.
struct Rig {
    quad: Quad,
    estimator: Estimator,
    worst_error: f32,
}

impl Rig {
    fn new() -> Self {
        Self {
            quad: quad(),
            estimator: Estimator::new(1.0),
            worst_error: 0.0,
        }
    }

    // Holds the four motor commands for `seconds`. Each step the estimator
    // is given the body's exact gyro and accelerometer readings.
    fn run(&mut self, commands: [f32; 4], seconds: f32) {
        let commands = MotorCommands {
            fl: commands[0],
            fr: commands[1],
            br: commands[2],
            bl: commands[3],
        };
        let steps = (seconds / DT).round() as usize;
        for _ in 0..steps {
            self.quad.step(commands, DT);
            let estimate = self.estimator.update(
                to_gyro_rates(self.quad.rates()),
                to_flight_accel(self.quad.accel()),
                DT,
            );
            let truth = self.quad.angles();
            let roll_error = (estimate.roll - truth.roll.to_degrees()).abs();
            let pitch_error = (estimate.pitch - truth.pitch.to_degrees()).abs();
            self.worst_error = self.worst_error.max(roll_error).max(pitch_error);
        }
    }

    // Pushes one way, then the other way to stop, then waits.
    fn tilt(&mut self, push: [f32; 4], stop: [f32; 4]) {
        self.run(push, PUSH);
        self.run(stop, PUSH);
        self.run(EQUAL, SETTLE);
    }

    // The true (roll, pitch) in degrees.
    fn true_angles(&self) -> (f32, f32) {
        let angles = self.quad.angles();
        (angles.roll.to_degrees(), angles.pitch.to_degrees())
    }
}

#[test]
fn test_roll_agrees() {
    // Roll the body to about +30°: the estimated roll stays within 0.5° of
    // the true roll throughout.
    let mut rig = Rig::new();
    rig.tilt(ROLL_RIGHT, ROLL_LEFT);
    let (roll, pitch) = rig.true_angles();
    assert!((20.0..45.0).contains(&roll), "true roll {roll}°");
    assert!(pitch.abs() < 0.1, "true pitch {pitch}°");

    // Keep watching for another second.
    rig.run(EQUAL, 1.0);
    assert!(rig.worst_error < 0.5, "worst error {}°", rig.worst_error);
}

#[test]
fn test_pitch_agrees() {
    // The same about pitch.
    let mut rig = Rig::new();
    rig.tilt(NOSE_UP, NOSE_DOWN);
    let (roll, pitch) = rig.true_angles();
    assert!((20.0..45.0).contains(&pitch), "true pitch {pitch}°");
    assert!(roll.abs() < 0.1, "true roll {roll}°");

    // Keep watching for another second.
    rig.run(EQUAL, 1.0);
    assert!(rig.worst_error < 0.5, "worst error {}°", rig.worst_error);
}

#[test]
fn test_signs_agree() {
    // Roll to about −30°, and pitch to about −30°: the estimates are
    // negative and within 0.5° of the true angles.
    let mut rig = Rig::new();
    rig.tilt(ROLL_LEFT, ROLL_RIGHT);
    let (roll, _) = rig.true_angles();
    assert!((-45.0..-20.0).contains(&roll), "true roll {roll}°");
    assert!(rig.worst_error < 0.5, "worst error {}°", rig.worst_error);

    let mut rig = Rig::new();
    rig.tilt(NOSE_DOWN, NOSE_UP);
    let (_, pitch) = rig.true_angles();
    assert!((-45.0..-20.0).contains(&pitch), "true pitch {pitch}°");
    assert!(rig.worst_error < 0.5, "worst error {}°", rig.worst_error);
}

#[test]
fn test_tilt_then_yaw() {
    // Roll to about 30°, then spin up in yaw and keep turning: the tilt
    // moves round the body, from roll into pitch and back, and the estimate
    // stays within 1° of the true angles. Without the terms of "Turning
    // while tilted" the error is tens of degrees.
    let mut rig = Rig::new();
    rig.tilt(ROLL_RIGHT, ROLL_LEFT);
    let (roll_before, _) = rig.true_angles();

    rig.run(YAW_RIGHT, 1.5);
    rig.run(EQUAL, 2.0);

    let yaw_rate = rig.quad.rates().yaw.to_degrees();
    assert!(yaw_rate > 60.0, "yaw rate {yaw_rate} °/s");
    // The tilt really has moved: roll is no longer what it was.
    let (roll_after, _) = rig.true_angles();
    assert!(
        (roll_after - roll_before).abs() > 10.0,
        "roll before {roll_before}°, after {roll_after}°"
    );
    assert!(rig.worst_error < 1.0, "worst error {}°", rig.worst_error);
}
