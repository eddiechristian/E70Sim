//! The controller and the body together: the whole loop.
//!
//! The controller is in the `flight` crate and the body in the `physics`
//! crate, and neither knows about the other. Each test runs the loop the
//! simulation runs: read the body's gyro and accelerometer, call the
//! controller, step the body with the motor commands it returns. The first
//! second is the calibration, with the motors off. See "The whole loop on
//! the body" in `Stage2-c.md` in the repo root.
//!
//! One step of the loop, in order:
//!
//! 1. `to_gyro_rates(quad.rates())` and `to_flight_accel(quad.accel())`
//! 2. `controller.update(gyro, accel, targets, DT)`
//! 3. `quad.step(to_body(commands), DT)`
//!
//! The true angles to compare with are `quad.angles()`, in radians.

#[allow(unused_imports)] // remove once the tests are written
use flight::controller::{Controller, Gains, State, Targets};
use flight::estimator::GyroRates;
use physics::motor::Motor;
#[allow(unused_imports)] // remove once the tests are written
use physics::quad::{G, Inertia, MotorCommands, Motors, Quad};
#[allow(unused_imports)] // remove once the tests are written
use physics::quat::{Angles, Rates};

#[allow(dead_code)] // remove once the tests are written
const DT: f32 = 0.001;

// The calibration takes this long, with the motors off. Times in the
// comments below are counted from its end.
#[allow(dead_code)] // remove once the tests are written
const CALIBRATION: f32 = 1.0; // s

#[allow(dead_code)] // remove once the tests are written
const THROTTLE: f32 = 0.7;

// The body's rates are in rad/s and the controller's in °/s.
#[allow(dead_code)] // remove once the tests are written
fn to_gyro_rates(rates: Rates) -> GyroRates {
    GyroRates {
        roll: rates.roll.to_degrees(),
        pitch: rates.pitch.to_degrees(),
        yaw: rates.yaw.to_degrees(),
    }
}

// The body's accelerometer reading is in m/s² and the controller's in g.
#[allow(dead_code)] // remove once the tests are written
fn to_flight_accel(accel: physics::quad::Accel) -> flight::estimator::Accel {
    flight::estimator::Accel {
        x: accel.x / G,
        y: accel.y / G,
        z: accel.z / G,
    }
}

// The mixer and the body each have their own `MotorCommands`. This copies
// one into the other by name.
#[allow(dead_code)] // remove once the tests are written
fn to_body(commands: flight::mixer::MotorCommands) -> MotorCommands {
    MotorCommands {
        fl: commands.fl,
        fr: commands.fr,
        br: commands.br,
        bl: commands.bl,
    }
}

// A quad with the starting values.
#[allow(dead_code)] // remove once the tests are written
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

#[test]
fn test_holds_level() {
    // Targets zero, 3 s: roll and pitch stay within 0.1° of 0.
    todo!()
}

#[test]
fn test_roll_step() {
    // Target roll 20°: reaches 18° within 0.4 s, overshoot under 2% (the
    // roll never passes 20.4°), ends within 0.2° of 20°.
    todo!()
}

#[test]
fn test_pitch_step() {
    // The same about pitch.
    todo!()
}

#[test]
fn test_negative_step() {
    // Target roll −20°: the same, negative.
    todo!()
}

#[test]
fn test_levels_from_a_tilt() {
    // The body starts at 20° of roll (`Quad::set_attitude`), targets zero:
    // back within 1° of level in under 0.6 s.
    todo!()
}

#[test]
fn test_calibrates_on_a_slope() {
    // The body starts at 10° of roll and stays there through calibration;
    // then target roll 10°: no jump when the motors start, the roll stays
    // within 0.5° of 10°.
    todo!()
}

#[test]
fn test_bias_calibrated() {
    // A gyro bias of 1 °/s added to every reading, targets zero, 6 s: the
    // true roll ends within 0.2° of 0.
    todo!()
}

#[test]
fn test_two_axes_at_once() {
    // Target roll 15° and pitch −10°: both reached, and the yaw rate stays
    // under 2 °/s.
    todo!()
}
