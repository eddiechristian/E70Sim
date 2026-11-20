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

use flight::controller::{Controller, Gains, State, Targets};
use flight::estimator::GyroRates;
use physics::motor::Motor;
use physics::quad::{G, Inertia, MotorCommands, Motors, Quad};
use physics::quat::{Angles, Rates};

const DT: f32 = 0.001;

// The calibration takes this long, with the motors off. Times in the
// comments below are counted from its end.
const CALIBRATION: f32 = 1.0; // s

const THROTTLE: f32 = 0.7;

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

// The true attitude at one moment of a flight, in degrees and °/s.
#[derive(Clone, Copy, Debug)]
struct Sample {
    time: f32, // s, counted from the end of the calibration
    roll: f32,
    pitch: f32,
    yaw_rate: f32,
}

const LEVEL: Targets = Targets {
    roll: 0.0,
    pitch: 0.0,
    yaw_rate: 0.0,
    throttle: THROTTLE,
};

const NO_BIAS: GyroRates = GyroRates {
    roll: 0.0,
    pitch: 0.0,
    yaw: 0.0,
};

// Runs the whole loop: the calibration, then `seconds` of flying with
// `targets`. `gyro_bias` is added to every gyro reading. Returns one sample
// for each step flown; the calibration is not in it.
fn fly(quad: &mut Quad, gyro_bias: GyroRates, targets: Targets, seconds: f32) -> Vec<Sample> {
    let mut controller = Controller::new(Gains::default());
    let calibration_steps = (CALIBRATION / DT).round() as usize;
    let flying_steps = (seconds / DT).round() as usize;
    let mut samples = Vec::with_capacity(flying_steps);

    for step in 0..calibration_steps + flying_steps {
        let flying = step >= calibration_steps;
        assert_eq!(
            controller.state(),
            if flying {
                State::Flying
            } else {
                State::Calibrating
            },
            "at step {step}"
        );

        let mut gyro = to_gyro_rates(quad.rates());
        gyro.roll += gyro_bias.roll;
        gyro.pitch += gyro_bias.pitch;
        gyro.yaw += gyro_bias.yaw;
        let accel = to_flight_accel(quad.accel());

        let commands = controller.update(gyro, accel, targets, DT);
        quad.step(to_body(commands), DT);

        if flying {
            let angles = quad.angles();
            samples.push(Sample {
                time: (step + 1 - calibration_steps) as f32 * DT,
                roll: angles.roll.to_degrees(),
                pitch: angles.pitch.to_degrees(),
                yaw_rate: quad.rates().yaw.to_degrees(),
            });
        }
    }
    samples
}

// A quad that starts at the given roll, in degrees.
fn quad_rolled(roll: f32) -> Quad {
    let mut quad = quad();
    quad.set_attitude(Angles {
        roll: roll.to_radians(),
        pitch: 0.0,
        yaw: 0.0,
    });
    quad
}

// Checks a step to `target` degrees, where `angle` picks the roll or the
// pitch out of a sample: 90% of the way within 0.4 s, never more than 2%
// past the target, and within 0.2° of it at the end. Works for a negative
// target too.
fn assert_step(samples: &[Sample], target: f32, angle: fn(&Sample) -> f32) {
    // Dividing by the target makes every step a step from 0 to 1.
    let progress = |s: &Sample| angle(s) / target;

    let rise = samples
        .iter()
        .find(|s| progress(s) >= 0.9)
        .map(|s| s.time)
        .expect("never reached 90% of the target");
    assert!(rise <= 0.4, "reached 90% after {rise} s");

    let peak = samples.iter().map(progress).fold(0.0, f32::max);
    assert!(peak <= 1.02, "peak is {}% of the target", peak * 100.0);

    let end = angle(samples.last().unwrap());
    assert!((end - target).abs() < 0.2, "ended at {end}°");
}

#[test]
fn test_holds_level() {
    // Targets zero, 3 s: roll and pitch stay within 0.1° of 0.
    let samples = fly(&mut quad(), NO_BIAS, LEVEL, 3.0);
    for s in &samples {
        assert!(s.roll.abs() < 0.1 && s.pitch.abs() < 0.1, "{s:?}");
    }
}

#[test]
fn test_roll_step() {
    // Target roll 20°: reaches 18° within 0.4 s, overshoot under 2% (the
    // roll never passes 20.4°), ends within 0.2° of 20°.
    let targets = Targets {
        roll: 20.0,
        ..LEVEL
    };
    let samples = fly(&mut quad(), NO_BIAS, targets, 2.0);
    assert_step(&samples, 20.0, |s| s.roll);
    // The pitch is not disturbed.
    assert!(samples.iter().all(|s| s.pitch.abs() < 0.1));
}

#[test]
fn test_pitch_step() {
    // The same about pitch.
    let targets = Targets {
        pitch: 20.0,
        ..LEVEL
    };
    let samples = fly(&mut quad(), NO_BIAS, targets, 2.0);
    assert_step(&samples, 20.0, |s| s.pitch);
    assert!(samples.iter().all(|s| s.roll.abs() < 0.1));
}

#[test]
fn test_negative_step() {
    // Target roll −20°: the same, negative.
    let targets = Targets {
        roll: -20.0,
        ..LEVEL
    };
    let samples = fly(&mut quad(), NO_BIAS, targets, 2.0);
    assert_step(&samples, -20.0, |s| s.roll);
}

#[test]
fn test_levels_from_a_tilt() {
    // The body starts at 20° of roll (`Quad::set_attitude`), targets zero:
    // back within 1° of level in under 0.6 s.
    let samples = fly(&mut quad_rolled(20.0), NO_BIAS, LEVEL, 2.0);
    // It did start tilted: the calibration, with the motors off, left it be.
    assert!(samples[0].roll > 19.9, "{:?}", samples[0]);
    for s in samples.iter().filter(|s| s.time >= 0.6) {
        assert!(s.roll.abs() < 1.0, "{s:?}");
    }
    // It did not swing through to the other side.
    assert!(samples.iter().all(|s| s.roll > -0.4));
}

#[test]
fn test_calibrates_on_a_slope() {
    // The body starts at 10° of roll and stays there through calibration;
    // then target roll 10°: no jump when the motors start, the roll stays
    // within 0.5° of 10°.
    let targets = Targets {
        roll: 10.0,
        ..LEVEL
    };
    let samples = fly(&mut quad_rolled(10.0), NO_BIAS, targets, 2.0);
    for s in &samples {
        assert!((s.roll - 10.0).abs() < 0.5, "{s:?}");
    }
}

#[test]
fn test_bias_calibrated() {
    // A gyro bias of 1 °/s added to every reading, targets zero, 6 s: the
    // true roll ends within 0.2° of 0.
    let bias = GyroRates {
        roll: 1.0,
        pitch: 1.0,
        yaw: 1.0,
    };
    let samples = fly(&mut quad(), bias, LEVEL, 6.0);
    let end = samples.last().unwrap();
    assert!(end.roll.abs() < 0.2 && end.pitch.abs() < 0.2, "{end:?}");
}

#[test]
fn test_two_axes_at_once() {
    // Target roll 15° and pitch −10°: both reached, and the yaw rate stays
    // under 2 °/s.
    let targets = Targets {
        roll: 15.0,
        pitch: -10.0,
        ..LEVEL
    };
    let samples = fly(&mut quad(), NO_BIAS, targets, 2.0);
    let end = samples.last().unwrap();
    assert!((end.roll - 15.0).abs() < 0.2, "{end:?}");
    assert!((end.pitch + 10.0).abs() < 0.2, "{end:?}");
    for s in &samples {
        assert!(s.yaw_rate.abs() < 2.0, "{s:?}");
    }
}
