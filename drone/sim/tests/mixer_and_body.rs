//! The mixer and the body together.
//!
//! The mixer is in the `flight` crate and the body in the `physics` crate,
//! and neither knows about the other. Each test feeds the mixer's output
//! into the body, which catches a sign error or a spin direction that
//! differs between the two. On a real drone the same mistake is a flip on
//! take-off. See "The two together" in `Stage2-a.md` in the repo root.

use flight::mixer::{MixerInput, mix};
use physics::motor::Motor;
use physics::quad::{Inertia, Motors, Quad};
use physics::quat::Rates;

const DT: f32 = 0.001;

// An axis that should not be turning is compared with this, in rad/s.
const ZERO_TOLERANCE: f32 = 1e-4;

// The mixer and the body each have their own `MotorCommands`. This copies
// one into the other by name, so no motor can end up in the wrong place.
fn to_body(commands: flight::mixer::MotorCommands) -> physics::quad::MotorCommands {
    physics::quad::MotorCommands {
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

// Mixes throttle 0.7 with the given roll, pitch and yaw commands, holds the
// result on a new quad for 0.2 s and returns the body's rates.
fn rates_after(roll: f32, pitch: f32, yaw: f32) -> Rates {
    let commands = to_body(mix(MixerInput {
        throttle: 0.7,
        roll,
        pitch,
        yaw,
    }));
    let mut quad = quad();
    for _ in 0..200 {
        quad.step(commands, DT);
    }
    quad.rates()
}

fn assert_near_zero(name: &str, rate: f32) {
    assert!(
        rate.abs() < ZERO_TOLERANCE,
        "{name} rate should be 0, got {rate} rad/s"
    );
}

#[test]
fn test_roll_agrees() {
    // Mixer input roll +0.05: roll rate positive, the others 0.
    let rates = rates_after(0.05, 0.0, 0.0);
    assert!(rates.roll > 1.0, "roll rate {} rad/s", rates.roll);
    assert_near_zero("pitch", rates.pitch);
    assert_near_zero("yaw", rates.yaw);
}

#[test]
fn test_pitch_agrees() {
    // Mixer input pitch +0.05: pitch rate positive, the others 0.
    let rates = rates_after(0.0, 0.05, 0.0);
    assert!(rates.pitch > 1.0, "pitch rate {} rad/s", rates.pitch);
    assert_near_zero("roll", rates.roll);
    assert_near_zero("yaw", rates.yaw);
}

#[test]
fn test_yaw_agrees() {
    // Mixer input yaw +0.05: yaw rate positive, the others 0.
    let rates = rates_after(0.0, 0.0, 0.05);
    assert!(rates.yaw > 0.05, "yaw rate {} rad/s", rates.yaw);
    assert_near_zero("roll", rates.roll);
    assert_near_zero("pitch", rates.pitch);
}

#[test]
fn test_negative_commands_agree() {
    // The same three with the sign reversed: each rate is negative.
    assert!(rates_after(-0.05, 0.0, 0.0).roll < -1.0);
    assert!(rates_after(0.0, -0.05, 0.0).pitch < -1.0);
    assert!(rates_after(0.0, 0.0, -0.05).yaw < -0.05);
}
