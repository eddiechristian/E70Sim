//! The mixer and the body together.
//!
//! The mixer is in the `flight` crate and the body in the `physics` crate,
//! and neither knows about the other. Each test feeds the mixer's output
//! straight into the body, which catches a sign error or a motor order that
//! differs between the two. On a real drone the same mistake is a flip on
//! take-off. See "The two together" in `Stage2-a.md` in the repo root.

#[allow(unused_imports)] // remove once the tests are written
use flight::mixer::mix;
#[allow(unused_imports)] // remove once the tests are written
use physics::motor::Motor;
#[allow(unused_imports)] // remove once the tests are written
use physics::quad::Quad;

#[allow(dead_code)] // remove once the tests are written
const DT: f32 = 0.001;

#[test]
fn test_motor_order_matches() {
    // The two crates each define FL, FR, BR and BL. They must be equal.
    assert_eq!(flight::mixer::FL, physics::quad::FL);
    assert_eq!(flight::mixer::FR, physics::quad::FR);
    assert_eq!(flight::mixer::BR, physics::quad::BR);
    assert_eq!(flight::mixer::BL, physics::quad::BL);
}

#[test]
fn test_roll_agrees() {
    // Mixer input roll +0.05: roll rate positive, the others 0.
    todo!()
}

#[test]
fn test_pitch_agrees() {
    // Mixer input pitch +0.05: pitch rate positive, the others 0.
    todo!()
}

#[test]
fn test_yaw_agrees() {
    // Mixer input yaw +0.05: yaw rate positive, the others 0.
    todo!()
}
