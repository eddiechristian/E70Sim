//! The X mixer: throttle and three torque commands in, four motor commands
//! out.
//!
//! Explained in `Stage2-a.md` in the repo root. It is the physics of the
//! body run backwards, and has no state.

// Motor order, clockwise from the front left, seen from above. The body in
// the `physics` crate uses the same order; the tests in `drone/sim/tests`
// check that the two agree.
pub const FL: usize = 0; // front left, spins clockwise
pub const FR: usize = 1; // front right, spins counter-clockwise
pub const BR: usize = 2; // back right, spins clockwise
pub const BL: usize = 3; // back left, spins counter-clockwise

/// Turns throttle and the roll, pitch and yaw commands into four motor
/// commands, in the order FL, FR, BR, BL, each clamped to 0.0 to 1.0.
///
/// Positive `roll` rolls right, positive `pitch` raises the nose, positive
/// `yaw` turns the nose right.
///
///    FL = throttle + roll + pitch − yaw
///    FR = throttle − roll + pitch + yaw
///    BR = throttle − roll − pitch − yaw
///    BL = throttle + roll − pitch + yaw
#[allow(unused_variables)] // remove once written
pub fn mix(throttle: f32, roll: f32, pitch: f32, yaw: f32) -> [f32; 4] {
    todo!()
}

#[cfg(test)]
mod tests {
    #[allow(unused_imports)] // remove once the tests are written
    use super::*;

    #[test]
    fn test_throttle_only() {
        // Throttle 0.7, the rest 0: all four 0.7.
        todo!()
    }

    #[test]
    fn test_roll() {
        // Throttle 0.7, roll 0.05: FL and BL 0.75, FR and BR 0.65.
        todo!()
    }

    #[test]
    fn test_pitch() {
        // Throttle 0.7, pitch 0.05: FL and FR 0.75, BR and BL 0.65.
        todo!()
    }

    #[test]
    fn test_yaw() {
        // Throttle 0.7, yaw 0.05: FR and BL 0.75, FL and BR 0.65.
        todo!()
    }

    #[test]
    fn test_sum_unchanged() {
        // Any roll, pitch and yaw that do not clamp: the four commands add
        // up to 4 × throttle.
        todo!()
    }

    #[test]
    fn test_clamp() {
        // Throttle 0.9, roll 0.3: no command above 1.0 or below 0.0.
        todo!()
    }
}
