//! Three axes, four motors: a quadcopter body held at its centre, free to
//! rotate any way but not to move.
//!
//! Explained in `Stage2-a.md` in the repo root.
//!
//! Body frame: x forward (roll, positive right side down), y right (pitch,
//! positive nose up), z down (yaw, positive nose right).

use crate::motor::Motor;
use crate::quat::Quat;

// Motor order, clockwise from the front left, seen from above. The mixer in
// the `flight` crate uses the same order; the tests in `drone/sim/tests`
// check that the two agree.
pub const FL: usize = 0; // front left, spins clockwise
pub const FR: usize = 1; // front right, spins counter-clockwise
pub const BR: usize = 2; // back right, spins clockwise
pub const BL: usize = 3; // back left, spins counter-clockwise

pub struct Quad {
    // Parameters
    pub arm_length: f32, // m; centre to motor. Start with 0.2
    pub yaw_coeff: f32,  // m; reaction torque per newton of thrust (`c`). Start with 0.015
    pub ixx: f32,        // kg·m²; how hard it is to roll. Start with 0.015
    pub iyy: f32,        // kg·m²; how hard it is to pitch. Start with 0.015
    pub izz: f32,        // kg·m²; how hard it is to yaw. Start with 0.025

    // Parts
    pub motors: [Motor; 4], // in the order FL, FR, BR, BL

    // State
    roll_rate: f32,  // rad/s, body frame
    pitch_rate: f32, // rad/s, body frame
    yaw_rate: f32,   // rad/s, body frame
    attitude: Quat,
}

impl Quad {
    /// `inertia` is `[ixx, iyy, izz]`; `motors` is in the order FL, FR, BR, BL.
    pub fn new(arm_length: f32, yaw_coeff: f32, inertia: [f32; 3], motors: [Motor; 4]) -> Self {
        Self {
            arm_length,
            yaw_coeff,
            ixx: inertia[0],
            iyy: inertia[1],
            izz: inertia[2],
            motors,
            roll_rate: 0.0,
            pitch_rate: 0.0,
            yaw_rate: 0.0,
            attitude: Quat::new(),
        }
    }

    /// Advances the model by `dt` seconds with the four motor commands
    /// (0.0 to 1.0 each, in the order FL, FR, BR, BL).
    ///
    /// 1. Motor commands to thrusts: step each motor and read its thrust.
    /// 2. Thrusts to three torques, with d = arm_length / √2:
    ///    roll torque  = d × (T_FL + T_BL − T_FR − T_BR)
    ///    pitch torque = d × (T_FL + T_FR − T_BR − T_BL)
    ///    yaw torque   = yaw_coeff × (T_FR + T_BL − T_FL − T_BR)
    /// 3. Torques to rates, with the cross term (Euler's rotation equations):
    ///    roll rate  += (roll torque  − (izz − iyy) × pitch rate × yaw rate)   / ixx × dt
    ///    pitch rate += (pitch torque − (ixx − izz) × yaw rate   × roll rate)  / iyy × dt
    ///    yaw rate   += (yaw torque   − (iyy − ixx) × roll rate  × pitch rate) / izz × dt
    ///    Work out the three accelerations from the old rates first, then
    ///    update the three rates.
    /// 4. Rates to attitude: `Quat::update`.
    #[allow(unused_variables)] // remove once written
    pub fn step(&mut self, commands: [f32; 4], dt: f32) {
        todo!()
    }

    /// (roll rate, pitch rate, yaw rate) in rad/s, in the body frame: what
    /// the three gyros read.
    pub fn rates(&self) -> (f32, f32, f32) {
        (self.roll_rate, self.pitch_rate, self.yaw_rate)
    }

    /// (roll, pitch, yaw) in radians: the true attitude.
    pub fn angles(&self) -> (f32, f32, f32) {
        self.attitude.angles()
    }

    pub fn reset(&mut self) {
        for motor in &mut self.motors {
            motor.reset();
        }
        self.roll_rate = 0.0;
        self.pitch_rate = 0.0;
        self.yaw_rate = 0.0;
        self.attitude = Quat::new();
    }
}

#[cfg(test)]
mod tests {
    #[allow(unused_imports)] // remove once the tests are written
    use super::*;

    #[allow(dead_code)] // remove once the tests are written
    const DT: f32 = 0.001;

    // A quad with the starting values.
    #[allow(dead_code)] // remove once the tests are written
    fn quad() -> Quad {
        Quad::new(
            0.2,
            0.015,
            [0.015, 0.015, 0.025],
            [
                Motor::new(0.04, 6.0),
                Motor::new(0.04, 6.0),
                Motor::new(0.04, 6.0),
                Motor::new(0.04, 6.0),
            ],
        )
    }

    #[test]
    fn test_equal_commands_no_rotation() {
        // All four at 0.7 for 1 s: all three rates exactly 0.
        todo!()
    }

    #[test]
    fn test_left_stronger_roll_positive() {
        // FL and BL at 0.75, FR and BR at 0.65: roll rate positive, pitch
        // and yaw rates 0.
        todo!()
    }

    #[test]
    fn test_front_stronger_pitch_positive() {
        // FL and FR at 0.75, BR and BL at 0.65: pitch rate positive, the
        // others 0.
        todo!()
    }

    #[test]
    fn test_counter_clockwise_stronger_yaw_positive() {
        // FR and BL at 0.75, FL and BR at 0.65: yaw rate positive, the
        // others 0.
        todo!()
    }

    #[test]
    fn test_yaw_is_weak() {
        // The same size of command on roll and on yaw: the yaw rate is far
        // smaller.
        todo!()
    }

    #[test]
    fn test_no_drag_keeps_spinning() {
        // Push, then equal commands: the rates hold.
        todo!()
    }

    #[test]
    fn test_cross_term() {
        // Roll and yaw rates together, equal commands: a pitch rate appears.
        // The rates are private fields, which this module can set directly.
        todo!()
    }

    #[test]
    fn test_reset() {
        // After a reset it behaves the same as a brand-new quad.
        todo!()
    }
}
