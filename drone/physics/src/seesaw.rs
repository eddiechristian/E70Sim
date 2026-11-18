//! One axis, two motors: a bar on a central pivot with a motor at each end.
//!
//! Explained in `Stage1-b.md` in the repo root.

use crate::motor::Motor;

pub struct Seesaw {
    // Parameters
    pub arm_length: f32, // m; pivot to motor. Start with 0.2
    pub inertia: f32,    // kg·m²; how hard the bar is to spin. Start with 0.015

    // Parts
    pub left: Motor,
    pub right: Motor,

    // State
    rate: f32,  // rad/s; what the gyro reads
    angle: f32, // rad; positive when the right side is up
}

impl Seesaw {
    pub fn new(arm_length: f32, inertia: f32, left: Motor, right: Motor) -> Self {
        Self {
            arm_length,
            inertia,
            left,
            right,
            rate: 0.0,
            angle: 0.0,
        }
    }

    /// Advances the model by `dt` seconds with the two motor commands
    /// (0.0 to 1.0 each).
    ///
    /// 1. Motor commands to thrusts.
    /// 2. Thrusts to torque.
    /// 3. Torque to rotation rate.
    /// 4. Rate to angle.
    pub fn step(&mut self, left_command: f32, right_command: f32, dt: f32) {
        self.left.step(left_command, dt);
        self.right.step(right_command, dt);
        let left_thrust = self.left.thrust();
        let right_thrust = self.right.thrust();
        let torque = (right_thrust - left_thrust) * self.arm_length;
        self.rate += (torque / self.inertia) * dt;
        self.angle += self.rate * dt;
    }

    /// Rotation rate in rad/s: the gyro reading.
    pub fn rate(&self) -> f32 {
        self.rate
    }

    /// Angle in radians.
    pub fn angle(&self) -> f32 {
        self.angle
    }

    pub fn reset(&mut self) {
        self.left.reset();
        self.right.reset();
        self.rate = 0.0;
        self.angle = 0.0;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const DT: f32 = 0.001;

    // A seesaw with the starting values and the given inertia.
    fn seesaw_with_inertia(inertia: f32) -> Seesaw {
        Seesaw::new(0.2, inertia, Motor::new(0.04, 6.0), Motor::new(0.04, 6.0))
    }

    fn seesaw() -> Seesaw {
        seesaw_with_inertia(0.015)
    }

    #[test]
    fn test_equal_commands_no_rotation() {
        let mut seesaw = seesaw();
        for _ in 0..1000 {
            seesaw.step(0.7, 0.7, DT);
        }
        // The two thrusts are identical, so the torque is exactly zero.
        assert!(seesaw.rate() == 0.0);
        assert!(seesaw.angle() == 0.0);
    }

    #[test]
    fn test_right_stronger_rate_positive() {
        let mut seesaw = seesaw();
        // 1 second lets the motors settle at their commands.
        for _ in 0..1000 {
            seesaw.step(0.6, 0.8, DT);
        }
        println!("rate: {}, angle: {}", seesaw.rate(), seesaw.angle());
        assert!(seesaw.rate() > 0.0);
        assert!(seesaw.angle() > 0.0);

        // With the motors settled the thrusts are 6 x 0.6^2 = 2.16 N and
        // 6 x 0.8^2 = 3.84 N. The torque is 1.68 x 0.2 = 0.336 N.m, and
        // 0.336 / 0.015 = 22.4 rad/s^2. Over 100 steps the rate rises by 2.24.
        let before = seesaw.rate();
        for _ in 0..100 {
            seesaw.step(0.6, 0.8, DT);
        }
        let gained = seesaw.rate() - before;
        println!("rate gained in 0.1 s: {}", gained);
        assert!((gained - 2.24).abs() < 0.01);
    }

    #[test]
    fn test_left_stronger_rate_negative() {
        let mut seesaw = seesaw();
        for _ in 0..1000 {
            seesaw.step(0.8, 0.6, DT);
        }
        println!("rate: {}, angle: {}", seesaw.rate(), seesaw.angle());
        assert!(seesaw.rate() < 0.0);
        assert!(seesaw.angle() < 0.0);
    }

    #[test]
    fn test_no_drag_keeps_spinning() {
        let mut seesaw = seesaw();
        // Push for 0.2 s.
        for _ in 0..200 {
            seesaw.step(0.6, 0.8, DT);
        }
        // Equal commands. The motors need a moment to reach the same speed.
        for _ in 0..1000 {
            seesaw.step(0.7, 0.7, DT);
        }
        let rate = seesaw.rate();
        println!("rate once the motors match: {}", rate);
        assert!(rate > 0.1);

        // Two more seconds with no torque: the rate must hold, not decay.
        for _ in 0..2000 {
            seesaw.step(0.7, 0.7, DT);
        }
        println!("rate 2 s later: {}", seesaw.rate());
        // Not exactly equal: in f32 the two motors stop a hair apart, which
        // leaves a tiny torque. Drag would have lost far more than this.
        assert!((seesaw.rate() - rate).abs() < 0.01);
    }

    #[test]
    fn test_angle_integrates_rate() {
        let mut seesaw = seesaw();
        // Get it spinning at a steady rate, as in the test above.
        for _ in 0..200 {
            seesaw.step(0.6, 0.8, DT);
        }
        for _ in 0..1000 {
            seesaw.step(0.7, 0.7, DT);
        }
        let rate = seesaw.rate();
        let angle = seesaw.angle();

        // At a steady rate, 0.5 s adds rate x 0.5 to the angle.
        for _ in 0..500 {
            seesaw.step(0.7, 0.7, DT);
        }
        let turned = seesaw.angle() - angle;
        println!("rate: {}, turned in 0.5 s: {}", rate, turned);
        assert!((turned - rate * 0.5).abs() < 1e-3);
    }

    #[test]
    fn test_more_inertia_slower_response() {
        let mut light = seesaw_with_inertia(0.015);
        let mut heavy = seesaw_with_inertia(0.030);
        for _ in 0..500 {
            light.step(0.6, 0.8, DT);
            heavy.step(0.6, 0.8, DT);
        }
        println!("light: {}, heavy: {}", light.rate(), heavy.rate());
        // Twice the inertia gives half the rate from the same torque.
        assert!(heavy.rate() > 0.0);
        assert!((heavy.rate() - light.rate() / 2.0).abs() < 1e-3);
    }

    #[test]
    fn test_reset() {
        let mut seesaw = seesaw();
        for _ in 0..500 {
            seesaw.step(0.6, 0.8, DT);
        }
        seesaw.reset();
        assert!(seesaw.rate() == 0.0);
        assert!(seesaw.angle() == 0.0);
        assert!(seesaw.left.speed() == 0.0);
        assert!(seesaw.right.speed() == 0.0);

        // After a reset it behaves the same as a brand-new seesaw.
        let mut fresh = self::seesaw();
        seesaw.step(0.6, 0.8, DT);
        fresh.step(0.6, 0.8, DT);
        assert!(seesaw.rate() == fresh.rate());
        assert!(seesaw.angle() == fresh.angle());
    }
}
