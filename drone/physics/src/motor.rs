//! One motor and propeller: the speed follows the command with a short lag,
//! and thrust is proportional to speed squared.
//!
//! Step 1 of the physics in `Stage1-b.md` in the repo root. Stage 2 reuses
//! this unchanged for all four motors.

pub struct Motor {
    // Parameters
    pub time_constant: f32, // s; how quickly the speed follows the command. Start with 0.04
    pub thrust_coeff: f32,  // N; thrust at full speed (speed = 1.0). Start with 6.0

    // State
    speed: f32, // no unit: fraction of full speed, 0.0 (stopped) to 1.0 (full)
}

impl Motor {
    pub fn new(time_constant: f32, thrust_coeff: f32) -> Self {
        Self {
            time_constant,
            thrust_coeff,
            speed: 0.0,
        }
    }

    /// Advances the motor by `dt` seconds towards `command` (0.0 to 1.0).
    pub fn step(&mut self, command: f32, dt: f32) {
        self.speed += (command - self.speed) * dt / self.time_constant;
    }

    /// Current thrust in newtons.
    pub fn thrust(&self) -> f32 {
        self.speed * self.speed * self.thrust_coeff
    }

    pub fn speed(&self) -> f32 {
        self.speed
    }

    pub fn reset(&mut self) {
        self.speed = 0.0;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const DT: f32 = 0.001;

    #[test]
    fn test_speed_starts_at_zero() {
        let motor = Motor::new(0.04, 6.0);
        assert!(motor.speed() == 0.0);
        assert!(motor.thrust() == 0.0);
    }

    #[test]
    fn test_speed_approaches_command() {
        let mut motor = Motor::new(0.04, 6.0);
        // 1 second is 25 time constants, far more than enough to settle.
        for _ in 0..1000 {
            motor.step(0.7, DT);
        }
        println!("speed: {}", motor.speed());
        assert!((motor.speed() - 0.7).abs() < 1e-4);

        // It follows the command down as well as up.
        for _ in 0..1000 {
            motor.step(0.2, DT);
        }
        println!("speed: {}", motor.speed());
        assert!((motor.speed() - 0.2).abs() < 1e-4);
    }

    #[test]
    fn test_speed_lags_command() {
        let mut motor = Motor::new(0.04, 6.0);

        // One step closes dt / time_constant = 1/40 of the gap.
        motor.step(1.0, DT);
        println!("speed after 1 step: {}", motor.speed());
        assert!((motor.speed() - 0.025).abs() < 1e-6);

        // After one time constant (40 steps) the speed is about 63% of the
        // command.
        for _ in 1..40 {
            motor.step(1.0, DT);
        }
        println!("speed after 40 steps: {}", motor.speed());
        assert!((motor.speed() - 0.63).abs() < 0.01);
    }

    #[test]
    fn test_thrust_is_speed_squared() {
        let mut motor = Motor::new(0.04, 6.0);

        // Full speed gives thrust_coeff newtons.
        for _ in 0..1000 {
            motor.step(1.0, DT);
        }
        println!("thrust at full speed: {}", motor.thrust());
        assert!((motor.thrust() - 6.0).abs() < 1e-3);

        // Half speed gives a quarter of the thrust, not half.
        for _ in 0..1000 {
            motor.step(0.5, DT);
        }
        println!("thrust at half speed: {}", motor.thrust());
        assert!((motor.thrust() - 1.5).abs() < 1e-3);
    }
}
