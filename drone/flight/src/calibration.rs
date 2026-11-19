//! Start-up calibration: one second of readings with the motors off gives
//! the gyro's bias and the starting angle.
//!
//! Explained in "Start-up calibration" in `Stage2-c.md` in the repo root.
//!
//! This crate is `no_std`, so `sqrt` is not a method of `f32` here. Use
//! `libm::sqrtf(x)`.

#[allow(unused_imports)] // remove once written
use crate::estimator::angles_from_accel;
use crate::estimator::{Accel, Attitude, GyroRates};

/// How many readings make one window: one second at 1 kHz.
pub const CALIBRATION_STEPS: u32 = 1000;

/// The largest average a still gyro may show on any axis, in °/s. More than
/// this is a steady turn, not a bias.
pub const MAX_GYRO_BIAS: f32 = 5.0;

/// The furthest apart the smallest and the largest reading of one gyro axis
/// may be within a window, in °/s. More than this is a bump.
pub const MAX_GYRO_SPREAD: f32 = 30.0;

/// The length the average accelerometer reading must have, in g.
pub const MIN_GRAVITY: f32 = 0.9;
pub const MAX_GRAVITY: f32 = 1.1;

/// What a finished window gives.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CalibrationResult {
    pub gyro_bias: GyroRates, // °/s; subtract from every gyro reading
    pub attitude: Attitude,   // °; the starting angle, for the estimator
}

/// What `Calibration::update` answers.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum CalibrationStatus {
    /// The window is not full yet.
    Collecting,
    /// The window was full but the drone was not still. A new window has
    /// started.
    Failed,
    /// The window was full and passed the checks.
    Done(CalibrationResult),
}

const ZERO_RATES: GyroRates = GyroRates {
    roll: 0.0,
    pitch: 0.0,
    yaw: 0.0,
};

const ZERO_ACCEL: Accel = Accel {
    x: 0.0,
    y: 0.0,
    z: 0.0,
};

#[allow(dead_code)] // remove once `update` is written
pub struct Calibration {
    // State: one window of readings
    count: u32,          // readings taken so far
    gyro_sum: GyroRates, // °/s; the readings added up
    accel_sum: Accel,    // g; the readings added up
    gyro_min: GyroRates, // °/s; the smallest reading of each axis
    gyro_max: GyroRates, // °/s; the largest reading of each axis
}

impl Calibration {
    pub fn new() -> Self {
        Self {
            count: 0,
            gyro_sum: ZERO_RATES,
            accel_sum: ZERO_ACCEL,
            gyro_min: ZERO_RATES,
            gyro_max: ZERO_RATES,
        }
    }

    /// Takes one reading of each sensor, with the motors off.
    ///
    /// 1. Add the readings to the two sums. Keep the smallest and the
    ///    largest reading of each gyro axis; the first reading of a window
    ///    sets both.
    /// 2. Count the reading. Below `CALIBRATION_STEPS`: return `Collecting`.
    /// 3. The window is full. Divide the six sums by the count to get the
    ///    six averages.
    /// 4. Check that the drone was still. For each gyro axis:
    ///    |average| ≤ MAX_GYRO_BIAS
    ///    largest − smallest ≤ MAX_GYRO_SPREAD
    ///    and for the accelerometer, with (x, y, z) its three averages:
    ///    MIN_GRAVITY ≤ √(x² + y² + z²) ≤ MAX_GRAVITY
    /// 5. Call `reset`, whatever the checks said, so that the next call
    ///    starts a new window.
    /// 6. A check failed: return `Failed`. Otherwise return `Done`, with the
    ///    three gyro averages as `gyro_bias` and `angles_from_accel` of the
    ///    accelerometer averages as `attitude`.
    #[allow(unused_variables)] // remove once written
    pub fn update(&mut self, gyro: GyroRates, accel: Accel) -> CalibrationStatus {
        todo!()
    }

    /// Throws the window away, so that the next reading starts a new one.
    pub fn reset(&mut self) {
        *self = Self::new();
    }
}

impl Default for Calibration {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    #[allow(unused_imports)] // remove once the tests are written
    use super::*;

    #[allow(dead_code)] // remove once the tests are written
    const LEVEL: Accel = Accel {
        x: 0.0,
        y: 0.0,
        z: -1.0,
    };

    // The reading, in g, of a still accelerometer at the given roll and
    // pitch in degrees. The same as in the estimator's tests.
    #[allow(dead_code)] // remove once the tests are written
    fn reading_at(roll: f32, pitch: f32) -> Accel {
        let (phi, theta) = (roll.to_radians(), pitch.to_radians());
        Accel {
            x: theta.sin(),
            y: -phi.sin() * theta.cos(),
            z: -phi.cos() * theta.cos(),
        }
    }

    // A small random number generator for the noise test. The same as in
    // the estimator's tests.
    #[allow(dead_code)] // remove once the tests are written
    struct Noise(u32);

    #[allow(dead_code)] // remove once the tests are written
    impl Noise {
        // Spread evenly over −1..1.
        fn next(&mut self) -> f32 {
            self.0 = self.0.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
            (self.0 >> 8) as f32 / (1u32 << 23) as f32 - 1.0
        }

        // Roughly normal, with standard deviation `std`.
        fn normal(&mut self, std: f32) -> f32 {
            let sum: f32 = (0..12).map(|_| self.next()).sum();
            sum / 2.0 * std
        }
    }

    #[test]
    fn test_collecting_until_full() {
        // 999 updates, still and level: `Collecting` every time.
        todo!()
    }

    #[test]
    fn test_done_on_the_last_one() {
        // The 1000th update: `Done`.
        todo!()
    }

    #[test]
    fn test_bias_is_the_average() {
        // A constant gyro reading of (1, −2, 0.5) °/s, level accelerometer:
        // `gyro_bias` is (1, −2, 0.5).
        todo!()
    }

    #[test]
    fn test_starting_angle() {
        // Zero gyro, an accelerometer reading for 10° of roll: `attitude`
        // is roll 10°, pitch 0°.
        todo!()
    }

    #[test]
    fn test_noise_is_averaged() {
        // Level, with noise on all six readings (2 °/s on the gyro, 0.05 g
        // on the accelerometer): bias within 0.3 °/s of zero, angle within
        // 0.3° of level.
        todo!()
    }

    #[test]
    fn test_turning_steadily_fails() {
        // A gyro reading of 10 °/s for the whole window: `Failed`.
        todo!()
    }

    #[test]
    fn test_bumped_fails() {
        // Still, with one reading of 50 °/s in the middle: `Failed`.
        todo!()
    }

    #[test]
    fn test_wrong_size_of_gravity_fails() {
        // An accelerometer reading 1.5 g long: `Failed`.
        todo!()
    }

    #[test]
    fn test_starts_again_after_failing() {
        // A failed window, then 1000 good updates: `Collecting` 999 times,
        // then `Done`.
        todo!()
    }
}
