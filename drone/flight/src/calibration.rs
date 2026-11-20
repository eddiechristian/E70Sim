//! Start-up calibration: one second of readings with the motors off gives
//! the gyro's bias and the starting angle.
//!
//! Explained in "Start-up calibration" in `Stage2-c.md` in the repo root.
//!
//! This crate is `no_std`, so `sqrt` is not a method of `f32` here. Use
//! `libm::sqrtf(x)`.

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
    Failed(&'static str),
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
    pub fn update(&mut self, sensor_gyro: GyroRates, sensor_accel: Accel) -> CalibrationStatus {
        // 1. Sums, and the smallest and largest reading of each gyro axis.
        self.gyro_sum.roll += sensor_gyro.roll;
        self.gyro_sum.pitch += sensor_gyro.pitch;
        self.gyro_sum.yaw += sensor_gyro.yaw;

        self.accel_sum.x += sensor_accel.x;
        self.accel_sum.y += sensor_accel.y;
        self.accel_sum.z += sensor_accel.z;

        if self.count == 0 {
            // The first reading of a window sets both.
            self.gyro_min = sensor_gyro;
            self.gyro_max = sensor_gyro;
        } else {
            self.gyro_min.roll = self.gyro_min.roll.min(sensor_gyro.roll);
            self.gyro_min.pitch = self.gyro_min.pitch.min(sensor_gyro.pitch);
            self.gyro_min.yaw = self.gyro_min.yaw.min(sensor_gyro.yaw);

            self.gyro_max.roll = self.gyro_max.roll.max(sensor_gyro.roll);
            self.gyro_max.pitch = self.gyro_max.pitch.max(sensor_gyro.pitch);
            self.gyro_max.yaw = self.gyro_max.yaw.max(sensor_gyro.yaw);
        }

        // 2. Count the reading.
        self.count += 1;
        if self.count < CALIBRATION_STEPS {
            return CalibrationStatus::Collecting;
        }

        // 3. The window is full: the six averages.
        let n = self.count as f32;
        let gyro_bias = GyroRates {
            roll: self.gyro_sum.roll / n,
            pitch: self.gyro_sum.pitch / n,
            yaw: self.gyro_sum.yaw / n,
        };
        let accel_avg = Accel {
            x: self.accel_sum.x / n,
            y: self.accel_sum.y / n,
            z: self.accel_sum.z / n,
        };
        let spread = GyroRates {
            roll: self.gyro_max.roll - self.gyro_min.roll,
            pitch: self.gyro_max.pitch - self.gyro_min.pitch,
            yaw: self.gyro_max.yaw - self.gyro_min.yaw,
        };
        let gravity = libm::sqrtf(
            accel_avg.x * accel_avg.x + accel_avg.y * accel_avg.y + accel_avg.z * accel_avg.z,
        );

        // 5. Start a new window, whatever the checks say. Everything the
        //    checks need is in the locals above.
        self.reset();

        // 4. Was the drone still?
        if spread.roll > MAX_GYRO_SPREAD {
            return CalibrationStatus::Failed("gyro roll spread too large");
        }
        if spread.pitch > MAX_GYRO_SPREAD {
            return CalibrationStatus::Failed("gyro pitch spread too large");
        }
        if spread.yaw > MAX_GYRO_SPREAD {
            return CalibrationStatus::Failed("gyro yaw spread too large");
        }
        if gyro_bias.roll.abs() > MAX_GYRO_BIAS {
            return CalibrationStatus::Failed("gyro roll average too large");
        }
        if gyro_bias.pitch.abs() > MAX_GYRO_BIAS {
            return CalibrationStatus::Failed("gyro pitch average too large");
        }
        if gyro_bias.yaw.abs() > MAX_GYRO_BIAS {
            return CalibrationStatus::Failed("gyro yaw average too large");
        }
        if !(MIN_GRAVITY..=MAX_GRAVITY).contains(&gravity) {
            return CalibrationStatus::Failed("accelerometer average is not 1 g long");
        }

        // 6. All checks passed.
        CalibrationStatus::Done(CalibrationResult {
            gyro_bias,
            attitude: angles_from_accel(accel_avg),
        })
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
    use super::*;

    const LEVEL: Accel = Accel {
        x: 0.0,
        y: 0.0,
        z: -1.0,
    };

    // The reading, in g, of a still accelerometer at the given roll and
    // pitch in degrees. The same as in the estimator's tests.
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
    struct Noise(u32);

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

    // Feeds one full window. `reading(i)` gives the gyro and accelerometer
    // readings of update `i`. Checks that every update but the last answers
    // `Collecting`, and returns what the last one answers.
    fn run_window(
        cal: &mut Calibration,
        mut reading: impl FnMut(u32) -> (GyroRates, Accel),
    ) -> CalibrationStatus {
        for i in 0..CALIBRATION_STEPS - 1 {
            let (gyro, accel) = reading(i);
            assert_eq!(cal.update(gyro, accel), CalibrationStatus::Collecting);
        }
        let (gyro, accel) = reading(CALIBRATION_STEPS - 1);
        cal.update(gyro, accel)
    }

    // The result of a window that must have passed.
    fn expect_done(status: CalibrationStatus) -> CalibrationResult {
        match status {
            CalibrationStatus::Done(result) => result,
            other => panic!("expected Done, got {other:?}"),
        }
    }

    fn assert_failed(status: CalibrationStatus) {
        assert!(
            matches!(status, CalibrationStatus::Failed(_)),
            "expected Failed, got {status:?}"
        );
    }

    fn assert_near(actual: f32, expected: f32, tolerance: f32) {
        assert!(
            (actual - expected).abs() <= tolerance,
            "{actual} is not within {tolerance} of {expected}"
        );
    }

    #[test]
    fn test_collecting_until_full() {
        // 999 updates, still and level: `Collecting` every time.
        let mut cal = Calibration::new();
        for _ in 0..CALIBRATION_STEPS - 1 {
            assert_eq!(cal.update(ZERO_RATES, LEVEL), CalibrationStatus::Collecting);
        }
    }

    #[test]
    fn test_done_on_the_last_one() {
        // The 1000th update: `Done`.
        let mut cal = Calibration::new();
        let status = run_window(&mut cal, |_| (ZERO_RATES, LEVEL));
        expect_done(status);
    }

    #[test]
    fn test_bias_is_the_average() {
        // A constant gyro reading of (1, −2, 0.5) °/s, level accelerometer:
        // `gyro_bias` is (1, −2, 0.5).
        let gyro = GyroRates {
            roll: 1.0,
            pitch: -2.0,
            yaw: 0.5,
        };
        let mut cal = Calibration::new();
        let result = expect_done(run_window(&mut cal, |_| (gyro, LEVEL)));
        assert_near(result.gyro_bias.roll, 1.0, 1e-3);
        assert_near(result.gyro_bias.pitch, -2.0, 1e-3);
        assert_near(result.gyro_bias.yaw, 0.5, 1e-3);
    }

    #[test]
    fn test_starting_angle() {
        // Zero gyro, an accelerometer reading for 10° of roll: `attitude`
        // is roll 10°, pitch 0°.
        let accel = reading_at(10.0, 0.0);
        let mut cal = Calibration::new();
        let result = expect_done(run_window(&mut cal, |_| (ZERO_RATES, accel)));
        assert_near(result.attitude.roll, 10.0, 0.01);
        assert_near(result.attitude.pitch, 0.0, 0.01);
    }

    #[test]
    fn test_noise_is_averaged() {
        // Level, with noise on all six readings (2 °/s on the gyro, 0.05 g
        // on the accelerometer): bias within 0.3 °/s of zero, angle within
        // 0.3° of level.
        let mut noise = Noise(12345);
        let mut cal = Calibration::new();
        let status = run_window(&mut cal, |_| {
            let gyro = GyroRates {
                roll: noise.normal(2.0),
                pitch: noise.normal(2.0),
                yaw: noise.normal(2.0),
            };
            let accel = Accel {
                x: LEVEL.x + noise.normal(0.05),
                y: LEVEL.y + noise.normal(0.05),
                z: LEVEL.z + noise.normal(0.05),
            };
            (gyro, accel)
        });
        let result = expect_done(status);
        assert_near(result.gyro_bias.roll, 0.0, 0.3);
        assert_near(result.gyro_bias.pitch, 0.0, 0.3);
        assert_near(result.gyro_bias.yaw, 0.0, 0.3);
        assert_near(result.attitude.roll, 0.0, 0.3);
        assert_near(result.attitude.pitch, 0.0, 0.3);
    }

    #[test]
    fn test_turning_steadily_fails() {
        // A gyro reading of 10 °/s for the whole window: `Failed`. Each axis
        // in turn, and both directions.
        for rate in [10.0, -10.0] {
            let turns = [
                GyroRates {
                    roll: rate,
                    ..ZERO_RATES
                },
                GyroRates {
                    pitch: rate,
                    ..ZERO_RATES
                },
                GyroRates {
                    yaw: rate,
                    ..ZERO_RATES
                },
            ];
            for gyro in turns {
                let mut cal = Calibration::new();
                assert_failed(run_window(&mut cal, |_| (gyro, LEVEL)));
            }
        }
    }

    #[test]
    fn test_bumped_fails() {
        // Still, with one reading of 50 °/s in the middle: `Failed`. The
        // average stays at 0.05 °/s, so only the spread can catch it.
        let bumps = [
            GyroRates {
                roll: 50.0,
                ..ZERO_RATES
            },
            GyroRates {
                pitch: 50.0,
                ..ZERO_RATES
            },
            GyroRates {
                yaw: 50.0,
                ..ZERO_RATES
            },
        ];
        for bump in bumps {
            let mut cal = Calibration::new();
            let status = run_window(&mut cal, |i| {
                let gyro = if i == CALIBRATION_STEPS / 2 {
                    bump
                } else {
                    ZERO_RATES
                };
                (gyro, LEVEL)
            });
            assert_failed(status);
        }
    }

    #[test]
    fn test_wrong_size_of_gravity_fails() {
        // An accelerometer reading 1.5 g long: `Failed`. So is one that is
        // too short.
        for z in [-1.5, -0.5] {
            let accel = Accel { z, ..LEVEL };
            let mut cal = Calibration::new();
            assert_failed(run_window(&mut cal, |_| (ZERO_RATES, accel)));
        }
    }

    #[test]
    fn test_starts_again_after_failing() {
        // A failed window, then 1000 good updates: `Collecting` 999 times,
        // then `Done`. Nothing of the failed window is left in the result.
        let turning = GyroRates {
            roll: 10.0,
            ..ZERO_RATES
        };
        let mut cal = Calibration::new();
        assert_failed(run_window(&mut cal, |_| (turning, LEVEL)));

        let result = expect_done(run_window(&mut cal, |_| (ZERO_RATES, LEVEL)));
        assert_near(result.gyro_bias.roll, 0.0, 1e-3);
        assert_near(result.attitude.roll, 0.0, 0.01);
    }
}
