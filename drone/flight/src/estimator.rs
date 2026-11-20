//! The estimator: roll and pitch angles from the gyro and the accelerometer,
//! by a complementary filter.
//!
//! Explained in `Stage2-b.md` in the repo root.
//!
//! This crate is `no_std`, so `atan2`, `sqrt`, `sin`, `cos` and `tan` are not
//! methods of `f32` here. Use the `libm` crate: `libm::atan2f(y, x)`,
//! `libm::sqrtf(x)`, `libm::sinf(x)`, `libm::cosf(x)`, `libm::tanf(x)`. They
//! work in radians.

/// What the three gyros read, in °/s.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct GyroRates {
    pub roll: f32,  // °/s; positive right side down
    pub pitch: f32, // °/s; positive nose up
    pub yaw: f32,   // °/s; positive nose right
}

/// What the accelerometer reads, in g: the push felt along each body axis.
/// Level and still it is `(0, 0, −1)`, because z points down and the push
/// holding the drone up is upwards.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Accel {
    pub x: f32, // g; positive towards the nose
    pub y: f32, // g; positive towards the right side
    pub z: f32, // g; positive towards the belly
}

/// The estimated attitude, in degrees. There is no yaw: the accelerometer
/// cannot see it.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Attitude {
    pub roll: f32,  // °; positive right side down
    pub pitch: f32, // °; positive nose up
}

/// The two angles that one accelerometer reading gives, in degrees.
///
///    roll  = atan2(−y, −z)
///    pitch = atan2( x, √(y² + z²))
///
/// Only the direction of the reading matters, not its size. `atan2f` returns
/// radians; convert with `to_degrees()`.
pub fn angles_from_accel(accel: Accel) -> Attitude {
    let Accel { x, y, z } = accel;
    Attitude {
        roll: libm::atan2f(-y, -z).to_degrees(),
        pitch: libm::atan2f(x, libm::sqrtf(y * y + z * z)).to_degrees(),
    }
}

// The pitch estimate is kept inside this before its tangent is taken, in
// degrees: tan grows without limit near 90°.
const PITCH_LIMIT_FOR_TAN: f32 = 85.0;

pub struct Estimator {
    // Tuning
    pub time_constant: f32, // s; how long the estimate takes to follow the accelerometer. Start with 1.0

    // State
    attitude: Option<Attitude>, // None until the first update
}

impl Estimator {
    pub fn new(time_constant: f32) -> Self {
        Self {
            time_constant,
            attitude: None,
        }
    }

    /// Takes one reading of each sensor and returns the new estimate. `dt`
    /// is the time since the last call, in seconds.
    ///
    /// 1. Work out the accelerometer's angles with `angles_from_accel`.
    /// 2. On the first call there is no estimate yet: store the
    ///    accelerometer's angles as the estimate and return them.
    /// 3. Otherwise work out how fast each angle is changing. With p, q and
    ///    r the roll, pitch and yaw rates and φ and θ the current roll and
    ///    pitch estimates ("Turning while tilted" in `Stage2-b.md`):
    ///    change of roll  = p + (q × sin φ + r × cos φ) × tan θ
    ///    change of pitch = q × cos φ − r × sin φ
    ///    At level these are just p and q.
    /// 4. Blend, for roll and for pitch:
    ///    alpha = time_constant / (time_constant + dt)
    ///    angle = alpha × (angle + change × dt) + (1 − alpha) × accelerometer angle
    /// 5. Store the new estimate and return it.
    ///
    /// A `dt` that is not a positive number is a bug in the caller: the
    /// estimate is returned unchanged, with a `debug_assert!`, as in `Pid`.
    pub fn update(&mut self, gyro: GyroRates, accel: Accel, dt: f32) -> Attitude {
        // 1. The accelerometer's angles.
        let from_accel = angles_from_accel(accel);

        // A bad dt is treated as "no time passed". Also catches NaN.
        if !(dt > 0.0) {
            debug_assert!(false, "Estimator::update called with dt = {dt}");
            return self.attitude.unwrap_or(from_accel);
        }

        // 2. The first reading sets the estimate.
        let Some(previous) = self.attitude else {
            self.attitude = Some(from_accel);
            return from_accel;
        };

        // 3. How fast each angle is changing, in °/s. The rates are in °/s
        //    and the formulas are linear in them, so only the two angles
        //    need converting to radians.
        let phi = previous.roll.to_radians();
        let theta = previous
            .pitch
            .clamp(-PITCH_LIMIT_FOR_TAN, PITCH_LIMIT_FOR_TAN)
            .to_radians();
        let (sin_phi, cos_phi) = (libm::sinf(phi), libm::cosf(phi));
        let roll_change =
            gyro.roll + (gyro.pitch * sin_phi + gyro.yaw * cos_phi) * libm::tanf(theta);
        let pitch_change = gyro.pitch * cos_phi - gyro.yaw * sin_phi;

        // 4. Blend the gyro's prediction with the accelerometer's answer.
        let alpha = self.time_constant / (self.time_constant + dt);
        let attitude = Attitude {
            roll: alpha * (previous.roll + roll_change * dt) + (1.0 - alpha) * from_accel.roll,
            pitch: alpha * (previous.pitch + pitch_change * dt) + (1.0 - alpha) * from_accel.pitch,
        };

        // 5. Store and return.
        self.attitude = Some(attitude);
        attitude
    }

    /// Sets the estimate, in place of letting the first reading set it. The
    /// start-up calibration calls this with the angle it averaged
    /// (`Stage2-c.md`).
    pub fn set_attitude(&mut self, attitude: Attitude) {
        self.attitude = Some(attitude);
    }

    /// Forgets the estimate, so that the next reading sets it again.
    pub fn reset(&mut self) {
        self.attitude = None;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const DT: f32 = 0.001;

    // Angles are compared in degrees.
    const TOLERANCE_DEG: f32 = 0.05;

    const ZERO_RATES: GyroRates = GyroRates {
        roll: 0.0,
        pitch: 0.0,
        yaw: 0.0,
    };

    const LEVEL: Accel = Accel {
        x: 0.0,
        y: 0.0,
        z: -1.0,
    };

    // The reading, in g, of a still accelerometer at the given roll and
    // pitch in degrees. This is "The reading at any attitude" in
    // `Stage2-b.md`, written out separately from the code under test.
    fn reading_at(roll: f32, pitch: f32) -> Accel {
        let (phi, theta) = (roll.to_radians(), pitch.to_radians());
        Accel {
            x: theta.sin(),
            y: -phi.sin() * theta.cos(),
            z: -phi.cos() * theta.cos(),
        }
    }

    // Updates `estimator` with the same readings for `seconds` and returns
    // the last estimate.
    fn run(
        estimator: &mut Estimator,
        gyro: GyroRates,
        accel: Accel,
        seconds: f32,
        dt: f32,
    ) -> Attitude {
        let steps = (seconds / dt).round() as usize;
        let mut attitude = estimator.update(gyro, accel, dt);
        for _ in 1..steps {
            attitude = estimator.update(gyro, accel, dt);
        }
        attitude
    }

    fn assert_attitude(attitude: Attitude, roll: f32, pitch: f32) {
        assert!(
            (attitude.roll - roll).abs() < TOLERANCE_DEG,
            "roll: got {}°, expected {roll}°",
            attitude.roll
        );
        assert!(
            (attitude.pitch - pitch).abs() < TOLERANCE_DEG,
            "pitch: got {}°, expected {pitch}°",
            attitude.pitch
        );
    }

    // ---- Angles from a reading

    #[test]
    fn test_accel_level() {
        // (0, 0, −1): roll 0°, pitch 0°.
        assert_attitude(angles_from_accel(LEVEL), 0.0, 0.0);
    }

    #[test]
    fn test_accel_nose_up() {
        // (0.5, 0, −0.866): pitch 30°, roll 0°.
        let accel = Accel {
            x: 0.5,
            y: 0.0,
            z: -0.866,
        };
        assert_attitude(angles_from_accel(accel), 0.0, 30.0);
    }

    #[test]
    fn test_accel_right_side_down() {
        // (0, −0.5, −0.866): roll 30°, pitch 0°.
        let accel = Accel {
            x: 0.0,
            y: -0.5,
            z: -0.866,
        };
        assert_attitude(angles_from_accel(accel), 30.0, 0.0);
    }

    #[test]
    fn test_accel_negative_angles() {
        // (−0.5, 0, −0.866): pitch −30°. (0, 0.5, −0.866): roll −30°.
        let nose_down = Accel {
            x: -0.5,
            y: 0.0,
            z: -0.866,
        };
        assert_attitude(angles_from_accel(nose_down), 0.0, -30.0);

        let left_side_down = Accel {
            x: 0.0,
            y: 0.5,
            z: -0.866,
        };
        assert_attitude(angles_from_accel(left_side_down), -30.0, 0.0);
    }

    #[test]
    fn test_accel_size_does_not_matter() {
        // The nose-up reading × 2: the same angles.
        let accel = Accel {
            x: 1.0,
            y: 0.0,
            z: -1.732,
        };
        assert_attitude(angles_from_accel(accel), 0.0, 30.0);
    }

    #[test]
    fn test_accel_both_angles_at_once() {
        // A reading built for roll 20° and pitch −10° gives both back.
        assert_attitude(angles_from_accel(reading_at(20.0, -10.0)), 20.0, -10.0);
    }

    // ---- The filter. Time constant 1 s unless stated.

    #[test]
    fn test_first_reading_sets_the_estimate() {
        // One update with a reading for 20° of roll: roll 20° at once.
        let mut estimator = Estimator::new(1.0);
        let attitude = estimator.update(ZERO_RATES, reading_at(20.0, 0.0), DT);
        assert_attitude(attitude, 20.0, 0.0);
    }

    #[test]
    fn test_gyro_alone() {
        // A very large time constant; level reading; roll rate 10 °/s for
        // 1 s: roll 10°.
        let mut estimator = Estimator::new(1e9);
        estimator.update(ZERO_RATES, LEVEL, DT);
        let gyro = GyroRates {
            roll: 10.0,
            ..ZERO_RATES
        };
        let attitude = run(&mut estimator, gyro, LEVEL, 1.0, DT);
        assert_attitude(attitude, 10.0, 0.0);

        // The same about pitch, the other way.
        let mut estimator = Estimator::new(1e9);
        estimator.update(ZERO_RATES, LEVEL, DT);
        let gyro = GyroRates {
            pitch: -10.0,
            ..ZERO_RATES
        };
        let attitude = run(&mut estimator, gyro, LEVEL, 1.0, DT);
        assert_attitude(attitude, 0.0, -10.0);
    }

    #[test]
    fn test_stays_still() {
        // Level reading, zero rates, 5 s: roll and pitch stay 0.
        let mut estimator = Estimator::new(1.0);
        let attitude = run(&mut estimator, ZERO_RATES, LEVEL, 5.0, DT);
        assert_attitude(attitude, 0.0, 0.0);
    }

    // Starts level, then holds a reading for 10° of roll with zero rates.
    // Returns the roll after 1, 2 and 3 seconds.
    fn follow_ten_degrees(dt: f32) -> [f32; 3] {
        let mut estimator = Estimator::new(1.0);
        estimator.update(ZERO_RATES, LEVEL, dt);
        let accel = reading_at(10.0, 0.0);
        let mut rolls = [0.0; 3];
        for roll in &mut rolls {
            *roll = run(&mut estimator, ZERO_RATES, accel, 1.0, dt).roll;
        }
        rolls
    }

    #[test]
    fn test_follows_the_accelerometer() {
        // Start at 0 with a level reading; then a reading for 10° of roll
        // and zero rates: 6.3° after 1 s, 8.6° after 2 s, 9.5° after 3 s.
        let rolls = follow_ten_degrees(DT);
        assert!((rolls[0] - 6.32).abs() < TOLERANCE_DEG, "1 s: {}", rolls[0]);
        assert!((rolls[1] - 8.65).abs() < TOLERANCE_DEG, "2 s: {}", rolls[1]);
        assert!((rolls[2] - 9.50).abs() < TOLERANCE_DEG, "3 s: {}", rolls[2]);
    }

    // Level reading and a roll rate of 1 °/s that is not real (a bias), for
    // 20 s. Returns the roll the estimate settles at.
    fn roll_with_a_bias(time_constant: f32) -> f32 {
        let mut estimator = Estimator::new(time_constant);
        let gyro = GyroRates {
            roll: 1.0,
            ..ZERO_RATES
        };
        run(&mut estimator, gyro, LEVEL, 20.0, DT).roll
    }

    #[test]
    fn test_bias_is_bounded() {
        // Level reading; roll rate 1 °/s (a bias) for 20 s: roll settles at
        // 1.0°, not 20°.
        let roll = roll_with_a_bias(1.0);
        assert!((roll - 1.0).abs() < TOLERANCE_DEG, "roll {roll}°");
    }

    #[test]
    fn test_bias_scales_with_the_time_constant() {
        // The same with time constants of 0.5 s and 2 s: 0.5° and 2.0°.
        let short = roll_with_a_bias(0.5);
        let long = roll_with_a_bias(2.0);
        assert!((short - 0.5).abs() < TOLERANCE_DEG, "roll {short}°");
        assert!((long - 2.0).abs() < TOLERANCE_DEG, "roll {long}°");
    }

    // A small random number generator for the noise test, since this crate
    // has no random number crate. Returns values spread evenly over −1..1.
    struct Noise(u32);

    impl Noise {
        fn next(&mut self) -> f32 {
            // A linear congruential generator (Numerical Recipes constants).
            self.0 = self.0.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
            (self.0 >> 8) as f32 / (1u32 << 23) as f32 - 1.0
        }

        // Roughly normal, with standard deviation `std`: the sum of twelve
        // even values has a standard deviation of 2.
        fn normal(&mut self, std: f32) -> f32 {
            let sum: f32 = (0..12).map(|_| self.next()).sum();
            sum / 2.0 * std
        }
    }

    #[test]
    fn test_noise_is_averaged() {
        // Level reading with noise of 0.05 g on each axis, 20 s: the
        // estimate stays within about 0.3° of 0.
        let mut noise = Noise(1);
        let mut noisy_level = || Accel {
            x: noise.normal(0.05),
            y: noise.normal(0.05),
            z: -1.0 + noise.normal(0.05),
        };

        // The first estimate is one raw reading, so let the filter settle
        // for 5 s before judging it.
        let mut estimator = Estimator::new(1.0);
        for _ in 0..5_000 {
            estimator.update(ZERO_RATES, noisy_level(), DT);
        }

        let mut worst_estimate: f32 = 0.0;
        let mut worst_raw: f32 = 0.0;
        for _ in 0..20_000 {
            let accel = noisy_level();
            let attitude = estimator.update(ZERO_RATES, accel, DT);
            worst_estimate = worst_estimate.max(attitude.roll.abs().max(attitude.pitch.abs()));
            worst_raw = worst_raw.max(angles_from_accel(accel).roll.abs());
        }
        assert!(worst_estimate < 0.3, "worst estimate {worst_estimate}°");
        // The accelerometer's own angle is far worse, which is the point.
        assert!(worst_raw > 5.0, "worst raw angle {worst_raw}°");
    }

    #[test]
    fn test_dt_independence() {
        // "Follows the accelerometer" at 1 kHz and at 500 Hz: the same
        // angles, within 0.05°.
        let fast = follow_ten_degrees(0.001);
        let slow = follow_ten_degrees(0.002);
        for (fast, slow) in fast.iter().zip(slow.iter()) {
            assert!((fast - slow).abs() < 0.05, "1 kHz {fast}°, 500 Hz {slow}°");
        }
    }

    #[test]
    fn test_reset() {
        // Run, `reset()`, then one update: the same as a new estimator.
        let mut used = Estimator::new(1.0);
        let gyro = GyroRates {
            roll: 30.0,
            pitch: -20.0,
            yaw: 10.0,
        };
        run(&mut used, gyro, reading_at(5.0, 5.0), 2.0, DT);
        used.reset();

        let mut fresh = Estimator::new(1.0);
        let accel = reading_at(-12.0, 7.0);
        assert_eq!(used.update(gyro, accel, DT), fresh.update(gyro, accel, DT));
        assert_eq!(used.update(gyro, accel, DT), fresh.update(gyro, accel, DT));
    }

    #[test]
    fn test_set_attitude() {
        // `set_attitude` with roll 10°, then one update with a level
        // reading and zero rates: the estimate is still close to 10°, not
        // the reading's 0°. The first-reading rule did not apply.
        let mut estimator = Estimator::new(1.0);
        estimator.set_attitude(Attitude {
            roll: 10.0,
            pitch: 0.0,
        });
        let attitude = estimator.update(ZERO_RATES, LEVEL, DT);
        // One step of 1 ms with a time constant of 1 s moves the estimate
        // a thousandth of the way to the reading: 10° becomes 9.99°.
        assert_attitude(attitude, 9.99, 0.0);
    }

    // `dt` of 0, negative or NaN: panics in debug, one test per value.

    #[test]
    #[should_panic]
    fn test_bad_dt_zero() {
        Estimator::new(1.0).update(ZERO_RATES, LEVEL, 0.0);
    }

    #[test]
    #[should_panic]
    fn test_bad_dt_negative() {
        Estimator::new(1.0).update(ZERO_RATES, LEVEL, -0.001);
    }

    #[test]
    #[should_panic]
    fn test_bad_dt_nan() {
        Estimator::new(1.0).update(ZERO_RATES, LEVEL, f32::NAN);
    }

    // ---- The terms of "Turning while tilted"

    #[test]
    fn test_level_yaw_changes_nothing() {
        // Zero roll and pitch; yaw rate 90 °/s: roll and pitch stay 0.
        let mut estimator = Estimator::new(1.0);
        let gyro = GyroRates {
            yaw: 90.0,
            ..ZERO_RATES
        };
        let attitude = run(&mut estimator, gyro, LEVEL, 2.0, DT);
        assert_attitude(attitude, 0.0, 0.0);
    }

    #[test]
    fn test_yaw_moves_a_tilt_round() {
        // A very large time constant; start at roll 30°; then yaw rate
        // 90 °/s for 1 s: roll 0°, pitch −30°. Without the extra terms the
        // roll would stay at 30°.
        //
        // With a very large time constant the accelerometer is ignored
        // after the first call, so the first reading is simply repeated.
        let mut estimator = Estimator::new(1e9);
        let accel = reading_at(30.0, 0.0);
        estimator.update(ZERO_RATES, accel, DT);
        let gyro = GyroRates {
            yaw: 90.0,
            ..ZERO_RATES
        };
        let attitude = run(&mut estimator, gyro, accel, 1.0, DT);
        assert!(attitude.roll.abs() < 0.5, "roll {}°", attitude.roll);
        assert!(
            (attitude.pitch + 30.0).abs() < 0.5,
            "pitch {}°",
            attitude.pitch
        );
    }

    #[test]
    fn test_pitch_rate_while_rolled_90_changes_no_pitch() {
        // Rolled 90° onto its right side, the pitch gyro's axis points
        // straight down, so a pitch rate is a turn about the vertical: the
        // pitch angle does not change.
        let mut estimator = Estimator::new(1e9);
        let accel = reading_at(90.0, 0.0);
        estimator.update(ZERO_RATES, accel, DT);
        let gyro = GyroRates {
            pitch: 45.0,
            ..ZERO_RATES
        };
        let attitude = run(&mut estimator, gyro, accel, 1.0, DT);
        assert_attitude(attitude, 90.0, 0.0);
    }
}
