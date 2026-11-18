//! A quaternion: four numbers that record which way the drone is facing.
//!
//! Step 4 of the physics in `Stage2-a.md` in the repo root. Three angles
//! break down at 90° of pitch; a quaternion has no bad attitudes.

/// Rotation rates about the three body axes, in rad/s: what the three gyros
/// read.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Rates {
    pub roll: f32,  // rad/s; positive right side down
    pub pitch: f32, // rad/s; positive nose up
    pub yaw: f32,   // rad/s; positive nose right
}

/// The attitude as three angles, in radians.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Angles {
    pub roll: f32,  // rad; positive right side down
    pub pitch: f32, // rad; positive nose up
    pub yaw: f32,   // rad; positive nose right
}

pub struct Quat {
    // The length √(w² + x² + y² + z²) is always 1.
    pub w: f32,
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

impl Quat {
    /// Level, nose forward: `(1, 0, 0, 0)`.
    pub fn new() -> Self {
        Self {
            w: 1.0,
            x: 0.0,
            y: 0.0,
            z: 0.0,
        }
    }

    /// Turns the attitude by the three body rates over `dt` seconds, then
    /// normalises.
    ///
    /// 1. Work out the four changes from the old values:
    ///    dw = 0.5 × (−x × rates.roll − y × rates.pitch − z × rates.yaw)
    ///    dx = 0.5 × ( w × rates.roll  + y × rates.yaw   − z × rates.pitch)
    ///    dy = 0.5 × ( w × rates.pitch + z × rates.roll  − x × rates.yaw)
    ///    dz = 0.5 × ( w × rates.yaw   + x × rates.pitch − y × rates.roll)
    /// 2. Add each change × dt. All four changes must be computed before any
    ///    of them is applied.
    /// 3. Call `normalise`.
    pub fn update(&mut self, rates: Rates, dt: f32) {
        let dw = 0.5 * (-self.x * rates.roll - self.y * rates.pitch - self.z * rates.yaw);
        let dx = 0.5 * (self.w * rates.roll + self.y * rates.yaw - self.z * rates.pitch);
        let dy = 0.5 * (self.w * rates.pitch + self.z * rates.roll - self.x * rates.yaw);
        let dz = 0.5 * (self.w * rates.yaw + self.x * rates.pitch - self.y * rates.roll);
        self.w += dw * dt;
        self.x += dx * dt;
        self.y += dy * dt;
        self.z += dz * dt;
        self.normalise();
    }

    /// Divides all four numbers by the length, so the length is 1 again.
    pub fn normalise(&mut self) {
        let length = self.length();
        self.w /= length;
        self.x /= length;
        self.y /= length;
        self.z /= length;
    }

    /// √(w² + x² + y² + z²). Should stay at 1; the tests check it.
    pub fn length(&self) -> f32 {
        (self.w * self.w + self.x * self.x + self.y * self.y + self.z * self.z).sqrt()
    }

    /// The attitude as three angles, for the log and plots.
    ///
    ///    roll  = atan2(2 × (w×x + y×z), 1 − 2 × (x² + y²))
    ///    pitch = asin (2 × (w×y − z×x))
    ///    yaw   = atan2(2 × (w×z + x×y), 1 − 2 × (y² + z²))
    ///
    /// Clamp the value given to `asin` to −1..1 first: rounding can push it a
    /// hair outside, and `asin` then returns NaN.
    pub fn angles(&self) -> Angles {
        let (w, x, y, z) = (self.w, self.x, self.y, self.z);

        let roll = (2.0 * (w * x + y * z)).atan2(1.0 - 2.0 * (x * x + y * y));
        let pitch = (2.0 * (w * y - z * x)).clamp(-1.0, 1.0).asin();
        let yaw = (2.0 * (w * z + x * y)).atan2(1.0 - 2.0 * (y * y + z * z));

        Angles { roll, pitch, yaw }
    }
}

impl Default for Quat {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const DT: f32 = 0.001;

    // Angles are compared in degrees. Stepping at 1 ms leaves errors of a
    // few ten-thousandths of a degree.
    const TOLERANCE_DEG: f32 = 0.01;

    // Turns `quat` at the given rates, in °/s, for `seconds`.
    fn spin(quat: &mut Quat, roll: f32, pitch: f32, yaw: f32, seconds: f32) {
        let rates = Rates {
            roll: roll.to_radians(),
            pitch: pitch.to_radians(),
            yaw: yaw.to_radians(),
        };
        let steps = (seconds / DT).round() as usize;
        for _ in 0..steps {
            quat.update(rates, DT);
        }
    }

    // Checks the three angles of `quat` against the expected ones, in degrees.
    fn assert_angles(quat: &Quat, roll: f32, pitch: f32, yaw: f32) {
        let angles = quat.angles();
        let got = [
            angles.roll.to_degrees(),
            angles.pitch.to_degrees(),
            angles.yaw.to_degrees(),
        ];
        let expected = [roll, pitch, yaw];
        for (name, (got, expected)) in ["roll", "pitch", "yaw"]
            .iter()
            .zip(got.iter().zip(expected.iter()))
        {
            assert!(
                (got - expected).abs() < TOLERANCE_DEG,
                "{name}: got {got}°, expected {expected}°"
            );
        }
    }

    #[test]
    fn test_starts_level() {
        // A new quaternion: all three angles 0.
        let quat = Quat::new();
        assert_angles(&quat, 0.0, 0.0, 0.0);
    }

    #[test]
    fn test_roll_only() {
        // 90 °/s about roll for 0.5 s: roll 45°, pitch and yaw 0°.
        let mut quat = Quat::new();
        spin(&mut quat, 90.0, 0.0, 0.0, 0.5);
        assert_angles(&quat, 45.0, 0.0, 0.0);
    }

    #[test]
    fn test_pitch_only() {
        // 90 °/s about pitch for 0.5 s: pitch 45°, roll and yaw 0°.
        let mut quat = Quat::new();
        spin(&mut quat, 0.0, 90.0, 0.0, 0.5);
        assert_angles(&quat, 0.0, 45.0, 0.0);
    }

    #[test]
    fn test_yaw_only() {
        // 90 °/s about yaw for 0.5 s: yaw 45°, roll and pitch 0°.
        let mut quat = Quat::new();
        spin(&mut quat, 0.0, 0.0, 90.0, 0.5);
        assert_angles(&quat, 0.0, 0.0, 45.0);
    }

    #[test]
    fn test_stays_at_length_one() {
        // Tumble for 10 s with all three rates non-zero: length within
        // 0.0001 of 1.
        let mut quat = Quat::new();
        spin(&mut quat, 200.0, -150.0, 300.0, 10.0);
        assert!((quat.length() - 1.0).abs() < 0.0001);
    }

    #[test]
    fn test_there_and_back() {
        // 90 °/s for 0.5 s, then −90 °/s for 0.5 s: back to all three
        // angles 0.
        let mut quat = Quat::new();
        spin(&mut quat, 90.0, 0.0, 0.0, 0.5);
        spin(&mut quat, -90.0, 0.0, 0.0, 0.5);
        assert_angles(&quat, 0.0, 0.0, 0.0);
    }
}
