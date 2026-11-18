//! A quaternion: four numbers that record which way the drone is facing.
//!
//! Step 4 of the physics in `Stage2-a.md` in the repo root. Three angles
//! break down at 90° of pitch; a quaternion has no bad attitudes.

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

    /// Turns the attitude by the three body rates (rad/s) over `dt` seconds,
    /// then normalises.
    ///
    /// 1. Work out the four changes from the old values:
    ///    dw = 0.5 × (−x × roll_rate − y × pitch_rate − z × yaw_rate)
    ///    dx = 0.5 × ( w × roll_rate + y × yaw_rate   − z × pitch_rate)
    ///    dy = 0.5 × ( w × pitch_rate + z × roll_rate − x × yaw_rate)
    ///    dz = 0.5 × ( w × yaw_rate  + x × pitch_rate − y × roll_rate)
    /// 2. Add each change × dt. All four changes must be computed before any
    ///    of them is applied.
    /// 3. Call `normalise`.
    #[allow(unused_variables)] // remove once written
    pub fn update(&mut self, roll_rate: f32, pitch_rate: f32, yaw_rate: f32, dt: f32) {
        todo!()
    }

    /// Divides all four numbers by the length, so the length is 1 again.
    pub fn normalise(&mut self) {
        todo!()
    }

    /// √(w² + x² + y² + z²). Should stay at 1; the tests check it.
    pub fn length(&self) -> f32 {
        todo!()
    }

    /// The attitude as (roll, pitch, yaw) in radians, for the log and plots.
    ///
    ///    roll  = atan2(2 × (w×x + y×z), 1 − 2 × (x² + y²))
    ///    pitch = asin (2 × (w×y − z×x))
    ///    yaw   = atan2(2 × (w×z + x×y), 1 − 2 × (y² + z²))
    ///
    /// Clamp the value given to `asin` to −1..1 first: rounding can push it a
    /// hair outside, and `asin` then returns NaN.
    pub fn angles(&self) -> (f32, f32, f32) {
        todo!()
    }
}

impl Default for Quat {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    #[allow(unused_imports)] // remove once the tests are written
    use super::*;

    #[allow(dead_code)] // remove once the tests are written
    const DT: f32 = 0.001;

    #[test]
    fn test_starts_level() {
        // A new quaternion: all three angles 0.
        todo!()
    }

    #[test]
    fn test_roll_only() {
        // 90 °/s about roll for 0.5 s: roll 45°, pitch and yaw 0°.
        todo!()
    }

    #[test]
    fn test_pitch_only() {
        // 90 °/s about pitch for 0.5 s: pitch 45°, roll and yaw 0°.
        todo!()
    }

    #[test]
    fn test_yaw_only() {
        // 90 °/s about yaw for 0.5 s: yaw 45°, roll and pitch 0°.
        todo!()
    }

    #[test]
    fn test_stays_at_length_one() {
        // Tumble for 10 s with all three rates non-zero: length within
        // 0.0001 of 1.
        todo!()
    }

    #[test]
    fn test_there_and_back() {
        // 90 °/s for 0.5 s, then −90 °/s for 0.5 s: back to all three
        // angles 0.
        todo!()
    }
}
