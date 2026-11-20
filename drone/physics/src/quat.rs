//! A quaternion: four numbers that record which way the drone is facing.
//!
//! Step 4 of the physics in `Stage2-a.md` in the repo root. Three angles
//! break down at 90° of pitch; a quaternion has no bad attitudes.

use crate::vec3::Vec3;

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

    /// The attitude with the given three angles: `angles` run backwards.
    /// Used to start the body tilted (`Stage2-c.md`).
    ///
    /// With cr, cp and cy the cosines of half the roll, pitch and yaw
    /// angles, and sr, sp and sy the sines of the same halves:
    ///    w = cr × cp × cy + sr × sp × sy
    ///    x = sr × cp × cy − cr × sp × sy
    ///    y = cr × sp × cy + sr × cp × sy
    ///    z = cr × cp × sy − sr × sp × cy
    pub fn from_angles(angles: Angles) -> Self {
        let (sr, cr) = (angles.roll * 0.5).sin_cos();
        let (sp, cp) = (angles.pitch * 0.5).sin_cos();
        let (sy, cy) = (angles.yaw * 0.5).sin_cos();
        Self {
            w: cr * cp * cy + sr * sp * sy,
            x: sr * cp * cy - cr * sp * sy,
            y: cr * sp * cy + sr * cp * sy,
            z: cr * cp * sy - sr * sp * cy,
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

    /// A vector given along the body's axes, along the world's
    /// (`Stage3-a.md`). The body's z axis, `(0, 0, 1)`, comes out as where
    /// the belly points: straight down when level.
    ///
    /// With `v` the vector and w, x, y and z the four numbers:
    ///    world x = (1 − 2×(y² + z²)) × v.x + 2×(x×y − w×z) × v.y + 2×(x×z + w×y) × v.z
    ///    world y = 2×(x×y + w×z) × v.x + (1 − 2×(x² + z²)) × v.y + 2×(y×z − w×x) × v.z
    ///    world z = 2×(x×z − w×y) × v.x + 2×(y×z + w×x) × v.y + (1 − 2×(x² + y²)) × v.z
    pub fn to_world(&self, v: Vec3) -> Vec3 {
        let (w, x, y, z) = (self.w, self.x, self.y, self.z);
        Vec3 {
            x: (1.0 - 2.0 * (y * y + z * z)) * v.x
                + 2.0 * (x * y - w * z) * v.y
                + 2.0 * (x * z + w * y) * v.z,
            y: 2.0 * (x * y + w * z) * v.x
                + (1.0 - 2.0 * (x * x + z * z)) * v.y
                + 2.0 * (y * z - w * x) * v.z,
            z: 2.0 * (x * z - w * y) * v.x
                + 2.0 * (y * z + w * x) * v.y
                + (1.0 - 2.0 * (x * x + y * y)) * v.z,
        }
    }

    /// The other way: a vector given along the world's axes, along the
    /// body's. The nine numbers are the same as in `to_world`, with rows and
    /// columns changed over:
    ///    body x = (1 − 2×(y² + z²)) × v.x + 2×(x×y + w×z) × v.y + 2×(x×z − w×y) × v.z
    ///    body y = 2×(x×y − w×z) × v.x + (1 − 2×(x² + z²)) × v.y + 2×(y×z + w×x) × v.z
    ///    body z = 2×(x×z + w×y) × v.x + 2×(y×z − w×x) × v.y + (1 − 2×(x² + y²)) × v.z
    pub fn to_body(&self, v: Vec3) -> Vec3 {
        let (w, x, y, z) = (self.w, self.x, self.y, self.z);
        Vec3 {
            x: (1.0 - 2.0 * (y * y + z * z)) * v.x
                + 2.0 * (x * y + w * z) * v.y
                + 2.0 * (x * z - w * y) * v.z,
            y: 2.0 * (x * y - w * z) * v.x
                + (1.0 - 2.0 * (x * x + z * z)) * v.y
                + 2.0 * (y * z + w * x) * v.z,
            z: 2.0 * (x * z + w * y) * v.x
                + 2.0 * (y * z - w * x) * v.y
                + (1.0 - 2.0 * (x * x + y * y)) * v.z,
        }
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

    // ---- From three angles

    // The attitude with the given three angles, in degrees.
    fn from_degrees(roll: f32, pitch: f32, yaw: f32) -> Quat {
        Quat::from_angles(Angles {
            roll: roll.to_radians(),
            pitch: pitch.to_radians(),
            yaw: yaw.to_radians(),
        })
    }

    #[test]
    fn test_from_angles_level() {
        // All three angles 0: (1, 0, 0, 0), the same as `Quat::new()`.
        let quat = from_degrees(0.0, 0.0, 0.0);
        assert_eq!((quat.w, quat.x, quat.y, quat.z), (1.0, 0.0, 0.0, 0.0));
    }

    #[test]
    fn test_from_angles_one_axis() {
        // Roll 30° alone, pitch 30° alone, yaw 30° alone: `angles()` gives
        // each one back, with the other two 0.
        assert_angles(&from_degrees(30.0, 0.0, 0.0), 30.0, 0.0, 0.0);
        assert_angles(&from_degrees(0.0, 30.0, 0.0), 0.0, 30.0, 0.0);
        assert_angles(&from_degrees(0.0, 0.0, 30.0), 0.0, 0.0, 30.0);
    }

    #[test]
    fn test_from_angles_all_three() {
        // Roll 20°, pitch −10°, yaw 45°: `angles()` gives all three back.
        assert_angles(&from_degrees(20.0, -10.0, 45.0), 20.0, -10.0, 45.0);
    }

    #[test]
    fn test_from_angles_matches_turning() {
        // Roll 30° from angles is the same four numbers as a new quaternion
        // turned at 30 °/s about roll for 1 s.
        let set = from_degrees(30.0, 0.0, 0.0);
        let mut turned = Quat::new();
        spin(&mut turned, 30.0, 0.0, 0.0, 1.0);
        for (name, set, turned) in [
            ("w", set.w, turned.w),
            ("x", set.x, turned.x),
            ("y", set.y, turned.y),
            ("z", set.z, turned.z),
        ] {
            assert!(
                (set - turned).abs() < 1e-4,
                "{name}: {set} from angles, {turned} from turning"
            );
        }
    }

    #[test]
    fn test_from_angles_has_length_one() {
        // Any three angles: the length is 1.
        for (roll, pitch, yaw) in [
            (0.0, 0.0, 0.0),
            (20.0, -10.0, 45.0),
            (-170.0, 80.0, -135.0),
            (90.0, 90.0, 90.0),
            (123.0, -45.0, 300.0),
        ] {
            let length = from_degrees(roll, pitch, yaw).length();
            assert!(
                (length - 1.0).abs() < 1e-6,
                "roll {roll}°, pitch {pitch}°, yaw {yaw}°: length {length}"
            );
        }
    }

    // ---- Turning a vector. Named after the table in `Stage3-a.md`.

    // A quaternion at the given three angles, in degrees.
    fn quat_at(roll: f32, pitch: f32, yaw: f32) -> Quat {
        Quat::from_angles(Angles {
            roll: roll.to_radians(),
            pitch: pitch.to_radians(),
            yaw: yaw.to_radians(),
        })
    }

    // Vectors are compared part by part.
    const VECTOR_TOLERANCE: f32 = 1e-3;

    fn assert_vec(got: Vec3, x: f32, y: f32, z: f32) {
        let near = |a: f32, b: f32| (a - b).abs() < VECTOR_TOLERANCE;
        assert!(
            near(got.x, x) && near(got.y, y) && near(got.z, z),
            "got {got:?}, expected ({x}, {y}, {z})"
        );
    }

    // The body's three axes, and the same three names for the world's.
    const X: Vec3 = Vec3 {
        x: 1.0,
        y: 0.0,
        z: 0.0,
    };
    const Z: Vec3 = Vec3 {
        x: 0.0,
        y: 0.0,
        z: 1.0,
    };

    // A few attitudes and vectors with nothing special about them.
    const ATTITUDES: [(f32, f32, f32); 4] = [
        (10.0, 0.0, 0.0),
        (20.0, -35.0, 50.0),
        (-120.0, 15.0, -170.0),
        (5.0, 80.0, 90.0),
    ];
    const VECTORS: [Vec3; 3] = [
        Vec3 {
            x: 1.0,
            y: 2.0,
            z: 3.0,
        },
        Vec3 {
            x: -0.5,
            y: 0.0,
            z: 9.8,
        },
        Vec3 {
            x: 4.0,
            y: -7.0,
            z: -1.5,
        },
    ];

    #[test]
    fn test_level_changes_nothing() {
        // A new quaternion, any vector: the same vector from `to_world` and
        // from `to_body`.
        let level = Quat::new();
        for v in VECTORS {
            assert_vec(level.to_world(v), v.x, v.y, v.z);
            assert_vec(level.to_body(v), v.x, v.y, v.z);
        }
    }

    #[test]
    fn test_nose_up_tilts_the_z_axis_back() {
        // 30° of pitch: the body's z axis, (0, 0, 1), is (0.5, 0, 0.866) in
        // the world. Thrust pushes the other way, up and backwards.
        let nose_up = quat_at(0.0, 30.0, 0.0);
        assert_vec(nose_up.to_world(Z), 0.5, 0.0, 0.866);
        // Nose down tilts it forward.
        let nose_down = quat_at(0.0, -30.0, 0.0);
        assert_vec(nose_down.to_world(Z), -0.5, 0.0, 0.866);
    }

    #[test]
    fn test_right_side_down_tilts_the_z_axis_left() {
        // 30° of roll: the body's z axis is (0, −0.5, 0.866) in the world.
        // Thrust pushes up and to the right.
        let right_down = quat_at(30.0, 0.0, 0.0);
        assert_vec(right_down.to_world(Z), 0.0, -0.5, 0.866);
        let left_down = quat_at(-30.0, 0.0, 0.0);
        assert_vec(left_down.to_world(Z), 0.0, 0.5, 0.866);
    }

    #[test]
    fn test_yaw_turns_north_to_east() {
        // 90° of yaw: the body's x axis, (1, 0, 0), is (0, 1, 0) in the
        // world.
        let facing_east = quat_at(0.0, 0.0, 90.0);
        assert_vec(facing_east.to_world(X), 0.0, 1.0, 0.0);
        // And the other way: seen from that body, north is to its left.
        assert_vec(facing_east.to_body(X), 0.0, -1.0, 0.0);
        // Yaw leaves the z axis alone.
        assert_vec(facing_east.to_world(Z), 0.0, 0.0, 1.0);
    }

    #[test]
    fn test_to_world_and_back() {
        // Any attitude, any vector: `to_body(to_world(v))` is `v` again.
        for (roll, pitch, yaw) in ATTITUDES {
            let quat = quat_at(roll, pitch, yaw);
            for v in VECTORS {
                let back = quat.to_body(quat.to_world(v));
                assert_vec(back, v.x, v.y, v.z);
                let back = quat.to_world(quat.to_body(v));
                assert_vec(back, v.x, v.y, v.z);
            }
        }
    }

    #[test]
    fn test_turning_keeps_the_length() {
        // Any attitude, any vector: the same length before and after, both
        // ways.
        for (roll, pitch, yaw) in ATTITUDES {
            let quat = quat_at(roll, pitch, yaw);
            for v in VECTORS {
                let length = v.length();
                assert!((quat.to_world(v).length() - length).abs() < VECTOR_TOLERANCE);
                assert!((quat.to_body(v).length() - length).abs() < VECTOR_TOLERANCE);
            }
        }
    }
}
