//! Three numbers along x, y and z: a position, a velocity or a force.
//!
//! Explained in `Stage3-a.md` in the repo root. A `Vec3` does not say which
//! axes it is measured along. The comment where one is declared has to: the
//! world's (north, east, down) or the body's (forward, right, down).
//! `Quat::to_world` and `Quat::to_body` turn one into the other.

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Vec3 {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

impl Vec3 {
    pub const ZERO: Vec3 = Vec3 {
        x: 0.0,
        y: 0.0,
        z: 0.0,
    };

    /// √(x² + y² + z²).
    pub fn length(&self) -> f32 {
        (self.x * self.x + self.y * self.y + self.z * self.z).sqrt()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_length() {
        // (3, 4, 0) is 5 long, (0, 0, −2) is 2 long, and `ZERO` is 0 long.
        let v = Vec3 {
            x: 3.0,
            y: 4.0,
            z: 0.0,
        };
        assert_eq!(v.length(), 5.0);

        let v = Vec3 {
            x: 0.0,
            y: 0.0,
            z: -2.0,
        };
        assert_eq!(v.length(), 2.0);

        assert_eq!(Vec3::ZERO.length(), 0.0);
    }
}
