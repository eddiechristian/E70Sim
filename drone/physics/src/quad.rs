//! Three axes, four motors: a quadcopter body held at its centre, free to
//! rotate any way but not to move.
//!
//! Explained in `Stage2-a.md` in the repo root.
//!
//! Body frame: x forward (roll, positive right side down), y right (pitch,
//! positive nose up), z down (yaw, positive nose right).

use crate::motor::Motor;
use crate::quat::{Angles, Quat, Rates};

/// How hard the body is to turn about each axis, in kg·m².
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Inertia {
    pub ixx: f32, // kg·m²; how hard it is to roll. Start with 0.015
    pub iyy: f32, // kg·m²; how hard it is to pitch. Start with 0.015
    pub izz: f32, // kg·m²; how hard it is to yaw. Start with 0.025
}

/// One command per motor, each 0.0 to 1.0. The motors are named by position,
/// seen from above. The mixer in the `flight` crate has its own type with the
/// same four names; the tests in `drone/sim/tests` copy one into the other.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MotorCommands {
    pub fl: f32, // front left, spins clockwise
    pub fr: f32, // front right, spins counter-clockwise
    pub br: f32, // back right, spins clockwise
    pub bl: f32, // back left, spins counter-clockwise
}

/// What the accelerometer reads, in m/s²: the push felt along each body
/// axis. Level and still it is `(0, 0, −9.80665)`, because z points down and
/// the push holding the drone up is upwards.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Accel {
    pub x: f32, // m/s²; positive towards the nose
    pub y: f32, // m/s²; positive towards the right side
    pub z: f32, // m/s²; positive towards the belly
}

/// Standard gravity, in m/s².
pub const G: f32 = 9.80665;

/// A torque on each axis from outside the drone, in N·m: a gust, a bump, a
/// hand. Explained in `Stage2-d.md` in the repo root.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Torques {
    pub roll: f32,  // N·m; positive pushes the right side down
    pub pitch: f32, // N·m; positive pushes the nose up
    pub yaw: f32,   // N·m; positive pushes the nose right
}

/// No push on any axis.
pub const NO_PUSH: Torques = Torques {
    roll: 0.0,
    pitch: 0.0,
    yaw: 0.0,
};

/// The four motors, named the same way as `MotorCommands`.
pub struct Motors {
    pub fl: Motor,
    pub fr: Motor,
    pub br: Motor,
    pub bl: Motor,
}

pub struct Quad {
    // Parameters
    pub arm_length: f32, // m; centre to motor. Start with 0.2
    pub yaw_coeff: f32,  // m; reaction torque per newton of thrust (`c`). Start with 0.015
    pub inertia: Inertia,

    // Parts
    pub motors: Motors,

    /// Added to the motors' torques on every step, until it is changed.
    /// Zero unless a test or the simulation sets it. The body knows nothing
    /// about time: the caller sets the push at the start of a kick and sets
    /// it back to `NO_PUSH` at the end.
    pub push: Torques,

    // State
    rates: Rates, // rad/s, body frame
    attitude: Quat,
}

impl Quad {
    pub fn new(arm_length: f32, yaw_coeff: f32, inertia: Inertia, motors: Motors) -> Self {
        Self {
            arm_length,
            yaw_coeff,
            inertia,
            motors,
            push: NO_PUSH,
            rates: Rates {
                roll: 0.0,
                pitch: 0.0,
                yaw: 0.0,
            },
            attitude: Quat::new(),
        }
    }

    /// Advances the model by `dt` seconds with the four motor commands.
    ///
    /// 1. Motor commands to thrusts: step each motor and read its thrust.
    /// 2. Thrusts to three torques, with d = arm_length / √2:
    ///    roll torque  = d × (T_FL + T_BL − T_FR − T_BR)
    ///    pitch torque = d × (T_FL + T_FR − T_BR − T_BL)
    ///    yaw torque   = yaw_coeff × (T_FR + T_BL − T_FL − T_BR)
    ///    Then add the push (`Stage2-d.md`): each of the three torques gets
    ///    its part of `self.push`.
    /// 3. Torques to rates, with the cross term (Euler's rotation equations):
    ///    roll rate  += (roll torque  − (izz − iyy) × pitch rate × yaw rate)   / ixx × dt
    ///    pitch rate += (pitch torque − (ixx − izz) × yaw rate   × roll rate)  / iyy × dt
    ///    yaw rate   += (yaw torque   − (iyy − ixx) × roll rate  × pitch rate) / izz × dt
    ///    Work out the three accelerations from the old rates first, then
    ///    update the three rates.
    /// 4. Rates to attitude: `Quat::update`.
    pub fn step(&mut self, commands: MotorCommands, dt: f32) {
        // 1. Motor commands to thrusts.
        self.motors.fl.step(commands.fl, dt);
        self.motors.fr.step(commands.fr, dt);
        self.motors.br.step(commands.br, dt);
        self.motors.bl.step(commands.bl, dt);
        let fl = self.motors.fl.thrust();
        let fr = self.motors.fr.thrust();
        let br = self.motors.br.thrust();
        let bl = self.motors.bl.thrust();

        // 2. Thrusts to three torques.
        let d = self.arm_length / 2.0_f32.sqrt();
        //    The push from outside is added to what the motors give.
        let roll_torque = d * (fl + bl - fr - br) + self.push.roll;
        let pitch_torque = d * (fl + fr - br - bl) + self.push.pitch;
        let yaw_torque = self.yaw_coeff * (fr + bl - fl - br) + self.push.yaw;

        // 3. Torques to rates. All three accelerations come from the old
        //    rates, before any rate is changed.
        let Inertia { ixx, iyy, izz } = self.inertia;
        let Rates { roll, pitch, yaw } = self.rates;
        let roll_accel = (roll_torque - (izz - iyy) * pitch * yaw) / ixx;
        let pitch_accel = (pitch_torque - (ixx - izz) * yaw * roll) / iyy;
        let yaw_accel = (yaw_torque - (iyy - ixx) * roll * pitch) / izz;
        self.rates.roll += roll_accel * dt;
        self.rates.pitch += pitch_accel * dt;
        self.rates.yaw += yaw_accel * dt;

        // 4. Rates to attitude.
        self.attitude.update(self.rates, dt);
    }

    /// The three rates in the body frame: what the three gyros read.
    pub fn rates(&self) -> Rates {
        self.rates
    }

    /// The true attitude as three angles.
    pub fn angles(&self) -> Angles {
        self.attitude.angles()
    }

    /// Puts the body at the given attitude, for a run that starts tilted
    /// (`Stage2-c.md`). The rates and the motors are left as they are: with
    /// zero rates and equal motor commands the body then stays there.
    pub fn set_attitude(&mut self, angles: Angles) {
        self.attitude = Quat::from_angles(angles);
    }

    /// What an accelerometer at the centre reads: the push of the ball joint
    /// that holds the body up, seen from the body's own axes. Explained in
    /// `Stage2-b.md` in the repo root.
    ///
    /// With w, x, y and z the four numbers of `self.attitude`:
    ///    ax = −G × 2 × (x×z − w×y)
    ///    ay = −G × 2 × (y×z + w×x)
    ///    az = −G × (1 − 2 × (x² + y²))
    pub fn accel(&self) -> Accel {
        let Quat { w, x, y, z } = self.attitude;
        Accel {
            x: -G * 2.0 * (x * z - w * y),
            y: -G * 2.0 * (y * z + w * x),
            z: -G * (1.0 - 2.0 * (x * x + y * y)),
        }
    }

    pub fn reset(&mut self) {
        self.motors.fl.reset();
        self.motors.fr.reset();
        self.motors.br.reset();
        self.motors.bl.reset();
        self.rates = Rates {
            roll: 0.0,
            pitch: 0.0,
            yaw: 0.0,
        };
        self.attitude = Quat::new();
        self.push = NO_PUSH;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const DT: f32 = 0.001;

    // An axis that should not be turning is compared with this, in rad/s.
    // Only equal commands give exactly 0; in the other cases the thrusts
    // cancel to within rounding.
    const ZERO_TOLERANCE: f32 = 1e-4;

    // A quad with the starting values.
    fn quad() -> Quad {
        Quad::new(
            0.2,
            0.015,
            Inertia {
                ixx: 0.015,
                iyy: 0.015,
                izz: 0.025,
            },
            Motors {
                fl: Motor::new(0.04, 6.0),
                fr: Motor::new(0.04, 6.0),
                br: Motor::new(0.04, 6.0),
                bl: Motor::new(0.04, 6.0),
            },
        )
    }

    // Steps `quad` with the same four commands for `seconds`.
    fn run(quad: &mut Quad, fl: f32, fr: f32, br: f32, bl: f32, seconds: f32) {
        let commands = MotorCommands { fl, fr, br, bl };
        let steps = (seconds / DT).round() as usize;
        for _ in 0..steps {
            quad.step(commands, DT);
        }
    }

    fn assert_near_zero(name: &str, rate: f32) {
        assert!(
            rate.abs() < ZERO_TOLERANCE,
            "{name} rate should be 0, got {rate} rad/s"
        );
    }

    #[test]
    fn test_equal_commands_no_rotation() {
        // All four at 0.7 for 1 s: all three rates exactly 0.
        let mut quad = quad();
        run(&mut quad, 0.7, 0.7, 0.7, 0.7, 1.0);
        let rates = quad.rates();
        assert_eq!(rates.roll, 0.0);
        assert_eq!(rates.pitch, 0.0);
        assert_eq!(rates.yaw, 0.0);
    }

    #[test]
    fn test_left_stronger_roll_positive() {
        // FL and BL at 0.75, FR and BR at 0.65: roll rate positive, pitch
        // and yaw rates 0.
        let mut quad = quad();
        run(&mut quad, 0.75, 0.65, 0.65, 0.75, 0.2);
        let rates = quad.rates();
        assert!(rates.roll > 1.0, "roll rate {} rad/s", rates.roll);
        assert_near_zero("pitch", rates.pitch);
        assert_near_zero("yaw", rates.yaw);
    }

    #[test]
    fn test_front_stronger_pitch_positive() {
        // FL and FR at 0.75, BR and BL at 0.65: pitch rate positive, the
        // others 0.
        let mut quad = quad();
        run(&mut quad, 0.75, 0.75, 0.65, 0.65, 0.2);
        let rates = quad.rates();
        assert!(rates.pitch > 1.0, "pitch rate {} rad/s", rates.pitch);
        assert_near_zero("roll", rates.roll);
        assert_near_zero("yaw", rates.yaw);
    }

    #[test]
    fn test_counter_clockwise_stronger_yaw_positive() {
        // FR and BL at 0.75, FL and BR at 0.65: yaw rate positive, the
        // others 0.
        let mut quad = quad();
        run(&mut quad, 0.65, 0.75, 0.65, 0.75, 0.2);
        let rates = quad.rates();
        assert!(rates.yaw > 0.05, "yaw rate {} rad/s", rates.yaw);
        assert_near_zero("roll", rates.roll);
        assert_near_zero("pitch", rates.pitch);
    }

    #[test]
    fn test_yaw_is_weak() {
        // The same size of command on roll and on yaw: the yaw rate is far
        // smaller.
        let mut rolling = quad();
        run(&mut rolling, 0.75, 0.65, 0.65, 0.75, 0.2);
        let mut yawing = quad();
        run(&mut yawing, 0.65, 0.75, 0.65, 0.75, 0.2);

        let roll_rate = rolling.rates().roll;
        let yaw_rate = yawing.rates().yaw;
        assert!(yaw_rate > 0.0);
        assert!(
            yaw_rate < roll_rate / 10.0,
            "yaw rate {yaw_rate} rad/s, roll rate {roll_rate} rad/s"
        );
    }

    #[test]
    fn test_no_drag_keeps_spinning() {
        // Push, then equal commands: the rates hold.
        let mut quad = quad();
        run(&mut quad, 0.75, 0.65, 0.65, 0.75, 0.2);

        // The motors take a moment to settle back to equal thrust, and the
        // rate still rises while they do. Compare two readings after that.
        run(&mut quad, 0.7, 0.7, 0.7, 0.7, 0.5);
        let settled = quad.rates().roll;
        run(&mut quad, 0.7, 0.7, 0.7, 0.7, 1.0);
        let later = quad.rates().roll;

        assert!(settled > 1.0, "roll rate {settled} rad/s");
        assert!(
            (later - settled).abs() < 1e-3,
            "roll rate went from {settled} to {later} rad/s"
        );
    }

    #[test]
    fn test_cross_term() {
        // Roll and yaw rates together, equal commands: a pitch rate appears.
        // `rates` is a private field, which this module can set directly.
        let mut quad = quad();
        quad.rates = Rates {
            roll: 1.0,
            pitch: 0.0,
            yaw: 1.0,
        };
        run(&mut quad, 0.7, 0.7, 0.7, 0.7, 0.1);
        // Pitch acceleration = −(ixx − izz) × yaw × roll / iyy
        //                    = −(0.015 − 0.025) × 1 × 1 / 0.015 = 0.667 rad/s²,
        // so about 0.067 rad/s after 0.1 s.
        let pitch = quad.rates().pitch;
        assert!((pitch - 0.067).abs() < 0.01, "pitch rate {pitch} rad/s");

        // With a roll rate alone there is nothing to couple, and no pitch
        // rate appears.
        let mut quad = self::quad();
        quad.rates = Rates {
            roll: 1.0,
            pitch: 0.0,
            yaw: 0.0,
        };
        run(&mut quad, 0.7, 0.7, 0.7, 0.7, 0.1);
        assert_eq!(quad.rates().pitch, 0.0);
    }

    // ---- The accelerometer reading. The attitude is a private field, which
    //      this module can turn directly with `quad.attitude.update(..)`.

    // Turns the attitude of `quad` at the given rates, in °/s, for
    // `seconds`, without going through the motors.
    fn turn(quad: &mut Quad, roll: f32, pitch: f32, yaw: f32, seconds: f32) {
        let rates = Rates {
            roll: roll.to_radians(),
            pitch: pitch.to_radians(),
            yaw: yaw.to_radians(),
        };
        let steps = (seconds / DT).round() as usize;
        for _ in 0..steps {
            quad.attitude.update(rates, DT);
        }
    }

    // Checks an accelerometer reading against the expected one, in m/s².
    fn assert_accel(accel: Accel, x: f32, y: f32, z: f32) {
        let got = [accel.x, accel.y, accel.z];
        let expected = [x, y, z];
        for (name, (got, expected)) in ["ax", "ay", "az"]
            .iter()
            .zip(got.iter().zip(expected.iter()))
        {
            assert!(
                (got - expected).abs() < 0.005,
                "{name}: got {got}, expected {expected}"
            );
        }
    }

    #[test]
    fn test_accel_level() {
        // A new quad: (0, 0, −9.807).
        let quad = quad();
        assert_accel(quad.accel(), 0.0, 0.0, -9.807);
    }

    #[test]
    fn test_accel_pitched() {
        // Turned 30° about pitch: (4.903, 0, −8.493).
        let mut quad = quad();
        turn(&mut quad, 0.0, 30.0, 0.0, 1.0);
        assert_accel(quad.accel(), 4.903, 0.0, -8.493);
    }

    #[test]
    fn test_accel_rolled() {
        // Turned 30° about roll: (0, −4.903, −8.493).
        let mut quad = quad();
        turn(&mut quad, 30.0, 0.0, 0.0, 1.0);
        assert_accel(quad.accel(), 0.0, -4.903, -8.493);
    }

    #[test]
    fn test_accel_negative_angles() {
        // The signs reverse with the angles: nose down gives a negative ax,
        // left side down a positive ay.
        let mut quad = quad();
        turn(&mut quad, 0.0, -30.0, 0.0, 1.0);
        assert_accel(quad.accel(), -4.903, 0.0, -8.493);

        let mut quad = self::quad();
        turn(&mut quad, -30.0, 0.0, 0.0, 1.0);
        assert_accel(quad.accel(), 0.0, 4.903, -8.493);
    }

    #[test]
    fn test_accel_level_any_yaw() {
        // Level, turned any amount about yaw: still (0, 0, −9.807).
        let mut quad = quad();
        turn(&mut quad, 0.0, 0.0, 77.0, 1.0);
        assert_accel(quad.accel(), 0.0, 0.0, -9.807);
    }

    #[test]
    fn test_accel_is_always_one_g() {
        // Any attitude: the length √(ax² + ay² + az²) is 9.807.
        let mut quad = quad();
        for _ in 0..10 {
            turn(&mut quad, 200.0, -150.0, 300.0, 0.37);
            let accel = quad.accel();
            let length = (accel.x * accel.x + accel.y * accel.y + accel.z * accel.z).sqrt();
            assert!((length - G).abs() < 0.001, "length {length}");
        }
    }

    // ---- Starting tilted

    #[test]
    fn test_set_attitude() {
        // Set to roll 30°: `angles()` gives roll 30°, pitch 0°, yaw 0°, and
        // the accelerometer reads (0, −4.903, −8.493), as in
        // `test_accel_rolled`.
        let mut quad = quad();
        quad.set_attitude(Angles {
            roll: 30f32.to_radians(),
            pitch: 0.0,
            yaw: 0.0,
        });
        let angles = quad.angles();
        assert!((angles.roll.to_degrees() - 30.0).abs() < 0.01);
        assert!(angles.pitch.to_degrees().abs() < 0.01);
        assert!(angles.yaw.to_degrees().abs() < 0.01);
        assert_accel(quad.accel(), 0.0, -4.903, -8.493);
    }

    #[test]
    fn test_set_attitude_stays() {
        // Set to roll 20° and pitch −10°, then equal commands for 1 s: the
        // angles do not change and the rates stay 0.
        let mut quad = quad();
        quad.set_attitude(Angles {
            roll: 20f32.to_radians(),
            pitch: -10f32.to_radians(),
            yaw: 0.0,
        });
        run(&mut quad, 0.7, 0.7, 0.7, 0.7, 1.0);

        let angles = quad.angles();
        assert!((angles.roll.to_degrees() - 20.0).abs() < 0.01);
        assert!((angles.pitch.to_degrees() + 10.0).abs() < 0.01);
        assert!(angles.yaw.to_degrees().abs() < 0.01);

        let rates = quad.rates();
        assert_eq!(rates.roll, 0.0);
        assert_eq!(rates.pitch, 0.0);
        assert_eq!(rates.yaw, 0.0);
    }

    // ---- The push. Named after the table in `Stage2-d.md`.

    // The standard kick of Stage 2d.
    const KICK: f32 = 0.3; // N·m
    const KICK_FOR: f32 = 0.05; // s

    // A rate that a push should have given is compared with this, in rad/s.
    const PUSH_TOLERANCE: f32 = 1e-3;

    #[test]
    fn test_no_push_no_change() {
        // Push zero, all four at 0.7 for 1 s: the rates stay exactly 0, as
        // in `test_equal_commands_no_rotation`.
        let mut quad = quad();
        assert_eq!(quad.push, NO_PUSH);
        run(&mut quad, 0.7, 0.7, 0.7, 0.7, 1.0);
        let rates = quad.rates();
        assert_eq!(rates.roll, 0.0);
        assert_eq!(rates.pitch, 0.0);
        assert_eq!(rates.yaw, 0.0);
    }

    #[test]
    fn test_roll_push() {
        // 0.3 N·m about roll for 0.05 s, all four at 0.7:
        // change in rate = torque × time / inertia = 0.3 × 0.05 / 0.015,
        // so the roll rate is 1 rad/s. Pitch and yaw rates are 0.
        let mut quad = quad();
        quad.push = Torques {
            roll: KICK,
            ..NO_PUSH
        };
        run(&mut quad, 0.7, 0.7, 0.7, 0.7, KICK_FOR);
        let rates = quad.rates();
        assert!(
            (rates.roll - 1.0).abs() < PUSH_TOLERANCE,
            "roll rate {} rad/s",
            rates.roll
        );
        assert_near_zero("pitch", rates.pitch);
        assert_near_zero("yaw", rates.yaw);

        // A push the other way turns it the other way.
        let mut quad = self::quad();
        quad.push = Torques {
            roll: -KICK,
            ..NO_PUSH
        };
        run(&mut quad, 0.7, 0.7, 0.7, 0.7, KICK_FOR);
        let roll = quad.rates().roll;
        assert!(
            (roll + 1.0).abs() < PUSH_TOLERANCE,
            "roll rate {roll} rad/s"
        );
    }

    #[test]
    fn test_pitch_and_yaw_push() {
        // The same about pitch, and then about yaw on a new quad: that axis
        // alone turns, at torque × time / inertia. Pitch gives 1 rad/s and
        // yaw 0.3 × 0.05 / 0.025 = 0.6 rad/s.
        let mut quad = quad();
        quad.push = Torques {
            pitch: KICK,
            ..NO_PUSH
        };
        run(&mut quad, 0.7, 0.7, 0.7, 0.7, KICK_FOR);
        let rates = quad.rates();
        assert!(
            (rates.pitch - 1.0).abs() < PUSH_TOLERANCE,
            "pitch rate {} rad/s",
            rates.pitch
        );
        assert_near_zero("roll", rates.roll);
        assert_near_zero("yaw", rates.yaw);

        let mut quad = self::quad();
        quad.push = Torques {
            yaw: KICK,
            ..NO_PUSH
        };
        run(&mut quad, 0.7, 0.7, 0.7, 0.7, KICK_FOR);
        let rates = quad.rates();
        assert!(
            (rates.yaw - 0.6).abs() < PUSH_TOLERANCE,
            "yaw rate {} rad/s",
            rates.yaw
        );
        assert_near_zero("roll", rates.roll);
        assert_near_zero("pitch", rates.pitch);
    }

    #[test]
    fn test_push_stops() {
        // A roll push for 0.05 s, then `NO_PUSH` for 1 s: the roll rate
        // stays where the push left it, since the model has no drag.
        let mut quad = quad();
        quad.push = Torques {
            roll: KICK,
            ..NO_PUSH
        };
        run(&mut quad, 0.7, 0.7, 0.7, 0.7, KICK_FOR);
        let pushed = quad.rates().roll;
        assert!(pushed > 0.9, "roll rate {pushed} rad/s");

        quad.push = NO_PUSH;
        run(&mut quad, 0.7, 0.7, 0.7, 0.7, 1.0);
        let later = quad.rates().roll;
        assert_eq!(later, pushed);
    }

    #[test]
    fn test_reset_clears_push() {
        // A push set, then `reset()`: the push is `NO_PUSH`, and equal
        // commands leave the rates at 0.
        let mut quad = quad();
        quad.push = Torques {
            roll: KICK,
            pitch: -KICK,
            yaw: KICK,
        };
        run(&mut quad, 0.7, 0.7, 0.7, 0.7, KICK_FOR);

        quad.reset();
        assert_eq!(quad.push, NO_PUSH);
        run(&mut quad, 0.7, 0.7, 0.7, 0.7, 1.0);
        let rates = quad.rates();
        assert_eq!(rates.roll, 0.0);
        assert_eq!(rates.pitch, 0.0);
        assert_eq!(rates.yaw, 0.0);
    }

    #[test]
    fn test_reset() {
        // After a reset it behaves the same as a brand-new quad.
        let mut used = quad();
        run(&mut used, 0.75, 0.65, 0.6, 0.8, 0.3);
        used.reset();
        let mut fresh = quad();

        assert_eq!(used.rates(), fresh.rates());
        assert_eq!(used.angles(), fresh.angles());

        run(&mut used, 0.75, 0.65, 0.6, 0.8, 0.3);
        run(&mut fresh, 0.75, 0.65, 0.6, 0.8, 0.3);
        assert_eq!(used.rates(), fresh.rates());
        assert_eq!(used.angles(), fresh.angles());
    }
}
