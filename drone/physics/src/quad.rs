//! Three axes, four motors: a quadcopter body. Held at its centre by a ball
//! joint, it is free to rotate any way but not to move. With the joint taken
//! away it has a weight and a position: it flies, falls, and stands on the
//! ground.
//!
//! The rotation is explained in `Stage2-a.md` in the repo root, and the
//! free body in `Stage3-a.md`.
//!
//! Body frame: x forward (roll, positive right side down), y right (pitch,
//! positive nose up), z down (yaw, positive nose right).

use crate::motor::Motor;
use crate::quat::{Angles, Quat, Rates};
use crate::vec3::Vec3;

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

/// What holds the drone up when its motors do not.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Support {
    /// A ball joint at the centre: it turns but goes nowhere. Stage 2.
    Joint,
    /// Nothing: it flies, falls, and stands on the ground. Stage 3.
    Free,
}

/// How the drone last arrived on the ground. The physics does not judge it;
/// whether it was a landing or a crash is for the caller to say.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Touchdown {
    pub speed_down: f32,     // m/s; how fast it was coming down
    pub speed_sideways: f32, // m/s; how fast it was moving over the ground
    pub tilt: f32,           // rad; the angle between the body's z axis and straight down
}

/// What a free body standing still on the ground feels, in m/s², world
/// frame: the ground's push, 1 g upwards.
const FELT_AT_REST: Vec3 = Vec3 {
    x: 0.0,
    y: 0.0,
    z: -G,
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
    pub mass: f32, // kg; not used on the joint. Start with 1.2
    pub drag: f32, // N per m/s of speed, against the movement; not used on the joint. Start with 0.4
    pub support: Support,

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

    // State of a free body. On the joint these never change.
    position: Vec3, // m, world frame: north, east, down. The ground is at z = 0, so height is −z
    velocity: Vec3, // m/s, world frame
    on_ground: bool, // the ground is holding it: it neither moves nor turns
    touchdown: Option<Touchdown>, // how it last arrived on the ground

    // What the accelerometer feels, in m/s², world frame: every force but
    // gravity, divided by the mass. `step_free` works it out and `accel`
    // turns it into the body frame.
    felt: Vec3,
}

impl Quad {
    /// A body on the ball joint, as in Stage 2.
    pub fn new(arm_length: f32, yaw_coeff: f32, inertia: Inertia, motors: Motors) -> Self {
        Self {
            arm_length,
            yaw_coeff,
            inertia,
            mass: 0.0,
            drag: 0.0,
            support: Support::Joint,
            motors,
            push: NO_PUSH,
            rates: Rates {
                roll: 0.0,
                pitch: 0.0,
                yaw: 0.0,
            },
            attitude: Quat::new(),
            position: Vec3::ZERO,
            velocity: Vec3::ZERO,
            on_ground: false,
            touchdown: None,
            felt: FELT_AT_REST,
        }
    }

    /// A free body with the given mass, in kg, and drag, in N per m/s. It
    /// starts standing on the ground, level, at the origin.
    pub fn new_free(
        arm_length: f32,
        yaw_coeff: f32,
        inertia: Inertia,
        motors: Motors,
        mass: f32,
        drag: f32,
    ) -> Self {
        Self {
            mass,
            drag,
            support: Support::Free,
            on_ground: true,
            ..Self::new(arm_length, yaw_coeff, inertia, motors)
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
    ///
    /// A free body standing on the ground skips steps 3 and 4: the ground
    /// lets it neither turn nor keep a rate. Then, for a free body only:
    ///
    /// 5. Forces to movement, and the ground: `step_free`.
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

        // The ground holds a body that stands on it. On the joint
        // `on_ground` is never true.
        if !self.on_ground {
            // 3. Torques to rates. All three accelerations come from the
            //    old rates, before any rate is changed.
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

        // 5. Forces to movement, and the ground.
        if self.support == Support::Free {
            self.step_free(fl + fr + br + bl, dt);
        }
    }

    /// Step 5, for a free body: moves it by `dt` seconds with `thrust`, the
    /// four motors' thrusts added up, in newtons. Explained in `Stage3-a.md`.
    ///
    /// 1. The forces other than gravity, in the world frame:
    ///    thrust force = `self.attitude.to_world((0, 0, −thrust))`
    ///    drag         = −`self.drag` × `self.velocity`, axis by axis
    ///    push         = thrust force + drag
    ///
    /// 2. On the ground, and staying there: when `self.on_ground` and the
    ///    push does not lift the weight, which is `push.z + mass × G ≥ 0`
    ///    since z points down. The velocity stays zero, `self.felt` is
    ///    `FELT_AT_REST`, and there is nothing more to do.
    ///
    /// 3. Otherwise it is flying. Clear `self.on_ground`, then:
    ///    acceleration = push / mass, with `G` added to its z part
    ///    velocity    += acceleration × dt
    ///    position    += velocity × dt
    ///    `self.felt`  = push / mass
    ///
    /// 4. Landing: when `position.z` has reached 0 or passed it. Record a
    ///    `Touchdown` from the velocity it arrived with and from the tilt,
    ///    which is `acos` of the z part of `to_world((0, 0, 1))`. Then put
    ///    `position.z` at 0, the velocity and the three rates at zero, set
    ///    `self.on_ground`, and make `self.felt` `FELT_AT_REST`.
    fn step_free(&mut self, thrust: f32, dt: f32) {
        // 1. The forces other than gravity. Thrust pushes along the body's
        //    −z axis, wherever that points in the world.
        let thrust_force = self.attitude.to_world(Vec3 {
            x: 0.0,
            y: 0.0,
            z: -thrust,
        });
        let push = Vec3 {
            x: thrust_force.x - self.drag * self.velocity.x,
            y: thrust_force.y - self.drag * self.velocity.y,
            z: thrust_force.z - self.drag * self.velocity.z,
        };

        // 2. On the ground, and staying there: the push does not lift the
        //    weight. z points down, so a lifting push is negative.
        let weight = self.mass * G;
        if self.on_ground && push.z + weight >= 0.0 {
            self.velocity = Vec3::ZERO;
            self.felt = FELT_AT_REST;
            return;
        }

        // 3. Flying. Gravity is the one force left out of the push.
        self.on_ground = false;
        self.velocity.x += push.x / self.mass * dt;
        self.velocity.y += push.y / self.mass * dt;
        self.velocity.z += (push.z / self.mass + G) * dt;
        self.position.x += self.velocity.x * dt;
        self.position.y += self.velocity.y * dt;
        self.position.z += self.velocity.z * dt;
        self.felt = Vec3 {
            x: push.x / self.mass,
            y: push.y / self.mass,
            z: push.z / self.mass,
        };

        // 4. Landing: it has reached the ground or passed it.
        if self.position.z >= 0.0 {
            // How far the body's z axis is from straight down. Rounding can
            // put the cosine a hair outside −1..1, and `acos` then gives NaN.
            let belly = self.attitude.to_world(Vec3 {
                x: 0.0,
                y: 0.0,
                z: 1.0,
            });
            let sideways = Vec3 {
                z: 0.0,
                ..self.velocity
            };
            self.touchdown = Some(Touchdown {
                speed_down: self.velocity.z,
                speed_sideways: sideways.length(),
                tilt: belly.z.clamp(-1.0, 1.0).acos(),
            });

            self.position.z = 0.0;
            self.velocity = Vec3::ZERO;
            self.rates = Rates {
                roll: 0.0,
                pitch: 0.0,
                yaw: 0.0,
            };
            self.on_ground = true;
            self.felt = FELT_AT_REST;
        }
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

    /// Where the centre of a free body is, in metres, world frame. Always
    /// the origin on the joint.
    pub fn position(&self) -> Vec3 {
        self.position
    }

    /// How fast a free body is moving, in m/s, world frame.
    pub fn velocity(&self) -> Vec3 {
        self.velocity
    }

    /// Metres above the ground: `−position.z`, since z points down.
    pub fn height(&self) -> f32 {
        -self.position.z
    }

    /// Whether the ground is holding the body. Never true on the joint.
    pub fn on_ground(&self) -> bool {
        self.on_ground
    }

    /// How the body last arrived on the ground. `None` if it has not landed
    /// since it was made or reset; starting on the ground is not a landing.
    pub fn touchdown(&self) -> Option<Touchdown> {
        self.touchdown
    }

    /// Puts a free body at `position`, at rest, for a test or a run that
    /// starts in the air. Above the ground it is flying from the next step;
    /// at z = 0 or below it stands on the ground.
    pub fn set_position(&mut self, position: Vec3) {
        self.position = position;
        self.velocity = Vec3::ZERO;
        self.on_ground = self.support == Support::Free && position.z >= 0.0;
    }

    /// The command that, on all four motors, holds the drone's weight:
    ///    √(mass × G / (thrust_fl + thrust_fr + thrust_br + thrust_bl))
    /// where each thrust is that motor's `thrust_coeff`, its thrust at full
    /// speed. With four motors of 6 N and 1.2 kg it is 0.7002.
    pub fn hover_command(&self) -> f32 {
        let full_thrust = self.motors.fl.thrust_coeff
            + self.motors.fr.thrust_coeff
            + self.motors.br.thrust_coeff
            + self.motors.bl.thrust_coeff;
        (self.mass * G / full_thrust).sqrt()
    }

    /// What an accelerometer at the centre reads: the push on the body from
    /// everything except gravity, seen from the body's own axes.
    ///
    /// On the joint that push is the joint's, 1 g straight up, as explained
    /// in `Stage2-b.md` in the repo root. With w, x, y and z the four
    /// numbers of `self.attitude`:
    ///    ax = −G × 2 × (x×z − w×y)
    ///    ay = −G × 2 × (y×z + w×x)
    ///    az = −G × (1 − 2 × (x² + y²))
    ///
    /// A free body is pushed by its thrust, by drag and by the ground:
    /// `accel_free`.
    pub fn accel(&self) -> Accel {
        if self.support == Support::Free {
            return self.accel_free();
        }
        let Quat { w, x, y, z } = self.attitude;
        Accel {
            x: -G * 2.0 * (x * z - w * y),
            y: -G * 2.0 * (y * z + w * x),
            z: -G * (1.0 - 2.0 * (x * x + y * y)),
        }
    }

    /// The reading of a free body (`Stage3-a.md`): `self.felt`, which
    /// `step_free` keeps in the world frame, turned into the body frame with
    /// `self.attitude.to_body`.
    ///
    /// Standing on the ground `self.felt` is 1 g upwards, so this gives what
    /// the joint gave at the same attitude, with no special case.
    fn accel_free(&self) -> Accel {
        let felt = self.attitude.to_body(self.felt);
        Accel {
            x: felt.x,
            y: felt.y,
            z: felt.z,
        }
    }

    /// Back to how it was made: level, at rest, the motors stopped. A free
    /// body is back on the ground at the origin. The parameters are kept.
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
        self.position = Vec3::ZERO;
        self.velocity = Vec3::ZERO;
        self.on_ground = self.support == Support::Free;
        self.touchdown = None;
        self.felt = FELT_AT_REST;
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

    // ---- The free body. Named after the tables in `Stage3-a.md`.

    const MASS: f32 = 1.2; // kg
    const DRAG: f32 = 0.4; // N per m/s

    // A free quad with the starting values, standing on the ground. For a
    // run with no drag, set `quad.drag = 0.0`.
    fn free_quad() -> Quad {
        Quad::new_free(
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
            MASS,
            DRAG,
        )
    }

    // A free quad at rest `height` metres up, at the given roll and pitch
    // in degrees. With equal motor commands nothing turns it, so it keeps
    // that attitude: this is what "held at 10°" means in the tests below.
    //
    // Its motors start stopped and take about 0.2 s to reach their command.
    // A test that needs them at speed from the first step calls `spin_up`.
    fn free_quad_at(height: f32, roll: f32, pitch: f32) -> Quad {
        let mut quad = free_quad();
        quad.set_attitude(Angles {
            roll: roll.to_radians(),
            pitch: pitch.to_radians(),
            yaw: 0.0,
        });
        quad.set_position(Vec3 {
            x: 0.0,
            y: 0.0,
            z: -height,
        });
        quad
    }

    // Brings the four motors to `command` without the body moving: a
    // second of stepping the motors alone, which is 25 time constants.
    fn spin_up(quad: &mut Quad, command: f32) {
        for _ in 0..1000 {
            quad.motors.fl.step(command, DT);
            quad.motors.fr.step(command, DT);
            quad.motors.br.step(command, DT);
            quad.motors.bl.step(command, DT);
        }
    }

    // Steps `quad` with all four motors at `command` for `seconds`.
    fn run_all(quad: &mut Quad, command: f32, seconds: f32) {
        run(quad, command, command, command, command, seconds);
    }

    // A free quad in the air as `free_quad_at`, with its motors already at
    // the hover command.
    fn hovering_at(height: f32, roll: f32, pitch: f32) -> (Quad, f32) {
        let mut quad = free_quad_at(height, roll, pitch);
        let hover = quad.hover_command();
        spin_up(&mut quad, hover);
        (quad, hover)
    }

    fn assert_at_rest(quad: &Quad) {
        assert_eq!(quad.velocity(), Vec3::ZERO);
        let rates = quad.rates();
        assert_eq!(rates.roll, 0.0);
        assert_eq!(rates.pitch, 0.0);
        assert_eq!(rates.yaw, 0.0);
    }

    #[test]
    fn test_hover_command() {
        // Four motors of 6 N and 1.2 kg: 0.7002.
        let hover = free_quad().hover_command();
        assert!((hover - 0.7002).abs() < 1e-4, "hover command {hover}");

        // A heavier drone needs more, and stronger motors need less.
        let mut heavy = free_quad();
        heavy.mass = 1.5;
        assert!(heavy.hover_command() > hover);
        let mut strong = free_quad();
        strong.motors.fl.thrust_coeff = 8.0;
        assert!(strong.hover_command() < hover);
    }

    #[test]
    fn test_joint_is_unchanged() {
        // `Support::Joint`, uneven commands for 1 s: position and velocity
        // stay zero, it is not on the ground, and `accel` is 1 g long.
        let mut quad = quad();
        assert_eq!(quad.support, Support::Joint);
        run(&mut quad, 0.75, 0.65, 0.6, 0.8, 1.0);

        assert_eq!(quad.position(), Vec3::ZERO);
        assert_eq!(quad.velocity(), Vec3::ZERO);
        assert_eq!(quad.height(), 0.0);
        assert!(!quad.on_ground());
        assert_eq!(quad.touchdown(), None);

        // It did turn, and the joint still pushes with 1 g.
        assert!(quad.rates().roll.abs() > 0.1);
        let accel = quad.accel();
        let length = (accel.x * accel.x + accel.y * accel.y + accel.z * accel.z).sqrt();
        assert!((length - G).abs() < 0.001, "length {length}");
    }

    #[test]
    fn test_stands_on_the_ground() {
        // Free, motors off, 1 s: on the ground, height 0, velocity 0.
        let mut quad = free_quad();
        assert!(quad.on_ground());
        run_all(&mut quad, 0.0, 1.0);

        assert!(quad.on_ground());
        assert_eq!(quad.height(), 0.0);
        assert_eq!(quad.position(), Vec3::ZERO);
        assert_at_rest(&quad);
        assert_eq!(quad.touchdown(), None);
    }

    #[test]
    fn test_falls() {
        // Free, starting 10 m up, motors off, no drag, 1 s: 4.9 m lower and
        // falling at 9.8 m/s.
        let mut quad = free_quad_at(10.0, 0.0, 0.0);
        quad.drag = 0.0;
        assert!(!quad.on_ground());
        run_all(&mut quad, 0.0, 1.0);

        // ½ × G × 1² = 4.9 m, and G × 1 = 9.8 m/s. Stepping at 1 ms puts the
        // distance a few millimetres out.
        let fallen = 10.0 - quad.height();
        assert!((fallen - 4.903).abs() < 0.02, "fell {fallen} m");
        let speed = quad.velocity().z;
        assert!((speed - 9.807).abs() < 0.01, "falling at {speed} m/s");
        // Straight down.
        assert_eq!(quad.position().x, 0.0);
        assert_eq!(quad.position().y, 0.0);
    }

    #[test]
    fn test_hovers() {
        // In the air with the motors at speed, all four at the hover
        // command: the height changes by less than 1 cm in 5 s.
        let (mut quad, hover) = hovering_at(10.0, 0.0, 0.0);
        run_all(&mut quad, hover, 5.0);

        let height = quad.height();
        assert!((height - 10.0).abs() < 0.01, "height {height} m");
        assert!(!quad.on_ground());
    }

    #[test]
    fn test_climbs() {
        // All four at 0.76, no drag: once the motors are at speed, the
        // climb speeds up at 1.7 m/s².
        // (4 × 6 × 0.76² − 1.2 × 9.807) / 1.2 = 1.74
        let mut quad = free_quad_at(10.0, 0.0, 0.0);
        quad.drag = 0.0;
        spin_up(&mut quad, 0.76);
        run_all(&mut quad, 0.76, 1.0);

        // Up is negative z.
        let climb = -quad.velocity().z;
        assert!(
            (climb - 1.745).abs() < 0.02,
            "climbing at {climb} m/s after 1 s"
        );
        assert!(quad.height() > 10.0);
    }

    #[test]
    fn test_climb_settles() {
        // The same with drag, 20 s: climbing at 5.2 m/s, where the drag
        // equals the spare thrust. 2.09 N / 0.4 = 5.2
        let mut quad = free_quad_at(10.0, 0.0, 0.0);
        spin_up(&mut quad, 0.76);
        run_all(&mut quad, 0.76, 20.0);

        let climb = -quad.velocity().z;
        assert!((climb - 5.24).abs() < 0.05, "climbing at {climb} m/s");

        // It has stopped speeding up.
        run_all(&mut quad, 0.76, 1.0);
        let later = -quad.velocity().z;
        assert!((later - climb).abs() < 0.01, "from {climb} to {later} m/s");
    }

    #[test]
    fn test_leaves_the_ground() {
        // On the ground at a command of 0.6 for 1 s: still on the ground,
        // since 4 × 6 × 0.6² = 8.6 N is less than the weight of 11.8 N.
        // Raised to 0.76: off the ground within 0.2 s.
        let mut quad = free_quad();
        run_all(&mut quad, 0.6, 1.0);
        assert!(quad.on_ground());
        assert_eq!(quad.height(), 0.0);

        run_all(&mut quad, 0.76, 0.2);
        assert!(!quad.on_ground());
        assert!(quad.height() > 0.0);
        // Leaving the ground is not a landing.
        assert_eq!(quad.touchdown(), None);
    }

    #[test]
    fn test_nose_down_goes_forward() {
        // Held at −10° of pitch, hover command: x grows and y stays 0.
        let (mut quad, hover) = hovering_at(10.0, 0.0, -10.0);
        run_all(&mut quad, hover, 1.0);

        // G × sin 10° = 1.7 m/s², so about 0.8 m in the first second.
        let position = quad.position();
        assert!(position.x > 0.5, "x {} m", position.x);
        assert!(position.y.abs() < 1e-3, "y {} m", position.y);

        // Nose up goes backwards.
        let (mut quad, hover) = hovering_at(10.0, 0.0, 10.0);
        run_all(&mut quad, hover, 1.0);
        assert!(quad.position().x < -0.5);
    }

    #[test]
    fn test_right_side_down_goes_right() {
        // Held at 10° of roll, hover command: y grows and x stays 0.
        let (mut quad, hover) = hovering_at(10.0, 10.0, 0.0);
        run_all(&mut quad, hover, 1.0);

        let position = quad.position();
        assert!(position.y > 0.5, "y {} m", position.y);
        assert!(position.x.abs() < 1e-3, "x {} m", position.x);

        // Left side down goes left.
        let (mut quad, hover) = hovering_at(10.0, -10.0, 0.0);
        run_all(&mut quad, hover, 1.0);
        assert!(quad.position().y < -0.5);
    }

    #[test]
    fn test_a_tilt_costs_lift() {
        // Held at 30°, hover command, no drag: sinks, speeding up at
        // G × (1 − cos 30°) = 1.3 m/s².
        let (mut quad, hover) = hovering_at(50.0, 30.0, 0.0);
        quad.drag = 0.0;
        run_all(&mut quad, hover, 1.0);

        // 9.807 × (1 − 0.866) = 1.314 m/s after 1 s. Down is positive z.
        let sink = quad.velocity().z;
        assert!(
            (sink - 1.314).abs() < 0.02,
            "sinking at {sink} m/s after 1 s"
        );

        // Level, the same command holds the height.
        let (mut level, hover) = hovering_at(50.0, 0.0, 0.0);
        level.drag = 0.0;
        run_all(&mut level, hover, 1.0);
        assert!(level.velocity().z.abs() < 0.001);
    }

    #[test]
    fn test_speed_settles() {
        // Held at 10° of roll for 20 s: 5.1 m/s sideways, where the drag
        // equals the sideways part of the thrust.
        // 1.2 × 9.807 × sin 10° / 0.4 = 5.1
        let (mut quad, hover) = hovering_at(100.0, 10.0, 0.0);
        run_all(&mut quad, hover, 20.0);

        let speed = quad.velocity().y;
        assert!((speed - 5.11).abs() < 0.05, "sideways at {speed} m/s");
        assert!(quad.velocity().x.abs() < 1e-3);

        // It has stopped speeding up.
        run_all(&mut quad, hover, 1.0);
        let later = quad.velocity().y;
        assert!((later - speed).abs() < 0.01, "from {speed} to {later} m/s");
    }

    #[test]
    fn test_coasts_to_a_stop() {
        // Level at the hover command, moving at 5 m/s: 37% of the speed is
        // left after 3 s, which is mass / drag.
        // `velocity` is a private field, which this module can set.
        let (mut quad, hover) = hovering_at(10.0, 0.0, 0.0);
        quad.velocity = Vec3 {
            x: 5.0,
            y: 0.0,
            z: 0.0,
        };
        run_all(&mut quad, hover, 3.0);

        // e⁻¹ = 0.368 of it is left after one time constant.
        let left = quad.velocity().x / 5.0;
        assert!(
            (left - 0.368).abs() < 0.01,
            "{}% of the speed left",
            left * 100.0
        );

        // With no drag it does not slow down at all.
        let (mut quad, hover) = hovering_at(10.0, 0.0, 0.0);
        quad.drag = 0.0;
        quad.velocity = Vec3 {
            x: 5.0,
            y: 0.0,
            z: 0.0,
        };
        run_all(&mut quad, hover, 3.0);
        assert!((quad.velocity().x - 5.0).abs() < 1e-3);
    }

    #[test]
    fn test_lands() {
        // Coming down at 2 m/s onto the ground: stops at height 0, on the
        // ground, with zero velocity, and `touchdown()` gives 2 m/s down.
        // Before that, `touchdown()` is `None`.
        //
        // Dropped from 0.204 m with the motors off and no drag, it arrives
        // at √(2 × G × 0.204) = 2.0 m/s.
        let mut quad = free_quad_at(0.204, 0.0, 0.0);
        quad.drag = 0.0;
        assert_eq!(quad.touchdown(), None);
        run_all(&mut quad, 0.0, 0.1);
        assert!(!quad.on_ground());
        assert_eq!(quad.touchdown(), None);

        run_all(&mut quad, 0.0, 0.4);
        assert!(quad.on_ground());
        assert_eq!(quad.height(), 0.0);
        assert_at_rest(&quad);

        let touchdown = quad.touchdown().expect("it landed");
        assert!(
            (touchdown.speed_down - 2.0).abs() < 0.03,
            "arrived at {} m/s",
            touchdown.speed_down
        );
        assert!(touchdown.speed_sideways.abs() < 1e-3);
        assert!(touchdown.tilt.to_degrees().abs() < 0.1);

        // Arriving tilted and moving sideways is recorded too.
        let mut quad = free_quad_at(0.204, 20.0, 0.0);
        quad.drag = 0.0;
        quad.velocity = Vec3 {
            x: 0.0,
            y: 1.5,
            z: 0.0,
        };
        run_all(&mut quad, 0.0, 0.5);
        let touchdown = quad.touchdown().expect("it landed");
        assert!((touchdown.speed_sideways - 1.5).abs() < 0.01);
        assert!((touchdown.tilt.to_degrees() - 20.0).abs() < 0.1);
        // It stays at the angle it landed at.
        assert!((quad.angles().roll.to_degrees() - 20.0).abs() < 0.1);
    }

    #[test]
    fn test_held_on_the_ground() {
        // On the ground, uneven motor commands that do not lift the weight,
        // 1 s: the rates stay 0 and the attitude does not change.
        //
        // 6 × (0.3² + 0.5² + 0.2² + 0.4²) = 3.2 N, against a weight of 11.8 N.
        let mut quad = free_quad();
        run(&mut quad, 0.3, 0.5, 0.2, 0.4, 1.0);
        assert!(quad.on_ground());
        assert_at_rest(&quad);
        let angles = quad.angles();
        assert_eq!(angles.roll, 0.0);
        assert_eq!(angles.pitch, 0.0);
        assert_eq!(angles.yaw, 0.0);

        // The same on a slope of 20°: it keeps the angle it was set down at.
        let mut quad = free_quad();
        quad.set_attitude(Angles {
            roll: 20f32.to_radians(),
            pitch: 0.0,
            yaw: 0.0,
        });
        let before = quad.angles();
        run(&mut quad, 0.3, 0.5, 0.2, 0.4, 1.0);
        assert!(quad.on_ground());
        assert_at_rest(&quad);
        assert_eq!(quad.angles(), before);

        // The motors did follow their commands all the while.
        assert!((quad.motors.fr.speed() - 0.5).abs() < 1e-3);
    }

    #[test]
    fn test_reset_free() {
        // Flown, then `reset()`: on the ground at the origin, at rest, with
        // no touchdown, and behaving as a new one.
        let mut used = free_quad();
        run(&mut used, 0.9, 0.85, 0.9, 0.85, 1.0);
        assert!(!used.on_ground());
        assert!(used.height() > 0.5);

        used.reset();
        assert!(used.on_ground());
        assert_eq!(used.position(), Vec3::ZERO);
        assert_at_rest(&used);
        assert_eq!(used.touchdown(), None);
        assert_eq!(used.support, Support::Free);
        assert_eq!(used.mass, MASS);
        assert_eq!(used.drag, DRAG);
        assert_accel(used.accel(), 0.0, 0.0, -9.807);

        let mut fresh = free_quad();
        run(&mut used, 0.9, 0.85, 0.9, 0.85, 1.0);
        run(&mut fresh, 0.9, 0.85, 0.9, 0.85, 1.0);
        assert_eq!(used.position(), fresh.position());
        assert_eq!(used.velocity(), fresh.velocity());
        assert_eq!(used.angles(), fresh.angles());

        // A landing is forgotten as well.
        let mut landed = free_quad_at(0.204, 0.0, 0.0);
        run_all(&mut landed, 0.0, 0.5);
        assert!(landed.touchdown().is_some());
        landed.reset();
        assert_eq!(landed.touchdown(), None);
    }

    // ---- The accelerometer of a free body

    #[test]
    fn test_accel_on_the_ground_level() {
        // Free, motors off: (0, 0, −9.807).
        let mut quad = free_quad();
        assert_accel(quad.accel(), 0.0, 0.0, -9.807);
        run_all(&mut quad, 0.0, 0.5);
        assert_accel(quad.accel(), 0.0, 0.0, -9.807);

        // Motors running, but not lifting it: the ground makes up the rest.
        run_all(&mut quad, 0.5, 0.5);
        assert!(quad.on_ground());
        assert_accel(quad.accel(), 0.0, 0.0, -9.807);
    }

    #[test]
    fn test_accel_on_the_ground_tilted() {
        // Set down at 20° of roll: what the joint reads at 20° of roll.
        let slope = Angles {
            roll: 20f32.to_radians(),
            pitch: 0.0,
            yaw: 0.0,
        };
        let mut on_joint = quad();
        on_joint.set_attitude(slope);
        let expected = on_joint.accel();

        let mut free = free_quad();
        free.set_attitude(slope);
        assert_accel(free.accel(), expected.x, expected.y, expected.z);
        run_all(&mut free, 0.0, 0.5);
        assert_accel(free.accel(), expected.x, expected.y, expected.z);

        // Which is (0, −G × sin 20°, −G × cos 20°).
        assert_accel(free.accel(), 0.0, -3.354, -9.215);
    }

    #[test]
    fn test_accel_hovering() {
        // In the air, level, motors at speed at the hover command:
        // (0, 0, −9.807).
        let (mut quad, hover) = hovering_at(10.0, 0.0, 0.0);
        run_all(&mut quad, hover, 0.5);
        assert!(!quad.on_ground());
        assert_accel(quad.accel(), 0.0, 0.0, -9.807);
    }

    #[test]
    fn test_accel_falling() {
        // In the air, motors off, no drag: (0, 0, 0). Nothing pushes on it
        // but gravity, which an accelerometer cannot feel.
        let mut quad = free_quad_at(10.0, 0.0, 0.0);
        quad.drag = 0.0;
        run_all(&mut quad, 0.0, 0.5);
        assert!(!quad.on_ground());
        assert_accel(quad.accel(), 0.0, 0.0, 0.0);

        // The same falling tilted.
        let mut quad = free_quad_at(10.0, 30.0, -20.0);
        quad.drag = 0.0;
        run_all(&mut quad, 0.0, 0.5);
        assert_accel(quad.accel(), 0.0, 0.0, 0.0);
    }

    #[test]
    fn test_accel_thrust_alone_says_level() {
        // Held at 30° of roll, hover command, no drag: the x and y parts
        // are 0, as if it were level.
        let (mut quad, hover) = hovering_at(50.0, 30.0, 0.0);
        quad.drag = 0.0;
        run_all(&mut quad, hover, 0.5);

        // It is tilted and moving sideways.
        assert!((quad.angles().roll.to_degrees() - 30.0).abs() < 0.01);
        assert!(quad.velocity().y > 1.0);
        // The reading is the thrust alone, straight along the body's z
        // axis: what a level, hovering drone reads.
        assert_accel(quad.accel(), 0.0, 0.0, -9.807);
    }

    #[test]
    fn test_accel_drag_puts_the_tilt_back() {
        // Held at 10° of roll with drag, 20 s: the reading gives a roll of
        // 10°, from roll = atan2(−ay, −az).
        let (mut quad, hover) = hovering_at(100.0, 10.0, 0.0);

        // At the start the reading says level, as with no drag.
        run_all(&mut quad, hover, 0.01);
        let accel = quad.accel();
        let roll = (-accel.y).atan2(-accel.z).to_degrees();
        assert!(
            roll.abs() < 0.1,
            "roll from the reading {roll}° at the start"
        );

        // A third of the way there after 1.2 s: 1 − e^(−1.2 / 3) = 0.33.
        run_all(&mut quad, hover, 1.2);
        let accel = quad.accel();
        let roll = (-accel.y).atan2(-accel.z).to_degrees();
        assert!(
            (roll - 3.3).abs() < 0.3,
            "roll from the reading {roll}° at 1.2 s"
        );

        // Settled: the reading is the joint's, 1 g long and 10° over.
        run_all(&mut quad, hover, 20.0);
        let accel = quad.accel();
        let roll = (-accel.y).atan2(-accel.z).to_degrees();
        assert!(
            (roll - 10.0).abs() < 0.1,
            "roll from the reading {roll}° when settled"
        );
        let length = (accel.x * accel.x + accel.y * accel.y + accel.z * accel.z).sqrt();
        assert!((length - G).abs() < 0.02, "length {length}");
    }
}
