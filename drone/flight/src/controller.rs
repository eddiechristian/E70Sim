//! The controller: calibration, estimator, two angle loops, three rate loops
//! and the mixer, run in the right order. Sensor readings and targets in,
//! four motor commands out.
//!
//! Explained in `Stage2-c.md` in the repo root. The throttle, and raising it
//! in a tilt, is explained in `Stage3-b.md`.
//!
//! This crate is `no_std`, so `cos` and `sqrt` are not methods of `f32`
//! here. Use `libm::cosf(x)` and `libm::sqrtf(x)`, which take radians. The simulation, and later
//! the firmware, does three things each step: read the sensors, call
//! `Controller::update`, write the motor commands.

use crate::calibration::Calibration;
use crate::calibration::CalibrationStatus;
use crate::estimator::{Accel, Attitude, Estimator, GyroRates};
use crate::mixer::{MixerInput, MotorCommands, mix};
use crate::pid::Pid;

/// The angle loops run once in this many steps: 250 Hz when the step is
/// 1 kHz. Their last answer is held in between.
pub const ANGLE_LOOP_DIVIDER: u32 = 4;

/// All four motors off: what `update` returns while calibrating.
const MOTORS_OFF: MotorCommands = MotorCommands {
    fl: 0.0,
    fr: 0.0,
    br: 0.0,
    bl: 0.0,
};

const ZERO_RATES: GyroRates = GyroRates {
    roll: 0.0,
    pitch: 0.0,
    yaw: 0.0,
};

/// What the pilot asks for. In this stage the simulation sets it.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Targets {
    pub roll: f32,     // °; positive right side down
    pub pitch: f32,    // °; positive nose up
    pub yaw_rate: f32, // °/s; positive nose right
    pub throttle: f32, // 0.0 to 1.0; the same on all four motors
}

/// What the controller is doing.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum State {
    /// Collecting readings; the motors are off.
    Calibrating,
    /// The loops are running.
    Flying,
}

/// The gains and limits of the five loops, and the estimator's time
/// constant. `Gains::default()` holds the values chosen so far.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Gains {
    // The angle loops, roll and pitch. P only.
    pub angle_kp: f32,   // °/s per degree. Start with 6
    pub rate_limit: f32, // °/s; the angle loops ask for no more than this. Start with 200

    // The roll and pitch rate loops; outputs in throttle units.
    pub rate_kp: f32, // throttle per °/s
    pub rate_ki: f32,
    pub rate_kd: f32,

    // The yaw rate loop.
    pub yaw_kp: f32, // throttle per °/s
    pub yaw_ki: f32,
    pub yaw_kd: f32,

    // All three rate loops.
    pub d_filter: f32,  // s; time constant of the D-term low-pass filter
    pub out_limit: f32, // each output stays within ±out_limit
    pub i_limit: f32,

    // The estimator.
    pub estimator_time_constant: f32, // s

    // The throttle (`Stage3-b.md`).
    pub hover_throttle: f32, // 0.0 to 1.0; the command that holds the drone's weight
    pub tilt_compensation: bool, // more throttle in a tilt
    pub tilt_compensation_limit: f32, // °; no more is added past this tilt. Start with 45
}

impl Default for Gains {
    /// The rate loops' P and D gains are those of Stage 2a, the angle gain
    /// that of Stage 2c, the rate loops' I gains those of Stage 2d, the
    /// estimator's time constant that of Stage 3a and the throttle settings
    /// those of Stage 3b.
    fn default() -> Self {
        Self {
            angle_kp: 6.0,
            rate_limit: 200.0,
            rate_kp: 0.0014,
            rate_ki: 0.003,
            rate_kd: 0.00003,
            yaw_kp: 0.01,
            yaw_ki: 0.02,
            yaw_kd: 0.0,
            d_filter: 0.01,
            out_limit: 0.3,
            i_limit: 0.1,
            estimator_time_constant: 10.0,
            // The simulated drone's: 1.2 kg on four motors of 6 N. The
            // simulation sets it from the body it flies.
            hover_throttle: 0.7002,
            tilt_compensation: true,
            tilt_compensation_limit: 45.0,
        }
    }
}

/// `throttle` raised to make up for the lift lost at this attitude
/// (`Stage3-b.md`). Tilted, only part of the thrust still points up:
///    lift = thrust × cos(tilt), and cos(tilt) = cos(roll) × cos(pitch)
/// Thrust goes with the square of the command, so the command needs the
/// square root of what the thrust needs:
///    throttle / √(cos(roll) × cos(pitch))
///
/// Past `limit` degrees of tilt no more is added: when the product of the
/// two cosines is below cos(limit), use cos(limit) in its place.
///
/// `attitude` is in degrees, and so is `limit`; `libm::cosf` takes radians.
fn compensate_for_tilt(throttle: f32, attitude: Attitude, limit: f32) -> f32 {
    // How much of the thrust still points up: 1 when level, 0 on its side.
    let upright = libm::cosf(attitude.roll.to_radians()) * libm::cosf(attitude.pitch.to_radians());
    // Its value at the limit. Taking the larger of the two stops the
    // compensation there, and keeps the square root away from zero and from
    // the negative values of a drone that is upside down.
    let least = libm::cosf(limit.to_radians());
    throttle / libm::sqrtf(upright.max(least))
}

/// An angle PID: P only, with its output, a target rate, kept within the
/// rate limit.
fn angle_pid(gains: &Gains) -> Pid {
    let mut pid = Pid::new(gains.angle_kp, 0.0, 0.0);
    pid.out_min = -gains.rate_limit;
    pid.out_max = gains.rate_limit;
    pid
}

/// A rate PID with the limits and the D-term filter set.
fn rate_pid(kp: f32, ki: f32, kd: f32, gains: &Gains) -> Pid {
    let mut pid = Pid::new(kp, ki, kd);
    pid.out_min = -gains.out_limit;
    pid.out_max = gains.out_limit;
    pid.i_limit = gains.i_limit;
    pid.d_filter = gains.d_filter;
    pid
}

pub struct Controller {
    // Parts
    calibration: Calibration,
    estimator: Estimator,
    roll_angle_pid: Pid,
    pitch_angle_pid: Pid,
    roll_rate_pid: Pid,
    pitch_rate_pid: Pid,
    yaw_rate_pid: Pid,

    // Settings, from the gains
    tilt_compensation: bool,
    tilt_compensation_limit: f32, // °

    // State
    state: State,
    gyro_bias: GyroRates,       // °/s; from the calibration, zero until then
    attitude: Option<Attitude>, // the latest estimate; None while calibrating
    step: u32,                  // steps flown; picks the steps the angle loops run on
    roll_rate_target: f32,      // °/s; the roll angle loop's last answer
    pitch_rate_target: f32,     // °/s; the pitch angle loop's last answer
    throttle: f32,              // what last went to the mixer; 0 while calibrating
    last_failure: Option<&'static str>, // why the last failed calibration window failed
}

impl Controller {
    pub fn new(gains: Gains) -> Self {
        Self {
            calibration: Calibration::new(),
            estimator: Estimator::new(gains.estimator_time_constant),
            roll_angle_pid: angle_pid(&gains),
            pitch_angle_pid: angle_pid(&gains),
            roll_rate_pid: rate_pid(gains.rate_kp, gains.rate_ki, gains.rate_kd, &gains),
            pitch_rate_pid: rate_pid(gains.rate_kp, gains.rate_ki, gains.rate_kd, &gains),
            yaw_rate_pid: rate_pid(gains.yaw_kp, gains.yaw_ki, gains.yaw_kd, &gains),
            tilt_compensation: gains.tilt_compensation,
            tilt_compensation_limit: gains.tilt_compensation_limit,
            state: State::Calibrating,
            gyro_bias: ZERO_RATES,
            attitude: None,
            step: 0,
            roll_rate_target: 0.0,
            pitch_rate_target: 0.0,
            throttle: 0.0,
            last_failure: None,
        }
    }

    /// Takes one reading of each sensor and the targets, and returns the
    /// four motor commands. `dt` is the time since the last call, in
    /// seconds.
    ///
    /// While `Calibrating`:
    /// 1. Give the readings to `self.calibration`.
    /// 2. On `Done`: store the bias, hand the starting angle to the
    ///    estimator with `set_attitude`, store it in `self.attitude`, and
    ///    change the state to `Flying`. On `Collecting` and `Failed` there
    ///    is nothing to do; the calibration starts its own new window.
    /// 3. Return `MOTORS_OFF`, also on the step that finishes.
    ///
    /// While `Flying`:
    /// 1. Subtract the bias from the gyro reading, axis by axis. Everything
    ///    below uses the result.
    /// 2. Update the estimator and store its answer in `self.attitude`.
    /// 3. When `self.step` is a multiple of `ANGLE_LOOP_DIVIDER`: run the
    ///    two angle PIDs, with the target angle as setpoint, the estimated
    ///    angle as measurement and `dt × ANGLE_LOOP_DIVIDER` as their time
    ///    step. Store the answers in the two rate targets. Then count the
    ///    step, on every call.
    /// 4. Run the three rate PIDs: roll and pitch with the stored rate
    ///    targets as setpoints, yaw with `targets.yaw_rate`.
    /// 5. Raise `targets.throttle` for the tilt, with `compensate_for_tilt`
    ///    and the estimate of this step, if the gains ask for it.
    /// 6. Mix the throttle with the three outputs and return the result.
    ///
    /// A bad `dt` is passed through: the PIDs and the estimator guard
    /// against it themselves.
    pub fn update(
        &mut self,
        sensor_gyro: GyroRates,
        sensor_accel: Accel,
        targets: Targets,
        dt: f32,
    ) -> MotorCommands {
        if self.state == State::Calibrating {
            match self.calibration.update(sensor_gyro, sensor_accel) {
                // Nothing to do: the calibration starts its own new window.
                CalibrationStatus::Collecting => {}
                CalibrationStatus::Failed(reason) => self.last_failure = Some(reason),
                CalibrationStatus::Done(result) => {
                    self.gyro_bias = result.gyro_bias;
                    self.estimator.set_attitude(result.attitude);
                    self.attitude = Some(result.attitude);
                    self.state = State::Flying;
                }
            }
            return MOTORS_OFF;
        }
        // Flying. The loops and the estimator use the gyro reading with the
        // bias removed, never `sensor_gyro`.
        let gyro_minus_bias = GyroRates {
            roll: sensor_gyro.roll - self.gyro_bias.roll,
            pitch: sensor_gyro.pitch - self.gyro_bias.pitch,
            yaw: sensor_gyro.yaw - self.gyro_bias.yaw,
        };

        let attitude = self.estimator.update(gyro_minus_bias, sensor_accel, dt);
        self.attitude = Some(attitude);

        // The angle loops, on every fourth step. In between, the rate
        // targets keep their last values.
        if self.step.is_multiple_of(ANGLE_LOOP_DIVIDER) {
            let angle_dt = dt * ANGLE_LOOP_DIVIDER as f32;
            self.roll_rate_target =
                self.roll_angle_pid
                    .update(targets.roll, attitude.roll, angle_dt);
            self.pitch_rate_target =
                self.pitch_angle_pid
                    .update(targets.pitch, attitude.pitch, angle_dt);
        }
        self.step += 1;

        // The rate loops, on every step.
        let roll = self
            .roll_rate_pid
            .update(self.roll_rate_target, gyro_minus_bias.roll, dt);
        let pitch = self
            .pitch_rate_pid
            .update(self.pitch_rate_target, gyro_minus_bias.pitch, dt);
        let yaw = self
            .yaw_rate_pid
            .update(targets.yaw_rate, gyro_minus_bias.yaw, dt);

        // The throttle, raised for the tilt. The answer is not clamped
        // here: a command above 1.0 is the mixer's business.
        let throttle = if self.tilt_compensation {
            compensate_for_tilt(targets.throttle, attitude, self.tilt_compensation_limit)
        } else {
            targets.throttle
        };
        self.throttle = throttle;

        mix(MixerInput {
            throttle,
            roll,
            pitch,
            yaw,
        })
    }

    pub fn state(&self) -> State {
        self.state
    }

    /// Why the last failed calibration window failed, for the log. `None`
    /// when no window has failed. This crate cannot print; the caller can.
    pub fn last_failure(&self) -> Option<&'static str> {
        self.last_failure
    }

    /// The latest estimate, for the log. `None` while calibrating.
    pub fn attitude(&self) -> Option<Attitude> {
        self.attitude
    }

    /// The gyro bias found by the calibration, in °/s. Zero until then.
    pub fn gyro_bias(&self) -> GyroRates {
        self.gyro_bias
    }

    /// What the roll angle loop last asked of the roll rate loop, in °/s,
    /// for the log.
    pub fn roll_rate_target(&self) -> f32 {
        self.roll_rate_target
    }

    /// What the pitch angle loop last asked of the pitch rate loop, in °/s,
    /// for the log.
    pub fn pitch_rate_target(&self) -> f32 {
        self.pitch_rate_target
    }

    /// The throttle that last went to the mixer, for the log: what the
    /// targets asked for, raised for the tilt if the gains ask for that.
    /// Zero while calibrating, when the motors are off.
    pub fn throttle(&self) -> f32 {
        self.throttle
    }

    /// Back to `Calibrating`, with everything cleared. The gains are kept.
    pub fn reset(&mut self) {
        self.calibration.reset();
        self.estimator.reset();
        self.roll_angle_pid.reset();
        self.pitch_angle_pid.reset();
        self.roll_rate_pid.reset();
        self.pitch_rate_pid.reset();
        self.yaw_rate_pid.reset();
        self.state = State::Calibrating;
        self.gyro_bias = ZERO_RATES;
        self.attitude = None;
        self.step = 0;
        self.roll_rate_target = 0.0;
        self.pitch_rate_target = 0.0;
        self.throttle = 0.0;
        self.last_failure = None;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::calibration::CALIBRATION_STEPS;

    const DT: f32 = 0.001;

    // Motor commands are compared in throttle units.
    const TOLERANCE: f32 = 1e-5;

    const LEVEL: Accel = Accel {
        x: 0.0,
        y: 0.0,
        z: -1.0,
    };

    // Level, no yaw, throttle 0.7.
    const LEVEL_TARGETS: Targets = Targets {
        roll: 0.0,
        pitch: 0.0,
        yaw_rate: 0.0,
        throttle: 0.7,
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

    // The default gains with the rate loops' I terms and the tilt
    // compensation off. These tests check one step of the chain by its
    // arithmetic: angle error × angle `Kp` × rate `Kp`. An I term would add
    // a little to every figure, and the compensation would raise the
    // throttle of every test that is tilted.
    fn gains() -> Gains {
        Gains {
            rate_ki: 0.0,
            yaw_ki: 0.0,
            tilt_compensation: false,
            ..Gains::default()
        }
    }

    // A controller that has just finished calibrating, with the same gyro
    // and accelerometer reading on every step. The next update is the
    // first one flown.
    fn calibrated(gyro: GyroRates, accel: Accel) -> Controller {
        calibrated_with(gains(), gyro, accel)
    }

    // The same with the given gains.
    fn calibrated_with(gains: Gains, gyro: GyroRates, accel: Accel) -> Controller {
        let mut controller = Controller::new(gains);
        for _ in 0..CALIBRATION_STEPS {
            controller.update(gyro, accel, LEVEL_TARGETS, DT);
        }
        assert_eq!(controller.state(), State::Flying);
        controller
    }

    fn assert_commands(commands: MotorCommands, fl: f32, fr: f32, br: f32, bl: f32) {
        let near = |actual: f32, expected: f32| (actual - expected).abs() < TOLERANCE;
        assert!(
            near(commands.fl, fl)
                && near(commands.fr, fr)
                && near(commands.br, br)
                && near(commands.bl, bl),
            "got {commands:?}, expected fl {fl}, fr {fr}, br {br}, bl {bl}"
        );
    }

    fn assert_same(a: MotorCommands, b: MotorCommands) {
        assert_commands(a, b.fl, b.fr, b.br, b.bl);
    }

    // "More power" in the comments below means a larger motor command.

    #[test]
    fn test_motors_off_while_calibrating() {
        // The first 1000 updates, level, throttle 0.7: all four commands 0,
        // and the state is `Calibrating` until the last of them.
        let mut controller = Controller::new(Gains::default());
        for _ in 0..CALIBRATION_STEPS {
            assert_eq!(controller.state(), State::Calibrating);
            assert_eq!(controller.attitude(), None);
            let commands = controller.update(ZERO_RATES, LEVEL, LEVEL_TARGETS, DT);
            assert_eq!(commands, MOTORS_OFF);
        }
        assert_eq!(controller.state(), State::Flying);
    }

    #[test]
    fn test_flying_after_calibrating() {
        // The 1001st update, level, targets zero, throttle 0.7: the state is
        // `Flying` and all four commands are 0.7.
        let mut controller = calibrated(ZERO_RATES, LEVEL);
        let commands = controller.update(ZERO_RATES, LEVEL, LEVEL_TARGETS, DT);
        assert_eq!(controller.state(), State::Flying);
        assert_commands(commands, 0.7, 0.7, 0.7, 0.7);
    }

    #[test]
    fn test_tilted_right_target_level() {
        // Calibrated at 10° of roll, zero rates, targets level: the right
        // motors (fr, br) get more power than the left ones (fl, bl), to
        // lift the right side.
        //
        // The angle loop asks for 6 × (0 − 10) = −60 °/s, and the rate loop
        // answers 0.0014 × −60 = −0.084.
        let accel = reading_at(10.0, 0.0);
        let mut controller = calibrated(ZERO_RATES, accel);
        let commands = controller.update(ZERO_RATES, accel, LEVEL_TARGETS, DT);
        assert!((controller.roll_rate_target() + 60.0).abs() < 0.01);
        assert_commands(commands, 0.616, 0.784, 0.784, 0.616);
    }

    #[test]
    fn test_tilted_nose_up_target_level() {
        // Calibrated at 10° of pitch, zero rates, targets level: the back
        // motors (br, bl) get more power than the front ones (fl, fr), to
        // lift the tail and bring the nose down.
        let accel = reading_at(0.0, 10.0);
        let mut controller = calibrated(ZERO_RATES, accel);
        let commands = controller.update(ZERO_RATES, accel, LEVEL_TARGETS, DT);
        assert!((controller.pitch_rate_target() + 60.0).abs() < 0.01);
        assert_commands(commands, 0.616, 0.616, 0.784, 0.784);
    }

    #[test]
    fn test_target_right_drone_level() {
        // Level readings, target roll 10°: the left motors get more power
        // than the right ones, to push the right side down.
        let mut controller = calibrated(ZERO_RATES, LEVEL);
        let targets = Targets {
            roll: 10.0,
            ..LEVEL_TARGETS
        };
        let commands = controller.update(ZERO_RATES, LEVEL, targets, DT);
        assert!((controller.roll_rate_target() - 60.0).abs() < 0.01);
        assert_commands(commands, 0.784, 0.616, 0.616, 0.784);
    }

    #[test]
    fn test_yaw_untouched_by_tilt() {
        // Readings for 10° of roll: no yaw part, so `fl + br` equals
        // `fr + bl`.
        let accel = reading_at(10.0, 0.0);
        let mut controller = calibrated(ZERO_RATES, accel);
        let commands = controller.update(ZERO_RATES, accel, LEVEL_TARGETS, DT);
        assert_ne!(commands.fl, commands.fr); // the tilt did do something
        let cw = commands.fl + commands.br;
        let ccw = commands.fr + commands.bl;
        assert!((cw - ccw).abs() < TOLERANCE, "cw {cw}, ccw {ccw}");
    }

    #[test]
    fn test_rate_limit() {
        // Readings for 80° of roll give the same commands as readings for
        // 60°: both ask for the rate limit.
        //
        // The rate loop answers 0.0014 × −200 = −0.28, which is inside its
        // own limit of 0.3, so it is the rate limit that is tested.
        let mut commands = [MOTORS_OFF; 2];
        for (i, roll) in [60.0, 80.0].into_iter().enumerate() {
            let accel = reading_at(roll, 0.0);
            let mut controller = calibrated(ZERO_RATES, accel);
            commands[i] = controller.update(ZERO_RATES, accel, LEVEL_TARGETS, DT);
            assert_eq!(controller.roll_rate_target(), -200.0);
        }
        assert_same(commands[0], commands[1]);
        assert_commands(commands[0], 0.42, 0.98, 0.98, 0.42);

        // The other way: level, with a target of 80° of roll.
        let mut controller = calibrated(ZERO_RATES, LEVEL);
        let targets = Targets {
            roll: 80.0,
            ..LEVEL_TARGETS
        };
        let other_way = controller.update(ZERO_RATES, LEVEL, targets, DT);
        assert_eq!(controller.roll_rate_target(), 200.0);
        assert_commands(other_way, 0.98, 0.42, 0.42, 0.98);
    }

    #[test]
    fn test_bias_is_removed() {
        // Calibrate with a gyro reading of 1 °/s on each axis, then give the
        // same reading while flying: the same commands as a controller that
        // saw no bias at all.
        let biased_reading = GyroRates {
            roll: 1.0,
            pitch: 1.0,
            yaw: 1.0,
        };
        let mut biased = calibrated(biased_reading, LEVEL);
        let mut clean = calibrated(ZERO_RATES, LEVEL);
        assert!((biased.gyro_bias().roll - 1.0).abs() < 1e-3);
        assert!((biased.gyro_bias().pitch - 1.0).abs() < 1e-3);
        assert!((biased.gyro_bias().yaw - 1.0).abs() < 1e-3);

        // A target that makes the loops work, so that the commands are not
        // simply the throttle.
        let targets = Targets {
            roll: 10.0,
            pitch: -5.0,
            yaw_rate: 20.0,
            throttle: 0.6,
        };
        for _ in 0..20 {
            let with_bias = biased.update(biased_reading, LEVEL, targets, DT);
            let without = clean.update(ZERO_RATES, LEVEL, targets, DT);
            assert_same(with_bias, without);
        }
    }

    #[test]
    fn test_runs_every_fourth_step() {
        // Change the target roll between two steps that are not a fourth
        // step: `roll_rate_target()` and the commands do not change until
        // the next fourth step.
        let mut controller = calibrated(ZERO_RATES, LEVEL);
        let tilted = Targets {
            roll: 10.0,
            ..LEVEL_TARGETS
        };

        // Step 0: the angle loops run, with a level target.
        let first = controller.update(ZERO_RATES, LEVEL, LEVEL_TARGETS, DT);
        assert_eq!(controller.roll_rate_target(), 0.0);

        // Steps 1, 2 and 3: the new target is not looked at yet.
        for _ in 1..ANGLE_LOOP_DIVIDER {
            let commands = controller.update(ZERO_RATES, LEVEL, tilted, DT);
            assert_eq!(controller.roll_rate_target(), 0.0);
            assert_same(commands, first);
        }

        // Step 4: the angle loops run again.
        let commands = controller.update(ZERO_RATES, LEVEL, tilted, DT);
        assert!((controller.roll_rate_target() - 60.0).abs() < 0.01);
        assert!(commands.fl > first.fl + 0.05);
    }

    #[test]
    fn test_reset() {
        // Fly, then `reset()`: `Calibrating` again, `attitude()` is `None`,
        // and the next update returns all four motors off.
        let biased_reading = GyroRates {
            roll: 1.0,
            pitch: 0.0,
            yaw: 0.0,
        };
        let mut controller = calibrated(biased_reading, reading_at(10.0, 0.0));
        controller.update(biased_reading, LEVEL, LEVEL_TARGETS, DT);
        assert!(controller.attitude().is_some());

        controller.reset();
        assert_eq!(controller.state(), State::Calibrating);
        assert_eq!(controller.attitude(), None);
        assert_eq!(controller.gyro_bias(), ZERO_RATES);
        assert_eq!(controller.roll_rate_target(), 0.0);
        let commands = controller.update(ZERO_RATES, LEVEL, LEVEL_TARGETS, DT);
        assert_eq!(commands, MOTORS_OFF);
        assert_eq!(controller.state(), State::Calibrating);
    }

    // ---- The tilt compensation. Named after the table in `Stage3-b.md`.

    // The limit the tests use, in degrees.
    const LIMIT: f32 = 45.0;

    // An attitude at the given roll and pitch, in degrees.
    fn attitude_at(roll: f32, pitch: f32) -> Attitude {
        Attitude { roll, pitch }
    }

    // What the throttle is multiplied by at the given roll and pitch.
    fn multiplier(roll: f32, pitch: f32) -> f32 {
        compensate_for_tilt(1.0, attitude_at(roll, pitch), LIMIT)
    }

    // Multipliers are compared to four decimal places.
    fn assert_multiplier(roll: f32, pitch: f32, expected: f32) {
        let got = multiplier(roll, pitch);
        assert!(
            (got - expected).abs() < 1e-4,
            "roll {roll}°, pitch {pitch}°: got × {got}, expected × {expected}"
        );
    }

    // The first commands flown by a controller calibrated at 30° of roll,
    // with a level target, a throttle of 0.5 and no rotation, with the tilt
    // compensation on or off.
    fn first_commands_at_30(tilt_compensation: bool) -> MotorCommands {
        let gains = Gains {
            tilt_compensation,
            tilt_compensation_limit: LIMIT,
            ..gains()
        };
        let accel = reading_at(30.0, 0.0);
        let mut controller = calibrated_with(gains, ZERO_RATES, accel);
        let targets = Targets {
            throttle: 0.5,
            ..LEVEL_TARGETS
        };
        controller.update(ZERO_RATES, accel, targets, DT)
    }

    fn average(commands: MotorCommands) -> f32 {
        (commands.fl + commands.fr + commands.br + commands.bl) / 4.0
    }

    #[test]
    fn test_level_changes_nothing() {
        // Roll 0°, pitch 0°: the throttle that went in.
        for throttle in [0.0, 0.3, 0.7002, 1.0] {
            let out = compensate_for_tilt(throttle, attitude_at(0.0, 0.0), LIMIT);
            assert_eq!(out, throttle);
        }
    }

    #[test]
    fn test_compensates_for_roll() {
        // Roll 30°: × 1.0746, which is 1 / √(cos 30°).
        assert_multiplier(30.0, 0.0, 1.0746);
        // A small tilt needs very little, and a larger one more.
        assert_multiplier(10.0, 0.0, 1.0077);
        assert_multiplier(20.0, 0.0, 1.0316);

        // It multiplies whatever throttle it is given.
        let out = compensate_for_tilt(0.7, attitude_at(30.0, 0.0), LIMIT);
        assert!((out - 0.7 * 1.0746).abs() < 1e-4, "got {out}");
    }

    #[test]
    fn test_compensates_for_pitch() {
        // Pitch 30°: the same.
        assert_multiplier(0.0, 30.0, 1.0746);
        assert_eq!(multiplier(0.0, 30.0), multiplier(30.0, 0.0));
    }

    #[test]
    fn test_the_sign_does_not_matter() {
        // Roll −30°: the same as roll 30°.
        assert_multiplier(-30.0, 0.0, 1.0746);
        assert_multiplier(0.0, -30.0, 1.0746);
        assert_eq!(multiplier(-20.0, 10.0), multiplier(20.0, -10.0));
    }

    #[test]
    fn test_compensates_for_both() {
        // Roll 20° and pitch 20°: × 1.0642, which is
        // 1 / √(cos 20° × cos 20°).
        assert_multiplier(20.0, 20.0, 1.0642);
        // More than for either alone.
        assert!(multiplier(20.0, 20.0) > multiplier(20.0, 0.0));
    }

    #[test]
    fn test_no_more_past_the_limit() {
        // Roll 60°, limit 45°: × 1.1892, as at 45°. So is roll 50° with
        // pitch 50°.
        assert_multiplier(45.0, 0.0, 1.1892);
        assert_multiplier(60.0, 0.0, 1.1892);
        assert_multiplier(50.0, 50.0, 1.1892);
        // On its side and upside down, where the cosine is zero and then
        // negative: still the same, and still a number.
        assert_multiplier(90.0, 0.0, 1.1892);
        assert_multiplier(180.0, 0.0, 1.1892);

        // A lower limit stops it sooner.
        let out = compensate_for_tilt(1.0, attitude_at(30.0, 0.0), 20.0);
        assert!((out - 1.0316).abs() < 1e-4, "got {out}");
    }

    #[test]
    fn test_zero_stays_zero() {
        // Throttle 0, any tilt: 0.
        for (roll, pitch) in [(0.0, 0.0), (30.0, -20.0), (80.0, 0.0), (180.0, 0.0)] {
            let out = compensate_for_tilt(0.0, attitude_at(roll, pitch), LIMIT);
            assert_eq!(out, 0.0);
        }
    }

    #[test]
    fn test_off_leaves_the_throttle_alone() {
        // `tilt_compensation` false, calibrated at 30° of roll, throttle
        // 0.5, zero rates: the four commands average 0.5, as before this
        // item.
        //
        // The angle loop asks for 6 × (0 − 30) = −180 °/s and the rate loop
        // answers 0.0014 × −180 = −0.252, so the right motors get
        // 0.5 + 0.252 and the left ones 0.5 − 0.252.
        let commands = first_commands_at_30(false);
        assert_commands(commands, 0.248, 0.752, 0.752, 0.248);
        assert!((average(commands) - 0.5).abs() < TOLERANCE);
    }

    #[test]
    fn test_on_raises_the_throttle_in_the_controller() {
        // The same with `tilt_compensation` true: the four commands average
        // 0.5 × 1.0746 = 0.537. The differences between them, which are the
        // loops' work, are the same as with it off.
        let off = first_commands_at_30(false);
        let on = first_commands_at_30(true);
        let raised = 0.5 * 1.0746;

        // `throttle()` reports it: 0 while calibrating, then what went to
        // the mixer.
        let accel = reading_at(30.0, 0.0);
        let mut controller = Controller::new(Gains {
            tilt_compensation: true,
            ..gains()
        });
        let targets = Targets {
            throttle: 0.5,
            ..LEVEL_TARGETS
        };
        for _ in 0..CALIBRATION_STEPS {
            controller.update(ZERO_RATES, accel, targets, DT);
            assert_eq!(controller.throttle(), 0.0);
        }
        controller.update(ZERO_RATES, accel, targets, DT);
        assert!((controller.throttle() - raised).abs() < 1e-4);
        controller.reset();
        assert_eq!(controller.throttle(), 0.0);

        assert!(
            (average(on) - raised).abs() < 1e-4,
            "the four average {}",
            average(on)
        );

        // All four went up by the same amount.
        let lift = raised - 0.5;
        assert!((on.fl - off.fl - lift).abs() < 1e-4);
        assert!((on.fr - off.fr - lift).abs() < 1e-4);
        assert!((on.br - off.br - lift).abs() < 1e-4);
        assert!((on.bl - off.bl - lift).abs() < 1e-4);
    }
}
