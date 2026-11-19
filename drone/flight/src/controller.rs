//! The controller: calibration, estimator, two angle loops, three rate loops
//! and the mixer, run in the right order. Sensor readings and targets in,
//! four motor commands out.
//!
//! Explained in `Stage2-c.md` in the repo root. The simulation, and later
//! the firmware, does three things each step: read the sensors, call
//! `Controller::update`, write the motor commands.

use crate::calibration::Calibration;
use crate::calibration::CalibrationStatus;
use crate::estimator::{Accel, Attitude, Estimator, GyroRates};
use crate::mixer::MotorCommands;
#[allow(unused_imports)] // remove once written
use crate::mixer::{MixerInput, mix};
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
}

impl Default for Gains {
    /// The rate gains are those of Stage 2a, the estimator's time constant
    /// that of Stage 2b and the angle gain that of Stage 2c.
    fn default() -> Self {
        Self {
            angle_kp: 6.0,
            rate_limit: 200.0,
            rate_kp: 0.0014,
            rate_ki: 0.0,
            rate_kd: 0.00003,
            yaw_kp: 0.01,
            yaw_ki: 0.0,
            yaw_kd: 0.0,
            d_filter: 0.01,
            out_limit: 0.3,
            i_limit: 0.1,
            estimator_time_constant: 1.0,
        }
    }
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

#[allow(dead_code)] // remove once `update` is written
pub struct Controller {
    // Parts
    calibration: Calibration,
    estimator: Estimator,
    roll_angle_pid: Pid,
    pitch_angle_pid: Pid,
    roll_rate_pid: Pid,
    pitch_rate_pid: Pid,
    yaw_rate_pid: Pid,

    // State
    state: State,
    gyro_bias: GyroRates,       // °/s; from the calibration, zero until then
    attitude: Option<Attitude>, // the latest estimate; None while calibrating
    step: u32,                  // steps flown; picks the steps the angle loops run on
    roll_rate_target: f32,      // °/s; the roll angle loop's last answer
    pitch_rate_target: f32,     // °/s; the pitch angle loop's last answer
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
            state: State::Calibrating,
            gyro_bias: ZERO_RATES,
            attitude: None,
            step: 0,
            roll_rate_target: 0.0,
            pitch_rate_target: 0.0,
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
    /// 5. Mix `targets.throttle` with the three outputs and return the
    ///    result.
    ///
    /// A bad `dt` is passed through: the PIDs and the estimator guard
    /// against it themselves.
    #[allow(unused_variables)] // remove once written
    pub fn update(
        &mut self,
        gyro: GyroRates,
        accel: Accel,
        targets: Targets,
        dt: f32,
    ) -> MotorCommands {
        if self.state == State::Calibrating {
            match self.calibration.update(gyro, accel) {
                // Nothing to do: the calibration starts its own new window.
                CalibrationStatus::Collecting => {},
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
        // Flying
        let adj_gyro_roll = gyro.roll - self.gyro_bias.roll;
        let adj_gyro_pitch = gyro.pitch - self.gyro_bias.pitch;
        let adj_gyro_yaw = gyro.yaw - self.gyro_bias.yaw;
        let gyro = GyroRates { roll: adj_gyro_roll, pitch: adj_gyro_pitch, yaw: adj_gyro_yaw };
        
        self.attitude= Some(self.estimator.update(gyro, accel, dt));
        
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
        self.last_failure = None;
    }
}

#[cfg(test)]
mod tests {
    #[allow(unused_imports)] // remove once the tests are written
    use super::*;
    #[allow(unused_imports)] // remove once the tests are written
    use crate::calibration::CALIBRATION_STEPS;

    #[allow(dead_code)] // remove once the tests are written
    const DT: f32 = 0.001;

    #[allow(dead_code)] // remove once the tests are written
    const LEVEL: Accel = Accel {
        x: 0.0,
        y: 0.0,
        z: -1.0,
    };

    // Level, no yaw, throttle 0.7.
    #[allow(dead_code)] // remove once the tests are written
    const LEVEL_TARGETS: Targets = Targets {
        roll: 0.0,
        pitch: 0.0,
        yaw_rate: 0.0,
        throttle: 0.7,
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

    // "More power" in the comments below means a larger motor command.

    #[test]
    fn test_motors_off_while_calibrating() {
        // The first 1000 updates, level, throttle 0.7: all four commands 0,
        // and the state is `Calibrating` until the last of them.
        todo!()
    }

    #[test]
    fn test_flying_after_calibrating() {
        // The 1001st update, level, targets zero, throttle 0.7: the state is
        // `Flying` and all four commands are 0.7.
        todo!()
    }

    #[test]
    fn test_tilted_right_target_level() {
        // Calibrated at 10° of roll, zero rates, targets level: the right
        // motors (fr, br) get more power than the left ones (fl, bl), to
        // lift the right side.
        todo!()
    }

    #[test]
    fn test_tilted_nose_up_target_level() {
        // Calibrated at 10° of pitch, zero rates, targets level: the back
        // motors (br, bl) get more power than the front ones (fl, fr), to
        // lift the tail and bring the nose down.
        todo!()
    }

    #[test]
    fn test_target_right_drone_level() {
        // Level readings, target roll 10°: the left motors get more power
        // than the right ones, to push the right side down.
        todo!()
    }

    #[test]
    fn test_yaw_untouched_by_tilt() {
        // Readings for 10° of roll: no yaw part, so `fl + br` equals
        // `fr + bl`.
        todo!()
    }

    #[test]
    fn test_rate_limit() {
        // Readings for 80° of roll give the same commands as readings for
        // 60°: both ask for the rate limit.
        todo!()
    }

    #[test]
    fn test_bias_is_removed() {
        // Calibrate with a gyro reading of 1 °/s on each axis, then give the
        // same reading while flying: the same commands as a controller that
        // saw no bias at all.
        todo!()
    }

    #[test]
    fn test_runs_every_fourth_step() {
        // Change the target roll between two steps that are not a fourth
        // step: `roll_rate_target()` and the commands do not change until
        // the next fourth step.
        todo!()
    }

    #[test]
    fn test_reset() {
        // Fly, then `reset()`: `Calibrating` again, `attitude()` is `None`,
        // and the next update returns all four motors off.
        todo!()
    }
}
