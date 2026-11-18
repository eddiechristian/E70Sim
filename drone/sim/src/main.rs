//! Rate loop on the seesaw: one PID, two motors, CSV on standard output.
//!
//! Explained in `Stage1-c.md` in the repo root.
//!
//! Run with `cargo run > run.csv`, then plot with `python3 plot.py`.
//! The gains and the seesaw's size can be set on the command line:
//! `cargo run -- --kp 0.002 > run.csv`. `cargo run -- --help` lists them.

use clap::Parser;
use flight::pid::Pid;
use physics::gyro::Gyro;
use physics::motor::Motor;
use physics::seesaw::Seesaw;

// Time
const DT: f32 = 0.001; // s; one step, 1 kHz
const STEPS: u32 = 3000; // 3 simulated seconds

// The run: target 0, then STEP_TARGET from STEP_START to STEP_END, then 0.
const STEP_START: f32 = 0.5; // s
const STEP_END: f32 = 1.5; // s
const STEP_TARGET: f32 = 50.0; // °/s

// Mixer
const BASE: f32 = 0.7; // throttle both motors share, 0.0 to 1.0

// PID tuning; the output is in throttle units. The gains are command-line
// arguments; see `Args`.
const OUT_LIMIT: f32 = 0.3; // output stays within ±OUT_LIMIT
const I_LIMIT: f32 = 0.1;

// Seesaw. The arm length, inertia and thrust coefficient are command-line
// arguments.
const MOTOR_TIME_CONSTANT: f32 = 0.04; // s

/// Rate loop on the seesaw. Writes CSV to standard output.
#[derive(Parser)]
struct Args {
    /// Proportional gain, in throttle per °/s
    #[arg(long, default_value_t = 0.001)]
    kp: f32,

    /// Integral gain
    #[arg(long, default_value_t = 0.0)]
    ki: f32,

    /// Derivative gain
    #[arg(long, default_value_t = 0.0)]
    kd: f32,

    /// Time constant of the D-term low-pass filter, in seconds; 0 for none
    #[arg(long, default_value_t = 0.0)]
    d_filter: f32,

    /// Pivot to motor, in metres
    #[arg(long, default_value_t = 0.2)]
    arm_length: f32,

    /// How hard the bar is to spin, in kg·m²
    #[arg(long, default_value_t = 0.015)]
    inertia: f32,

    /// Thrust of each motor at full speed, in newtons
    #[arg(long, default_value_t = 6.0)]
    thrust_coeff: f32,

    /// Gyro noise: standard deviation in °/s
    #[arg(long, default_value_t = 0.0)]
    noise: f32,

    /// Loop delay: the PID sees the reading from this many steps ago
    #[arg(long, default_value_t = 0)]
    delay: usize,

    /// Seed for the gyro noise; the same seed gives the same run
    #[arg(long, default_value_t = 1)]
    seed: u64,
}

/// The target rate in °/s at `time` seconds.
fn target_at(time: f32) -> f32 {
    if time < STEP_START {
        0.0
    } else if time < STEP_END {
        STEP_TARGET
    } else {
        0.0
    }
}

/// Splits the PID output into (left, right) motor commands, each clamped to
/// 0.0 to 1.0. A positive output makes the right motor the stronger one.
fn mix(base: f32, output: f32) -> (f32, f32) {
    let left = (base - output).clamp(0.0, 1.0);
    let right = (base + output).clamp(0.0, 1.0);
    (left, right)
}

fn main() {
    let args = Args::parse();

    let mut seesaw = Seesaw::new(
        args.arm_length,
        args.inertia,
        // Use a lower value on the right for the weak-motor experiment.
        Motor::new(MOTOR_TIME_CONSTANT, args.thrust_coeff),
        Motor::new(MOTOR_TIME_CONSTANT, args.thrust_coeff),
    );

    // The gyro works in rad/s, as the rest of the physics crate does.
    let mut gyro = Gyro::new(args.noise.to_radians(), args.delay, args.seed);

    let mut pid = Pid::new(args.kp, args.ki, args.kd);
    pid.out_min = -OUT_LIMIT;
    pid.out_max = OUT_LIMIT;
    pid.i_limit = I_LIMIT;
    pid.d_filter = args.d_filter;

    println!("time,target,rate,measured,angle,output,left,right");

    for step in 0..STEPS {
        let time = step as f32 * DT;
        let target = target_at(time);

        // 1. Read the gyro. `rate` is the true rate and `measured` is what
        //    the PID is told, both in °/s.
        let rate = seesaw.rate().to_degrees();
        let angle = seesaw.angle().to_degrees();
        let measured = gyro.read(seesaw.rate()).to_degrees();

        // 2. Run the PID.
        let output = pid.update(target, measured, DT);

        // 3. Mix the output into two motor commands.
        let (left, right) = mix(BASE, output);

        // 4. Step the seesaw.
        seesaw.step(left, right, DT);

        // 5. Print one CSV line.
        println!("{time:.3},{target:.1},{rate:.3},{measured:.3},{angle:.3},{output:.5},{left:.5},{right:.5}");
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_target_is_zero_before_the_step() {
        assert!(target_at(0.0) == 0.0);
        assert!(target_at(STEP_START - 0.001) == 0.0);
    }

    #[test]
    fn test_target_during_the_step() {
        assert!(target_at(STEP_START) == STEP_TARGET);
        assert!(target_at((STEP_START + STEP_END) / 2.0) == STEP_TARGET);
        assert!(target_at(STEP_END - 0.001) == STEP_TARGET);
    }

    #[test]
    fn test_target_is_zero_after_the_step() {
        assert!(target_at(STEP_END) == 0.0);
        assert!(target_at(STEP_END + 1.0) == 0.0);
    }

    #[test]
    fn test_mix_zero_output_gives_base_on_both() {
        assert!(mix(0.7, 0.0) == (0.7, 0.7));
    }

    #[test]
    fn test_mix_positive_output_right_stronger() {
        let (left, right) = mix(0.7, 0.05);
        assert!((left - 0.65).abs() < 1e-6);
        assert!((right - 0.75).abs() < 1e-6);
    }

    #[test]
    fn test_mix_negative_output_left_stronger() {
        let (left, right) = mix(0.7, -0.05);
        assert!((left - 0.75).abs() < 1e-6);
        assert!((right - 0.65).abs() < 1e-6);
    }

    #[test]
    fn test_mix_clamps_to_zero_and_one() {
        // 0.7 + 0.5 is past full speed; 0.7 - 0.9 is below stopped.
        assert!(mix(0.7, 0.5) == (0.7 - 0.5, 1.0));
        assert!(mix(0.7, -0.9) == (1.0, 0.0));
    }
}
