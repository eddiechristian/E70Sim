//! Rate loops on the quad: three PIDs, the X mixer and four motors, CSV on
//! standard output.
//!
//! This is the loop of `src/main.rs` with three axes in place of one. The
//! parts are explained in `Stage2-a.md` in the repo root.
//!
//! Run with `cargo run --bin quad > quad.csv`, then plot with
//! `python3 plot_quad.py`. `cargo run --bin quad -- --help` lists the
//! arguments.

use clap::{Parser, ValueEnum};
use flight::mixer::{MixerInput, mix};
use flight::pid::Pid;
use physics::gyro::Gyro;
use physics::motor::Motor;
use physics::quad::{Inertia, Motors, Quad};

// Time
const DT: f32 = 0.001; // s; one step, 1 kHz
const STEPS: u32 = 3000; // 3 simulated seconds

// The run: every target 0, then `--target` on the chosen axis from
// STEP_START to STEP_END, then 0 again.
const STEP_START: f32 = 0.5; // s
const STEP_END: f32 = 1.5; // s

// Mixer
const THROTTLE: f32 = 0.7; // shared by all four motors, 0.0 to 1.0

// PID limits; the outputs are in throttle units.
const OUT_LIMIT: f32 = 0.3; // each output stays within ±OUT_LIMIT
const I_LIMIT: f32 = 0.1;

// The body: the starting values from `Stage2-a.md`.
const ARM_LENGTH: f32 = 0.2; // m
const YAW_COEFF: f32 = 0.015; // m
const IXX: f32 = 0.015; // kg·m²
const IYY: f32 = 0.015; // kg·m²
const IZZ: f32 = 0.025; // kg·m²
const MOTOR_TIME_CONSTANT: f32 = 0.04; // s

/// The axis that gets the step.
#[derive(Clone, Copy, Debug, PartialEq, ValueEnum)]
enum Axis {
    Roll,
    Pitch,
    Yaw,
}

/// One value per axis, in the flight code's units: °/s for rates and
/// throttle for PID outputs.
#[derive(Clone, Copy, Debug, PartialEq)]
struct PerAxis {
    roll: f32,
    pitch: f32,
    yaw: f32,
}

/// Rate loops on the quad. Writes CSV to standard output.
#[derive(Parser)]
struct Args {
    /// Which axis gets the step
    #[arg(long, value_enum, default_value_t = Axis::Roll)]
    axis: Axis,

    /// Target rate during the step, in °/s
    #[arg(long, default_value_t = 50.0)]
    target: f32,

    /// Roll and pitch proportional gain, in throttle per °/s
    #[arg(long, default_value_t = 0.0014)]
    kp: f32,

    /// Roll and pitch integral gain
    #[arg(long, default_value_t = 0.0)]
    ki: f32,

    /// Roll and pitch derivative gain
    #[arg(long, default_value_t = 0.00003)]
    kd: f32,

    /// Yaw proportional gain, in throttle per °/s
    #[arg(long, default_value_t = 0.01)]
    yaw_kp: f32,

    /// Yaw integral gain
    #[arg(long, default_value_t = 0.0)]
    yaw_ki: f32,

    /// Yaw derivative gain
    #[arg(long, default_value_t = 0.0)]
    yaw_kd: f32,

    /// Time constant of the D-term low-pass filter on all three axes, in
    /// seconds; 0 for none
    #[arg(long, default_value_t = 0.01)]
    d_filter: f32,

    /// Thrust of each motor at full speed, in newtons
    #[arg(long, default_value_t = 6.0)]
    thrust_coeff: f32,

    /// Gyro noise: standard deviation in °/s
    #[arg(long, default_value_t = 0.0)]
    noise: f32,

    /// Loop delay: the PIDs see the readings from this many steps ago
    #[arg(long, default_value_t = 0)]
    delay: usize,

    /// Seed for the gyro noise; the same seed gives the same run
    #[arg(long, default_value_t = 1)]
    seed: u64,
}

/// The three target rates in °/s at `time` seconds: `target` on `axis`
/// during the step, and 0 everywhere else.
fn targets_at(time: f32, axis: Axis, target: f32) -> PerAxis {
    let mut targets = PerAxis {
        roll: 0.0,
        pitch: 0.0,
        yaw: 0.0,
    };
    if (STEP_START..STEP_END).contains(&time) {
        match axis {
            Axis::Roll => targets.roll = target,
            Axis::Pitch => targets.pitch = target,
            Axis::Yaw => targets.yaw = target,
        }
    }
    targets
}

/// A rate PID with the limits and the D-term filter set.
fn rate_pid(kp: f32, ki: f32, kd: f32, d_filter: f32) -> Pid {
    let mut pid = Pid::new(kp, ki, kd);
    pid.out_min = -OUT_LIMIT;
    pid.out_max = OUT_LIMIT;
    pid.i_limit = I_LIMIT;
    pid.d_filter = d_filter;
    pid
}

/// The mixer and the body each have their own `MotorCommands`. This copies
/// one into the other by name.
fn to_body(commands: flight::mixer::MotorCommands) -> physics::quad::MotorCommands {
    physics::quad::MotorCommands {
        fl: commands.fl,
        fr: commands.fr,
        br: commands.br,
        bl: commands.bl,
    }
}

fn main() {
    let args = Args::parse();

    let motor = || Motor::new(MOTOR_TIME_CONSTANT, args.thrust_coeff);
    let mut quad = Quad::new(
        ARM_LENGTH,
        YAW_COEFF,
        Inertia {
            ixx: IXX,
            iyy: IYY,
            izz: IZZ,
        },
        Motors {
            fl: motor(),
            fr: motor(),
            br: motor(),
            bl: motor(),
        },
    );

    // One gyro per axis, each with its own noise. They work in rad/s, as the
    // rest of the physics crate does.
    let noise = args.noise.to_radians();
    let mut roll_gyro = Gyro::new(noise, args.delay, args.seed);
    let mut pitch_gyro = Gyro::new(noise, args.delay, args.seed + 1);
    let mut yaw_gyro = Gyro::new(noise, args.delay, args.seed + 2);

    let mut roll_pid = rate_pid(args.kp, args.ki, args.kd, args.d_filter);
    let mut pitch_pid = rate_pid(args.kp, args.ki, args.kd, args.d_filter);
    let mut yaw_pid = rate_pid(args.yaw_kp, args.yaw_ki, args.yaw_kd, args.d_filter);

    println!(
        "time,\
         roll_target,pitch_target,yaw_target,\
         roll_rate,pitch_rate,yaw_rate,\
         roll_measured,pitch_measured,yaw_measured,\
         roll_angle,pitch_angle,yaw_angle,\
         roll_output,pitch_output,yaw_output,\
         fl,fr,br,bl"
    );

    for step in 0..STEPS {
        let time = step as f32 * DT;
        let target = targets_at(time, args.axis, args.target);

        // 1. Read the three gyros. `rate` is the true rate and `measured` is
        //    what the PIDs are told, both in °/s.
        let true_rates = quad.rates();
        let rate = PerAxis {
            roll: true_rates.roll.to_degrees(),
            pitch: true_rates.pitch.to_degrees(),
            yaw: true_rates.yaw.to_degrees(),
        };
        let measured = PerAxis {
            roll: roll_gyro.read(true_rates.roll).to_degrees(),
            pitch: pitch_gyro.read(true_rates.pitch).to_degrees(),
            yaw: yaw_gyro.read(true_rates.yaw).to_degrees(),
        };
        let true_angles = quad.angles();
        let angle = PerAxis {
            roll: true_angles.roll.to_degrees(),
            pitch: true_angles.pitch.to_degrees(),
            yaw: true_angles.yaw.to_degrees(),
        };

        // 2. Run the three PIDs.
        let output = PerAxis {
            roll: roll_pid.update(target.roll, measured.roll, DT),
            pitch: pitch_pid.update(target.pitch, measured.pitch, DT),
            yaw: yaw_pid.update(target.yaw, measured.yaw, DT),
        };

        // 3. Mix the three outputs into four motor commands.
        let commands = mix(MixerInput {
            throttle: THROTTLE,
            roll: output.roll,
            pitch: output.pitch,
            yaw: output.yaw,
        });

        // 4. Step the body.
        quad.step(to_body(commands), DT);

        // 5. Print one CSV line.
        println!(
            "{time:.3},\
             {:.1},{:.1},{:.1},\
             {:.3},{:.3},{:.3},\
             {:.3},{:.3},{:.3},\
             {:.3},{:.3},{:.3},\
             {:.5},{:.5},{:.5},\
             {:.5},{:.5},{:.5},{:.5}",
            target.roll,
            target.pitch,
            target.yaw,
            rate.roll,
            rate.pitch,
            rate.yaw,
            measured.roll,
            measured.pitch,
            measured.yaw,
            angle.roll,
            angle.pitch,
            angle.yaw,
            output.roll,
            output.pitch,
            output.yaw,
            commands.fl,
            commands.fr,
            commands.br,
            commands.bl,
        );
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const ZERO: PerAxis = PerAxis {
        roll: 0.0,
        pitch: 0.0,
        yaw: 0.0,
    };

    #[test]
    fn test_targets_are_zero_outside_the_step() {
        for axis in [Axis::Roll, Axis::Pitch, Axis::Yaw] {
            assert_eq!(targets_at(0.0, axis, 50.0), ZERO);
            assert_eq!(targets_at(STEP_START - 0.001, axis, 50.0), ZERO);
            assert_eq!(targets_at(STEP_END, axis, 50.0), ZERO);
            assert_eq!(targets_at(STEP_END + 1.0, axis, 50.0), ZERO);
        }
    }

    #[test]
    fn test_only_the_chosen_axis_gets_the_step() {
        let time = (STEP_START + STEP_END) / 2.0;
        assert_eq!(
            targets_at(time, Axis::Roll, 50.0),
            PerAxis { roll: 50.0, ..ZERO }
        );
        assert_eq!(
            targets_at(time, Axis::Pitch, 50.0),
            PerAxis {
                pitch: 50.0,
                ..ZERO
            }
        );
        assert_eq!(
            targets_at(time, Axis::Yaw, -20.0),
            PerAxis { yaw: -20.0, ..ZERO }
        );
    }

    #[test]
    fn test_step_starts_and_ends_on_time() {
        assert_eq!(targets_at(STEP_START, Axis::Roll, 50.0).roll, 50.0);
        assert_eq!(targets_at(STEP_END - 0.001, Axis::Roll, 50.0).roll, 50.0);
    }

    #[test]
    fn test_rate_pid_has_the_limits() {
        let pid = rate_pid(0.001, 0.0, 0.0, 0.01);
        assert_eq!(pid.out_min, -OUT_LIMIT);
        assert_eq!(pid.out_max, OUT_LIMIT);
        assert_eq!(pid.i_limit, I_LIMIT);
        assert_eq!(pid.d_filter, 0.01);
    }
}
