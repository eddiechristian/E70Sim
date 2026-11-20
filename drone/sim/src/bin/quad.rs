//! The quad in the whole loop: sensors, the `Controller` of the flight
//! crate, four motors and the body, CSV on standard output.
//!
//! The body is free (`Stage3-a.md`): it stands on the ground through one
//! second of calibration, climbs, gets the hover command, and is then asked
//! for a step in angle on roll or pitch, or a step in rate on yaw. With
//! `--support joint` it hangs on the ball joint of Stage 2 for a shorter run
//! with the same step. The loop is explained in `Stage2-a.md`, `Stage2-b.md`
//! and `Stage2-c.md` in the repo root.
//!
//! A run can also be disturbed (`Stage2-d.md`): a kick, which is a torque on
//! the body for a short time, and one weak motor.
//!
//! With `--rate-only` the program runs as it did before Stage 2c: three rate
//! PIDs and the mixer, no angle loops and no calibration, with the estimator
//! only watching.
//!
//! `--stage` runs the program as it ran at an earlier stage: `--stage 2c`
//! sets every argument that has changed since Stage 2c back to what it was
//! then. Each stage's script passes its own.
//!
//! Run with `cargo run --bin quad > quad.csv`, then plot with
//! `python3 plot_angles.py`. `cargo run --bin quad -- --help` lists the
//! arguments.

use clap::parser::ValueSource;
use clap::{CommandFactory, FromArgMatches, Parser, ValueEnum};
use flight::calibration::CALIBRATION_STEPS;
use flight::controller::{Controller, Gains, State, Targets};
use flight::estimator::{Accel, Estimator, GyroRates};
use flight::mixer::{MixerInput, mix};
use flight::pid::Pid;
use physics::gyro::Gyro;
use physics::motor::Motor;
use physics::quad::{G, Inertia, Motors, NO_PUSH, Quad, Support, Torques};
use physics::quat::Angles;

// Time
const DT: f32 = 0.001; // s; one step, 1 kHz

/// When things happen in a run. The calibration takes the first second of
/// every run, with the motors off. Then every target is 0, then `--target`
/// on the chosen axis from `step_start` to `step_end`, then 0 again.
#[derive(Clone, Copy, Debug, PartialEq)]
struct Run {
    steps: u32,       // how long the run is
    step_start: f32,  // s
    step_end: f32,    // s
    climb_until: f32, // s; a free body gets the take-off throttle until here
}

// On the joint: 5 simulated seconds, as in Stage 2. There is no climb.
const JOINT_RUN: Run = Run {
    steps: 5000,
    step_start: 1.5,
    step_end: 3.5,
    climb_until: 0.0,
};

// Free: 12 simulated seconds. The drone climbs from the end of the
// calibration until 3 s, then gets the hover command, and has two seconds
// to settle before the step.
const FREE_RUN: Run = Run {
    steps: 12000,
    step_start: 5.0,
    step_end: 9.0,
    climb_until: 3.0,
};

// The run with `--rate-only`: no calibration, so everything is earlier.
const RATE_ONLY_STEPS: u32 = 3000; // 3 simulated seconds
const RATE_ONLY_STEP_START: f32 = 0.5; // s
const RATE_ONLY_STEP_END: f32 = 1.5; // s

// The step when `--target` is not given.
const DEFAULT_ANGLE_TARGET: f32 = 20.0; // °; roll and pitch
const DEFAULT_RATE_TARGET: f32 = 50.0; // °/s; yaw, and all three with `--rate-only`

// Mixer
const THROTTLE: f32 = 0.7; // on the joint, and with `--rate-only`; 0.0 to 1.0

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

/// A stage of `drone.md` that this program has run, for `--stage`.
#[derive(Clone, Copy, Debug, PartialEq, ValueEnum)]
enum Stage {
    /// Three rate loops and the mixer
    #[value(name = "2a")]
    S2a,
    /// The same, with the estimator watching
    #[value(name = "2b")]
    S2b,
    /// The whole controller, with no I term in the rate loops
    #[value(name = "2c")]
    S2c,
    /// The rate loops have their I terms
    #[value(name = "2d")]
    S2d,
    /// A free body, with gravity and a ground, and a longer estimator time
    /// constant
    #[value(name = "3a")]
    S3a,
    /// More throttle in a tilt, to make up for the lift lost
    #[value(name = "3b")]
    S3b,
}

/// What holds the body up, for `--support`.
#[derive(Clone, Copy, Debug, PartialEq, ValueEnum)]
enum SupportArg {
    /// The ball joint of Stage 2: it turns but goes nowhere
    Joint,
    /// Nothing: it flies, falls and stands on the ground
    Free,
}

/// The arguments whose defaults have changed from one stage to another,
/// with the value each had at one stage.
///
/// When a later stage changes the default of another argument, that
/// argument gets a field here, and every stage gets its value for it. The
/// earlier stages then keep running as they did.
#[derive(Clone, Copy, Debug, PartialEq)]
struct Preset {
    rate_only: bool,
    ki: f32,
    yaw_ki: f32,
    support: SupportArg,
    estimator_tau: f32,
    tilt_compensation: bool,
}

/// What `stage` used.
fn preset(stage: Stage) -> Preset {
    match stage {
        // Before the angle loops and the calibration.
        Stage::S2a | Stage::S2b => Preset {
            rate_only: true,
            ki: 0.0,
            yaw_ki: 0.0,
            support: SupportArg::Joint,
            estimator_tau: 1.0,
            tilt_compensation: false,
        },
        // Before the rate loops had an I term.
        Stage::S2c => Preset {
            rate_only: false,
            ki: 0.0,
            yaw_ki: 0.0,
            support: SupportArg::Joint,
            estimator_tau: 1.0,
            tilt_compensation: false,
        },
        // Before the joint was taken away.
        Stage::S2d => Preset {
            rate_only: false,
            ki: 0.003,
            yaw_ki: 0.02,
            support: SupportArg::Joint,
            estimator_tau: 1.0,
            tilt_compensation: false,
        },
        // Before the throttle was raised in a tilt.
        Stage::S3a => Preset {
            rate_only: false,
            ki: 0.003,
            yaw_ki: 0.02,
            support: SupportArg::Free,
            estimator_tau: 10.0,
            tilt_compensation: false,
        },
        // These are also the defaults of the arguments below, and a test
        // checks that they stay the same.
        Stage::S3b => Preset {
            rate_only: false,
            ki: 0.003,
            yaw_ki: 0.02,
            support: SupportArg::Free,
            estimator_tau: 10.0,
            tilt_compensation: true,
        },
    }
}

/// One of the four motors, to pick the weak one.
#[derive(Clone, Copy, Debug, PartialEq, ValueEnum)]
enum Corner {
    /// Front left
    Fl,
    /// Front right
    Fr,
    /// Back right
    Br,
    /// Back left
    Bl,
}

/// One value per axis, in the flight code's units: °/s for rates and
/// throttle for PID outputs.
#[derive(Clone, Copy, Debug, PartialEq)]
struct PerAxis {
    roll: f32,
    pitch: f32,
    yaw: f32,
}

/// The quad in the whole loop. Writes CSV to standard output.
#[derive(Parser)]
struct Args {
    /// Run as the program ran at this stage: sets the arguments that have
    /// changed since to what they were then. Arguments given as well win
    /// over it [default: the latest stage]
    #[arg(long, value_enum)]
    stage: Option<Stage>,

    /// Which axis gets the step
    #[arg(long, value_enum, default_value_t = Axis::Roll)]
    axis: Axis,

    /// Target during the step: an angle in degrees for roll and pitch
    /// [default: 20], a rate in °/s for yaw and with --rate-only
    /// [default: 50]
    #[arg(long, allow_negative_numbers = true)]
    target: Option<f32>,

    /// Angle loop gain for roll and pitch, in °/s per degree
    #[arg(long, default_value_t = 6.0)]
    angle_kp: f32,

    /// The angle loops ask for no more than this rate, in °/s
    #[arg(long, default_value_t = 200.0)]
    rate_limit: f32,

    /// Fly without what the calibration finds: the gyro bias is taken as
    /// zero and the starting angle comes from one accelerometer reading.
    /// The motors still stay off for the first second
    #[arg(long)]
    no_calibration: bool,

    /// Leave the throttle alone in a tilt, as before Stage 3b. Without
    /// this it is raised to make up for the lift the tilt takes away
    #[arg(long)]
    no_tilt_compensation: bool,

    /// How far off the hover throttle of a free body is, as a fraction:
    /// 0.01 flies with 1% too much
    #[arg(long, default_value_t = 0.0, allow_negative_numbers = true)]
    throttle_error: f32,

    /// Run as before Stage 2c: rate loops only, with --target a rate on
    /// any axis, no calibration, and a 3 s run with the step at 0.5 s
    #[arg(long)]
    rate_only: bool,

    /// Roll the body starts at, in degrees
    #[arg(long, default_value_t = 0.0, allow_negative_numbers = true)]
    start_roll: f32,

    /// Pitch the body starts at, in degrees
    #[arg(long, default_value_t = 0.0, allow_negative_numbers = true)]
    start_pitch: f32,

    /// Roll and pitch rate loop proportional gain, in throttle per °/s
    #[arg(long, default_value_t = 0.0014)]
    kp: f32,

    /// Roll and pitch rate loop integral gain
    #[arg(long, default_value_t = 0.003)]
    ki: f32,

    /// Roll and pitch rate loop derivative gain
    #[arg(long, default_value_t = 0.00003)]
    kd: f32,

    /// Yaw proportional gain, in throttle per °/s
    #[arg(long, default_value_t = 0.01)]
    yaw_kp: f32,

    /// Yaw integral gain
    #[arg(long, default_value_t = 0.02)]
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

    /// What holds the body up. Free, the run is 12 s with the step from
    /// 5 s to 9 s; on the joint it is 5 s with the step from 1.5 s to 3.5 s.
    /// --rate-only is always on the joint
    #[arg(long, value_enum, default_value_t = SupportArg::Free)]
    support: SupportArg,

    /// Mass of a free body, in kg
    #[arg(long, default_value_t = 1.2)]
    mass: f32,

    /// Drag on a free body, in newtons per m/s of speed
    #[arg(long, default_value_t = 0.4)]
    drag: f32,

    /// Throttle of a free body while it climbs, from the end of the
    /// calibration until 3 s. After that it gets the hover command
    #[arg(long, default_value_t = 0.76)]
    takeoff_throttle: f32,

    /// A kick: a torque about roll, in N·m, for a short time. The three
    /// kicks can be given together. Not with --rate-only
    #[arg(long, default_value_t = 0.0, allow_negative_numbers = true)]
    kick_roll: f32,

    /// A kick about pitch, in N·m
    #[arg(long, default_value_t = 0.0, allow_negative_numbers = true)]
    kick_pitch: f32,

    /// A kick about yaw, in N·m
    #[arg(long, default_value_t = 0.0, allow_negative_numbers = true)]
    kick_yaw: f32,

    /// When the kick starts, in seconds from the start of the run
    #[arg(long, default_value_t = 2.0)]
    kick_at: f32,

    /// How long the kick lasts, in seconds
    #[arg(long, default_value_t = 0.05)]
    kick_for: f32,

    /// One motor that is weaker than the other three, from the start
    #[arg(long, value_enum)]
    weak_motor: Option<Corner>,

    /// How much weaker the weak motor is, as a percentage of its thrust
    #[arg(long, default_value_t = 10.0)]
    weak_by: f32,

    /// Gyro noise: standard deviation in °/s
    #[arg(long, default_value_t = 0.0)]
    noise: f32,

    /// Loop delay: the flight code sees the gyro readings from this many
    /// steps ago
    #[arg(long, default_value_t = 0)]
    delay: usize,

    /// Gyro bias: a constant error on all three axes, in °/s
    #[arg(long, default_value_t = 0.0)]
    gyro_bias: f32,

    /// Accelerometer noise: standard deviation on each axis, in m/s²
    #[arg(long, default_value_t = 0.0)]
    accel_noise: f32,

    /// Time constant of the estimator, in seconds
    #[arg(long, default_value_t = 10.0)]
    estimator_tau: f32,

    /// Seed for the gyro noise; the same seed gives the same run
    #[arg(long, default_value_t = 1)]
    seed: u64,
}

/// Reads the arguments from `from`, the program's name first, and applies
/// `--stage`. An argument given on the command line keeps its value; the
/// stage sets only the ones left alone.
fn parse_args<I, T>(from: I) -> Args
where
    I: IntoIterator<Item = T>,
    T: Into<std::ffi::OsString> + Clone,
{
    let matches = Args::command().get_matches_from(from);
    let mut args = Args::from_arg_matches(&matches).unwrap_or_else(|error| error.exit());

    if let Some(stage) = args.stage {
        let given = |name: &str| matches.value_source(name) == Some(ValueSource::CommandLine);
        let preset = preset(stage);
        if !given("rate_only") {
            args.rate_only = preset.rate_only;
        }
        if !given("ki") {
            args.ki = preset.ki;
        }
        if !given("yaw_ki") {
            args.yaw_ki = preset.yaw_ki;
        }
        if !given("support") {
            args.support = preset.support;
        }
        if !given("estimator_tau") {
            args.estimator_tau = preset.estimator_tau;
        }
        if !given("no_tilt_compensation") {
            args.no_tilt_compensation = !preset.tilt_compensation;
        }
    }
    args
}

/// The timing of the run for a body held up by `support`.
fn run_for(support: SupportArg) -> Run {
    match support {
        SupportArg::Joint => JOINT_RUN,
        SupportArg::Free => FREE_RUN,
    }
}

/// The throttle asked for at `time` seconds. On the joint it is `THROTTLE`
/// throughout. A free body gets `--takeoff-throttle` until the climb is
/// over. After that it gets `hover`, the body's hover command, made wrong
/// by `--throttle-error` if that is given.
fn throttle_at(time: f32, args: &Args, hover: f32) -> f32 {
    match args.support {
        SupportArg::Joint => THROTTLE,
        SupportArg::Free if time < FREE_RUN.climb_until => args.takeoff_throttle,
        SupportArg::Free => hover * (1.0 + args.throttle_error),
    }
}

/// What the controller is asked for at `time` seconds of `run`: level, no
/// yaw and `throttle`, with `target` on `axis` during the step. For roll
/// and pitch the target is an angle in degrees; for yaw it is a rate in °/s.
fn targets_at(time: f32, axis: Axis, target: f32, run: &Run, throttle: f32) -> Targets {
    let mut targets = Targets {
        roll: 0.0,
        pitch: 0.0,
        yaw_rate: 0.0,
        throttle,
    };
    if (run.step_start..run.step_end).contains(&time) {
        match axis {
            Axis::Roll => targets.roll = target,
            Axis::Pitch => targets.pitch = target,
            Axis::Yaw => targets.yaw_rate = target,
        }
    }
    targets
}

/// The push on the body during step number `step`: the three kicks while
/// the kick lasts, and nothing at any other time. The kick is counted in
/// whole steps, so that rounding cannot make it a step longer or shorter.
fn push_at(step: u32, args: &Args) -> Torques {
    let first = (args.kick_at / DT).round() as u32;
    let count = (args.kick_for / DT).round() as u32;
    if (first..first + count).contains(&step) {
        Torques {
            roll: args.kick_roll,
            pitch: args.kick_pitch,
            yaw: args.kick_yaw,
        }
    } else {
        NO_PUSH
    }
}

/// The four motors. All have `--thrust-coeff`, but for the one named by
/// `--weak-motor`, which has `--weak-by` percent less.
fn motors(args: &Args) -> Motors {
    let motor = |corner: Corner| {
        let weak = args.weak_motor == Some(corner);
        let by = if weak { args.weak_by / 100.0 } else { 0.0 };
        Motor::new(MOTOR_TIME_CONSTANT, args.thrust_coeff * (1.0 - by))
    };
    Motors {
        fl: motor(Corner::Fl),
        fr: motor(Corner::Fr),
        br: motor(Corner::Br),
        bl: motor(Corner::Bl),
    }
}

/// With `--rate-only`: the three target rates in °/s at `time` seconds,
/// `target` on `axis` during the step and 0 everywhere else.
fn rate_targets_at(time: f32, axis: Axis, target: f32) -> PerAxis {
    let mut targets = PerAxis {
        roll: 0.0,
        pitch: 0.0,
        yaw: 0.0,
    };
    if (RATE_ONLY_STEP_START..RATE_ONLY_STEP_END).contains(&time) {
        match axis {
            Axis::Roll => targets.roll = target,
            Axis::Pitch => targets.pitch = target,
            Axis::Yaw => targets.yaw = target,
        }
    }
    targets
}

/// The controller's gains and limits, from the arguments.
fn gains(args: &Args) -> Gains {
    Gains {
        angle_kp: args.angle_kp,
        rate_limit: args.rate_limit,
        rate_kp: args.kp,
        rate_ki: args.ki,
        rate_kd: args.kd,
        yaw_kp: args.yaw_kp,
        yaw_ki: args.yaw_ki,
        yaw_kd: args.yaw_kd,
        d_filter: args.d_filter,
        out_limit: OUT_LIMIT,
        i_limit: I_LIMIT,
        estimator_time_constant: args.estimator_tau,
        tilt_compensation: !args.no_tilt_compensation,
        // The hover throttle is the default's until `run_controller` has a
        // body to ask. The limit of the compensation is not an argument.
        ..Gains::default()
    }
}

/// With `--rate-only`: a rate PID with the limits and the D-term filter set.
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

/// The gyro and the accelerometer: the body's true values in, what the
/// flight code is told out.
struct Sensors {
    // One gyro per axis, each with its own noise. They work in rad/s, as
    // the rest of the physics crate does.
    roll_gyro: Gyro,
    pitch_gyro: Gyro,
    yaw_gyro: Gyro,

    // One noise source per accelerometer axis, in m/s². `Gyro` is a
    // noise-and-delay stage for any one number. The accelerometer gets no
    // delay: the queue of a delayed `Gyro` starts with zeros, which is not
    // what a still accelerometer reads.
    accel_x: Gyro,
    accel_y: Gyro,
    accel_z: Gyro,
}

/// One step's readings, and the true values next to them for the log.
struct Readings {
    rate: PerAxis,     // °/s; the true rates
    measured: PerAxis, // °/s; what the gyro says
    angle: PerAxis,    // °; the true attitude
    accel: Accel,      // g; what the accelerometer says
}

impl Sensors {
    fn new(args: &Args) -> Self {
        let noise = args.noise.to_radians();
        let mut roll_gyro = Gyro::new(noise, args.delay, args.seed);
        let mut pitch_gyro = Gyro::new(noise, args.delay, args.seed + 1);
        let mut yaw_gyro = Gyro::new(noise, args.delay, args.seed + 2);
        for gyro in [&mut roll_gyro, &mut pitch_gyro, &mut yaw_gyro] {
            gyro.bias = args.gyro_bias.to_radians();
        }
        Self {
            roll_gyro,
            pitch_gyro,
            yaw_gyro,
            accel_x: Gyro::new(args.accel_noise, 0, args.seed + 3),
            accel_y: Gyro::new(args.accel_noise, 0, args.seed + 4),
            accel_z: Gyro::new(args.accel_noise, 0, args.seed + 5),
        }
    }

    fn read(&mut self, quad: &Quad) -> Readings {
        let true_rates = quad.rates();
        let true_angles = quad.angles();
        let true_accel = quad.accel();
        Readings {
            rate: PerAxis {
                roll: true_rates.roll.to_degrees(),
                pitch: true_rates.pitch.to_degrees(),
                yaw: true_rates.yaw.to_degrees(),
            },
            measured: PerAxis {
                roll: self.roll_gyro.read(true_rates.roll).to_degrees(),
                pitch: self.pitch_gyro.read(true_rates.pitch).to_degrees(),
                yaw: self.yaw_gyro.read(true_rates.yaw).to_degrees(),
            },
            angle: PerAxis {
                roll: true_angles.roll.to_degrees(),
                pitch: true_angles.pitch.to_degrees(),
                yaw: true_angles.yaw.to_degrees(),
            },
            accel: Accel {
                x: self.accel_x.read(true_accel.x) / G,
                y: self.accel_y.read(true_accel.y) / G,
                z: self.accel_z.read(true_accel.z) / G,
            },
        }
    }
}

/// `accel` scaled to a length of 1 g. Its direction, and so the angles it
/// gives, do not change.
fn one_g_long(accel: Accel) -> Accel {
    let length = (accel.x * accel.x + accel.y * accel.y + accel.z * accel.z).sqrt();
    Accel {
        x: accel.x / length,
        y: accel.y / length,
        z: accel.z / length,
    }
}

fn main() {
    let mut args = parse_args(std::env::args_os());

    // The rate-only loop is from before the joint was taken away.
    if args.rate_only {
        args.support = SupportArg::Joint;
    }

    let inertia = Inertia {
        ixx: IXX,
        iyy: IYY,
        izz: IZZ,
    };
    let mut quad = match args.support {
        SupportArg::Joint => Quad::new(ARM_LENGTH, YAW_COEFF, inertia, motors(&args)),
        // Standing on the ground. A starting angle is then a slope.
        SupportArg::Free => Quad::new_free(
            ARM_LENGTH,
            YAW_COEFF,
            inertia,
            motors(&args),
            args.mass,
            args.drag,
        ),
    };
    quad.set_attitude(Angles {
        roll: args.start_roll.to_radians(),
        pitch: args.start_pitch.to_radians(),
        yaw: 0.0,
    });
    let mut sensors = Sensors::new(&args);

    if args.rate_only {
        run_rate_only(&args, &mut quad, &mut sensors);
    } else {
        run_controller(&args, &mut quad, &mut sensors);
    }
}

/// The whole loop: each step reads the sensors, calls the controller and
/// steps the body with the motor commands it returns.
fn run_controller(args: &Args, quad: &mut Quad, sensors: &mut Sensors) {
    let step_target = args.target.unwrap_or(match args.axis {
        Axis::Roll | Axis::Pitch => DEFAULT_ANGLE_TARGET,
        Axis::Yaw => DEFAULT_RATE_TARGET,
    });
    let run = run_for(args.support);
    // The hover command of the body that is flown, so that `--mass` and
    // `--thrust-coeff` change it too. On the joint, which has no mass, the
    // gains keep their default and `throttle_at` does not use `hover`.
    let mut gains = gains(args);
    let hover = match quad.support {
        Support::Joint => 0.0,
        Support::Free => quad.hover_command(),
    };
    if quad.support == Support::Free {
        gains.hover_throttle = hover;
    }
    let mut controller = Controller::new(gains);

    // With `--no-calibration`: the one accelerometer reading the controller
    // is shown while it calibrates.
    let mut first_accel = None;

    println!(
        "time,state,\
         roll_angle_target,pitch_angle_target,\
         roll_angle,pitch_angle,yaw_angle,\
         roll_estimate,pitch_estimate,\
         roll_target,pitch_target,yaw_target,\
         roll_rate,pitch_rate,yaw_rate,\
         roll_measured,pitch_measured,yaw_measured,\
         fl,fr,br,bl,\
         push_roll,push_pitch,push_yaw,\
         north,east,height,\
         north_speed,east_speed,climb,\
         accel_x,accel_y,accel_z,\
         on_ground,throttle,throttle_mixed"
    );

    for step in 0..run.steps {
        let time = step as f32 * DT;
        let throttle = throttle_at(time, args, hover);
        let targets = targets_at(time, args.axis, step_target, &run, throttle);

        // 1. Read the sensors, and where the body is for the log. On the
        //    joint it is always at the origin and never on the ground.
        let readings = sensors.read(quad);
        let position = quad.position();
        let velocity = quad.velocity();
        let on_ground = u8::from(quad.on_ground());
        let mut sensor_gyro = GyroRates {
            roll: readings.measured.roll,
            pitch: readings.measured.pitch,
            yaw: readings.measured.yaw,
        };
        let mut sensor_accel = readings.accel;

        //    Without calibration the controller would take the gyro bias as
        //    zero and its starting angle from the first accelerometer
        //    reading. Showing it a gyro reading of zero and that one
        //    accelerometer reading while it calibrates gives exactly that.
        //    The reading is made 1 g long so that noise on it cannot fail
        //    the calibration's check of its length.
        if args.no_calibration && controller.state() == State::Calibrating {
            sensor_gyro = GyroRates {
                roll: 0.0,
                pitch: 0.0,
                yaw: 0.0,
            };
            sensor_accel = *first_accel.get_or_insert(one_g_long(readings.accel));
        }

        // 2. Call the controller. The state logged is the one it was in
        //    when it worked out the commands.
        let state = match controller.state() {
            State::Calibrating => 0,
            State::Flying => 1,
        };
        let commands = controller.update(sensor_gyro, sensor_accel, targets, DT);

        //    A calibration window that failed: say why. The controller has
        //    started a new one.
        if controller.state() == State::Calibrating && (step + 1) % CALIBRATION_STEPS == 0 {
            eprintln!(
                "calibration failed at {:.3} s: {}",
                time + DT,
                controller.last_failure().unwrap_or("no reason given")
            );
        }

        // 3. Step the body, with the kick if this is its time.
        let push = push_at(step, args);
        quad.push = push;
        quad.step(to_body(commands), DT);

        // 4. Print one CSV line. There is no estimate while calibrating.
        let (roll_estimate, pitch_estimate) = match controller.attitude() {
            Some(attitude) => (attitude.roll, attitude.pitch),
            None => (f32::NAN, f32::NAN),
        };
        println!(
            "{time:.3},{state},\
             {:.1},{:.1},\
             {:.3},{:.3},{:.3},\
             {:.3},{:.3},\
             {:.2},{:.2},{:.2},\
             {:.3},{:.3},{:.3},\
             {:.3},{:.3},{:.3},\
             {:.5},{:.5},{:.5},{:.5},\
             {:.3},{:.3},{:.3},\
             {:.3},{:.3},{:.3},\
             {:.3},{:.3},{:.3},\
             {:.4},{:.4},{:.4},\
             {on_ground},{:.4},{:.4}",
            targets.roll,
            targets.pitch,
            readings.angle.roll,
            readings.angle.pitch,
            readings.angle.yaw,
            roll_estimate,
            pitch_estimate,
            controller.roll_rate_target(),
            controller.pitch_rate_target(),
            targets.yaw_rate,
            readings.rate.roll,
            readings.rate.pitch,
            readings.rate.yaw,
            readings.measured.roll,
            readings.measured.pitch,
            readings.measured.yaw,
            commands.fl,
            commands.fr,
            commands.br,
            commands.bl,
            push.roll,
            push.pitch,
            push.yaw,
            position.x,
            position.y,
            quad_height(position.z),
            velocity.x,
            velocity.y,
            quad_height(velocity.z),
            readings.accel.x,
            readings.accel.y,
            readings.accel.z,
            targets.throttle,
            controller.throttle(),
        );
    }
}

/// z points down, so a height or a climb is a z with its sign turned
/// round. Zero stays a plain zero, not −0.
fn quad_height(z: f32) -> f32 {
    if z == 0.0 { 0.0 } else { -z }
}

/// The loop as it was before Stage 2c: three rate PIDs and the mixer, held
/// here and not in a `Controller`. The estimator watches and its angles are
/// logged; it is not used for control.
fn run_rate_only(args: &Args, quad: &mut Quad, sensors: &mut Sensors) {
    let step_target = args.target.unwrap_or(DEFAULT_RATE_TARGET);
    let mut estimator = Estimator::new(args.estimator_tau);

    let mut roll_pid = rate_pid(args.kp, args.ki, args.kd, args.d_filter);
    let mut pitch_pid = rate_pid(args.kp, args.ki, args.kd, args.d_filter);
    let mut yaw_pid = rate_pid(args.yaw_kp, args.yaw_ki, args.yaw_kd, args.d_filter);

    println!(
        "time,\
         roll_target,pitch_target,yaw_target,\
         roll_rate,pitch_rate,yaw_rate,\
         roll_measured,pitch_measured,yaw_measured,\
         roll_angle,pitch_angle,yaw_angle,\
         roll_estimate,pitch_estimate,\
         roll_output,pitch_output,yaw_output,\
         fl,fr,br,bl"
    );

    for step in 0..RATE_ONLY_STEPS {
        let time = step as f32 * DT;
        let target = rate_targets_at(time, args.axis, step_target);

        // 1. Read the sensors. The estimator takes the same gyro readings
        //    as the PIDs.
        let Readings {
            rate,
            measured,
            angle,
            accel,
        } = sensors.read(quad);
        let gyro_rates = GyroRates {
            roll: measured.roll,
            pitch: measured.pitch,
            yaw: measured.yaw,
        };
        let estimate = estimator.update(gyro_rates, accel, DT);

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
             {:.3},{:.3},\
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
            estimate.roll,
            estimate.pitch,
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

    const LEVEL: Targets = Targets {
        roll: 0.0,
        pitch: 0.0,
        yaw_rate: 0.0,
        throttle: THROTTLE,
    };

    const AXES: [Axis; 3] = [Axis::Roll, Axis::Pitch, Axis::Yaw];

    // The targets of a run on the joint, where the throttle never changes.
    fn joint_targets_at(time: f32, axis: Axis, target: f32) -> Targets {
        targets_at(time, axis, target, &JOINT_RUN, THROTTLE)
    }

    // ---- The controller's targets

    #[test]
    fn test_targets_are_level_outside_the_step() {
        for axis in AXES {
            assert_eq!(joint_targets_at(0.0, axis, 20.0), LEVEL);
            assert_eq!(
                joint_targets_at(JOINT_RUN.step_start - 0.001, axis, 20.0),
                LEVEL
            );
            assert_eq!(joint_targets_at(JOINT_RUN.step_end, axis, 20.0), LEVEL);
            assert_eq!(
                joint_targets_at(JOINT_RUN.step_end + 1.0, axis, 20.0),
                LEVEL
            );
        }
    }

    #[test]
    fn test_only_the_chosen_axis_gets_the_step() {
        let time = (JOINT_RUN.step_start + JOINT_RUN.step_end) / 2.0;
        assert_eq!(
            joint_targets_at(time, Axis::Roll, 20.0),
            Targets {
                roll: 20.0,
                ..LEVEL
            }
        );
        assert_eq!(
            joint_targets_at(time, Axis::Pitch, 20.0),
            Targets {
                pitch: 20.0,
                ..LEVEL
            }
        );
        // On yaw the target is a rate.
        assert_eq!(
            joint_targets_at(time, Axis::Yaw, -30.0),
            Targets {
                yaw_rate: -30.0,
                ..LEVEL
            }
        );
    }

    #[test]
    fn test_step_starts_and_ends_on_time() {
        assert_eq!(
            joint_targets_at(JOINT_RUN.step_start, Axis::Roll, 20.0).roll,
            20.0
        );
        assert_eq!(
            joint_targets_at(JOINT_RUN.step_end - 0.001, Axis::Roll, 20.0).roll,
            20.0
        );
    }

    #[test]
    fn test_step_starts_after_the_calibration() {
        assert!(JOINT_RUN.step_start > CALIBRATION_STEPS as f32 * DT);
        assert!(JOINT_RUN.step_end < JOINT_RUN.steps as f32 * DT);
    }

    // ---- The free run

    #[test]
    fn test_free_run_is_longer_with_a_later_step() {
        assert_eq!(run_for(SupportArg::Joint), JOINT_RUN);
        assert_eq!(run_for(SupportArg::Free), FREE_RUN);

        // The same step, later. The throttle is whatever it is given.
        let level = Targets {
            throttle: 0.5,
            ..LEVEL
        };
        let at = |time: f32| targets_at(time, Axis::Pitch, -10.0, &FREE_RUN, 0.5);
        assert_eq!(at(0.0), level);
        assert_eq!(at(FREE_RUN.step_start - 0.001), level);
        assert_eq!(at(FREE_RUN.step_start).pitch, -10.0);
        assert_eq!(at(FREE_RUN.step_end - 0.001).pitch, -10.0);
        assert_eq!(at(FREE_RUN.step_end), level);
    }

    #[test]
    fn test_free_run_has_time_for_everything() {
        // The calibration, then the climb, then the step, then the end.
        let calibrated = CALIBRATION_STEPS as f32 * DT;
        assert!(FREE_RUN.climb_until > calibrated);
        const { assert!(FREE_RUN.step_start > FREE_RUN.climb_until) };
        assert!(FREE_RUN.step_end < FREE_RUN.steps as f32 * DT);
    }

    #[test]
    fn test_free_throttle_climbs_then_hovers() {
        let args = parse_args(["quad"]);
        assert_eq!(args.support, SupportArg::Free);
        let hover = 0.7002;
        assert_eq!(throttle_at(0.0, &args, hover), 0.76);
        assert_eq!(
            throttle_at(FREE_RUN.climb_until - 0.001, &args, hover),
            0.76
        );
        assert_eq!(throttle_at(FREE_RUN.climb_until, &args, hover), hover);
        assert_eq!(throttle_at(11.0, &args, hover), hover);

        let args = parse_args(["quad", "--takeoff-throttle", "0.8"]);
        assert_eq!(throttle_at(2.0, &args, hover), 0.8);
    }

    #[test]
    fn test_joint_throttle_never_changes() {
        let args = parse_args(["quad", "--support", "joint"]);
        for time in [0.0, 2.0, 4.9] {
            assert_eq!(throttle_at(time, &args, 0.0), THROTTLE);
        }
    }

    #[test]
    fn test_height_is_z_turned_round() {
        assert_eq!(quad_height(-5.0), 5.0);
        assert_eq!(quad_height(2.0), -2.0);
        // Not −0, which would print as "-0.000".
        assert!(quad_height(0.0).is_sign_positive());
    }

    #[test]
    fn test_default_gains_are_the_flight_crates() {
        // The arguments' defaults are written out a second time above, for
        // `--help`. They must stay the same as `Gains::default()`.
        let args = parse_args(["quad"]);
        assert_eq!(gains(&args), Gains::default());
    }

    // ---- `--stage`

    // The stage the program runs as when `--stage` is not given. Change it
    // when a new stage is added.
    const LATEST: Stage = Stage::S3b;

    // The settings a stage's preset covers, as the program ends up with
    // them.
    fn settings(args: &Args) -> Preset {
        Preset {
            rate_only: args.rate_only,
            ki: args.ki,
            yaw_ki: args.yaw_ki,
            support: args.support,
            estimator_tau: args.estimator_tau,
            tilt_compensation: !args.no_tilt_compensation,
        }
    }

    #[test]
    fn test_no_stage_is_the_latest_stage() {
        // The arguments' own defaults are the latest stage's preset.
        let args = parse_args(["quad"]);
        assert_eq!(args.stage, None);
        assert_eq!(settings(&args), preset(LATEST));
    }

    #[test]
    fn test_each_stage_gets_its_preset() {
        for &stage in Stage::value_variants() {
            let name = stage.to_possible_value().unwrap();
            let args = parse_args(["quad", "--stage", name.get_name()]);
            assert_eq!(args.stage, Some(stage));
            assert_eq!(
                settings(&args),
                preset(stage),
                "--stage {}",
                name.get_name()
            );
        }
    }

    #[test]
    fn test_stage_2a_and_2b_are_rate_only_with_no_i() {
        for name in ["2a", "2b"] {
            let args = parse_args(["quad", "--stage", name]);
            assert!(args.rate_only);
            assert_eq!(args.ki, 0.0);
            assert_eq!(args.yaw_ki, 0.0);
        }
    }

    #[test]
    fn test_stage_2c_has_the_stage_2c_gains() {
        // The whole controller on the joint, with the gains as they were
        // before the rate loops had an I term.
        let args = parse_args(["quad", "--stage", "2c"]);
        assert!(!args.rate_only);
        assert_eq!(args.support, SupportArg::Joint);
        assert_eq!(
            gains(&args),
            Gains {
                rate_ki: 0.0,
                yaw_ki: 0.0,
                estimator_time_constant: 1.0,
                tilt_compensation: false,
                ..Gains::default()
            }
        );
    }

    #[test]
    fn test_stage_2d_has_the_stage_2d_gains() {
        // Still on the joint, with the I terms and the estimator's time
        // constant of 1 s.
        let args = parse_args(["quad", "--stage", "2d"]);
        assert!(!args.rate_only);
        assert_eq!(args.support, SupportArg::Joint);
        assert_eq!(
            gains(&args),
            Gains {
                estimator_time_constant: 1.0,
                tilt_compensation: false,
                ..Gains::default()
            }
        );
    }

    #[test]
    fn test_stage_3a_is_free_with_no_tilt_compensation() {
        let args = parse_args(["quad", "--stage", "3a"]);
        assert!(!args.rate_only);
        assert_eq!(args.support, SupportArg::Free);
        assert_eq!(
            gains(&args),
            Gains {
                tilt_compensation: false,
                ..Gains::default()
            }
        );
    }

    #[test]
    fn test_stage_3b_has_todays_gains() {
        let args = parse_args(["quad", "--stage", "3b"]);
        assert_eq!(args.support, SupportArg::Free);
        assert_eq!(gains(&args), Gains::default());
        assert!(Gains::default().tilt_compensation);
    }

    #[test]
    fn test_tilt_compensation_can_be_turned_off() {
        let args = parse_args(["quad", "--no-tilt-compensation"]);
        assert!(!gains(&args).tilt_compensation);
        let args = parse_args(["quad", "--stage", "3b", "--no-tilt-compensation"]);
        assert!(!gains(&args).tilt_compensation);
        // An earlier stage has it off without being told.
        let args = parse_args(["quad", "--stage", "2d"]);
        assert!(!gains(&args).tilt_compensation);
    }

    #[test]
    fn test_throttle_error_makes_the_hover_throttle_wrong() {
        let hover = 0.7002;
        let args = parse_args(["quad", "--throttle-error", "0.014"]);
        // Not while it climbs.
        assert_eq!(throttle_at(2.0, &args, hover), 0.76);
        assert!((throttle_at(4.0, &args, hover) - 0.7100).abs() < 1e-4);

        let args = parse_args(["quad", "--throttle-error", "-0.014"]);
        assert!((throttle_at(4.0, &args, hover) - 0.6904).abs() < 1e-4);

        // Not on the joint.
        let args = parse_args(["quad", "--stage", "2d", "--throttle-error", "0.1"]);
        assert_eq!(throttle_at(4.0, &args, hover), THROTTLE);
    }

    #[test]
    fn test_support_and_tau_win_over_the_stage() {
        let args = parse_args(["quad", "--stage", "2d", "--support", "free"]);
        assert_eq!(args.support, SupportArg::Free);
        assert_eq!(args.estimator_tau, 1.0);

        let args = parse_args(["quad", "--stage", "3a", "--estimator-tau", "1"]);
        assert_eq!(args.support, SupportArg::Free);
        assert_eq!(args.estimator_tau, 1.0);
    }

    #[test]
    fn test_an_argument_wins_over_the_stage() {
        // Whichever comes first on the command line.
        for line in [
            ["quad", "--stage", "2c", "--ki", "0.01"],
            ["quad", "--ki", "0.01", "--stage", "2c"],
        ] {
            let args = parse_args(line);
            assert_eq!(args.ki, 0.01);
            // The stage still sets what was not given.
            assert_eq!(args.yaw_ki, 0.0);
        }

        // An argument given with its own default value still counts.
        let args = parse_args(["quad", "--stage", "2c", "--ki", "0.003"]);
        assert_eq!(args.ki, 0.003);

        let args = parse_args(["quad", "--stage", "2c", "--rate-only"]);
        assert!(args.rate_only);
    }

    #[test]
    fn test_a_stage_leaves_the_other_arguments_alone() {
        let args = parse_args(["quad", "--stage", "2a", "--axis", "yaw", "--noise", "2"]);
        assert_eq!(args.axis, Axis::Yaw);
        assert_eq!(args.noise, 2.0);
        assert_eq!(args.kp, 0.0014);
    }

    // ---- The kick and the weak motor

    #[test]
    fn test_no_kick_unless_asked_for() {
        let args = parse_args(["quad"]);
        for step in 0..JOINT_RUN.steps {
            assert_eq!(push_at(step, &args), NO_PUSH);
        }
    }

    #[test]
    fn test_kick_starts_and_ends_on_time() {
        // The defaults: from 2.0 s for 0.05 s, which is steps 2000 to 2049.
        let args = parse_args(["quad", "--kick-roll", "0.3"]);
        let kick = Torques {
            roll: 0.3,
            ..NO_PUSH
        };
        assert_eq!(push_at(1999, &args), NO_PUSH);
        assert_eq!(push_at(2000, &args), kick);
        assert_eq!(push_at(2049, &args), kick);
        assert_eq!(push_at(2050, &args), NO_PUSH);
        let pushed = (0..JOINT_RUN.steps).filter(|&step| push_at(step, &args) == kick);
        assert_eq!(pushed.count(), 50);

        // Moved and made longer: steps 1200 to 1299.
        let args = parse_args([
            "quad",
            "--kick-roll",
            "0.3",
            "--kick-at",
            "1.2",
            "--kick-for",
            "0.1",
        ]);
        assert_eq!(push_at(1199, &args), NO_PUSH);
        assert_eq!(push_at(1200, &args), kick);
        assert_eq!(push_at(1299, &args), kick);
        assert_eq!(push_at(1300, &args), NO_PUSH);
    }

    #[test]
    fn test_kicks_can_be_given_together() {
        let args = parse_args([
            "quad",
            "--kick-roll",
            "0.3",
            "--kick-pitch",
            "-0.3",
            "--kick-yaw",
            "0.1",
        ]);
        assert_eq!(
            push_at(2000, &args),
            Torques {
                roll: 0.3,
                pitch: -0.3,
                yaw: 0.1,
            }
        );
    }

    #[test]
    fn test_kick_comes_after_the_calibration() {
        let args = parse_args(["quad"]);
        assert!(args.kick_at > CALIBRATION_STEPS as f32 * DT);
        assert!(args.kick_at + args.kick_for < JOINT_RUN.steps as f32 * DT);
    }

    // The thrust of each motor of `motors` at full speed, in newtons, in
    // the order FL, FR, BR, BL.
    fn full_thrusts(motors: &mut Motors) -> [f32; 4] {
        let mut thrusts = [0.0; 4];
        let each = [
            &mut motors.fl,
            &mut motors.fr,
            &mut motors.br,
            &mut motors.bl,
        ];
        for (thrust, motor) in thrusts.iter_mut().zip(each) {
            // Two seconds at a command of 1.0 is 50 time constants.
            for _ in 0..2000 {
                motor.step(1.0, DT);
            }
            *thrust = motor.thrust();
        }
        thrusts
    }

    fn assert_thrusts(got: [f32; 4], expected: [f32; 4]) {
        for (got, expected) in got.iter().zip(expected) {
            assert!(
                (got - expected).abs() < 1e-3,
                "got {got}, expected {expected}"
            );
        }
    }

    #[test]
    fn test_four_good_motors_unless_asked_for() {
        let args = parse_args(["quad"]);
        assert_thrusts(full_thrusts(&mut motors(&args)), [6.0; 4]);
    }

    #[test]
    fn test_only_the_chosen_motor_is_weak() {
        // 10% weak unless `--weak-by` says otherwise: 5.4 N against 6.0 N.
        let args = parse_args(["quad", "--weak-motor", "fr"]);
        assert_thrusts(full_thrusts(&mut motors(&args)), [6.0, 5.4, 6.0, 6.0]);

        let args = parse_args(["quad", "--weak-motor", "bl", "--weak-by", "20"]);
        assert_thrusts(full_thrusts(&mut motors(&args)), [6.0, 6.0, 6.0, 4.8]);
    }

    #[test]
    fn test_one_g_long() {
        let accel = one_g_long(Accel {
            x: 0.0,
            y: 0.6,
            z: -1.2,
        });
        assert!((accel.y - 0.4472).abs() < 1e-4);
        assert!((accel.z + 0.8944).abs() < 1e-4);
    }

    // ---- With `--rate-only`

    #[test]
    fn test_rate_targets_are_zero_outside_the_step() {
        for axis in AXES {
            assert_eq!(rate_targets_at(0.0, axis, 50.0), ZERO);
            assert_eq!(
                rate_targets_at(RATE_ONLY_STEP_START - 0.001, axis, 50.0),
                ZERO
            );
            assert_eq!(rate_targets_at(RATE_ONLY_STEP_END, axis, 50.0), ZERO);
            assert_eq!(rate_targets_at(RATE_ONLY_STEP_END + 1.0, axis, 50.0), ZERO);
        }
    }

    #[test]
    fn test_only_the_chosen_axis_gets_the_rate_step() {
        let time = (RATE_ONLY_STEP_START + RATE_ONLY_STEP_END) / 2.0;
        assert_eq!(
            rate_targets_at(time, Axis::Roll, 50.0),
            PerAxis { roll: 50.0, ..ZERO }
        );
        assert_eq!(
            rate_targets_at(time, Axis::Pitch, 50.0),
            PerAxis {
                pitch: 50.0,
                ..ZERO
            }
        );
        assert_eq!(
            rate_targets_at(time, Axis::Yaw, -20.0),
            PerAxis { yaw: -20.0, ..ZERO }
        );
    }

    #[test]
    fn test_rate_step_starts_and_ends_on_time() {
        let start = rate_targets_at(RATE_ONLY_STEP_START, Axis::Roll, 50.0);
        let end = rate_targets_at(RATE_ONLY_STEP_END - 0.001, Axis::Roll, 50.0);
        assert_eq!(start.roll, 50.0);
        assert_eq!(end.roll, 50.0);
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
