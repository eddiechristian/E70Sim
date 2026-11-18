//! The X mixer: throttle and three torque commands in, four motor commands
//! out.
//!
//! Explained in `Stage2-a.md` in the repo root. It is the physics of the
//! body run backwards, and has no state.

/// What the mixer is asked for: throttle and the three torque commands.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MixerInput {
    pub throttle: f32, // the same on all four motors
    pub roll: f32,     // from the roll rate PID; positive rolls right
    pub pitch: f32,    // from the pitch rate PID; positive raises the nose
    pub yaw: f32,      // from the yaw rate PID; positive turns the nose right
}

/// One command per motor, each 0.0 to 1.0. The motors are named by position,
/// seen from above. The body in the `physics` crate has its own type with the
/// same four names; the tests in `drone/sim/tests` copy one into the other.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MotorCommands {
    pub fl: f32, // front left, spins clockwise
    pub fr: f32, // front right, spins counter-clockwise
    pub br: f32, // back right, spins clockwise
    pub bl: f32, // back left, spins counter-clockwise
}

/// Turns throttle and the roll, pitch and yaw commands into four motor
/// commands, each clamped to 0.0 to 1.0.
///
///    fl = throttle + roll + pitch − yaw
///    fr = throttle − roll + pitch + yaw
///    br = throttle − roll − pitch − yaw
///    bl = throttle + roll − pitch + yaw
pub fn mix(input: MixerInput) -> MotorCommands {
    let MixerInput {
        throttle,
        roll,
        pitch,
        yaw,
    } = input;

    MotorCommands {
        fl: (throttle + roll + pitch - yaw).clamp(0.0, 1.0),
        fr: (throttle - roll + pitch + yaw).clamp(0.0, 1.0),
        br: (throttle - roll - pitch - yaw).clamp(0.0, 1.0),
        bl: (throttle + roll - pitch + yaw).clamp(0.0, 1.0),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const TOLERANCE: f32 = 1e-6;

    // Mixes with throttle 0.7 and the given roll, pitch and yaw commands.
    fn mix_at_0_7(roll: f32, pitch: f32, yaw: f32) -> MotorCommands {
        mix(MixerInput {
            throttle: 0.7,
            roll,
            pitch,
            yaw,
        })
    }

    // Checks the four commands against the expected ones.
    fn assert_commands(commands: MotorCommands, fl: f32, fr: f32, br: f32, bl: f32) {
        let got = [commands.fl, commands.fr, commands.br, commands.bl];
        let expected = [fl, fr, br, bl];
        for (name, (got, expected)) in ["fl", "fr", "br", "bl"]
            .iter()
            .zip(got.iter().zip(expected.iter()))
        {
            assert!(
                (got - expected).abs() < TOLERANCE,
                "{name}: got {got}, expected {expected}"
            );
        }
    }

    #[test]
    fn test_throttle_only() {
        // Throttle 0.7, the rest 0: all four 0.7.
        let commands = mix_at_0_7(0.0, 0.0, 0.0);
        assert_commands(commands, 0.7, 0.7, 0.7, 0.7);
    }

    #[test]
    fn test_roll() {
        // Throttle 0.7, roll 0.05: FL and BL 0.75, FR and BR 0.65.
        let commands = mix_at_0_7(0.05, 0.0, 0.0);
        assert_commands(commands, 0.75, 0.65, 0.65, 0.75);
    }

    #[test]
    fn test_pitch() {
        // Throttle 0.7, pitch 0.05: FL and FR 0.75, BR and BL 0.65.
        let commands = mix_at_0_7(0.0, 0.05, 0.0);
        assert_commands(commands, 0.75, 0.75, 0.65, 0.65);
    }

    #[test]
    fn test_yaw() {
        // Throttle 0.7, yaw 0.05: FR and BL 0.75, FL and BR 0.65.
        let commands = mix_at_0_7(0.0, 0.0, 0.05);
        assert_commands(commands, 0.65, 0.75, 0.65, 0.75);
    }

    #[test]
    fn test_sum_unchanged() {
        // Any roll, pitch and yaw that do not clamp: the four commands add
        // up to 4 × throttle.
        for (throttle, roll, pitch, yaw) in [
            (0.7, 0.05, 0.0, 0.0),
            (0.5, 0.03, -0.02, 0.04),
            (0.3, -0.1, 0.1, -0.05),
        ] {
            let commands = mix(MixerInput {
                throttle,
                roll,
                pitch,
                yaw,
            });
            let sum = commands.fl + commands.fr + commands.br + commands.bl;
            assert!(
                (sum - 4.0 * throttle).abs() < 1e-5,
                "sum {sum}, expected {}",
                4.0 * throttle
            );
        }
    }

    #[test]
    fn test_clamp() {
        // Throttle 0.9, roll 0.3: no command above 1.0 or below 0.0.
        let high = mix(MixerInput {
            throttle: 0.9,
            roll: 0.3,
            pitch: 0.0,
            yaw: 0.0,
        });
        assert_commands(high, 1.0, 0.6, 0.6, 1.0);

        // The same at the bottom: throttle 0.1, roll 0.3.
        let low = mix(MixerInput {
            throttle: 0.1,
            roll: 0.3,
            pitch: 0.0,
            yaw: 0.0,
        });
        assert_commands(low, 0.4, 0.0, 0.0, 0.4);
    }
}
