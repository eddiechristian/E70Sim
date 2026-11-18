//! PID controller with output limits and integrator anti-windup.
//!
//! Explained in `Stage1-a.md` in the repo root.

pub struct Pid {
    // Tuning
    pub kp: f32,
    pub ki: f32,
    pub kd: f32,
    pub out_min: f32,
    pub out_max: f32,
    pub i_limit: f32,

    // State
    i_term: f32,                   // Ki × integral, in output units
    prev_measurement: Option<f32>, // None until the first update
}

impl Pid {
    pub fn update(&mut self, setpoint: f32, measurement: f32, dt: f32) -> f32 {
        let error = setpoint - measurement;
        let p = self.kp * error;

        // A bad dt is treated as "no time passed". Also catches NaN.
        if !(dt > 0.0) {
            debug_assert!(false, "Pid::update called with dt = {dt}");
            return (p + self.i_term).clamp(self.out_min, self.out_max);
        }

        // Derivative on the measurement; zero on the first call.
        let d = match self.prev_measurement {
            Some(prev) => -self.kd * (measurement - prev) / dt,
            None => 0.0,
        };
        self.prev_measurement = Some(measurement);

        // Would the output saturate with the I term as it stands?
        let unclamped = p + self.i_term + d;
        let pushing_further = (unclamped >= self.out_max && error > 0.0)
            || (unclamped <= self.out_min && error < 0.0);

        if !pushing_further {
            self.i_term += self.ki * error * dt;
            self.i_term = self.i_term.clamp(-self.i_limit, self.i_limit);
        }

        (p + self.i_term + d).clamp(self.out_min, self.out_max)
    }

    pub fn reset(&mut self) {
        self.i_term = 0.0;
        self.prev_measurement = None;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_p() {
        let mut pid = Pid {
            kp: 2.0,
            ki: 0.0,
            kd: 0.0,
            out_min: -20.0,
            out_max: 20.0,
            i_limit: 100.0,

            // State
            i_term: 0.0,            // Ki × integral, in output units
            prev_measurement: None, // None until the first update
        };
        let out = pid.update(10.0, 4.0, 0.001);
        println!("out: {}", out);
        assert!(out == 12.0);
    }

    #[test]
    fn test_p_sign() {
        let mut pid = Pid {
            kp: 2.0,
            ki: 0.0,
            kd: 0.0,
            out_min: -20.0,
            out_max: 20.0,
            i_limit: 100.0,

            // State
            i_term: 0.0,            // Ki × integral, in output units
            prev_measurement: None, // None until the first update
        };
        let out = pid.update(10.0, 16.0, 0.001);
        println!("out: {}", out);

        assert!(out == -12.0);
    }

    #[test]
    fn test_i_accumlates() {
        let mut pid = Pid {
            kp: 0.0,
            ki: 2.0,
            kd: 0.0,
            out_min: -20.0,
            out_max: 20.0,
            i_limit: 100.0,

            // State
            i_term: 0.0,            // Ki × integral, in output units
            prev_measurement: None, // None until the first update
        };
        let mut out = 0.0;
        for _ in 0..10 {
            out = pid.update(10.0, 5.0, 0.01);
            println!("out: {}", out);
        }

        println!("out: {}", out);

        assert!(out == 1.0);
    }
}
