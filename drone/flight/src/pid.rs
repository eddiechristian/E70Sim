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
    pub d_filter: f32, // s; time constant of the D-term low-pass filter. 0.0 means no filter

    // State
    i_term: f32,                   // Ki × integral, in output units
    prev_measurement: Option<f32>, // None until the first update
    d_filtered: f32,               // filtered rate of change of the measurement
}

impl Pid {
    /// A PID with the given gains, no effective limits and no D-term filter.
    /// Set `out_min`, `out_max`, `i_limit` and `d_filter` afterwards; they are
    /// public fields.
    pub fn new(kp: f32, ki: f32, kd: f32) -> Self {
        Self {
            kp,
            ki,
            kd,
            out_min: f32::MIN,
            out_max: f32::MAX,
            i_limit: f32::MAX,
            d_filter: 0.0,
            i_term: 0.0,
            prev_measurement: None,
            d_filtered: 0.0,
        }
    }

    pub fn update(&mut self, setpoint: f32, measurement: f32, dt: f32) -> f32 {
        let error = setpoint - measurement;
        let p = self.kp * error;

        // A bad dt is treated as "no time passed". Also catches NaN.
        if !(dt > 0.0) {
            debug_assert!(false, "Pid::update called with dt = {dt}");
            return (p + self.i_term).clamp(self.out_min, self.out_max);
        }

        // Derivative on the measurement; zero on the first call.
        let raw = match self.prev_measurement {
            Some(prev) => (measurement - prev) / dt,
            None => 0.0,
        };
        self.prev_measurement = Some(measurement);

        // Low-pass filter on the derivative, to keep sensor noise out of the
        // D term. The fraction is capped at 1 so that a filter shorter than
        // dt passes the raw value through instead of overshooting it.
        if self.d_filter > 0.0 {
            let fraction = (dt / self.d_filter).min(1.0);
            self.d_filtered += (raw - self.d_filtered) * fraction;
        } else {
            self.d_filtered = raw;
        }
        let d = -self.kd * self.d_filtered;

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
        self.d_filtered = 0.0;
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
            d_filter: 0.0,

            // State
            i_term: 0.0,            // Ki × integral, in output units
            prev_measurement: None, // None until the first update
            d_filtered: 0.0,
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
            d_filter: 0.0,

            // State
            i_term: 0.0,            // Ki × integral, in output units
            prev_measurement: None, // None until the first update
            d_filtered: 0.0,
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
            d_filter: 0.0,

            // State
            i_term: 0.0,            // Ki × integral, in output units
            prev_measurement: None, // None until the first update
            d_filtered: 0.0,
        };
        let mut out = 0.0;
        for _ in 0..10 {
            out = pid.update(10.0, 5.0, 0.01);
            println!("out: {}", out);
        }

        println!("out: {}", out);

        assert!(out == 1.0);
    }

    #[test]
    fn test_d_zero() {
        let mut pid = Pid {
            kp: 0.0,
            ki: 0.0,
            kd: 0.01,
            out_min: -20.0,
            out_max: 20.0,
            i_limit: 100.0,
            d_filter: 0.0,

            // State
            i_term: 0.0,            // Ki × integral, in output units
            prev_measurement: None, // None until the first update
            d_filtered: 0.0,
        };
        let out = pid.update(10.0, 7.0, 0.001);
        println!("out: {}", out);

        assert!(out == 0.0);
    }

    #[test]
    fn test_d_opposes() {
        let mut pid = Pid {
            kp: 0.0,
            ki: 0.0,
            kd: 0.01,
            out_min: -20.0,
            out_max: 20.0,
            i_limit: 100.0,
            d_filter: 0.0,

            // State
            i_term: 0.0,            // Ki × integral, in output units
            prev_measurement: None, // None until the first update
            d_filtered: 0.0,
        };
        let mut out = pid.update(0.0, 0.0, 0.001);
        out = pid.update(0.0, 0.5, 0.001);
        println!("out: {}", out);

        assert!((out - -5.0).abs() < 0.001);
    }

    #[test]
    fn test_no_derivative_kick() {
        let mut pid = Pid {
            kp: 0.0,
            ki: 0.0,
            kd: 0.01,
            out_min: -20.0,
            out_max: 20.0,
            i_limit: 100.0,
            d_filter: 0.0,

            // State
            i_term: 0.0,            // Ki × integral, in output units
            prev_measurement: None, // None until the first update
            d_filtered: 0.0,
        };
        let out1 = pid.update(0.0, 0.0, 0.001);
        println!("out1: {}", out1);
         assert!(out1 ==0.0);
        let out2 = pid.update(100.0, 0.0, 0.001);
        println!("out2: {}", out2);
        assert!(out2 ==0.0);
    }

    #[test]
    fn test_output_clamp() {
        let mut pid = Pid {
            kp: 1.0,
            ki: 0.0,
            kd: 0.0,
            out_min: -10.0,
            out_max: 10.0,
            i_limit: 100.0,
            d_filter: 0.0,

            // State
            i_term: 0.0,            // Ki × integral, in output units
            prev_measurement: None, // None until the first update
            d_filtered: 0.0,
        };
        let out1 = pid.update(0.0, -1000.0, 0.001);
        println!("out1: {}", out1);
        assert!(out1 ==10.0);

        
        let out2 = pid.update(0.0, 1000.0, 0.001);
        println!("out2: {}", out2);
        assert!(out2 ==-10.0);
    }

    #[test]
    fn test_i_term_clamp() {
        let mut pid = Pid {
            kp: 0.0,
            ki: 1.0,
            kd: 0.0,
            out_min: -1.0,
            out_max: 1.0,
            i_limit: 0.3,
            d_filter: 0.0,

            // State
            i_term: 0.0,            // Ki × integral, in output units
            prev_measurement: None, // None until the first update
            d_filtered: 0.0,
        };
        let mut out = 0.0;
        for _ in 0..1000 {
            out = pid.update(1.0, 0.0, 0.01);
        }
        println!("out: {}", out);

        // Without i_limit the I term would have reached 10.
        assert!(out == 0.3);
    }

    #[test]
    fn test_no_windup_while_saturated() {
        let mut pid = Pid {
            kp: 1.0,
            ki: 1.0,
            kd: 0.0,
            out_min: -1.0,
            out_max: 1.0,
            i_limit: 0.5,
            d_filter: 0.0,

            // State
            i_term: 0.0,            // Ki × integral, in output units
            prev_measurement: None, // None until the first update
            d_filtered: 0.0,
        };
        // P alone (5) is past the limit, so the I term must not grow.
        for _ in 0..1000 {
            let out = pid.update(5.0, 0.0, 0.01);
            assert!(out == 1.0);
        }

        // With error 0, P is 0 and the output is the I term alone.
        // Without anti-windup it would be 0.5.
        let out = pid.update(0.0, 0.0, 0.01);
        println!("out: {}", out);
        assert!(out == 0.0);
    }

    #[test]
    fn test_integrates_out_of_saturation() {
        let mut pid = Pid {
            kp: 0.0,
            ki: 1.0,
            kd: 0.0,
            out_min: -0.2,
            out_max: 0.2,
            i_limit: 0.5,
            d_filter: 0.0,

            // State
            i_term: 0.0,            // Ki × integral, in output units
            prev_measurement: None, // None until the first update
            d_filtered: 0.0,
        };
        // The I term climbs until the output reaches 0.2, then stops.
        let mut out = 0.0;
        for _ in 0..100 {
            out = pid.update(1.0, 0.0, 0.01);
        }
        println!("out after 100 steps: {}", out);
        assert!(out == 0.2);

        // A negative error leads out of saturation, so the I term must
        // shrink at once.
        let out = pid.update(-1.0, 0.0, 0.01);
        println!("out after one step back: {}", out);
        assert!(out < 0.2);
    }

     #[test]
    fn test_plant_test() {
        let mut pid = Pid {
            kp: 2.0,
            ki: 0.0,
            kd: 0.0,
            out_min: -20.0,
            out_max: 20.0,
            i_limit: 100.0,
            d_filter: 0.0,

            // State
            i_term: 0.0,            // Ki × integral, in output units
            prev_measurement: None, // None until the first update
            d_filtered: 0.0,
        };

        let dt = 0.001;
        let disturbance = -1.0;
        let mut rate = 0.0;
        for _ in 0..20_000 {
            let output = pid.update(0.0, rate, dt);
            rate += (output + disturbance - rate) * dt;
        }
        println!("rate: {}",rate);
        assert!((rate - -0.333).abs() < 0.01);               // PI: Kp = 2, Ki = 5, Kd = 0
    }

    #[test]
    fn test_d_filter_smooths_a_jump() {
        // Kd = 0.01 and a jump of 0.5 in one step of 0.001: unfiltered, the
        // D term is -5 (see test_d_opposes). With a filter of 0.005 s, one
        // step lets through dt / d_filter = 1/5 of that.
        let mut pid = Pid::new(0.0, 0.0, 0.01);
        pid.d_filter = 0.005;
        pid.update(0.0, 0.0, 0.001);
        let out = pid.update(0.0, 0.5, 0.001);
        println!("out: {}", out);
        assert!((out - -1.0).abs() < 0.001);
    }

    #[test]
    fn test_d_filter_reaches_a_steady_slope() {
        // The measurement rises by 0.5 every step. Unfiltered, the D term is
        // -5 from the second step on. Filtered, it gets there gradually.
        let mut pid = Pid::new(0.0, 0.0, 0.01);
        pid.d_filter = 0.005;
        let mut out = 0.0;
        let mut measurement = 0.0;
        for step in 0..100 {
            out = pid.update(0.0, measurement, 0.001);
            if step == 5 {
                // One time constant after the slope began: about 63%.
                println!("out after one time constant: {}", out);
                assert!(out < -2.5 && out > -4.0);
            }
            measurement += 0.5;
        }
        println!("out after 100 steps: {}", out);
        assert!((out - -5.0).abs() < 0.001);
    }

    #[test]
    fn test_d_filter_shorter_than_dt_is_no_filter() {
        // A filter shorter than dt must pass the raw value, not overshoot it.
        let mut pid = Pid::new(0.0, 0.0, 0.01);
        pid.d_filter = 0.0001;
        pid.update(0.0, 0.0, 0.001);
        let out = pid.update(0.0, 0.5, 0.001);
        println!("out: {}", out);
        assert!((out - -5.0).abs() < 0.001);
    }

    #[test]
    fn test_reset_clears_the_d_filter() {
        let mut pid = Pid::new(0.0, 0.0, 0.01);
        pid.d_filter = 0.005;
        pid.update(0.0, 0.0, 0.001);
        pid.update(0.0, 0.5, 0.001);

        // Without the reset, the filter would still hold part of the jump
        // and this output would not be zero.
        pid.reset();
        let out = pid.update(0.0, 0.5, 0.001);
        println!("out: {}", out);
        assert!(out == 0.0);
    }
}
