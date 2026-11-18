//! A one-axis gyro: the true rate with noise added, handed over late.
//!
//! Explained in `Stage1-d.md` in the repo root.

use std::collections::VecDeque;

use rand::SeedableRng;
use rand::rngs::StdRng;
use rand_distr::{Distribution, Normal};

pub struct Gyro {
    // Parameters
    pub noise_std: f32, // rad/s; standard deviation of the error. 0.0 means no noise

    // State
    rng: StdRng,         // seeded, so two runs with the same seed match
    held: VecDeque<f32>, // readings waiting to be handed over; its length is the delay in steps
}

impl Gyro {
    /// A gyro whose readings are `delay_steps` steps old. The same `seed`
    /// gives the same noise every run.
    pub fn new(noise_std: f32, delay_steps: usize, seed: u64) -> Self {
        Self {
            noise_std,
            rng: StdRng::seed_from_u64(seed),
            // The model starts at rest, so the readings from before the run
            // began are zeros.
            held: VecDeque::from(vec![0.0; delay_steps]),
        }
    }

    /// Takes the true rate in rad/s and returns what the gyro reports.
    ///
    /// 1. Add noise to the true rate: one sample from a normal distribution
    ///    with mean 0 and standard deviation `noise_std` (`rand_distr::Normal`).
    /// 2. Put the noisy reading at the back of the queue.
    /// 3. Return the reading at the front, which is `delay_steps` old.
    pub fn read(&mut self, true_rate: f32) -> f32 {
        // A standard deviation of 0 (or a bad one) means no noise at all, so
        // the reading is then exactly the true rate.
        let noise = match Normal::new(0.0, self.noise_std) {
            Ok(normal) if self.noise_std > 0.0 => normal.sample(&mut self.rng),
            _ => 0.0,
        };

        self.held.push_back(true_rate + noise);
        // Never empty: a reading was pushed on the line above.
        self.held.pop_front().unwrap()
    }

    /// The delay in steps.
    pub fn delay_steps(&self) -> usize {
        self.held.len()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // 10,000 readings of a true rate of 10 with noise 2.
    fn noisy_readings(seed: u64) -> Vec<f32> {
        let mut gyro = Gyro::new(2.0, 0, seed);
        (0..10_000).map(|_| gyro.read(10.0)).collect()
    }

    fn mean(values: &[f32]) -> f32 {
        values.iter().sum::<f32>() / values.len() as f32
    }

    #[test]
    fn test_no_noise_no_delay() {
        let mut gyro = Gyro::new(0.0, 0, 1);
        assert!(gyro.delay_steps() == 0);
        for rate in [0.0, 1.5, -3.25, 100.0] {
            assert!(gyro.read(rate) == rate);
        }
    }

    #[test]
    fn test_delay_only() {
        let mut gyro = Gyro::new(0.0, 3, 1);
        assert!(gyro.delay_steps() == 3);

        let readings: Vec<f32> = [1.0, 2.0, 3.0, 4.0, 5.0]
            .into_iter()
            .map(|rate| gyro.read(rate))
            .collect();
        println!("readings: {:?}", readings);
        assert!(readings == [0.0, 0.0, 0.0, 1.0, 2.0]);

        // The delay stays the same length as readings pass through.
        assert!(gyro.delay_steps() == 3);
    }

    #[test]
    fn test_noise_averages_out() {
        let readings = noisy_readings(1);
        let average = mean(&readings);
        println!("average: {}", average);
        assert!((average - 10.0).abs() < 0.1);
    }

    #[test]
    fn test_noise_has_the_right_size() {
        let readings = noisy_readings(1);
        let average = mean(&readings);
        let squares: Vec<f32> = readings.iter().map(|r| (r - average).powi(2)).collect();
        let std_dev = mean(&squares).sqrt();
        println!("standard deviation: {}", std_dev);
        assert!((std_dev - 2.0).abs() < 0.1);
    }

    #[test]
    fn test_same_seed_same_readings() {
        assert!(noisy_readings(7) == noisy_readings(7));
    }

    #[test]
    fn test_different_seed_different_readings() {
        assert!(noisy_readings(7) != noisy_readings(8));
    }
}
