//! Quadcopter flight code: estimator, PIDs and mixer.
//!
//! This crate touches no hardware. It takes sensor values and a time step and
//! returns motor commands, so the same code runs in the host simulation and in
//! the firmware. See `drone.md` in the repo root for the plan.
//!
//! `no_std` except under `cargo test`, where the test harness needs `std`.
#![cfg_attr(not(test), no_std)]

pub mod estimator;
pub mod mixer;
pub mod pid;
