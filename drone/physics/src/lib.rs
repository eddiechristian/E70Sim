//! Physics models for the quadcopter simulation.
//!
//! This crate runs on the host only. It takes motor commands and a time step
//! and returns what the sensors would read. See `drone.md` in the repo root
//! for the plan.
//!
//! Units are SI throughout: radians, rad/s, newtons, metres, seconds. The
//! flight code works in degrees and °/s; the conversion belongs to the caller.

pub mod gyro;
pub mod motor;
pub mod seesaw;
