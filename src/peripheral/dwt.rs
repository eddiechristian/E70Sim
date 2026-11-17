//! DWT (Data Watchpoint and Trace unit), base 0xE000_1000.
//!
//! Debugging hardware in the Arm core. The one part firmware uses on
//! its own is the cycle counter, DWT_CYCCNT: a 32-bit count of clock
//! cycles, which makes a stopwatch for measuring how long code takes.
//!
//! Firmware switches it on in two places: TRCENA (bit 24) in DEMCR,
//! which is in the SCB's address range, and CYCCNTENA (bit 0) in
//! DWT_CTRL. It counts only while both are set.
//!
//! The count is of the emulator's estimated cycles (see the `CYCLES_`
//! constants in `cpu.rs`), so measurements are repeatable but are not
//! what the real chip would report. A read sees the count as it was
//! when the reading instruction started.
//!
//! The watchpoints and the other counters are not modelled: their
//! registers read 0. Register layout is from the ARMv7-M ARM, section
//! C1.8; the number of comparators is from Cortex-M7 documentation that
//! is not in the repo.

use crate::peripheral::Peripheral;

const DWT_CTRL: u32 = 0x000; // Control
const DWT_CYCCNT: u32 = 0x004; // Cycle Count

/// DWT_CTRL bit 0: the cycle counter runs.
const CTRL_CYCCNTENA: u32 = 1 << 0;

/// DWT_CTRL bits 31-28 (NUMCOMP): four comparators exist. Read-only.
const CTRL_NUMCOMP: u32 = 4 << 28;

pub struct Dwt {
    /// The writable bits of DWT_CTRL.
    control: u32,
    cycle_count: u32,
}

impl Dwt {
    pub fn new() -> Self {
        Dwt {
            control: 0,
            cycle_count: 0,
        }
    }

    /// Let `cycles` clock cycles pass. `trace_enabled` is TRCENA from
    /// DEMCR.
    pub fn tick(&mut self, cycles: u32, trace_enabled: bool) {
        if trace_enabled && self.control & CTRL_CYCCNTENA != 0 {
            // Wraps round to 0 after 2^32 cycles, about 6 minutes at
            // 12 MHz.
            self.cycle_count = self.cycle_count.wrapping_add(cycles);
        }
    }
}

impl Peripheral for Dwt {
    fn read(&mut self, offset: u32, _size: usize) -> u32 {
        match offset & !3 {
            DWT_CTRL => self.control | CTRL_NUMCOMP,
            DWT_CYCCNT => self.cycle_count,
            _ => 0,
        }
    }

    fn write(&mut self, offset: u32, _size: usize, value: u32) {
        match offset & !3 {
            DWT_CTRL => self.control = value & !CTRL_NUMCOMP,
            // Firmware can set the count, usually to 0 before a
            // measurement.
            DWT_CYCCNT => self.cycle_count = value,
            // Everything else, including the lock register some code
            // writes first, is accepted and ignored.
            _ => {}
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn counter_is_off_after_reset() {
        let mut dwt = Dwt::new();
        dwt.tick(100, true);
        assert_eq!(dwt.read(DWT_CYCCNT, 4), 0);
        assert_eq!(dwt.read(DWT_CTRL, 4), CTRL_NUMCOMP);
    }

    #[test]
    fn counter_needs_both_enables() {
        let mut dwt = Dwt::new();
        dwt.write(DWT_CTRL, 4, CTRL_CYCCNTENA);
        dwt.tick(100, false); // TRCENA clear
        assert_eq!(dwt.read(DWT_CYCCNT, 4), 0);
        dwt.tick(100, true);
        dwt.tick(23, true);
        assert_eq!(dwt.read(DWT_CYCCNT, 4), 123);
        dwt.write(DWT_CTRL, 4, 0);
        dwt.tick(100, true);
        assert_eq!(dwt.read(DWT_CYCCNT, 4), 123);
    }

    #[test]
    fn counter_can_be_set_and_wraps_round() {
        let mut dwt = Dwt::new();
        dwt.write(DWT_CTRL, 4, CTRL_CYCCNTENA);
        dwt.write(DWT_CYCCNT, 4, 0xFFFF_FFF0);
        dwt.tick(0x20, true);
        assert_eq!(dwt.read(DWT_CYCCNT, 4), 0x10);
    }

    #[test]
    fn other_registers_read_zero_and_ignore_writes() {
        let mut dwt = Dwt::new();
        dwt.write(0xFB0, 4, 0xC5AC_CE55); // the lock register
        assert_eq!(dwt.read(0xFB0, 4), 0);
        assert_eq!(dwt.read(0x020, 4), 0); // first comparator
    }
}
