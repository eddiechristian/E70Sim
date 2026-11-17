//! SysTick (the Arm core's system timer), base 0xE000_E010.
//!
//! A 24-bit counter that counts down to zero, reloads, and can raise
//! exception 15 each time it gets there. Firmware uses it for a regular
//! "tick", typically every millisecond.
//!
//! The timer counts emulated clock cycles: after every instruction the
//! CPU reports how many cycles it took (an estimate, see the `CYCLES_`
//! constants in `cpu.rs`) and `tick` moves the counter by that much. The
//! CLKSOURCE bit is stored but makes no difference.
//!
//! Register offsets, bit positions and reset values are from
//! `ATSAME70Q21.svd`; the counting behaviour is from the ARMv7-M ARM,
//! section B3.3.

use crate::peripheral::Peripheral;

// Register offsets.
const SYST_CSR: u32 = 0x00; // Control and Status
const SYST_RVR: u32 = 0x04; // Reload Value
const SYST_CVR: u32 = 0x08; // Current Value
const SYST_CALIB: u32 = 0x0C; // Calibration Value (read-only)

// SYST_CSR bits.
const CSR_ENABLE: u32 = 1 << 0; // counter runs
const CSR_TICKINT: u32 = 1 << 1; // reaching zero raises the SysTick exception
const CSR_CLKSOURCE: u32 = 1 << 2; // 1 = processor clock, 0 = external reference
const CSR_COUNTFLAG: u32 = 1 << 16; // reached zero since this register was last read

/// The bits of SYST_CSR that firmware can write.
const CSR_WRITABLE: u32 = CSR_ENABLE | CSR_TICKINT | CSR_CLKSOURCE;

/// SYST_CSR after reset: stopped, clocked from the processor clock.
const CSR_RESET: u32 = CSR_CLKSOURCE;

/// The counter and the reload value are 24 bits wide.
const VALUE_MASK: u32 = 0x00FF_FFFF;

pub struct SysTick {
    /// SYST_CSR bits ENABLE, TICKINT and CLKSOURCE.
    control: u32,
    /// COUNTFLAG: set when the counter reaches zero, cleared when SYST_CSR
    /// is read or SYST_CVR is written.
    count_flag: bool,
    reload: u32,
    current: u32,
}

impl SysTick {
    pub fn new() -> Self {
        SysTick {
            control: CSR_RESET,
            count_flag: false,
            reload: 0,
            current: 0,
        }
    }

    /// Advance the timer by `cycles` clock cycles. Returns true when the
    /// SysTick exception should be made pending.
    ///
    /// The counter goes N, N-1, ..., 1, 0, N, ... where N is the reload
    /// value, so it reaches zero once every N + 1 cycles. A reload value
    /// of 0 leaves it sitting at zero, which never counts as "reaching"
    /// zero: the timer is effectively off.
    pub fn tick(&mut self, cycles: u32) -> bool {
        if self.control & CSR_ENABLE == 0 {
            return false;
        }
        let mut remaining = cycles;
        let mut fire = false;
        // One instruction can take the counter through zero, and with a
        // very short period even more than once.
        while remaining > 0 {
            if self.current == 0 {
                if self.reload == 0 {
                    break;
                }
                // Reloading takes one cycle.
                self.current = self.reload;
                remaining -= 1;
                continue;
            }
            let step = remaining.min(self.current);
            self.current -= step;
            remaining -= step;
            if self.current == 0 {
                self.count_flag = true;
                fire |= self.control & CSR_TICKINT != 0;
            }
        }
        fire
    }
}

impl Peripheral for SysTick {
    /// All registers are 32 bits wide, so every access is treated as a
    /// whole-word access to the register containing `offset`.
    fn read(&mut self, offset: u32, _size: usize) -> u32 {
        match offset & !3 {
            SYST_CSR => {
                let flag = if self.count_flag { CSR_COUNTFLAG } else { 0 };
                self.count_flag = false; // cleared on read
                self.control | flag
            }
            SYST_RVR => self.reload,
            SYST_CVR => self.current,
            // 0 means no calibration value is known.
            SYST_CALIB => 0,
            _ => 0,
        }
    }

    fn write(&mut self, offset: u32, _size: usize, value: u32) {
        match offset & !3 {
            SYST_CSR => self.control = value & CSR_WRITABLE,
            SYST_RVR => self.reload = value & VALUE_MASK,
            // Writing any value clears the counter and COUNTFLAG. The
            // counter then reloads on the next count, without raising the
            // exception.
            SYST_CVR => {
                self.current = 0;
                self.count_flag = false;
            }
            // SYST_CALIB is read-only; nothing else is mapped.
            _ => {}
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A timer set up the way firmware does it: reload value, clear the
    /// counter, then enable.
    fn started(reload: u32, control: u32) -> SysTick {
        let mut systick = SysTick::new();
        systick.write(SYST_RVR, 4, reload);
        systick.write(SYST_CVR, 4, 0);
        systick.write(SYST_CSR, 4, control);
        systick
    }

    #[test]
    fn reset_state_is_stopped_on_the_processor_clock() {
        let mut systick = SysTick::new();
        assert_eq!(systick.read(SYST_CSR, 4), CSR_CLKSOURCE);
        assert_eq!(systick.read(SYST_RVR, 4), 0);
        assert_eq!(systick.read(SYST_CVR, 4), 0);
        assert_eq!(systick.read(SYST_CALIB, 4), 0);
    }

    #[test]
    fn disabled_timer_does_not_count() {
        let mut systick = started(3, CSR_TICKINT); // ENABLE not set
        for _ in 0..10 {
            assert!(!systick.tick(1));
        }
        assert_eq!(systick.read(SYST_CVR, 4), 0);
    }

    #[test]
    fn counts_down_from_the_reload_value() {
        let mut systick = started(3, CSR_ENABLE);
        let mut seen = Vec::new();
        for _ in 0..9 {
            systick.tick(1);
            seen.push(systick.read(SYST_CVR, 4));
        }
        // First count loads the reload value, then 3, 2, 1, 0 repeats.
        assert_eq!(seen, [3, 2, 1, 0, 3, 2, 1, 0, 3]);
    }

    #[test]
    fn raises_the_exception_every_reload_plus_one_counts() {
        let mut systick = started(3, CSR_ENABLE | CSR_TICKINT);
        let fired: Vec<bool> = (0..12).map(|_| systick.tick(1)).collect();
        // Zero is reached on counts 4, 8 and 12.
        let expected = [
            false, false, false, true, false, false, false, true, false, false, false, true,
        ];
        assert_eq!(fired, expected);
    }

    #[test]
    fn several_cycles_at_once_count_the_same_as_one_at_a_time() {
        let mut together = started(9, CSR_ENABLE | CSR_TICKINT);
        let mut singly = started(9, CSR_ENABLE | CSR_TICKINT);
        // Steps of mixed size, as instructions of different cost produce.
        for cycles in [1, 3, 2, 12, 1, 11, 3, 3, 2, 7] {
            let fired_together = together.tick(cycles);
            let mut fired_singly = false;
            for _ in 0..cycles {
                fired_singly |= singly.tick(1);
            }
            assert_eq!(fired_together, fired_singly);
            assert_eq!(together.read(SYST_CVR, 4), singly.read(SYST_CVR, 4));
        }
    }

    #[test]
    fn a_step_that_passes_zero_carries_on_counting() {
        let mut systick = started(9, CSR_ENABLE | CSR_TICKINT); // period 10
        assert!(!systick.tick(8)); // load 9, then down to 2
        assert_eq!(systick.read(SYST_CVR, 4), 2);
        // 5 cycles: 2 to reach zero, 1 to reload, 2 more.
        assert!(systick.tick(5));
        assert_eq!(systick.read(SYST_CVR, 4), 7);
    }

    #[test]
    fn a_step_longer_than_the_period_fires_once() {
        let mut systick = started(2, CSR_ENABLE | CSR_TICKINT); // period 3
        assert!(systick.tick(12)); // four periods in one step
        assert_eq!(systick.read(SYST_CVR, 4), 0);
        assert_ne!(systick.read(SYST_CSR, 4) & CSR_COUNTFLAG, 0);
    }

    #[test]
    fn without_tickint_only_the_flag_is_set() {
        let mut systick = started(1, CSR_ENABLE);
        assert!(!systick.tick(1)); // load 1
        assert!(!systick.tick(1)); // 1 -> 0, but TICKINT is off
        assert_eq!(systick.read(SYST_CSR, 4), CSR_ENABLE | CSR_COUNTFLAG);
    }

    #[test]
    fn count_flag_is_cleared_by_reading_the_control_register() {
        let mut systick = started(1, CSR_ENABLE);
        systick.tick(1);
        systick.tick(1);
        assert_ne!(systick.read(SYST_CSR, 4) & CSR_COUNTFLAG, 0);
        assert_eq!(systick.read(SYST_CSR, 4) & CSR_COUNTFLAG, 0);
    }

    #[test]
    fn writing_the_current_value_clears_it_and_the_flag() {
        let mut systick = started(5, CSR_ENABLE);
        systick.tick(1); // load 5
        systick.tick(1); // 4
        systick.write(SYST_CVR, 4, 0x1234); // the value written is ignored
        assert_eq!(systick.read(SYST_CVR, 4), 0);

        let mut systick = started(1, CSR_ENABLE);
        systick.tick(1);
        systick.tick(1); // sets COUNTFLAG
        systick.write(SYST_CVR, 4, 0);
        assert_eq!(systick.read(SYST_CSR, 4) & CSR_COUNTFLAG, 0);
    }

    #[test]
    fn reload_of_zero_never_fires() {
        let mut systick = started(0, CSR_ENABLE | CSR_TICKINT);
        for _ in 0..10 {
            assert!(!systick.tick(1));
        }
        assert_eq!(systick.read(SYST_CSR, 4) & CSR_COUNTFLAG, 0);
    }

    #[test]
    fn values_are_24_bits_wide() {
        let mut systick = SysTick::new();
        systick.write(SYST_RVR, 4, 0xFFFF_FFFF);
        assert_eq!(systick.read(SYST_RVR, 4), 0x00FF_FFFF);
    }

    #[test]
    fn control_register_keeps_only_real_bits_and_calib_is_read_only() {
        let mut systick = SysTick::new();
        systick.write(SYST_CSR, 4, 0xFFFF_FFFF);
        assert_eq!(systick.read(SYST_CSR, 4), CSR_WRITABLE);
        systick.write(SYST_CALIB, 4, 0xFFFF_FFFF);
        assert_eq!(systick.read(SYST_CALIB, 4), 0);
    }
}
