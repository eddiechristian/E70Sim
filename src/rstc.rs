//! RSTC (Reset Controller) stub, base 0x400E_1800.
//!
//! Firmware reads RSTC_SR to find out why the chip was reset, sets up the
//! NRST pin behaviour in RSTC_MR, and writes RSTC_CR to reset itself.
//!
//! Here the last reset was always a power-up, the NRST pin is never pulled
//! low, and the mode register remembers what was written. A software reset
//! request is logged but not carried out: a peripheral has no way to reach
//! the CPU through the `Peripheral` trait.
//!
//! Register offsets and bit positions are from `ATSAME70Q21.svd`. The
//! RSTC_MR reset value is from the SAME70 datasheet; the SVD does not
//! record it.

use crate::peripheral::Peripheral;

// Register offsets.
const RSTC_CR: u32 = 0x00; // Control Register (write-only)
const RSTC_SR: u32 = 0x04; // Status Register (read-only)
const RSTC_MR: u32 = 0x08; // Mode Register

// RSTC_CR and RSTC_MR: bits 31-24 are a write key (0xA5). Without it the
// write is dropped. The key reads back as 0.
const KEY_SHIFT: u32 = 24;
const KEY_PASSWD: u32 = 0xA5;

// RSTC_CR bits.
const CR_PROCRST: u32 = 1 << 0; // reset the processor and peripherals

// RSTC_SR: RSTTYP (bits 10-8) stays 0, meaning a power-up reset. URSTS
// (NRST seen low) and SRCMP (software reset in progress) stay 0 too.
const SR_NRSTL: u32 = 1 << 16; // NRST pin level: high, nobody is pressing it

// RSTC_MR: the bits that exist (URSTEN, URSTIEN, ERSTL).
const MR_MASK: u32 = (1 << 0) | (1 << 4) | (0xF << 8);

/// RSTC_MR after reset: URSTEN set, so a low level on NRST resets the chip.
const MR_RESET: u32 = 1 << 0;

pub struct RstcStub {
    mr: u32,
}

impl RstcStub {
    pub fn new() -> Self {
        RstcStub { mr: MR_RESET }
    }
}

impl Peripheral for RstcStub {
    /// All registers are 32 bits wide, so every access is treated as a
    /// whole-word access to the register containing `offset`.
    fn read(&mut self, offset: u32, _size: usize) -> u32 {
        match offset & !3 {
            RSTC_SR => SR_NRSTL,
            RSTC_MR => self.mr,
            // RSTC_CR is write-only; nothing else is mapped.
            _ => 0,
        }
    }

    fn write(&mut self, offset: u32, _size: usize, value: u32) {
        let key_ok = (value >> KEY_SHIFT) == KEY_PASSWD;
        match offset & !3 {
            RSTC_CR => {
                if key_ok && value & CR_PROCRST != 0 {
                    eprintln!("WARNING: RSTC software reset (PROCRST) is not implemented");
                }
                // EXTRST (bit 3) only pulses the NRST pin for external
                // devices, and there are none.
            }
            RSTC_MR => {
                if key_ok {
                    self.mr = value & MR_MASK;
                }
            }
            // RSTC_SR is read-only; nothing else is mapped.
            _ => {}
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const KEY: u32 = KEY_PASSWD << KEY_SHIFT;

    #[test]
    fn status_reports_a_power_up_reset_with_nrst_high() {
        let mut rstc = RstcStub::new();
        let sr = rstc.read(RSTC_SR, 4);
        assert_eq!((sr >> 8) & 0x7, 0, "RSTTYP should be 0 (power-up)");
        assert_ne!(sr & SR_NRSTL, 0);
        assert_eq!(sr & 1, 0, "URSTS should be clear");
    }

    #[test]
    fn status_ignores_writes() {
        let mut rstc = RstcStub::new();
        rstc.write(RSTC_SR, 4, 0);
        assert_eq!(rstc.read(RSTC_SR, 4), SR_NRSTL);
    }

    #[test]
    fn mode_register_has_its_reset_value() {
        let mut rstc = RstcStub::new();
        assert_eq!(rstc.read(RSTC_MR, 4), MR_RESET);
    }

    #[test]
    fn mode_register_needs_the_key_and_does_not_store_it() {
        let mut rstc = RstcStub::new();
        rstc.write(RSTC_MR, 4, 0x0000_0300); // no key
        assert_eq!(rstc.read(RSTC_MR, 4), MR_RESET);
        rstc.write(RSTC_MR, 4, KEY | 0x0000_0300);
        assert_eq!(rstc.read(RSTC_MR, 4), 0x0000_0300);
    }

    #[test]
    fn mode_register_keeps_only_real_bits() {
        let mut rstc = RstcStub::new();
        rstc.write(RSTC_MR, 4, KEY | 0x00FF_FFFF);
        assert_eq!(rstc.read(RSTC_MR, 4), MR_MASK);
    }

    #[test]
    fn reset_request_changes_nothing_visible() {
        let mut rstc = RstcStub::new();
        rstc.write(RSTC_CR, 4, KEY | CR_PROCRST);
        assert_eq!(rstc.read(RSTC_CR, 4), 0);
        assert_eq!(rstc.read(RSTC_SR, 4), SR_NRSTL);
        assert_eq!(rstc.read(RSTC_MR, 4), MR_RESET);
    }
}
