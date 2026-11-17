//! WDT (Watchdog Timer) stub, base 0x400E_1850.
//!
//! The watchdog is running when the chip leaves reset, so nearly all
//! firmware either disables it straight away (WDDIS in WDT_MR) or restarts
//! it regularly through WDT_CR. Nothing counts down here: the watchdog
//! never expires, never resets the chip and never raises its interrupt.
//!
//! WDT_MR behaves as on the real chip in one respect that firmware can
//! see: it can be written only once after reset. Later writes are ignored.
//!
//! Register offsets and bit positions are from `ATSAME70Q21.svd`. The
//! WDT_MR reset value and the write-once rule are from the SAME70
//! datasheet; the SVD does not record them.

use crate::peripheral::Peripheral;

// Register offsets.
const WDT_CR: u32 = 0x00; // Control Register (write-only)
const WDT_MR: u32 = 0x04; // Mode Register (write-once)
const WDT_SR: u32 = 0x08; // Status Register (read-only)

// WDT_CR fields.
const CR_WDRSTT: u32 = 1 << 0; // restart the watchdog
const CR_KEY_SHIFT: u32 = 24; // must be 0xA5 or the write is dropped
const CR_KEY_PASSWD: u32 = 0xA5;

// WDT_MR: the bits that exist (WDV, WDFIEN, WDRSTEN, WDDIS, WDD,
// WDDBGHLT, WDIDLEHLT). Bits 14, 30 and 31 read 0.
const MR_MASK: u32 = 0x3FFF_BFFF;

/// WDT_MR after reset: watchdog enabled, reset on expiry enabled, counter
/// (WDV) and window (WDD) both at their maximum of 0xFFF.
const MR_RESET: u32 = 0x3FFF_2FFF;

pub struct WdtStub {
    mr: u32,
    /// WDT_MR accepts only the first write after reset.
    mr_written: bool,
}

impl WdtStub {
    pub fn new() -> Self {
        WdtStub {
            mr: MR_RESET,
            mr_written: false,
        }
    }
}

impl Peripheral for WdtStub {
    /// All registers are 32 bits wide, so every access is treated as a
    /// whole-word access to the register containing `offset`.
    fn read(&mut self, offset: u32, _size: usize) -> u32 {
        match offset & !3 {
            WDT_MR => self.mr,
            // WDT_SR: the watchdog never underflows (WDUNF) and is never
            // restarted at the wrong time (WDERR), so both flags stay 0.
            WDT_SR => 0,
            // WDT_CR is write-only; nothing else is mapped.
            _ => 0,
        }
    }

    fn write(&mut self, offset: u32, _size: usize, value: u32) {
        match offset & !3 {
            WDT_CR => {
                // A restart with the right key would reload the counter.
                // There is no counter, so there is nothing to do. A wrong
                // key is worth a warning: on the real chip the restart
                // would be dropped and the watchdog would later expire.
                if value & CR_WDRSTT != 0 && (value >> CR_KEY_SHIFT) != CR_KEY_PASSWD {
                    eprintln!("WARNING: WDT restart ignored, bad key in {:#010X}", value);
                }
            }
            WDT_MR => {
                if self.mr_written {
                    eprintln!(
                        "WARNING: WDT_MR is write-once, ignored second write of {:#010X}",
                        value
                    );
                } else {
                    self.mr = value & MR_MASK;
                    self.mr_written = true;
                }
            }
            // WDT_SR is read-only; nothing else is mapped.
            _ => {}
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const MR_WDDIS: u32 = 1 << 15;

    #[test]
    fn mode_register_has_its_reset_value() {
        let mut wdt = WdtStub::new();
        assert_eq!(wdt.read(WDT_MR, 4), MR_RESET);
        // The watchdog starts out enabled.
        assert_eq!(wdt.read(WDT_MR, 4) & MR_WDDIS, 0);
    }

    #[test]
    fn disable_write_reads_back() {
        let mut wdt = WdtStub::new();
        wdt.write(WDT_MR, 4, MR_WDDIS);
        assert_eq!(wdt.read(WDT_MR, 4), MR_WDDIS);
    }

    #[test]
    fn mode_register_accepts_only_the_first_write() {
        let mut wdt = WdtStub::new();
        wdt.write(WDT_MR, 4, MR_WDDIS);
        wdt.write(WDT_MR, 4, 0x0FFF_2FFF);
        assert_eq!(wdt.read(WDT_MR, 4), MR_WDDIS);
    }

    #[test]
    fn mode_register_keeps_only_real_bits() {
        let mut wdt = WdtStub::new();
        wdt.write(WDT_MR, 4, 0xFFFF_FFFF);
        assert_eq!(wdt.read(WDT_MR, 4), MR_MASK);
    }

    #[test]
    fn status_never_reports_a_fault() {
        let mut wdt = WdtStub::new();
        assert_eq!(wdt.read(WDT_SR, 4), 0);
        wdt.write(WDT_SR, 4, 0xFFFF_FFFF);
        assert_eq!(wdt.read(WDT_SR, 4), 0);
    }

    #[test]
    fn restart_changes_nothing_visible() {
        let mut wdt = WdtStub::new();
        wdt.write(WDT_CR, 4, (CR_KEY_PASSWD << CR_KEY_SHIFT) | CR_WDRSTT);
        wdt.write(WDT_CR, 4, CR_WDRSTT); // no key
        assert_eq!(wdt.read(WDT_CR, 4), 0);
        assert_eq!(wdt.read(WDT_MR, 4), MR_RESET);
        assert_eq!(wdt.read(WDT_SR, 4), 0);
        // A restart does not use up the one write to the mode register.
        wdt.write(WDT_MR, 4, MR_WDDIS);
        assert_eq!(wdt.read(WDT_MR, 4), MR_WDDIS);
    }

    #[test]
    fn unused_offsets_read_zero_and_ignore_writes() {
        let mut wdt = WdtStub::new();
        wdt.write(0x0C, 4, 0x1234);
        assert_eq!(wdt.read(0x0C, 4), 0);
    }
}
