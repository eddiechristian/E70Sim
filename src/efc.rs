//! EFC (Embedded Flash Controller) stub, base 0x400E_0C00.
//!
//! Startup code writes the flash wait-state count to EEFC_FMR before it
//! raises the clock speed, and may poll EEFC_FSR for FRDY. Flash here is a
//! plain byte array with no timing, so the controller is always ready and
//! the mode register simply remembers what was written.
//!
//! Flash commands (erase, write page, lock bits, ...) are not carried out:
//! a write to EEFC_FCR is logged and otherwise ignored.
//!
//! Register offsets and bit positions are from `ATSAME70Q21.svd`.

use crate::peripheral::Peripheral;

// Register offsets.
const EEFC_FMR: u32 = 0x00; // Flash Mode Register
const EEFC_FCR: u32 = 0x04; // Flash Command Register (write-only)
const EEFC_FSR: u32 = 0x08; // Flash Status Register (read-only)
const EEFC_FRR: u32 = 0x0C; // Flash Result Register (read-only)
const EEFC_WPMR: u32 = 0xE4; // Write Protection Mode Register

// EEFC_FMR: the bits that exist (FRDY, FWS, SCOD, CLOE). The rest read 0.
const FMR_MASK: u32 = (1 << 0) | (0xF << 8) | (1 << 16) | (1 << 26);

// EEFC_FCR fields.
const FCR_FCMD_MASK: u32 = 0xFF; // command number
const FCR_FARG_SHIFT: u32 = 8; // 16-bit command argument
const FCR_FARG_MASK: u32 = 0xFFFF;
const FCR_FKEY_SHIFT: u32 = 24; // must be 0x5A or the command is rejected
const FCR_FKEY_PASSWD: u32 = 0x5A;

// EEFC_FSR bits.
const FSR_FRDY: u32 = 1 << 0; // flash ready for a new command
const FSR_FCMDE: u32 = 1 << 1; // bad command or bad key (cleared on read)

// EEFC_WPMR: bit 0 is WPEN; bits 31-8 are a write key that reads back as 0.
const WPMR_WPEN: u32 = 1 << 0;
const WPMR_WPKEY_SHIFT: u32 = 8;
const WPMR_WPKEY_PASSWD: u32 = 0x45_4643; // "EFC" in ASCII

pub struct EfcStub {
    fmr: u32,
    /// FCMDE: set by a command with a bad key, cleared when EEFC_FSR is
    /// read or the next command is written.
    command_error: bool,
    /// WPEN: while set, writes to EEFC_FMR are ignored.
    write_protect: bool,
}

impl EfcStub {
    pub fn new() -> Self {
        EfcStub {
            fmr: 0,
            command_error: false,
            write_protect: false,
        }
    }
}

impl Peripheral for EfcStub {
    /// All registers are 32 bits wide, so every access is treated as a
    /// whole-word access to the register containing `offset`.
    fn read(&mut self, offset: u32, _size: usize) -> u32 {
        match offset & !3 {
            EEFC_FMR => self.fmr,
            EEFC_FSR => {
                let error = if self.command_error { FSR_FCMDE } else { 0 };
                self.command_error = false; // cleared on read
                FSR_FRDY | error
            }
            EEFC_WPMR => {
                if self.write_protect {
                    WPMR_WPEN
                } else {
                    0
                }
            }
            // EEFC_FCR is write-only; EEFC_FRR has no result to report
            // because no command is ever carried out.
            EEFC_FCR | EEFC_FRR => 0,
            _ => 0,
        }
    }

    fn write(&mut self, offset: u32, _size: usize, value: u32) {
        match offset & !3 {
            EEFC_FMR => {
                if !self.write_protect {
                    self.fmr = value & FMR_MASK;
                }
            }
            EEFC_FCR => {
                let command = value & FCR_FCMD_MASK;
                let argument = (value >> FCR_FARG_SHIFT) & FCR_FARG_MASK;
                // Writing a command clears the old error flag; a bad key
                // sets it again.
                self.command_error = (value >> FCR_FKEY_SHIFT) != FCR_FKEY_PASSWD;
                if self.command_error {
                    eprintln!(
                        "WARNING: EFC command {:#04X} rejected, bad key in {:#010X}",
                        command, value
                    );
                } else {
                    eprintln!(
                        "WARNING: EFC command {:#04X} (argument {:#06X}) is not implemented",
                        command, argument
                    );
                }
            }
            EEFC_WPMR => {
                // Without the right key the write is dropped.
                if (value >> WPMR_WPKEY_SHIFT) == WPMR_WPKEY_PASSWD {
                    self.write_protect = value & WPMR_WPEN != 0;
                }
            }
            // EEFC_FSR and EEFC_FRR are read-only; nothing else is mapped.
            _ => {}
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const WPMR_KEY: u32 = WPMR_WPKEY_PASSWD << WPMR_WPKEY_SHIFT;
    const FCR_KEY: u32 = FCR_FKEY_PASSWD << FCR_FKEY_SHIFT;

    #[test]
    fn status_reports_flash_ready() {
        let mut efc = EfcStub::new();
        assert_eq!(efc.read(EEFC_FSR, 4), FSR_FRDY);
    }

    #[test]
    fn status_ignores_writes() {
        let mut efc = EfcStub::new();
        efc.write(EEFC_FSR, 4, 0);
        assert_eq!(efc.read(EEFC_FSR, 4), FSR_FRDY);
    }

    #[test]
    fn mode_register_reads_back() {
        let mut efc = EfcStub::new();
        efc.write(EEFC_FMR, 4, 6 << 8); // FWS = 6, as used at 150 MHz
        assert_eq!(efc.read(EEFC_FMR, 4), 6 << 8);
    }

    #[test]
    fn mode_register_keeps_only_real_bits() {
        let mut efc = EfcStub::new();
        efc.write(EEFC_FMR, 4, 0xFFFF_FFFF);
        assert_eq!(efc.read(EEFC_FMR, 4), FMR_MASK);
    }

    #[test]
    fn command_with_good_key_leaves_flash_ready_without_error() {
        let mut efc = EfcStub::new();
        efc.write(EEFC_FCR, 4, FCR_KEY | 0x0D); // GGPB: get GPNVM bits
        assert_eq!(efc.read(EEFC_FSR, 4), FSR_FRDY);
        // The command register is write-only and nothing was executed.
        assert_eq!(efc.read(EEFC_FCR, 4), 0);
        assert_eq!(efc.read(EEFC_FRR, 4), 0);
    }

    #[test]
    fn command_with_bad_key_sets_error_until_status_is_read() {
        let mut efc = EfcStub::new();
        efc.write(EEFC_FCR, 4, 0x0D); // no key
        assert_eq!(efc.read(EEFC_FSR, 4), FSR_FRDY | FSR_FCMDE);
        assert_eq!(efc.read(EEFC_FSR, 4), FSR_FRDY);
    }

    #[test]
    fn next_command_clears_the_error_flag() {
        let mut efc = EfcStub::new();
        efc.write(EEFC_FCR, 4, 0x0D); // no key
        efc.write(EEFC_FCR, 4, FCR_KEY | 0x0D);
        assert_eq!(efc.read(EEFC_FSR, 4), FSR_FRDY);
    }

    #[test]
    fn write_protection_needs_the_key() {
        let mut efc = EfcStub::new();
        efc.write(EEFC_WPMR, 4, WPMR_WPEN); // no key
        assert_eq!(efc.read(EEFC_WPMR, 4), 0);
        efc.write(EEFC_WPMR, 4, WPMR_KEY | WPMR_WPEN);
        // The key itself reads back as 0.
        assert_eq!(efc.read(EEFC_WPMR, 4), WPMR_WPEN);
    }

    #[test]
    fn write_protection_locks_the_mode_register() {
        let mut efc = EfcStub::new();
        efc.write(EEFC_FMR, 4, 2 << 8);
        efc.write(EEFC_WPMR, 4, WPMR_KEY | WPMR_WPEN);
        efc.write(EEFC_FMR, 4, 6 << 8);
        assert_eq!(efc.read(EEFC_FMR, 4), 2 << 8);

        efc.write(EEFC_WPMR, 4, WPMR_KEY);
        efc.write(EEFC_FMR, 4, 6 << 8);
        assert_eq!(efc.read(EEFC_FMR, 4), 6 << 8);
    }

    #[test]
    fn unused_offsets_read_zero_and_ignore_writes() {
        let mut efc = EfcStub::new();
        efc.write(0x10, 4, 0x1234);
        assert_eq!(efc.read(0x10, 4), 0);
        assert_eq!(efc.read(0x1FC, 4), 0);
    }
}
