//! MATRIX (AHB Bus Matrix) stub, base 0x4008_8000.
//!
//! The bus matrix arbitrates between bus masters and slaves inside the
//! chip, which an emulator has no need for. It also holds a few chip
//! configuration registers; the one startup code touches most is
//! CCFG_SYSIO (+0x114), which releases pins such as PB4-PB7 and PB12 from
//! their debug / erase function so they can be used as ordinary I/O.
//!
//! Nothing here has any effect: every register remembers what was written.
//!
//! Register offsets are from `ATSAME70Q21.svd`.

use crate::peripheral::Peripheral;

const MATRIX_WPMR: u32 = 0x1E4; // Write Protection Mode Register
const MATRIX_WPSR: u32 = 0x1E8; // Write Protection Status Register (read-only)

// MATRIX_WPMR: bit 0 is WPEN; bits 31-8 are a write key that reads back as 0.
const WPMR_WPEN: u32 = 1 << 0;
const WPMR_WPKEY_SHIFT: u32 = 8;
const WPMR_WPKEY_PASSWD: u32 = 0x4D_4154; // "MAT" in ASCII

/// Size of the register block, from the SVD address block.
const BLOCK_SIZE: u32 = 0x1EC;

pub struct MatrixStub {
    /// One slot per 32-bit register, indexed by offset / 4.
    regs: [u32; (BLOCK_SIZE / 4) as usize],
}

impl MatrixStub {
    pub fn new() -> Self {
        MatrixStub {
            regs: [0; (BLOCK_SIZE / 4) as usize],
        }
    }
}

impl Peripheral for MatrixStub {
    /// All registers are 32 bits wide, so every access is treated as a
    /// whole-word access to the register containing `offset`.
    fn read(&mut self, offset: u32, _size: usize) -> u32 {
        let offset = offset & !3;
        if offset >= BLOCK_SIZE {
            return 0;
        }
        self.regs[(offset / 4) as usize]
    }

    fn write(&mut self, offset: u32, _size: usize, value: u32) {
        let offset = offset & !3;
        if offset >= BLOCK_SIZE {
            return;
        }
        let value = match offset {
            // No violation is ever recorded.
            MATRIX_WPSR => return,
            // Without the right key the write is dropped. WPEN is stored
            // but protects nothing.
            MATRIX_WPMR => {
                if (value >> WPMR_WPKEY_SHIFT) != WPMR_WPKEY_PASSWD {
                    return;
                }
                value & WPMR_WPEN
            }
            _ => value,
        };
        self.regs[(offset / 4) as usize] = value;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const MATRIX_MCFG0: u32 = 0x000;
    const CCFG_SYSIO: u32 = 0x114;

    #[test]
    fn configuration_registers_read_back() {
        let mut matrix = MatrixStub::new();
        matrix.write(MATRIX_MCFG0, 4, 0x0000_0004);
        matrix.write(CCFG_SYSIO, 4, 0x0000_10F0); // PB4-PB7 and PB12 as I/O
        assert_eq!(matrix.read(MATRIX_MCFG0, 4), 0x0000_0004);
        assert_eq!(matrix.read(CCFG_SYSIO, 4), 0x0000_10F0);
    }

    #[test]
    fn write_protection_needs_the_key() {
        let mut matrix = MatrixStub::new();
        matrix.write(MATRIX_WPMR, 4, WPMR_WPEN); // no key
        assert_eq!(matrix.read(MATRIX_WPMR, 4), 0);
        matrix.write(
            MATRIX_WPMR,
            4,
            (WPMR_WPKEY_PASSWD << WPMR_WPKEY_SHIFT) | WPMR_WPEN,
        );
        // The key itself reads back as 0.
        assert_eq!(matrix.read(MATRIX_WPMR, 4), WPMR_WPEN);
    }

    #[test]
    fn write_protection_status_is_read_only() {
        let mut matrix = MatrixStub::new();
        matrix.write(MATRIX_WPSR, 4, 0xFFFF_FFFF);
        assert_eq!(matrix.read(MATRIX_WPSR, 4), 0);
    }

    #[test]
    fn access_past_the_register_block_is_harmless() {
        let mut matrix = MatrixStub::new();
        matrix.write(0x1FC, 4, 0x1234);
        assert_eq!(matrix.read(0x1FC, 4), 0);
    }
}
