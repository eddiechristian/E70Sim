//! SUPC (Supply Controller) stub, base 0x400E_1810.
//!
//! The supply controller looks after the voltage regulator, the brownout
//! detector, the supply monitor, wake-up from backup mode and the choice of
//! 32 kHz slow clock source. None of that exists here: the supply is always
//! good and the chip never enters backup mode. The configuration registers
//! remember what was written.
//!
//! The one thing firmware waits for is the slow clock switch: after it
//! selects the 32 kHz crystal (XTALSEL in SUPC_CR) it polls SUPC_SR until
//! OSCSEL is set. The switch is instant here.
//!
//! Register offsets and bit positions are from `ATSAME70Q21.svd`.

use crate::peripheral::Peripheral;

// Register offsets.
const SUPC_CR: u32 = 0x00; // Control Register (write-only)
const SUPC_SMMR: u32 = 0x04; // Supply Monitor Mode Register
const SUPC_MR: u32 = 0x08; // Mode Register
const SUPC_WUMR: u32 = 0x0C; // Wake-up Mode Register
const SUPC_WUIR: u32 = 0x10; // Wake-up Inputs Register
const SUPC_SR: u32 = 0x14; // Status Register (read-only)

// SUPC_CR and SUPC_MR: bits 31-24 are a write key (0xA5). Without it the
// write is dropped. The key reads back as 0.
const KEY_SHIFT: u32 = 24;
const KEY_PASSWD: u32 = 0xA5;

// SUPC_CR bits.
const CR_VROFF: u32 = 1 << 2; // turn the regulator off (enter backup mode)
const CR_XTALSEL: u32 = 1 << 3; // use the 32 kHz crystal as slow clock

// The bits that exist in each configuration register. The rest read 0.
const SMMR_MASK: u32 = 0xF | (0x7 << 8) | (1 << 12) | (1 << 13);
const MR_MASK: u32 = (1 << 12) | (1 << 13) | (1 << 14) | (1 << 17) | (1 << 20);
const WUMR_MASK: u32 = (0x7 << 1) | (0x7 << 5) | (0x7 << 12) | (0x7 << 16);
const WUIR_MASK: u32 = 0x3FFF | (0x3FFF << 16);

// SUPC_SR bits.
const SR_OSCSEL: u32 = 1 << 7; // slow clock comes from the 32 kHz crystal

pub struct SupcStub {
    smmr: u32,
    mr: u32,
    wumr: u32,
    wuir: u32,
    /// OSCSEL: set once firmware selects the 32 kHz crystal. On the real
    /// chip only a loss of backup power clears it again.
    crystal_selected: bool,
}

impl SupcStub {
    pub fn new() -> Self {
        SupcStub {
            smmr: 0,
            mr: 0,
            wumr: 0,
            wuir: 0,
            crystal_selected: false,
        }
    }
}

impl Peripheral for SupcStub {
    /// All registers are 32 bits wide, so every access is treated as a
    /// whole-word access to the register containing `offset`.
    fn read(&mut self, offset: u32, _size: usize) -> u32 {
        match offset & !3 {
            SUPC_SMMR => self.smmr,
            SUPC_MR => self.mr,
            SUPC_WUMR => self.wumr,
            SUPC_WUIR => self.wuir,
            // Every other status bit reports a brownout, supply monitor or
            // wake-up event, and none of those ever happen.
            SUPC_SR => {
                if self.crystal_selected {
                    SR_OSCSEL
                } else {
                    0
                }
            }
            // SUPC_CR is write-only; nothing else is mapped.
            _ => 0,
        }
    }

    fn write(&mut self, offset: u32, _size: usize, value: u32) {
        let key_ok = (value >> KEY_SHIFT) == KEY_PASSWD;
        match offset & !3 {
            SUPC_CR => {
                if !key_ok {
                    return;
                }
                if value & CR_XTALSEL != 0 {
                    self.crystal_selected = true;
                }
                if value & CR_VROFF != 0 {
                    eprintln!("WARNING: SUPC backup mode (VROFF) is not implemented");
                }
            }
            SUPC_SMMR => self.smmr = value & SMMR_MASK,
            SUPC_MR => {
                if key_ok {
                    self.mr = value & MR_MASK;
                }
            }
            SUPC_WUMR => self.wumr = value & WUMR_MASK,
            SUPC_WUIR => self.wuir = value & WUIR_MASK,
            // SUPC_SR is read-only; nothing else is mapped.
            _ => {}
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const KEY: u32 = KEY_PASSWD << KEY_SHIFT;

    #[test]
    fn slow_clock_starts_on_the_rc_oscillator() {
        let mut supc = SupcStub::new();
        assert_eq!(supc.read(SUPC_SR, 4), 0);
    }

    #[test]
    fn selecting_the_crystal_sets_oscsel_at_once() {
        let mut supc = SupcStub::new();
        supc.write(SUPC_CR, 4, KEY | CR_XTALSEL);
        assert_eq!(supc.read(SUPC_SR, 4), SR_OSCSEL);
        // The status is not cleared by reading it.
        assert_eq!(supc.read(SUPC_SR, 4), SR_OSCSEL);
        // The control register is write-only.
        assert_eq!(supc.read(SUPC_CR, 4), 0);
    }

    #[test]
    fn control_write_needs_the_key() {
        let mut supc = SupcStub::new();
        supc.write(SUPC_CR, 4, CR_XTALSEL); // no key
        assert_eq!(supc.read(SUPC_SR, 4), 0);
    }

    #[test]
    fn mode_register_needs_the_key_and_does_not_store_it() {
        let mut supc = SupcStub::new();
        supc.write(SUPC_MR, 4, 1 << 20); // OSCBYPASS, no key
        assert_eq!(supc.read(SUPC_MR, 4), 0);
        supc.write(SUPC_MR, 4, KEY | (1 << 20));
        assert_eq!(supc.read(SUPC_MR, 4), 1 << 20);
    }

    #[test]
    fn other_configuration_registers_read_back() {
        let mut supc = SupcStub::new();
        supc.write(SUPC_SMMR, 4, 0x0000_110A);
        supc.write(SUPC_WUMR, 4, 0x0000_500E);
        supc.write(SUPC_WUIR, 4, 0x0001_0001);
        assert_eq!(supc.read(SUPC_SMMR, 4), 0x0000_110A);
        assert_eq!(supc.read(SUPC_WUMR, 4), 0x0000_500E);
        assert_eq!(supc.read(SUPC_WUIR, 4), 0x0001_0001);
    }

    #[test]
    fn configuration_registers_keep_only_real_bits() {
        let mut supc = SupcStub::new();
        supc.write(SUPC_SMMR, 4, 0xFFFF_FFFF);
        supc.write(SUPC_WUIR, 4, 0xFFFF_FFFF);
        assert_eq!(supc.read(SUPC_SMMR, 4), SMMR_MASK);
        assert_eq!(supc.read(SUPC_WUIR, 4), WUIR_MASK);
    }

    #[test]
    fn status_ignores_writes() {
        let mut supc = SupcStub::new();
        supc.write(SUPC_SR, 4, 0xFFFF_FFFF);
        assert_eq!(supc.read(SUPC_SR, 4), 0);
    }
}
