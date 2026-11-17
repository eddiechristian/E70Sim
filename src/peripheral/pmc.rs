//! PMC (Power Management Controller) stub, base 0x400E_0600.
//!
//! Startup code switches clock sources and then busy-waits on status bits
//! that say the oscillator or PLL has settled. There are no real clocks
//! here, so the status bits always read "ready" and the configuration
//! registers simply remember what was written.
//!
//! Register offsets and bit positions are from `ATSAME70Q21.svd`.

use crate::peripheral::Peripheral;

// Register offsets.
const PMC_SCER: u32 = 0x00; // System Clock Enable (write-only)
const PMC_SCDR: u32 = 0x04; // System Clock Disable (write-only)
const PMC_SCSR: u32 = 0x08; // System Clock Status (read-only)
const PMC_PCER0: u32 = 0x10; // Peripheral Clock Enable 0 (write-only)
const PMC_PCDR0: u32 = 0x14; // Peripheral Clock Disable 0 (write-only)
const PMC_PCSR0: u32 = 0x18; // Peripheral Clock Status 0 (read-only)
const CKGR_MOR: u32 = 0x20; // Main Oscillator Register
const CKGR_MCFR: u32 = 0x24; // Main Clock Frequency Register
const PMC_IER: u32 = 0x60; // Interrupt Enable (write-only)
const PMC_IDR: u32 = 0x64; // Interrupt Disable (write-only)
const PMC_SR: u32 = 0x68; // Status (read-only)
const PMC_IMR: u32 = 0x6C; // Interrupt Mask (read-only)
const PMC_PCER1: u32 = 0x100; // Peripheral Clock Enable 1 (write-only)
const PMC_PCDR1: u32 = 0x104; // Peripheral Clock Disable 1 (write-only)
const PMC_PCSR1: u32 = 0x108; // Peripheral Clock Status 1 (read-only)

/// Size of the register block, from the SVD address block.
const BLOCK_SIZE: u32 = 0x148;

// PMC_SR bits.
const SR_MOSCXTS: u32 = 1 << 0; // main crystal oscillator stabilized
const SR_LOCKA: u32 = 1 << 1; // PLLA locked
const SR_MCKRDY: u32 = 1 << 3; // master clock ready
const SR_LOCKU: u32 = 1 << 6; // UTMI PLL locked
const SR_PCKRDY: u32 = 0x7F << 8; // programmable clocks 0-6 ready
const SR_MOSCSELS: u32 = 1 << 16; // main oscillator selection done
const SR_MOSCRCS: u32 = 1 << 17; // main RC oscillator stabilized

/// Every "ready" bit in PMC_SR. The fault and error bits stay 0.
const SR_ALL_READY: u32 =
    SR_MOSCXTS | SR_LOCKA | SR_MCKRDY | SR_LOCKU | SR_PCKRDY | SR_MOSCSELS | SR_MOSCRCS;

// CKGR_MOR: bits 23-16 are a write key (0x37) that reads back as 0.
const MOR_KEY_MASK: u32 = 0xFF << 16;

// CKGR_MCFR: report a finished measurement of a 12 MHz main clock.
// MAINF counts main clock cycles in 16 slow clock (32.768 kHz) periods.
const MCFR_MAINFRDY: u32 = 1 << 16;
const MCFR_MAINF_12MHZ: u32 = 5859;

pub struct PmcStub {
    /// One slot per 32-bit register, indexed by offset / 4.
    regs: [u32; (BLOCK_SIZE / 4) as usize],
}

impl PmcStub {
    pub fn new() -> Self {
        PmcStub {
            regs: [0; (BLOCK_SIZE / 4) as usize],
        }
    }

    fn get(&self, offset: u32) -> u32 {
        self.regs[(offset / 4) as usize]
    }

    fn set(&mut self, offset: u32, value: u32) {
        self.regs[(offset / 4) as usize] = value;
    }
}

impl Peripheral for PmcStub {
    /// All registers are 32 bits wide, so every access is treated as a
    /// whole-word access to the register containing `offset`.
    fn read(&mut self, offset: u32, _size: usize) -> u32 {
        let offset = offset & !3;
        if offset >= BLOCK_SIZE {
            return 0;
        }
        match offset {
            PMC_SR => SR_ALL_READY,
            CKGR_MCFR => MCFR_MAINFRDY | MCFR_MAINF_12MHZ,
            // Write-only registers read as 0.
            PMC_SCER | PMC_SCDR | PMC_PCER0 | PMC_PCDR0 | PMC_PCER1 | PMC_PCDR1 | PMC_IER
            | PMC_IDR => 0,
            // Everything else, including the status halves of the
            // enable/disable/status triplets, reads back what is stored.
            _ => self.get(offset),
        }
    }

    fn write(&mut self, offset: u32, _size: usize, value: u32) {
        let offset = offset & !3;
        if offset >= BLOCK_SIZE {
            return;
        }
        match offset {
            // Enable / disable pairs set or clear bits in a status register.
            PMC_SCER => self.set(PMC_SCSR, self.get(PMC_SCSR) | value),
            PMC_SCDR => self.set(PMC_SCSR, self.get(PMC_SCSR) & !value),
            PMC_PCER0 => self.set(PMC_PCSR0, self.get(PMC_PCSR0) | value),
            PMC_PCDR0 => self.set(PMC_PCSR0, self.get(PMC_PCSR0) & !value),
            PMC_PCER1 => self.set(PMC_PCSR1, self.get(PMC_PCSR1) | value),
            PMC_PCDR1 => self.set(PMC_PCSR1, self.get(PMC_PCSR1) & !value),
            PMC_IER => self.set(PMC_IMR, self.get(PMC_IMR) | value),
            PMC_IDR => self.set(PMC_IMR, self.get(PMC_IMR) & !value),
            // Read-only registers ignore writes.
            PMC_SCSR | PMC_PCSR0 | PMC_PCSR1 | PMC_SR | PMC_IMR | CKGR_MCFR => {}
            CKGR_MOR => self.set(CKGR_MOR, value & !MOR_KEY_MASK),
            // Store-and-readback for the remaining configuration registers
            // (PLLA, master clock, USB, programmable clocks, ...).
            _ => self.set(offset, value),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const CKGR_PLLAR: u32 = 0x28;
    const PMC_MCKR: u32 = 0x30;

    #[test]
    fn status_reports_every_clock_ready() {
        let mut pmc = PmcStub::new();
        let sr = pmc.read(PMC_SR, 4);
        for bit in [
            SR_MOSCXTS,
            SR_LOCKA,
            SR_MCKRDY,
            SR_LOCKU,
            SR_MOSCSELS,
            SR_MOSCRCS,
        ] {
            assert_ne!(sr & bit, 0, "bit {:#X} should be set", bit);
        }
        // Clock failure / error flags (bits 18-21) stay clear.
        assert_eq!(sr & (0xF << 18), 0);
    }

    #[test]
    fn status_ignores_writes() {
        let mut pmc = PmcStub::new();
        pmc.write(PMC_SR, 4, 0);
        assert_eq!(pmc.read(PMC_SR, 4), SR_ALL_READY);
    }

    #[test]
    fn configuration_registers_read_back() {
        let mut pmc = PmcStub::new();
        pmc.write(CKGR_PLLAR, 4, 0x2018_3F01);
        pmc.write(PMC_MCKR, 4, 0x0000_0102);
        assert_eq!(pmc.read(CKGR_PLLAR, 4), 0x2018_3F01);
        assert_eq!(pmc.read(PMC_MCKR, 4), 0x0000_0102);
    }

    #[test]
    fn main_oscillator_key_is_not_stored() {
        let mut pmc = PmcStub::new();
        pmc.write(CKGR_MOR, 4, 0x0137_FF09); // KEY = 0x37
        assert_eq!(pmc.read(CKGR_MOR, 4), 0x0100_FF09);
    }

    #[test]
    fn main_clock_frequency_measurement_is_ready() {
        let mut pmc = PmcStub::new();
        let mcfr = pmc.read(CKGR_MCFR, 4);
        assert_ne!(mcfr & MCFR_MAINFRDY, 0);
        assert_ne!(mcfr & 0xFFFF, 0);
    }

    #[test]
    fn peripheral_clock_enable_and_disable() {
        let mut pmc = PmcStub::new();
        pmc.write(PMC_PCER0, 4, (1 << 7) | (1 << 10)); // UART0 is ID 7
        assert_eq!(pmc.read(PMC_PCSR0, 4), (1 << 7) | (1 << 10));
        pmc.write(PMC_PCDR0, 4, 1 << 10);
        assert_eq!(pmc.read(PMC_PCSR0, 4), 1 << 7);
        // The enable register itself is write-only.
        assert_eq!(pmc.read(PMC_PCER0, 4), 0);

        pmc.write(PMC_PCER1, 4, 1 << 3);
        assert_eq!(pmc.read(PMC_PCSR1, 4), 1 << 3);
        pmc.write(PMC_PCDR1, 4, 1 << 3);
        assert_eq!(pmc.read(PMC_PCSR1, 4), 0);
    }

    #[test]
    fn system_clock_and_interrupt_mask_triplets() {
        let mut pmc = PmcStub::new();
        pmc.write(PMC_SCER, 4, 0x0000_0700);
        pmc.write(PMC_SCDR, 4, 0x0000_0200);
        assert_eq!(pmc.read(PMC_SCSR, 4), 0x0000_0500);

        pmc.write(PMC_IER, 4, 0b1011);
        pmc.write(PMC_IDR, 4, 0b0010);
        assert_eq!(pmc.read(PMC_IMR, 4), 0b1001);
    }

    #[test]
    fn status_registers_ignore_direct_writes() {
        let mut pmc = PmcStub::new();
        pmc.write(PMC_PCSR0, 4, 0xFFFF_FFFF);
        assert_eq!(pmc.read(PMC_PCSR0, 4), 0);
    }

    #[test]
    fn access_past_the_register_block_is_harmless() {
        let mut pmc = PmcStub::new();
        pmc.write(0x1FC, 4, 0x1234);
        assert_eq!(pmc.read(0x1FC, 4), 0);
    }
}
