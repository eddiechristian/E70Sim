//! SCB (System Control Block) stub, base 0xE000_ED00.
//!
//! This is the Arm core's own register block rather than a Microchip
//! peripheral. Startup code uses it to move the vector table (VTOR), turn
//! on the FPU (CPACR), turn on the caches (CCR plus the cache maintenance
//! registers) and set exception priorities (SHPR).
//!
//! The exception priorities (SHPR) and the priority grouping (AIRCR) are
//! used by the bus to decide which exception can interrupt which, and
//! VTOR is where the CPU finds the vector table. The rest has no effect
//! here: there are no caches and no FPU. Registers remember what was
//! written, the identification registers return the values of a SAME70's
//! Cortex-M7, and a system reset request, which the emulator
//! cannot carry out, is logged.
//!
//! Two registers in this range are not handled here but by the bus,
//! because they act on the pending and active exceptions, which the bus
//! keeps: ICSR (0xE000_ED04) and STIR (0xE000_EF00).
//!
//! The stub covers 0xE000_ED00 to 0xE000_EFFF, which also takes in the MPU
//! (0xE000_ED90), the debug registers (0xE000_EDF0) and the FPU control
//! registers (0xE000_EF30). Apart from MPU_TYPE those are plain storage.
//!
//! Unlike the Microchip peripherals, these registers are accessed a byte
//! at a time by real code (CMSIS writes exception priorities as single
//! bytes of SHPR), so byte and halfword accesses are supported.
//!
//! Register offsets, CPUID, CLIDR, CTR and MPU_TYPE are from
//! `ATSAME70Q21.svd`; register behaviour is from the ARMv7-M ARM, section
//! B3.2. The cache geometry in CCSIDR and the CCR reset value are from
//! Cortex-M7 documentation that is not in the repo.

use crate::peripheral::Peripheral;

// Register offsets from 0xE000_ED00.
const CPUID: u32 = 0x00; // CPUID Base Register (read-only)
const ICSR: u32 = 0x04; // Interrupt Control and State
const VTOR: u32 = 0x08; // Vector Table Offset
const AIRCR: u32 = 0x0C; // Application Interrupt and Reset Control
const CCR: u32 = 0x14; // Configuration and Control
const SHCSR: u32 = 0x24; // System Handler Control and State
const BFAR: u32 = 0x38; // BusFault Address
const SHPR1: u32 = 0x18; // System Handler Priority 1-3: one byte each
const SHPR3: u32 = 0x20; //   for exceptions 4 to 15
const CFSR: u32 = 0x28; // Configurable Fault Status
const HFSR: u32 = 0x2C; // HardFault Status
const DFSR: u32 = 0x30; // Debug Fault Status
const CLIDR: u32 = 0x78; // Cache Level ID (read-only)
const CTR: u32 = 0x7C; // Cache Type (read-only)
const CCSIDR: u32 = 0x80; // Cache Size ID (read-only)
const CSSELR: u32 = 0x84; // Cache Size Selection
const MPU_TYPE: u32 = 0x90; // MPU Type (read-only)
const STIR: u32 = 0x200; // Software Triggered Interrupt (write-only)
const MVFR0: u32 = 0x240; // Media and VFP Feature 0-2 (read-only)
const MVFR1: u32 = 0x244;
const MVFR2: u32 = 0x248;
const FPCCR: u32 = 0x234; // FP Context Control
const ICIALLU: u32 = 0x250; // first cache maintenance register (write-only)
const BPIALL: u32 = 0x278; // last cache maintenance register (write-only)

/// Size of the area covered: up to the end of the System Control Space.
const BLOCK_SIZE: u32 = 0x300;

// Identification values.
const CPUID_VALUE: u32 = 0x411F_C271; // Arm Cortex-M7 r1p1
const CLIDR_VALUE: u32 = 0x0900_0003; // level 1 instruction and data caches
const CTR_VALUE: u32 = 0x8303_C003; // 32-byte cache lines
const MPU_TYPE_VALUE: u32 = 0x0000_1000; // 16 MPU regions

// What the FPU can do: FPv5 with 16 double-precision registers, single
// and double precision, divide, square root and fused multiply-add.
const MVFR0_VALUE: u32 = 0x1011_0221;
const MVFR1_VALUE: u32 = 0x1200_0011;
const MVFR2_VALUE: u32 = 0x0000_0040;

/// FPCCR after reset: ASPEN and LSPEN set, so FPU use is tracked and the
/// FPU registers are saved lazily on exception entry.
const FPCCR_RESET: u32 = 0xC000_0000;

// CCSIDR for each cache: 16 KB, 32-byte lines. The fields are
// (number of sets - 1) << 13, (ways - 1) << 3 and log2(line bytes) - 4.
// CMSIS reads these to loop over every set and way when it enables or
// flushes a cache.
const CCSIDR_DCACHE: u32 = 0xF000_0000 | (127 << 13) | (3 << 3) | 1; // 4-way
const CCSIDR_ICACHE: u32 = 0xF000_0000 | (255 << 13) | (1 << 3) | 1; // 2-way
const CSSELR_IND: u32 = 1 << 0; // 1 selects the instruction cache

// The SAME70 implements 3 priority bits, the top 3 of each priority
// byte. The other bits always read 0.
const PRIORITY_MASK: u32 = 0xE0E0_E0E0;

// VTOR: bits 6-0 are always 0.
const VTOR_MASK: u32 = 0xFFFF_FF80;

// AIRCR: the top half is a write key (0x05FA) that reads back as 0xFA05.
const AIRCR_VECTKEY: u32 = 0x05FA;
const AIRCR_VECTKEYSTAT: u32 = 0xFA05 << 16;
const AIRCR_PRIGROUP: u32 = 0x7 << 8; // the only field that is stored
const AIRCR_SYSRESETREQ: u32 = 1 << 2;

// CCR: the writable bits (NONBASETHRDENA, USERSETMPEND, UNALIGN_TRP,
// DIV_0_TRP, BFHFNMIGN, and the DC and IC cache enables), plus the two
// bits that always read 1 on a Cortex-M7 (STKALIGN and BP).
const CCR_WRITABLE: u32 = (1 << 0) | (1 << 1) | (1 << 3) | (1 << 4) | (1 << 8) | (3 << 16);
const CCR_ALWAYS_SET: u32 = (1 << 9) | (1 << 18);

pub struct ScbStub {
    /// One slot per 32-bit register, indexed by offset / 4.
    regs: [u32; (BLOCK_SIZE / 4) as usize],
}

impl ScbStub {
    pub fn new() -> Self {
        let mut scb = ScbStub {
            regs: [0; (BLOCK_SIZE / 4) as usize],
        };
        scb.set(CCR, CCR_ALWAYS_SET);
        scb.set(FPCCR, FPCCR_RESET);
        scb
    }

    /// The priority firmware has given system exception `number` (4 to
    /// 15, for example 15 = SysTick): its byte of SHPR1-3.
    pub fn system_priority(&self, number: u32) -> u8 {
        let byte = number - 4;
        (self.get(SHPR1 + (byte & !3)) >> (8 * (byte & 3))) as u8
    }

    /// SHCSR, whose bits 16 to 18 enable MemManage, BusFault and
    /// UsageFault.
    pub fn shcsr(&self) -> u32 {
        self.get(SHCSR)
    }

    /// Record a fault: set `cfsr_bits` in CFSR and `hfsr_bits` in HFSR,
    /// and put the address of a failed data access in BFAR.
    pub fn record_fault(&mut self, cfsr_bits: u32, hfsr_bits: u32, address: Option<u32>) {
        self.set(CFSR, self.get(CFSR) | cfsr_bits);
        self.set(HFSR, self.get(HFSR) | hfsr_bits);
        if let Some(address) = address {
            self.set(BFAR, address);
        }
    }

    /// AIRCR.PRIGROUP: where a priority value splits into group priority
    /// and subpriority.
    pub fn priority_group(&self) -> u32 {
        (self.get(AIRCR) & AIRCR_PRIGROUP) >> 8
    }

    fn get(&self, offset: u32) -> u32 {
        self.regs[(offset / 4) as usize]
    }

    fn set(&mut self, offset: u32, value: u32) {
        self.regs[(offset / 4) as usize] = value;
    }

    /// Read the whole 32-bit register at `offset` (a multiple of 4).
    fn read_word(&self, offset: u32) -> u32 {
        match offset {
            CPUID => CPUID_VALUE,
            // ICSR is about which exceptions are pending and active,
            // which the bus knows and this stub does not. The bus
            // answers ICSR itself, so this is never reached through it.
            ICSR => 0,
            AIRCR => AIRCR_VECTKEYSTAT | self.get(AIRCR),
            // There is no debugger, so no debug event is ever recorded.
            DFSR => 0,
            CLIDR => CLIDR_VALUE,
            CTR => CTR_VALUE,
            CCSIDR => {
                if self.get(CSSELR) & CSSELR_IND != 0 {
                    CCSIDR_ICACHE
                } else {
                    CCSIDR_DCACHE
                }
            }
            MPU_TYPE => MPU_TYPE_VALUE,
            MVFR0 => MVFR0_VALUE,
            MVFR1 => MVFR1_VALUE,
            MVFR2 => MVFR2_VALUE,
            // Write-only registers read as 0.
            STIR | ICIALLU..=BPIALL => 0,
            _ => self.get(offset),
        }
    }

    /// Write to the 32-bit register at `offset` (a multiple of 4). `mask`
    /// has 1s in the bytes being written; `bits` holds the new value of
    /// those bytes, already in position, and 0 elsewhere.
    fn write_word(&mut self, offset: u32, bits: u32, mask: u32) {
        // What the register would hold after a plain store.
        let merged = (self.get(offset) & !mask) | bits;
        match offset {
            CPUID | CLIDR | CTR | CCSIDR | MPU_TYPE | MVFR0..=MVFR2 => {}
            // Handled by the bus, like STIR below.
            ICSR | STIR => {}
            VTOR => self.set(VTOR, merged & VTOR_MASK),
            AIRCR => {
                // Without the key the whole write is dropped, which also
                // rules out byte writes to the low half.
                if (bits >> 16) != AIRCR_VECTKEY {
                    return;
                }
                self.set(AIRCR, bits & AIRCR_PRIGROUP);
                if bits & AIRCR_SYSRESETREQ != 0 {
                    eprintln!("WARNING: system reset request (SYSRESETREQ) is not implemented");
                }
            }
            CCR => self.set(CCR, (merged & CCR_WRITABLE) | CCR_ALWAYS_SET),
            SHPR1..=SHPR3 => self.set(offset, merged & PRIORITY_MASK),
            // Fault status bits stay set until firmware writes 1 to them.
            CFSR | HFSR => self.set(offset, self.get(offset) & !bits),
            DFSR => {}
            // Cache invalidate / clean operations: there are no caches.
            ICIALLU..=BPIALL => {}
            // Store-and-readback for everything else: SCR, SHCSR,
            // MMFAR, BFAR, CSSELR, CPACR, the MPU, debug and FPU registers.
            _ => self.set(offset, merged),
        }
    }
}

impl Peripheral for ScbStub {
    fn read(&mut self, offset: u32, size: usize) -> u32 {
        if offset >= BLOCK_SIZE {
            return 0;
        }
        let shift = (offset & 3) * 8;
        let word = self.read_word(offset & !3) >> shift;
        match size {
            1 => word & 0xFF,
            2 => word & 0xFFFF,
            _ => word,
        }
    }

    fn write(&mut self, offset: u32, size: usize, value: u32) {
        if offset >= BLOCK_SIZE {
            return;
        }
        let shift = (offset & 3) * 8;
        let mask = match size {
            1 => 0xFF,
            2 => 0xFFFF,
            _ => 0xFFFF_FFFF,
        } << shift;
        self.write_word(offset & !3, (value << shift) & mask, mask);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const SHPR3: u32 = 0x20;
    const CPACR: u32 = 0x88;
    const MPU_CTRL: u32 = 0x94;
    const DCISW: u32 = 0x260;

    #[test]
    fn identification_registers_are_fixed() {
        let mut scb = ScbStub::new();
        scb.write(CPUID, 4, 0);
        scb.write(MPU_TYPE, 4, 0);
        assert_eq!(scb.read(CPUID, 4), 0x411F_C271);
        assert_eq!(scb.read(CLIDR, 4), 0x0900_0003);
        assert_eq!(scb.read(CTR, 4), 0x8303_C003);
        assert_eq!(scb.read(MPU_TYPE, 4), 0x0000_1000);
    }

    #[test]
    fn vector_table_offset_reads_back_without_its_low_bits() {
        let mut scb = ScbStub::new();
        scb.write(VTOR, 4, 0x0040_00FF);
        assert_eq!(scb.read(VTOR, 4), 0x0040_0080);
    }

    #[test]
    fn coprocessor_access_reads_back() {
        let mut scb = ScbStub::new();
        scb.write(CPACR, 4, 0xF << 20); // full access to CP10 and CP11 (FPU)
        assert_eq!(scb.read(CPACR, 4), 0xF << 20);
    }

    #[test]
    fn cache_enable_bits_read_back() {
        let mut scb = ScbStub::new();
        assert_eq!(scb.read(CCR, 4), CCR_ALWAYS_SET);
        scb.write(CCR, 4, CCR_ALWAYS_SET | (1 << 17)); // IC
        scb.write(CCR, 4, scb.get(CCR) | (1 << 16)); // DC
        assert_eq!(scb.read(CCR, 4), CCR_ALWAYS_SET | (3 << 16));
        // The always-set bits cannot be cleared.
        scb.write(CCR, 4, 0);
        assert_eq!(scb.read(CCR, 4), CCR_ALWAYS_SET);
    }

    #[test]
    fn cache_size_follows_the_selection_register() {
        let mut scb = ScbStub::new();
        // Data cache: 128 sets, 4 ways, 32-byte lines = 16 KB.
        let d = scb.read(CCSIDR, 4);
        assert_eq!((d >> 13) & 0x7FFF, 127);
        assert_eq!((d >> 3) & 0x3FF, 3);
        assert_eq!(d & 0x7, 1);
        // Instruction cache: 256 sets, 2 ways.
        scb.write(CSSELR, 4, CSSELR_IND);
        let i = scb.read(CCSIDR, 4);
        assert_eq!((i >> 13) & 0x7FFF, 255);
        assert_eq!((i >> 3) & 0x3FF, 1);
    }

    #[test]
    fn cache_maintenance_writes_are_accepted_and_read_zero() {
        let mut scb = ScbStub::new();
        scb.write(ICIALLU, 4, 0);
        scb.write(DCISW, 4, 0xC000_0FE0);
        assert_eq!(scb.read(ICIALLU, 4), 0);
        assert_eq!(scb.read(DCISW, 4), 0);
    }

    #[test]
    fn priority_grouping_needs_the_key() {
        let mut scb = ScbStub::new();
        assert_eq!(scb.read(AIRCR, 4), 0xFA05_0000);
        scb.write(AIRCR, 4, 0x0000_0300); // no key
        assert_eq!(scb.read(AIRCR, 4), 0xFA05_0000);
        scb.write(AIRCR, 4, 0x05FA_0300);
        assert_eq!(scb.read(AIRCR, 4), 0xFA05_0300);
    }

    #[test]
    fn handler_priorities_can_be_written_a_byte_at_a_time() {
        let mut scb = ScbStub::new();
        scb.write(SHPR3 + 3, 1, 0xE0); // SysTick priority
        scb.write(SHPR3 + 2, 1, 0xC0); // PendSV priority
        assert_eq!(scb.read(SHPR3, 4), 0xE0C0_0000);
        assert_eq!(scb.read(SHPR3 + 3, 1), 0xE0);
        assert_eq!(scb.read(SHPR3 + 2, 2), 0xE0C0);
        // A byte write leaves the other bytes alone.
        scb.write(SHPR3 + 3, 1, 0x20);
        assert_eq!(scb.read(SHPR3, 4), 0x20C0_0000);
    }

    #[test]
    fn narrow_reads_of_fixed_registers_pick_the_right_bytes() {
        let mut scb = ScbStub::new();
        assert_eq!(scb.read(CPUID, 1), 0x71);
        assert_eq!(scb.read(CPUID + 2, 2), 0x411F);
    }

    #[test]
    fn recorded_fault_shows_until_firmware_clears_it() {
        let mut scb = ScbStub::new();
        scb.record_fault(1 << 9 | 1 << 15, 1 << 30, Some(0x6000_0000));
        scb.record_fault(1 << 16, 0, None); // a second fault adds its bit
        assert_eq!(scb.read(CFSR, 4), 1 << 9 | 1 << 15 | 1 << 16);
        assert_eq!(scb.read(HFSR, 4), 1 << 30);
        assert_eq!(scb.read(BFAR, 4), 0x6000_0000);
        // Writing 1 clears a bit; writing 0 leaves it.
        scb.write(CFSR, 4, 1 << 9);
        assert_eq!(scb.read(CFSR, 4), 1 << 15 | 1 << 16);
        scb.write(CFSR + 2, 1, 0x01); // byte write: bit 16
        assert_eq!(scb.read(CFSR, 4), 1 << 15);
        scb.write(HFSR, 4, 0xFFFF_FFFF);
        assert_eq!(scb.read(HFSR, 4), 0);
    }

    #[test]
    fn fault_and_exception_state_stays_clear() {
        let mut scb = ScbStub::new();
        scb.write(ICSR, 4, 1 << 28); // PENDSVSET: the bus's business
        scb.write(CFSR, 4, 0xFFFF_FFFF);
        assert_eq!(scb.read(ICSR, 4), 0);
        assert_eq!(scb.read(CFSR, 4), 0);
        assert_eq!(scb.read(HFSR, 4), 0);
    }

    #[test]
    fn mpu_registers_read_back() {
        let mut scb = ScbStub::new();
        scb.write(MPU_CTRL, 4, 0x5);
        assert_eq!(scb.read(MPU_CTRL, 4), 0x5);
    }

    #[test]
    fn access_past_the_block_is_harmless() {
        let mut scb = ScbStub::new();
        scb.write(0x300, 4, 0x1234);
        assert_eq!(scb.read(0x300, 4), 0);
    }
}
