//! System instructions: the ones that read and change the processor's
//! special-purpose registers. Section numbers (B5.2.x) refer to the
//! ARMv7-M Architecture Reference Manual.
//!
//! - `CPSID` / `CPSIE` set and clear PRIMASK and FAULTMASK. `cpsid i` is
//!   "interrupts off" and `cpsie i` is "interrupts on".
//! - `MRS` copies a special-purpose register into an ordinary one.
//! - `MSR` copies an ordinary register into a special-purpose one.
//!
//! The special-purpose registers are the three exception masks (PRIMASK,
//! FAULTMASK, BASEPRI), CONTROL, the two stack pointers by name (MSP and
//! PSP, whichever is in use), and the xPSR.
//!
//! Unprivileged code cannot use most of this: its writes are ignored and
//! its reads return 0. Code is unprivileged only in Thread mode with
//! CONTROL bit 0 set, which an operating system does for its tasks.

use super::exception::CONTROL_SPSEL;
use super::{CYCLES_DATA, Cpu, SP};
use crate::bus::Bus;

// The SYSm field of MRS and MSR: which special-purpose register (B5.1.1).
// Values 0 to 7 are combinations of the three parts of the xPSR: bit 0
// includes IPSR, bit 1 EPSR, and APSR is included unless bit 2 is set.
const SYSM_XPSR_LAST: u32 = 7;
const SYSM_MSP: u32 = 8;
const SYSM_PSP: u32 = 9;
const SYSM_PRIMASK: u32 = 16;
const SYSM_BASEPRI: u32 = 17;
const SYSM_BASEPRI_MAX: u32 = 18; // writes can only raise the masking
const SYSM_FAULTMASK: u32 = 19;
const SYSM_CONTROL: u32 = 20;

const XPSR_IPSR: u32 = 0x1FF; // bits 8-0: exception number
const XPSR_NZCVQ: u32 = 0x1F << 27; // the flags, bits 31-27
const XPSR_GE: u32 = 0xF << 16; // the DSP extension's GE flags, bits 19-16

/// CONTROL bit 0 (nPRIV): 1 = Thread mode is unprivileged.
const CONTROL_NPRIV: u8 = 1 << 0;

/// The SAME70 implements 3 priority bits, so only the top 3 bits of
/// BASEPRI exist.
const BASEPRI_MASK: u32 = 0xE0;

/// MRS and MSR take two cycles.
const CYCLES_SPECIAL_REGISTER: u32 = 2;

impl Cpu {
    /// Handlers are always privileged; Thread mode is unless CONTROL says
    /// otherwise.
    fn is_privileged(&self) -> bool {
        self.in_handler_mode() || self.control & CONTROL_NPRIV == 0
    }

    /// The main stack pointer, whether or not it is the one in use.
    pub(super) fn main_sp(&self) -> u32 {
        if self.control & CONTROL_SPSEL != 0 {
            self.msp
        } else {
            self.regs[SP]
        }
    }

    /// The process stack pointer, whether or not it is the one in use.
    pub(super) fn process_sp(&self) -> u32 {
        if self.control & CONTROL_SPSEL != 0 {
            self.regs[SP]
        } else {
            self.psp
        }
    }

    /// CPSIE / CPSID <iflags> -- clear (IE, "interrupt enable") or set
    /// (ID, "interrupt disable") PRIMASK, FAULTMASK or both.
    /// Encoding T1: 10110110011 im 0 0 I F  (B5.2.1)
    pub(super) fn cps(&mut self, instr: u16, pc: u32, bus: &Bus) -> u32 {
        let disable = instr & (1 << 4) != 0; // im
        let affect_pri = instr & (1 << 1) != 0; // I
        let affect_fault = instr & (1 << 0) != 0; // F
        if !affect_pri && !affect_fault {
            // UNPREDICTABLE in the architecture; a compiler never emits it.
            panic!("CPS that names no mask at {:#010X}", pc);
        }

        if self.is_privileged() {
            if affect_pri {
                self.primask = disable;
            }
            if affect_fault {
                // FAULTMASK cannot be set from the NMI or HardFault
                // handler, where the priority is already that high.
                if !disable || self.execution_priority(bus, true) > -1 {
                    self.faultmask = disable;
                }
            }
        }
        trace!(
            self,
            "  cps{} {}{}  -> primask = {}, faultmask = {}",
            if disable { "id" } else { "ie" },
            if affect_pri { "i" } else { "" },
            if affect_fault { "f" } else { "" },
            self.primask as u8,
            self.faultmask as u8
        );
        CYCLES_DATA
    }

    /// MRS <Rd>, <spec_reg> -- read a special-purpose register.
    /// Encoding T1: 1111001111101111 | 1000 Rd SYSm  (B5.2.2)
    pub(super) fn mrs(&mut self, hw2: u16, pc: u32) -> u32 {
        let d = ((hw2 >> 8) & 0xF) as usize; // bits 11-8
        let sysm = (hw2 & 0xFF) as u32; // bits 7-0
        let privileged = self.is_privileged();

        let value = match sysm {
            0..=SYSM_XPSR_LAST => {
                let mut value = 0;
                if sysm & 1 != 0 {
                    value |= self.xpsr & XPSR_IPSR;
                }
                // Bit 1 would add EPSR (the Thumb bit and the IT block
                // state), but that always reads as 0.
                if sysm & 4 == 0 {
                    value |= self.xpsr & (XPSR_NZCVQ | XPSR_GE);
                }
                value
            }
            // Unprivileged code reads 0 for the stack pointers and masks.
            SYSM_MSP..=SYSM_FAULTMASK if !privileged => 0,
            SYSM_MSP => self.main_sp(),
            SYSM_PSP => self.process_sp(),
            SYSM_PRIMASK => self.primask as u32,
            SYSM_BASEPRI | SYSM_BASEPRI_MAX => self.basepri as u32,
            SYSM_FAULTMASK => self.faultmask as u32,
            // Bit 2 (FPCA) belongs to the FPU, which is not emulated.
            SYSM_CONTROL => (self.control & (CONTROL_NPRIV | CONTROL_SPSEL)) as u32,
            _ => panic!("MRS from unknown special register {} at {:#010X}", sysm, pc),
        };
        self.regs[d] = value;
        trace!(
            self,
            "  mrs r{}, <special {}>  -> r{} = {:#010X}", d, sysm, d, value
        );
        CYCLES_SPECIAL_REGISTER
    }

    /// MSR <spec_reg>, <Rn> -- write a special-purpose register.
    /// Encoding T1: 111100111000 Rn | 1000 mask 00 SYSm  (B5.2.3)
    pub(super) fn msr(&mut self, hw1: u16, hw2: u16, pc: u32, bus: &Bus) -> u32 {
        let n = (hw1 & 0xF) as usize; // bits 3-0 of hw1
        let mask = (hw2 >> 10) & 0x3; // bits 11-10 of hw2
        let sysm = (hw2 & 0xFF) as u32; // bits 7-0 of hw2
        let value = self.regs[n];
        let privileged = self.is_privileged();

        match sysm {
            0..=SYSM_XPSR_LAST => {
                // Only the APSR part can be written; IPSR and EPSR are
                // read-only. `mask` picks which flags.
                if sysm & 4 == 0 {
                    if mask & 0b10 != 0 {
                        self.xpsr = (self.xpsr & !XPSR_NZCVQ) | (value & XPSR_NZCVQ);
                    }
                    if mask & 0b01 != 0 {
                        self.xpsr = (self.xpsr & !XPSR_GE) | (value & XPSR_GE);
                    }
                }
            }
            // Unprivileged writes to everything else are ignored.
            SYSM_MSP..=SYSM_CONTROL if !privileged => {}
            // A stack pointer is always a multiple of 4.
            SYSM_MSP => {
                if self.control & CONTROL_SPSEL != 0 {
                    self.msp = value & !3;
                } else {
                    self.regs[SP] = value & !3;
                }
            }
            SYSM_PSP => {
                if self.control & CONTROL_SPSEL != 0 {
                    self.regs[SP] = value & !3;
                } else {
                    self.psp = value & !3;
                }
            }
            SYSM_PRIMASK => self.primask = value & 1 != 0,
            SYSM_BASEPRI => self.basepri = (value & BASEPRI_MASK) as u8,
            SYSM_BASEPRI_MAX => {
                // Only takes effect if it masks more than BASEPRI does
                // now: a lower value that is not 0, or any value when
                // BASEPRI is off.
                let new = (value & BASEPRI_MASK) as u8;
                if new != 0 && (new < self.basepri || self.basepri == 0) {
                    self.basepri = new;
                }
            }
            SYSM_FAULTMASK => {
                if self.execution_priority(bus, true) > -1 {
                    self.faultmask = value & 1 != 0;
                }
            }
            SYSM_CONTROL => {
                self.control = (self.control & !CONTROL_NPRIV) | (value as u8 & CONTROL_NPRIV);
                // The stack can only be chosen in Thread mode; handlers
                // always use the main stack.
                if !self.in_handler_mode() {
                    self.select_stack(value as u8 & CONTROL_SPSEL != 0);
                }
            }
            _ => panic!("MSR to unknown special register {} at {:#010X}", sysm, pc),
        }
        trace!(self, "  msr <special {}>, r{}  ({:#010X})", sysm, n, value);
        CYCLES_SPECIAL_REGISTER
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::cpu::{cpu_with_code, nzcv};

    #[test]
    fn cpsid_i_sets_primask_and_cpsie_i_clears_it() {
        // cpsid i ; cpsie i
        let (mut cpu, mut bus) = cpu_with_code(&[0xB672, 0xB662]);
        cpu.step(&mut bus);
        assert!(cpu.primask);
        assert!(!cpu.faultmask);
        cpu.step(&mut bus);
        assert!(!cpu.primask);
    }

    #[test]
    fn cpsid_f_sets_faultmask_and_cpsie_f_clears_it() {
        // cpsid f ; cpsie f
        let (mut cpu, mut bus) = cpu_with_code(&[0xB671, 0xB661]);
        cpu.step(&mut bus);
        assert!(cpu.faultmask);
        assert!(!cpu.primask);
        cpu.step(&mut bus);
        assert!(!cpu.faultmask);
    }

    #[test]
    fn cps_can_change_both_masks_at_once() {
        // cpsid if
        let (mut cpu, mut bus) = cpu_with_code(&[0xB673]);
        cpu.step(&mut bus);
        assert!(cpu.primask && cpu.faultmask);
    }

    #[test]
    fn cpsid_holds_a_pending_exception_until_cpsie() {
        // cpsid i ; movs r0, #1 ; cpsie i ; movs r0, #2, with a SysTick
        // vector that would stop the test if it were taken too early
        // (it has no Thumb bit).
        let (mut cpu, mut bus) = cpu_with_code(&[0xB672, 0x2001, 0xB662, 0x2002]);
        cpu.step(&mut bus); // cpsid i
        bus.pend_exception(15);
        cpu.step(&mut bus); // movs r0, #1: not interrupted
        assert_eq!(cpu.regs[0], 1);
        cpu.step(&mut bus); // cpsie i
        assert!(bus.has_pending_exception(crate::bus::PRIORITY_THREAD));
    }

    #[test]
    fn mrs_and_msr_round_trip_primask() {
        // msr primask, r0 ; mrs r1, primask
        let (mut cpu, mut bus) = cpu_with_code(&[0xF380, 0x8810, 0xF3EF, 0x8110]);
        cpu.regs[0] = 0xFFFF_FFFF; // only bit 0 counts
        cpu.step(&mut bus);
        assert!(cpu.primask);
        cpu.step(&mut bus);
        assert_eq!(cpu.regs[1], 1);
    }

    #[test]
    fn basepri_keeps_only_the_three_priority_bits() {
        // msr basepri, r1 ; mrs r1, basepri
        let (mut cpu, mut bus) = cpu_with_code(&[0xF381, 0x8811, 0xF3EF, 0x8111]);
        cpu.regs[1] = 0x5F;
        cpu.step(&mut bus);
        assert_eq!(cpu.basepri, 0x40);
        cpu.step(&mut bus);
        assert_eq!(cpu.regs[1], 0x40);
    }

    #[test]
    fn basepri_max_only_ever_masks_more() {
        // msr basepri_max, r0, four times
        let code = [
            0xF380, 0x8812, 0xF380, 0x8812, 0xF380, 0x8812, 0xF380, 0x8812,
        ];
        let (mut cpu, mut bus) = cpu_with_code(&code);
        cpu.regs[0] = 0x80;
        cpu.step(&mut bus); // BASEPRI was off: taken
        assert_eq!(cpu.basepri, 0x80);
        cpu.regs[0] = 0xC0;
        cpu.step(&mut bus); // would mask less: ignored
        assert_eq!(cpu.basepri, 0x80);
        cpu.regs[0] = 0x40;
        cpu.step(&mut bus); // masks more: taken
        assert_eq!(cpu.basepri, 0x40);
        cpu.regs[0] = 0;
        cpu.step(&mut bus); // 0 would switch it off: ignored
        assert_eq!(cpu.basepri, 0x40);
    }

    #[test]
    fn faultmask_round_trips() {
        // msr faultmask, r0 ; mrs r2, faultmask
        let (mut cpu, mut bus) = cpu_with_code(&[0xF380, 0x8813, 0xF3EF, 0x8213]);
        cpu.regs[0] = 1;
        cpu.step(&mut bus);
        assert!(cpu.faultmask);
        cpu.step(&mut bus);
        assert_eq!(cpu.regs[2], 1);
    }

    #[test]
    fn mrs_reads_the_flags_and_msr_writes_them() {
        // mrs r2, apsr ; msr apsr_nzcvq, r0
        let (mut cpu, mut bus) = cpu_with_code(&[0xF3EF, 0x8200, 0xF380, 0x8800]);
        cpu.xpsr |= 0x6 << 28; // Z and C
        cpu.step(&mut bus);
        // The Thumb bit (EPSR) is not part of what MRS returns.
        assert_eq!(cpu.regs[2], 0x6000_0000);

        cpu.regs[0] = 0x9000_01FF; // N and V, plus bits that are read-only
        cpu.step(&mut bus);
        assert_eq!(nzcv(&cpu), 0b1001);
        assert_eq!(cpu.xpsr & XPSR_IPSR, 0);
    }

    #[test]
    fn mrs_ipsr_reads_the_exception_number() {
        // mrs r0, ipsr ; mrs r1, xpsr
        let (mut cpu, mut bus) = cpu_with_code(&[0xF3EF, 0x8005, 0xF3EF, 0x8103]);
        cpu.xpsr |= 15 | (1 << 31);
        cpu.step(&mut bus);
        assert_eq!(cpu.regs[0], 15);
        cpu.step(&mut bus);
        assert_eq!(cpu.regs[1], 0x8000_000F);
    }

    #[test]
    fn msr_psp_sets_up_the_stack_that_is_not_in_use() {
        // msr psp, r0 ; mrs r1, psp ; mrs r2, msp
        let code = [0xF380, 0x8809, 0xF3EF, 0x8109, 0xF3EF, 0x8208];
        let (mut cpu, mut bus) = cpu_with_code(&code);
        cpu.regs[0] = 0x2041_0003; // the low two bits are dropped
        for _ in 0..3 {
            cpu.step(&mut bus);
        }
        assert_eq!(cpu.regs[1], 0x2041_0000);
        assert_eq!(cpu.regs[2], 0x2046_0000);
        assert_eq!(cpu.regs[SP], 0x2046_0000); // still on the main stack
    }

    #[test]
    fn msr_msp_moves_the_stack_in_use() {
        // msr msp, r0
        let (mut cpu, mut bus) = cpu_with_code(&[0xF380, 0x8808]);
        cpu.regs[0] = 0x2042_0000;
        cpu.step(&mut bus);
        assert_eq!(cpu.regs[SP], 0x2042_0000);
    }

    #[test]
    fn writing_control_switches_thread_mode_to_the_process_stack() {
        // msr psp, r0 ; msr control, r1 ; mrs r2, control ; mrs r3, msp
        let code = [
            0xF380, 0x8809, 0xF381, 0x8814, 0xF3EF, 0x8214, 0xF3EF, 0x8308,
        ];
        let (mut cpu, mut bus) = cpu_with_code(&code);
        cpu.regs[0] = 0x2041_0000;
        cpu.regs[1] = 0b10; // SPSEL
        for _ in 0..4 {
            cpu.step(&mut bus);
        }
        assert_eq!(cpu.regs[SP], 0x2041_0000);
        assert_eq!(cpu.regs[2], 0b10);
        assert_eq!(cpu.regs[3], 0x2046_0000); // the main stack is kept
    }

    #[test]
    fn handler_cannot_switch_itself_to_the_process_stack() {
        // msr control, r1
        let (mut cpu, mut bus) = cpu_with_code(&[0xF381, 0x8814]);
        cpu.xpsr |= 15; // in the SysTick handler
        cpu.regs[1] = 0b10;
        cpu.step(&mut bus);
        assert_eq!(cpu.control & CONTROL_SPSEL, 0);
        assert_eq!(cpu.regs[SP], 0x2046_0000);
    }

    #[test]
    fn unprivileged_code_cannot_touch_the_masks_or_control() {
        // msr control, r1 (drop privilege) ; cpsid i ; msr basepri, r0 ;
        // msr control, r2 (try to get it back) ; mrs r3, msp
        let code = [
            0xF381, 0x8814, 0xB672, 0xF380, 0x8811, 0xF382, 0x8814, 0xF3EF, 0x8308,
        ];
        let (mut cpu, mut bus) = cpu_with_code(&code);
        cpu.regs[0] = 0x40;
        cpu.regs[1] = 0b01; // nPRIV
        cpu.regs[2] = 0;
        cpu.regs[3] = 0xFFFF_FFFF;
        for _ in 0..5 {
            cpu.step(&mut bus);
        }
        assert!(!cpu.primask);
        assert_eq!(cpu.basepri, 0);
        assert_eq!(cpu.control & CONTROL_NPRIV, CONTROL_NPRIV);
        assert_eq!(cpu.regs[3], 0); // the stack pointer reads as 0
    }

    #[test]
    fn mrs_and_msr_take_two_cycles_and_cps_one() {
        // mrs r0, primask ; msr primask, r0 ; cpsie i
        let (mut cpu, mut bus) = cpu_with_code(&[0xF3EF, 0x8010, 0xF380, 0x8810, 0xB662]);
        cpu.step(&mut bus);
        assert_eq!(cpu.cycles, 2);
        cpu.step(&mut bus);
        assert_eq!(cpu.cycles, 4);
        cpu.step(&mut bus);
        assert_eq!(cpu.cycles, 5);
    }
}
