//! Exception entry and return (ARMv7-M ARM, B1.5.6 to B1.5.8).
//!
//! Entry saves eight registers on the stack and jumps to a handler from the
//! vector table. Return is triggered by the handler loading a special
//! EXC_RETURN value into the PC; it restores the eight registers, and the
//! interrupted code carries on. See `exceptionInterrupt.md` in the repo
//! root for the background.
//!
//! `Cpu::step` decides *when*: it calls `exception_entry` if the bus has
//! an exception pending and no handler is running, and charges the clock
//! cycles that entry takes. There are no
//! priorities yet, so one handler never interrupts another.
//!
//! Stack pointers: `regs[SP]` is always the stack in use. Of `msp` and
//! `psp`, the field for the stack in use is stale, and the other holds
//! the value of the stack that is not in use. CONTROL bit 1 (SPSEL) says
//! which is which, and `select_stack` swaps them.

use super::fault::Fault;
use super::fpu::{FPCAR, FPCCR, FPCCR_HFRDY, FPCCR_LSPACT, FPCCR_LSPEN, FPCCR_THREAD, FPCCR_USER};
use super::{CYCLES_EXCEPTION_RETURN, CYCLES_REFILL, Cpu, LR, PC, SP, XPSR_IT};
use crate::bus::Bus;

/// Vector Table Offset Register, in the SCB.
const VTOR: u32 = 0xE000_ED08;

/// Size of the exception frame: eight 32-bit words.
const FRAME_SIZE: u32 = 0x20;

// Offsets of the saved registers within the frame.
const FRAME_R0: u32 = 0x00;
const FRAME_R1: u32 = 0x04;
const FRAME_R2: u32 = 0x08;
const FRAME_R3: u32 = 0x0C;
const FRAME_R12: u32 = 0x10;
const FRAME_LR: u32 = 0x14;
const FRAME_RETURN_ADDRESS: u32 = 0x18;
const FRAME_XPSR: u32 = 0x1C;

/// Exception number of the non-maskable interrupt.
const EXCEPTION_NMI: u32 = 2;

// xPSR fields.
const XPSR_IPSR: u32 = 0x1FF; // bits 8-0: number of the exception being handled
const XPSR_STACK_ALIGN: u32 = 1 << 9; // in a saved xPSR: 4 bytes of padding were added
const XPSR_T: u32 = 1 << 24; // Thumb state, always 1 on a Cortex-M

// CONTROL bit 1 (SPSEL): 1 = Thread mode uses the process stack.
pub(super) const CONTROL_SPSEL: u8 = 1 << 1;

// CONTROL bit 2 (FPCA): the running code has used the FPU, so an
// exception must make room to save the FPU registers too.
pub(super) const CONTROL_FPCA: u8 = 1 << 2;

// Size of the extended frame, used when the interrupted code had used
// the FPU: the 8 words above, then S0-S15, FPSCR and one unused word.
const FRAME_SIZE_FP: u32 = 0x68;
const FRAME_S0: u32 = 0x20;
const FRAME_FPSCR: u32 = 0x60;

// EXC_RETURN values: what the handler's LR holds, and what it loads into
// the PC to return. Each also has a form with bit 4 clear (0xFFFF_FFE1,
// E9, ED), which means the frame is an extended one.
const EXC_RETURN_HANDLER: u32 = 0xFFFF_FFF1; // back to another handler, main stack
const EXC_RETURN_THREAD_MSP: u32 = 0xFFFF_FFF9; // back to Thread mode, main stack
const EXC_RETURN_THREAD_PSP: u32 = 0xFFFF_FFFD; // back to Thread mode, process stack
const EXC_RETURN_NO_FP: u32 = 1 << 4; // the frame is the basic 8 words

impl Cpu {
    /// True while an exception handler is running. The processor is in
    /// Handler mode exactly when IPSR is not zero.
    pub(super) fn in_handler_mode(&self) -> bool {
        self.xpsr & XPSR_IPSR != 0
    }

    /// Make `regs[SP]` the process stack (`true`) or the main stack
    /// (`false`), parking the other one in its field.
    pub(super) fn select_stack(&mut self, process: bool) {
        let on_process = self.control & CONTROL_SPSEL != 0;
        if process == on_process {
            return;
        }
        if process {
            self.msp = self.regs[SP];
            self.regs[SP] = self.psp;
            self.control |= CONTROL_SPSEL;
        } else {
            self.psp = self.regs[SP];
            self.regs[SP] = self.msp;
            self.control &= !CONTROL_SPSEL;
        }
    }

    /// Take exception `number` (15 = SysTick, 16 + n = IRQ n, ...) between
    /// two instructions: save the interrupted state on the stack and start
    /// the handler. `regs[PC]` must hold the address of the next
    /// instruction, which is where the handler will return to.
    pub fn exception_entry(&mut self, number: u32, bus: &mut Bus) {
        let return_address = self.regs[PC];

        // 1. Push the frame onto the stack in use. The frame is aligned to
        //    8 bytes; if that took 4 bytes of padding, bit 9 of the saved
        //    xPSR records it so the return can undo it.
        //    If the interrupted code has used the FPU, the frame also has
        //    room for the FPU registers the handler is free to change.
        let with_fp = self.control & CONTROL_FPCA != 0;
        let frame_size = if with_fp { FRAME_SIZE_FP } else { FRAME_SIZE };
        let padded = self.regs[SP] & 4 != 0;
        let frame = self.regs[SP].wrapping_sub(frame_size) & !4;
        let saved_xpsr = if padded {
            self.xpsr | XPSR_STACK_ALIGN
        } else {
            self.xpsr & !XPSR_STACK_ALIGN
        };
        bus.write32(frame + FRAME_R0, self.regs[0]);
        bus.write32(frame + FRAME_R1, self.regs[1]);
        bus.write32(frame + FRAME_R2, self.regs[2]);
        bus.write32(frame + FRAME_R3, self.regs[3]);
        bus.write32(frame + FRAME_R12, self.regs[12]);
        bus.write32(frame + FRAME_LR, self.regs[LR]);
        bus.write32(frame + FRAME_RETURN_ADDRESS, return_address);
        bus.write32(frame + FRAME_XPSR, saved_xpsr);
        self.regs[SP] = frame;
        if with_fp {
            let fpccr = bus.read32(FPCCR);
            if fpccr & FPCCR_LSPEN == 0 {
                // Save the FPU registers now.
                for i in 0..16 {
                    bus.write32(frame + FRAME_S0 + 4 * i, self.fp[i as usize]);
                }
                bus.write32(frame + FRAME_FPSCR, self.fpscr);
            } else {
                // Lazy save (the default): only note where they belong.
                // They are written there if and when the handler uses
                // the FPU itself (`preserve_fp_state` in fpu.rs). Most
                // handlers never do, which saves 17 memory writes
                // (UpdateFPCCR, B1.5.6).
                let mut flags = FPCCR_LSPACT;
                if !self.is_privileged() {
                    flags |= FPCCR_USER;
                }
                if !self.in_handler_mode() {
                    flags |= FPCCR_THREAD;
                }
                if self.execution_priority(bus, true) > -1 {
                    flags |= FPCCR_HFRDY;
                }
                let kept = fpccr & !(FPCCR_LSPACT | FPCCR_USER | FPCCR_THREAD | FPCCR_HFRDY);
                bus.write32(FPCCR, kept | flags);
                bus.write32(FPCAR, frame + FRAME_S0);
            }
        }

        // 2. LR tells the handler's return how to get back: to which mode
        //    and onto which stack.
        self.regs[LR] = if self.in_handler_mode() {
            EXC_RETURN_HANDLER
        } else if self.control & CONTROL_SPSEL != 0 {
            EXC_RETURN_THREAD_PSP
        } else {
            EXC_RETURN_THREAD_MSP
        };
        if with_fp {
            self.regs[LR] &= !EXC_RETURN_NO_FP;
        }
        // The handler starts as code that has not used the FPU.
        self.control &= !CONTROL_FPCA;

        // 3. Handlers always run on the main stack, in Handler mode, with
        //    the exception number in IPSR.
        self.select_stack(false);
        // The handler does not start inside the interrupted code's IT
        // block; the saved xPSR keeps that for the return.
        self.xpsr = (self.xpsr & !(XPSR_IPSR | XPSR_IT)) | (number & XPSR_IPSR);
        // The exception is active from now until its handler returns,
        // and its priority is the one to beat.
        bus.activate_exception(number);
        bus.set_current_exception(number);

        // 4. Load the PC from the vector table.
        let table = bus.read32(VTOR);
        let vector = bus.read32(table.wrapping_add(4 * number));
        // Bit 0 of the vector becomes the Thumb bit. A vector without it
        // (an empty table entry, say) gives a handler that cannot execute:
        // its first instruction raises a UsageFault.
        self.regs[PC] = vector & !1;
        self.xpsr = (self.xpsr & !XPSR_T) | ((vector & 1) << 24);

        trace!(
            self,
            "EXCEPTION {} entry: frame at {:#010X}, return address {:#010X}, handler {:#010X}",
            number,
            frame,
            return_address,
            self.regs[PC]
        );
    }

    /// Write `value` to the PC the way BX and POP {..., pc} do
    /// (BXWritePC in the ARM pseudocode). In Handler mode a value whose top
    /// four bits are all 1 is not an address but EXC_RETURN, and ends the
    /// exception. `pc` is the address of the instruction doing the write,
    /// for error messages.
    ///
    /// Returns the clock cycles the write costs on top of the instruction
    /// itself: a pipeline refill for a branch, or the whole unstacking
    /// sequence for an exception return.
    pub(super) fn bx_write_pc(&mut self, value: u32, pc: u32, bus: &mut Bus) -> u32 {
        if self.in_handler_mode() && value >> 28 == 0xF {
            self.exception_return(value, pc, bus);
            return CYCLES_EXCEPTION_RETURN;
        }
        // Bit 0 of the address becomes the Thumb bit, and must be set.
        // The branch itself works either way; with the bit clear, the
        // instruction at the target raises a UsageFault (see `fault.rs`).
        self.regs[PC] = value & !1;
        self.xpsr = (self.xpsr & !XPSR_T) | ((value & 1) << 24);
        CYCLES_REFILL
    }

    /// Return from the current exception: pop the frame and resume the
    /// interrupted code. `pc` is the address of the returning instruction.
    fn exception_return(&mut self, exc_return: u32, pc: u32, bus: &mut Bus) {
        let returning = self.xpsr & XPSR_IPSR;

        // 0. Check that the return can be carried out, before changing
        //    anything. If not, it is a UsageFault (INVPC), raised by
        //    `step` against the returning instruction.
        //    Bit 4 only says which kind of frame it is.
        let with_fp = exc_return & EXC_RETURN_NO_FP == 0;
        let exc_return = exc_return | EXC_RETURN_NO_FP;
        let frame = match exc_return {
            EXC_RETURN_HANDLER | EXC_RETURN_THREAD_MSP => self.main_sp(),
            EXC_RETURN_THREAD_PSP => self.process_sp(),
            _ => {
                self.fault = Some(Fault::InvalidPcLoad);
                return;
            }
        };
        // The stacked xPSR must be for the mode EXC_RETURN asks for.
        let wants_handler = exc_return == EXC_RETURN_HANDLER;
        let stacked_is_handler = bus.read32(frame + FRAME_XPSR) & XPSR_IPSR != 0;
        if wants_handler != stacked_is_handler {
            self.fault = Some(Fault::InvalidPcLoad);
            return;
        }

        bus.deactivate_exception(returning);
        // FAULTMASK does not outlive the handler that set it, except
        // for NMI's (B5.2.1).
        if returning != EXCEPTION_NMI {
            self.faultmask = false;
        }

        // 1. EXC_RETURN picks the stack that holds the frame.
        self.select_stack(exc_return == EXC_RETURN_THREAD_PSP);

        // 2. Pop the frame.
        let frame = self.regs[SP];
        self.regs[0] = bus.read32(frame + FRAME_R0);
        self.regs[1] = bus.read32(frame + FRAME_R1);
        self.regs[2] = bus.read32(frame + FRAME_R2);
        self.regs[3] = bus.read32(frame + FRAME_R3);
        self.regs[12] = bus.read32(frame + FRAME_R12);
        self.regs[LR] = bus.read32(frame + FRAME_LR);
        self.regs[PC] = bus.read32(frame + FRAME_RETURN_ADDRESS) & !1;
        let saved_xpsr = bus.read32(frame + FRAME_XPSR);

        //    An extended frame also gives back the FPU registers, unless
        //    they were never saved because the handler left them alone.
        if with_fp {
            let fpccr = bus.read32(FPCCR);
            if fpccr & FPCCR_LSPACT != 0 {
                bus.write32(FPCCR, fpccr & !FPCCR_LSPACT);
            } else {
                for i in 0..16 {
                    self.fp[i as usize] = bus.read32(frame + FRAME_S0 + 4 * i);
                }
                self.fpscr = bus.read32(frame + FRAME_FPSCR);
            }
            // The resumed code is "FPU-using" exactly if its frame was
            // an extended one.
            self.control |= CONTROL_FPCA;
        } else {
            self.control &= !CONTROL_FPCA;
        }

        // 3. Remove the frame, and the padding if entry added any.
        let frame_size = if with_fp { FRAME_SIZE_FP } else { FRAME_SIZE };
        self.regs[SP] = frame.wrapping_add(frame_size);
        if saved_xpsr & XPSR_STACK_ALIGN != 0 {
            self.regs[SP] |= 4;
        }

        // 4. Restore the flags, the Thumb bit and IPSR. Bit 9 is not part
        //    of the live xPSR.
        self.xpsr = saved_xpsr & !XPSR_STACK_ALIGN;
        bus.set_current_exception(self.xpsr & XPSR_IPSR);

        // With the bus-faults switch on: the frame was not in mapped
        // memory. The real chip raises a BusFault (UNSTKERR) here, which
        // is not emulated.
        if let Some((address, _)) = bus.take_bus_fault() {
            panic!(
                "instruction at {:#010X} returned from exception {} with its frame at unmapped address {:#010X}",
                pc, returning, address
            );
        }

        trace!(
            self,
            "EXCEPTION {} return: frame at {:#010X}, resuming at {:#010X}",
            returning,
            frame,
            self.regs[PC]
        );
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const MAIN: u32 = 0x0040_0100; // where the reset vector points
    const HANDLER: u32 = 0x0040_0200; // where the test's exception vector points
    const STACK_TOP: u32 = 0x2046_0000;
    const SYSTICK: u32 = 15;

    /// A CPU and bus with a vector table, `main_code` at MAIN, and
    /// `handler_code` at HANDLER as the handler for every exception in
    /// `numbers`.
    fn cpu_with_handler(main_code: &[u16], handler_code: &[u16], numbers: &[u32]) -> (Cpu, Bus) {
        let mut image = vec![0u8; 0x300];
        let mut put32 = |offset: usize, value: u32| {
            image[offset..offset + 4].copy_from_slice(&value.to_le_bytes());
        };
        put32(0, STACK_TOP);
        put32(4, MAIN | 1);
        for &number in numbers {
            put32(4 * number as usize, HANDLER | 1);
        }
        for (i, halfword) in main_code.iter().enumerate() {
            let offset = (MAIN - 0x0040_0000) as usize + 2 * i;
            image[offset..offset + 2].copy_from_slice(&halfword.to_le_bytes());
        }
        for (i, halfword) in handler_code.iter().enumerate() {
            let offset = (HANDLER - 0x0040_0000) as usize + 2 * i;
            image[offset..offset + 2].copy_from_slice(&halfword.to_le_bytes());
        }
        let mut bus = Bus::new();
        bus.load_firmware(&image);
        let mut cpu = Cpu::new();
        cpu.reset(&mut bus);
        (cpu, bus)
    }

    /// Give every register that entry saves a recognisable value.
    fn fill_saved_registers(cpu: &mut Cpu) {
        cpu.regs[0] = 0xA0;
        cpu.regs[1] = 0xA1;
        cpu.regs[2] = 0xA2;
        cpu.regs[3] = 0xA3;
        cpu.regs[12] = 0xAC;
        cpu.regs[LR] = 0x0040_0055;
    }

    #[test]
    fn entry_pushes_the_eight_word_frame() {
        let (mut cpu, mut bus) = cpu_with_handler(&[], &[], &[SYSTICK]);
        fill_saved_registers(&mut cpu);
        cpu.xpsr |= 0x6 << 28; // Z and C set
        let xpsr = cpu.xpsr;

        cpu.exception_entry(SYSTICK, &mut bus);

        let frame = STACK_TOP - 0x20;
        assert_eq!(cpu.regs[SP], frame);
        assert_eq!(bus.read32(frame), 0xA0); // R0
        assert_eq!(bus.read32(frame + 0x04), 0xA1); // R1
        assert_eq!(bus.read32(frame + 0x08), 0xA2); // R2
        assert_eq!(bus.read32(frame + 0x0C), 0xA3); // R3
        assert_eq!(bus.read32(frame + 0x10), 0xAC); // R12
        assert_eq!(bus.read32(frame + 0x14), 0x0040_0055); // LR
        assert_eq!(bus.read32(frame + 0x18), MAIN); // return address
        assert_eq!(bus.read32(frame + 0x1C), xpsr); // xPSR, no padding bit
    }

    #[test]
    fn entry_starts_the_handler_in_handler_mode() {
        let (mut cpu, mut bus) = cpu_with_handler(&[], &[], &[SYSTICK]);
        assert!(!cpu.in_handler_mode());

        cpu.exception_entry(SYSTICK, &mut bus);

        assert_eq!(cpu.regs[PC], HANDLER);
        assert_eq!(cpu.regs[LR], 0xFFFF_FFF9);
        assert_eq!(cpu.xpsr & XPSR_IPSR, SYSTICK);
        assert!(cpu.in_handler_mode());
    }

    #[test]
    fn entry_finds_the_handler_for_an_irq() {
        // IRQ 7 (UART0) is exception 23, vector at table offset 0x5C.
        let (mut cpu, mut bus) = cpu_with_handler(&[], &[], &[16 + 7]);
        cpu.exception_entry(16 + 7, &mut bus);
        assert_eq!(cpu.regs[PC], HANDLER);
        assert_eq!(cpu.xpsr & XPSR_IPSR, 23);
    }

    #[test]
    fn entry_reads_the_vector_table_through_vtor() {
        // Move the table to 0x0040_0200; exception 15's vector is then at
        // 0x0040_023C.
        let mut image = vec![0u8; 0x300];
        image[0..4].copy_from_slice(&STACK_TOP.to_le_bytes());
        image[4..8].copy_from_slice(&(MAIN | 1).to_le_bytes());
        image[0x23C..0x240].copy_from_slice(&0x0040_0281u32.to_le_bytes());
        let mut bus = Bus::new();
        bus.load_firmware(&image);
        let mut cpu = Cpu::new();
        cpu.reset(&mut bus);
        bus.write32(VTOR, 0x0040_0200);

        cpu.exception_entry(SYSTICK, &mut bus);

        assert_eq!(cpu.regs[PC], 0x0040_0280);
    }

    #[test]
    fn handler_returning_with_bx_lr_resumes_the_interrupted_code() {
        // Handler: movs r0, #7 ; bx lr
        let (mut cpu, mut bus) = cpu_with_handler(&[], &[0x2007, 0x4770], &[SYSTICK]);
        fill_saved_registers(&mut cpu);
        cpu.xpsr |= 0x6 << 28; // Z and C set
        let xpsr = cpu.xpsr;

        cpu.exception_entry(SYSTICK, &mut bus);
        cpu.step(&mut bus); // movs r0, #7: changes R0 and clears Z
        assert_eq!(cpu.regs[0], 7);
        cpu.step(&mut bus); // bx lr: exception return

        // Everything is as it was before the exception.
        assert_eq!(cpu.regs[PC], MAIN);
        assert_eq!(cpu.regs[SP], STACK_TOP);
        assert_eq!(cpu.regs[0], 0xA0);
        assert_eq!(cpu.regs[1], 0xA1);
        assert_eq!(cpu.regs[2], 0xA2);
        assert_eq!(cpu.regs[3], 0xA3);
        assert_eq!(cpu.regs[12], 0xAC);
        assert_eq!(cpu.regs[LR], 0x0040_0055);
        assert_eq!(cpu.xpsr, xpsr);
        assert!(!cpu.in_handler_mode());
    }

    #[test]
    fn handler_returning_with_pop_pc_resumes_the_interrupted_code() {
        // Handler as a compiler writes one that calls other functions:
        // push {r7, lr} ; movs r0, #7 ; pop {r7, pc}
        let (mut cpu, mut bus) = cpu_with_handler(&[], &[0xB580, 0x2007, 0xBD80], &[SYSTICK]);
        fill_saved_registers(&mut cpu);
        cpu.regs[7] = 0x77;

        cpu.exception_entry(SYSTICK, &mut bus);
        cpu.step(&mut bus);
        cpu.step(&mut bus);
        cpu.step(&mut bus);

        assert_eq!(cpu.regs[PC], MAIN);
        assert_eq!(cpu.regs[SP], STACK_TOP);
        assert_eq!(cpu.regs[0], 0xA0);
        assert_eq!(cpu.regs[7], 0x77);
        assert!(!cpu.in_handler_mode());
    }

    #[test]
    fn interrupted_code_continues_after_the_handler() {
        // Main: movs r4, #1 ; movs r5, #2. The exception arrives between
        // the two instructions.
        let (mut cpu, mut bus) = cpu_with_handler(&[0x2401, 0x2502], &[0x4770], &[SYSTICK]);
        cpu.step(&mut bus); // movs r4, #1
        cpu.exception_entry(SYSTICK, &mut bus);
        cpu.step(&mut bus); // bx lr
        cpu.step(&mut bus); // movs r5, #2
        assert_eq!(cpu.regs[4], 1);
        assert_eq!(cpu.regs[5], 2);
        assert_eq!(cpu.regs[PC], MAIN + 4);
    }

    #[test]
    fn unaligned_stack_is_padded_and_restored() {
        let (mut cpu, mut bus) = cpu_with_handler(&[], &[0x4770], &[SYSTICK]);
        cpu.regs[SP] = STACK_TOP - 4; // a multiple of 4 but not of 8

        cpu.exception_entry(SYSTICK, &mut bus);

        // 4 bytes of padding, then the 32-byte frame.
        let frame = STACK_TOP - 4 - 4 - 0x20;
        assert_eq!(cpu.regs[SP], frame);
        assert_ne!(bus.read32(frame + 0x1C) & XPSR_STACK_ALIGN, 0);

        cpu.step(&mut bus); // bx lr
        assert_eq!(cpu.regs[SP], STACK_TOP - 4);
        // The padding marker does not leak into the live xPSR.
        assert_eq!(cpu.xpsr & XPSR_STACK_ALIGN, 0);
    }

    #[test]
    fn nested_exception_returns_to_the_first_handler() {
        // SysTick handler at HANDLER; a second exception (IRQ 7) arrives
        // while it runs and uses the same handler code: bx lr.
        let (mut cpu, mut bus) = cpu_with_handler(&[], &[0x4770], &[SYSTICK, 23]);

        cpu.exception_entry(SYSTICK, &mut bus);
        let sp_in_first = cpu.regs[SP];
        cpu.exception_entry(23, &mut bus);

        // Entered from Handler mode: LR says "return to a handler".
        assert_eq!(cpu.regs[LR], 0xFFFF_FFF1);
        assert_eq!(cpu.xpsr & XPSR_IPSR, 23);
        assert_eq!(cpu.regs[SP], sp_in_first - 0x20);

        cpu.step(&mut bus); // bx lr: back in the SysTick handler
        assert_eq!(cpu.xpsr & XPSR_IPSR, SYSTICK);
        assert_eq!(cpu.regs[LR], 0xFFFF_FFF9);
        assert_eq!(cpu.regs[SP], sp_in_first);
        assert_eq!(cpu.regs[PC], HANDLER);

        cpu.step(&mut bus); // bx lr: back in Thread mode
        assert!(!cpu.in_handler_mode());
        assert_eq!(cpu.regs[SP], STACK_TOP);
        assert_eq!(cpu.regs[PC], MAIN);
    }

    const SYST_CSR: u32 = 0xE000_E010;
    const SYST_RVR: u32 = 0xE000_E014;
    const SYST_CVR: u32 = 0xE000_E018;

    /// Main: an endless loop (`b .`). SysTick handler: `adds r4, #1 ;
    /// bx lr`, so R4 counts how many times the handler has run. R4 is not
    /// one of the registers the exception frame restores.
    fn cpu_counting_systicks() -> (Cpu, Bus) {
        cpu_with_handler(&[0xE7FE], &[0x3401, 0x4770], &[SYSTICK])
    }

    #[test]
    fn systick_runs_its_handler_while_the_main_loop_spins() {
        let (mut cpu, mut bus) = cpu_counting_systicks();
        // `b .` costs 3 cycles, so a 30-cycle period is 10 trips round
        // the main loop.
        bus.write32(SYST_RVR, 29);
        bus.write32(SYST_CVR, 0);
        bus.write32(SYST_CSR, 0b011); // ENABLE | TICKINT

        // The timer reaches zero as the tenth instruction finishes, but
        // the handler has not started.
        for _ in 0..10 {
            cpu.step(&mut bus);
        }
        assert_eq!(cpu.cycles, 30);
        assert_eq!(cpu.regs[4], 0);
        assert!(!cpu.in_handler_mode());

        // The next step takes the exception (12 cycles) and runs
        // `adds r4, #1` (1 cycle).
        cpu.step(&mut bus);
        assert_eq!(cpu.cycles, 43);
        assert_eq!(cpu.regs[4], 1);
        assert!(cpu.in_handler_mode());

        // `bx lr` returns to the main loop (1 + 10 cycles).
        cpu.step(&mut bus);
        assert_eq!(cpu.cycles, 54);
        assert!(!cpu.in_handler_mode());
        assert_eq!(cpu.regs[PC], MAIN);
        assert_eq!(cpu.regs[SP], STACK_TOP);

        // Over a long run the handler fires once per 30 cycles, whatever
        // mix of instructions those cycles were spent on. The last period
        // may have ended without its handler having started yet.
        for _ in 0..1000 {
            cpu.step(&mut bus);
        }
        let periods = (cpu.cycles / 30) as u32;
        assert!(
            cpu.regs[4] == periods || cpu.regs[4] + 1 == periods,
            "handler ran {} times in {} periods",
            cpu.regs[4],
            periods
        );
    }

    #[test]
    fn systick_without_tickint_never_runs_the_handler() {
        let (mut cpu, mut bus) = cpu_counting_systicks();
        bus.write32(SYST_RVR, 29);
        bus.write32(SYST_CVR, 0);
        bus.write32(SYST_CSR, 0b001); // ENABLE only
        for _ in 0..100 {
            cpu.step(&mut bus);
        }
        assert_eq!(cpu.regs[4], 0);
        // The timer still ran: COUNTFLAG (bit 16) is set.
        assert_ne!(bus.read32(SYST_CSR) & (1 << 16), 0);
    }

    /// Main: `wfi ; b` back to the wfi. SysTick handler: `adds r4, #1 ;
    /// bx lr`. The idle loop of tick-driven firmware.
    fn cpu_sleeping_between_systicks() -> (Cpu, Bus) {
        cpu_with_handler(&[0xBF30, 0xE7FD], &[0x3401, 0x4770], &[SYSTICK])
    }

    #[test]
    fn wfi_sleeps_until_the_next_systick() {
        let (mut cpu, mut bus) = cpu_sleeping_between_systicks();
        bus.write32(SYST_RVR, 11_999); // 12,000-cycle period
        bus.write32(SYST_CVR, 0);
        bus.write32(SYST_CSR, 0b011); // ENABLE | TICKINT

        cpu.step(&mut bus); // wfi
        assert_eq!(cpu.cycles, 1);
        assert_eq!(cpu.regs[PC], MAIN + 2);

        // One step skips the whole wait: no instruction runs, and time
        // jumps to the moment the timer reaches zero.
        cpu.step(&mut bus);
        assert_eq!(cpu.cycles, 12_000);
        assert_eq!(cpu.regs[PC], MAIN + 2);
        assert_eq!(cpu.regs[4], 0);

        // Awake: the exception is taken and the handler runs.
        cpu.step(&mut bus);
        assert_eq!(cpu.regs[4], 1);
        assert!(cpu.in_handler_mode());

        // After the handler, execution continues after the wfi.
        cpu.step(&mut bus); // bx lr
        assert_eq!(cpu.regs[PC], MAIN + 2);
        cpu.step(&mut bus); // b back to the wfi
        assert_eq!(cpu.regs[PC], MAIN);
    }

    #[test]
    fn sleeping_firmware_gets_a_tick_every_period_exactly() {
        let (mut cpu, mut bus) = cpu_sleeping_between_systicks();
        bus.write32(SYST_RVR, 11_999);
        bus.write32(SYST_CVR, 0);
        bus.write32(SYST_CSR, 0b011);
        // Each trip is 5 steps: wfi, sleep, handler (2 steps), branch.
        for _ in 0..5 * 100 {
            cpu.step(&mut bus);
        }
        assert_eq!(cpu.regs[4], 100);
        // The 100th period ended at 1,200,000 cycles; the handler and the
        // branch back took 12 + 1 + 11 + 3 more.
        assert_eq!(cpu.cycles, 1_200_000 + 27);
    }

    #[test]
    fn wfi_with_an_exception_already_pending_does_not_sleep() {
        let (mut cpu, mut bus) = cpu_sleeping_between_systicks();
        bus.pend_exception(SYSTICK);
        // Hold the exception back for one step with PRIMASK ("interrupts
        // off"), so that wfi executes while it is still pending.
        cpu.primask = true;
        cpu.step(&mut bus); // wfi
        cpu.primask = false;

        cpu.step(&mut bus); // wakes at once: entry and adds r4, #1
        assert_eq!(cpu.regs[4], 1);
        assert_eq!(cpu.cycles, 1 + 12 + 1);
    }

    #[test]
    fn wfi_with_nothing_to_wake_it_stays_asleep() {
        // SysTick is off, so no exception will ever arrive.
        let (mut cpu, mut bus) = cpu_sleeping_between_systicks();
        cpu.step(&mut bus); // wfi
        for _ in 0..50 {
            cpu.step(&mut bus);
        }
        // Still parked after the wfi; time has crept on a cycle per step.
        assert_eq!(cpu.regs[PC], MAIN + 2);
        assert_eq!(cpu.cycles, 51);
    }

    #[test]
    fn exception_pending_during_a_handler_waits_for_it_to_finish() {
        let (mut cpu, mut bus) = cpu_counting_systicks();
        bus.pend_exception(SYSTICK);
        cpu.step(&mut bus); // entry, then adds r4, #1
        assert_eq!(cpu.regs[4], 1);

        // A second request arrives while the handler is still running.
        bus.pend_exception(SYSTICK);
        cpu.step(&mut bus); // bx lr: the first handler finishes normally
        assert!(!cpu.in_handler_mode());
        assert_eq!(cpu.regs[SP], STACK_TOP);

        // Only now is the second one taken.
        cpu.step(&mut bus);
        assert_eq!(cpu.regs[4], 2);
        assert!(cpu.in_handler_mode());
    }

    #[test]
    fn thread_code_on_the_process_stack_keeps_its_own_stack() {
        let (mut cpu, mut bus) = cpu_with_handler(&[], &[0x4770], &[SYSTICK]);
        // Thread mode running on the process stack, as under an operating
        // system: regs[SP] is PSP, and MSP is parked in its field.
        const PROCESS_STACK: u32 = 0x2044_0000;
        const MAIN_STACK: u32 = 0x2045_0000;
        cpu.control = CONTROL_SPSEL;
        cpu.regs[SP] = PROCESS_STACK;
        cpu.msp = MAIN_STACK;
        cpu.regs[0] = 0xA0;

        cpu.exception_entry(SYSTICK, &mut bus);

        // The frame went onto the process stack...
        assert_eq!(bus.read32(PROCESS_STACK - 0x20), 0xA0);
        // ...but the handler runs on the main stack.
        assert_eq!(cpu.regs[SP], MAIN_STACK);
        assert_eq!(cpu.regs[LR], 0xFFFF_FFFD);
        assert_eq!(cpu.control & CONTROL_SPSEL, 0);

        cpu.step(&mut bus); // bx lr

        assert_eq!(cpu.regs[SP], PROCESS_STACK);
        assert_eq!(cpu.control & CONTROL_SPSEL, CONTROL_SPSEL);
        assert_eq!(cpu.msp, MAIN_STACK);
        assert_eq!(cpu.regs[PC], MAIN);
    }

    #[test]
    fn exc_return_value_is_an_ordinary_address_in_thread_mode() {
        // In Thread mode there is no exception to return from, so the
        // value is just a (strange) branch target. Main: bx lr.
        let (mut cpu, mut bus) = cpu_with_handler(&[0x4770], &[], &[SYSTICK]);
        cpu.regs[LR] = 0xFFFF_FFF9;
        cpu.step(&mut bus);
        assert_eq!(cpu.regs[PC], 0xFFFF_FFF8);
        assert_eq!(cpu.regs[SP], STACK_TOP);
    }

    const CPACR: u32 = 0xE000_ED88;
    const FPCCR_ADDR: u32 = 0xE000_EF34;
    const FPCAR_ADDR: u32 = 0xE000_EF38;
    const FPDSCR_ADDR: u32 = 0xE000_EF3C;

    /// Main: vmov.f32 s0, #1.0 ; b .
    /// SysTick handler: `handler_code`. The FPU is switched on.
    fn cpu_using_the_fpu(handler_code: &[u16]) -> (Cpu, Bus) {
        let (mut cpu, mut bus) =
            cpu_with_handler(&[0xEEB7, 0x0A00, 0xE7FE], handler_code, &[SYSTICK]);
        bus.write32(CPACR, 0xF << 20);
        cpu.step(&mut bus); // vmov.f32 s0, #1.0
        (cpu, bus)
    }

    #[test]
    fn code_that_has_not_used_the_fpu_gets_the_basic_frame() {
        let (mut cpu, mut bus) = cpu_with_handler(&[0xE7FE], &[0x4770], &[SYSTICK]);
        bus.write32(CPACR, 0xF << 20); // enabled, but never used
        cpu.exception_entry(SYSTICK, &mut bus);
        assert_eq!(cpu.regs[SP], STACK_TOP - 0x20);
        assert_eq!(cpu.regs[LR], 0xFFFF_FFF9);
    }

    #[test]
    fn code_that_has_used_the_fpu_gets_the_extended_frame() {
        // Handler: bx lr
        let (mut cpu, mut bus) = cpu_using_the_fpu(&[0x4770]);
        assert_eq!(cpu.control & CONTROL_FPCA, CONTROL_FPCA);
        cpu.exception_entry(SYSTICK, &mut bus);
        assert_eq!(cpu.regs[SP], STACK_TOP - 0x68);
        assert_eq!(cpu.regs[LR], 0xFFFF_FFE9); // bit 4 clear: extended
        assert_eq!(cpu.control & CONTROL_FPCA, 0); // the handler starts clean
        // Lazy save: the space is reserved and its address noted, but
        // nothing has been written there.
        assert_ne!(bus.read32(FPCCR_ADDR) & 1, 0); // LSPACT
        assert_eq!(bus.read32(FPCAR_ADDR), STACK_TOP - 0x68 + 0x20);
        assert_eq!(bus.read32(STACK_TOP - 0x68 + 0x20), 0);

        cpu.step(&mut bus); // bx lr: everything is back
        assert!(!cpu.in_handler_mode());
        assert_eq!(cpu.regs[SP], STACK_TOP);
        assert_eq!(cpu.control & CONTROL_FPCA, CONTROL_FPCA);
        assert_eq!(cpu.fp[0], 1.0f32.to_bits());
        assert_eq!(bus.read32(FPCCR_ADDR) & 1, 0);
    }

    #[test]
    fn handler_that_uses_the_fpu_triggers_the_lazy_save() {
        // Handler: vmov.f32 s0, #-2.5 ; bx lr
        let (mut cpu, mut bus) = cpu_using_the_fpu(&[0xEEB8, 0x0A04, 0x4770]);
        cpu.fpscr = 0x2000_0000; // a flag the main code has set
        bus.pend_exception(SYSTICK);
        cpu.step(&mut bus); // entry, then the handler's vmov
        // The main code's S0 and FPSCR were written to the frame just
        // before the handler changed S0.
        let frame = STACK_TOP - 0x68;
        assert_eq!(bus.read32(frame + 0x20), 1.0f32.to_bits());
        assert_eq!(bus.read32(frame + 0x60), 0x2000_0000);
        assert_eq!(bus.read32(FPCCR_ADDR) & 1, 0); // save done
        assert_eq!(cpu.fp[0], (-2.5f32).to_bits());
        // The handler is now an FPU-using context itself.
        assert_eq!(cpu.control & CONTROL_FPCA, CONTROL_FPCA);

        cpu.step(&mut bus); // bx lr: the main code's values come back
        assert!(!cpu.in_handler_mode());
        assert_eq!(cpu.fp[0], 1.0f32.to_bits());
        assert_eq!(cpu.fpscr, 0x2000_0000);
        assert_eq!(cpu.regs[SP], STACK_TOP);
    }

    #[test]
    fn handler_starts_with_the_default_fpscr_mode_bits() {
        // Handler: vmov.f32 s0, #-2.5 ; bx lr
        let (mut cpu, mut bus) = cpu_using_the_fpu(&[0xEEB8, 0x0A04, 0x4770]);
        cpu.fpscr = 3 << 22; // main code rounds toward zero
        bus.write32(FPDSCR_ADDR, 1 << 22); // default for new contexts
        bus.pend_exception(SYSTICK);
        cpu.step(&mut bus);
        assert_eq!(cpu.fpscr & (3 << 22), 1 << 22);
        cpu.step(&mut bus); // bx lr
        assert_eq!(cpu.fpscr, 3 << 22);
    }

    #[test]
    fn with_lazy_saving_off_the_registers_are_saved_at_entry() {
        let (mut cpu, mut bus) = cpu_using_the_fpu(&[0x4770]);
        bus.write32(FPCCR_ADDR, 1 << 31); // ASPEN only, LSPEN off
        cpu.fp[15] = 0x1234_5678;
        cpu.exception_entry(SYSTICK, &mut bus);
        let frame = STACK_TOP - 0x68;
        assert_eq!(bus.read32(frame + 0x20), 1.0f32.to_bits());
        assert_eq!(bus.read32(frame + 0x20 + 4 * 15), 0x1234_5678);
        assert_eq!(bus.read32(FPCCR_ADDR) & 1, 0);

        cpu.fp[0] = 0;
        cpu.fp[15] = 0;
        cpu.step(&mut bus); // bx lr
        assert_eq!(cpu.fp[0], 1.0f32.to_bits());
        assert_eq!(cpu.fp[15], 0x1234_5678);
    }

    #[test]
    fn extended_frame_is_aligned_like_the_basic_one() {
        let (mut cpu, mut bus) = cpu_using_the_fpu(&[0x4770]);
        cpu.regs[SP] = STACK_TOP - 4;
        cpu.exception_entry(SYSTICK, &mut bus);
        assert_eq!(cpu.regs[SP], STACK_TOP - 8 - 0x68);
        cpu.step(&mut bus); // bx lr
        assert_eq!(cpu.regs[SP], STACK_TOP - 4);
    }

    #[test]
    fn vector_without_the_thumb_bit_gives_a_handler_that_cannot_run() {
        // Exception 14 has no vector in this image: the table entry is 0.
        let (mut cpu, mut bus) = cpu_with_handler(&[], &[], &[SYSTICK]);
        cpu.exception_entry(14, &mut bus);
        // Entry itself works; the handler's first instruction would fault
        // (tested in fault.rs).
        assert_eq!(cpu.regs[PC], 0);
        assert_eq!(cpu.xpsr & XPSR_T, 0);
    }

    const UART0: u32 = 0x400E_0800;
    const UART_CR: u32 = UART0;
    const UART_IER: u32 = UART0 + 0x08;
    const NVIC_ISER0: u32 = 0xE000_E100;
    const NVIC_ISPR0: u32 = 0xE000_E200;
    const UART0_IRQ: u32 = 7;
    const UART0_EXCEPTION: u32 = 16 + UART0_IRQ;

    /// `main_code` with a UART0 handler that reads the received byte into
    /// R4: `ldr r0, =UART0 ; ldr r4, [r0, #0x18] ; bx lr`. The UART's
    /// receiver is on and fed from the returned channel, but no interrupt
    /// is enabled yet.
    fn cpu_receiving_on_uart0(main_code: &[u16]) -> (Cpu, Bus, std::sync::mpsc::Sender<u8>) {
        let handler = [0x4801, 0x6984, 0x4770, 0x0000, 0x0800, 0x400E];
        let (cpu, mut bus) = cpu_with_handler(main_code, &handler, &[UART0_EXCEPTION]);
        let (sender, receiver) = std::sync::mpsc::channel();
        bus.set_uart0_input(receiver);
        bus.write32(UART_CR, 1 << 4); // RXEN
        (cpu, bus, sender)
    }

    #[test]
    fn byte_received_on_uart0_runs_its_handler() {
        // Main: b .
        let (mut cpu, mut bus, sender) = cpu_receiving_on_uart0(&[0xE7FE]);
        bus.write32(UART_IER, 1 << 0); // RXRDY
        bus.write32(NVIC_ISER0, 1 << UART0_IRQ);

        // Nothing received: the main loop just spins.
        for _ in 0..10 {
            cpu.step(&mut bus);
        }
        assert!(!cpu.in_handler_mode());

        sender.send(b'a').unwrap();
        cpu.step(&mut bus); // the byte arrives as this instruction ends
        assert!(!cpu.in_handler_mode());
        cpu.step(&mut bus); // exception entry and `ldr r0, =UART0`
        assert!(cpu.in_handler_mode());
        assert_eq!(cpu.xpsr & 0x1FF, UART0_EXCEPTION);
        cpu.step(&mut bus); // ldr r4, [r0, #0x18]: reads UART_RHR
        assert_eq!(cpu.regs[4], b'a' as u32);
        cpu.step(&mut bus); // bx lr
        assert!(!cpu.in_handler_mode());
        assert_eq!(cpu.regs[PC], MAIN);

        // The byte was read, so the handler does not run again.
        for _ in 0..10 {
            cpu.step(&mut bus);
            assert!(!cpu.in_handler_mode());
        }
    }

    #[test]
    fn bytes_received_together_each_get_their_own_handler_run() {
        let (mut cpu, mut bus, sender) = cpu_receiving_on_uart0(&[0xE7FE]);
        bus.write32(UART_IER, 1 << 0);
        bus.write32(NVIC_ISER0, 1 << UART0_IRQ);
        for byte in *b"hi!" {
            sender.send(byte).unwrap();
        }
        let mut received = Vec::new();
        let mut was_in_handler = false;
        for _ in 0..40 {
            cpu.step(&mut bus);
            // Note R4 each time a handler run finishes.
            if was_in_handler && !cpu.in_handler_mode() {
                received.push(cpu.regs[4] as u8);
            }
            was_in_handler = cpu.in_handler_mode();
        }
        assert_eq!(received, b"hi!");
    }

    #[test]
    fn uart0_handler_does_not_run_unless_enabled_in_the_nvic() {
        let (mut cpu, mut bus, sender) = cpu_receiving_on_uart0(&[0xE7FE]);
        bus.write32(UART_IER, 1 << 0); // enabled in the UART only
        sender.send(b'a').unwrap();
        for _ in 0..20 {
            cpu.step(&mut bus);
            assert!(!cpu.in_handler_mode());
        }
        // Enabling it lets the waiting interrupt through.
        bus.write32(NVIC_ISER0, 1 << UART0_IRQ);
        cpu.step(&mut bus);
        assert!(cpu.in_handler_mode());
    }

    #[test]
    fn byte_received_on_uart0_wakes_a_sleeping_cpu() {
        // Main: wfi ; b back to the wfi.
        let (mut cpu, mut bus, sender) = cpu_receiving_on_uart0(&[0xBF30, 0xE7FD]);
        bus.write32(UART_IER, 1 << 0);
        bus.write32(NVIC_ISER0, 1 << UART0_IRQ);
        for _ in 0..20 {
            cpu.step(&mut bus);
        }
        assert_eq!(cpu.regs[PC], MAIN + 2); // asleep after the wfi

        sender.send(b'z').unwrap();
        cpu.step(&mut bus); // still asleep; the byte arrives
        cpu.step(&mut bus); // wakes, takes the exception
        assert!(cpu.in_handler_mode());
        cpu.step(&mut bus);
        assert_eq!(cpu.regs[4], b'z' as u32);
    }

    #[test]
    fn irq_pended_by_firmware_runs_its_handler_once() {
        let (mut cpu, mut bus, _sender) = cpu_receiving_on_uart0(&[0xE7FE]);
        bus.write32(NVIC_ISER0, 1 << UART0_IRQ);
        bus.write32(NVIC_ISPR0, 1 << UART0_IRQ);
        let mut entries = 0;
        let mut was_in_handler = false;
        for _ in 0..30 {
            cpu.step(&mut bus);
            if cpu.in_handler_mode() && !was_in_handler {
                entries += 1;
            }
            was_in_handler = cpu.in_handler_mode();
        }
        assert_eq!(entries, 1);
    }

    #[test]
    fn exception_in_the_middle_of_an_it_block_resumes_it() {
        // Main: ite eq ; moveq r0, #1 ; movne r0, #2.
        // Handler: movs r1, #5 ; bx lr. Outside an IT block movs sets the
        // flags, so it clears Z while the handler runs.
        let (mut cpu, mut bus) =
            cpu_with_handler(&[0xBF0C, 0x2001, 0x2002], &[0x2105, 0x4770], &[SYSTICK]);
        cpu.xpsr |= 1 << 30; // Z
        cpu.step(&mut bus); // ite eq
        bus.pend_exception(SYSTICK);

        cpu.step(&mut bus); // entry, then movs r1, #5
        assert!(cpu.in_handler_mode());
        assert_eq!(cpu.regs[1], 5);
        // The handler did not inherit the IT block: movs set the flags.
        assert_eq!(cpu.xpsr & (1 << 30), 0);

        cpu.step(&mut bus); // bx lr: Z and the IT block come back
        cpu.step(&mut bus); // moveq r0, #1
        cpu.step(&mut bus); // movne r0, #2, skipped
        assert_eq!(cpu.regs[0], 1);
        assert_eq!(cpu.regs[PC], MAIN + 6);
    }

    const LOW: u32 = 0x0040_0200; // handler for the less urgent exception
    const HIGH: u32 = 0x0040_0240; // handler for the more urgent one
    const SHPR3_SYSTICK: u32 = 0xE000_ED23; // SysTick's priority byte
    const NVIC_IPR7: u32 = 0xE000_E407; // IRQ 7's priority byte
    const NVIC_IABR0: u32 = 0xE000_E300;

    /// Main: `b .`. SysTick (exception 15) and IRQ 7 (exception 23) each
    /// have their own handler, and IRQ 7 is enabled in the NVIC.
    ///
    /// SysTick handler at LOW:  adds r4, #1 ; adds r4, #1 ; bx lr
    /// IRQ 7 handler at HIGH:   mov r6, r4 ; bx lr
    ///
    /// R6 records how far the SysTick handler had got when the IRQ 7
    /// handler ran: 0 before it, 1 in the middle of it, 2 after it.
    fn cpu_with_two_handlers(systick_priority: u8, irq7_priority: u8) -> (Cpu, Bus) {
        let (cpu, mut bus) = cpu_with_handlers_at(SYSTICK, UART0_EXCEPTION);
        bus.write8(SHPR3_SYSTICK, systick_priority);
        bus.write8(NVIC_IPR7, irq7_priority);
        (cpu, bus)
    }

    /// The same image with the handler at LOW serving exception `low` and
    /// the one at HIGH serving exception `high`. Priorities are left at 0.
    fn cpu_with_handlers_at(low: u32, high: u32) -> (Cpu, Bus) {
        let mut image = vec![0u8; 0x300];
        let mut put32 = |offset: usize, value: u32| {
            image[offset..offset + 4].copy_from_slice(&value.to_le_bytes());
        };
        put32(0, STACK_TOP);
        put32(4, MAIN | 1);
        put32(4 * low as usize, LOW | 1);
        put32(4 * high as usize, HIGH | 1);
        let mut put_code = |address: u32, code: &[u16]| {
            for (i, halfword) in code.iter().enumerate() {
                let offset = (address - 0x0040_0000) as usize + 2 * i;
                image[offset..offset + 2].copy_from_slice(&halfword.to_le_bytes());
            }
        };
        put_code(MAIN, &[0xE7FE]);
        put_code(LOW, &[0x3401, 0x3401, 0x4770]);
        put_code(HIGH, &[0x4626, 0x4770]);

        let mut bus = Bus::new();
        bus.load_firmware(&image);
        bus.write32(NVIC_ISER0, 1 << UART0_IRQ);
        let mut cpu = Cpu::new();
        cpu.reset(&mut bus);
        cpu.regs[6] = 0xFFFF_FFFF; // "the IRQ 7 handler has not run"
        (cpu, bus)
    }

    #[test]
    fn more_urgent_exception_interrupts_a_running_handler() {
        let (mut cpu, mut bus) = cpu_with_two_handlers(0xE0, 0x20);
        bus.pend_exception(SYSTICK);
        cpu.step(&mut bus); // SysTick entry, first adds
        assert_eq!(cpu.xpsr & XPSR_IPSR, SYSTICK);
        let sp_in_systick = cpu.regs[SP];

        bus.pend_exception(UART0_EXCEPTION);
        cpu.step(&mut bus); // IRQ 7 entry, mov r6, r4
        assert_eq!(cpu.xpsr & XPSR_IPSR, UART0_EXCEPTION);
        assert_eq!(cpu.regs[6], 1); // SysTick's handler was half done
        // A second frame sits below the first, and LR says "back to a
        // handler".
        assert_eq!(cpu.regs[SP], sp_in_systick - 0x20);
        assert_eq!(cpu.regs[LR], 0xFFFF_FFF1);
        // Both are active.
        assert_eq!(bus.read32(NVIC_IABR0), 1 << UART0_IRQ);

        cpu.step(&mut bus); // bx lr: back into the SysTick handler
        assert_eq!(cpu.xpsr & XPSR_IPSR, SYSTICK);
        assert_eq!(cpu.regs[PC], LOW + 2);
        assert_eq!(cpu.regs[SP], sp_in_systick);
        assert_eq!(bus.read32(NVIC_IABR0), 0);

        cpu.step(&mut bus); // second adds
        assert_eq!(cpu.regs[4], 2);
        cpu.step(&mut bus); // bx lr: back to main
        assert!(!cpu.in_handler_mode());
        assert_eq!(cpu.regs[PC], MAIN);
        assert_eq!(cpu.regs[SP], STACK_TOP);
    }

    #[test]
    fn less_urgent_exception_waits_for_the_running_handler() {
        // The other way round: SysTick is the urgent one.
        let (mut cpu, mut bus) = cpu_with_two_handlers(0x20, 0xE0);
        bus.pend_exception(SYSTICK);
        cpu.step(&mut bus); // SysTick entry, first adds
        bus.pend_exception(UART0_EXCEPTION);
        cpu.step(&mut bus); // second adds: not interrupted
        assert_eq!(cpu.xpsr & XPSR_IPSR, SYSTICK);
        cpu.step(&mut bus); // bx lr
        assert!(!cpu.in_handler_mode());

        cpu.step(&mut bus); // now IRQ 7 is taken
        assert_eq!(cpu.xpsr & XPSR_IPSR, UART0_EXCEPTION);
        assert_eq!(cpu.regs[6], 2); // SysTick's handler had finished
    }

    #[test]
    fn exception_of_equal_priority_waits_for_the_running_handler() {
        let (mut cpu, mut bus) = cpu_with_two_handlers(0x40, 0x40);
        bus.pend_exception(SYSTICK);
        cpu.step(&mut bus);
        bus.pend_exception(UART0_EXCEPTION);
        cpu.step(&mut bus);
        cpu.step(&mut bus); // SysTick handler runs to its end
        assert!(!cpu.in_handler_mode());
        cpu.step(&mut bus);
        assert_eq!(cpu.regs[6], 2);
    }

    #[test]
    fn of_two_waiting_exceptions_the_more_urgent_runs_first() {
        // IRQ 7 has the higher exception number but the higher priority.
        let (mut cpu, mut bus) = cpu_with_two_handlers(0xE0, 0x20);
        bus.pend_exception(SYSTICK);
        bus.pend_exception(UART0_EXCEPTION);
        cpu.step(&mut bus);
        assert_eq!(cpu.xpsr & XPSR_IPSR, UART0_EXCEPTION);
        assert_eq!(cpu.regs[6], 0); // SysTick's handler had not started
        cpu.step(&mut bus); // bx lr
        cpu.step(&mut bus); // SysTick follows
        assert_eq!(cpu.xpsr & XPSR_IPSR, SYSTICK);
    }

    #[test]
    fn handler_cannot_be_interrupted_by_its_own_exception() {
        let (mut cpu, mut bus) = cpu_with_two_handlers(0xE0, 0x20);
        bus.pend_exception(SYSTICK);
        cpu.step(&mut bus); // entry, first adds
        bus.pend_exception(SYSTICK);
        cpu.step(&mut bus); // second adds, same handler run
        assert_eq!(cpu.regs[4], 2);
        assert_eq!(cpu.regs[SP], STACK_TOP - 0x20);
    }

    #[test]
    fn primask_holds_back_every_configurable_exception() {
        let (mut cpu, mut bus) = cpu_with_two_handlers(0xE0, 0x00);
        cpu.primask = true;
        bus.pend_exception(SYSTICK);
        bus.pend_exception(UART0_EXCEPTION);
        for _ in 0..5 {
            cpu.step(&mut bus);
            assert!(!cpu.in_handler_mode());
        }
        // They were only waiting.
        cpu.primask = false;
        cpu.step(&mut bus);
        assert_eq!(cpu.xpsr & XPSR_IPSR, UART0_EXCEPTION);
    }

    #[test]
    fn basepri_holds_back_its_own_priority_and_below() {
        let (mut cpu, mut bus) = cpu_with_two_handlers(0x40, 0x20);
        cpu.basepri = 0x40;
        bus.pend_exception(SYSTICK);
        for _ in 0..5 {
            cpu.step(&mut bus);
            assert!(!cpu.in_handler_mode());
        }
        // A more urgent one still gets through.
        bus.pend_exception(UART0_EXCEPTION);
        cpu.step(&mut bus);
        assert_eq!(cpu.xpsr & XPSR_IPSR, UART0_EXCEPTION);
        cpu.step(&mut bus); // bx lr
        cpu.step(&mut bus);
        assert!(!cpu.in_handler_mode()); // SysTick is still held back

        cpu.basepri = 0;
        cpu.step(&mut bus);
        assert_eq!(cpu.xpsr & XPSR_IPSR, SYSTICK);
    }

    #[test]
    fn faultmask_holds_back_everything_but_nmi() {
        let (mut cpu, mut bus) = cpu_with_two_handlers(0x00, 0x00);
        cpu.faultmask = true;
        bus.pend_exception(SYSTICK);
        bus.pend_exception(3); // HardFault
        for _ in 0..5 {
            cpu.step(&mut bus);
            assert!(!cpu.in_handler_mode());
        }
    }

    #[test]
    fn sleeping_handler_is_woken_only_by_a_more_urgent_exception() {
        let (mut cpu, mut bus) = cpu_with_two_handlers(0xE0, 0x20);
        // Asleep while SysTick is active, as after a wfi in its handler.
        bus.pend_exception(SYSTICK);
        cpu.step(&mut bus);
        cpu.sleeping = true;

        bus.pend_exception(SYSTICK); // no more urgent than itself
        cpu.step(&mut bus);
        assert!(cpu.sleeping);

        bus.pend_exception(UART0_EXCEPTION);
        cpu.step(&mut bus);
        assert!(!cpu.sleeping);
        assert_eq!(cpu.xpsr & XPSR_IPSR, UART0_EXCEPTION);
    }

    #[test]
    fn critical_section_delays_the_handler_until_interrupts_are_back_on() {
        // Main: cpsid i ; adds r5, #1 ; adds r5, #1 ; cpsie i ; b .
        // SysTick handler: mov r6, r5 ; bx lr (R6 = R5 when it ran).
        let (mut cpu, mut bus) = cpu_with_handler(
            &[0xB672, 0x3501, 0x3501, 0xB662, 0xE7FE],
            &[0x462E, 0x4770],
            &[SYSTICK],
        );
        cpu.step(&mut bus); // cpsid i
        bus.pend_exception(SYSTICK); // arrives inside the critical section
        cpu.step(&mut bus);
        cpu.step(&mut bus);
        cpu.step(&mut bus); // cpsie i
        assert!(!cpu.in_handler_mode());
        assert_eq!(cpu.regs[5], 2);

        cpu.step(&mut bus); // taken before the next instruction
        assert!(cpu.in_handler_mode());
        assert_eq!(cpu.regs[6], 2); // both adds had run, uninterrupted
    }

    #[test]
    fn faultmask_is_cleared_when_the_handler_returns() {
        // SysTick handler: cpsid f ; bx lr
        let (mut cpu, mut bus) = cpu_with_handler(&[0xE7FE], &[0xB671, 0x4770], &[SYSTICK]);
        bus.pend_exception(SYSTICK);
        cpu.step(&mut bus); // entry, cpsid f
        assert!(cpu.faultmask);
        cpu.step(&mut bus); // bx lr
        assert!(!cpu.in_handler_mode());
        assert!(!cpu.faultmask);
    }

    #[test]
    fn primask_is_kept_when_the_handler_returns() {
        // SysTick handler: cpsid i ; bx lr
        let (mut cpu, mut bus) = cpu_with_handler(&[0xE7FE], &[0xB672, 0x4770], &[SYSTICK]);
        bus.pend_exception(SYSTICK);
        cpu.step(&mut bus);
        cpu.step(&mut bus);
        assert!(!cpu.in_handler_mode());
        assert!(cpu.primask);
    }

    const ICSR: u32 = 0xE000_ED04;

    #[test]
    fn handler_can_ask_for_pendsv_which_runs_after_it() {
        // The pattern an operating system uses: a handler asks for PendSV,
        // which has the lowest priority and so runs once every other
        // handler has finished.
        //
        // PendSV's handler is the one at LOW (adds r4, #1 ; ...) and
        // SysTick's the one at HIGH (mov r6, r4 ; bx lr).
        const PENDSV: u32 = 14;
        let (mut cpu, mut bus) = cpu_with_handlers_at(PENDSV, SYSTICK);
        bus.write8(0xE000_ED22, 0xE0); // SHPR3: PendSV at the lowest priority

        bus.pend_exception(SYSTICK);
        cpu.step(&mut bus); // SysTick entry, mov r6, r4
        assert_eq!(bus.read32(ICSR) & 0x1FF, SYSTICK); // VECTACTIVE
        bus.write32(ICSR, 1 << 28); // what the handler would write: PENDSVSET
        cpu.step(&mut bus); // bx lr: PendSV did not interrupt it
        assert!(!cpu.in_handler_mode());

        cpu.step(&mut bus); // PendSV entry, adds r4, #1
        assert_eq!(cpu.xpsr & XPSR_IPSR, PENDSV);
        assert_eq!(cpu.regs[4], 1);
        assert_eq!(bus.read32(ICSR) & (1 << 28), 0); // no longer pending
    }

    #[test]
    fn icsr_shows_no_running_handler_after_the_return() {
        let (mut cpu, mut bus) = cpu_counting_systicks();
        bus.pend_exception(SYSTICK);
        cpu.step(&mut bus);
        assert_eq!(bus.read32(ICSR) & 0x1FF, SYSTICK);
        cpu.step(&mut bus); // bx lr
        assert_eq!(bus.read32(ICSR) & 0x1FF, 0);
    }
}
