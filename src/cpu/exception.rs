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

use super::{CYCLES_EXCEPTION_RETURN, CYCLES_REFILL, Cpu, LR, PC, SP};
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

// xPSR fields.
const XPSR_IPSR: u32 = 0x1FF; // bits 8-0: number of the exception being handled
const XPSR_STACK_ALIGN: u32 = 1 << 9; // in a saved xPSR: 4 bytes of padding were added
const XPSR_T: u32 = 1 << 24; // Thumb state, always 1 on a Cortex-M

// CONTROL bit 1 (SPSEL): 1 = Thread mode uses the process stack.
const CONTROL_SPSEL: u8 = 1 << 1;

// EXC_RETURN values: what the handler's LR holds, and what it loads into
// the PC to return. The values with bit 4 clear, used when the FPU has
// saved state as well, are not supported.
const EXC_RETURN_HANDLER: u32 = 0xFFFF_FFF1; // back to another handler, main stack
const EXC_RETURN_THREAD_MSP: u32 = 0xFFFF_FFF9; // back to Thread mode, main stack
const EXC_RETURN_THREAD_PSP: u32 = 0xFFFF_FFFD; // back to Thread mode, process stack

impl Cpu {
    /// True while an exception handler is running. The processor is in
    /// Handler mode exactly when IPSR is not zero.
    pub(super) fn in_handler_mode(&self) -> bool {
        self.xpsr & XPSR_IPSR != 0
    }

    /// Make `regs[SP]` the process stack (`true`) or the main stack
    /// (`false`), parking the other one in its field.
    fn select_stack(&mut self, process: bool) {
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
        let padded = self.regs[SP] & 4 != 0;
        let frame = self.regs[SP].wrapping_sub(FRAME_SIZE) & !4;
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

        // 2. LR tells the handler's return how to get back: to which mode
        //    and onto which stack.
        self.regs[LR] = if self.in_handler_mode() {
            EXC_RETURN_HANDLER
        } else if self.control & CONTROL_SPSEL != 0 {
            EXC_RETURN_THREAD_PSP
        } else {
            EXC_RETURN_THREAD_MSP
        };

        // 3. Handlers always run on the main stack, in Handler mode, with
        //    the exception number in IPSR.
        self.select_stack(false);
        self.xpsr = (self.xpsr & !XPSR_IPSR) | (number & XPSR_IPSR);

        // 4. Load the PC from the vector table.
        let table = bus.read32(VTOR);
        let vector = bus.read32(table.wrapping_add(4 * number));
        if vector & 1 == 0 {
            // The real chip would fault on the handler's first instruction.
            panic!(
                "vector for exception {} is {:#010X}, without the Thumb bit",
                number, vector
            );
        }
        self.regs[PC] = vector & !1;

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
        // An address must have bit 0 set to mark it as Thumb code. The
        // real chip raises a UsageFault on the next instruction if it is
        // clear; faults are not implemented, so stop here instead.
        if value & 1 == 0 {
            panic!(
                "instruction at {:#010X} loaded PC value {:#010X} without the Thumb bit",
                pc, value
            );
        }
        self.regs[PC] = value & !1;
        CYCLES_REFILL
    }

    /// Return from the current exception: pop the frame and resume the
    /// interrupted code. `pc` is the address of the returning instruction.
    fn exception_return(&mut self, exc_return: u32, pc: u32, bus: &mut Bus) {
        let returning = self.xpsr & XPSR_IPSR;

        // 1. EXC_RETURN picks the stack that holds the frame.
        match exc_return {
            EXC_RETURN_HANDLER | EXC_RETURN_THREAD_MSP => self.select_stack(false),
            EXC_RETURN_THREAD_PSP => self.select_stack(true),
            // The real chip raises a UsageFault for a bad value.
            _ => panic!(
                "instruction at {:#010X} returned from exception {} with unsupported EXC_RETURN {:#010X}",
                pc, returning, exc_return
            ),
        }

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

        // 3. Remove the frame, and the padding if entry added any.
        self.regs[SP] = frame.wrapping_add(FRAME_SIZE);
        if saved_xpsr & XPSR_STACK_ALIGN != 0 {
            self.regs[SP] |= 4;
        }

        // 4. Restore the flags and IPSR. Bit 9 is not part of the live
        //    xPSR, and the Thumb bit is always set.
        self.xpsr = (saved_xpsr & !XPSR_STACK_ALIGN) | XPSR_T;

        // 5. The restored IPSR must agree with the mode EXC_RETURN asked
        //    for. The real chip raises a UsageFault if it does not.
        let wants_handler = exc_return == EXC_RETURN_HANDLER;
        if wants_handler != self.in_handler_mode() {
            panic!(
                "instruction at {:#010X} returned from exception {} with EXC_RETURN {:#010X}, \
                 but the stacked xPSR {:#010X} is for the other mode",
                pc, returning, exc_return, saved_xpsr
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

    #[test]
    #[should_panic(expected = "unsupported EXC_RETURN 0xFFFFFFE9")]
    fn fpu_exc_return_values_are_rejected() {
        // Handler: bx lr, with LR changed to the FPU form of EXC_RETURN.
        let (mut cpu, mut bus) = cpu_with_handler(&[], &[0x4770], &[SYSTICK]);
        cpu.exception_entry(SYSTICK, &mut bus);
        cpu.regs[LR] = 0xFFFF_FFE9;
        cpu.step(&mut bus);
    }

    #[test]
    #[should_panic(expected = "without the Thumb bit")]
    fn vector_without_the_thumb_bit_stops() {
        // Exception 14 has no vector in this image: the table entry is 0.
        let (mut cpu, mut bus) = cpu_with_handler(&[], &[], &[SYSTICK]);
        cpu.exception_entry(14, &mut bus);
    }
}
