//! Faults: the exceptions the processor raises against the running code
//! when an instruction cannot be carried out (ARM manual B1.5.14).
//!
//! What counts as a fault here:
//!
//! | Fault | Exception | Status bit |
//! |---|---|---|
//! | Undefined instruction (`udf`) | UsageFault | UNDEFINSTR |
//! | Executing with the Thumb bit clear | UsageFault | INVSTATE |
//! | Bad exception return | UsageFault | INVPC |
//! | Executing from an address with no memory | BusFault | IBUSERR |
//! | Data access to an unmapped address (only with `--bus-faults`) | BusFault | PRECISERR |
//! | Executing from a peripheral or system address | MemManage | IACCVIOL |
//!
//! MemManage, BusFault and UsageFault are switched off after reset, and
//! while one is off its faults are delivered as HardFault instead. The
//! same happens if the fault's own exception is not urgent enough to
//! interrupt the running code. This is called escalation.
//!
//! A fault that happens while HardFault or NMI is already running has
//! nowhere to go. The real chip stops in a state called lockup; the
//! emulator stops with a panic.
//!
//! An instruction the emulator has not implemented is NOT a fault: it
//! still panics, so that the emulator's to-do list is not mistaken for a
//! bug in the firmware.

use super::{CYCLES_EXCEPTION_ENTRY, Cpu, LR, PC, SP};
use crate::bus::{Bus, Executable};

/// What went wrong.
#[derive(Clone, Copy, Debug, PartialEq)]
pub(super) enum Fault {
    /// The instruction is one the architecture defines as undefined.
    UndefinedInstruction,
    /// An instruction was about to execute with the Thumb bit clear,
    /// because an address without bit 0 set was loaded into the PC.
    InvalidState,
    /// An exception return that cannot be carried out.
    InvalidPcLoad,
    /// The PC points where there is no memory at all.
    InstructionBusError,
    /// A load or store to an address where nothing is mapped.
    DataBusError { address: u32, write: bool },
    /// The PC points into a region that can never hold code.
    ExecuteNever,
}

// Exception numbers.
const HARD_FAULT: u32 = 3;
const MEM_MANAGE: u32 = 4;
const BUS_FAULT: u32 = 5;
const USAGE_FAULT: u32 = 6;

// CFSR bits. The register is three in one: MemManage status in bits 7-0,
// BusFault status in bits 15-8, UsageFault status in bits 31-16.
const CFSR_IACCVIOL: u32 = 1 << 0;
const CFSR_IBUSERR: u32 = 1 << 8;
const CFSR_PRECISERR: u32 = 1 << 9;
const CFSR_BFARVALID: u32 = 1 << 15; // BFAR holds the address
const CFSR_UNDEFINSTR: u32 = 1 << 16;
const CFSR_INVSTATE: u32 = 1 << 17;
const CFSR_INVPC: u32 = 1 << 18;

/// HFSR bit 30: a fault that could not use its own exception became this
/// HardFault.
const HFSR_FORCED: u32 = 1 << 30;

impl Fault {
    /// The exception this fault uses when that exception is enabled.
    fn exception(self) -> u32 {
        match self {
            Fault::UndefinedInstruction | Fault::InvalidState | Fault::InvalidPcLoad => USAGE_FAULT,
            Fault::InstructionBusError | Fault::DataBusError { .. } => BUS_FAULT,
            Fault::ExecuteNever => MEM_MANAGE,
        }
    }

    /// The bits to set in CFSR.
    fn status_bits(self) -> u32 {
        match self {
            Fault::UndefinedInstruction => CFSR_UNDEFINSTR,
            Fault::InvalidState => CFSR_INVSTATE,
            Fault::InvalidPcLoad => CFSR_INVPC,
            Fault::InstructionBusError => CFSR_IBUSERR,
            Fault::DataBusError { .. } => CFSR_PRECISERR | CFSR_BFARVALID,
            Fault::ExecuteNever => CFSR_IACCVIOL,
        }
    }

    /// One line for the dump: what happened, and the status bit by name.
    fn describe(self) -> String {
        match self {
            Fault::UndefinedInstruction => {
                "UsageFault: undefined instruction (CFSR.UNDEFINSTR)".to_string()
            }
            Fault::InvalidState => "UsageFault: executing with the Thumb bit clear, after an \
                 address without bit 0 set was loaded into the PC (CFSR.INVSTATE)"
                .to_string(),
            Fault::InvalidPcLoad => "UsageFault: invalid exception return (CFSR.INVPC)".to_string(),
            Fault::InstructionBusError => {
                "BusFault: no memory at the address being executed (CFSR.IBUSERR)".to_string()
            }
            Fault::DataBusError { address, write } => format!(
                "BusFault: {} unmapped address {:#010X} (CFSR.PRECISERR, BFAR valid)",
                if write { "write to" } else { "read of" },
                address
            ),
            Fault::ExecuteNever => {
                "MemManage: executing from a region that cannot hold code (CFSR.IACCVIOL)"
                    .to_string()
            }
        }
    }
}

fn exception_name(number: u32) -> &'static str {
    match number {
        HARD_FAULT => "HardFault",
        MEM_MANAGE => "MemManage",
        BUS_FAULT => "BusFault",
        USAGE_FAULT => "UsageFault",
        _ => "exception",
    }
}

impl Cpu {
    /// A fault found before the instruction at the PC is even fetched:
    /// the Thumb bit is clear, or the PC is somewhere code cannot be.
    pub(super) fn fetch_fault(&self, bus: &Bus) -> Option<Fault> {
        if self.xpsr & (1 << 24) == 0 {
            return Some(Fault::InvalidState);
        }
        match bus.executable(self.regs[PC]) {
            Executable::Yes => None,
            Executable::NoMemory => Some(Fault::InstructionBusError),
            Executable::Never => Some(Fault::ExecuteNever),
        }
    }

    /// Raise `fault` for the instruction at `regs[PC]`: record it in the
    /// SCB's fault status registers, print a dump of the state, and enter
    /// the handler. The registers must be as they were before the
    /// instruction, so that the saved return address is the instruction
    /// that faulted. Returns the clock cycles the entry takes.
    pub(super) fn raise_fault(&mut self, fault: Fault, bus: &mut Bus) -> u32 {
        let current = self.execution_priority(bus, true);

        // Use the fault's own exception if firmware has enabled it and it
        // is urgent enough to interrupt what is running. Otherwise the
        // fault escalates to HardFault.
        let own = fault.exception();
        let number = if bus.fault_exception_usable(own, current) {
            own
        } else {
            HARD_FAULT
        };

        let address = match fault {
            Fault::DataBusError { address, .. } => Some(address),
            _ => None,
        };
        let hfsr = if number == HARD_FAULT { HFSR_FORCED } else { 0 };
        bus.record_fault(fault.status_bits(), hfsr, address);

        self.dump_fault(fault, number, bus);

        // HardFault's priority is -1. If the running code is already at
        // that level or above (in the HardFault or NMI handler, or with
        // FAULTMASK set), nothing can take the fault.
        if number == HARD_FAULT && current <= -1 {
            panic!(
                "LOCKUP at {:#010X}: fault while running at priority {}, where no fault handler can run",
                self.regs[PC], current
            );
        }

        self.exception_entry(number, bus);
        CYCLES_EXCEPTION_ENTRY
    }

    /// Print the state at the moment of a fault to stderr.
    fn dump_fault(&self, fault: Fault, number: u32, bus: &mut Bus) {
        let pc = self.regs[PC];
        eprintln!();
        eprintln!("==== FAULT at PC {:#010X} ====", pc);
        eprintln!("  cause: {}", fault.describe());
        if number != fault.exception() {
            eprintln!(
                "  taken as: HardFault ({} is disabled or cannot interrupt the running code; HFSR.FORCED)",
                exception_name(fault.exception())
            );
        } else {
            eprintln!("  taken as: {}", exception_name(number));
        }

        // The instruction, if the PC is somewhere it can be read from.
        if bus.executable(pc) == Executable::Yes {
            let hw1 = bus.read16(pc);
            if hw1 >> 11 >= 0b11101 {
                let hw2 = bus.read16(pc.wrapping_add(2));
                eprintln!("  instruction: {:#06X} {:#06X}", hw1, hw2);
            } else {
                eprintln!("  instruction: {:#06X}", hw1);
            }
        }

        for row in 0..3 {
            let r = 4 * row;
            eprintln!(
                "  r{:<2} = {:08X}  r{:<2} = {:08X}  r{:<2} = {:08X}  r{:<2} = {:08X}",
                r,
                self.regs[r],
                r + 1,
                self.regs[r + 1],
                r + 2,
                self.regs[r + 2],
                r + 3,
                self.regs[r + 3]
            );
        }
        eprintln!(
            "  r12 = {:08X}  sp  = {:08X}  lr  = {:08X}  pc  = {:08X}",
            self.regs[12], self.regs[SP], self.regs[LR], pc
        );

        let ipsr = self.xpsr & 0x1FF;
        let flags: String = [(31, 'N'), (30, 'Z'), (29, 'C'), (28, 'V')]
            .iter()
            .map(|&(bit, name)| {
                if self.xpsr & (1 << bit) != 0 {
                    name
                } else {
                    '-'
                }
            })
            .collect();
        eprintln!("  xpsr = {:08X}  flags {}", self.xpsr, flags);
        if ipsr == 0 {
            eprintln!(
                "  mode: Thread, {} stack",
                if self.control & super::exception::CONTROL_SPSEL != 0 {
                    "process"
                } else {
                    "main"
                }
            );
        } else {
            eprintln!("  mode: Handler, in the handler for exception {}", ipsr);
        }
        eprintln!(
            "  masks: primask = {}, faultmask = {}, basepri = {:#04X}",
            self.primask as u8, self.faultmask as u8, self.basepri
        );
        eprintln!("  cycles since reset: {}", self.cycles);
        eprintln!("====");
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const MAIN: u32 = 0x0040_0100;
    const HARD_FAULT_HANDLER: u32 = 0x0040_0200;
    const OTHER_HANDLER: u32 = 0x0040_0240;
    const STACK_TOP: u32 = 0x2046_0000;

    const CFSR: u32 = 0xE000_ED28;
    const HFSR: u32 = 0xE000_ED2C;
    const BFAR: u32 = 0xE000_ED38;
    const SHCSR: u32 = 0xE000_ED24;
    const SHCSR_BUSFAULTENA: u32 = 1 << 17;
    const SHCSR_USGFAULTENA: u32 = 1 << 18;

    /// `main_code` at MAIN, with a HardFault handler (`adds r7, #1 ; b .`)
    /// and a second handler (`adds r6, #1 ; b .`) that serves MemManage,
    /// BusFault and UsageFault. R7 and R6 count how often each ran.
    fn cpu_with_fault_handlers(main_code: &[u16]) -> (Cpu, Bus) {
        cpu_with_handler_code(main_code, &[0x3701, 0xE7FE])
    }

    /// The same, with `hard_fault_code` as the HardFault handler.
    fn cpu_with_handler_code(main_code: &[u16], hard_fault_code: &[u16]) -> (Cpu, Bus) {
        let mut image = vec![0u8; 0x300];
        let mut put32 = |offset: usize, value: u32| {
            image[offset..offset + 4].copy_from_slice(&value.to_le_bytes());
        };
        put32(0, STACK_TOP);
        put32(4, MAIN | 1);
        put32(4 * 3, HARD_FAULT_HANDLER | 1);
        for number in [MEM_MANAGE, BUS_FAULT, USAGE_FAULT] {
            put32(4 * number as usize, OTHER_HANDLER | 1);
        }
        let mut put_code = |address: u32, code: &[u16]| {
            for (i, halfword) in code.iter().enumerate() {
                let offset = (address - 0x0040_0000) as usize + 2 * i;
                image[offset..offset + 2].copy_from_slice(&halfword.to_le_bytes());
            }
        };
        put_code(MAIN, main_code);
        put_code(HARD_FAULT_HANDLER, hard_fault_code);
        put_code(OTHER_HANDLER, &[0x3601, 0xE7FE]);
        let mut bus = Bus::new();
        bus.load_firmware(&image);
        let mut cpu = Cpu::new();
        cpu.reset(&mut bus);
        (cpu, bus)
    }

    /// The return address in the frame the fault pushed.
    fn stacked_return_address(cpu: &Cpu, bus: &mut Bus) -> u32 {
        bus.read32(cpu.regs[SP] + 0x18)
    }

    #[test]
    fn udf_raises_a_usage_fault_taken_as_hardfault() {
        // movs r0, #5 ; udf #0xfe -- what Rust emits for an abort.
        let (mut cpu, mut bus) = cpu_with_fault_handlers(&[0x2005, 0xDEFE]);
        cpu.step(&mut bus);
        cpu.step(&mut bus); // udf: fault, HardFault entry
        assert_eq!(cpu.xpsr & 0x1FF, HARD_FAULT);
        assert_eq!(cpu.regs[PC], HARD_FAULT_HANDLER);
        assert_eq!(bus.read32(CFSR), CFSR_UNDEFINSTR);
        assert_eq!(bus.read32(HFSR), HFSR_FORCED);
        // The frame points back at the udf itself, with R0 as it was.
        assert_eq!(stacked_return_address(&cpu, &mut bus), MAIN + 2);
        assert_eq!(bus.read32(cpu.regs[SP]), 5);

        cpu.step(&mut bus); // the handler runs
        assert_eq!(cpu.regs[7], 1);
    }

    #[test]
    fn wide_udf_is_undefined_too() {
        // udf.w #0
        let (mut cpu, mut bus) = cpu_with_fault_handlers(&[0xF7F0, 0xA000]);
        cpu.step(&mut bus);
        assert_eq!(cpu.xpsr & 0x1FF, HARD_FAULT);
        assert_eq!(bus.read32(CFSR), CFSR_UNDEFINSTR);
    }

    #[test]
    fn enabled_usage_fault_gets_its_own_handler() {
        let (mut cpu, mut bus) = cpu_with_fault_handlers(&[0xDEFE]);
        bus.write32(SHCSR, SHCSR_USGFAULTENA);
        cpu.step(&mut bus);
        assert_eq!(cpu.xpsr & 0x1FF, USAGE_FAULT);
        assert_eq!(cpu.regs[PC], OTHER_HANDLER);
        assert_eq!(bus.read32(HFSR), 0); // not escalated
    }

    #[test]
    fn enabled_fault_still_escalates_if_it_cannot_interrupt() {
        // UsageFault is enabled, but interrupts are off (PRIMASK), which
        // puts the running code above every configurable priority.
        let (mut cpu, mut bus) = cpu_with_fault_handlers(&[0xDEFE]);
        bus.write32(SHCSR, SHCSR_USGFAULTENA);
        cpu.primask = true;
        cpu.step(&mut bus);
        assert_eq!(cpu.xpsr & 0x1FF, HARD_FAULT);
        assert_eq!(bus.read32(HFSR), HFSR_FORCED);
    }

    #[test]
    fn branch_to_an_address_without_the_thumb_bit_faults_at_the_target() {
        // bx r0, with R0 an even address.
        let (mut cpu, mut bus) = cpu_with_fault_handlers(&[0x4700]);
        cpu.regs[0] = MAIN + 0x20;
        cpu.step(&mut bus); // the branch itself succeeds
        assert_eq!(cpu.regs[PC], MAIN + 0x20);
        cpu.step(&mut bus); // the next instruction cannot execute
        assert_eq!(cpu.xpsr & 0x1FF, HARD_FAULT);
        assert_eq!(bus.read32(CFSR), CFSR_INVSTATE);
        assert_eq!(stacked_return_address(&cpu, &mut bus), MAIN + 0x20);
        // The handler itself runs in Thumb state again.
        cpu.step(&mut bus);
        assert_eq!(cpu.regs[7], 1);
    }

    #[test]
    fn executing_where_there_is_no_memory_is_a_bus_fault() {
        // bx r0, to an address in the gap between flash and RAM.
        let (mut cpu, mut bus) = cpu_with_fault_handlers(&[0x4700]);
        cpu.regs[0] = 0x1000_0001;
        cpu.step(&mut bus);
        cpu.step(&mut bus);
        assert_eq!(cpu.xpsr & 0x1FF, HARD_FAULT);
        assert_eq!(bus.read32(CFSR), CFSR_IBUSERR);
        assert_eq!(stacked_return_address(&cpu, &mut bus), 0x1000_0000);
    }

    #[test]
    fn executing_from_a_peripheral_address_is_a_memmanage_fault() {
        // bx r0, into UART0's registers.
        let (mut cpu, mut bus) = cpu_with_fault_handlers(&[0x4700]);
        cpu.regs[0] = 0x400E_0801;
        cpu.step(&mut bus);
        cpu.step(&mut bus);
        assert_eq!(cpu.xpsr & 0x1FF, HARD_FAULT);
        assert_eq!(bus.read32(CFSR), CFSR_IACCVIOL);
    }

    #[test]
    fn code_in_ram_can_execute() {
        // bx r0, to `movs r1, #9` placed in RAM.
        let (mut cpu, mut bus) = cpu_with_fault_handlers(&[0x4700]);
        bus.write16(0x2040_0000, 0x2109);
        cpu.regs[0] = 0x2040_0001;
        cpu.step(&mut bus);
        cpu.step(&mut bus);
        assert_eq!(cpu.regs[1], 9);
        assert!(!cpu.in_handler_mode());
    }

    #[test]
    fn unmapped_data_access_only_warns_by_default() {
        // ldr r1, [r0]
        let (mut cpu, mut bus) = cpu_with_fault_handlers(&[0x6801]);
        cpu.regs[0] = 0x6000_0000;
        cpu.regs[1] = 0xFFFF_FFFF;
        cpu.step(&mut bus);
        assert!(!cpu.in_handler_mode());
        assert_eq!(cpu.regs[1], 0);
        assert_eq!(bus.read32(CFSR), 0);
    }

    #[test]
    fn unmapped_read_is_a_bus_fault_when_switched_on() {
        // ldr r1, [r0]
        let (mut cpu, mut bus) = cpu_with_fault_handlers(&[0x6801]);
        bus.set_bus_faults(true);
        cpu.regs[0] = 0x6000_0000;
        cpu.regs[1] = 0xFFFF_FFFF;
        cpu.step(&mut bus);
        assert_eq!(cpu.xpsr & 0x1FF, HARD_FAULT);
        assert_eq!(bus.read32(CFSR), CFSR_PRECISERR | CFSR_BFARVALID);
        assert_eq!(bus.read32(BFAR), 0x6000_0000);
        assert_eq!(stacked_return_address(&cpu, &mut bus), MAIN);
        // The load did not happen: R1 was not overwritten.
        assert_eq!(cpu.regs[1], 0xFFFF_FFFF);
    }

    #[test]
    fn unmapped_write_is_a_bus_fault_when_switched_on() {
        // str r1, [r0, #4]
        let (mut cpu, mut bus) = cpu_with_fault_handlers(&[0x6041]);
        bus.set_bus_faults(true);
        bus.write32(SHCSR, SHCSR_BUSFAULTENA);
        cpu.regs[0] = 0x0000_0000; // a null pointer... which is flash
        cpu.step(&mut bus);
        assert!(!cpu.in_handler_mode()); // writes to flash are only ignored

        let (mut cpu, mut bus) = cpu_with_fault_handlers(&[0x6041]);
        bus.set_bus_faults(true);
        bus.write32(SHCSR, SHCSR_BUSFAULTENA);
        cpu.regs[0] = 0x7000_0000;
        cpu.step(&mut bus);
        assert_eq!(cpu.xpsr & 0x1FF, BUS_FAULT);
        assert_eq!(bus.read32(BFAR), 0x7000_0004);
        assert_eq!(bus.read32(HFSR), 0);
    }

    #[test]
    fn faulting_instruction_in_an_it_block_keeps_its_place() {
        // ite eq ; ldreq r1, [r0] ; movne r2, #1, with the load faulting.
        let (mut cpu, mut bus) = cpu_with_fault_handlers(&[0xBF0C, 0x6801, 0x2201]);
        bus.set_bus_faults(true);
        cpu.xpsr |= 1 << 30; // Z
        cpu.regs[0] = 0x6000_0000;
        cpu.step(&mut bus); // ite eq
        cpu.step(&mut bus); // the load faults
        assert_eq!(cpu.xpsr & 0x1FF, HARD_FAULT);
        // The saved xPSR still has the IT block positioned on the load.
        let saved_xpsr = bus.read32(cpu.regs[SP] + 0x1C);
        assert_eq!((saved_xpsr >> 10) & 0x3F, 0x0C >> 2);
        assert_eq!(stacked_return_address(&cpu, &mut bus), MAIN + 2);
    }

    /// Put the CPU in the SysTick handler, about to execute the `bx lr`
    /// at MAIN with `exc_return` in LR.
    fn returning_from_systick_with(exc_return: u32) -> (Cpu, Bus) {
        let (mut cpu, mut bus) = cpu_with_fault_handlers(&[0x4770]);
        cpu.exception_entry(15, &mut bus);
        cpu.regs[PC] = MAIN;
        cpu.xpsr |= 1 << 24; // the image has no SysTick vector to set it
        cpu.regs[LR] = exc_return;
        (cpu, bus)
    }

    #[test]
    fn bad_exception_return_value_is_a_usage_fault() {
        let (mut cpu, mut bus) = returning_from_systick_with(0xFFFF_FFF3);
        let sp = cpu.regs[SP];
        cpu.step(&mut bus);
        assert_eq!(cpu.xpsr & 0x1FF, HARD_FAULT);
        assert_eq!(bus.read32(CFSR), CFSR_INVPC);
        // The fault interrupted the SysTick handler at its bx lr, with
        // nothing popped.
        assert_eq!(stacked_return_address(&cpu, &mut bus), MAIN);
        assert_eq!(cpu.regs[SP], sp - 0x20);
        assert_eq!(cpu.regs[LR], 0xFFFF_FFF1);
    }

    #[test]
    fn exception_return_to_the_wrong_mode_is_a_usage_fault() {
        // SysTick was taken from Thread mode, but the handler returns
        // with "back to a handler".
        let (mut cpu, mut bus) = returning_from_systick_with(0xFFFF_FFF1);
        cpu.step(&mut bus);
        assert_eq!(cpu.xpsr & 0x1FF, HARD_FAULT);
        assert_eq!(bus.read32(CFSR), CFSR_INVPC);
    }

    #[test]
    fn firmware_clears_fault_status_by_writing_ones() {
        let (mut cpu, mut bus) = cpu_with_fault_handlers(&[0xDEFE]);
        cpu.step(&mut bus);
        bus.write32(CFSR, CFSR_UNDEFINSTR);
        bus.write32(HFSR, HFSR_FORCED);
        assert_eq!(bus.read32(CFSR), 0);
        assert_eq!(bus.read32(HFSR), 0);
    }

    #[test]
    #[should_panic(expected = "LOCKUP at 0x00400202")]
    fn fault_inside_the_hardfault_handler_is_a_lockup() {
        // Main: udf. HardFault handler: adds r7, #1 ; udf.
        let (mut cpu, mut bus) = cpu_with_handler_code(&[0xDEFE], &[0x3701, 0xDEFE]);
        cpu.step(&mut bus); // first fault
        cpu.step(&mut bus); // adds r7, #1
        cpu.step(&mut bus); // second fault, at HardFault priority
    }

    #[test]
    #[should_panic(expected = "LOCKUP")]
    fn fault_with_no_usable_hardfault_handler_is_a_lockup() {
        // An image with only the first two vectors, like hello.s: the
        // HardFault vector is 0, which has no Thumb bit.
        let mut image = vec![0u8; 0x110];
        image[0..4].copy_from_slice(&STACK_TOP.to_le_bytes());
        image[4..8].copy_from_slice(&(MAIN | 1).to_le_bytes());
        image[0x100..0x102].copy_from_slice(&0xDEFEu16.to_le_bytes());
        let mut bus = Bus::new();
        bus.load_firmware(&image);
        let mut cpu = Cpu::new();
        cpu.reset(&mut bus);
        cpu.step(&mut bus); // udf: HardFault entry, PC = 0 without Thumb
        cpu.step(&mut bus); // cannot execute, and nothing can take it
    }
}
