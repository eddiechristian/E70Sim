use crate::bus::Bus;

const SP: usize = 13;
const LR: usize = 14;
const PC: usize = 15;

// How many clock cycles things take. Emulated time is counted in clock
// cycles: SysTick, and any timer added later, advances by these amounts.
//
// These are estimates. They follow the instruction timings Arm publishes
// for the Cortex-M4. The Cortex-M7 in the SAME70 has no such simple table:
// it can run two instructions at once and has caches and branch
// prediction, so its real timing depends on the surrounding code.

/// Arithmetic, logic, moves, shifts and compares.
const CYCLES_DATA: u32 = 1;
/// One load from or store to memory. PUSH and POP instead cost
/// CYCLES_DATA plus one cycle per register.
const CYCLES_MEMORY: u32 = 2;
/// Extra cost whenever the PC changes (a taken branch, a call, a return):
/// the instructions already fetched are thrown away.
const CYCLES_REFILL: u32 = 2;
/// Exception entry: pushing the frame and fetching the vector.
const CYCLES_EXCEPTION_ENTRY: u32 = 12;
/// Exception return: popping the frame. Replaces CYCLES_REFILL.
const CYCLES_EXCEPTION_RETURN: u32 = 10;

/// Print to stderr, but only when the CPU's `trace` flag is set.
macro_rules! trace {
    ($cpu:expr, $($arg:tt)*) => {
        if $cpu.trace {
            eprintln!($($arg)*);
        }
    };
}

// Each of these files adds methods to `Cpu`. `thumb16` and `thumb32` hold
// the instructions, split by encoding length; `step` below picks between
// them. `exception` holds exception entry and return.
mod exception;
mod thumb16;
mod thumb32;

pub struct Cpu {
    /// When true, log every instruction and the register state to stderr.
    pub trace: bool,
    /// Clock cycles executed since reset.
    cycles: u64,
    regs: [u32; 16],
    xpsr: u32,
    msp: u32,
    psp: u32,
    primask: bool,
    faultmask: bool,
    basepri: u8,
    control: u8,
}

impl std::fmt::Debug for Cpu {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        // R0-R12, four per line
        for (i, value) in self.regs[..13].iter().enumerate() {
            write!(f, "r{:<2} = {:#010X}  ", i, value)?;
            if i % 4 == 3 {
                writeln!(f)?;
            }
        }
        writeln!(f)?;
        writeln!(f, "sp  = {:#010X}", self.regs[SP])?;
        writeln!(f, "lr  = {:#010X}", self.regs[LR])?;
        writeln!(f, "pc  = {:#010X}", self.regs[PC])?;
        write!(f, "xpsr = {:#010X}", self.xpsr)
    }
}
impl Cpu {
    /// Create a CPU with all state zeroed. Call `reset` afterwards to load
    /// the initial SP and PC from the vector table.
    pub fn new() -> Self {
        Cpu {
            trace: false,
            cycles: 0,
            regs: [0; 16],
            xpsr: 0,
            msp: 0,
            psp: 0,
            primask: false,
            faultmask: false,
            basepri: 0,
            control: 0,
        }
    }

    pub fn reset(&mut self, bus: &mut Bus) {
        // 1. Read initial Stack Pointer from address 0x0 (remaps to Flash 0x0040_0000)
        self.regs[SP] = bus.read32(0x0000_0000);

        // 2. Read initial PC from address 0x4 (remaps to Flash 0x0040_0004)
        let raw_pc = bus.read32(0x0000_0004);

        // Clear bit 0 for the execution address
        self.regs[PC] = raw_pc & !1;

        // 3. Set the Thumb bit in the processor status register (XPSR)
        // Bit 24 of XPSR is the T (Thumb) bit, which MUST be 1 on Cortex-M
        self.xpsr = 1 << 24;

        trace!(
            self,
            "CPU Reset Complete. SP: {:#010X}, PC: {:#010X}", self.regs[SP], self.regs[PC]
        );
    }

    /// Fetch, decode and execute one instruction, then advance emulated
    /// time by the clock cycles it took. If an exception is waiting, it is
    /// taken first, and the instruction executed is then the first one of
    /// its handler.
    pub fn step(&mut self, bus: &mut Bus) {
        // Exceptions are taken between instructions. Until priorities are
        // implemented, a handler is never interrupted by another exception:
        // anything that becomes pending meanwhile waits for Thread mode.
        let mut cycles = 0;
        if !self.in_handler_mode() {
            if let Some(number) = bus.take_pending_exception() {
                self.exception_entry(number, bus);
                cycles += CYCLES_EXCEPTION_ENTRY;
            }
        }

        let pc = self.regs[PC];
        let hw1 = bus.read16(pc); // Thumb instructions are 16-bit units

        // A first halfword whose top 5 bits are 11101, 11110 or 11111 is the
        // start of a 32-bit instruction (A5.1). The PC is advanced past the
        // instruction before executing it, so a branch can simply overwrite
        // regs[PC].
        if hw1 >> 11 >= 0b11101 {
            let hw2 = bus.read16(pc.wrapping_add(2));
            trace!(self, "{:#010X}: {:#06X} {:#06X}", pc, hw1, hw2);
            self.regs[PC] = pc.wrapping_add(4);
            cycles += self.execute32(hw1, hw2, pc, bus);
        } else {
            trace!(self, "{:#010X}: {:#06X} = {:016b}", pc, hw1, hw1);
            self.regs[PC] = pc.wrapping_add(2);
            cycles += self.execute16(hw1, pc, bus);
        }
        // Emulated time moves on by the cycles this step took.
        self.cycles += cycles as u64;
        bus.tick(cycles);
        trace!(self, "{:?}\ncycles = {} (+{})\n", self, self.cycles, cycles);
    }

    fn set_nzcv(&mut self, result: u32, carry: bool, overflow: bool) {
        self.xpsr &= 0x0FFF_FFFF;
        self.xpsr |= result & 0x8000_0000; // N = bit 31 of result
        self.xpsr |= ((result == 0) as u32) << 30; // Z
        self.xpsr |= (carry as u32) << 29; // C
        self.xpsr |= (overflow as u32) << 28; // V
    }
}

/// A CPU and bus with `code` placed at the reset address 0x0040_0008.
#[cfg(test)]
fn cpu_with_code(code: &[u16]) -> (Cpu, Bus) {
    let mut image = Vec::new();
    image.extend_from_slice(&0x2046_0000u32.to_le_bytes()); // initial SP
    image.extend_from_slice(&0x0040_0009u32.to_le_bytes()); // reset vector
    for halfword in code {
        image.extend_from_slice(&halfword.to_le_bytes());
    }
    let mut bus = Bus::new();
    bus.load_firmware(&image);
    let mut cpu = Cpu::new();
    cpu.reset(&mut bus);
    (cpu, bus)
}

/// The N, Z, C and V flags as a 4-bit number, N in the top bit.
#[cfg(test)]
fn nzcv(cpu: &Cpu) -> u32 {
    cpu.xpsr >> 28
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Clock cycles one step of `code` takes, with R0-R7 pointing into RAM
    /// so that loads and stores have somewhere to go.
    fn cycles_of(code: &[u16]) -> u64 {
        let (mut cpu, mut bus) = cpu_with_code(code);
        for r in 0..8 {
            cpu.regs[r] = 0x2040_0000;
        }
        cpu.regs[LR] = 0x0040_0101;
        cpu.step(&mut bus);
        cpu.cycles
    }

    #[test]
    fn simple_instructions_take_one_cycle() {
        assert_eq!(cycles_of(&[0x3101]), 1); // adds r1, #1
        assert_eq!(cycles_of(&[0x2200]), 1); // movs r2, #0
        assert_eq!(cycles_of(&[0x4281]), 1); // cmp r1, r0
        assert_eq!(cycles_of(&[0x466F]), 1); // mov r7, sp
        assert_eq!(cycles_of(&[0xF640, 0x6104]), 1); // movw r1, #0xe04
    }

    #[test]
    fn loads_and_stores_take_two_cycles() {
        assert_eq!(cycles_of(&[0x6801]), 2); // ldr r1, [r0]
        assert_eq!(cycles_of(&[0x6001]), 2); // str r1, [r0]
        assert_eq!(cycles_of(&[0x4800]), 2); // ldr r0, [pc, #0]
        assert_eq!(cycles_of(&[0xF8C1, 0x0010]), 2); // str.w r0, [r1, #0x10]
    }

    #[test]
    fn changing_the_pc_takes_two_extra_cycles() {
        assert_eq!(cycles_of(&[0xE7FE]), 3); // b .
        assert_eq!(cycles_of(&[0xF000, 0xF85A]), 3); // bl
        assert_eq!(cycles_of(&[0x4770]), 3); // bx lr
        assert_eq!(cycles_of(&[0x4687]), 3); // mov pc, r0
    }

    #[test]
    fn conditional_branch_costs_more_when_taken() {
        // After reset Z is clear: bne is taken, beq is not.
        assert_eq!(cycles_of(&[0xD1FE]), 3); // bne .
        assert_eq!(cycles_of(&[0xD0FE]), 1); // beq .
    }

    #[test]
    fn push_and_pop_take_one_cycle_per_register() {
        assert_eq!(cycles_of(&[0xB401]), 2); // push {r0}
        assert_eq!(cycles_of(&[0xB580]), 3); // push {r7, lr}
        assert_eq!(cycles_of(&[0xB4FF]), 9); // push {r0-r7}
        assert_eq!(cycles_of(&[0xBC15]), 4); // pop {r0, r2, r4}
    }

    #[test]
    fn cycles_add_up_over_several_steps() {
        // movs r0, #1 (1) ; movs r1, #2 (1) ; b . (3)
        let (mut cpu, mut bus) = cpu_with_code(&[0x2001, 0x2102, 0xE7FE]);
        cpu.step(&mut bus);
        cpu.step(&mut bus);
        cpu.step(&mut bus);
        assert_eq!(cpu.cycles, 5);
    }

    #[test]
    fn step_16_bit_advances_pc_by_2() {
        let (mut cpu, mut bus) = cpu_with_code(&[0x3101]); // adds r1, #1
        cpu.step(&mut bus);
        assert_eq!(cpu.regs[1], 1);
        assert_eq!(cpu.regs[PC], 0x0040_000A);
    }

    #[test]
    #[should_panic(expected = "unhandled 32-bit instruction at 0x00400008: 0xFBB1 0xF0F2")]
    fn step_fetches_both_halves_of_a_32_bit_instruction() {
        let (mut cpu, mut bus) = cpu_with_code(&[0xFBB1, 0xF0F2]); // udiv r0, r1, r2
        cpu.step(&mut bus);
    }

    #[test]
    fn call_and_return_come_back_to_the_instruction_after_bl() {
        // 0x00400008: bl 0x0040000E
        // 0x0040000C: (return lands here)
        // 0x0040000E: push {r7, lr}
        // 0x00400010: pop {r7, pc}
        let (mut cpu, mut bus) = cpu_with_code(&[0xF000, 0xF801, 0xE7FE, 0xB580, 0xBD80]);
        cpu.step(&mut bus); // bl
        assert_eq!(cpu.regs[PC], 0x0040_000E);
        cpu.step(&mut bus); // push
        cpu.step(&mut bus); // pop
        assert_eq!(cpu.regs[PC], 0x0040_000C);
        assert_eq!(cpu.regs[SP], 0x2046_0000);
    }

    #[test]
    fn top_five_bits_select_the_instruction_length() {
        // 11100 is the 16-bit unconditional branch, not a 32-bit prefix.
        let (mut cpu, mut bus) = cpu_with_code(&[0xE7FE]); // b .
        cpu.step(&mut bus);
        assert_eq!(cpu.regs[PC], 0x0040_0008);
    }
}

/*
psudeo code for ADD.
// 'insn' is the 16-bit halfword fetched from memory
if (insn >> 9) == 0b0001100 {
    // --- 1. Decode Bitfields ---
    let rm_idx = ((insn >> 6) & 0x7) as usize; // Bits 8-6
    let rn_idx = ((insn >> 3) & 0x7) as usize; // Bits 5-3
    let rd_idx = (insn & 0x7) as usize;        // Bits 2-0

    // --- 2. Evaluate Pseudocode Context ---
    let setflags = !cpu.in_it_block();

    // --- 3. Execute Core Operation ---
    let val_n = cpu.regs[rn_idx];
    let val_m = cpu.regs[rm_idx];

    // AddWithCarry is a shared helper from ARM A5 pseudocode
    let (result, carry, overflow) = add_with_carry(val_n, val_m, 0);

    cpu.regs[rd_idx] = result;

    // --- 4. Conditionally Set Flags ---
    if setflags {
        cpu.apsr.n = (result >> 31) & 1 == 1;
        cpu.apsr.z = result == 0;
        cpu.apsr.c = carry;
        cpu.apsr.v = overflow;
    }
}
*/
