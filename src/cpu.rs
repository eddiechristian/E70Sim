use crate::bus::Bus;
use crate::helpers::{add_with_carry, condition_passed, lsl_c};

const SP: usize = 13;
const LR: usize = 14;
const PC: usize = 15;

/// Print to stderr, but only when the CPU's `trace` flag is set.
macro_rules! trace {
    ($cpu:expr, $($arg:tt)*) => {
        if $cpu.trace {
            eprintln!($($arg)*);
        }
    };
}

pub struct Cpu {
    /// When true, log every instruction and the register state to stderr.
    pub trace: bool,
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
    /// Fetch, decode and execute one instruction.
    pub fn step(&mut self, bus: &mut Bus) {
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
            self.execute32(hw1, hw2, pc, bus);
        } else {
            trace!(self, "{:#010X}: {:#06X} = {:016b}", pc, hw1, hw1);
            self.regs[PC] = pc.wrapping_add(2);
            self.execute16(hw1, pc, bus);
        }
        trace!(self, "{:?}\n", self);
    }

    /// Decode and execute a 32-bit instruction. `hw1` is the halfword at
    /// `pc` and `hw2` the one at `pc + 2`. Nothing is implemented yet, so
    /// this is the to-do list for 32-bit encodings.
    fn execute32(&mut self, hw1: u16, hw2: u16, pc: u32, _bus: &mut Bus) {
        panic!(
            "unhandled 32-bit instruction at {:#010X}: {:#06X} {:#06X} = {:016b} {:016b}",
            pc, hw1, hw2, hw1, hw2
        );
    }

    /// Decode and execute a 16-bit instruction located at `pc`.
    fn execute16(&mut self, instr: u16, pc: u32, bus: &mut Bus) {
        // LDR (literal), encoding T1: 01001 Rt imm8  (A7.7.44)
        if instr >> 11 == 0b01001 {
            self.ldr_literal(instr, pc, bus);
        // LDRB (immediate), encoding T1: 01111 imm5 Rn Rt  (A7.7.46)
        } else if instr >> 11 == 0b01111 {
            self.ldrb_immediate(instr, bus);
        // ADD (immediate), encoding T2: 00110 Rdn imm8  (A7.7.3)
        } else if instr >> 11 == 0b00110 {
            self.add_immediate(instr);
        // CMP (immediate), encoding T1: 00101 Rn imm8  (A7.7.27)
        } else if instr >> 11 == 0b00101 {
            self.cmp_immediate(instr);
        // B<cond>, encoding T1: 1101 cond imm8  (A7.7.12)
        // cond 1110 is UDF and 1111 is SVC, so they are excluded here.
        } else if instr >> 12 == 0b1101 && (instr >> 8) & 0xF < 0b1110 {
            self.b_conditional(instr, pc);
        // LDR (immediate), encoding T1: 01101 imm5 Rn Rt  (A7.7.43)
        } else if instr >> 11 == 0b01101 {
            self.ldr_immediate(instr, bus);
        // LSL (immediate), encoding T1: 00000 imm5 Rm Rd  (A7.7.68)
        // imm5 == 0 is MOVS Rd, Rm instead, which is not implemented yet.
        } else if instr >> 11 == 0b00000 && (instr >> 6) & 0x1F != 0 {
            self.lsl_immediate(instr);
        // STR (immediate), encoding T1: 01100 imm5 Rn Rt  (A7.7.161)
        } else if instr >> 11 == 0b01100 {
            self.str_immediate(instr, bus);
        // B (unconditional), encoding T2: 11100 imm11  (A7.7.12)
        } else if instr >> 11 == 0b11100 {
            self.b_unconditional(instr, pc);
        } else {
            panic!(
                "unhandled instruction at {:#010X}: {:#06X} = {:016b}",
                pc, instr, instr
            );
        }
    }

    /// LDR <Rt>, <label> -- load a word from a PC-relative literal pool.
    /// `pc` is the address of this instruction.
    fn ldr_literal(&mut self, instr: u16, pc: u32, bus: &mut Bus) {
        let t = ((instr >> 8) & 0x7) as usize; // bits 10-8
        let imm32 = ((instr & 0xFF) as u32) << 2; // imm8:'00'

        // The PC reads as instruction address + 4, then is word-aligned.
        let base = pc.wrapping_add(4) & !0b11;
        let address = base.wrapping_add(imm32);

        self.regs[t] = bus.read32(address);
        trace!(
            self,
            "  ldr r{}, [pc, #{}]  -> r{} = {:#010X}", t, imm32, t, self.regs[t]
        );
    }

    /// LDR <Rt>, [<Rn>, #imm] -- load a word from Rn + offset into Rt.
    fn ldr_immediate(&mut self, instr: u16, bus: &mut Bus) {
        let imm32 = (((instr >> 6) & 0x1F) as u32) << 2; // bits 10-6, imm5:'00'
        let n = ((instr >> 3) & 0x7) as usize; // bits 5-3, base register
        let t = (instr & 0x7) as usize; // bits 2-0, destination

        let address = self.regs[n].wrapping_add(imm32);

        self.regs[t] = bus.read32(address);
        trace!(
            self,
            "  ldr r{}, [r{}, #{}]  -> r{} = {:#010X}", t, n, imm32, t, self.regs[t]
        );
    }

    /// STR <Rt>, [<Rn>, #imm] -- store the word in Rt to Rn + offset.
    fn str_immediate(&mut self, instr: u16, bus: &mut Bus) {
        let imm32 = (((instr >> 6) & 0x1F) as u32) << 2; // bits 10-6, imm5:'00'
        let n = ((instr >> 3) & 0x7) as usize; // bits 5-3, base register
        let t = (instr & 0x7) as usize; // bits 2-0, value to store

        let address = self.regs[n].wrapping_add(imm32);

        trace!(
            self,
            "  str r{}, [r{}, #{}]  -> [{:#010X}] = {:#010X}", t, n, imm32, address, self.regs[t]
        );
        bus.write32(address, self.regs[t]);
    }

    /// B <label> -- always branch. `pc` is the address of this instruction.
    fn b_unconditional(&mut self, instr: u16, pc: u32) {
        // imm11:'0', sign-extended from 12 bits.
        let imm11 = (instr & 0x7FF) as u32;
        let imm32 = (((imm11 << 1) << 20) as i32) >> 20;
        // The PC reads as instruction address + 4.
        let target = pc.wrapping_add(4).wrapping_add(imm32 as u32);

        self.regs[PC] = target;
        trace!(self, "  b {:#010X}", target);
    }

    /// LDRB <Rt>, [<Rn>, #imm5] -- load one byte from Rn + offset and
    /// zero-extend it into Rt.
    fn ldrb_immediate(&mut self, instr: u16, bus: &mut Bus) {
        let imm32 = ((instr >> 6) & 0x1F) as u32; // bits 10-6, not scaled
        let n = ((instr >> 3) & 0x7) as usize; // bits 5-3, base register
        let t = (instr & 0x7) as usize; // bits 2-0, destination

        let address = self.regs[n].wrapping_add(imm32);

        self.regs[t] = bus.read8(address) as u32; // u8 -> u32 zero-extends
        trace!(
            self,
            "  ldrb r{}, [r{}, #{}]  -> r{} = {:#04X}", t, n, imm32, t, self.regs[t]
        );
    }

    fn set_nzcv(&mut self, result: u32, carry: bool, overflow: bool) {
        self.xpsr &= 0x0FFF_FFFF;
        self.xpsr |= result & 0x8000_0000; // N = bit 31 of result
        self.xpsr |= ((result == 0) as u32) << 30; // Z
        self.xpsr |= (carry as u32) << 29; // C
        self.xpsr |= (overflow as u32) << 28; // V
    }

    fn add_immediate(&mut self, instr: u16) {
        let d = ((instr >> 8) & 0x7) as usize; // bits 10-8, Rdn
        let imm32 = (instr & 0xFF) as u32; // bits 7-0

        let (result, carry, overflow) = add_with_carry(self.regs[d], imm32, false);
        self.regs[d] = result;
        self.set_nzcv(result, carry, overflow);
        trace!(
            self,
            "  adds r{}, #{}  -> r{} = {:#010X}", d, imm32, d, result
        );
    }

    /// B<cond> <label> -- branch if the condition holds.
    /// `pc` is the address of this instruction.
    fn b_conditional(&mut self, instr: u16, pc: u32) {
        let cond = (instr >> 8) & 0xF; // bits 11-8
        // imm8:'0', sign-extended: a signed byte offset in halfwords.
        let imm32 = ((instr & 0xFF) as u8 as i8 as i32) << 1;
        // The PC reads as instruction address + 4.
        let target = pc.wrapping_add(4).wrapping_add(imm32 as u32);

        let taken = condition_passed(cond, self.xpsr);
        if taken {
            self.regs[PC] = target;
        }
        trace!(
            self,
            "  b<cond {:04b}> {:#010X}  -> {}",
            cond,
            target,
            if taken { "taken" } else { "not taken" }
        );
    }

    /// LSLS <Rd>, <Rm>, #<imm5> -- shift left, setting N, Z and C.
    fn lsl_immediate(&mut self, instr: u16) {
        let shift_n = ((instr >> 6) & 0x1F) as u32; // bits 10-6, 1..=31 here
        let m = ((instr >> 3) & 0x7) as usize; // bits 5-3, source
        let d = (instr & 0x7) as usize; // bits 2-0, destination

        let (result, carry) = lsl_c(self.regs[m], shift_n);

        self.regs[d] = result;
        // A shift leaves V unchanged, so pass the current V back in.
        let v = self.xpsr & (1 << 28) != 0;
        self.set_nzcv(result, carry, v);
        trace!(
            self,
            "  lsls r{}, r{}, #{}  -> r{} = {:#010X}", d, m, shift_n, d, result
        );
    }

    /// CMP <Rn>, #<imm8> -- subtract and set the flags, discarding the result.
    fn cmp_immediate(&mut self, instr: u16) {
        let n = ((instr >> 8) & 0x7) as usize; // bits 10-8
        let imm32 = (instr & 0xFF) as u32; // bits 7-0

        // Rn - imm32, computed as Rn + NOT(imm32) + 1
        let (result, carry, overflow) = add_with_carry(self.regs[n], !imm32, true);
        self.set_nzcv(result, carry, overflow);
        trace!(
            self,
            "  cmp r{}, #{}  -> N={} Z={} C={} V={}",
            n,
            imm32,
            result >> 31,
            (result == 0) as u8,
            carry as u8,
            overflow as u8
        );
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A CPU and bus with `code` placed at the reset address 0x0040_0008.
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

    #[test]
    fn step_16_bit_advances_pc_by_2() {
        let (mut cpu, mut bus) = cpu_with_code(&[0x3101]); // adds r1, #1
        cpu.step(&mut bus);
        assert_eq!(cpu.regs[1], 1);
        assert_eq!(cpu.regs[PC], 0x0040_000A);
    }

    #[test]
    #[should_panic(expected = "unhandled 32-bit instruction at 0x00400008: 0xF000 0xF802")]
    fn step_fetches_both_halves_of_a_32_bit_instruction() {
        let (mut cpu, mut bus) = cpu_with_code(&[0xF000, 0xF802]); // bl
        cpu.step(&mut bus);
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
