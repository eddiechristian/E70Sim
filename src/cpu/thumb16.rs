//! 16-bit Thumb instructions.
//!
//! `execute16` decodes one halfword and calls the method for that
//! instruction. Section numbers (A7.7.x) refer to the ARMv7-M Architecture
//! Reference Manual.

use super::{Cpu, LR, PC, SP};
use crate::bus::Bus;
use crate::helpers::{add_with_carry, condition_passed, lsl_c};

impl Cpu {
    /// Decode and execute a 16-bit instruction located at `pc`.
    pub(super) fn execute16(&mut self, instr: u16, pc: u32, bus: &mut Bus) {
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
        // PUSH, encoding T1: 1011010 M register_list  (A7.7.101)
        } else if instr >> 9 == 0b1011010 {
            self.push(instr, pc, bus);
        // STR (register), encoding T1: 0101000 Rm Rn Rt  (A7.7.162)
        } else if instr >> 9 == 0b0101000 {
            self.str_register(instr, bus);
        // CMP (register), encoding T1: 0100001010 Rm Rn  (A7.7.28)
        } else if instr >> 6 == 0b0100001010 {
            self.cmp_register(instr);
        // MOV (immediate), encoding T1: 00100 Rd imm8  (A7.7.76)
        } else if instr >> 11 == 0b00100 {
            self.mov_immediate(instr);
        // POP, encoding T1: 1011110 P register_list  (A7.7.99)
        } else if instr >> 9 == 0b1011110 {
            self.pop(instr, pc, bus);
        // BX, encoding T1: 010001110 Rm 000  (A7.7.20)
        } else if instr & 0xFF87 == 0x4700 {
            self.bx(instr, pc, bus);
        // MOV (register), encoding T1: 01000110 D Rm Rd  (A7.7.77)
        } else if instr >> 8 == 0b01000110 {
            self.mov_register(instr, pc);
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

    /// STR <Rt>, [<Rn>, <Rm>] -- store the word in Rt to Rn + Rm. The
    /// offset comes from a register, so it can be any 32-bit value,
    /// including a negative one (the addition wraps round).
    fn str_register(&mut self, instr: u16, bus: &mut Bus) {
        let m = ((instr >> 6) & 0x7) as usize; // bits 8-6, offset register
        let n = ((instr >> 3) & 0x7) as usize; // bits 5-3, base register
        let t = (instr & 0x7) as usize; // bits 2-0, value to store

        let address = self.regs[n].wrapping_add(self.regs[m]);

        trace!(
            self,
            "  str r{}, [r{}, r{}]  -> [{:#010X}] = {:#010X}", t, n, m, address, self.regs[t]
        );
        bus.write32(address, self.regs[t]);
    }

    /// MOVS <Rd>, #<imm8> -- load a constant from 0 to 255 into R0-R7,
    /// setting N and Z.
    fn mov_immediate(&mut self, instr: u16) {
        let d = ((instr >> 8) & 0x7) as usize; // bits 10-8
        let imm32 = (instr & 0xFF) as u32; // bits 7-0

        self.regs[d] = imm32;
        // Only N and Z follow the result (N is always 0 here, since the
        // constant has no bit 31). C and V keep their current values.
        let c = self.xpsr & (1 << 29) != 0;
        let v = self.xpsr & (1 << 28) != 0;
        self.set_nzcv(imm32, c, v);
        trace!(
            self,
            "  movs r{}, #{}  -> r{} = {:#010X}", d, imm32, d, imm32
        );
    }

    /// BX <Rm> -- branch to the address in a register. `bx lr` is how a
    /// function that saved nothing on the stack returns, and in an
    /// exception handler it is the exception return.
    /// `pc` is the address of this instruction.
    fn bx(&mut self, instr: u16, pc: u32, bus: &mut Bus) {
        let m = ((instr >> 3) & 0xF) as usize; // bits 6-3
        if m == PC {
            // UNPREDICTABLE in the architecture; a compiler never emits it.
            panic!("BX PC at {:#010X}", pc);
        }
        let target = self.regs[m];
        trace!(self, "  bx r{}  -> {:#010X}", m, target);
        self.bx_write_pc(target, pc, bus);
    }

    /// MOV <Rd>, <Rm> -- copy one register to another. This encoding can
    /// use all 16 registers and never changes the flags.
    /// `pc` is the address of this instruction.
    fn mov_register(&mut self, instr: u16, pc: u32) {
        // Rd is 4 bits: D (bit 7) is its top bit, bits 2-0 the rest.
        let d = (((instr >> 4) & 0x8) | (instr & 0x7)) as usize;
        let m = ((instr >> 3) & 0xF) as usize; // bits 6-3

        // Reading the PC gives the instruction address + 4.
        let value = if m == PC {
            pc.wrapping_add(4)
        } else {
            self.regs[m]
        };

        if d == PC {
            // Writing the PC is a branch. Bit 0 is dropped: instructions
            // are always at even addresses.
            self.regs[PC] = value & !1;
        } else {
            self.regs[d] = value;
        }
        trace!(self, "  mov r{}, r{}  -> r{} = {:#010X}", d, m, d, value);
    }

    /// PUSH <registers> -- store registers on the stack and move SP down.
    /// `pc` is the address of this instruction.
    fn push(&mut self, instr: u16, pc: u32, bus: &mut Bus) {
        // Bits 7-0 select R7-R0; M (bit 8) selects LR. No other register
        // can be pushed by this encoding.
        let m = ((instr >> 8) & 1) as u32;
        let registers = (m << LR) | (instr & 0xFF) as u32;
        if registers == 0 {
            // UNPREDICTABLE in the architecture; a compiler never emits it.
            panic!("PUSH with an empty register list at {:#010X}", pc);
        }

        // The stack grows downwards: make room for all the registers, then
        // fill it upwards, so the lowest-numbered register ends up at the
        // lowest address.
        let start = self.regs[SP].wrapping_sub(4 * registers.count_ones());
        let mut address = start;
        for i in 0..=LR {
            if registers & (1 << i) != 0 {
                bus.write32(address, self.regs[i]);
                address = address.wrapping_add(4);
            }
        }
        self.regs[SP] = start;
        trace!(
            self,
            "  push {{list {:#06X}}}  -> sp = {:#010X}", registers, start
        );
    }

    /// POP <registers> -- load registers from the stack and move SP up.
    /// With PC in the list this is also how a function returns.
    /// `pc` is the address of this instruction.
    fn pop(&mut self, instr: u16, pc: u32, bus: &mut Bus) {
        // Bits 7-0 select R7-R0; P (bit 8) selects the PC. No other
        // register can be popped by this encoding.
        let p = ((instr >> 8) & 1) as u32;
        let registers = (p << PC) | (instr & 0xFF) as u32;
        if registers == 0 {
            // UNPREDICTABLE in the architecture; a compiler never emits it.
            panic!("POP with an empty register list at {:#010X}", pc);
        }

        // The mirror image of PUSH: read upwards from SP, so the
        // lowest-numbered register comes from the lowest address.
        let mut address = self.regs[SP];
        let mut new_pc = None;
        for i in 0..=PC {
            if registers & (1 << i) != 0 {
                let value = bus.read32(address);
                address = address.wrapping_add(4);
                if i == PC {
                    new_pc = Some(value);
                } else {
                    self.regs[i] = value;
                }
            }
        }
        self.regs[SP] = address;
        trace!(
            self,
            "  pop {{list {:#06X}}}  -> sp = {:#010X}", registers, address
        );
        // The PC is written last, after SP has moved: in a handler the
        // value may be EXC_RETURN, and the exception return then pops its
        // frame from the updated SP.
        if let Some(value) = new_pc {
            self.bx_write_pc(value, pc, bus);
        }
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

    /// CMP <Rn>, <Rm> -- subtract Rm from Rn and set the flags, discarding
    /// the result. This encoding only reaches R0-R7.
    fn cmp_register(&mut self, instr: u16) {
        let m = ((instr >> 3) & 0x7) as usize; // bits 5-3
        let n = (instr & 0x7) as usize; // bits 2-0

        // Rn - Rm, computed as Rn + NOT(Rm) + 1
        let (result, carry, overflow) = add_with_carry(self.regs[n], !self.regs[m], true);
        self.set_nzcv(result, carry, overflow);
        trace!(
            self,
            "  cmp r{}, r{}  -> N={} Z={} C={} V={}",
            n,
            m,
            result >> 31,
            (result == 0) as u8,
            carry as u8,
            overflow as u8
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
    use crate::cpu::{cpu_with_code, nzcv};

    #[test]
    fn cmp_of_equal_registers_sets_z_and_c() {
        // cmp r1, r0: the end-of-loop test in the startup code.
        let (mut cpu, mut bus) = cpu_with_code(&[0x4281]);
        cpu.regs[0] = 0x2040_0000;
        cpu.regs[1] = 0x2040_0000;
        cpu.step(&mut bus);
        // Equal: Z set. No borrow was needed, so C is set too.
        assert_eq!(nzcv(&cpu), 0b0110);
        // The registers are only compared, not changed.
        assert_eq!(cpu.regs[0], 0x2040_0000);
        assert_eq!(cpu.regs[1], 0x2040_0000);
        assert_eq!(cpu.regs[PC], 0x0040_000A);
    }

    #[test]
    fn cmp_larger_with_smaller_sets_only_c() {
        let (mut cpu, mut bus) = cpu_with_code(&[0x4281]); // cmp r1, r0
        cpu.regs[1] = 10;
        cpu.regs[0] = 3;
        cpu.step(&mut bus);
        assert_eq!(nzcv(&cpu), 0b0010);
    }

    #[test]
    fn cmp_smaller_with_larger_sets_n_and_clears_c() {
        let (mut cpu, mut bus) = cpu_with_code(&[0x4281]); // cmp r1, r0
        cpu.regs[1] = 3;
        cpu.regs[0] = 10;
        cpu.step(&mut bus);
        // 3 - 10 is negative (N) and needed a borrow (C clear).
        assert_eq!(nzcv(&cpu), 0b1000);
    }

    #[test]
    fn cmp_signed_overflow_sets_v() {
        let (mut cpu, mut bus) = cpu_with_code(&[0x4281]); // cmp r1, r0
        cpu.regs[1] = 0x8000_0000; // most negative signed value
        cpu.regs[0] = 1;
        cpu.step(&mut bus);
        // The result 0x7FFF_FFFF has the wrong sign: V. No borrow: C.
        assert_eq!(nzcv(&cpu), 0b0011);
    }

    #[test]
    fn cmp_register_operand_order() {
        // cmp r3, r5 is Rn = 3, Rm = 5: it computes R3 - R5.
        let (mut cpu, mut bus) = cpu_with_code(&[0x42AB]);
        cpu.regs[3] = 1;
        cpu.regs[5] = 2;
        cpu.step(&mut bus);
        assert_eq!(nzcv(&cpu), 0b1000);
    }

    #[test]
    fn str_register_reaches_a_peripheral_with_a_negative_offset() {
        // str r3, [r0, r2]: hello-rs enabling the UART0 clock. R0 holds
        // the UART0 base and R2 the distance back to PMC_PCER0.
        let (mut cpu, mut bus) = cpu_with_code(&[0x5083]);
        cpu.regs[0] = 0x400E_0800;
        cpu.regs[2] = 0xFFFF_FE10; // -0x1F0
        cpu.regs[3] = 1 << 7; // UART0 is peripheral ID 7
        cpu.step(&mut bus);
        // PMC_PCER0 (0x400E_0610) sets the bit in PMC_PCSR0 (0x400E_0618).
        assert_eq!(bus.read32(0x400E_0618), 1 << 7);
        assert_eq!(cpu.regs[PC], 0x0040_000A);
    }

    #[test]
    fn str_register_adds_base_and_offset() {
        // str r7, [r5, r6]
        let (mut cpu, mut bus) = cpu_with_code(&[0x51AF]);
        cpu.regs[5] = 0x2040_0000;
        cpu.regs[6] = 0x100;
        cpu.regs[7] = 0xCAFE_F00D;
        let xpsr = cpu.xpsr;
        cpu.step(&mut bus);
        assert_eq!(bus.read32(0x2040_0100), 0xCAFE_F00D);
        // Nothing but memory changes.
        assert_eq!(cpu.regs[5], 0x2040_0000);
        assert_eq!(cpu.regs[6], 0x100);
        assert_eq!(cpu.regs[7], 0xCAFE_F00D);
        assert_eq!(cpu.xpsr, xpsr);
    }

    #[test]
    fn movs_loads_zero_and_sets_z() {
        // movs r2, #0: the zero the startup code clears .bss with.
        let (mut cpu, mut bus) = cpu_with_code(&[0x2200]);
        cpu.regs[2] = 0xFFFF_FFFF;
        cpu.step(&mut bus);
        assert_eq!(cpu.regs[2], 0);
        assert_ne!(cpu.xpsr & (1 << 30), 0, "Z should be set");
        assert_eq!(cpu.xpsr & (1 << 31), 0, "N should be clear");
        assert_eq!(cpu.regs[PC], 0x0040_000A);
    }

    #[test]
    fn movs_loads_a_constant_and_clears_n_and_z() {
        // movs r1, #0xAC
        let (mut cpu, mut bus) = cpu_with_code(&[0x21AC]);
        cpu.xpsr |= (1 << 31) | (1 << 30); // N and Z set beforehand
        cpu.step(&mut bus);
        assert_eq!(cpu.regs[1], 0xAC);
        assert_eq!(cpu.xpsr & (0b11 << 30), 0, "N and Z should be clear");
    }

    #[test]
    fn movs_leaves_c_and_v_alone() {
        // movs r7, #255, once with C and V set and once with them clear.
        let (mut cpu, mut bus) = cpu_with_code(&[0x27FF, 0x27FF]);
        cpu.xpsr |= (1 << 29) | (1 << 28);
        cpu.step(&mut bus);
        assert_eq!(cpu.regs[7], 255);
        assert_eq!(cpu.xpsr & (0b11 << 28), 0b11 << 28);
        cpu.xpsr &= !(0b11 << 28);
        cpu.step(&mut bus);
        assert_eq!(cpu.xpsr & (0b11 << 28), 0);
    }

    #[test]
    fn bx_lr_returns_to_the_caller() {
        // bx lr
        let (mut cpu, mut bus) = cpu_with_code(&[0x4770]);
        cpu.regs[LR] = 0x0040_0405; // as left by BL, Thumb bit set
        let xpsr = cpu.xpsr;
        cpu.step(&mut bus);
        // The Thumb bit is dropped from the address that is executed.
        assert_eq!(cpu.regs[PC], 0x0040_0404);
        assert_eq!(cpu.regs[LR], 0x0040_0405);
        assert_eq!(cpu.xpsr, xpsr);
    }

    #[test]
    fn bx_can_branch_through_any_register() {
        // bx r3
        let (mut cpu, mut bus) = cpu_with_code(&[0x4718]);
        cpu.regs[3] = 0x0040_0101;
        cpu.step(&mut bus);
        assert_eq!(cpu.regs[PC], 0x0040_0100);

        // bx r12
        let (mut cpu, mut bus) = cpu_with_code(&[0x4760]);
        cpu.regs[12] = 0x0040_0201;
        cpu.step(&mut bus);
        assert_eq!(cpu.regs[PC], 0x0040_0200);
    }

    #[test]
    #[should_panic(expected = "without the Thumb bit")]
    fn bx_to_an_address_without_the_thumb_bit_stops() {
        let (mut cpu, mut bus) = cpu_with_code(&[0x4770]); // bx lr
        cpu.regs[LR] = 0x0040_0404; // bit 0 clear
        cpu.step(&mut bus);
    }

    #[test]
    fn mov_copies_sp_into_a_low_register() {
        // mov r7, sp: the second instruction of most functions.
        let (mut cpu, mut bus) = cpu_with_code(&[0x466F]);
        cpu.step(&mut bus);
        assert_eq!(cpu.regs[7], 0x2046_0000);
        assert_eq!(cpu.regs[SP], 0x2046_0000);
        assert_eq!(cpu.regs[PC], 0x0040_000A);
    }

    #[test]
    fn mov_can_write_a_high_register() {
        // mov r8, r1: D = 1 makes the destination R8 rather than R0.
        let (mut cpu, mut bus) = cpu_with_code(&[0x4688]);
        cpu.regs[1] = 0x1234_5678;
        cpu.step(&mut bus);
        assert_eq!(cpu.regs[8], 0x1234_5678);
        assert_eq!(cpu.regs[0], 0);
    }

    #[test]
    fn mov_does_not_change_the_flags() {
        // mov r0, r1 with R1 = 0 would set Z if this encoding set flags.
        let (mut cpu, mut bus) = cpu_with_code(&[0x4608]);
        cpu.regs[0] = 5;
        let xpsr = cpu.xpsr;
        cpu.step(&mut bus);
        assert_eq!(cpu.regs[0], 0);
        assert_eq!(cpu.xpsr, xpsr);
    }

    #[test]
    fn mov_from_pc_reads_the_instruction_address_plus_4() {
        // mov r0, pc at 0x0040_0008.
        let (mut cpu, mut bus) = cpu_with_code(&[0x4678]);
        cpu.step(&mut bus);
        assert_eq!(cpu.regs[0], 0x0040_000C);
        assert_eq!(cpu.regs[PC], 0x0040_000A);
    }

    #[test]
    fn mov_to_pc_branches_and_drops_bit_0() {
        // mov pc, r0
        let (mut cpu, mut bus) = cpu_with_code(&[0x4687]);
        cpu.regs[0] = 0x0040_0101;
        cpu.step(&mut bus);
        assert_eq!(cpu.regs[PC], 0x0040_0100);
    }

    #[test]
    fn push_stores_registers_lowest_first_and_moves_sp_down() {
        // push {r7, lr}: the first instruction of most functions.
        let (mut cpu, mut bus) = cpu_with_code(&[0xB580]);
        cpu.regs[7] = 0x1111_1111;
        cpu.regs[LR] = 0x0040_0405;
        cpu.step(&mut bus);
        // SP starts at 0x2046_0000 and drops by 4 per register.
        assert_eq!(cpu.regs[SP], 0x2045_FFF8);
        // R7 is at the lower address, LR above it.
        assert_eq!(bus.read32(0x2045_FFF8), 0x1111_1111);
        assert_eq!(bus.read32(0x2045_FFFC), 0x0040_0405);
        assert_eq!(cpu.regs[PC], 0x0040_000A);
    }

    #[test]
    fn push_of_several_low_registers() {
        // push {r0, r2, r4}: register list 0b0001_0101, M = 0.
        let (mut cpu, mut bus) = cpu_with_code(&[0xB415]);
        cpu.regs[0] = 0xA0;
        cpu.regs[2] = 0xA2;
        cpu.regs[4] = 0xA4;
        cpu.regs[LR] = 0xDEAD_BEEF; // not in the list
        cpu.step(&mut bus);
        assert_eq!(cpu.regs[SP], 0x2045_FFF4);
        assert_eq!(bus.read32(0x2045_FFF4), 0xA0);
        assert_eq!(bus.read32(0x2045_FFF8), 0xA2);
        assert_eq!(bus.read32(0x2045_FFFC), 0xA4);
    }

    #[test]
    fn push_does_not_change_the_pushed_registers_or_flags() {
        let (mut cpu, mut bus) = cpu_with_code(&[0xB580]);
        cpu.regs[7] = 0x1111_1111;
        cpu.regs[LR] = 0x0040_0405;
        let xpsr = cpu.xpsr;
        cpu.step(&mut bus);
        assert_eq!(cpu.regs[7], 0x1111_1111);
        assert_eq!(cpu.regs[LR], 0x0040_0405);
        assert_eq!(cpu.xpsr, xpsr);
    }

    #[test]
    fn pop_loads_registers_lowest_first_and_moves_sp_up() {
        // pop {r0, r2, r4}: register list 0b0001_0101, P = 0.
        let (mut cpu, mut bus) = cpu_with_code(&[0xBC15]);
        cpu.regs[SP] = 0x2045_FFF4;
        bus.write32(0x2045_FFF4, 0xA0);
        bus.write32(0x2045_FFF8, 0xA2);
        bus.write32(0x2045_FFFC, 0xA4);
        let xpsr = cpu.xpsr;
        cpu.step(&mut bus);
        assert_eq!(cpu.regs[0], 0xA0);
        assert_eq!(cpu.regs[2], 0xA2);
        assert_eq!(cpu.regs[4], 0xA4);
        assert_eq!(cpu.regs[SP], 0x2046_0000);
        assert_eq!(cpu.regs[PC], 0x0040_000A);
        assert_eq!(cpu.xpsr, xpsr);
    }

    #[test]
    fn pop_with_pc_returns_from_a_function() {
        // pop {r7, pc}: the last instruction of most functions.
        let (mut cpu, mut bus) = cpu_with_code(&[0xBD80]);
        cpu.regs[SP] = 0x2045_FFF8;
        bus.write32(0x2045_FFF8, 0x1111_1111); // saved R7
        bus.write32(0x2045_FFFC, 0x0040_0405); // saved LR, Thumb bit set
        cpu.step(&mut bus);
        assert_eq!(cpu.regs[7], 0x1111_1111);
        // The Thumb bit is dropped from the address that is executed.
        assert_eq!(cpu.regs[PC], 0x0040_0404);
        assert_eq!(cpu.regs[SP], 0x2046_0000);
    }

    #[test]
    fn push_then_pop_restores_the_registers_and_sp() {
        // push {r4, r5} ; pop {r4, r5}
        let (mut cpu, mut bus) = cpu_with_code(&[0xB430, 0xBC30]);
        cpu.regs[4] = 0x4444;
        cpu.regs[5] = 0x5555;
        cpu.step(&mut bus);
        cpu.regs[4] = 0;
        cpu.regs[5] = 0;
        cpu.step(&mut bus);
        assert_eq!(cpu.regs[4], 0x4444);
        assert_eq!(cpu.regs[5], 0x5555);
        assert_eq!(cpu.regs[SP], 0x2046_0000);
    }

    #[test]
    #[should_panic(expected = "without the Thumb bit")]
    fn pop_to_pc_without_the_thumb_bit_stops() {
        let (mut cpu, mut bus) = cpu_with_code(&[0xBD00]); // pop {pc}
        cpu.regs[SP] = 0x2045_FFFC;
        bus.write32(0x2045_FFFC, 0x0040_0404); // bit 0 clear
        cpu.step(&mut bus);
    }

    #[test]
    fn two_pushes_stack_below_each_other() {
        // push {r0} ; push {r1}
        let (mut cpu, mut bus) = cpu_with_code(&[0xB401, 0xB402]);
        cpu.regs[0] = 0xAAAA;
        cpu.regs[1] = 0xBBBB;
        cpu.step(&mut bus);
        cpu.step(&mut bus);
        assert_eq!(cpu.regs[SP], 0x2045_FFF8);
        assert_eq!(bus.read32(0x2045_FFFC), 0xAAAA);
        assert_eq!(bus.read32(0x2045_FFF8), 0xBBBB);
    }
}
