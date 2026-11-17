//! 32-bit Thumb instructions (Thumb-2).
//!
//! `execute32` decodes the two halfwords of an instruction and calls the
//! method for it. Every method returns the number of clock cycles the
//! instruction takes (see the `CYCLES_` constants in `cpu.rs`). Section numbers (A7.7.x) refer to the ARMv7-M
//! Architecture Reference Manual.

use super::{CYCLES_DATA, CYCLES_MEMORY, CYCLES_REFILL, Cpu, LR, PC, SP};
use crate::bus::Bus;
use crate::helpers::{add_with_carry, thumb_expand_imm_c};

impl Cpu {
    /// Decode and execute a 32-bit instruction. `hw1` is the halfword at
    /// `pc` and `hw2` the one at `pc + 2`. The panic at the end is the
    /// to-do list for 32-bit encodings.
    /// POP.W <registers> -- the 32-bit POP, which can load any of R0-R12,
    /// LR and the PC. The 16-bit one is limited to R0-R7 and the PC.
    fn pop_wide(&mut self, hw2: u16, pc: u32, bus: &mut Bus) -> u32 {
        // Bit n of hw2 selects register n: P (bit 15) is the PC and M
        // (bit 14) is LR. Bit 13 would be SP, which cannot be popped.
        let registers = hw2 as u32;
        if registers.count_ones() < 2 || registers >> 14 == 0b11 {
            // UNPREDICTABLE in the architecture; a compiler never emits it.
            panic!("POP.W with a bad register list at {:#010X}", pc);
        }
        self.pop_registers(registers, pc, bus)
    }

    pub(super) fn execute32(&mut self, hw1: u16, hw2: u16, pc: u32, bus: &mut Bus) -> u32 {
        // BL, encoding T1: 11110 S imm10 | 11 J1 1 J2 imm11  (A7.7.18)
        if hw1 >> 11 == 0b11110 && hw2 & 0xD000 == 0xD000 {
            self.bl(hw1, hw2, pc)
        // MOV (immediate), encoding T3 (MOVW):
        // 11110 i 100100 imm4 | 0 imm3 Rd imm8  (A7.7.76)
        } else if hw1 & 0xFBF0 == 0xF240 && hw2 & 0x8000 == 0 {
            self.movw(hw1, hw2, pc)
        // MOVT, encoding T1: 11110 i 101100 imm4 | 0 imm3 Rd imm8  (A7.7.79)
        } else if hw1 & 0xFBF0 == 0xF2C0 && hw2 & 0x8000 == 0 {
            self.movt(hw1, hw2, pc)
        // MOV (immediate), encoding T2 (MOV.W):
        // 11110 i 00010 S 1111 | 0 imm3 Rd imm8  (A7.7.76)
        // With anything other than 1111 in the low four bits of hw1 this
        // is ORR (immediate) instead.
        } else if hw1 & 0xFBEF == 0xF04F && hw2 & 0x8000 == 0 {
            self.mov_modified_immediate(hw1, hw2, pc)
        // LDRB (register), encoding T2:
        // 111110000001 Rn | Rt 000000 imm2 Rm  (A7.7.48)
        // Rn = 1111 is LDRB (literal) and Rt = 1111 is PLD, so both are
        // left to the panic below.
        } else if hw1 & 0xFFF0 == 0xF810
            && hw2 & 0x0FC0 == 0
            && hw1 & 0xF != 0xF
            && hw2 >> 12 != 0xF
        {
            self.ldrb_register_wide(hw1, hw2, pc, bus)
        // BIC (immediate), encoding T1:
        // 11110 i 00001 S Rn | 0 imm3 Rd imm8  (A7.7.15)
        } else if hw1 & 0xFBE0 == 0xF020 && hw2 & 0x8000 == 0 {
            self.bic_immediate(hw1, hw2, pc)
        // CMP (immediate), encoding T2:
        // 11110 i 011011 Rn | 0 imm3 1111 imm8  (A7.7.27)
        // With anything other than 1111 in bits 11-8 of hw2 this is SUBS
        // (immediate) instead.
        } else if hw1 & 0xFBF0 == 0xF1B0 && hw2 & 0x8F00 == 0x0F00 {
            self.cmp_modified_immediate(hw1, hw2, pc)
        // POP, encoding T2: 1110100010111101 | P M 0 register_list  (A7.7.99)
        } else if hw1 == 0xE8BD && hw2 & 0x2000 == 0 {
            self.pop_wide(hw2, pc, bus)
        // STR (immediate), encoding T3: 111110001100 Rn | Rt imm12  (A7.7.161)
        // Rn = 1111 is not a valid STR, so it is left to the panic below.
        } else if hw1 & 0xFFF0 == 0xF8C0 && hw1 & 0xF != 0xF {
            self.str_immediate_wide(hw1, hw2, pc, bus)
        } else {
            panic!(
                "unhandled 32-bit instruction at {:#010X}: {:#06X} {:#06X} = {:016b} {:016b}",
                pc, hw1, hw2, hw1, hw2
            );
        }
    }

    /// LDRB.W <Rt>, [<Rn>, <Rm>, LSL #<imm2>] -- load one byte from
    /// Rn + (Rm shifted left by 0 to 3) and zero-extend it into Rt.
    /// `pc` is the address of this instruction.
    fn ldrb_register_wide(&mut self, hw1: u16, hw2: u16, pc: u32, bus: &mut Bus) -> u32 {
        let n = (hw1 & 0xF) as usize; // base register
        let t = ((hw2 >> 12) & 0xF) as usize; // destination
        let shift_n = ((hw2 >> 4) & 0x3) as u32; // imm2
        let m = (hw2 & 0xF) as usize; // offset register
        if t == SP || m == SP || m == PC {
            // UNPREDICTABLE in the architecture; a compiler never emits it.
            panic!("LDRB.W with r{} and r{} at {:#010X}", t, m, pc);
        }

        let offset = self.regs[m] << shift_n;
        let address = self.regs[n].wrapping_add(offset);

        self.regs[t] = bus.read8(address) as u32; // u8 -> u32 zero-extends
        trace!(
            self,
            "  ldrb.w r{}, [r{}, r{}, lsl #{}]  -> r{} = {:#04X}",
            t,
            n,
            m,
            shift_n,
            t,
            self.regs[t]
        );
        CYCLES_MEMORY
    }

    /// CMP.W <Rn>, #<const> -- subtract a modified-immediate constant from
    /// Rn and set the flags, discarding the result. The 32-bit form of
    /// CMP: any register, and constants beyond the 0-255 of the 16-bit
    /// form. `pc` is the address of this instruction.
    fn cmp_modified_immediate(&mut self, hw1: u16, hw2: u16, pc: u32) -> u32 {
        let n = (hw1 & 0xF) as usize;
        if n == PC {
            // UNPREDICTABLE in the architecture; a compiler never emits it.
            panic!("CMP.W with the PC at {:#010X}", pc);
        }

        // The 12-bit encoded constant is i:imm3:imm8.
        let i = ((hw1 >> 10) & 1) as u32;
        let imm3 = ((hw2 >> 12) & 0x7) as u32;
        let imm8 = (hw2 & 0xFF) as u32;
        let imm12 = (i << 11) | (imm3 << 8) | imm8;
        // The carry from expanding the constant is not used: the flags
        // all come from the subtraction.
        let (imm32, _) = thumb_expand_imm_c(imm12, false);

        // Rn - imm32, computed as Rn + NOT(imm32) + 1
        let (result, carry, overflow) = add_with_carry(self.regs[n], !imm32, true);
        self.set_nzcv(result, carry, overflow);
        trace!(
            self,
            "  cmp.w r{}, #{:#X}  -> N={} Z={} C={} V={}",
            n,
            imm32,
            result >> 31,
            (result == 0) as u8,
            carry as u8,
            overflow as u8
        );
        CYCLES_DATA
    }

    /// BIC <Rd>, <Rn>, #<const> / BICS -- "bit clear": Rd = Rn AND NOT
    /// const. Every bit that is 1 in the constant is cleared; the rest of
    /// Rn is copied unchanged. The constant is a modified immediate, as
    /// for MOV.W. `pc` is the address of this instruction.
    fn bic_immediate(&mut self, hw1: u16, hw2: u16, pc: u32) -> u32 {
        let n = (hw1 & 0xF) as usize;
        let d = ((hw2 >> 8) & 0xF) as usize;
        if d == SP || d == PC || n == SP || n == PC {
            // UNPREDICTABLE in the architecture; a compiler never emits it.
            panic!("BIC with r{} and r{} at {:#010X}", d, n, pc);
        }
        let setflags = (hw1 >> 4) & 1 != 0; // S bit

        // The 12-bit encoded constant is i:imm3:imm8.
        let i = ((hw1 >> 10) & 1) as u32;
        let imm3 = ((hw2 >> 12) & 0x7) as u32;
        let imm8 = (hw2 & 0xFF) as u32;
        let imm12 = (i << 11) | (imm3 << 8) | imm8;

        let c = self.xpsr & (1 << 29) != 0;
        let (imm32, carry) = thumb_expand_imm_c(imm12, c);

        let result = self.regs[n] & !imm32;
        self.regs[d] = result;
        if setflags {
            // N and Z follow the result, C comes from expanding the
            // constant, V is unchanged.
            let v = self.xpsr & (1 << 28) != 0;
            self.set_nzcv(result, carry, v);
        }
        trace!(
            self,
            "  bic{} r{}, r{}, #{:#X}  -> r{} = {:#010X}",
            if setflags { "s" } else { "" },
            d,
            n,
            imm32,
            d,
            result
        );
        CYCLES_DATA
    }

    /// STR.W <Rt>, [<Rn>, #<imm12>] -- store the word in Rt to Rn + offset.
    /// The 32-bit form of STR: any register, and an offset from 0 to 4095
    /// that need not be a multiple of 4. `pc` is the address of this
    /// instruction.
    fn str_immediate_wide(&mut self, hw1: u16, hw2: u16, pc: u32, bus: &mut Bus) -> u32 {
        let n = (hw1 & 0xF) as usize; // base register
        let t = ((hw2 >> 12) & 0xF) as usize; // value to store
        let imm32 = (hw2 & 0xFFF) as u32;
        if t == PC {
            // UNPREDICTABLE in the architecture; a compiler never emits it.
            panic!("STR.W of the PC at {:#010X}", pc);
        }

        let address = self.regs[n].wrapping_add(imm32);

        trace!(
            self,
            "  str.w r{}, [r{}, #{}]  -> [{:#010X}] = {:#010X}", t, n, imm32, address, self.regs[t]
        );
        bus.write32(address, self.regs[t]);
        CYCLES_MEMORY
    }

    /// MOVW <Rd>, #<imm16> -- load a constant from 0 to 65535 into any
    /// register, clearing the upper half. Never changes the flags.
    /// `pc` is the address of this instruction.
    fn movw(&mut self, hw1: u16, hw2: u16, pc: u32) -> u32 {
        let d = ((hw2 >> 8) & 0xF) as usize;
        if d == SP || d == PC {
            // UNPREDICTABLE in the architecture; a compiler never emits it.
            panic!("MOVW to r{} at {:#010X}", d, pc);
        }

        // The 16-bit constant is stored in four pieces: imm4:i:imm3:imm8.
        let imm4 = (hw1 & 0xF) as u32;
        let i = ((hw1 >> 10) & 1) as u32;
        let imm3 = ((hw2 >> 12) & 0x7) as u32;
        let imm8 = (hw2 & 0xFF) as u32;
        let imm32 = (imm4 << 12) | (i << 11) | (imm3 << 8) | imm8;

        self.regs[d] = imm32;
        trace!(
            self,
            "  movw r{}, #{:#06X}  -> r{} = {:#010X}", d, imm32, d, imm32
        );
        CYCLES_DATA
    }

    /// MOVT <Rd>, #<imm16> -- write a constant into the upper half of a
    /// register, keeping the lower half. Paired with MOVW to build any
    /// 32-bit value. Never changes the flags.
    /// `pc` is the address of this instruction.
    fn movt(&mut self, hw1: u16, hw2: u16, pc: u32) -> u32 {
        let d = ((hw2 >> 8) & 0xF) as usize;
        if d == SP || d == PC {
            // UNPREDICTABLE in the architecture; a compiler never emits it.
            panic!("MOVT to r{} at {:#010X}", d, pc);
        }

        // Same four-piece constant as MOVW: imm4:i:imm3:imm8.
        let imm4 = (hw1 & 0xF) as u32;
        let i = ((hw1 >> 10) & 1) as u32;
        let imm3 = ((hw2 >> 12) & 0x7) as u32;
        let imm8 = (hw2 & 0xFF) as u32;
        let imm16 = (imm4 << 12) | (i << 11) | (imm3 << 8) | imm8;

        self.regs[d] = (imm16 << 16) | (self.regs[d] & 0xFFFF);
        trace!(
            self,
            "  movt r{}, #{:#06X}  -> r{} = {:#010X}", d, imm16, d, self.regs[d]
        );
        CYCLES_DATA
    }

    /// MOV.W <Rd>, #<const> / MOVS.W <Rd>, #<const> -- load a "modified
    /// immediate" constant into any register. The constant is an 8-bit
    /// value either rotated to any position in the word or repeated across
    /// its bytes, which covers values such as 0x8000 that MOVS and its
    /// 0-255 range cannot reach. `pc` is the address of this instruction.
    fn mov_modified_immediate(&mut self, hw1: u16, hw2: u16, pc: u32) -> u32 {
        let d = ((hw2 >> 8) & 0xF) as usize;
        if d == SP || d == PC {
            // UNPREDICTABLE in the architecture; a compiler never emits it.
            panic!("MOV.W to r{} at {:#010X}", d, pc);
        }
        let setflags = (hw1 >> 4) & 1 != 0; // S bit

        // The 12-bit encoded constant is i:imm3:imm8.
        let i = ((hw1 >> 10) & 1) as u32;
        let imm3 = ((hw2 >> 12) & 0x7) as u32;
        let imm8 = (hw2 & 0xFF) as u32;
        let imm12 = (i << 11) | (imm3 << 8) | imm8;

        let c = self.xpsr & (1 << 29) != 0;
        let (imm32, carry) = thumb_expand_imm_c(imm12, c);

        self.regs[d] = imm32;
        if setflags {
            // N and Z follow the result, C comes from expanding the
            // constant, V is unchanged.
            let v = self.xpsr & (1 << 28) != 0;
            self.set_nzcv(imm32, carry, v);
        }
        trace!(
            self,
            "  mov{}.w r{}, #{:#X}  -> r{} = {:#010X}",
            if setflags { "s" } else { "" },
            d,
            imm32,
            d,
            imm32
        );
        CYCLES_DATA
    }

    /// BL <label> -- call a function: save the return address in LR, then
    /// branch. `pc` is the address of this instruction.
    fn bl(&mut self, hw1: u16, hw2: u16, pc: u32) -> u32 {
        let s = ((hw1 >> 10) & 1) as u32; // sign bit
        let imm10 = (hw1 & 0x3FF) as u32;
        let j1 = ((hw2 >> 13) & 1) as u32;
        let j2 = ((hw2 >> 11) & 1) as u32;
        let imm11 = (hw2 & 0x7FF) as u32;

        // The two bits below the sign are stored inverted and mixed with
        // the sign: I1 = NOT(J1 EOR S), I2 = NOT(J2 EOR S).
        let i1 = !(j1 ^ s) & 1;
        let i2 = !(j2 ^ s) & 1;

        // S:I1:I2:imm10:imm11:'0', sign-extended from 25 bits.
        let imm25 = (s << 24) | (i1 << 23) | (i2 << 22) | (imm10 << 12) | (imm11 << 1);
        let imm32 = ((imm25 << 7) as i32) >> 7;

        // The PC reads as instruction address + 4, which for this 4-byte
        // instruction is also the address of the next instruction.
        let next = pc.wrapping_add(4);
        let target = next.wrapping_add(imm32 as u32);

        // Bit 0 of the return address is set to mark it as Thumb code;
        // BX LR and POP {PC} expect it.
        self.regs[LR] = next | 1;
        self.regs[PC] = target;
        trace!(
            self,
            "  bl {:#010X}  -> lr = {:#010X}", target, self.regs[LR]
        );
        CYCLES_DATA + CYCLES_REFILL
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::cpu::cpu_with_code;

    #[test]
    fn movw_loads_a_16_bit_constant() {
        // movw r1, #0xe04: the first instruction of main in hello-rs.
        let (mut cpu, mut bus) = cpu_with_code(&[0xF640, 0x6104]);
        cpu.step(&mut bus);
        assert_eq!(cpu.regs[1], 0x0E04);
        assert_eq!(cpu.regs[PC], 0x0040_000C);
    }

    #[test]
    fn movw_can_write_a_high_register() {
        // movw r12, #0x4c4
        let (mut cpu, mut bus) = cpu_with_code(&[0xF240, 0x4CC4]);
        cpu.step(&mut bus);
        assert_eq!(cpu.regs[12], 0x04C4);
    }

    #[test]
    fn movw_clears_the_upper_half_and_leaves_the_flags() {
        // movw r0, #0xffff: every bit of all four constant pieces set.
        let (mut cpu, mut bus) = cpu_with_code(&[0xF64F, 0x70FF]);
        cpu.regs[0] = 0xDEAD_BEEF;
        cpu.xpsr |= 0xF << 28;
        cpu.step(&mut bus);
        assert_eq!(cpu.regs[0], 0x0000_FFFF);
        assert_eq!(cpu.xpsr >> 28, 0xF);
    }

    #[test]
    fn movw_of_zero_does_not_set_z() {
        // movw r0, #0
        let (mut cpu, mut bus) = cpu_with_code(&[0xF240, 0x0000]);
        cpu.regs[0] = 1;
        cpu.step(&mut bus);
        assert_eq!(cpu.regs[0], 0);
        assert_eq!(cpu.xpsr & (1 << 30), 0);
    }

    #[test]
    fn movw_then_movt_builds_a_32_bit_address() {
        // movw r1, #0xe04 ; movt r1, #0x400e: the address of PIOA's
        // PIO_PDR register, as hello-rs builds it.
        let (mut cpu, mut bus) = cpu_with_code(&[0xF640, 0x6104, 0xF2C4, 0x010E]);
        cpu.step(&mut bus);
        cpu.step(&mut bus);
        assert_eq!(cpu.regs[1], 0x400E_0E04);
        assert_eq!(cpu.regs[PC], 0x0040_0010);
    }

    #[test]
    fn movt_replaces_only_the_upper_half() {
        // movt r12, #0x40
        let (mut cpu, mut bus) = cpu_with_code(&[0xF2C0, 0x0C40]);
        cpu.regs[12] = 0xDEAD_04C4;
        cpu.step(&mut bus);
        assert_eq!(cpu.regs[12], 0x0040_04C4);
    }

    #[test]
    fn movt_decodes_all_four_constant_pieces_and_leaves_the_flags() {
        // movt r2, #0xffff
        let (mut cpu, mut bus) = cpu_with_code(&[0xF6CF, 0x72FF]);
        cpu.regs[2] = 0x0000_FE10;
        cpu.xpsr |= 0x5 << 28;
        let xpsr = cpu.xpsr;
        cpu.step(&mut bus);
        assert_eq!(cpu.regs[2], 0xFFFF_FE10);
        assert_eq!(cpu.xpsr, xpsr);
    }

    #[test]
    fn mov_w_loads_a_rotated_constant() {
        // mov.w r0, #0x8000: the watchdog-disable value in hello-rs.
        let (mut cpu, mut bus) = cpu_with_code(&[0xF44F, 0x4000]);
        cpu.step(&mut bus);
        assert_eq!(cpu.regs[0], 0x8000);
        assert_eq!(cpu.regs[PC], 0x0040_000C);
    }

    #[test]
    fn mov_w_loads_a_repeated_byte_constant() {
        // mov.w r3, #0x00ff00ff
        let (mut cpu, mut bus) = cpu_with_code(&[0xF04F, 0x13FF]);
        cpu.step(&mut bus);
        assert_eq!(cpu.regs[3], 0x00FF_00FF);
    }

    #[test]
    fn mov_w_without_s_leaves_the_flags() {
        // mov.w r0, #0 would set Z if it set flags.
        let (mut cpu, mut bus) = cpu_with_code(&[0xF04F, 0x0000]);
        cpu.regs[0] = 1;
        let xpsr = cpu.xpsr;
        cpu.step(&mut bus);
        assert_eq!(cpu.regs[0], 0);
        assert_eq!(cpu.xpsr, xpsr);
    }

    #[test]
    fn movs_w_sets_n_and_takes_c_from_a_rotated_constant() {
        // movs.w r0, #0x80000000: a rotated constant sets C to its bit 31.
        let (mut cpu, mut bus) = cpu_with_code(&[0xF05F, 0x4000]);
        cpu.xpsr |= 1 << 28; // V set beforehand
        cpu.step(&mut bus);
        assert_eq!(cpu.regs[0], 0x8000_0000);
        // N set, Z clear, C set, V unchanged.
        assert_eq!(cpu.xpsr >> 28, 0b1011);
    }

    #[test]
    fn movs_w_sets_z_and_keeps_c_for_an_unrotated_constant() {
        // movs.w r0, #0: no rotation, so C keeps its old value.
        let (mut cpu, mut bus) = cpu_with_code(&[0xF05F, 0x0000]);
        cpu.regs[0] = 1;
        cpu.xpsr |= 1 << 29; // C set beforehand
        cpu.step(&mut bus);
        assert_eq!(cpu.regs[0], 0);
        assert_eq!(cpu.xpsr >> 28, 0b0110);
    }

    #[test]
    fn ldrb_w_loads_a_byte_at_base_plus_index() {
        // ldrb.w r3, [r12, r1]: hello-rs fetching one character of its
        // message. R12 points at the text and R1 counts through it.
        let (mut cpu, mut bus) = cpu_with_code(&[0xF81C, 0x3001]);
        bus.write8(0x2040_0005, b'H');
        cpu.regs[12] = 0x2040_0000;
        cpu.regs[1] = 5;
        cpu.step(&mut bus);
        assert_eq!(cpu.regs[3], b'H' as u32);
        assert_eq!(cpu.regs[PC], 0x0040_000C);
    }

    #[test]
    fn ldrb_w_zero_extends_and_leaves_the_flags() {
        // A byte with its top bit set must not turn into a negative word.
        let (mut cpu, mut bus) = cpu_with_code(&[0xF81C, 0x3001]);
        bus.write32(0x2040_0000, 0x1234_56F0);
        cpu.regs[12] = 0x2040_0000;
        cpu.regs[3] = 0xFFFF_FFFF;
        let xpsr = cpu.xpsr;
        cpu.step(&mut bus);
        assert_eq!(cpu.regs[3], 0xF0);
        assert_eq!(cpu.xpsr, xpsr);
    }

    #[test]
    fn ldrb_w_shifts_the_index_register() {
        // ldrb.w r0, [r1, r2, lsl #3]: offset is R2 * 8.
        let (mut cpu, mut bus) = cpu_with_code(&[0xF811, 0x0032]);
        bus.write8(0x2040_0010, 0xAB);
        cpu.regs[1] = 0x2040_0000;
        cpu.regs[2] = 2;
        cpu.step(&mut bus);
        assert_eq!(cpu.regs[0], 0xAB);
        // The index register itself is not changed.
        assert_eq!(cpu.regs[2], 2);
    }

    #[test]
    fn bic_clears_the_bits_set_in_the_constant() {
        // bic r2, r2, #0x600: hello-rs clearing the UART0 pin bits (9 and
        // 10) in the value read from PIO_ABCDSR.
        let (mut cpu, mut bus) = cpu_with_code(&[0xF422, 0x62C0]);
        cpu.regs[2] = 0xFFFF_FFFF;
        cpu.step(&mut bus);
        assert_eq!(cpu.regs[2], 0xFFFF_F9FF);
        assert_eq!(cpu.regs[PC], 0x0040_000C);
    }

    #[test]
    fn bic_can_use_different_source_and_destination_registers() {
        // bic r4, r9, #0xff00ff00
        let (mut cpu, mut bus) = cpu_with_code(&[0xF029, 0x24FF]);
        cpu.regs[9] = 0x1234_5678;
        cpu.step(&mut bus);
        assert_eq!(cpu.regs[4], 0x0034_0078);
        // The source register is not changed.
        assert_eq!(cpu.regs[9], 0x1234_5678);
    }

    #[test]
    fn bic_without_s_leaves_the_flags() {
        // bic r0, r1, #0xff with a zero result would set Z if it set flags.
        let (mut cpu, mut bus) = cpu_with_code(&[0xF021, 0x00FF]);
        cpu.regs[0] = 1;
        cpu.regs[1] = 0xFF;
        let xpsr = cpu.xpsr;
        cpu.step(&mut bus);
        assert_eq!(cpu.regs[0], 0);
        assert_eq!(cpu.xpsr, xpsr);
    }

    #[test]
    fn bics_sets_z_and_keeps_c_for_an_unrotated_constant() {
        // bics r0, r1, #0xff
        let (mut cpu, mut bus) = cpu_with_code(&[0xF031, 0x00FF]);
        cpu.regs[1] = 0xFF;
        cpu.xpsr |= (1 << 29) | (1 << 28); // C and V set beforehand
        cpu.step(&mut bus);
        assert_eq!(cpu.regs[0], 0);
        // N clear, Z set, C and V unchanged.
        assert_eq!(cpu.xpsr >> 28, 0b0111);
    }

    #[test]
    fn bics_takes_c_from_a_rotated_constant() {
        // bics r0, r1, #0x80000000: a rotated constant sets C to its bit 31.
        let (mut cpu, mut bus) = cpu_with_code(&[0xF031, 0x4000]);
        cpu.regs[1] = 0xFFFF_FFFF;
        cpu.step(&mut bus);
        assert_eq!(cpu.regs[0], 0x7FFF_FFFF);
        // N clear, Z clear, C set, V clear.
        assert_eq!(cpu.xpsr >> 28, 0b0010);
    }

    #[test]
    fn cmp_w_below_the_constant_clears_c() {
        // cmp.w r2, #0x1f4: delay_ms asking "have 500 ms passed yet?".
        // The `blo` after it loops while C is clear.
        let (mut cpu, mut bus) = cpu_with_code(&[0xF5B2, 0x7FFA]);
        cpu.regs[2] = 499;
        cpu.step(&mut bus);
        // 499 - 500: negative, and a borrow was needed (C clear).
        assert_eq!(cpu.xpsr >> 28, 0b1000);
        // Only the flags change.
        assert_eq!(cpu.regs[2], 499);
        assert_eq!(cpu.regs[PC], 0x0040_000C);
    }

    #[test]
    fn cmp_w_equal_to_the_constant_sets_z_and_c() {
        let (mut cpu, mut bus) = cpu_with_code(&[0xF5B2, 0x7FFA]); // cmp.w r2, #0x1f4
        cpu.regs[2] = 500;
        cpu.step(&mut bus);
        assert_eq!(cpu.xpsr >> 28, 0b0110);
    }

    #[test]
    fn cmp_w_above_the_constant_sets_only_c() {
        let (mut cpu, mut bus) = cpu_with_code(&[0xF5B2, 0x7FFA]); // cmp.w r2, #0x1f4
        cpu.regs[2] = 501;
        cpu.step(&mut bus);
        assert_eq!(cpu.xpsr >> 28, 0b0010);
    }

    #[test]
    fn cmp_w_can_use_a_high_register() {
        // cmp.w r10, #0xff00ff00
        let (mut cpu, mut bus) = cpu_with_code(&[0xF1BA, 0x2FFF]);
        cpu.regs[10] = 0xFF00_FF00;
        cpu.step(&mut bus);
        assert_eq!(cpu.xpsr >> 28, 0b0110);
    }

    #[test]
    fn str_w_writes_a_peripheral_register_at_a_large_offset() {
        // str.w r0, [r1, #0xa50]: hello-rs disabling the watchdog. R1 holds
        // the address of PIOA's PIO_PDR; 0xA50 further on is WDT_MR.
        let (mut cpu, mut bus) = cpu_with_code(&[0xF8C1, 0x0A50]);
        cpu.regs[0] = 0x8000; // WDDIS
        cpu.regs[1] = 0x400E_0E04;
        cpu.step(&mut bus);
        assert_eq!(bus.read32(0x400E_1854), 0x8000);
        assert_eq!(cpu.regs[PC], 0x0040_000C);
    }

    #[test]
    fn str_w_can_use_high_registers_and_the_largest_offsets() {
        // str.w r5, [r8, #0xffc]
        let (mut cpu, mut bus) = cpu_with_code(&[0xF8C8, 0x5FFC]);
        cpu.regs[5] = 0xCAFE_F00D;
        cpu.regs[8] = 0x2040_0000;
        cpu.step(&mut bus);
        assert_eq!(bus.read32(0x2040_0FFC), 0xCAFE_F00D);
    }

    #[test]
    fn str_w_changes_no_register_or_flag() {
        // str.w r0, [r1, #0x10]
        let (mut cpu, mut bus) = cpu_with_code(&[0xF8C1, 0x0010]);
        cpu.regs[0] = 0x1234_5678;
        cpu.regs[1] = 0x2040_0000;
        let xpsr = cpu.xpsr;
        cpu.step(&mut bus);
        assert_eq!(bus.read32(0x2040_0010), 0x1234_5678);
        assert_eq!(cpu.regs[0], 0x1234_5678);
        assert_eq!(cpu.regs[1], 0x2040_0000);
        assert_eq!(cpu.xpsr, xpsr);
    }

    #[test]
    fn bl_branches_forward_and_saves_the_return_address() {
        // The first instruction of hello-rs: at 0x00400400 it calls
        // 0x004004B8, an offset of +0xB4 from the next instruction.
        let (mut cpu, mut bus) = cpu_with_code(&[0xF000, 0xF85A]);
        cpu.step(&mut bus);
        assert_eq!(cpu.regs[PC], 0x0040_000C + 0xB4);
        // Return address is the next instruction, with the Thumb bit set.
        assert_eq!(cpu.regs[LR], 0x0040_000D);
    }

    #[test]
    fn bl_branches_backward() {
        // Offset -0x76: S = 1, J1 = J2 = 1, imm10 = 0x3FF, imm11 = 0x7C5.
        let (mut cpu, mut bus) = cpu_with_code(&[0xF7FF, 0xFFC5]);
        cpu.step(&mut bus);
        assert_eq!(cpu.regs[PC], 0x0040_000C - 0x76);
        assert_eq!(cpu.regs[LR], 0x0040_000D);
    }

    #[test]
    fn bl_decodes_the_inverted_high_offset_bits() {
        // Offset +0x800000 sets I1 only. With S = 0 that is stored as
        // J1 = 0, J2 = 1.
        let (mut cpu, mut bus) = cpu_with_code(&[0xF000, 0xD800]);
        cpu.step(&mut bus);
        assert_eq!(cpu.regs[PC], 0x0040_000C + 0x80_0000);
    }

    #[test]
    fn bl_does_not_change_the_flags_or_other_registers() {
        let (mut cpu, mut bus) = cpu_with_code(&[0xF000, 0xF85A]);
        let xpsr = cpu.xpsr;
        let sp = cpu.regs[SP];
        cpu.step(&mut bus);
        assert_eq!(cpu.xpsr, xpsr);
        assert_eq!(cpu.regs[SP], sp);
        assert_eq!(cpu.regs[..13], [0; 13]);
    }

    #[test]
    fn pop_wide_loads_a_high_register_and_lr() {
        // pop.w {r7, lr}: how hello-rs's UART0 handler restores LR.
        let (mut cpu, mut bus) = cpu_with_code(&[0xE8BD, 0x4080]);
        cpu.regs[SP] = 0x2045_FFF0;
        bus.write32(0x2045_FFF0, 0x1111_1111);
        bus.write32(0x2045_FFF4, 0xFFFF_FFF9);
        cpu.step(&mut bus);
        assert_eq!(cpu.regs[7], 0x1111_1111);
        assert_eq!(cpu.regs[LR], 0xFFFF_FFF9);
        assert_eq!(cpu.regs[SP], 0x2045_FFF8);
        assert_eq!(cpu.regs[PC], 0x0040_000C);
    }

    #[test]
    fn pop_wide_with_pc_returns_from_a_function() {
        // pop.w {r4, r8, pc}
        let (mut cpu, mut bus) = cpu_with_code(&[0xE8BD, 0x8110]);
        cpu.regs[SP] = 0x2045_FFF0;
        bus.write32(0x2045_FFF0, 4);
        bus.write32(0x2045_FFF4, 8);
        bus.write32(0x2045_FFF8, 0x0040_0101);
        cpu.step(&mut bus);
        assert_eq!(cpu.regs[4], 4);
        assert_eq!(cpu.regs[8], 8);
        assert_eq!(cpu.regs[PC], 0x0040_0100);
        assert_eq!(cpu.regs[SP], 0x2045_FFFC);
    }
}
