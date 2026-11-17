//! More instructions, added in one batch when the hard-float firmware
//! needed them: the general data-processing families (AND, ORR, EOR,
//! ADD, SUB, ... with a constant or a shifted register as the second
//! operand), shifts, and a number of loads, stores and small 16-bit
//! instructions.
//!
//! `execute16` and `execute32` try their own instructions first and
//! come here last. Each function returns `None` if the instruction is
//! not one it knows, which leads to the usual "unhandled" panic.
//! Section numbers (A7.7.x) refer to the ARMv7-M Architecture Reference
//! Manual.

use super::{CYCLES_DATA, CYCLES_MEMORY, CYCLES_REFILL, Cpu, PC, SP};
use crate::bus::Bus;
use crate::helpers::{add_with_carry, asr_c, lsl_c, lsr_c, ror_c, rrx_c, thumb_expand_imm_c};

// The four kinds of shift, as encoded in two bits.
const SHIFT_LSL: u32 = 0b00;
const SHIFT_LSR: u32 = 0b01;
const SHIFT_ASR: u32 = 0b10;
const SHIFT_ROR: u32 = 0b11;

/// Shift `value` by `amount` (Shift_C, A7.4.2). Returns the result and
/// the carry out, which is `carry_in` when nothing is shifted.
fn shift_c(value: u32, kind: u32, amount: u32, carry_in: bool) -> (u32, bool) {
    if amount == 0 {
        return (value, carry_in);
    }
    match kind {
        SHIFT_LSL => lsl_c(value, amount),
        SHIFT_LSR => lsr_c(value, amount),
        SHIFT_ASR => asr_c(value, amount),
        _ => ror_c(value, amount),
    }
}

/// A shift by a constant encoded in an instruction (DecodeImmShift then
/// Shift_C). An encoded amount of 0 means 32 for LSR and ASR, and for
/// ROR means "rotate right by one bit through the carry flag" (RRX).
fn shift_by_immediate(value: u32, kind: u32, imm5: u32, carry_in: bool) -> (u32, bool) {
    match (kind, imm5) {
        (SHIFT_LSR, 0) | (SHIFT_ASR, 0) => shift_c(value, kind, 32, carry_in),
        (SHIFT_ROR, 0) => rrx_c(value, carry_in),
        _ => shift_c(value, kind, imm5, carry_in),
    }
}

fn shift_name(kind: u32) -> &'static str {
    ["lsl", "lsr", "asr", "ror"][kind as usize]
}

impl Cpu {
    fn carry(&self) -> bool {
        self.xpsr & (1 << 29) != 0
    }

    fn overflow(&self) -> bool {
        self.xpsr & (1 << 28) != 0
    }

    /// Set N and Z from `result` and C from `carry`, leaving V alone:
    /// what the logical operations and shifts do.
    fn set_nzc(&mut self, result: u32, carry: bool) {
        let v = self.overflow();
        self.set_nzcv(result, carry, v);
    }

    /// The operation shared by the 32-bit data-processing instructions:
    /// combine register `n` with `operand` and put the result in
    /// register `d`. `op` is the 4-bit operation field of the encoding
    /// and `operand_carry` the carry from making the operand, which the
    /// logical operations pass on to the C flag.
    ///
    /// Two special register numbers in the encoding change the meaning:
    /// with Rd = 15 and the S bit, the result is discarded and only the
    /// flags are set (TST, TEQ, CMN, CMP); with Rn = 15, ORR and ORN
    /// have no first operand (MOV, MVN).
    fn data_processing(
        &mut self,
        op: u32,
        setflags: bool,
        n: usize,
        d: usize,
        operand: u32,
        operand_carry: bool,
    ) -> Option<&'static str> {
        let compare_only = d == PC && setflags;
        if d == PC && !setflags {
            return None;
        }
        let rn = if n == PC { 0 } else { self.regs[n] };
        // Only MOV and MVN are allowed to have no first operand.
        if n == PC && op != 0b0010 && op != 0b0011 {
            return None;
        }
        let carry_in = self.carry();
        // (name, result, carry, Some(overflow) for arithmetic)
        let (name, result, carry, overflow) = match op {
            0b0000 => (
                if compare_only { "tst" } else { "and" },
                rn & operand,
                operand_carry,
                None,
            ),
            0b0001 => ("bic", rn & !operand, operand_carry, None),
            0b0010 => (
                if n == PC { "mov" } else { "orr" },
                rn | operand,
                operand_carry,
                None,
            ),
            0b0011 => (
                if n == PC { "mvn" } else { "orn" },
                rn | !operand,
                operand_carry,
                None,
            ),
            0b0100 => (
                if compare_only { "teq" } else { "eor" },
                rn ^ operand,
                operand_carry,
                None,
            ),
            0b1000 => {
                let (r, c, v) = add_with_carry(rn, operand, false);
                (if compare_only { "cmn" } else { "add" }, r, c, Some(v))
            }
            0b1010 => {
                let (r, c, v) = add_with_carry(rn, operand, carry_in);
                ("adc", r, c, Some(v))
            }
            0b1011 => {
                let (r, c, v) = add_with_carry(rn, !operand, carry_in);
                ("sbc", r, c, Some(v))
            }
            0b1101 => {
                let (r, c, v) = add_with_carry(rn, !operand, true);
                (if compare_only { "cmp" } else { "sub" }, r, c, Some(v))
            }
            0b1110 => {
                let (r, c, v) = add_with_carry(!rn, operand, true);
                ("rsb", r, c, Some(v))
            }
            _ => return None,
        };
        // TST, TEQ, CMN and CMP exist only for AND, EOR, ADD and SUB.
        if compare_only && !matches!(op, 0b0000 | 0b0100 | 0b1000 | 0b1101) {
            return None;
        }
        if !compare_only {
            self.regs[d] = result;
        }
        if setflags {
            match overflow {
                Some(v) => self.set_nzcv(result, carry, v),
                None => self.set_nzc(result, carry),
            }
        }
        Some(name)
    }

    /// The 32-bit instructions added in this batch. `None` if the
    /// instruction is not one of them.
    pub(super) fn execute32_more(
        &mut self,
        hw1: u16,
        hw2: u16,
        pc: u32,
        bus: &mut Bus,
    ) -> Option<u32> {
        let (hw1, hw2) = (hw1 as u32, hw2 as u32);
        let n = (hw1 & 0xF) as usize;
        let d = ((hw2 >> 8) & 0xF) as usize;
        let t = ((hw2 >> 12) & 0xF) as usize;
        let m = (hw2 & 0xF) as usize;
        let setflags = hw1 & (1 << 4) != 0;
        // The 12-bit constant field i:imm3:imm8.
        let imm12 = ((hw1 & 0x400) << 1) | ((hw2 >> 4) & 0x700) | (hw2 & 0xFF);

        // Data processing with a "modified immediate" constant:
        // 11110 i 0 op S Rn | 0 imm3 Rd imm8  (A5.3.1)
        if hw1 & 0xFA00 == 0xF000 && hw2 & 0x8000 == 0 {
            let op = (hw1 >> 5) & 0xF;
            let (imm32, carry) = thumb_expand_imm_c(imm12, self.carry());
            let name = self.data_processing(op, setflags, n, d, imm32, carry)?;
            trace!(self, "  {} r{}, r{}, #{:#X}", name, d, n, imm32);
            Some(CYCLES_DATA)
        // ADDW / SUBW: a plain 12-bit constant, never setting flags.
        // 11110 i 100000 Rn | 0 imm3 Rd imm8  (A7.7.3 T4, A7.7.174 T4)
        } else if (hw1 & 0xFBF0 == 0xF200 || hw1 & 0xFBF0 == 0xF2A0)
            && hw2 & 0x8000 == 0
            && n != PC
            && d != PC
        {
            let subtract = hw1 & 0x00A0 == 0x00A0;
            self.regs[d] = if subtract {
                self.regs[n].wrapping_sub(imm12)
            } else {
                self.regs[n].wrapping_add(imm12)
            };
            trace!(
                self,
                "  {}w r{}, r{}, #{}",
                if subtract { "sub" } else { "add" },
                d,
                n,
                imm12
            );
            Some(CYCLES_DATA)
        // Data processing with a register shifted by a constant:
        // 1110101 op S Rn | 0 imm3 Rd imm2 type Rm  (A5.3.11)
        // With op = ORR and Rn = 1111 this is a plain shift: LSL, LSR,
        // ASR or ROR by a constant, or a 32-bit MOV.
        } else if hw1 & 0xFE00 == 0xEA00 && hw2 & 0x8000 == 0 {
            let op = (hw1 >> 5) & 0xF;
            let kind = (hw2 >> 4) & 3;
            let imm5 = ((hw2 >> 10) & 0x1C) | ((hw2 >> 6) & 3); // imm3:imm2
            let (operand, carry) = shift_by_immediate(self.regs[m], kind, imm5, self.carry());
            let name = self.data_processing(op, setflags, n, d, operand, carry)?;
            trace!(
                self,
                "  {} r{}, r{}, r{}, {} #{}",
                name,
                d,
                n,
                m,
                shift_name(kind),
                imm5
            );
            Some(CYCLES_DATA)
        // Shift by the amount in a register:
        // 11111010 0 type S Rn | 1111 Rd 0000 Rm  (A7.7.68 T2 and others)
        } else if hw1 & 0xFF80 == 0xFA00 && hw2 & 0xF0F0 == 0xF000 {
            let kind = (hw1 >> 5) & 3;
            // Only the low byte of the register counts.
            let amount = self.regs[m] & 0xFF;
            let (result, carry) = shift_c(self.regs[n], kind, amount, self.carry());
            self.regs[d] = result;
            if setflags {
                self.set_nzc(result, carry);
            }
            trace!(self, "  {} r{}, r{}, r{}", shift_name(kind), d, n, m);
            Some(CYCLES_DATA)
        // LDR / LDRB / LDRH and STR is elsewhere / STRB / STRH with a
        // 12-bit offset: 1111 1000 1 size L Rn | Rt imm12
        // (A7.7.43 T3, A7.7.46 T2, A7.7.55 T2, A7.7.163 T2, A7.7.170 T2)
        } else if hw1 & 0xFF80 == 0xF880 && n != PC {
            let size = (hw1 >> 5) & 3; // 0 byte, 1 halfword, 2 word
            let load = hw1 & (1 << 4) != 0;
            let address = self.regs[n].wrapping_add(hw2 & 0xFFF);
            if size == 3 || (!load && t == PC) || (load && size != 2 && t == PC) {
                return None; // not a load or store: a hint, or undefined
            }
            if load {
                let value = match size {
                    0 => bus.read8(address) as u32,
                    1 => bus.read16(address) as u32,
                    _ => bus.read32(address),
                };
                trace!(
                    self,
                    "  ldr r{}, [r{}, #{}]  -> {:#010X}",
                    t,
                    n,
                    hw2 & 0xFFF,
                    value
                );
                if t == PC {
                    // Loading the PC is a branch, or an exception return.
                    return Some(CYCLES_MEMORY + self.bx_write_pc(value, pc, bus));
                }
                self.regs[t] = value;
            } else {
                match size {
                    0 => bus.write8(address, self.regs[t] as u8),
                    1 => bus.write16(address, self.regs[t] as u16),
                    _ => bus.write32(address, self.regs[t]),
                }
                trace!(self, "  str r{}, [r{}, #{}]", t, n, hw2 & 0xFFF);
            }
            Some(CYCLES_MEMORY)
        // LDRD / STRD (immediate): two words at once.
        // 1110100 P U 1 W L Rn | Rt Rt2 imm8  (A7.7.50, A7.7.166)
        // With P and W both 0 the encoding means something else.
        } else if hw1 & 0xFE40 == 0xE840 && hw1 & 0x0120 != 0 && n != PC {
            let index = hw1 & (1 << 8) != 0; // P
            let add = hw1 & (1 << 7) != 0; // U
            let writeback = hw1 & (1 << 5) != 0; // W
            let load = hw1 & (1 << 4) != 0;
            let t2 = d; // Rt2 is where other encodings have Rd
            let imm32 = (hw2 & 0xFF) << 2;
            let offset_address = if add {
                self.regs[n].wrapping_add(imm32)
            } else {
                self.regs[n].wrapping_sub(imm32)
            };
            let address = if index { offset_address } else { self.regs[n] };
            if load {
                self.regs[t] = bus.read32(address);
                self.regs[t2] = bus.read32(address.wrapping_add(4));
            } else {
                bus.write32(address, self.regs[t]);
                bus.write32(address.wrapping_add(4), self.regs[t2]);
            }
            if writeback {
                self.regs[n] = offset_address;
            }
            trace!(
                self,
                "  {} r{}, r{}, [{:#010X}]",
                if load { "ldrd" } else { "strd" },
                t,
                t2,
                address
            );
            Some(CYCLES_MEMORY + 1)
        // DSB, DMB, ISB: wait for memory accesses or the pipeline to
        // settle. Nothing to wait for here.
        // 1111001110111111 | 100011110 op 1111  (A7.7.33, A7.7.34, A7.7.37)
        } else if hw1 == 0xF3BF && hw2 & 0xFF00 == 0x8F00 {
            trace!(self, "  dsb / dmb / isb");
            Some(CYCLES_DATA)
        } else {
            None
        }
    }

    /// The 16-bit instructions added in this batch. `None` if the
    /// instruction is not one of them.
    pub(super) fn execute16_more(&mut self, instr: u16, pc: u32, bus: &mut Bus) -> Option<u32> {
        let instr = instr as u32;
        let low = (instr & 0x7) as usize; // bits 2-0
        let mid = ((instr >> 3) & 0x7) as usize; // bits 5-3
        let high = ((instr >> 6) & 0x7) as usize; // bits 8-6
        let reg8 = ((instr >> 8) & 0x7) as usize; // bits 10-8
        let imm8 = instr & 0xFF;
        let setflags = !self.in_it_block;

        // NOP: 1011111100000000  (A7.7.88)
        if instr == 0xBF00 {
            trace!(self, "  nop");
            Some(CYCLES_DATA)
        // ADD / SUB SP, #imm: 10110000 op imm7  (A7.7.5 T2, A7.7.176 T1)
        } else if instr & 0xFF00 == 0xB000 {
            let imm32 = (instr & 0x7F) << 2;
            self.regs[SP] = if instr & 0x80 != 0 {
                self.regs[SP].wrapping_sub(imm32)
            } else {
                self.regs[SP].wrapping_add(imm32)
            };
            trace!(
                self,
                "  add/sub sp, #{}  -> sp = {:#010X}", imm32, self.regs[SP]
            );
            Some(CYCLES_DATA)
        // ADD Rd, SP, #imm: 10101 Rd imm8  (A7.7.5 T1)
        } else if instr >> 11 == 0b10101 {
            self.regs[reg8] = self.regs[SP].wrapping_add(imm8 << 2);
            trace!(self, "  add r{}, sp, #{}", reg8, imm8 << 2);
            Some(CYCLES_DATA)
        // ADR Rd, label: 10100 Rd imm8  (A7.7.7 T1). The PC reads as the
        // instruction's address + 4, word-aligned.
        } else if instr >> 11 == 0b10100 {
            self.regs[reg8] = (pc.wrapping_add(4) & !3).wrapping_add(imm8 << 2);
            trace!(self, "  adr r{}  -> {:#010X}", reg8, self.regs[reg8]);
            Some(CYCLES_DATA)
        // LDR / STR Rt, [SP, #imm]: 1001 L Rt imm8  (A7.7.43 T2, A7.7.161 T2)
        } else if instr >> 12 == 0b1001 {
            let address = self.regs[SP].wrapping_add(imm8 << 2);
            if instr & (1 << 11) != 0 {
                self.regs[reg8] = bus.read32(address);
                trace!(self, "  ldr r{}, [sp, #{}]", reg8, imm8 << 2);
            } else {
                bus.write32(address, self.regs[reg8]);
                trace!(self, "  str r{}, [sp, #{}]", reg8, imm8 << 2);
            }
            Some(CYCLES_MEMORY)
        // CBZ / CBNZ: 1011 op 0 i 1 imm5 Rn  (A7.7.21). Branch forward
        // if the register is (CBZ) or is not (CBNZ) zero. No flags.
        } else if instr & 0xF500 == 0xB100 {
            let nonzero = instr & (1 << 11) != 0;
            let imm32 = ((instr >> 3) & 0x40) | ((instr >> 2) & 0x3E); // i:imm5:'0'
            let taken = (self.regs[low] != 0) == nonzero;
            if taken {
                self.regs[PC] = pc.wrapping_add(4).wrapping_add(imm32);
            }
            trace!(
                self,
                "  cb{}z r{}  -> {}",
                if nonzero { "n" } else { "" },
                low,
                if taken { "taken" } else { "not taken" }
            );
            Some(CYCLES_DATA + if taken { CYCLES_REFILL } else { 0 })
        // LDM Rn!, {registers}: 11001 Rn register_list  (A7.7.41 T1)
        // Rn is updated unless it is one of the registers loaded.
        } else if instr >> 11 == 0b11001 {
            let registers = imm8;
            if registers == 0 {
                return None;
            }
            let mut address = self.regs[reg8];
            for i in 0..8 {
                if registers & (1 << i) != 0 {
                    self.regs[i] = bus.read32(address);
                    address = address.wrapping_add(4);
                }
            }
            if registers & (1 << reg8) == 0 {
                self.regs[reg8] = address;
            }
            trace!(self, "  ldm r{}, {{list {:#04X}}}", reg8, registers);
            Some(CYCLES_DATA + registers.count_ones())
        // Loads and stores with a register offset: 0101 op Rm Rn Rt
        // (A7.7.45, A7.7.48, A7.7.164 and others). STR (000) is handled
        // by `execute16`.
        } else if instr >> 12 == 0b0101 {
            let address = self.regs[mid].wrapping_add(self.regs[high]);
            match (instr >> 9) & 0x7 {
                0b001 => bus.write16(address, self.regs[low] as u16), // strh
                0b010 => bus.write8(address, self.regs[low] as u8),   // strb
                0b011 => self.regs[low] = bus.read8(address) as i8 as u32, // ldrsb
                0b100 => self.regs[low] = bus.read32(address),        // ldr
                0b101 => self.regs[low] = bus.read16(address) as u32, // ldrh
                0b110 => self.regs[low] = bus.read8(address) as u32,  // ldrb
                0b111 => self.regs[low] = bus.read16(address) as i16 as u32, // ldrsh
                _ => return None,
            }
            trace!(self, "  load/store r{}, [r{}, r{}]", low, mid, high);
            Some(CYCLES_MEMORY)
        // STRB Rt, [Rn, #imm5]: 01110 imm5 Rn Rt  (A7.7.163 T1)
        } else if instr >> 11 == 0b01110 {
            let address = self.regs[mid].wrapping_add((instr >> 6) & 0x1F);
            bus.write8(address, self.regs[low] as u8);
            trace!(
                self,
                "  strb r{}, [r{}, #{}]",
                low,
                mid,
                (instr >> 6) & 0x1F
            );
            Some(CYCLES_MEMORY)
        // ADDS / SUBS Rd, Rn, #imm3: 000111 op imm3 Rn Rd, and
        // ADDS Rd, Rn, Rm: 0001100 Rm Rn Rd  (A7.7.3 T1, A7.7.174 T1,
        // A7.7.4 T1). SUBS with a register is handled by `execute16`.
        } else if instr >> 9 == 0b0001110 || instr >> 9 == 0b0001111 || instr >> 9 == 0b0001100 {
            let subtract = instr >> 9 == 0b0001111;
            let operand = if instr >> 9 == 0b0001100 {
                self.regs[high]
            } else {
                high as u32 // the imm3 field
            };
            let (result, carry, overflow) = if subtract {
                add_with_carry(self.regs[mid], !operand, true)
            } else {
                add_with_carry(self.regs[mid], operand, false)
            };
            self.regs[low] = result;
            if setflags {
                self.set_nzcv(result, carry, overflow);
            }
            trace!(
                self,
                "  {} r{}, r{}, {:#X}",
                if subtract { "subs" } else { "adds" },
                low,
                mid,
                operand
            );
            Some(CYCLES_DATA)
        // SUBS Rdn, #imm8: 00111 Rdn imm8  (A7.7.174 T2)
        } else if instr >> 11 == 0b00111 {
            let (result, carry, overflow) = add_with_carry(self.regs[reg8], !imm8, true);
            self.regs[reg8] = result;
            if setflags {
                self.set_nzcv(result, carry, overflow);
            }
            trace!(self, "  subs r{}, #{}", reg8, imm8);
            Some(CYCLES_DATA)
        // MOVS Rd, Rm: 0000000000 Rm Rd  (A7.7.77 T2). The LSL encoding
        // with a shift of 0. Sets N and Z.
        } else if instr >> 6 == 0 {
            self.regs[low] = self.regs[mid];
            if setflags {
                let carry = self.carry();
                self.set_nzc(self.regs[low], carry);
            }
            trace!(self, "  movs r{}, r{}", low, mid);
            Some(CYCLES_DATA)
        // ASRS Rd, Rm, #imm5: 00010 imm5 Rm Rd  (A7.7.10 T1)
        } else if instr >> 11 == 0b00010 {
            let (result, carry) =
                shift_by_immediate(self.regs[mid], SHIFT_ASR, (instr >> 6) & 0x1F, self.carry());
            self.regs[low] = result;
            if setflags {
                self.set_nzc(result, carry);
            }
            trace!(self, "  asrs r{}, r{}, #{}", low, mid, (instr >> 6) & 0x1F);
            Some(CYCLES_DATA)
        // CMP Rn, Rm with high registers: 01000101 N Rm Rn  (A7.7.28 T2)
        } else if instr >> 8 == 0b01000101 {
            let n = (((instr >> 4) & 0x8) | (instr & 0x7)) as usize;
            let m = ((instr >> 3) & 0xF) as usize;
            let (result, carry, overflow) = add_with_carry(self.regs[n], !self.regs[m], true);
            self.set_nzcv(result, carry, overflow);
            trace!(self, "  cmp r{}, r{}", n, m);
            Some(CYCLES_DATA)
        // ADD Rdn, Rm with high registers: 01000100 DN Rm Rdn  (A7.7.4 T2)
        // No flags. Adding to the PC is a branch.
        } else if instr >> 8 == 0b01000100 {
            let dn = (((instr >> 4) & 0x8) | (instr & 0x7)) as usize;
            let m = ((instr >> 3) & 0xF) as usize;
            // As an operand the PC reads as the instruction's address + 4.
            let read = |cpu: &Cpu, r: usize| {
                if r == PC {
                    pc.wrapping_add(4)
                } else {
                    cpu.regs[r]
                }
            };
            let result = read(self, dn).wrapping_add(read(self, m));
            trace!(self, "  add r{}, r{}  -> {:#010X}", dn, m, result);
            if dn == PC {
                self.regs[PC] = result & !1;
                return Some(CYCLES_DATA + CYCLES_REFILL);
            }
            self.regs[dn] = result;
            Some(CYCLES_DATA)
        // Data processing on two low registers: 010000 op Rm Rdn  (A5.2.2)
        } else if instr >> 10 == 0b010000 {
            let op = (instr >> 6) & 0xF;
            let (rdn, rm) = (self.regs[low], self.regs[mid]);
            let carry_in = self.carry();
            // The shifts use only the low byte of Rm as the amount.
            let amount = rm & 0xFF;
            // (result, carry, overflow if arithmetic, keep the result?)
            let (result, carry, overflow, store) = match op {
                0b0000 => (rdn & rm, carry_in, None, true), // ands
                0b0001 => (rdn ^ rm, carry_in, None, true), // eors
                0b0010 | 0b0011 | 0b0100 | 0b0111 => {
                    // lsls, lsrs, asrs, rors
                    let kind = match op {
                        0b0010 => SHIFT_LSL,
                        0b0011 => SHIFT_LSR,
                        0b0100 => SHIFT_ASR,
                        _ => SHIFT_ROR,
                    };
                    let (r, c) = shift_c(rdn, kind, amount, carry_in);
                    (r, c, None, true)
                }
                0b0101 => {
                    let (r, c, v) = add_with_carry(rdn, rm, carry_in); // adcs
                    (r, c, Some(v), true)
                }
                0b0110 => {
                    let (r, c, v) = add_with_carry(rdn, !rm, carry_in); // sbcs
                    (r, c, Some(v), true)
                }
                0b1000 => (rdn & rm, carry_in, None, false), // tst
                0b1001 => {
                    let (r, c, v) = add_with_carry(!rm, 0, true); // rsbs Rd, Rm, #0
                    (r, c, Some(v), true)
                }
                0b1011 => {
                    let (r, c, v) = add_with_carry(rdn, rm, false); // cmn
                    (r, c, Some(v), false)
                }
                0b1100 => (rdn | rm, carry_in, None, true), // orrs
                0b1101 => (rdn.wrapping_mul(rm), carry_in, None, true), // muls
                0b1110 => (rdn & !rm, carry_in, None, true), // bics
                0b1111 => (!rm, carry_in, None, true),      // mvns
                // 1010 is CMP, handled by `execute16`.
                _ => return None,
            };
            if store {
                self.regs[low] = result;
            }
            // TST and CMN set the flags even inside an IT block.
            if setflags || !store {
                match overflow {
                    Some(v) => self.set_nzcv(result, carry, v),
                    None => self.set_nzc(result, carry),
                }
            }
            trace!(
                self,
                "  data op {:04b} r{}, r{}  -> {:#010X}", op, low, mid, result
            );
            Some(CYCLES_DATA)
        // SXTH, SXTB, UXTH: 10110010 op Rm Rd  (A7.7.184, A7.7.182,
        // A7.7.223). Widen a 16-bit or 8-bit value to 32 bits, copying
        // the sign bit (S) or filling with zeros (U). UXTB is handled
        // by `execute16`.
        } else if instr >> 8 == 0b10110010 {
            let value = self.regs[mid];
            self.regs[low] = match (instr >> 6) & 3 {
                0b00 => value as i16 as u32,
                0b01 => value as i8 as u32,
                _ => value & 0xFFFF,
            };
            trace!(
                self,
                "  extend r{}, r{}  -> {:#010X}", low, mid, self.regs[low]
            );
            Some(CYCLES_DATA)
        } else {
            None
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::cpu::{LR, cpu_with_code, nzcv};

    const N: u32 = 0b1000;
    const Z: u32 = 0b0100;
    const C: u32 = 0b0010;
    const V: u32 = 0b0001;

    /// Run one instruction with R0, R1 and R2 preset (and the carry flag
    /// if `carry`), and return the CPU afterwards.
    fn after(code: &[u16], r0: u32, r1: u32, r2: u32, carry: bool) -> Cpu {
        let (mut cpu, mut bus) = cpu_with_code(code);
        cpu.regs[0] = r0;
        cpu.regs[1] = r1;
        cpu.regs[2] = r2;
        if carry {
            cpu.xpsr |= 1 << 29;
        }
        cpu.step(&mut bus);
        cpu
    }

    /// R0 after one instruction.
    fn r0_after(code: &[u16], r0: u32, r1: u32, r2: u32) -> u32 {
        after(code, r0, r1, r2, false).regs[0]
    }

    #[test]
    fn sixteen_bit_logic_on_two_registers() {
        assert_eq!(r0_after(&[0x4008], 0b1100, 0b1010, 0), 0b1000); // ands r0, r1
        assert_eq!(r0_after(&[0x4048], 0b1100, 0b1010, 0), 0b0110); // eors r0, r1
        assert_eq!(r0_after(&[0x4308], 0b1100, 0b1010, 0), 0b1110); // orrs r0, r1
        assert_eq!(r0_after(&[0x4388], 0b1100, 0b1010, 0), 0b0100); // bics r0, r1
        assert_eq!(r0_after(&[0x43C8], 0, 0x0000_FFFF, 0), 0xFFFF_0000); // mvns r0, r1
        assert_eq!(r0_after(&[0x4348], 6, 7, 0), 42); // muls r0, r1, r0
        assert_eq!(r0_after(&[0x4248], 0, 5, 0), -5i32 as u32); // rsbs r0, r1, #0
    }

    #[test]
    fn sixteen_bit_logic_sets_n_and_z_and_keeps_c_and_v() {
        // ands r0, r1 giving 0, with C set beforehand.
        let cpu = after(&[0x4008], 0xF0, 0x0F, 0, true);
        assert_eq!(nzcv(&cpu), Z | C);
        // orrs r0, r1 giving a negative value.
        let cpu = after(&[0x4308], 0x8000_0000, 1, 0, false);
        assert_eq!(nzcv(&cpu), N);
    }

    #[test]
    fn tst_and_cmn_only_set_the_flags() {
        let cpu = after(&[0x4208], 0b0101, 0b1010, 0, false); // tst r0, r1
        assert_eq!(cpu.regs[0], 0b0101);
        assert_eq!(nzcv(&cpu), Z);
        let cpu = after(&[0x42C8], 1, 0xFFFF_FFFF, 0, false); // cmn r0, r1
        assert_eq!(cpu.regs[0], 1);
        assert_eq!(nzcv(&cpu), Z | C); // 1 + (-1) = 0 with a carry out
    }

    #[test]
    fn sixteen_bit_shifts_by_a_register() {
        assert_eq!(r0_after(&[0x4088], 1, 4, 0), 16); // lsls r0, r1
        assert_eq!(r0_after(&[0x40C8], 0x80, 4, 0), 8); // lsrs r0, r1
        assert_eq!(r0_after(&[0x4108], 0x8000_0000, 4, 0), 0xF800_0000); // asrs r0, r1
        assert_eq!(r0_after(&[0x41C8], 0x0000_00F1, 4, 0), 0x1000_000F); // rors r0, r1
        // Only the low byte of the amount counts, and 32 or more
        // shifts everything out.
        assert_eq!(r0_after(&[0x4088], 1, 0x100, 0), 1);
        assert_eq!(r0_after(&[0x4088], 1, 32, 0), 0);
        // The carry is the last bit shifted out.
        let cpu = after(&[0x40C8], 0b1000, 4, 0, false);
        assert_eq!(nzcv(&cpu), Z | C);
    }

    #[test]
    fn add_and_subtract_with_carry() {
        // adcs r0, r1: 5 + 6 + carry
        assert_eq!(after(&[0x4148], 5, 6, 0, true).regs[0], 12);
        assert_eq!(after(&[0x4148], 5, 6, 0, false).regs[0], 11);
        // sbcs r0, r1: 10 - 3 - (1 - carry)
        assert_eq!(after(&[0x4188], 10, 3, 0, true).regs[0], 7);
        assert_eq!(after(&[0x4188], 10, 3, 0, false).regs[0], 6);
        // adcs.w r0, r1, r2 ; sbc.w r0, r1, r2
        assert_eq!(after(&[0xEB51, 0x0002], 0, 5, 6, true).regs[0], 12);
        assert_eq!(after(&[0xEB61, 0x0002], 0, 10, 3, false).regs[0], 6);
    }

    #[test]
    fn small_adds_and_subtracts() {
        assert_eq!(r0_after(&[0x1888], 0, 30, 12), 42); // adds r0, r1, r2
        assert_eq!(r0_after(&[0x1D48], 0, 30, 0), 35); // adds r0, r1, #5
        assert_eq!(r0_after(&[0x38C8], 250, 0, 0), 50); // subs r0, #200
        // subs r0, r3, #1 (from the float firmware): 0 - 1 borrows.
        let (mut cpu, mut bus) = cpu_with_code(&[0x1E58]);
        cpu.step(&mut bus);
        assert_eq!(cpu.regs[0], 0xFFFF_FFFF);
        assert_eq!(nzcv(&cpu), N);
        // asrs r0, r1, #3
        assert_eq!(r0_after(&[0x10C8], 0, -64i32 as u32, 0), -8i32 as u32);
    }

    #[test]
    fn movs_between_low_registers_sets_n_and_z() {
        let cpu = after(&[0x0008], 5, 0, 0, true); // movs r0, r1
        assert_eq!(cpu.regs[0], 0);
        assert_eq!(nzcv(&cpu), Z | C); // C is kept
    }

    #[test]
    fn high_register_add_and_compare() {
        // add r8, r1
        let (mut cpu, mut bus) = cpu_with_code(&[0x4488]);
        cpu.regs[8] = 100;
        cpu.regs[1] = 23;
        cpu.xpsr |= 0xF << 28;
        cpu.step(&mut bus);
        assert_eq!(cpu.regs[8], 123);
        assert_eq!(nzcv(&cpu), 0xF); // no flags changed

        // cmp r0, r12
        let (mut cpu, mut bus) = cpu_with_code(&[0x4560]);
        cpu.regs[0] = 5;
        cpu.regs[12] = 5;
        cpu.step(&mut bus);
        assert_eq!(nzcv(&cpu), Z | C);
    }

    #[test]
    fn widening_a_byte_or_halfword() {
        assert_eq!(r0_after(&[0xB208], 0, 0x1234_8001, 0), 0xFFFF_8001); // sxth r0, r1
        assert_eq!(r0_after(&[0xB248], 0, 0x1234_5680, 0), 0xFFFF_FF80); // sxtb r0, r1
        assert_eq!(r0_after(&[0xB288], 0, 0x1234_8001, 0), 0x0000_8001); // uxth r0, r1
    }

    #[test]
    fn stack_pointer_arithmetic() {
        // sub sp, #4 ; add r7, sp, #8 ; add sp, #4
        let (mut cpu, mut bus) = cpu_with_code(&[0xB081, 0xAF02, 0xB001]);
        cpu.step(&mut bus);
        assert_eq!(cpu.regs[SP], 0x2045_FFFC);
        cpu.step(&mut bus);
        assert_eq!(cpu.regs[7], 0x2046_0004);
        cpu.step(&mut bus);
        assert_eq!(cpu.regs[SP], 0x2046_0000);
    }

    #[test]
    fn loads_and_stores_relative_to_the_stack_pointer() {
        // str r1, [sp] ; ldr r0, [sp, #8]
        let (mut cpu, mut bus) = cpu_with_code(&[0x9100, 0x9802]);
        cpu.regs[SP] = 0x2040_0100;
        cpu.regs[1] = 0xAABB_CCDD;
        bus.write32(0x2040_0108, 0x1122_3344);
        cpu.step(&mut bus);
        assert_eq!(bus.read32(0x2040_0100), 0xAABB_CCDD);
        cpu.step(&mut bus);
        assert_eq!(cpu.regs[0], 0x1122_3344);
    }

    #[test]
    fn cbz_and_cbnz_branch_on_a_register_without_touching_flags() {
        // cbz r0, +0x26 (from the float firmware), at 0x0040_0008.
        let (mut cpu, mut bus) = cpu_with_code(&[0xB198]);
        cpu.xpsr |= 0xF << 28;
        cpu.step(&mut bus);
        assert_eq!(cpu.regs[PC], 0x0040_0008 + 4 + 0x26);
        assert_eq!(nzcv(&cpu), 0xF);
        let (mut cpu, mut bus) = cpu_with_code(&[0xB198]);
        cpu.regs[0] = 1;
        cpu.step(&mut bus);
        assert_eq!(cpu.regs[PC], 0x0040_000A); // not taken

        // cbnz r1, +0 (skips one instruction)
        let (mut cpu, mut bus) = cpu_with_code(&[0xB901]);
        cpu.regs[1] = 1;
        cpu.step(&mut bus);
        assert_eq!(cpu.regs[PC], 0x0040_000C);
    }

    #[test]
    fn ldm_loads_registers_and_advances_the_pointer() {
        // ldm r2!, {r3}: how the startup code copies .data
        let (mut cpu, mut bus) = cpu_with_code(&[0xCA08]);
        cpu.regs[2] = 0x2040_0000;
        bus.write32(0x2040_0000, 0xCAFE_F00D);
        cpu.step(&mut bus);
        assert_eq!(cpu.regs[3], 0xCAFE_F00D);
        assert_eq!(cpu.regs[2], 0x2040_0004);
    }

    #[test]
    fn loads_and_stores_with_a_register_offset() {
        const BASE: u32 = 0x2040_0000;
        // Each: R1 = base, R2 = offset 0x10.
        let load = |code: u16, memory: u32| {
            let (mut cpu, mut bus) = cpu_with_code(&[code]);
            cpu.regs[1] = BASE;
            cpu.regs[2] = 0x10;
            bus.write32(BASE + 0x10, memory);
            cpu.step(&mut bus);
            cpu.regs[0]
        };
        assert_eq!(load(0x5888, 0x8765_4321), 0x8765_4321); // ldr r0, [r1, r2]
        assert_eq!(load(0x5A88, 0x8765_C321), 0x0000_C321); // ldrh
        assert_eq!(load(0x5E88, 0x8765_C321), 0xFFFF_C321); // ldrsh
        assert_eq!(load(0x5688, 0x8765_43A1), 0xFFFF_FFA1); // ldrsb

        // ldrb r2, [r1, r0] (from the float firmware)
        let (mut cpu, mut bus) = cpu_with_code(&[0x5C0A]);
        cpu.regs[1] = BASE;
        cpu.regs[0] = 3;
        bus.write32(BASE, 0xAB00_0000);
        cpu.step(&mut bus);
        assert_eq!(cpu.regs[2], 0xAB);

        let store = |code: u16| {
            let (mut cpu, mut bus) = cpu_with_code(&[code]);
            cpu.regs[0] = 0x1122_3344;
            cpu.regs[1] = BASE;
            cpu.regs[2] = 0x10;
            bus.write32(BASE + 0x10, 0xFFFF_FFFF);
            cpu.step(&mut bus);
            bus.read32(BASE + 0x10)
        };
        assert_eq!(store(0x5288), 0xFFFF_3344); // strh r0, [r1, r2]
        assert_eq!(store(0x5488), 0xFFFF_FF44); // strb r0, [r1, r2]
        // strb r0, [r1, #3]
        let (mut cpu, mut bus) = cpu_with_code(&[0x70C8]);
        cpu.regs[0] = 0x77;
        cpu.regs[1] = BASE;
        cpu.step(&mut bus);
        assert_eq!(bus.read32(BASE), 0x7700_0000);
    }

    #[test]
    fn wide_operations_with_a_constant() {
        assert_eq!(r0_after(&[0xF101, 0x0037], 0, 5, 0), 5 + 0x37); // add.w r0, r1, #0x37
        assert_eq!(r0_after(&[0xF601, 0x70FF], 0, 1, 0), 0x1000); // addw r0, r1, #0xfff
        assert_eq!(r0_after(&[0xF6A1, 0x70FF], 0, 0x1000, 0), 1); // subw r0, r1, #0xfff
        assert_eq!(r0_after(&[0xF1C1, 0x0000], 0, 9, 0), -9i32 as u32); // rsb.w r0, r1, #0
        assert_eq!(r0_after(&[0xF06F, 0x0006], 0, 0, 0), !6); // mvn r0, #6
        assert_eq!(r0_after(&[0xF061, 0x00FF], 0, 0xF000, 0), 0xFFFF_FF00); // orn r0, r1, #0xff
        // and r1, r1, #0xf ; sub.w r12, lr, #1 (from the float firmware)
        assert_eq!(after(&[0xF001, 0x010F], 0, 0xABCD, 0, false).regs[1], 0xD);
        let (mut cpu, mut bus) = cpu_with_code(&[0xF1AE, 0x0C01]);
        cpu.regs[LR] = 10;
        cpu.step(&mut bus);
        assert_eq!(cpu.regs[12], 9);
    }

    #[test]
    fn wide_operations_set_flags_only_with_the_s_bit() {
        // add.w r0, r1, #0x37 leaves the flags alone.
        let cpu = after(&[0xF101, 0x0037], 0, 0xFFFF_FFFF, 0, false);
        assert_eq!(nzcv(&cpu), 0);
        // ands r0, r1, #0x80000000: N from the result, and C from the
        // constant, whose top bit is set.
        let cpu = after(&[0xF011, 0x4000], 0, 0xFFFF_FFFF, 0, false);
        assert_eq!(cpu.regs[0], 0x8000_0000);
        assert_eq!(nzcv(&cpu), N | C);
    }

    #[test]
    fn wide_compare_forms_only_set_the_flags() {
        let cpu = after(&[0xF010, 0x0F01], 2, 0, 0, false); // tst.w r0, #1
        assert_eq!((cpu.regs[0], nzcv(&cpu)), (2, Z));
        let cpu = after(&[0xEA90, 0x0F01], 7, 7, 0, false); // teq.w r0, r1
        assert_eq!((cpu.regs[0], nzcv(&cpu)), (7, Z));
        let cpu = after(&[0xF110, 0x0F01], 0xFFFF_FFFF, 0, 0, false); // cmn.w r0, #1
        assert_eq!(nzcv(&cpu), Z | C);
        let cpu = after(&[0xEBB0, 0x0F41], 10, 5, 0, false); // cmp.w r0, r1, lsl #1
        assert_eq!((cpu.regs[0], nzcv(&cpu)), (10, Z | C));
        // Signed overflow: 0x7FFFFFFF + 1.
        let cpu = after(&[0xF110, 0x0F01], 0x7FFF_FFFF, 0, 0, false);
        assert_eq!(nzcv(&cpu), N | V);
    }

    #[test]
    fn wide_operations_with_a_shifted_register() {
        // add.w r12, r0, r1 (from the float firmware)
        assert_eq!(after(&[0xEB00, 0x0C01], 40, 2, 0, false).regs[12], 42);
        assert_eq!(after(&[0xEA42, 0x0201], 0, 0x0F, 0xF0, false).regs[2], 0xFF); // orr.w r2, r2, r1
        assert_eq!(r0_after(&[0xEA81, 0x00D2], 0, 0xFF, 0xF8), 0xFF ^ 0x1F); // eor.w r0, r1, r2, lsr #3
        assert_eq!(r0_after(&[0xEA21, 0x0002], 0, 0xFF, 0x0F), 0xF0); // bic.w r0, r1, r2
        // mov.w r8, r9
        let (mut cpu, mut bus) = cpu_with_code(&[0xEA4F, 0x0809]);
        cpu.regs[9] = 0x1234;
        cpu.step(&mut bus);
        assert_eq!(cpu.regs[8], 0x1234);
    }

    #[test]
    fn wide_shifts_by_a_constant() {
        // lsl.w r1, r12, #2 (from the float firmware)
        let (mut cpu, mut bus) = cpu_with_code(&[0xEA4F, 0x018C]);
        cpu.regs[12] = 0x11;
        cpu.step(&mut bus);
        assert_eq!(cpu.regs[1], 0x44);

        assert_eq!(r0_after(&[0xEA4F, 0x70E1], 0, 0x8000_0000, 0), 0xFFFF_FFFF); // asr.w r0, r1, #31
        assert_eq!(r0_after(&[0xEA4F, 0x1031], 0, 0x0000_00F1, 0), 0x1000_000F); // ror.w r0, r1, #4
        // rrx r0, r1: the carry flag comes in at the top.
        assert_eq!(
            after(&[0xEA4F, 0x0031], 0, 0b10, 0, true).regs[0],
            0x8000_0001
        );
        assert_eq!(after(&[0xEA4F, 0x0031], 0, 0b10, 0, false).regs[0], 1);
    }

    #[test]
    fn wide_shifts_by_a_register() {
        assert_eq!(r0_after(&[0xFA01, 0xF002], 0, 3, 4), 48); // lsl.w r0, r1, r2
        assert_eq!(
            r0_after(&[0xFA41, 0xF002], 0, -64i32 as u32, 3),
            -8i32 as u32
        ); // asr.w
        // lsr.w r1, r2, r1 (from the float firmware)
        assert_eq!(after(&[0xFA22, 0xF101], 0, 4, 0xF0, false).regs[1], 0xF);
        // lsrs.w r0, r1, r2 sets the flags; the plain form does not.
        let cpu = after(&[0xFA31, 0xF002], 0, 0b1000, 4, false);
        assert_eq!((cpu.regs[0], nzcv(&cpu)), (0, Z | C));
        let cpu = after(&[0xFA22, 0xF101], 0, 4, 0b1000, false);
        assert_eq!(nzcv(&cpu), 0);
    }

    #[test]
    fn wide_loads_and_stores_with_a_12_bit_offset() {
        const BASE: u32 = 0x2040_0000;
        // ldr.w r0, [r12] ; ldrb.w lr, [r0] (from the float firmware)
        let (mut cpu, mut bus) = cpu_with_code(&[0xF8DC, 0x0000, 0xF890, 0xE000]);
        cpu.regs[12] = BASE;
        bus.write32(BASE, BASE + 0x20);
        bus.write32(BASE + 0x20, 0x1234_56AB);
        cpu.step(&mut bus);
        assert_eq!(cpu.regs[0], BASE + 0x20);
        cpu.step(&mut bus);
        assert_eq!(cpu.regs[LR], 0xAB);

        // ldrh.w r0, [r1, #0x100] ; strb.w r0, [r1, #0x200] ;
        // strh.w r0, [r1, #0x300]
        let (mut cpu, mut bus) = cpu_with_code(&[0xF8B1, 0x0100, 0xF881, 0x0200, 0xF8A1, 0x0300]);
        cpu.regs[1] = BASE;
        bus.write32(BASE + 0x100, 0xFFFF_BEEF);
        bus.write32(BASE + 0x200, 0xFFFF_FFFF);
        bus.write32(BASE + 0x300, 0xFFFF_FFFF);
        cpu.step(&mut bus);
        assert_eq!(cpu.regs[0], 0xBEEF);
        cpu.step(&mut bus);
        assert_eq!(bus.read32(BASE + 0x200), 0xFFFF_FFEF);
        cpu.step(&mut bus);
        assert_eq!(bus.read32(BASE + 0x300), 0xFFFF_BEEF);
    }

    #[test]
    fn strd_and_ldrd_move_two_words() {
        const BASE: u32 = 0x2040_0100;
        // strd r0, r1, [sp] (from the float firmware)
        let (mut cpu, mut bus) = cpu_with_code(&[0xE9CD, 0x0100]);
        cpu.regs[SP] = BASE;
        cpu.regs[0] = 0x1111_1111;
        cpu.regs[1] = 0x2222_2222;
        cpu.step(&mut bus);
        assert_eq!(bus.read32(BASE), 0x1111_1111);
        assert_eq!(bus.read32(BASE + 4), 0x2222_2222);

        // ldrd r0, r1, [r2, #8]: offset, no write-back
        let (mut cpu, mut bus) = cpu_with_code(&[0xE9D2, 0x0102]);
        cpu.regs[2] = BASE;
        bus.write32(BASE + 8, 0xAAAA_AAAA);
        bus.write32(BASE + 12, 0xBBBB_BBBB);
        cpu.step(&mut bus);
        assert_eq!((cpu.regs[0], cpu.regs[1]), (0xAAAA_AAAA, 0xBBBB_BBBB));
        assert_eq!(cpu.regs[2], BASE);

        // strd r0, r1, [r2, #-8]!: move the base down first, and keep it
        let (mut cpu, mut bus) = cpu_with_code(&[0xE962, 0x0102]);
        cpu.regs[0] = 1;
        cpu.regs[1] = 2;
        cpu.regs[2] = BASE;
        cpu.step(&mut bus);
        assert_eq!(cpu.regs[2], BASE - 8);
        assert_eq!((bus.read32(BASE - 8), bus.read32(BASE - 4)), (1, 2));

        // ldrd r0, r1, [r2], #8: use the base, then move it up
        let (mut cpu, mut bus) = cpu_with_code(&[0xE8F2, 0x0102]);
        cpu.regs[2] = BASE;
        bus.write32(BASE, 7);
        bus.write32(BASE + 4, 8);
        cpu.step(&mut bus);
        assert_eq!((cpu.regs[0], cpu.regs[1]), (7, 8));
        assert_eq!(cpu.regs[2], BASE + 8);
    }

    #[test]
    fn nop_and_barriers_do_nothing() {
        // nop ; dsb ; isb ; dmb
        let (mut cpu, mut bus) =
            cpu_with_code(&[0xBF00, 0xF3BF, 0x8F4F, 0xF3BF, 0x8F6F, 0xF3BF, 0x8F5F]);
        let regs = cpu.regs;
        for _ in 0..4 {
            cpu.step(&mut bus);
        }
        assert_eq!(cpu.regs[..15], regs[..15]);
        assert_eq!(cpu.regs[PC], 0x0040_0016);
        assert_eq!(cpu.cycles, 4);
    }

    #[test]
    fn sixteen_bit_arithmetic_in_an_it_block_keeps_the_flags() {
        // it ne ; addne r5, #1 (from the float firmware) is in thumb16;
        // here: it eq ; subeq r0, #200 ; it eq ; andeq r0, r1
        let (mut cpu, mut bus) = cpu_with_code(&[0xBF08, 0x38C8, 0xBF08, 0x4008]);
        cpu.xpsr |= 1 << 30; // Z
        cpu.regs[0] = 250;
        cpu.regs[1] = 0;
        let flags = nzcv(&cpu);
        for _ in 0..4 {
            cpu.step(&mut bus);
        }
        assert_eq!(cpu.regs[0], 0);
        assert_eq!(nzcv(&cpu), flags);
    }
}
