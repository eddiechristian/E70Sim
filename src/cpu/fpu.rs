//! The floating-point unit (FPU): the Cortex-M7's FPv5 extension with
//! single and double precision. Section numbers refer to the ARMv7-M
//! Architecture Reference Manual; chapter A6 has the encoding tables.
//!
//! Registers:
//!
//! - S0-S31, 32 bits each, hold single-precision (`f32`) values.
//! - D0-D15, 64 bits each, hold double-precision (`f64`) values. They
//!   are the same storage: D0 is S1:S0, D1 is S3:S2, and so on.
//! - FPSCR holds the comparison flags N, Z, C, V in bits 31-28 and the
//!   rounding mode in bits 23-22.
//!
//! The FPU is off after reset. Firmware turns it on by setting bits
//! 20-23 of CPACR (0xE000_ED88); until then every FPU instruction is a
//! UsageFault (NOCP, "no coprocessor").
//!
//! The arithmetic is done with the host's `f32` and `f64`, which follow
//! the same IEEE 754 standard. Differences from the real FPU:
//!
//! - Arithmetic always rounds to nearest. The rounding mode in FPSCR is
//!   only used when converting to an integer with VCVTR.
//! - The cumulative exception flags in FPSCR (invalid operation, divide
//!   by zero, overflow, ...) are never set.
//! - The "default NaN" and "flush to zero" modes are ignored, and which
//!   NaN comes out of an operation on NaNs follows the host.

use std::ops::{Add, Div, Mul, Neg, Sub};

use super::exception::CONTROL_FPCA;
use super::fault::Fault;
use super::{CYCLES_DATA, CYCLES_MEMORY, Cpu, PC};
use crate::bus::Bus;
use crate::helpers::condition_passed;

// System Control Space registers for the FPU.
const CPACR: u32 = 0xE000_ED88; // Coprocessor Access Control
pub(super) const FPCCR: u32 = 0xE000_EF34; // FP Context Control
pub(super) const FPCAR: u32 = 0xE000_EF38; // FP Context Address
const FPDSCR: u32 = 0xE000_EF3C; // FP Default Status Control

// FPCCR bits.
pub(super) const FPCCR_LSPACT: u32 = 1 << 0; // a lazy save is outstanding
pub(super) const FPCCR_USER: u32 = 1 << 1; // ... made from unprivileged code
pub(super) const FPCCR_THREAD: u32 = 1 << 3; // ... made from Thread mode
pub(super) const FPCCR_HFRDY: u32 = 1 << 4; // ... when HardFault could be taken
pub(super) const FPCCR_LSPEN: u32 = 1 << 30; // save FP registers lazily
const FPCCR_ASPEN: u32 = 1 << 31; // track FP use in CONTROL.FPCA

/// The FPSCR bits a new FP context takes from FPDSCR: AHP, DN, FZ and
/// the rounding mode.
const FPSCR_DEFAULTED: u32 = 0x1F << 22;

/// The FPSCR bits that exist: flags, the mode bits above, and the
/// cumulative exception flags.
const FPSCR_MASK: u32 = 0xF7C0_009F;

/// Divide and square root take many cycles.
const CYCLES_FP_SLOW: u32 = 14;

/// A floating-point type the FPU can work in: `f32` in the S registers
/// or `f64` in the D registers.
trait Fp:
    Copy
    + PartialOrd
    + Add<Output = Self>
    + Sub<Output = Self>
    + Mul<Output = Self>
    + Div<Output = Self>
    + Neg<Output = Self>
{
    /// Register number from a 4-bit field and its extra bit (A6.3). For
    /// S registers the extra bit is the lowest; for D, the highest.
    fn index(field: u32, extra: u32) -> usize;
    fn load(cpu: &Cpu, index: usize) -> Self;
    fn store(self, cpu: &mut Cpu, index: usize);
    fn sqrt(self) -> Self;
    fn abs(self) -> Self;
    /// self * a + b with a single rounding (a "fused" multiply-add).
    fn mul_add(self, a: Self, b: Self) -> Self;
    fn to_f64(self) -> f64;
    fn from_f64(value: f64) -> Self;
    fn is_nan(self) -> bool;
    /// True for negative numbers and for -0.0.
    fn is_sign_negative(self) -> bool;
    /// VFPExpandImm (A6.4.1): the constant an 8-bit immediate stands for.
    fn expand_imm(imm8: u32) -> Self;
    const ZERO: Self;
    /// "s" or "d", for the trace.
    const PREFIX: char;
}

impl Fp for f32 {
    fn index(field: u32, extra: u32) -> usize {
        ((field << 1) | extra) as usize
    }
    fn load(cpu: &Cpu, index: usize) -> Self {
        f32::from_bits(cpu.fp[index])
    }
    fn store(self, cpu: &mut Cpu, index: usize) {
        cpu.fp[index] = self.to_bits();
    }
    fn sqrt(self) -> Self {
        f32::sqrt(self)
    }
    fn abs(self) -> Self {
        f32::abs(self)
    }
    fn mul_add(self, a: Self, b: Self) -> Self {
        f32::mul_add(self, a, b)
    }
    fn to_f64(self) -> f64 {
        self as f64
    }
    fn from_f64(value: f64) -> Self {
        value as f32
    }
    fn is_nan(self) -> bool {
        f32::is_nan(self)
    }
    fn is_sign_negative(self) -> bool {
        f32::is_sign_negative(self)
    }
    fn expand_imm(imm8: u32) -> Self {
        // sign : NOT(b) : bbbbb : cd : efgh : 19 zeros
        let sign = imm8 >> 7;
        let b = (imm8 >> 6) & 1;
        let exp = ((b ^ 1) << 7) | (if b == 1 { 0x1F << 2 } else { 0 }) | ((imm8 >> 4) & 3);
        f32::from_bits((sign << 31) | (exp << 23) | ((imm8 & 0xF) << 19))
    }
    const ZERO: Self = 0.0;
    const PREFIX: char = 's';
}

impl Fp for f64 {
    fn index(field: u32, extra: u32) -> usize {
        ((extra << 4) | field) as usize
    }
    fn load(cpu: &Cpu, index: usize) -> Self {
        f64::from_bits(cpu.d(index))
    }
    fn store(self, cpu: &mut Cpu, index: usize) {
        cpu.set_d(index, self.to_bits());
    }
    fn sqrt(self) -> Self {
        f64::sqrt(self)
    }
    fn abs(self) -> Self {
        f64::abs(self)
    }
    fn mul_add(self, a: Self, b: Self) -> Self {
        f64::mul_add(self, a, b)
    }
    fn to_f64(self) -> f64 {
        self
    }
    fn from_f64(value: f64) -> Self {
        value
    }
    fn is_nan(self) -> bool {
        f64::is_nan(self)
    }
    fn is_sign_negative(self) -> bool {
        f64::is_sign_negative(self)
    }
    fn expand_imm(imm8: u32) -> Self {
        // sign : NOT(b) : bbbbbbbb : cd : efgh : 48 zeros
        let imm8 = imm8 as u64;
        let sign = imm8 >> 7;
        let b = (imm8 >> 6) & 1;
        let exp = ((b ^ 1) << 10) | (if b == 1 { 0xFF << 2 } else { 0 }) | ((imm8 >> 4) & 3);
        f64::from_bits((sign << 63) | (exp << 52) | ((imm8 & 0xF) << 48))
    }
    const ZERO: Self = 0.0;
    const PREFIX: char = 'd';
}

/// The larger (`maximum`) or smaller of two values, the way VMAXNM and
/// VMINNM define it (FPMaxNum / FPMinNum, A2.5.7):
///
/// - If exactly one is a NaN ("not a number"), the result is the other
///   one. A NaN only comes out when both are.
/// - +0.0 counts as larger than -0.0, although they compare equal.
fn max_or_min_number<F: Fp>(a: F, b: F, maximum: bool) -> F {
    if a.is_nan() {
        return b;
    }
    if b.is_nan() {
        return a;
    }
    if a == b {
        // Equal values differ at most in the sign of a zero.
        return if a.is_sign_negative() == maximum {
            b
        } else {
            a
        };
    }
    if (a > b) == maximum { a } else { b }
}

/// Round to an integer value in the direction FPSCR's rounding mode (or
/// a fixed one) says: 0 nearest (ties to even), 1 up, 2 down, 3 toward
/// zero.
fn round_to_integral(value: f64, mode: u32) -> f64 {
    match mode {
        0 => value.round_ties_even(),
        1 => value.ceil(),
        2 => value.floor(),
        _ => value.trunc(),
    }
}

impl Cpu {
    /// D register `index` as 64 bits: two S registers, low half first.
    fn d(&self, index: usize) -> u64 {
        if index >= 16 {
            // The Cortex-M7 has D0-D15 only; a compiler never asks for more.
            panic!("FPU register D{} does not exist", index);
        }
        ((self.fp[2 * index + 1] as u64) << 32) | self.fp[2 * index] as u64
    }

    fn set_d(&mut self, index: usize, value: u64) {
        if index >= 16 {
            panic!("FPU register D{} does not exist", index);
        }
        self.fp[2 * index] = value as u32;
        self.fp[2 * index + 1] = (value >> 32) as u32;
    }

    /// Carry out a lazy save that is still outstanding: write S0-S15 and
    /// FPSCR into the space an earlier exception entry left for them
    /// (PreserveFPState, B1.5.7).
    fn preserve_fp_state(&mut self, bus: &mut Bus) {
        let address = bus.read32(FPCAR);
        for i in 0..16 {
            bus.write32(address + 4 * i, self.fp[i as usize]);
        }
        bus.write32(address + 0x40, self.fpscr);
        let fpccr = bus.read32(FPCCR);
        bus.write32(FPCCR, fpccr & !FPCCR_LSPACT);
    }

    /// What happens before every FPU instruction (ExecuteFPCheck,
    /// B1.5.7). Returns false if the instruction must not execute because
    /// the FPU is not enabled; the fault is then recorded for `step`.
    fn fp_check(&mut self, bus: &mut Bus) -> bool {
        // CPACR bits 21-20 (CP10): 00 = no access, 01 = privileged code
        // only, 11 = everyone.
        let access = (bus.read32(CPACR) >> 20) & 3;
        let allowed = match access {
            0b11 => true,
            0b01 => self.is_privileged(),
            _ => false,
        };
        if !allowed {
            self.fault = Some(Fault::NoCoprocessor);
            return false;
        }
        let fpccr = bus.read32(FPCCR);
        // An interrupted context's FP registers are still waiting to be
        // saved, and this instruction is about to change them.
        if fpccr & FPCCR_LSPACT != 0 {
            self.preserve_fp_state(bus);
        }
        // Note that this context uses the FPU, so that an exception entry
        // knows to make room for its registers.
        if fpccr & FPCCR_ASPEN != 0 && self.control & CONTROL_FPCA == 0 {
            self.fpscr = (self.fpscr & !FPSCR_DEFAULTED) | (bus.read32(FPDSCR) & FPSCR_DEFAULTED);
            self.control |= CONTROL_FPCA;
        }
        true
    }

    /// Decode and execute an FPU instruction: any 32-bit instruction for
    /// coprocessor 10 or 11. `execute32` has checked that much.
    pub(super) fn execute_fp(&mut self, hw1: u16, hw2: u16, pc: u32, bus: &mut Bus) -> u32 {
        if !self.fp_check(bus) {
            return CYCLES_DATA;
        }
        let (hw1, hw2) = (hw1 as u32, hw2 as u32);
        let double = hw2 & (1 << 8) != 0; // sz
        if hw1 >> 8 == 0xFE && hw2 & (1 << 4) == 0 {
            // Data processing added by FPv5 (A6.4, the rows with T = 1).
            if double {
                self.fp_data_processing_v5::<f64>(hw1, hw2, pc)
            } else {
                self.fp_data_processing_v5::<f32>(hw1, hw2, pc)
            }
        } else if hw1 >> 8 == 0xEE && hw2 & (1 << 4) == 0 {
            // Data processing (A6.4).
            if double {
                self.fp_data_processing::<f64>(hw1, hw2, pc)
            } else {
                self.fp_data_processing::<f32>(hw1, hw2, pc)
            }
        } else if hw1 >> 8 == 0xEE {
            self.fp_transfer_32(hw1, hw2, pc)
        } else if hw1 & 0xFFE0 == 0xEC40 {
            self.fp_transfer_64(hw1, hw2, pc)
        } else if hw1 >> 9 == 0b1110110 {
            self.fp_load_store(hw1, hw2, pc, bus)
        } else {
            self.fp_unhandled(hw1, hw2, pc)
        }
    }

    fn fp_unhandled(&self, hw1: u32, hw2: u32, pc: u32) -> ! {
        panic!(
            "unhandled FPU instruction at {:#010X}: {:#06X} {:#06X} = {:016b} {:016b}",
            pc, hw1, hw2, hw1, hw2
        );
    }

    /// The data-processing instructions (table A6-5), in precision `F`:
    /// 11101110 opc1 opc2 | Vd 101 sz opc3 0 Vm, with D, N and M the
    /// extra register bits.
    fn fp_data_processing<F: Fp>(&mut self, hw1: u32, hw2: u32, pc: u32) -> u32 {
        let extra_d = (hw1 >> 6) & 1; // D
        let extra_n = (hw2 >> 7) & 1; // N
        let extra_m = (hw2 >> 5) & 1; // M
        let d = F::index(hw2 >> 12, extra_d);
        let n = F::index(hw1 & 0xF, extra_n);
        let m = F::index(hw2 & 0xF, extra_m);
        let op = hw2 & (1 << 6) != 0; // low bit of opc3
        let p = F::PREFIX;

        // opc1 with the D bit taken out: bits 7, 5 and 4 of hw1.
        let opc1 = ((hw1 >> 5) & 0b100) | ((hw1 >> 4) & 0b011);
        if opc1 != 0b111 {
            let (vd, vn, vm) = (F::load(self, d), F::load(self, n), F::load(self, m));
            let (name, result) = match (opc1, op) {
                // Multiply, then add or subtract, as two separate steps.
                (0b000, false) => ("vmla", vd + vn * vm),
                (0b000, true) => ("vmls", vd - vn * vm),
                (0b001, false) => ("vnmls", -vd + vn * vm),
                (0b001, true) => ("vnmla", -vd - vn * vm),
                (0b010, false) => ("vmul", vn * vm),
                (0b010, true) => ("vnmul", -(vn * vm)),
                (0b011, false) => ("vadd", vn + vm),
                (0b011, true) => ("vsub", vn - vm),
                (0b100, false) => ("vdiv", vn / vm),
                // Fused: the product is not rounded before the addition.
                (0b101, false) => ("vfnms", vn.mul_add(vm, -vd)),
                (0b101, true) => ("vfnma", (-vn).mul_add(vm, -vd)),
                (0b110, false) => ("vfma", vn.mul_add(vm, vd)),
                (0b110, true) => ("vfms", (-vn).mul_add(vm, vd)),
                _ => self.fp_unhandled(hw1, hw2, pc),
            };
            result.store(self, d);
            trace!(self, "  {} {}{}, {}{}, {}{}", name, p, d, p, n, p, m);
            return if opc1 == 0b100 {
                CYCLES_FP_SLOW
            } else {
                CYCLES_DATA
            };
        }

        // opc1 = 1x11: everything else.
        if !op {
            // VMOV (immediate): the constant is in opc2 and opc4.
            let imm8 = ((hw1 & 0xF) << 4) | (hw2 & 0xF);
            F::expand_imm(imm8).store(self, d);
            trace!(self, "  vmov {}{}, #imm {:#04X}", p, d, imm8);
            return CYCLES_DATA;
        }
        let opc2 = hw1 & 0xF;
        let high = hw2 & (1 << 7) != 0; // high bit of opc3
        let vm = F::load(self, m);
        match (opc2, high) {
            (0b0000, false) => {
                vm.store(self, d);
                trace!(self, "  vmov {}{}, {}{}", p, d, p, m);
            }
            (0b0000, true) => {
                vm.abs().store(self, d);
                trace!(self, "  vabs {}{}, {}{}", p, d, p, m);
            }
            (0b0001, false) => {
                (-vm).store(self, d);
                trace!(self, "  vneg {}{}, {}{}", p, d, p, m);
            }
            (0b0001, true) => {
                vm.sqrt().store(self, d);
                trace!(self, "  vsqrt {}{}, {}{}", p, d, p, m);
                return CYCLES_FP_SLOW;
            }
            // VCMP / VCMPE: with a register (0100) or with zero (0101).
            // The E form differs only in raising an exception flag for a
            // quiet NaN, and those flags are not kept.
            (0b0100, _) | (0b0101, _) => {
                let left = F::load(self, d);
                let right = if opc2 == 0b0101 { F::ZERO } else { vm };
                // FPCompare (A2.5.7): N Z C V.
                let nzcv = match left.partial_cmp(&right) {
                    Some(std::cmp::Ordering::Equal) => 0b0110,
                    Some(std::cmp::Ordering::Less) => 0b1000,
                    Some(std::cmp::Ordering::Greater) => 0b0010,
                    None => 0b0011, // unordered: one of them is a NaN
                };
                self.fpscr = (self.fpscr & 0x0FFF_FFFF) | (nzcv << 28);
                trace!(self, "  vcmp {}{}  -> nzcv = {:04b}", p, d, nzcv);
            }
            // VCVT between double and single: the source is in `F`.
            (0b0111, true) => {
                let value = vm.to_f64();
                if F::PREFIX == 'd' {
                    // The destination is an S register: Vd with D as
                    // the low bit.
                    let sd = f32::index(hw2 >> 12, extra_d);
                    (value as f32).store(self, sd);
                    trace!(self, "  vcvt s{}, d{}", sd, m);
                } else {
                    let dd = f64::index(hw2 >> 12, extra_d);
                    value.store(self, dd);
                    trace!(self, "  vcvt d{}, s{}", dd, m);
                }
            }
            // VCVT integer to floating-point. The integer is in an S
            // register; `high` says it is signed.
            (0b1000, signed) => {
                let sm = f32::index(hw2 & 0xF, extra_m);
                let integer = self.fp[sm];
                let value = if signed {
                    integer as i32 as f64
                } else {
                    integer as f64
                };
                F::from_f64(value).store(self, d);
                trace!(self, "  vcvt {}{}, s{} (from integer)", p, d, sm);
            }
            // VCVT / VCVTR floating-point to integer. The integer goes
            // to an S register. opc2 bit 0 says signed; `high` says
            // round toward zero (VCVT) instead of by FPSCR (VCVTR).
            (0b1100, toward_zero) | (0b1101, toward_zero) => {
                let sd = f32::index(hw2 >> 12, extra_d);
                let mode = if toward_zero {
                    3
                } else {
                    (self.fpscr >> 22) & 3
                };
                let rounded = round_to_integral(vm.to_f64(), mode);
                // A Rust cast does what the FPU does with values that do
                // not fit: the nearest representable integer, and 0 for
                // a NaN.
                self.fp[sd] = if opc2 & 1 != 0 {
                    rounded as i32 as u32
                } else {
                    rounded as u32
                };
                trace!(self, "  vcvt s{}, {}{} (to integer)", sd, p, m);
            }
            _ => self.fp_unhandled(hw1, hw2, pc),
        }
        CYCLES_DATA
    }

    /// The data-processing instructions FPv5 added, in precision `F`:
    /// 11111110 opc1 Vn | Vd 101 sz N op M 0 Vm.
    fn fp_data_processing_v5<F: Fp>(&mut self, hw1: u32, hw2: u32, pc: u32) -> u32 {
        let d = F::index(hw2 >> 12, (hw1 >> 6) & 1);
        let n = F::index(hw1 & 0xF, (hw2 >> 7) & 1);
        let m = F::index(hw2 & 0xF, (hw2 >> 5) & 1);
        let op = hw2 & (1 << 6) != 0;
        let (vn, vm) = (F::load(self, n), F::load(self, m));
        let p = F::PREFIX;

        if hw1 & 0x80 == 0 && !op {
            // VSEL<cond>: 11111110 0 D cc Vn  (A7.7.256). Copy Vn if the
            // condition holds, Vm if not: an "if ... else" on the CPU's
            // flags without a branch. Only four conditions can be
            // encoded; a compiler gets the opposite ones by swapping
            // the two registers.
            let cc = (hw1 >> 4) & 3;
            // EQ (0000), VS (0110), GE (1010) or GT (1100).
            let cond = (cc << 2) | (((cc >> 1) ^ (cc & 1)) << 1);
            let holds = condition_passed(cond as u16, self.xpsr);
            (if holds { vn } else { vm }).store(self, d);
            trace!(
                self,
                "  vsel<cond {:04b}> {}{}, {}{}, {}{}  -> {}",
                cond,
                p,
                d,
                p,
                n,
                p,
                m,
                if holds { "first" } else { "second" }
            );
        } else if hw1 & 0xB0 == 0x80 {
            // VMAXNM / VMINNM: 11111110 1 D 00 Vn  (A7.7.237)
            max_or_min_number(vn, vm, !op).store(self, d);
            trace!(
                self,
                "  {} {}{}, {}{}, {}{}",
                if op { "vminnm" } else { "vmaxnm" },
                p,
                d,
                p,
                n,
                p,
                m
            );
        } else {
            self.fp_unhandled(hw1, hw2, pc);
        }
        CYCLES_DATA
    }

    /// 32-bit transfers between a core register and the FPU (table A6-8):
    /// 11101110 A L Vn | Rt 101 C N B 1 0000.
    fn fp_transfer_32(&mut self, hw1: u32, hw2: u32, pc: u32) -> u32 {
        let to_core = hw1 & (1 << 4) != 0; // L
        let a = (hw1 >> 5) & 0b111;
        let c = hw2 & (1 << 8) != 0;
        let t = ((hw2 >> 12) & 0xF) as usize; // Rt
        let field = hw1 & 0xF; // Vn
        let extra = (hw2 >> 7) & 1; // N
        match (a, c) {
            // VMOV between a core register and an S register.
            (0b000, false) => {
                let s = f32::index(field, extra);
                if to_core {
                    self.regs[t] = self.fp[s];
                    trace!(self, "  vmov r{}, s{}  -> {:#010X}", t, s, self.regs[t]);
                } else {
                    self.fp[s] = self.regs[t];
                    trace!(self, "  vmov s{}, r{}  -> {:#010X}", s, t, self.fp[s]);
                }
            }
            // VMRS / VMSR: read or write FPSCR (register number 1 in the
            // Vn field).
            (0b111, false) if field == 1 => {
                if !to_core {
                    self.fpscr = self.regs[t] & FPSCR_MASK;
                    trace!(self, "  vmsr fpscr, r{}", t);
                } else if t == PC {
                    // `vmrs APSR_nzcv, fpscr`: copy the comparison flags
                    // to where conditional branches look.
                    self.xpsr = (self.xpsr & 0x0FFF_FFFF) | (self.fpscr & 0xF000_0000);
                    trace!(self, "  vmrs APSR_nzcv, fpscr");
                } else {
                    self.regs[t] = self.fpscr;
                    trace!(self, "  vmrs r{}, fpscr", t);
                }
            }
            // VMOV between a core register and one half of a D register.
            // Bit 5 of hw1 picks the half.
            (0b000, true) | (0b001, true) if hw2 & 0x60 == 0 => {
                let d = f64::index(field, extra);
                let high = hw1 & (1 << 5) != 0;
                let s = 2 * d + high as usize;
                if d >= 16 {
                    self.fp_unhandled(hw1, hw2, pc);
                }
                if to_core {
                    self.regs[t] = self.fp[s];
                } else {
                    self.fp[s] = self.regs[t];
                }
                trace!(self, "  vmov r{} <-> d{}[{}]", t, d, high as u8);
            }
            _ => self.fp_unhandled(hw1, hw2, pc),
        }
        CYCLES_DATA
    }

    /// 64-bit transfers between two core registers and the FPU (table
    /// A6-9): 11101100010 L Rt2 | Rt 101 C 00 M 1 Vm.
    fn fp_transfer_64(&mut self, hw1: u32, hw2: u32, pc: u32) -> u32 {
        if hw2 & 0xD0 != 0x10 {
            self.fp_unhandled(hw1, hw2, pc);
        }
        let to_core = hw1 & (1 << 4) != 0;
        let t2 = (hw1 & 0xF) as usize;
        let t = ((hw2 >> 12) & 0xF) as usize;
        let extra = (hw2 >> 5) & 1; // M
        // The low word is in Rt and the high word in Rt2. With C set
        // they are the two halves of a D register; otherwise two S
        // registers one after the other.
        let low = if hw2 & (1 << 8) != 0 {
            2 * f64::index(hw2 & 0xF, extra)
        } else {
            f32::index(hw2 & 0xF, extra)
        };
        if low + 1 >= 32 {
            self.fp_unhandled(hw1, hw2, pc);
        }
        if to_core {
            self.regs[t] = self.fp[low];
            self.regs[t2] = self.fp[low + 1];
        } else {
            self.fp[low] = self.regs[t];
            self.fp[low + 1] = self.regs[t2];
        }
        trace!(self, "  vmov r{}, r{} <-> s{}, s{}", t, t2, low, low + 1);
        CYCLES_DATA
    }

    /// Loads and stores of FPU registers (table A6-7):
    /// 1110110 P U D W L Rn | Vd 101 sz imm8.
    fn fp_load_store(&mut self, hw1: u32, hw2: u32, pc: u32, bus: &mut Bus) -> u32 {
        let p = hw1 & (1 << 8) != 0;
        let add = hw1 & (1 << 7) != 0; // U
        let extra_d = (hw1 >> 6) & 1; // D
        let writeback = hw1 & (1 << 5) != 0; // W
        let load = hw1 & (1 << 4) != 0; // L
        let n = (hw1 & 0xF) as usize;
        let double = hw2 & (1 << 8) != 0;
        let imm8 = hw2 & 0xFF;
        let imm32 = imm8 << 2;
        // First S register involved, and how many 32-bit words.
        let first = if double {
            2 * f64::index(hw2 >> 12, extra_d)
        } else {
            f32::index(hw2 >> 12, extra_d)
        };
        // As a base register, the PC reads as the instruction's address
        // + 4, word-aligned.
        let base = if n == PC {
            pc.wrapping_add(4) & !3
        } else {
            self.regs[n]
        };

        if p && !writeback {
            // VLDR / VSTR: one register at base +/- offset.
            let address = if add {
                base.wrapping_add(imm32)
            } else {
                base.wrapping_sub(imm32)
            };
            let words = if double { 2 } else { 1 };
            if first + words > 32 {
                self.fp_unhandled(hw1, hw2, pc);
            }
            for i in 0..words {
                let word_address = address.wrapping_add(4 * i as u32);
                if load {
                    self.fp[first + i] = bus.read32(word_address);
                } else {
                    bus.write32(word_address, self.fp[first + i]);
                }
            }
            trace!(
                self,
                "  {} {}{}, [{:#010X}]",
                if load { "vldr" } else { "vstr" },
                if double { 'd' } else { 's' },
                if double { first / 2 } else { first },
                address
            );
            return CYCLES_MEMORY;
        }

        // VSTM / VLDM, which include VPUSH and VPOP: `imm8` words of
        // consecutive registers. Either upwards from the base, or (with
        // write-back only) downwards ending just below it.
        let valid = (!p && add) || (p && !add && writeback);
        // An odd count with D registers is an old format (FSTMX) that
        // compilers no longer produce.
        if !valid || n == PC || imm8 == 0 || (double && imm8 & 1 != 0) {
            self.fp_unhandled(hw1, hw2, pc);
        }
        let words = imm8 as usize;
        if first + words > 32 {
            self.fp_unhandled(hw1, hw2, pc);
        }
        let start = if add { base } else { base.wrapping_sub(imm32) };
        for i in 0..words {
            let word_address = start.wrapping_add(4 * i as u32);
            if load {
                self.fp[first + i] = bus.read32(word_address);
            } else {
                bus.write32(word_address, self.fp[first + i]);
            }
        }
        if writeback {
            self.regs[n] = if add {
                base.wrapping_add(imm32)
            } else {
                base.wrapping_sub(imm32)
            };
        }
        trace!(
            self,
            "  {} r{}{}, {} words from s{}",
            if load { "vldm" } else { "vstm" },
            n,
            if writeback { "!" } else { "" },
            words,
            first
        );
        // One cycle, plus one per word moved.
        CYCLES_DATA + words as u32
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::cpu::{SP, cpu_with_code, nzcv};

    /// A CPU with `code` to run and the FPU switched on.
    fn fp_cpu(code: &[u16]) -> (Cpu, Bus) {
        let (cpu, mut bus) = cpu_with_code(code);
        bus.write32(CPACR, 0xF << 20);
        (cpu, bus)
    }

    fn set_s(cpu: &mut Cpu, index: usize, value: f32) {
        cpu.fp[index] = value.to_bits();
    }

    fn s(cpu: &Cpu, index: usize) -> f32 {
        f32::from_bits(cpu.fp[index])
    }

    fn set_d(cpu: &mut Cpu, index: usize, value: f64) {
        cpu.set_d(index, value.to_bits());
    }

    fn d(cpu: &Cpu, index: usize) -> f64 {
        f64::from_bits(cpu.d(index))
    }

    /// Run one single-precision instruction that computes S0 from S0, S1
    /// and S2.
    fn s0_after(code: [u16; 2], s0: f32, s1: f32, s2: f32) -> f32 {
        let (mut cpu, mut bus) = fp_cpu(&code);
        set_s(&mut cpu, 0, s0);
        set_s(&mut cpu, 1, s1);
        set_s(&mut cpu, 2, s2);
        cpu.step(&mut bus);
        s(&cpu, 0)
    }

    #[test]
    fn fpu_instruction_with_the_fpu_off_is_a_usage_fault() {
        // vadd.f32 s0, s1, s2, without enabling the FPU. The image has
        // no HardFault handler, so the entry ends at address 0.
        let (mut cpu, mut bus) = cpu_with_code(&[0xEE30, 0x0A81]);
        set_s(&mut cpu, 1, 1.0);
        set_s(&mut cpu, 2, 2.0);
        cpu.step(&mut bus);
        assert_eq!(cpu.xpsr & 0x1FF, 3); // HardFault
        assert_eq!(bus.read32(0xE000_ED28), 1 << 19); // CFSR.NOCP
        assert_eq!(s(&cpu, 0), 0.0); // nothing was computed
    }

    #[test]
    fn privileged_only_access_keeps_unprivileged_code_out() {
        let (mut cpu, mut bus) = cpu_with_code(&[0xEE30, 0x0A81, 0xEE30, 0x0A81]);
        bus.write32(CPACR, 0x5 << 20); // privileged access only
        set_s(&mut cpu, 1, 1.0);
        set_s(&mut cpu, 2, 2.0);
        cpu.step(&mut bus);
        assert_eq!(s(&cpu, 0), 3.0);
        cpu.control |= 1; // nPRIV
        cpu.step(&mut bus);
        assert_eq!(cpu.xpsr & 0x1FF, 3);
    }

    #[test]
    fn single_precision_arithmetic() {
        assert_eq!(s0_after([0xEE30, 0x0A81], 0.0, 1.5, 2.25), 3.75); // vadd
        assert_eq!(s0_after([0xEE20, 0x0A81], 0.0, 1.5, 2.0), 3.0); // vmul
        assert_eq!(s0_after([0xEE80, 0x0A81], 0.0, 1.0, 4.0), 0.25); // vdiv
        assert_eq!(s0_after([0xEE20, 0x0AC1], 0.0, 1.5, 2.0), -3.0); // vnmul
        // vsub.f32 s3, s4, s5
        let (mut cpu, mut bus) = fp_cpu(&[0xEE72, 0x1A62]);
        set_s(&mut cpu, 4, 10.0);
        set_s(&mut cpu, 5, 0.5);
        cpu.step(&mut bus);
        assert_eq!(s(&cpu, 3), 9.5);
    }

    #[test]
    fn single_precision_rounds_like_f32() {
        // 0.1 + 0.2 in f32 is not what it is in f64.
        let sum = s0_after([0xEE30, 0x0A81], 0.0, 0.1, 0.2);
        assert_eq!(sum.to_bits(), (0.1f32 + 0.2f32).to_bits());
        // Division by zero gives infinity, not a fault.
        assert_eq!(s0_after([0xEE80, 0x0A81], 0.0, 1.0, 0.0), f32::INFINITY);
        assert!(s0_after([0xEE80, 0x0A81], 0.0, 0.0, 0.0).is_nan());
    }

    #[test]
    fn multiply_accumulate_family() {
        // S0 = 10, S1 = 2, S2 = 3 throughout.
        assert_eq!(s0_after([0xEE00, 0x0A81], 10.0, 2.0, 3.0), 16.0); // vmla
        assert_eq!(s0_after([0xEE00, 0x0AC1], 10.0, 2.0, 3.0), 4.0); // vmls
        assert_eq!(s0_after([0xEE10, 0x0AC1], 10.0, 2.0, 3.0), -16.0); // vnmla
        assert_eq!(s0_after([0xEE10, 0x0A81], 10.0, 2.0, 3.0), -4.0); // vnmls
        assert_eq!(s0_after([0xEEA0, 0x0A81], 10.0, 2.0, 3.0), 16.0); // vfma
        assert_eq!(s0_after([0xEEA0, 0x0AC1], 10.0, 2.0, 3.0), 4.0); // vfms
        assert_eq!(s0_after([0xEE90, 0x0AC1], 10.0, 2.0, 3.0), -16.0); // vfnma
        assert_eq!(s0_after([0xEE90, 0x0A81], 10.0, 2.0, 3.0), -4.0); // vfnms
    }

    #[test]
    fn fused_multiply_add_rounds_once() {
        // (1 + e)(1 - e) - 1, with e the smallest step above 1.0. The
        // exact product, 1 - e^2, needs more bits than an f32 has:
        // rounded on its own (vmla) it becomes 1.0 and the answer 0,
        // where vfma keeps the -e^2.
        let (x, y) = (1.0 + f32::EPSILON, 1.0 - f32::EPSILON);
        let fused = s0_after([0xEEA0, 0x0A81], -1.0, x, y);
        let separate = s0_after([0xEE00, 0x0A81], -1.0, x, y);
        assert_eq!(fused, -f32::EPSILON * f32::EPSILON);
        assert_eq!(separate, 0.0);
    }

    #[test]
    fn double_precision_arithmetic() {
        // vadd.f64 d0, d1, d2 ; vmul.f64 d0, d1, d2 ; vdiv.f64 d0, d1, d2 ;
        // vsub.f64 d3, d4, d5 ; vfma.f64 d0, d1, d2
        let code = [
            0xEE31, 0x0B02, 0xEE21, 0x0B02, 0xEE81, 0x0B02, 0xEE34, 0x3B45, 0xEEA1, 0x0B02,
        ];
        let (mut cpu, mut bus) = fp_cpu(&code);
        set_d(&mut cpu, 1, 0.1);
        set_d(&mut cpu, 2, 0.2);
        set_d(&mut cpu, 4, 5.0);
        set_d(&mut cpu, 5, 7.5);
        cpu.step(&mut bus);
        assert_eq!(d(&cpu, 0), 0.1 + 0.2);
        cpu.step(&mut bus);
        assert_eq!(d(&cpu, 0), 0.1 * 0.2);
        cpu.step(&mut bus);
        assert_eq!(d(&cpu, 0), 0.5);
        cpu.step(&mut bus);
        assert_eq!(d(&cpu, 3), -2.5);
        cpu.step(&mut bus);
        assert_eq!(d(&cpu, 0), 0.1f64.mul_add(0.2, 0.5));
    }

    #[test]
    fn d_registers_are_pairs_of_s_registers() {
        // vmov.f64 d1, #31.0
        let (mut cpu, mut bus) = fp_cpu(&[0xEEB3, 0x1B0F]);
        cpu.step(&mut bus);
        assert_eq!(d(&cpu, 1), 31.0);
        let bits = 31.0f64.to_bits();
        assert_eq!(cpu.fp[2], bits as u32); // S2 is the low half
        assert_eq!(cpu.fp[3], (bits >> 32) as u32); // S3 the high half
    }

    #[test]
    fn immediate_constants() {
        // vmov.f32 s0, #1.0 ; vmov.f32 s1, #-2.5 ; vmov.f64 d1, #0.5
        let (mut cpu, mut bus) = fp_cpu(&[0xEEB7, 0x0A00, 0xEEF8, 0x0A04, 0xEEB6, 0x1B00]);
        for _ in 0..3 {
            cpu.step(&mut bus);
        }
        assert_eq!(s(&cpu, 0), 1.0);
        assert_eq!(s(&cpu, 1), -2.5);
        assert_eq!(d(&cpu, 1), 0.5);
    }

    #[test]
    fn every_immediate_matches_the_manuals_formula() {
        // (-1)^sign * 2^(exp-3) * (16 + mantissa)/16, exp from 0 to 7
        // with the top bit inverted (table A6-6).
        for imm8 in 0..=255u32 {
            let sign = if imm8 & 0x80 != 0 { -1.0 } else { 1.0 };
            let exp = (((imm8 >> 4) & 7) ^ 4) as i32 - 3;
            let expected = sign * 2f64.powi(exp) * (16 + (imm8 & 0xF)) as f64 / 16.0;
            assert_eq!(f64::expand_imm(imm8), expected, "imm8 = {:#04X}", imm8);
            assert_eq!(
                f32::expand_imm(imm8),
                expected as f32,
                "imm8 = {:#04X}",
                imm8
            );
        }
    }

    #[test]
    fn register_moves_and_one_operand_operations() {
        // vmov.f32 s31, s1 ; vabs.f32 s0, s1 ; vneg.f32 s0, s1 ;
        // vsqrt.f32 s0, s1 ; vmov.f64 d15, d1 ; vsqrt.f64 d0, d1 ;
        // vneg.f64 d0, d1
        let code = [
            0xEEF0, 0xFA60, 0xEEB0, 0x0AE0, 0xEEB1, 0x0A60, 0xEEB1, 0x0AE0, 0xEEB0, 0xFB41, 0xEEB1,
            0x0BC1, 0xEEB1, 0x0B41,
        ];
        let (mut cpu, mut bus) = fp_cpu(&code);
        set_s(&mut cpu, 1, -6.25);
        cpu.step(&mut bus);
        assert_eq!(s(&cpu, 31), -6.25);
        cpu.step(&mut bus);
        assert_eq!(s(&cpu, 0), 6.25);
        cpu.step(&mut bus);
        assert_eq!(s(&cpu, 0), 6.25);
        set_s(&mut cpu, 1, 6.25);
        cpu.step(&mut bus);
        assert_eq!(s(&cpu, 0), 2.5);

        set_d(&mut cpu, 1, 2.0);
        cpu.step(&mut bus);
        assert_eq!(d(&cpu, 15), 2.0);
        cpu.step(&mut bus);
        assert_eq!(d(&cpu, 0), 2f64.sqrt());
        cpu.step(&mut bus);
        assert_eq!(d(&cpu, 0), -2.0);
    }

    #[test]
    fn compare_sets_the_fpscr_flags() {
        // vcmp.f32 s0, s1
        let flags = |a: f32, b: f32| {
            let (mut cpu, mut bus) = fp_cpu(&[0xEEB4, 0x0A60]);
            set_s(&mut cpu, 0, a);
            set_s(&mut cpu, 1, b);
            cpu.step(&mut bus);
            cpu.fpscr >> 28
        };
        assert_eq!(flags(1.0, 1.0), 0b0110); // equal: Z and C
        assert_eq!(flags(1.0, 2.0), 0b1000); // less: N
        assert_eq!(flags(2.0, 1.0), 0b0010); // greater: C
        assert_eq!(flags(f32::NAN, 1.0), 0b0011); // unordered: C and V
        assert_eq!(flags(-0.0, 0.0), 0b0110); // the two zeros are equal
    }

    #[test]
    fn compare_then_vmrs_drives_a_conditional_branch() {
        // vcmp.f32 s0, #0 ; vmrs APSR_nzcv, fpscr ; bmi +0 ; movs r0, #1
        let (mut cpu, mut bus) = fp_cpu(&[0xEEB5, 0x0A40, 0xEEF1, 0xFA10, 0xD400, 0x2001]);
        set_s(&mut cpu, 0, -3.0);
        cpu.step(&mut bus);
        // The CPU's own flags are untouched until vmrs copies them.
        assert_eq!(nzcv(&cpu), 0);
        cpu.step(&mut bus);
        assert_eq!(nzcv(&cpu), 0b1000);
        cpu.step(&mut bus); // bmi: taken, skipping the movs
        assert_eq!(cpu.regs[PC], 0x0040_0014);
    }

    #[test]
    fn double_compare_and_the_e_form() {
        // vcmp.f64 d0, d1 ; vcmpe.f32 s4, s5 (as vcmpe.f32 s0, s1 would
        // overlap d0)
        let (mut cpu, mut bus) = fp_cpu(&[0xEEB4, 0x0B41]);
        set_d(&mut cpu, 0, 1e300);
        set_d(&mut cpu, 1, 1e301);
        cpu.step(&mut bus);
        assert_eq!(cpu.fpscr >> 28, 0b1000);

        let (mut cpu, mut bus) = fp_cpu(&[0xEEB4, 0x0AE0]); // vcmpe.f32 s0, s1
        set_s(&mut cpu, 0, 2.0);
        set_s(&mut cpu, 1, 1.0);
        cpu.step(&mut bus);
        assert_eq!(cpu.fpscr >> 28, 0b0010);
    }

    #[test]
    fn fpscr_can_be_read_and_written() {
        // vmsr fpscr, r0 ; vmrs r0, fpscr
        let (mut cpu, mut bus) = fp_cpu(&[0xEEE1, 0x0A10, 0xEEF1, 0x0A10]);
        cpu.regs[0] = 0xFFFF_FFFF;
        cpu.step(&mut bus);
        cpu.regs[0] = 0;
        cpu.step(&mut bus);
        assert_eq!(cpu.regs[0], FPSCR_MASK); // only the real bits stick
    }

    #[test]
    fn conversions_between_single_and_double() {
        // vcvt.f64.f32 d0, s2 ; vcvt.f32.f64 s0, d1
        let (mut cpu, mut bus) = fp_cpu(&[0xEEB7, 0x0AC1, 0xEEB7, 0x0BC1]);
        set_s(&mut cpu, 2, 0.1);
        cpu.step(&mut bus);
        assert_eq!(d(&cpu, 0), 0.1f32 as f64);
        set_d(&mut cpu, 1, 1.0 / 3.0);
        cpu.step(&mut bus);
        assert_eq!(s(&cpu, 0), (1.0f64 / 3.0) as f32);
    }

    #[test]
    fn conversions_from_integer() {
        // vcvt.f32.s32 s0, s1 ; vcvt.f32.u32 s0, s1 ; vcvt.f64.s32 d0, s2 ;
        // vcvt.f64.u32 d0, s2
        let (mut cpu, mut bus) = fp_cpu(&[
            0xEEB8, 0x0AE0, 0xEEB8, 0x0A60, 0xEEB8, 0x0BC1, 0xEEB8, 0x0B41,
        ]);
        cpu.fp[1] = -7i32 as u32;
        cpu.step(&mut bus);
        assert_eq!(s(&cpu, 0), -7.0);
        cpu.step(&mut bus);
        assert_eq!(s(&cpu, 0), 4_294_967_289u32 as f32);
        cpu.fp[2] = 0x8000_0001;
        cpu.step(&mut bus);
        assert_eq!(d(&cpu, 0), -2_147_483_647.0);
        cpu.fp[2] = 0x8000_0001;
        cpu.step(&mut bus);
        assert_eq!(d(&cpu, 0), 2_147_483_649.0);
    }

    #[test]
    fn conversions_to_integer_round_toward_zero_and_saturate() {
        // vcvt.s32.f32 s0, s1
        let to_i32 = |value: f32| {
            let (mut cpu, mut bus) = fp_cpu(&[0xEEBD, 0x0AE0]);
            set_s(&mut cpu, 1, value);
            cpu.step(&mut bus);
            cpu.fp[0] as i32
        };
        assert_eq!(to_i32(2.9), 2);
        assert_eq!(to_i32(-2.9), -2);
        assert_eq!(to_i32(1e20), i32::MAX);
        assert_eq!(to_i32(-1e20), i32::MIN);
        assert_eq!(to_i32(f32::NAN), 0);

        // vcvt.u32.f32 s0, s1
        let to_u32 = |value: f32| {
            let (mut cpu, mut bus) = fp_cpu(&[0xEEBC, 0x0AE0]);
            set_s(&mut cpu, 1, value);
            cpu.step(&mut bus);
            cpu.fp[0]
        };
        assert_eq!(to_u32(3.99), 3);
        assert_eq!(to_u32(-1.0), 0);
        assert_eq!(to_u32(1e20), u32::MAX);

        // vcvt.s32.f64 s0, d1 ; vcvt.u32.f64 s0, d1
        let (mut cpu, mut bus) = fp_cpu(&[0xEEBD, 0x0BC1, 0xEEBC, 0x0BC1]);
        set_d(&mut cpu, 1, -123456.789);
        cpu.step(&mut bus);
        assert_eq!(cpu.fp[0] as i32, -123456);
        set_d(&mut cpu, 1, 4_000_000_000.5);
        cpu.step(&mut bus);
        assert_eq!(cpu.fp[0], 4_000_000_000);
    }

    #[test]
    fn vcvtr_uses_the_rounding_mode_in_fpscr() {
        // vcvtr.s32.f32 s0, s1, with 2.5 under each rounding mode.
        let with_mode = |mode: u32, value: f32| {
            let (mut cpu, mut bus) = fp_cpu(&[0xEEBD, 0x0A60]);
            // Already an FPU-using context: a new one would start with
            // the default rounding mode from FPDSCR.
            cpu.control |= CONTROL_FPCA;
            cpu.fpscr = mode << 22;
            set_s(&mut cpu, 1, value);
            cpu.step(&mut bus);
            cpu.fp[0] as i32
        };
        assert_eq!(with_mode(0, 2.5), 2); // nearest, ties to even
        assert_eq!(with_mode(0, 3.5), 4);
        assert_eq!(with_mode(1, 2.5), 3); // up
        assert_eq!(with_mode(2, -2.5), -3); // down
        assert_eq!(with_mode(3, -2.5), -2); // toward zero
    }

    #[test]
    fn moves_between_core_and_fpu_registers() {
        // vmov s0, r1 ; vmov r3, s0 (as vmov r1, s0 with Rt = 3) ;
        // vmov s31, r2
        let (mut cpu, mut bus) = fp_cpu(&[0xEE00, 0x1A10, 0xEE10, 0x3A10, 0xEE0F, 0x2A90]);
        cpu.regs[1] = 0x3F80_0000; // the bits of 1.0f32
        cpu.regs[2] = 0xDEAD_BEEF;
        cpu.step(&mut bus);
        assert_eq!(s(&cpu, 0), 1.0);
        cpu.step(&mut bus);
        assert_eq!(cpu.regs[3], 0x3F80_0000);
        cpu.step(&mut bus);
        assert_eq!(cpu.fp[31], 0xDEAD_BEEF);
    }

    #[test]
    fn moves_between_two_core_registers_and_a_double() {
        // vmov d0, r1, r2 ; vmov r1, r2, d0 ; vmov s2, s3, r1, r2
        let (mut cpu, mut bus) = fp_cpu(&[0xEC42, 0x1B10, 0xEC52, 0x1B10, 0xEC42, 0x1A11]);
        let bits = 1234.5678f64.to_bits();
        cpu.regs[1] = bits as u32; // low word
        cpu.regs[2] = (bits >> 32) as u32; // high word
        cpu.step(&mut bus);
        assert_eq!(d(&cpu, 0), 1234.5678);
        cpu.regs[1] = 0;
        cpu.regs[2] = 0;
        cpu.step(&mut bus);
        assert_eq!(cpu.regs[1], bits as u32);
        assert_eq!(cpu.regs[2], (bits >> 32) as u32);
        cpu.step(&mut bus);
        assert_eq!(d(&cpu, 1), 1234.5678); // S2 and S3 are D1
    }

    #[test]
    fn moves_of_one_half_of_a_double() {
        // vmov.32 d0[1], r1 ; vmov.32 r2, d0[1] (Rt = 2)
        let (mut cpu, mut bus) = fp_cpu(&[0xEE20, 0x1B10, 0xEE30, 0x2B10]);
        cpu.regs[1] = 0x4000_0000; // the high word of 2.0f64
        cpu.step(&mut bus);
        assert_eq!(d(&cpu, 0), 2.0);
        cpu.step(&mut bus);
        assert_eq!(cpu.regs[2], 0x4000_0000);
    }

    #[test]
    fn loads_and_stores_of_one_register() {
        // vldr s0, [r1] ; vldr s0, [r1, #8] ; vldr d0, [r1, #-8] ;
        // vstr s1, [r2, #4] ; vstr d1, [r2]
        let code = [
            0xED91, 0x0A00, 0xED91, 0x0A02, 0xED11, 0x0B02, 0xEDC2, 0x0A01, 0xED82, 0x1B00,
        ];
        let (mut cpu, mut bus) = fp_cpu(&code);
        cpu.regs[1] = 0x2040_0010;
        cpu.regs[2] = 0x2040_0100;
        bus.write32(0x2040_0010, 1.5f32.to_bits());
        bus.write32(0x2040_0018, 2.5f32.to_bits());
        let bits = (-9.75f64).to_bits();
        bus.write32(0x2040_0008, bits as u32); // low word at the lower address
        bus.write32(0x2040_000C, (bits >> 32) as u32);

        cpu.step(&mut bus);
        assert_eq!(s(&cpu, 0), 1.5);
        cpu.step(&mut bus);
        assert_eq!(s(&cpu, 0), 2.5);
        cpu.step(&mut bus);
        assert_eq!(d(&cpu, 0), -9.75);

        set_s(&mut cpu, 1, 7.0);
        cpu.step(&mut bus);
        assert_eq!(bus.read32(0x2040_0104), 7.0f32.to_bits());
        set_d(&mut cpu, 1, 0.1);
        cpu.step(&mut bus);
        assert_eq!(bus.read32(0x2040_0100), 0.1f64.to_bits() as u32);
        assert_eq!(bus.read32(0x2040_0104), (0.1f64.to_bits() >> 32) as u32);
    }

    #[test]
    fn vldr_from_a_literal_pool() {
        // vldr s0, [pc, #8], at 0x0040_0008: the PC reads as 0x0040_000C,
        // so the constant is at 0x0040_0014.
        let pi = std::f32::consts::PI.to_bits();
        let code = [0xED9F, 0x0A02, 0, 0, 0, 0, pi as u16, (pi >> 16) as u16];
        let (mut cpu, mut bus) = fp_cpu(&code);
        cpu.step(&mut bus);
        assert_eq!(s(&cpu, 0), std::f32::consts::PI);
    }

    #[test]
    fn vpush_and_vpop_save_and_restore_registers() {
        // vpush {s16-s19} ; vpop {s16-s19}
        let (mut cpu, mut bus) = fp_cpu(&[0xED2D, 0x8A04, 0xECBD, 0x8A04]);
        for i in 0..4 {
            set_s(&mut cpu, 16 + i, i as f32 + 0.5);
        }
        cpu.step(&mut bus);
        assert_eq!(cpu.regs[SP], 0x2045_FFF0);
        // The lowest-numbered register is at the lowest address.
        assert_eq!(bus.read32(0x2045_FFF0), 0.5f32.to_bits());
        assert_eq!(bus.read32(0x2045_FFFC), 3.5f32.to_bits());
        for i in 0..4 {
            set_s(&mut cpu, 16 + i, 0.0);
        }
        cpu.step(&mut bus);
        assert_eq!(cpu.regs[SP], 0x2046_0000);
        assert_eq!(s(&cpu, 16), 0.5);
        assert_eq!(s(&cpu, 19), 3.5);
    }

    #[test]
    fn vpush_and_vpop_of_doubles() {
        // vpush {d8-d9} ; vpop {d8-d9}
        let (mut cpu, mut bus) = fp_cpu(&[0xED2D, 0x8B04, 0xECBD, 0x8B04]);
        set_d(&mut cpu, 8, 1.25);
        set_d(&mut cpu, 9, -8.5);
        cpu.step(&mut bus);
        assert_eq!(cpu.regs[SP], 0x2045_FFF0);
        set_d(&mut cpu, 8, 0.0);
        set_d(&mut cpu, 9, 0.0);
        cpu.step(&mut bus);
        assert_eq!(d(&cpu, 8), 1.25);
        assert_eq!(d(&cpu, 9), -8.5);
        assert_eq!(cpu.regs[SP], 0x2046_0000);
    }

    #[test]
    fn store_and_load_multiple_through_any_register() {
        // vstmia r0!, {s0-s2} ; vldmia r0, {d0-d1} (no write-back)
        let (mut cpu, mut bus) = fp_cpu(&[0xECA0, 0x0A03, 0xEC90, 0x0B04]);
        cpu.regs[0] = 0x2040_0000;
        for i in 0..3 {
            set_s(&mut cpu, i, i as f32 + 1.0);
        }
        cpu.step(&mut bus);
        assert_eq!(cpu.regs[0], 0x2040_000C);
        assert_eq!(bus.read32(0x2040_0008), 3.0f32.to_bits());

        bus.write32(0x2040_000C, 0x1111_1111);
        bus.write32(0x2040_0010, 0x2222_2222);
        bus.write32(0x2040_0014, 0x3333_3333);
        bus.write32(0x2040_0018, 0x4444_4444);
        cpu.step(&mut bus);
        assert_eq!(cpu.d(0), 0x2222_2222_1111_1111);
        assert_eq!(cpu.d(1), 0x4444_4444_3333_3333);
        assert_eq!(cpu.regs[0], 0x2040_000C);
    }

    #[test]
    fn using_the_fpu_marks_the_context_in_control() {
        // movs r0, #1 ; vmov.f32 s0, #1.0
        let (mut cpu, mut bus) = fp_cpu(&[0x2001, 0xEEB7, 0x0A00]);
        cpu.step(&mut bus);
        assert_eq!(cpu.control & CONTROL_FPCA, 0);
        cpu.step(&mut bus);
        assert_eq!(cpu.control & CONTROL_FPCA, CONTROL_FPCA);
    }

    #[test]
    fn vsel_picks_by_the_cpu_flags() {
        // S1 = 1.0 is the "condition holds" value, S2 = 2.0 the other.
        let select = |hw1: u16, flags: u32| {
            let (mut cpu, mut bus) = fp_cpu(&[hw1, 0x0A81]);
            cpu.xpsr |= flags << 28;
            set_s(&mut cpu, 1, 1.0);
            set_s(&mut cpu, 2, 2.0);
            cpu.step(&mut bus);
            s(&cpu, 0)
        };
        const N: u32 = 0b1000;
        const Z: u32 = 0b0100;
        const V: u32 = 0b0001;
        // vseleq: Z set
        assert_eq!(select(0xFE00, Z), 1.0);
        assert_eq!(select(0xFE00, 0), 2.0);
        // vselvs: V set
        assert_eq!(select(0xFE10, V), 1.0);
        assert_eq!(select(0xFE10, 0), 2.0);
        // vselge: N equals V
        assert_eq!(select(0xFE20, 0), 1.0);
        assert_eq!(select(0xFE20, N | V), 1.0);
        assert_eq!(select(0xFE20, N), 2.0);
        // vselgt: N equals V, and Z clear
        assert_eq!(select(0xFE30, 0), 1.0);
        assert_eq!(select(0xFE30, Z), 2.0);
        assert_eq!(select(0xFE30, V), 2.0);
    }

    #[test]
    fn vsel_follows_a_floating_point_compare() {
        // vcmp.f32 s1, s2 would need another encoding; use the pattern a
        // compiler emits for "if s0 > s1 { s1 } else { s2 }"-style code:
        // vcmp.f32 s0, s1 ; vmrs APSR_nzcv, fpscr ; vselgt.f32 s0, s1, s2
        let code = [0xEEB4, 0x0A60, 0xEEF1, 0xFA10, 0xFE30, 0x0A81];
        let run = |s0: f32| {
            let (mut cpu, mut bus) = fp_cpu(&code);
            set_s(&mut cpu, 0, s0);
            set_s(&mut cpu, 1, 5.0);
            set_s(&mut cpu, 2, -1.0);
            for _ in 0..3 {
                cpu.step(&mut bus);
            }
            s(&cpu, 0)
        };
        assert_eq!(run(9.0), 5.0); // 9 > 5: first
        assert_eq!(run(3.0), -1.0); // not greater: second
        assert_eq!(run(f32::NAN), -1.0); // unordered is not "greater"
    }

    #[test]
    fn vsel_in_double_precision_moves_all_64_bits() {
        // vselgt.f64 d0, d1, d2
        let (mut cpu, mut bus) = fp_cpu(&[0xFE31, 0x0B02]);
        set_d(&mut cpu, 1, 1.0e300);
        set_d(&mut cpu, 2, -1.0e-300);
        cpu.step(&mut bus); // flags clear: GT holds
        assert_eq!(d(&cpu, 0), 1.0e300);
    }

    #[test]
    fn vmaxnm_and_vminnm_pick_the_larger_and_smaller() {
        // vmaxnm.f32 s0, s1, s2 ; vminnm.f32 s0, s1, s2
        let max = |a: f32, b: f32| s0_after([0xFE80, 0x0A81], 0.0, a, b);
        let min = |a: f32, b: f32| s0_after([0xFE80, 0x0AC1], 0.0, a, b);
        assert_eq!(max(1.0, 2.0), 2.0);
        assert_eq!(max(2.0, 1.0), 2.0);
        assert_eq!(min(1.0, 2.0), 1.0);
        assert_eq!(min(-3.0, -7.0), -7.0);
        assert_eq!(max(f32::NEG_INFINITY, -1.0e30), -1.0e30);
        assert_eq!(min(f32::INFINITY, 1.0e30), 1.0e30);
    }

    #[test]
    fn vmaxnm_and_vminnm_ignore_a_single_nan() {
        let max = |a: f32, b: f32| s0_after([0xFE80, 0x0A81], 0.0, a, b);
        let min = |a: f32, b: f32| s0_after([0xFE80, 0x0AC1], 0.0, a, b);
        assert_eq!(max(f32::NAN, 2.0), 2.0);
        assert_eq!(max(2.0, f32::NAN), 2.0);
        assert_eq!(min(f32::NAN, -2.0), -2.0);
        assert_eq!(min(-2.0, f32::NAN), -2.0);
        assert!(max(f32::NAN, f32::NAN).is_nan());
        assert!(min(f32::NAN, f32::NAN).is_nan());
    }

    #[test]
    fn positive_zero_is_larger_than_negative_zero() {
        let max = |a: f32, b: f32| s0_after([0xFE80, 0x0A81], 1.0, a, b);
        let min = |a: f32, b: f32| s0_after([0xFE80, 0x0AC1], 1.0, a, b);
        // Compare bit patterns: 0.0 == -0.0 would hide the difference.
        assert_eq!(max(0.0, -0.0).to_bits(), 0.0f32.to_bits());
        assert_eq!(max(-0.0, 0.0).to_bits(), 0.0f32.to_bits());
        assert_eq!(min(0.0, -0.0).to_bits(), (-0.0f32).to_bits());
        assert_eq!(min(-0.0, 0.0).to_bits(), (-0.0f32).to_bits());
    }

    #[test]
    fn vmaxnm_and_vminnm_in_double_precision() {
        // vmaxnm.f64 d0, d1, d2 ; vminnm.f64 d0, d1, d2
        let (mut cpu, mut bus) = fp_cpu(&[0xFE81, 0x0B02, 0xFE81, 0x0B42]);
        set_d(&mut cpu, 1, 0.1);
        set_d(&mut cpu, 2, 0.2);
        cpu.step(&mut bus);
        assert_eq!(d(&cpu, 0), 0.2);
        cpu.step(&mut bus);
        assert_eq!(d(&cpu, 0), 0.1);
    }

    #[test]
    #[should_panic(expected = "unhandled FPU instruction at 0x00400008: 0xFEB8 0x0A40")]
    fn fpu_instruction_not_yet_implemented_stops_the_emulator() {
        // vrinta.f32 s0, s0: one of the FPv5 additions.
        let (mut cpu, mut bus) = fp_cpu(&[0xFEB8, 0x0A40]);
        cpu.step(&mut bus);
    }
}
