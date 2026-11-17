//! Shared helper functions from the ARMv7-M pseudocode (DDI0403E).
//!
//! Shifts and `AddWithCarry` are in section A2.2.1, `ThumbExpandImm_C` in
//! A5.3.2 and `ConditionPassed` in A7.3.1.

/// AddWithCarry: returns (result, carry_out, overflow).
pub fn add_with_carry(x: u32, y: u32, carry_in: bool) -> (u32, bool, bool) {
    let unsigned_sum = x as u64 + y as u64 + carry_in as u64;
    let signed_sum = x as i32 as i64 + y as i32 as i64 + carry_in as i64;
    let result = unsigned_sum as u32;
    let carry = unsigned_sum > u32::MAX as u64;
    let overflow = signed_sum != result as i32 as i64;
    (result, carry, overflow)
}

/// LSL_C: logical shift left. Returns (result, carry_out), where the carry is
/// the last bit shifted out. `shift` must be greater than 0.
pub fn lsl_c(x: u32, shift: u32) -> (u32, bool) {
    assert!(shift > 0);
    match shift {
        1..=31 => (x << shift, (x >> (32 - shift)) & 1 == 1),
        32 => (0, x & 1 == 1),
        _ => (0, false),
    }
}

/// LSR_C: logical shift right, shifting in zeros. `shift` must be greater
/// than 0.
pub fn lsr_c(x: u32, shift: u32) -> (u32, bool) {
    assert!(shift > 0);
    match shift {
        1..=31 => (x >> shift, (x >> (shift - 1)) & 1 == 1),
        32 => (0, x >> 31 == 1),
        _ => (0, false),
    }
}

/// ASR_C: arithmetic shift right, shifting in copies of the sign bit.
/// `shift` must be greater than 0.
pub fn asr_c(x: u32, shift: u32) -> (u32, bool) {
    assert!(shift > 0);
    let signed = x as i32;
    match shift {
        1..=31 => ((signed >> shift) as u32, (x >> (shift - 1)) & 1 == 1),
        // Every bit, including the carry, is a copy of the sign bit.
        _ => ((signed >> 31) as u32, x >> 31 == 1),
    }
}

/// ROR_C: rotate right. The carry is bit 31 of the result. `shift` must not
/// be 0.
pub fn ror_c(x: u32, shift: u32) -> (u32, bool) {
    assert!(shift != 0);
    let result = x.rotate_right(shift % 32);
    (result, result >> 31 == 1)
}

/// RRX_C: rotate right by one bit through the carry flag.
pub fn rrx_c(x: u32, carry_in: bool) -> (u32, bool) {
    let result = ((carry_in as u32) << 31) | (x >> 1);
    (result, x & 1 == 1)
}

/// ThumbExpandImm_C: expand the 12-bit "modified immediate" used by 32-bit
/// Thumb data-processing instructions. Returns (imm32, carry_out).
///
/// The encodings the manual marks UNPREDICTABLE (a repeated-byte form with a
/// zero byte) are not rejected; they expand to 0.
pub fn thumb_expand_imm_c(imm12: u32, carry_in: bool) -> (u32, bool) {
    let imm12 = imm12 & 0xFFF;
    let byte = imm12 & 0xFF;

    if imm12 >> 10 == 0b00 {
        let imm32 = match (imm12 >> 8) & 0b11 {
            0b00 => byte,                                          // 000000XY
            0b01 => (byte << 16) | byte,                           // 00XY00XY
            0b10 => (byte << 24) | (byte << 8),                    // XY00XY00
            _ => (byte << 24) | (byte << 16) | (byte << 8) | byte, // XYXYXYXY
        };
        (imm32, carry_in)
    } else {
        // An 8-bit value with its top bit forced to 1, rotated right.
        let unrotated = 0x80 | (imm12 & 0x7F);
        ror_c(unrotated, imm12 >> 7)
    }
}

/// ConditionPassed: test a 4-bit condition code against the N, Z, C, V flags
/// held in bits 31-28 of `apsr`.
pub fn condition_passed(cond: u16, apsr: u32) -> bool {
    let n = apsr & (1 << 31) != 0;
    let z = apsr & (1 << 30) != 0;
    let c = apsr & (1 << 29) != 0;
    let v = apsr & (1 << 28) != 0;

    // Bits 3-1 pick the test; bit 0 inverts it.
    let result = match (cond >> 1) & 0b111 {
        0b000 => z,            // EQ / NE
        0b001 => c,            // CS / CC
        0b010 => n,            // MI / PL
        0b011 => v,            // VS / VC
        0b100 => c && !z,      // HI / LS
        0b101 => n == v,       // GE / LT
        0b110 => n == v && !z, // GT / LE
        _ => true,             // AL
    };
    if cond & 1 == 1 && cond & 0xF != 0b1111 {
        !result
    } else {
        result
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const N: u32 = 1 << 31;
    const Z: u32 = 1 << 30;
    const C: u32 = 1 << 29;
    const V: u32 = 1 << 28;

    #[test]
    fn add_with_carry_plain() {
        assert_eq!(add_with_carry(1, 2, false), (3, false, false));
        assert_eq!(add_with_carry(1, 2, true), (4, false, false));
    }

    #[test]
    fn add_with_carry_unsigned_carry() {
        assert_eq!(add_with_carry(0xFFFF_FFFF, 1, false), (0, true, false));
        assert_eq!(add_with_carry(0xFFFF_FFFF, 0, true), (0, true, false));
    }

    #[test]
    fn add_with_carry_signed_overflow() {
        // Largest positive + 1 wraps to the most negative value.
        assert_eq!(
            add_with_carry(0x7FFF_FFFF, 1, false),
            (0x8000_0000, false, true)
        );
        // Most negative + (-1) wraps to the largest positive value.
        assert_eq!(
            add_with_carry(0x8000_0000, 0xFFFF_FFFF, false),
            (0x7FFF_FFFF, true, true)
        );
    }

    #[test]
    fn add_with_carry_as_subtraction() {
        // x - y is computed as x + NOT(y) + 1; carry set means "no borrow".
        assert_eq!(add_with_carry(5, !3, true), (2, true, false));
        assert_eq!(add_with_carry(3, !5, true), (0xFFFF_FFFE, false, false));
        assert_eq!(add_with_carry(7, !7, true), (0, true, false));
    }

    #[test]
    fn lsl_c_cases() {
        assert_eq!(lsl_c(0x0000_0202, 30), (0x8000_0000, false));
        assert_eq!(lsl_c(0x8000_0001, 1), (0x0000_0002, true));
        assert_eq!(lsl_c(0x0000_0001, 31), (0x8000_0000, false));
        assert_eq!(lsl_c(0x0000_0001, 32), (0, true));
        assert_eq!(lsl_c(0xFFFF_FFFF, 33), (0, false));
    }

    #[test]
    fn lsr_c_cases() {
        assert_eq!(lsr_c(0x8000_0001, 1), (0x4000_0000, true));
        assert_eq!(lsr_c(0x0000_0004, 2), (0x0000_0001, false));
        assert_eq!(lsr_c(0x8000_0000, 31), (0x0000_0001, false));
        assert_eq!(lsr_c(0x8000_0000, 32), (0, true));
        assert_eq!(lsr_c(0xFFFF_FFFF, 33), (0, false));
    }

    #[test]
    fn asr_c_cases() {
        assert_eq!(asr_c(0x8000_0000, 1), (0xC000_0000, false));
        assert_eq!(asr_c(0x8000_0001, 1), (0xC000_0000, true));
        assert_eq!(asr_c(0x4000_0000, 30), (0x0000_0001, false));
        assert_eq!(asr_c(0x8000_0000, 32), (0xFFFF_FFFF, true));
        assert_eq!(asr_c(0x7FFF_FFFF, 40), (0, false));
    }

    #[test]
    fn ror_c_cases() {
        assert_eq!(ror_c(0x0000_0001, 1), (0x8000_0000, true));
        assert_eq!(ror_c(0x1234_5678, 8), (0x7812_3456, false));
        // A multiple of 32 leaves the value alone; carry is its top bit.
        assert_eq!(ror_c(0x8000_0000, 32), (0x8000_0000, true));
    }

    #[test]
    fn rrx_c_cases() {
        assert_eq!(rrx_c(0x0000_0001, false), (0x0000_0000, true));
        assert_eq!(rrx_c(0x0000_0002, true), (0x8000_0001, false));
    }

    #[test]
    fn thumb_expand_imm_c_byte_patterns() {
        // These forms pass the carry through unchanged.
        assert_eq!(thumb_expand_imm_c(0x0AB, false), (0x0000_00AB, false));
        assert_eq!(thumb_expand_imm_c(0x0AB, true), (0x0000_00AB, true));
        assert_eq!(thumb_expand_imm_c(0x1AB, false), (0x00AB_00AB, false));
        assert_eq!(thumb_expand_imm_c(0x2AB, false), (0xAB00_AB00, false));
        assert_eq!(thumb_expand_imm_c(0x3AB, true), (0xABAB_ABAB, true));
    }

    #[test]
    fn thumb_expand_imm_c_rotated() {
        // imm12<11:7> = 8, low 7 bits 0 -> 0x80 ror 8 = 0x8000_0000.
        assert_eq!(thumb_expand_imm_c(0x400, false), (0x8000_0000, true));
        // imm12<11:7> = 9 -> 0x80 ror 9 = 0x4000_0000.
        assert_eq!(thumb_expand_imm_c(0x480, true), (0x4000_0000, false));
        // imm12<11:7> = 31, low 7 bits 0x7F -> 0xFF ror 31 = 0x1FE.
        assert_eq!(thumb_expand_imm_c(0xFFF, false), (0x0000_01FE, false));
    }

    #[test]
    fn condition_passed_all_codes() {
        // (cond, flags that make it pass, flags that make it fail)
        let cases = [
            (0b0000, Z, 0),     // EQ
            (0b0001, 0, Z),     // NE
            (0b0010, C, 0),     // CS
            (0b0011, 0, C),     // CC
            (0b0100, N, 0),     // MI
            (0b0101, 0, N),     // PL
            (0b0110, V, 0),     // VS
            (0b0111, 0, V),     // VC
            (0b1000, C, C | Z), // HI
            (0b1001, Z, C),     // LS
            (0b1010, N | V, N), // GE
            (0b1011, V, 0),     // LT
            (0b1100, 0, Z),     // GT
            (0b1101, N, N | V), // LE
        ];
        for (cond, pass, fail) in cases {
            assert!(
                condition_passed(cond, pass),
                "cond {:04b} should pass",
                cond
            );
            assert!(
                !condition_passed(cond, fail),
                "cond {:04b} should fail",
                cond
            );
        }
    }

    #[test]
    fn condition_passed_always() {
        for flags in [0, N, Z, C, V, N | Z | C | V] {
            assert!(condition_passed(0b1110, flags));
            assert!(condition_passed(0b1111, flags));
        }
    }
}
