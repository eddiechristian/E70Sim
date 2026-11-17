//! NVIC (Nested Vectored Interrupt Controller), base 0xE000_E100.
//!
//! The part of the Arm core that keeps track of the peripheral interrupts
//! (IRQs). For each IRQ it holds an enable bit, a pending bit and a
//! priority. An IRQ's handler can run when the IRQ is both pending and
//! enabled.
//!
//! An IRQ is pending for one of two reasons:
//!
//! - its peripheral is asking for it right now (the peripheral's interrupt
//!   line is high). The request lasts until the firmware deals with the
//!   cause, for example by reading the received byte from a UART.
//! - firmware wrote its bit in ISPR. That lasts until the handler starts
//!   or firmware writes the bit in ICPR.
//!
//! Each IRQ also has a priority byte in IPR; a lower value is a higher
//! priority. The NVIC only stores the priorities and picks the most
//! urgent IRQ that is ready. Whether that IRQ may interrupt what is
//! running now is decided by the bus, which also knows the priorities of
//! the system exceptions.
//!
//! Register layout and behaviour are from the ARMv7-M ARM, section B3.4.
//! The number of IRQs is from `ATSAME70Q21.svd`.

use crate::peripheral::Peripheral;

// Register offsets from 0xE000_E100. Each of the first five is a row of
// 32-bit registers with one bit per IRQ: IRQ n is bit n % 32 of register
// n / 32.
const ISER: u32 = 0x000; // Interrupt Set-Enable
const ICER: u32 = 0x080; // Interrupt Clear-Enable
const ISPR: u32 = 0x100; // Interrupt Set-Pending
const ICPR: u32 = 0x180; // Interrupt Clear-Pending
const IABR: u32 = 0x200; // Interrupt Active Bit (read-only)
const IPR: u32 = 0x300; // Interrupt Priority, one byte per IRQ

/// Words of one-bit-per-IRQ state. The SAME70's highest IRQ number is 68,
/// so three words (IRQ 0 to 95) cover it.
const WORDS: usize = 3;
const IRQ_COUNT: u32 = 32 * WORDS as u32;

/// The SAME70 implements 3 priority bits, the top 3 of each priority
/// byte. The other bits always read 0.
const PRIORITY_MASK: u32 = 0xE0E0_E0E0;

pub struct Nvic {
    /// ISER / ICER: the IRQ is allowed to run its handler.
    enabled: [u32; WORDS],
    /// Pending bits set by firmware through ISPR.
    pending: [u32; WORDS],
    /// Interrupt lines: bit set while the peripheral is asking.
    lines: [u32; WORDS],
    /// IABR: the IRQ's handler has started and not yet returned.
    active: [u32; WORDS],
    /// IPR, four priority bytes per word.
    priority: [u32; IRQ_COUNT as usize / 4],
}

impl Nvic {
    pub fn new() -> Self {
        Nvic {
            enabled: [0; WORDS],
            pending: [0; WORDS],
            lines: [0; WORDS],
            active: [0; WORDS],
            priority: [0; IRQ_COUNT as usize / 4],
        }
    }

    /// Tell the NVIC whether the peripheral behind `irq` is asking for its
    /// interrupt at the moment.
    pub fn set_line(&mut self, irq: u32, asking: bool) {
        let (word, bit) = ((irq / 32) as usize, 1 << (irq % 32));
        if asking {
            self.lines[word] |= bit;
        } else {
            self.lines[word] &= !bit;
        }
    }

    /// Make `irq` pending, as a write to ISPR does.
    pub fn set_pending(&mut self, irq: u32) {
        // A number beyond the last IRQ is ignored.
        if let Some(word) = self.pending.get_mut((irq / 32) as usize) {
            *word |= 1 << (irq % 32);
        }
    }

    /// True if any IRQ is pending, enabled or not (ICSR.ISRPENDING).
    pub fn any_pending(&self) -> bool {
        (0..WORDS).any(|word| self.pending[word] | self.lines[word] != 0)
    }

    /// Record that the handler for `irq` has started (true) or returned
    /// (false). Shows in IABR.
    pub fn set_active(&mut self, irq: u32, active: bool) {
        let (word, bit) = ((irq / 32) as usize, 1 << (irq % 32));
        if active {
            self.active[word] |= bit;
        } else {
            self.active[word] &= !bit;
        }
    }

    /// The priority byte of `irq`. A lower value is a higher priority.
    pub fn priority(&self, irq: u32) -> u8 {
        (self.priority[(irq / 4) as usize] >> (8 * (irq % 4))) as u8
    }

    /// The most urgent IRQ that is pending and enabled, if any: the one
    /// with the lowest priority value, and of those the lowest number.
    pub fn next_irq(&self) -> Option<u32> {
        let mut best = None;
        for word in 0..WORDS {
            let mut ready = (self.pending[word] | self.lines[word]) & self.enabled[word];
            while ready != 0 {
                let irq = 32 * word as u32 + ready.trailing_zeros();
                ready &= ready - 1; // clear the lowest set bit
                // Going up through the numbers, so only a strictly more
                // urgent IRQ replaces the best so far.
                if best.is_none_or(|b| self.priority(irq) < self.priority(b)) {
                    best = Some(irq);
                }
            }
        }
        best
    }

    /// The handler for `irq` is about to start: a pending bit set through
    /// ISPR is cleared. A peripheral's request stays until the peripheral
    /// withdraws it.
    pub fn acknowledge(&mut self, irq: u32) {
        self.pending[(irq / 32) as usize] &= !(1 << (irq % 32));
    }

    /// `next_irq` and `acknowledge` in one step.
    #[cfg(test)]
    fn take_pending_irq(&mut self) -> Option<u32> {
        let irq = self.next_irq()?;
        self.acknowledge(irq);
        Some(irq)
    }

    /// Read the whole 32-bit register at `offset` (a multiple of 4).
    fn read_word(&self, offset: u32) -> u32 {
        let index = ((offset & 0x7F) / 4) as usize;
        match offset & !0x7F {
            // Both the set and the clear register read the current state.
            ISER | ICER => self.enabled.get(index).copied().unwrap_or(0),
            ISPR | ICPR => match index < WORDS {
                true => self.pending[index] | self.lines[index],
                false => 0,
            },
            IABR => self.active.get(index).copied().unwrap_or(0),
            _ if offset >= IPR => {
                let index = ((offset - IPR) / 4) as usize;
                self.priority.get(index).copied().unwrap_or(0)
            }
            _ => 0,
        }
    }

    /// Write to the 32-bit register at `offset` (a multiple of 4). `bits`
    /// is the value already moved to its place in the register and `mask`
    /// covers the bytes being written.
    fn write_word(&mut self, offset: u32, bits: u32, mask: u32) {
        if offset >= IPR {
            let index = ((offset - IPR) / 4) as usize;
            if let Some(word) = self.priority.get_mut(index) {
                *word = ((*word & !mask) | bits) & PRIORITY_MASK;
            }
            return;
        }
        let index = ((offset & 0x7F) / 4) as usize;
        if index >= WORDS {
            return;
        }
        // In these registers writing 1 acts on that IRQ and writing 0 does
        // nothing, so the bytes not being written need no special care.
        match offset & !0x7F {
            ISER => self.enabled[index] |= bits,
            ICER => self.enabled[index] &= !bits,
            ISPR => self.pending[index] |= bits,
            ICPR => self.pending[index] &= !bits,
            // IABR is read-only.
            _ => {}
        }
    }
}

impl Peripheral for Nvic {
    /// The priority bytes are normally written one at a time, so byte and
    /// halfword accesses are supported for the whole block.
    fn read(&mut self, offset: u32, size: usize) -> u32 {
        let shift = (offset & 3) * 8;
        let word = self.read_word(offset & !3) >> shift;
        match size {
            1 => word & 0xFF,
            2 => word & 0xFFFF,
            _ => word,
        }
    }

    fn write(&mut self, offset: u32, size: usize, value: u32) {
        let shift = (offset & 3) * 8;
        let mask = match size {
            1 => 0xFF,
            2 => 0xFFFF,
            _ => 0xFFFF_FFFF,
        } << shift;
        self.write_word(offset & !3, (value << shift) & mask, mask);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn nothing_is_enabled_or_pending_after_reset() {
        let mut nvic = Nvic::new();
        assert_eq!(nvic.read(ISER, 4), 0);
        assert_eq!(nvic.read(ISPR, 4), 0);
        assert_eq!(nvic.take_pending_irq(), None);
    }

    #[test]
    fn enable_bits_are_set_and_cleared_by_writing_ones() {
        let mut nvic = Nvic::new();
        nvic.write(ISER, 4, (1 << 7) | (1 << 9));
        // Writing 0 bits changes nothing.
        nvic.write(ISER, 4, 0);
        assert_eq!(nvic.read(ISER, 4), (1 << 7) | (1 << 9));
        // The clear register reads the same state.
        assert_eq!(nvic.read(ICER, 4), (1 << 7) | (1 << 9));
        nvic.write(ICER, 4, 1 << 9);
        assert_eq!(nvic.read(ISER, 4), 1 << 7);
    }

    #[test]
    fn irqs_above_31_use_the_following_registers() {
        let mut nvic = Nvic::new();
        nvic.write(ISER + 8, 4, 1 << 4); // IRQ 68
        nvic.write(ISPR + 8, 4, 1 << 4);
        assert_eq!(nvic.read(ISER, 4), 0);
        assert_eq!(nvic.take_pending_irq(), Some(68));
    }

    #[test]
    fn pending_irq_waits_until_it_is_enabled() {
        let mut nvic = Nvic::new();
        nvic.write(ISPR, 4, 1 << 7);
        assert_eq!(nvic.take_pending_irq(), None);
        // It still shows as pending.
        assert_eq!(nvic.read(ISPR, 4), 1 << 7);

        nvic.write(ISER, 4, 1 << 7);
        assert_eq!(nvic.take_pending_irq(), Some(7));
    }

    #[test]
    fn software_pending_bit_is_cleared_when_the_irq_is_taken() {
        let mut nvic = Nvic::new();
        nvic.write(ISER, 4, 1 << 7);
        nvic.write(ISPR, 4, 1 << 7);
        assert_eq!(nvic.take_pending_irq(), Some(7));
        assert_eq!(nvic.take_pending_irq(), None);
        assert_eq!(nvic.read(ISPR, 4), 0);
    }

    #[test]
    fn software_pending_bit_can_be_cleared_before_it_is_taken() {
        let mut nvic = Nvic::new();
        nvic.write(ISER, 4, 1 << 7);
        nvic.write(ISPR, 4, 1 << 7);
        nvic.write(ICPR, 4, 1 << 7);
        assert_eq!(nvic.take_pending_irq(), None);
    }

    #[test]
    fn peripheral_request_stays_pending_until_it_is_withdrawn() {
        let mut nvic = Nvic::new();
        nvic.write(ISER, 4, 1 << 7);
        nvic.set_line(7, true);
        assert_eq!(nvic.read(ISPR, 4), 1 << 7);
        // Taking it does not make the peripheral stop asking.
        assert_eq!(nvic.take_pending_irq(), Some(7));
        assert_eq!(nvic.take_pending_irq(), Some(7));
        nvic.set_line(7, false);
        assert_eq!(nvic.take_pending_irq(), None);
        assert_eq!(nvic.read(ISPR, 4), 0);
    }

    #[test]
    fn more_urgent_priority_goes_first_whatever_the_number() {
        let mut nvic = Nvic::new();
        nvic.write(ISER, 4, 0xFFFF_FFFF);
        nvic.write(ISER + 4, 4, 0xFFFF_FFFF);
        nvic.write(IPR + 7, 1, 0x80);
        nvic.write(IPR + 20, 1, 0x40);
        nvic.write(IPR + 33, 1, 0x20);
        nvic.write(IPR + 34, 1, 0x20);
        nvic.write(ISPR, 4, (1 << 20) | (1 << 7));
        nvic.write(ISPR + 4, 4, (1 << 1) | (1 << 2)); // IRQ 33 and 34
        assert_eq!(nvic.priority(20), 0x40);
        // 33 and 34 share the best priority: the lower number wins.
        assert_eq!(nvic.take_pending_irq(), Some(33));
        assert_eq!(nvic.take_pending_irq(), Some(34));
        assert_eq!(nvic.take_pending_irq(), Some(20));
        assert_eq!(nvic.take_pending_irq(), Some(7));
        assert_eq!(nvic.take_pending_irq(), None);
    }

    #[test]
    fn active_irqs_show_in_iabr() {
        let mut nvic = Nvic::new();
        nvic.set_active(7, true);
        nvic.set_active(68, true);
        assert_eq!(nvic.read(IABR, 4), 1 << 7);
        assert_eq!(nvic.read(IABR + 8, 4), 1 << 4);
        nvic.set_active(7, false);
        assert_eq!(nvic.read(IABR, 4), 0);
        // IABR is read-only.
        nvic.write(IABR + 8, 4, 0);
        assert_eq!(nvic.read(IABR + 8, 4), 1 << 4);
    }

    #[test]
    fn with_equal_priorities_the_lowest_irq_number_goes_first() {
        let mut nvic = Nvic::new();
        nvic.write(ISER, 4, 0xFFFF_FFFF);
        nvic.write(ISER + 4, 4, 0xFFFF_FFFF);
        nvic.write(ISPR + 4, 4, 1 << 1); // IRQ 33
        nvic.write(ISPR, 4, (1 << 20) | (1 << 7));
        assert_eq!(nvic.take_pending_irq(), Some(7));
        assert_eq!(nvic.take_pending_irq(), Some(20));
        assert_eq!(nvic.take_pending_irq(), Some(33));
        assert_eq!(nvic.take_pending_irq(), None);
    }

    #[test]
    fn priority_bytes_are_written_singly_and_keep_three_bits() {
        let mut nvic = Nvic::new();
        nvic.write(IPR + 7, 1, 0xFF); // IRQ 7
        assert_eq!(nvic.read(IPR + 7, 1), 0xE0);
        assert_eq!(nvic.read(IPR + 4, 4), 0xE000_0000);
        // The neighbouring bytes are untouched by another byte write.
        nvic.write(IPR + 6, 1, 0x40);
        assert_eq!(nvic.read(IPR + 4, 4), 0xE040_0000);
    }

    #[test]
    fn byte_write_to_an_enable_register_reaches_the_right_irqs() {
        let mut nvic = Nvic::new();
        nvic.write(ISER + 1, 1, 0x01); // bit 8 of the word: IRQ 8
        assert_eq!(nvic.read(ISER, 4), 1 << 8);
    }

    #[test]
    fn registers_beyond_the_last_irq_read_zero_and_ignore_writes() {
        let mut nvic = Nvic::new();
        nvic.write(ISER + 0x0C, 4, 0xFFFF_FFFF); // IRQ 96 and up
        nvic.write(IPR + 0xEC, 4, 0xFFFF_FFFF);
        assert_eq!(nvic.read(ISER + 0x0C, 4), 0);
        assert_eq!(nvic.read(IPR + 0xEC, 4), 0);
        assert_eq!(nvic.read(IABR, 4), 0);
    }
}
