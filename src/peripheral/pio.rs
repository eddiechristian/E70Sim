//! PIO (Parallel Input/Output Controller) stub. One instance per port:
//! PIOA 0x400E_0E00, PIOB 0x400E_1000, PIOC 0x400E_1200, PIOD 0x400E_1400,
//! PIOE 0x400E_1600.
//!
//! Firmware uses the PIO to hand pins over to peripherals (pin muxing), to
//! drive outputs such as LEDs and to read inputs such as buttons. Nothing
//! is connected to the pins here, so the stub only has to remember the
//! configuration and give sensible answers when firmware reads it back.
//!
//! Most PIO settings are a group of three registers: writing 1 bits to the
//! first sets those bits in a read-only status register, writing 1 bits to
//! the second clears them. Those groups are all handled by one table.
//!
//! Pin levels (PIO_PDSR): an output pin reads back what was driven onto it;
//! any other pin reads what the outside world drives, which is 1 (as if
//! pulled up with nothing pressing it) until `set_input` says otherwise.
//! The emulator's `--pin=` option is what calls `set_input`.
//!
//! Interrupts: a change on a pin sets its bit in PIO_ISR, and the port
//! asks for its interrupt while a bit set there is also enabled in
//! PIO_IMR. Reading PIO_ISR clears it. With PIO_AIMMR a pin can be
//! limited to one edge, or to a level (see `detect_edges`).
//!
//! Not modelled: the input glitch filter and debouncing, write protection
//! (PIO_WPMR stores WPEN but protects nothing), pin locking, parallel
//! capture, and per-port reset values (every register starts at 0).
//!
//! Register offsets are from `ATSAME70Q21.svd`.

use crate::peripheral::Peripheral;

/// A setting made of three registers. A write to `set` sets bits in
/// `status`, a write to `clear` clears them, and `status` is read-only.
struct Triplet {
    set: u32,
    clear: u32,
    status: u32,
}

const fn triplet(set: u32, clear: u32, status: u32) -> Triplet {
    Triplet { set, clear, status }
}

const PIO_ODSR: u32 = 0x38; // Output Data Status
const PIO_OSR: u32 = 0x18; // Output Status (1 = pin is an output)
const PIO_OWSR: u32 = 0xA8; // Output Write Status

#[rustfmt::skip]
const TRIPLETS: [Triplet; 14] = [
    triplet(0x00, 0x04, 0x08),    // PER / PDR -> PSR: PIO (1) or peripheral (0) controls the pin
    triplet(0x10, 0x14, PIO_OSR), // OER / ODR -> OSR: output enable
    triplet(0x20, 0x24, 0x28),    // IFER / IFDR -> IFSR: glitch input filter
    triplet(0x30, 0x34, PIO_ODSR),// SODR / CODR -> ODSR: output data
    triplet(0x40, 0x44, 0x48),    // IER / IDR -> IMR: interrupt mask
    triplet(0x50, 0x54, 0x58),    // MDER / MDDR -> MDSR: multi-driver (open drain)
    triplet(0x60, 0x64, 0x68),    // PUDR / PUER -> PUSR: 1 = pull-up disabled
    triplet(0x84, 0x80, 0x88),    // IFSCER / IFSCDR -> IFSCSR: slow clock for the filter
    triplet(0x90, 0x94, 0x98),    // PPDDR / PPDER -> PPDSR: 1 = pull-down disabled
    triplet(0xA0, 0xA4, PIO_OWSR),// OWER / OWDR -> OWSR: direct writes to ODSR allowed
    triplet(0xB0, 0xB4, 0xB8),    // AIMER / AIMDR -> AIMMR: additional interrupt modes
    triplet(0xC4, 0xC0, 0xC8),    // LSR / ESR -> ELSR: level (1) or edge (0) interrupt
    triplet(0xD4, 0xD0, 0xD8),    // REHLSR / FELLSR -> FRLHSR: rising/high (1) or falling/low (0)
    triplet(0x154, 0x158, 0x15C), // PCIER / PCIDR -> PCIMR: parallel capture interrupt mask
];

// Registers that are neither part of a triplet nor plain storage.
const PIO_PDSR: u32 = 0x3C; // Pin Data Status (read-only)
const PIO_ISR: u32 = 0x4C; // Interrupt Status (read-only)
const PIO_LOCKSR: u32 = 0xE0; // Lock Status (read-only)
const PIO_WPMR: u32 = 0xE4; // Write Protection Mode
const PIO_WPSR: u32 = 0xE8; // Write Protection Status (read-only)
const PIO_PCISR: u32 = 0x160; // Parallel Capture Interrupt Status (read-only)
const PIO_PCRHR: u32 = 0x164; // Parallel Capture Reception Holding (read-only)

// PIO_WPMR: bit 0 is WPEN; bits 31-8 are a write key that reads back as 0.
const WPMR_WPEN: u32 = 1 << 0;
const WPMR_WPKEY_SHIFT: u32 = 8;
const WPMR_WPKEY_PASSWD: u32 = 0x50_494F; // "PIO" in ASCII

/// Size of the register block, from the SVD address block.
const BLOCK_SIZE: u32 = 0x168;

const PIO_IMR: u32 = 0x48; // Interrupt Mask
const PIO_AIMMR: u32 = 0xB8; // Additional Interrupt Modes Mask
const PIO_ELSR: u32 = 0xC8; // Edge/Level Status (1 = level)
const PIO_FRLHSR: u32 = 0xD8; // Fall/Rise - Low/High Status (1 = rising or high)

pub struct PioStub {
    /// One slot per 32-bit register, indexed by offset / 4.
    regs: [u32; (BLOCK_SIZE / 4) as usize],
    /// The level the outside world puts on each pin: bit n is pin n.
    /// It shows on the pins that are not outputs. All high after reset,
    /// as if every pin had a pull-up and nothing connected.
    inputs: u32,
    /// PIO_ISR: pins on which an interrupt condition has been seen since
    /// the register was last read.
    interrupt_status: u32,
}

impl PioStub {
    pub fn new() -> Self {
        PioStub {
            regs: [0; (BLOCK_SIZE / 4) as usize],
            inputs: 0xFFFF_FFFF,
            interrupt_status: 0,
        }
    }

    /// The level on every pin (PIO_PDSR): an output pin shows what the
    /// firmware drives, any other pin what the outside world drives.
    fn pin_levels(&self) -> u32 {
        let outputs = self.get(PIO_OSR);
        (self.get(PIO_ODSR) & outputs) | (self.inputs & !outputs)
    }

    /// Pins set up for a level interrupt whose level is present now.
    /// Unlike an edge, a level keeps the interrupt condition true for as
    /// long as it lasts.
    fn active_levels(&self) -> u32 {
        let wanted = self.get(PIO_FRLHSR); // 1 = high level, 0 = low level
        self.get(PIO_AIMMR) & self.get(PIO_ELSR) & !(self.pin_levels() ^ wanted)
    }

    /// Note the edges between two sets of pin levels in PIO_ISR.
    ///
    /// By default any change counts. A pin with its bit set in
    /// PIO_AIMMR ("additional interrupt modes") is pickier: PIO_ELSR
    /// says edge or level, and PIO_FRLHSR which edge or which level.
    fn detect_edges(&mut self, before: u32, after: u32) {
        let changed = before ^ after;
        let additional = self.get(PIO_AIMMR);
        let edge_mode = additional & !self.get(PIO_ELSR);
        let rising_wanted = self.get(PIO_FRLHSR);
        let rising = changed & after;
        let falling = changed & !after;
        self.interrupt_status |= changed & !additional;
        self.interrupt_status |=
            edge_mode & ((rising & rising_wanted) | (falling & !rising_wanted));
    }

    /// The outside world drives pin `pin` (0 to 31) high or low.
    pub fn set_input(&mut self, pin: u32, high: bool) {
        let before = self.pin_levels();
        if high {
            self.inputs |= 1 << pin;
        } else {
            self.inputs &= !(1 << pin);
        }
        self.detect_edges(before, self.pin_levels());
    }

    /// True while the port is asking for its interrupt: an interrupt
    /// condition has been seen, or is present, on a pin enabled in
    /// PIO_IMR.
    pub fn interrupt_requested(&self) -> bool {
        (self.interrupt_status | self.active_levels()) & self.get(PIO_IMR) != 0
    }

    fn get(&self, offset: u32) -> u32 {
        self.regs[(offset / 4) as usize]
    }

    fn set(&mut self, offset: u32, value: u32) {
        self.regs[(offset / 4) as usize] = value;
    }
}

impl Peripheral for PioStub {
    /// All registers are 32 bits wide, so every access is treated as a
    /// whole-word access to the register containing `offset`.
    fn read(&mut self, offset: u32, _size: usize) -> u32 {
        let offset = offset & !3;
        if offset >= BLOCK_SIZE {
            return 0;
        }
        // The set and clear halves of a triplet are write-only.
        if TRIPLETS
            .iter()
            .any(|t| t.set == offset || t.clear == offset)
        {
            return 0;
        }
        match offset {
            PIO_PDSR => self.pin_levels(),
            PIO_ISR => {
                // Reading clears it; a level that is still present shows
                // again at once.
                let value = self.interrupt_status | self.active_levels();
                self.interrupt_status = 0;
                value
            }
            // No locked pins, no protection violations and no parallel
            // capture data.
            PIO_LOCKSR | PIO_WPSR | PIO_PCISR | PIO_PCRHR => 0,
            // Triplet status registers and plain configuration registers
            // (ABCDSR, SCDR, WPMR, SCHMITT, DRIVER, PCMR) read what is stored.
            _ => self.get(offset),
        }
    }

    fn write(&mut self, offset: u32, _size: usize, value: u32) {
        let offset = offset & !3;
        if offset >= BLOCK_SIZE {
            return;
        }
        // A write can change what is on the pins (an output driven the
        // other way, a pin turned into an output), and that is an edge
        // like any other.
        let before = self.pin_levels();
        self.write_register(offset, value);
        self.detect_edges(before, self.pin_levels());
    }
}

impl PioStub {
    fn write_register(&mut self, offset: u32, value: u32) {
        for t in &TRIPLETS {
            if offset == t.set {
                self.set(t.status, self.get(t.status) | value);
                return;
            }
            if offset == t.clear {
                self.set(t.status, self.get(t.status) & !value);
                return;
            }
            // PIO_ODSR is the one status register that can also be written
            // directly, but only on the pins enabled in PIO_OWSR.
            if offset == t.status {
                if offset == PIO_ODSR {
                    let writable = self.get(PIO_OWSR);
                    let merged = (self.get(PIO_ODSR) & !writable) | (value & writable);
                    self.set(PIO_ODSR, merged);
                }
                return;
            }
        }
        match offset {
            PIO_PDSR | PIO_ISR | PIO_LOCKSR | PIO_WPSR | PIO_PCISR | PIO_PCRHR => {}
            PIO_WPMR => {
                // Without the right key the write is dropped.
                if (value >> WPMR_WPKEY_SHIFT) == WPMR_WPKEY_PASSWD {
                    self.set(PIO_WPMR, value & WPMR_WPEN);
                }
            }
            // Store-and-readback for the remaining configuration registers.
            _ => self.set(offset, value),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const PIO_PER: u32 = 0x00;
    const PIO_PDR: u32 = 0x04;
    const PIO_PSR: u32 = 0x08;
    const PIO_OER: u32 = 0x10;
    const PIO_ODR: u32 = 0x14;
    const PIO_SODR: u32 = 0x30;
    const PIO_CODR: u32 = 0x34;
    const PIO_PUDR: u32 = 0x60;
    const PIO_PUER: u32 = 0x64;
    const PIO_PUSR: u32 = 0x68;
    const PIO_ABCDSR0: u32 = 0x70;
    const PIO_ABCDSR1: u32 = 0x74;
    const PIO_OWER: u32 = 0xA0;
    const PIO_ESR: u32 = 0xC0;
    const PIO_LSR: u32 = 0xC4;
    const PIO_ELSR: u32 = 0xC8;

    #[test]
    fn enable_and_disable_update_the_status_register() {
        let mut pio = PioStub::new();
        pio.write(PIO_PER, 4, 0b1110);
        pio.write(PIO_PDR, 4, 0b0100);
        assert_eq!(pio.read(PIO_PSR, 4), 0b1010);
        // The enable and disable registers are write-only.
        assert_eq!(pio.read(PIO_PER, 4), 0);
        assert_eq!(pio.read(PIO_PDR, 4), 0);
    }

    #[test]
    fn status_registers_ignore_direct_writes() {
        let mut pio = PioStub::new();
        pio.write(PIO_PSR, 4, 0xFFFF_FFFF);
        assert_eq!(pio.read(PIO_PSR, 4), 0);
    }

    #[test]
    fn groups_whose_disable_register_comes_first() {
        let mut pio = PioStub::new();
        // Pull-up: the status bit means "disabled".
        pio.write(PIO_PUDR, 4, 0b11);
        pio.write(PIO_PUER, 4, 0b01);
        assert_eq!(pio.read(PIO_PUSR, 4), 0b10);
        // Edge / level select: the status bit means "level".
        pio.write(PIO_LSR, 4, 0b11);
        pio.write(PIO_ESR, 4, 0b10);
        assert_eq!(pio.read(PIO_ELSR, 4), 0b01);
    }

    #[test]
    fn output_pin_reads_back_what_was_driven() {
        let mut pio = PioStub::new();
        let led = 1 << 8;
        pio.write(PIO_PER, 4, led);
        pio.write(PIO_OER, 4, led);
        pio.write(PIO_SODR, 4, led);
        assert_eq!(pio.read(PIO_ODSR, 4), led);
        assert_ne!(pio.read(PIO_PDSR, 4) & led, 0);
        pio.write(PIO_CODR, 4, led);
        assert_eq!(pio.read(PIO_ODSR, 4), 0);
        assert_eq!(pio.read(PIO_PDSR, 4) & led, 0);
    }

    #[test]
    fn input_pins_read_high() {
        let mut pio = PioStub::new();
        assert_eq!(pio.read(PIO_PDSR, 4), 0xFFFF_FFFF);
        // Making a pin an output and back again returns it to high.
        pio.write(PIO_OER, 4, 1);
        assert_eq!(pio.read(PIO_PDSR, 4), 0xFFFF_FFFE);
        pio.write(PIO_ODR, 4, 1);
        assert_eq!(pio.read(PIO_PDSR, 4), 0xFFFF_FFFF);
    }

    #[test]
    fn direct_output_write_only_touches_enabled_pins() {
        let mut pio = PioStub::new();
        pio.write(PIO_ODSR, 4, 0xFF);
        assert_eq!(pio.read(PIO_ODSR, 4), 0);
        pio.write(PIO_OWER, 4, 0x0F);
        pio.write(PIO_SODR, 4, 0xC0);
        pio.write(PIO_ODSR, 4, 0x35);
        assert_eq!(pio.read(PIO_ODSR, 4), 0xC5);
    }

    #[test]
    fn peripheral_select_registers_read_back() {
        let mut pio = PioStub::new();
        pio.write(PIO_ABCDSR0, 4, 0x0000_0600);
        pio.write(PIO_ABCDSR1, 4, 0x0000_0200);
        assert_eq!(pio.read(PIO_ABCDSR0, 4), 0x0000_0600);
        assert_eq!(pio.read(PIO_ABCDSR1, 4), 0x0000_0200);
    }

    #[test]
    fn interrupt_status_is_read_only() {
        let mut pio = PioStub::new();
        pio.write(PIO_ISR, 4, 0xFFFF_FFFF);
        assert_eq!(pio.read(PIO_ISR, 4), 0);
    }

    const PIO_IER: u32 = 0x40;
    const PIO_IDR: u32 = 0x44;
    const PIO_AIMER: u32 = 0xB0;
    const PIO_FELLSR: u32 = 0xD0;
    const PIO_REHLSR: u32 = 0xD4;
    const BUTTON: u32 = 1 << 11; // a button on pin 11

    #[test]
    fn outside_world_drives_the_pins_that_are_not_outputs() {
        let mut pio = PioStub::new();
        pio.set_input(11, false);
        assert_eq!(pio.read(PIO_PDSR, 4), !BUTTON);
        pio.set_input(11, true);
        assert_eq!(pio.read(PIO_PDSR, 4), 0xFFFF_FFFF);
        // An output pin shows what the firmware drives instead.
        pio.write(PIO_OER, 4, BUTTON);
        pio.write(PIO_SODR, 4, BUTTON);
        pio.set_input(11, false);
        assert_ne!(pio.read(PIO_PDSR, 4) & BUTTON, 0);
    }

    #[test]
    fn any_change_on_a_pin_is_noted_until_the_status_is_read() {
        let mut pio = PioStub::new();
        pio.set_input(11, false);
        assert_eq!(pio.read(PIO_ISR, 4), BUTTON);
        assert_eq!(pio.read(PIO_ISR, 4), 0); // cleared by the read
        pio.set_input(11, true); // the other edge counts too
        assert_eq!(pio.read(PIO_ISR, 4), BUTTON);
        pio.set_input(11, true); // no change
        assert_eq!(pio.read(PIO_ISR, 4), 0);
    }

    #[test]
    fn interrupt_is_requested_only_for_enabled_pins() {
        let mut pio = PioStub::new();
        pio.set_input(11, false);
        assert!(!pio.interrupt_requested());
        pio.write(PIO_IER, 4, BUTTON);
        assert!(pio.interrupt_requested()); // the change was remembered
        pio.read(PIO_ISR, 4);
        assert!(!pio.interrupt_requested());

        pio.set_input(3, false); // a pin that is not enabled
        assert!(!pio.interrupt_requested());
        pio.write(PIO_IDR, 4, BUTTON);
        pio.set_input(11, true);
        assert!(!pio.interrupt_requested());
    }

    #[test]
    fn one_edge_can_be_selected() {
        // Falling edge only (a button that pulls the pin low).
        let mut pio = PioStub::new();
        pio.write(PIO_AIMER, 4, BUTTON);
        pio.write(PIO_ESR, 4, BUTTON);
        pio.write(PIO_FELLSR, 4, BUTTON);
        pio.set_input(11, false);
        assert_eq!(pio.read(PIO_ISR, 4), BUTTON);
        pio.set_input(11, true); // released: not the edge asked for
        assert_eq!(pio.read(PIO_ISR, 4), 0);

        // Rising edge only.
        let mut pio = PioStub::new();
        pio.write(PIO_AIMER, 4, BUTTON);
        pio.write(PIO_ESR, 4, BUTTON);
        pio.write(PIO_REHLSR, 4, BUTTON);
        pio.set_input(11, false);
        assert_eq!(pio.read(PIO_ISR, 4), 0);
        pio.set_input(11, true);
        assert_eq!(pio.read(PIO_ISR, 4), BUTTON);
    }

    #[test]
    fn level_interrupt_lasts_as_long_as_the_level() {
        let mut pio = PioStub::new();
        pio.write(PIO_AIMER, 4, BUTTON);
        pio.write(PIO_LSR, 4, BUTTON);
        pio.write(PIO_FELLSR, 4, BUTTON); // low level
        pio.write(PIO_IER, 4, BUTTON);
        assert!(!pio.interrupt_requested()); // the pin is high
        pio.set_input(11, false);
        assert!(pio.interrupt_requested());
        // Reading the status does not make a level go away.
        assert_eq!(pio.read(PIO_ISR, 4), BUTTON);
        assert!(pio.interrupt_requested());
        pio.set_input(11, true);
        assert!(!pio.interrupt_requested());
    }

    #[test]
    fn output_driven_by_firmware_is_an_edge_too() {
        let mut pio = PioStub::new();
        pio.write(PIO_SODR, 4, 1);
        pio.write(PIO_OER, 4, 1); // output, driven high: no change
        assert_eq!(pio.read(PIO_ISR, 4), 0);
        pio.write(PIO_CODR, 4, 1); // driven low
        assert_eq!(pio.read(PIO_ISR, 4), 1);
    }

    #[test]
    fn write_protection_needs_the_key() {
        let mut pio = PioStub::new();
        pio.write(PIO_WPMR, 4, WPMR_WPEN); // no key
        assert_eq!(pio.read(PIO_WPMR, 4), 0);
        pio.write(
            PIO_WPMR,
            4,
            (WPMR_WPKEY_PASSWD << WPMR_WPKEY_SHIFT) | WPMR_WPEN,
        );
        // The key itself reads back as 0.
        assert_eq!(pio.read(PIO_WPMR, 4), WPMR_WPEN);
    }

    #[test]
    fn access_past_the_register_block_is_harmless() {
        let mut pio = PioStub::new();
        pio.write(0x1FC, 4, 0x1234);
        assert_eq!(pio.read(0x1FC, 4), 0);
    }
}
