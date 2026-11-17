//! TWIHS (Two-wire Interface, High Speed): the SAME70's I2C controller.
//! The chip has three, TWIHS0 to TWIHS2, each with its own bus.
//!
//! Only host mode is modelled: the SAME70 starts every transfer, which
//! is how it talks to sensors and memory chips. ("Host" and "client"
//! are the datasheet's words for what I2C traditionally calls master
//! and slave.) The devices on the bus implement `I2cDevice`; see
//! `src/i2c.rs` for how an I2C transfer works.
//!
//! How firmware uses it, in outline:
//!
//! - Set-up: the device's address and the direction go in TWIHS_MMR,
//!   and host mode is switched on with MSEN in TWIHS_CR.
//! - Writing: each byte written to TWIHS_THR is sent. The first one
//!   also starts the transfer. STOP in TWIHS_CR ends it.
//! - Reading: START in TWIHS_CR starts the transfer. Each received byte
//!   appears in TWIHS_RHR with RXRDY set, and reading it lets the next
//!   one in. Firmware sets STOP while the next-to-last byte is waiting
//!   (or together with START, to read a single byte); the controller
//!   then takes exactly one more byte and ends the transfer.
//! - "Internal address": if TWIHS_MMR asks for it, the controller first
//!   sends one to three bytes from TWIHS_IADR. For a sensor that is the
//!   register number, which saves firmware sending it as data.
//!
//! If no device has the address, or a device refuses a byte, NACK is
//! set in TWIHS_SR and the transfer ends.
//!
//! A transfer takes no emulated time: the bytes move at once. The bus
//! speed in TWIHS_CWGR is stored and otherwise ignored.
//!
//! Not modelled: client mode, high-speed mode, SMBus, the FIFO, 10-bit
//! addresses, and faults on the bus itself (arbitration loss, a held
//! clock line).
//!
//! Register offsets and bit positions are from `ATSAME70Q21.svd`; the
//! behaviour is from chapter 43 of the SAM E70 datasheet in
//! `reference/same70/`, host mode being section 43.7.3.

use crate::i2c::I2cDevice;
use crate::peripheral::Peripheral;

// Register offsets.
const TWIHS_CR: u32 = 0x00; // Control (write-only)
const TWIHS_MMR: u32 = 0x04; // Host Mode
const TWIHS_IADR: u32 = 0x0C; // Internal Address
const TWIHS_CWGR: u32 = 0x10; // Clock Waveform Generator
const TWIHS_SR: u32 = 0x20; // Status (read-only)
const TWIHS_IER: u32 = 0x24; // Interrupt Enable (write-only)
const TWIHS_IDR: u32 = 0x28; // Interrupt Disable (write-only)
const TWIHS_IMR: u32 = 0x2C; // Interrupt Mask (read-only)
const TWIHS_RHR: u32 = 0x30; // Receive Holding (read-only)
const TWIHS_THR: u32 = 0x34; // Transmit Holding (write-only)

// TWIHS_CR bits.
const CR_START: u32 = 1 << 0; // begin a read, or a repeated start
const CR_STOP: u32 = 1 << 1; // end the transfer
const CR_MSEN: u32 = 1 << 2; // enable host mode
const CR_MSDIS: u32 = 1 << 3; // disable host mode
const CR_SWRST: u32 = 1 << 7; // reset the controller

// TWIHS_MMR fields.
const MMR_IADRSZ_SHIFT: u32 = 8; // bits 9-8: number of internal address bytes
const MMR_MREAD: u32 = 1 << 12; // direction: 1 = read from the device
const MMR_DADR_SHIFT: u32 = 16; // bits 22-16: the device's address
const MMR_MASK: u32 = (0x7F << MMR_DADR_SHIFT) | MMR_MREAD | (0x3 << MMR_IADRSZ_SHIFT);

// TWIHS_SR bits. TWIHS_IER, TWIHS_IDR and TWIHS_IMR use the same
// positions.
const SR_TXCOMP: u32 = 1 << 0; // no transfer is in progress
const SR_RXRDY: u32 = 1 << 1; // a received byte is waiting in TWIHS_RHR
const SR_TXRDY: u32 = 1 << 2; // TWIHS_THR can take a byte
const SR_NACK: u32 = 1 << 8; // nothing answered, or a byte was refused

/// The TWIHS_SR bits that never change here: SVREAD, which is 1 after
/// reset and belongs to client mode, and the two lines of the bus, SCL
/// and SDA, both high as they are when the bus is idle.
const SR_FIXED: u32 = (1 << 3) | (1 << 24) | (1 << 25);

/// The status bits that can be interrupt sources.
const INTERRUPT_BITS: u32 = 0x003D_0FFF;

/// What the controller is in the middle of.
#[derive(Clone, Copy, PartialEq)]
enum Transfer {
    /// Nothing: the bus is free.
    Idle,
    /// Writing to the device at this index in `devices`.
    Writing(usize),
    /// Reading from the device at this index. `last` is set once STOP
    /// has been asked for: the next byte taken from the device is the
    /// last one.
    Reading { device: usize, last: bool },
}

pub struct Twihs {
    /// Everything attached to this bus.
    devices: Vec<Box<dyn I2cDevice>>,
    host_enabled: bool,
    mode: u32,
    internal_address: u32,
    clock_waveform: u32,
    interrupt_mask: u32,
    /// TXCOMP, RXRDY, TXRDY and NACK.
    status: u32,
    /// TWIHS_RHR: the last byte received.
    received: u8,
    transfer: Transfer,
}

impl Twihs {
    pub fn new() -> Self {
        Twihs {
            devices: Vec::new(),
            host_enabled: false,
            mode: 0,
            internal_address: 0,
            clock_waveform: 0,
            interrupt_mask: 0,
            status: SR_TXCOMP,
            received: 0,
            transfer: Transfer::Idle,
        }
    }

    /// Plug a device into this bus.
    pub fn attach(&mut self, device: Box<dyn I2cDevice>) {
        self.devices.push(device);
    }

    /// Let emulated time pass for the devices on the bus.
    pub fn tick(&mut self, cycles: u32) {
        for device in &mut self.devices {
            device.tick(cycles);
        }
    }

    /// The interrupt output of the attached device at `address`, if
    /// there is such a device and it has one.
    pub fn device_interrupt_line(&self, address: u8) -> Option<bool> {
        self.devices
            .iter()
            .find(|d| d.address() == address)
            .and_then(|d| d.interrupt_line())
    }

    /// How many clock cycles from now an attached device will next do
    /// something on its own.
    pub fn cycles_until_device_event(&self) -> Option<u32> {
        self.devices
            .iter()
            .filter_map(|d| d.cycles_until_event())
            .min()
    }

    /// True while the controller is asking for its interrupt: a status
    /// bit that is set is also enabled in TWIHS_IMR.
    pub fn interrupt_requested(&self) -> bool {
        self.status & self.interrupt_mask != 0
    }

    /// The attached device whose address is the one in TWIHS_MMR.
    fn addressed_device(&self) -> Option<usize> {
        let address = ((self.mode >> MMR_DADR_SHIFT) & 0x7F) as u8;
        self.devices.iter().position(|d| d.address() == address)
    }

    /// The transfer failed: nothing answered, or a byte was refused.
    /// The controller sends a STOP and reports it.
    fn fail(&mut self, device: Option<usize>) {
        if let Some(index) = device {
            self.devices[index].stop();
        }
        self.transfer = Transfer::Idle;
        self.status &= !SR_RXRDY;
        self.status |= SR_NACK | SR_TXCOMP | SR_TXRDY;
    }

    /// Send the START and the device's address, then the internal
    /// address bytes if TWIHS_MMR asks for any. Returns the device's
    /// index, or `None` if the transfer failed.
    fn begin(&mut self) -> Option<usize> {
        self.status &= !(SR_TXCOMP | SR_NACK);
        let Some(index) = self.addressed_device() else {
            self.fail(None);
            return None;
        };
        self.devices[index].start();
        // Most significant byte first.
        let size = (self.mode >> MMR_IADRSZ_SHIFT) & 0x3;
        for i in (0..size).rev() {
            let byte = (self.internal_address >> (8 * i)) as u8;
            if !self.devices[index].write(byte) {
                self.fail(Some(index));
                return None;
            }
        }
        Some(index)
    }

    /// Firmware wrote a byte to TWIHS_THR: send it, starting a transfer
    /// if none is in progress.
    fn transmit(&mut self, byte: u8) {
        if !self.host_enabled || self.mode & MMR_MREAD != 0 {
            return;
        }
        self.status &= !(SR_TXRDY | SR_TXCOMP);
        let index = match self.transfer {
            Transfer::Writing(index) => index,
            _ => match self.begin() {
                Some(index) => index,
                None => return,
            },
        };
        if self.devices[index].write(byte) {
            self.transfer = Transfer::Writing(index);
            self.status |= SR_TXRDY;
        } else {
            self.fail(Some(index));
        }
    }

    /// Take the next byte from the device being read into TWIHS_RHR. If
    /// it is the last one, end the transfer.
    fn receive(&mut self) {
        let Transfer::Reading { device, last } = self.transfer else {
            return;
        };
        self.received = self.devices[device].read();
        self.status |= SR_RXRDY;
        if last {
            self.devices[device].stop();
            self.transfer = Transfer::Idle;
            self.status |= SR_TXCOMP;
        }
    }

    /// START was written to TWIHS_CR. With MREAD set this begins a read;
    /// `stop` says STOP was written with it, for a single byte.
    ///
    /// During a write it is a repeated start: the write ends without a
    /// STOP, and the settings now in TWIHS_MMR apply to what follows.
    fn start(&mut self, stop: bool) {
        if self.mode & MMR_MREAD == 0 {
            // A write begins with the first byte put in TWIHS_THR, not
            // here. Forgetting the transfer in progress makes that byte
            // send a new START.
            self.transfer = Transfer::Idle;
            return;
        }
        let Some(index) = self.begin() else {
            return;
        };
        // After the internal address, a repeated start turns the bus
        // round. Without one, the START in `begin` was already the
        // read's own.
        if (self.mode >> MMR_IADRSZ_SHIFT) & 0x3 != 0 {
            self.devices[index].start();
        }
        self.status &= !SR_TXRDY;
        self.transfer = Transfer::Reading {
            device: index,
            last: stop,
        };
        self.receive();
    }

    /// STOP was written to TWIHS_CR, on its own.
    fn stop(&mut self) {
        match self.transfer {
            Transfer::Idle => {}
            Transfer::Writing(index) => {
                self.devices[index].stop();
                self.transfer = Transfer::Idle;
                self.status |= SR_TXCOMP;
            }
            // The byte waiting in TWIHS_RHR is not the last: the
            // controller is already taking in the one after it. That
            // one will be.
            Transfer::Reading { device, .. } => {
                self.transfer = Transfer::Reading { device, last: true };
            }
        }
    }

    fn reset(&mut self) {
        if let Transfer::Writing(index) | Transfer::Reading { device: index, .. } = self.transfer {
            self.devices[index].stop();
        }
        self.host_enabled = false;
        self.mode = 0;
        self.internal_address = 0;
        self.clock_waveform = 0;
        self.interrupt_mask = 0;
        self.status = SR_TXCOMP;
        self.received = 0;
        self.transfer = Transfer::Idle;
    }
}

impl Peripheral for Twihs {
    /// All registers are 32 bits wide, so every access is treated as a
    /// whole-word access to the register containing `offset`.
    fn read(&mut self, offset: u32, _size: usize) -> u32 {
        match offset & !3 {
            TWIHS_MMR => self.mode,
            TWIHS_IADR => self.internal_address,
            TWIHS_CWGR => self.clock_waveform,
            TWIHS_SR => {
                let value = self.status | SR_FIXED;
                self.status &= !SR_NACK; // cleared by reading
                value
            }
            TWIHS_IMR => self.interrupt_mask,
            TWIHS_RHR => {
                let byte = self.received;
                // Reading the byte makes room for the next one, which
                // the controller takes in straight away.
                if self.status & SR_RXRDY != 0 {
                    self.status &= !SR_RXRDY;
                    self.receive();
                }
                byte as u32
            }
            _ => 0,
        }
    }

    fn write(&mut self, offset: u32, _size: usize, value: u32) {
        match offset & !3 {
            TWIHS_CR => {
                if value & CR_SWRST != 0 {
                    self.reset();
                    return;
                }
                if value & CR_MSEN != 0 {
                    self.host_enabled = true;
                    self.status |= SR_TXRDY;
                }
                // Written together with MSEN, MSDIS wins.
                if value & CR_MSDIS != 0 {
                    self.host_enabled = false;
                    self.status &= !SR_TXRDY;
                }
                if !self.host_enabled {
                    return;
                }
                if value & CR_START != 0 {
                    self.start(value & CR_STOP != 0);
                } else if value & CR_STOP != 0 {
                    self.stop();
                }
            }
            TWIHS_MMR => self.mode = value & MMR_MASK,
            TWIHS_IADR => self.internal_address = value & 0x00FF_FFFF,
            TWIHS_CWGR => self.clock_waveform = value,
            TWIHS_IER => self.interrupt_mask |= value & INTERRUPT_BITS,
            TWIHS_IDR => self.interrupt_mask &= !value,
            TWIHS_THR => self.transmit(value as u8),
            _ => {}
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::i2c::I2cMemory;
    use std::cell::RefCell;
    use std::rc::Rc;

    const MEMORY: u32 = 0x57; // address of the test device
    const ABSENT: u32 = 0x29; // an address nothing is attached at

    /// A controller in host mode with a 16-byte `I2cMemory` attached,
    /// holding 0x10, 0x11, ... 0x1F.
    fn twihs_with_memory() -> Twihs {
        let mut memory = I2cMemory::new(MEMORY as u8, 16);
        for (i, byte) in memory.data_mut().iter_mut().enumerate() {
            *byte = 0x10 + i as u8;
        }
        let mut twihs = Twihs::new();
        twihs.attach(Box::new(memory));
        twihs.write(TWIHS_CR, 4, CR_MSEN);
        twihs
    }

    /// TWIHS_MMR for a write to `address` with `iadrsz` internal
    /// address bytes.
    fn mmr_write(address: u32, iadrsz: u32) -> u32 {
        (address << MMR_DADR_SHIFT) | (iadrsz << MMR_IADRSZ_SHIFT)
    }

    fn mmr_read(address: u32, iadrsz: u32) -> u32 {
        mmr_write(address, iadrsz) | MMR_MREAD
    }

    fn status(twihs: &mut Twihs) -> u32 {
        twihs.read(TWIHS_SR, 4) & !SR_FIXED
    }

    /// Read `count` bytes the way firmware does: START (with STOP for a
    /// single byte), then for each byte wait for RXRDY and read
    /// TWIHS_RHR, setting STOP before reading the next-to-last one.
    fn read_bytes(twihs: &mut Twihs, count: usize) -> Vec<u8> {
        let single = count == 1;
        twihs.write(
            TWIHS_CR,
            4,
            if single { CR_START | CR_STOP } else { CR_START },
        );
        let mut bytes = Vec::new();
        for i in 0..count {
            assert_ne!(
                twihs.read(TWIHS_SR, 4) & SR_RXRDY,
                0,
                "byte {} not ready",
                i
            );
            if !single && i == count - 2 {
                twihs.write(TWIHS_CR, 4, CR_STOP);
            }
            bytes.push(twihs.read(TWIHS_RHR, 4) as u8);
        }
        bytes
    }

    /// Read register `register` onwards using the internal address.
    fn read_registers(twihs: &mut Twihs, register: u32, count: usize) -> Vec<u8> {
        twihs.write(TWIHS_MMR, 4, mmr_read(MEMORY, 1));
        twihs.write(TWIHS_IADR, 4, register);
        read_bytes(twihs, count)
    }

    #[test]
    fn status_after_reset_is_idle_with_the_bus_lines_high() {
        let mut twihs = Twihs::new();
        assert_eq!(twihs.read(TWIHS_SR, 4), 0x0300_0009); // the datasheet's reset value
    }

    #[test]
    fn enabling_host_mode_makes_the_transmitter_ready() {
        let mut twihs = Twihs::new();
        assert_eq!(status(&mut twihs), SR_TXCOMP);
        twihs.write(TWIHS_CR, 4, CR_MSEN);
        assert_eq!(status(&mut twihs), SR_TXCOMP | SR_TXRDY);
        twihs.write(TWIHS_CR, 4, CR_MSDIS);
        assert_eq!(status(&mut twihs), SR_TXCOMP);
    }

    #[test]
    fn nothing_happens_until_host_mode_is_enabled() {
        let mut twihs = Twihs::new();
        twihs.attach(Box::new(I2cMemory::new(MEMORY as u8, 16)));
        twihs.write(TWIHS_MMR, 4, mmr_read(MEMORY, 0));
        twihs.write(TWIHS_CR, 4, CR_START | CR_STOP);
        assert_eq!(status(&mut twihs), SR_TXCOMP);
    }

    #[test]
    fn single_register_read_with_internal_address() {
        let mut twihs = twihs_with_memory();
        assert_eq!(read_registers(&mut twihs, 5, 1), [0x15]);
        // The transfer is over.
        assert_eq!(status(&mut twihs) & SR_TXCOMP, SR_TXCOMP);
        assert_eq!(status(&mut twihs) & SR_RXRDY, 0);
    }

    #[test]
    fn multi_byte_read_follows_on_from_the_register() {
        let mut twihs = twihs_with_memory();
        assert_eq!(read_registers(&mut twihs, 3, 4), [0x13, 0x14, 0x15, 0x16]);
        assert_eq!(read_registers(&mut twihs, 9, 2), [0x19, 0x1A]);
        assert_eq!(
            read_registers(&mut twihs, 0, 6),
            [0x10, 0x11, 0x12, 0x13, 0x14, 0x15]
        );
    }

    #[test]
    fn read_status_bits_follow_the_datasheet_sequence() {
        let mut twihs = twihs_with_memory();
        twihs.write(TWIHS_MMR, 4, mmr_read(MEMORY, 1));
        twihs.write(TWIHS_IADR, 4, 0);
        twihs.write(TWIHS_CR, 4, CR_START);
        // First byte in; the transfer is still going.
        assert_eq!(status(&mut twihs), SR_RXRDY);
        twihs.write(TWIHS_CR, 4, CR_STOP);
        assert_eq!(status(&mut twihs), SR_RXRDY); // one more byte to come
        assert_eq!(twihs.read(TWIHS_RHR, 4), 0x10);
        // The last byte is in, and with it the transfer has ended.
        assert_eq!(status(&mut twihs), SR_RXRDY | SR_TXCOMP);
        assert_eq!(twihs.read(TWIHS_RHR, 4), 0x11);
        assert_eq!(status(&mut twihs), SR_TXCOMP);
    }

    #[test]
    fn exactly_the_bytes_asked_for_are_taken_from_the_device() {
        /// Counts how often it is read and stopped.
        struct Counter(Rc<RefCell<(u32, u32)>>);
        impl I2cDevice for Counter {
            fn address(&self) -> u8 {
                MEMORY as u8
            }
            fn start(&mut self) {}
            fn write(&mut self, _byte: u8) -> bool {
                true
            }
            fn read(&mut self) -> u8 {
                self.0.borrow_mut().0 += 1;
                0
            }
            fn stop(&mut self) {
                self.0.borrow_mut().1 += 1;
            }
        }
        for count in 1..=5 {
            let counts = Rc::new(RefCell::new((0, 0)));
            let mut twihs = Twihs::new();
            twihs.attach(Box::new(Counter(counts.clone())));
            twihs.write(TWIHS_CR, 4, CR_MSEN);
            twihs.write(TWIHS_MMR, 4, mmr_read(MEMORY, 0));
            read_bytes(&mut twihs, count);
            assert_eq!(
                *counts.borrow(),
                (count as u32, 1),
                "reading {} bytes",
                count
            );
        }
    }

    #[test]
    fn stop_set_late_costs_one_extra_byte() {
        // The hazard the datasheet warns about: if STOP is set only
        // after the next-to-last byte has been read, the controller has
        // already begun the byte after, so one more than wanted arrives.
        let mut twihs = twihs_with_memory();
        twihs.write(TWIHS_MMR, 4, mmr_read(MEMORY, 1));
        twihs.write(TWIHS_IADR, 4, 0);
        twihs.write(TWIHS_CR, 4, CR_START);
        assert_eq!(twihs.read(TWIHS_RHR, 4), 0x10);
        assert_eq!(twihs.read(TWIHS_RHR, 4), 0x11); // the "last" byte wanted
        twihs.write(TWIHS_CR, 4, CR_STOP); // too late
        assert_eq!(status(&mut twihs), SR_RXRDY); // 0x12 is waiting, not finished
        assert_eq!(twihs.read(TWIHS_RHR, 4), 0x12);
        assert_eq!(status(&mut twihs), SR_RXRDY | SR_TXCOMP);
        assert_eq!(twihs.read(TWIHS_RHR, 4), 0x13); // the extra one
    }

    #[test]
    fn register_write_with_internal_address() {
        let mut twihs = twihs_with_memory();
        twihs.write(TWIHS_MMR, 4, mmr_write(MEMORY, 1));
        twihs.write(TWIHS_IADR, 4, 4);
        twihs.write(TWIHS_THR, 4, 0xAA);
        assert_eq!(status(&mut twihs), SR_TXRDY); // sent; the transfer is still open
        twihs.write(TWIHS_THR, 4, 0xBB);
        twihs.write(TWIHS_CR, 4, CR_STOP);
        assert_eq!(status(&mut twihs), SR_TXRDY | SR_TXCOMP);
        assert_eq!(read_registers(&mut twihs, 3, 4), [0x13, 0xAA, 0xBB, 0x16]);
    }

    #[test]
    fn register_number_sent_as_data_then_a_repeated_start_to_read() {
        // The other way to read a register, with no internal address:
        // write the register number, then START again in read mode
        // without a STOP in between.
        let mut twihs = twihs_with_memory();
        twihs.write(TWIHS_MMR, 4, mmr_write(MEMORY, 0));
        twihs.write(TWIHS_THR, 4, 7); // the register number
        assert_eq!(status(&mut twihs), SR_TXRDY);
        twihs.write(TWIHS_MMR, 4, mmr_read(MEMORY, 0));
        assert_eq!(read_bytes(&mut twihs, 3), [0x17, 0x18, 0x19]);
    }

    #[test]
    fn write_without_internal_address_uses_the_first_byte_as_the_register() {
        let mut twihs = twihs_with_memory();
        twihs.write(TWIHS_MMR, 4, mmr_write(MEMORY, 0));
        twihs.write(TWIHS_THR, 4, 2);
        twihs.write(TWIHS_THR, 4, 0xEE);
        twihs.write(TWIHS_CR, 4, CR_STOP);
        assert_eq!(read_registers(&mut twihs, 2, 1), [0xEE]);
    }

    #[test]
    fn each_write_transfer_starts_afresh_after_a_stop() {
        let mut twihs = twihs_with_memory();
        twihs.write(TWIHS_MMR, 4, mmr_write(MEMORY, 1));
        for (register, value) in [(1, 0x01), (9, 0x09)] {
            twihs.write(TWIHS_IADR, 4, register);
            twihs.write(TWIHS_THR, 4, value);
            twihs.write(TWIHS_CR, 4, CR_STOP);
        }
        assert_eq!(read_registers(&mut twihs, 1, 2), [0x01, 0x12]);
        assert_eq!(read_registers(&mut twihs, 9, 1), [0x09]);
    }

    #[test]
    fn absent_device_gives_nack_on_a_read() {
        let mut twihs = twihs_with_memory();
        twihs.write(TWIHS_MMR, 4, mmr_read(ABSENT, 1));
        twihs.write(TWIHS_CR, 4, CR_START | CR_STOP);
        assert_eq!(status(&mut twihs), SR_NACK | SR_TXCOMP | SR_TXRDY);
        // NACK is cleared by having been read.
        assert_eq!(status(&mut twihs), SR_TXCOMP | SR_TXRDY);
    }

    #[test]
    fn absent_device_gives_nack_on_a_write() {
        let mut twihs = twihs_with_memory();
        twihs.write(TWIHS_MMR, 4, mmr_write(ABSENT, 0));
        twihs.write(TWIHS_THR, 4, 0);
        assert_eq!(status(&mut twihs), SR_NACK | SR_TXCOMP | SR_TXRDY);
    }

    #[test]
    fn scanning_the_bus_finds_the_attached_devices() {
        let mut twihs = twihs_with_memory();
        twihs.attach(Box::new(I2cMemory::new(0x29, 4)));
        let mut found = Vec::new();
        for address in 0..0x80 {
            twihs.write(TWIHS_MMR, 4, mmr_read(address, 0));
            twihs.write(TWIHS_CR, 4, CR_START | CR_STOP);
            if twihs.read(TWIHS_SR, 4) & SR_NACK == 0 {
                found.push(address);
                twihs.read(TWIHS_RHR, 4);
            }
        }
        assert_eq!(found, [0x29, 0x57]);
    }

    #[test]
    fn the_right_device_is_picked_by_address() {
        let mut twihs = twihs_with_memory();
        let mut other = I2cMemory::new(0x29, 4);
        other.data_mut()[0] = 0xA0;
        twihs.attach(Box::new(other));
        twihs.write(TWIHS_MMR, 4, mmr_read(0x29, 1));
        twihs.write(TWIHS_IADR, 4, 0);
        assert_eq!(read_bytes(&mut twihs, 1), [0xA0]);
        assert_eq!(read_registers(&mut twihs, 0, 1), [0x10]);
    }

    #[test]
    fn byte_refused_by_the_device_gives_nack() {
        // Register 16 does not exist in a 16-byte memory.
        let mut twihs = twihs_with_memory();
        twihs.write(TWIHS_MMR, 4, mmr_read(MEMORY, 1));
        twihs.write(TWIHS_IADR, 4, 16);
        twihs.write(TWIHS_CR, 4, CR_START | CR_STOP);
        assert_eq!(status(&mut twihs), SR_NACK | SR_TXCOMP | SR_TXRDY);
        // The controller and the device both still work.
        assert_eq!(read_registers(&mut twihs, 1, 1), [0x11]);
    }

    #[test]
    fn two_byte_internal_address_is_sent_high_byte_first() {
        /// Records every byte written to it.
        struct Recorder(Rc<RefCell<Vec<u8>>>);
        impl I2cDevice for Recorder {
            fn address(&self) -> u8 {
                MEMORY as u8
            }
            fn start(&mut self) {}
            fn write(&mut self, byte: u8) -> bool {
                self.0.borrow_mut().push(byte);
                true
            }
            fn read(&mut self) -> u8 {
                0
            }
        }
        let written = Rc::new(RefCell::new(Vec::new()));
        let mut twihs = Twihs::new();
        twihs.attach(Box::new(Recorder(written.clone())));
        twihs.write(TWIHS_CR, 4, CR_MSEN);
        twihs.write(TWIHS_MMR, 4, mmr_write(MEMORY, 2));
        twihs.write(TWIHS_IADR, 4, 0x1234);
        twihs.write(TWIHS_THR, 4, 0x56);
        twihs.write(TWIHS_CR, 4, CR_STOP);
        assert_eq!(*written.borrow(), [0x12, 0x34, 0x56]);
    }

    #[test]
    fn interrupt_is_requested_for_enabled_status_bits() {
        let mut twihs = twihs_with_memory();
        twihs.write(TWIHS_IER, 4, SR_RXRDY);
        assert_eq!(twihs.read(TWIHS_IMR, 4), SR_RXRDY);
        assert!(!twihs.interrupt_requested());
        twihs.write(TWIHS_MMR, 4, mmr_read(MEMORY, 1));
        twihs.write(TWIHS_IADR, 4, 0);
        twihs.write(TWIHS_CR, 4, CR_START | CR_STOP);
        assert!(twihs.interrupt_requested());
        twihs.read(TWIHS_RHR, 4); // dealing with it withdraws the request
        assert!(!twihs.interrupt_requested());

        twihs.write(TWIHS_IDR, 4, SR_RXRDY);
        twihs.write(TWIHS_IER, 4, SR_NACK);
        twihs.write(TWIHS_MMR, 4, mmr_read(ABSENT, 0));
        twihs.write(TWIHS_CR, 4, CR_START | CR_STOP);
        assert!(twihs.interrupt_requested());
        twihs.read(TWIHS_SR, 4);
        assert!(!twihs.interrupt_requested());
    }

    #[test]
    fn configuration_registers_read_back() {
        let mut twihs = Twihs::new();
        twihs.write(TWIHS_MMR, 4, 0xFFFF_FFFF);
        assert_eq!(twihs.read(TWIHS_MMR, 4), MMR_MASK);
        twihs.write(TWIHS_IADR, 4, 0xFFFF_FFFF);
        assert_eq!(twihs.read(TWIHS_IADR, 4), 0x00FF_FFFF);
        twihs.write(TWIHS_CWGR, 4, 0x0001_2345);
        assert_eq!(twihs.read(TWIHS_CWGR, 4), 0x0001_2345);
    }

    #[test]
    fn software_reset_returns_everything_to_its_reset_state() {
        let mut twihs = twihs_with_memory();
        twihs.write(TWIHS_MMR, 4, mmr_read(MEMORY, 1));
        twihs.write(TWIHS_IER, 4, SR_RXRDY);
        twihs.write(TWIHS_CR, 4, CR_START); // leave a read half done
        twihs.write(TWIHS_CR, 4, CR_SWRST);
        assert_eq!(twihs.read(TWIHS_SR, 4), 0x0300_0009);
        assert_eq!(twihs.read(TWIHS_MMR, 4), 0);
        assert_eq!(twihs.read(TWIHS_IMR, 4), 0);
        // The device is still attached and usable.
        twihs.write(TWIHS_CR, 4, CR_MSEN);
        assert_eq!(read_registers(&mut twihs, 2, 1), [0x12]);
    }

    #[test]
    fn devices_are_told_when_time_passes() {
        struct Clock(Rc<RefCell<u32>>);
        impl I2cDevice for Clock {
            fn address(&self) -> u8 {
                1
            }
            fn start(&mut self) {}
            fn write(&mut self, _byte: u8) -> bool {
                true
            }
            fn read(&mut self) -> u8 {
                0
            }
            fn tick(&mut self, cycles: u32) {
                *self.0.borrow_mut() += cycles;
            }
        }
        let elapsed = Rc::new(RefCell::new(0));
        let mut twihs = Twihs::new();
        twihs.attach(Box::new(Clock(elapsed.clone())));
        twihs.tick(100);
        twihs.tick(23);
        assert_eq!(*elapsed.borrow(), 123);
    }
}
