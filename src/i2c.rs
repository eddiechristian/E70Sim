//! I2C devices: things attached to an I2C bus, as seen from the
//! controller's side.
//!
//! I2C is a two-wire bus (clock and data) shared by one controller and
//! any number of devices. Each device has a 7-bit address. A transfer
//! goes:
//!
//! 1. START, then the address of one device and a direction (write or
//!    read). The device with that address answers; the rest ignore the
//!    transfer.
//! 2. Bytes in that direction, each one acknowledged by the receiver.
//! 3. Either STOP, which ends it, or another START (a "repeated start")
//!    to carry on with the same or another device, usually to change
//!    direction.
//!
//! The `I2cDevice` trait is the device's half of that. The controller
//! (the SAME70's TWIHS peripheral, `src/peripheral/twihs.rs`) picks the device
//! by `address` and then calls `start`, `write` or `read` once per byte,
//! and `stop`.
//!
//! Nearly every I2C sensor and memory chip uses the bytes the same way:
//! the first byte written after a START is a register number, further
//! bytes written go into that register and the ones after it, and bytes
//! read come from that register onwards. `I2cMemory` below is the
//! simplest device of that kind.

/// A device on an I2C bus.
pub trait I2cDevice {
    /// The device's 7-bit address.
    fn address(&self) -> u8;

    /// A transfer addressed to this device begins: a START, or a
    /// repeated start, has been sent with this device's address.
    fn start(&mut self);

    /// The controller sends a byte. Returns true to acknowledge it (ACK)
    /// or false to refuse it (NACK).
    fn write(&mut self, byte: u8) -> bool;

    /// The controller asks for a byte.
    fn read(&mut self) -> u8;

    /// The transfer has ended with a STOP. Most devices have nothing to
    /// do.
    fn stop(&mut self) {}

    /// Let emulated time pass, for devices whose behaviour depends on
    /// it. Most do not.
    fn tick(&mut self, _cycles: u32) {}

    /// The level of the device's interrupt output pin: a separate wire
    /// from the I2C bus, which some devices use to get the processor's
    /// attention. `None` if the device has no such pin.
    fn interrupt_line(&self) -> Option<bool> {
        None
    }

    /// How many clock cycles from now the device will next do something
    /// on its own, such as take a measurement, or `None` if it only
    /// ever reacts. A sleeping CPU uses this to skip the wait.
    fn cycles_until_event(&self) -> Option<u32> {
        None
    }
}

/// A small memory on the I2C bus: `size` bytes, each one a "register".
///
/// The first byte written after a START chooses a register. Bytes
/// written after that are stored there and in the following registers;
/// bytes read come from there onwards. After the last register the
/// position goes back to the first.
///
/// It stands in for a real device in tests, and behaves much as a small
/// EEPROM chip does. Nothing outside the tests uses it yet.
#[cfg(test)]
pub struct I2cMemory {
    address: u8,
    data: Vec<u8>,
    /// The register the next byte is read from or written to.
    position: usize,
    /// True after a START, until a byte is written: that byte will be a
    /// register number, not data.
    expecting_register: bool,
}

#[cfg(test)]
impl I2cMemory {
    /// A memory of `size` zero bytes at I2C address `address`. `size`
    /// must be from 1 to 256, so that one byte can name any register.
    pub fn new(address: u8, size: usize) -> Self {
        assert!(
            (1..=256).contains(&size),
            "an I2cMemory holds 1 to 256 bytes"
        );
        I2cMemory {
            address,
            data: vec![0; size],
            position: 0,
            expecting_register: false,
        }
    }

    /// The stored bytes, for looking at from outside the bus.
    pub fn data(&self) -> &[u8] {
        &self.data
    }

    /// Change the stored bytes from outside the bus.
    pub fn data_mut(&mut self) -> &mut [u8] {
        &mut self.data
    }

    /// Move on to the next register, going round at the end.
    fn advance(&mut self) {
        self.position = (self.position + 1) % self.data.len();
    }
}

#[cfg(test)]
impl I2cDevice for I2cMemory {
    fn address(&self) -> u8 {
        self.address
    }

    fn start(&mut self) {
        // The position is kept, so that "write a register number, then
        // start again and read" reads from that register.
        self.expecting_register = true;
    }

    fn write(&mut self, byte: u8) -> bool {
        if self.expecting_register {
            // A register number that does not exist is refused.
            if byte as usize >= self.data.len() {
                return false;
            }
            self.position = byte as usize;
            self.expecting_register = false;
        } else {
            self.data[self.position] = byte;
            self.advance();
        }
        true
    }

    fn read(&mut self) -> u8 {
        let byte = self.data[self.position];
        self.advance();
        byte
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// What a controller does to write `bytes` starting at `register`.
    fn write_registers(device: &mut dyn I2cDevice, register: u8, bytes: &[u8]) -> bool {
        device.start();
        let mut acknowledged = device.write(register);
        for &byte in bytes {
            acknowledged &= device.write(byte);
        }
        device.stop();
        acknowledged
    }

    /// What a controller does to read `count` bytes starting at
    /// `register`: write the register number, then a repeated start and
    /// the reads.
    fn read_registers(device: &mut dyn I2cDevice, register: u8, count: usize) -> Vec<u8> {
        device.start();
        assert!(device.write(register));
        device.start();
        let bytes = (0..count).map(|_| device.read()).collect();
        device.stop();
        bytes
    }

    #[test]
    fn memory_reports_its_address() {
        let memory = I2cMemory::new(0x57, 16);
        assert_eq!(memory.address(), 0x57);
    }

    #[test]
    fn memory_starts_as_zeros() {
        let mut memory = I2cMemory::new(0x57, 4);
        assert_eq!(read_registers(&mut memory, 0, 4), [0, 0, 0, 0]);
    }

    #[test]
    fn first_byte_written_chooses_the_register() {
        let mut memory = I2cMemory::new(0x57, 16);
        assert!(write_registers(&mut memory, 5, &[0xAB]));
        assert_eq!(memory.data()[5], 0xAB);
        // The register number itself was not stored anywhere.
        assert_eq!(memory.data().iter().filter(|&&b| b != 0).count(), 1);
    }

    #[test]
    fn further_bytes_go_into_the_following_registers() {
        let mut memory = I2cMemory::new(0x57, 16);
        assert!(write_registers(&mut memory, 3, &[1, 2, 3]));
        assert_eq!(memory.data()[3..6], [1, 2, 3]);
    }

    #[test]
    fn reads_start_at_the_chosen_register_and_move_on() {
        let mut memory = I2cMemory::new(0x57, 16);
        memory.data_mut()[8..12].copy_from_slice(&[10, 20, 30, 40]);
        assert_eq!(read_registers(&mut memory, 9, 3), [20, 30, 40]);
    }

    #[test]
    fn what_is_written_is_read_back() {
        let mut memory = I2cMemory::new(0x57, 16);
        write_registers(&mut memory, 2, &[0xDE, 0xAD, 0xBE, 0xEF]);
        assert_eq!(read_registers(&mut memory, 2, 4), [0xDE, 0xAD, 0xBE, 0xEF]);
        // One register in the middle.
        assert_eq!(read_registers(&mut memory, 4, 1), [0xBE]);
    }

    #[test]
    fn a_read_with_no_register_chosen_carries_on_from_the_last_position() {
        let mut memory = I2cMemory::new(0x57, 16);
        memory.data_mut()[0..4].copy_from_slice(&[1, 2, 3, 4]);
        assert_eq!(read_registers(&mut memory, 0, 2), [1, 2]);
        // A new transfer that only reads.
        memory.start();
        assert_eq!(memory.read(), 3);
        assert_eq!(memory.read(), 4);
        memory.stop();
    }

    #[test]
    fn position_goes_round_at_the_end() {
        let mut memory = I2cMemory::new(0x57, 4);
        write_registers(&mut memory, 2, &[7, 8, 9]); // registers 2, 3 and 0
        assert_eq!(memory.data(), [9, 0, 7, 8]);
        assert_eq!(read_registers(&mut memory, 3, 2), [8, 9]);
    }

    #[test]
    fn a_register_number_that_does_not_exist_is_refused() {
        let mut memory = I2cMemory::new(0x57, 4);
        assert!(!write_registers(&mut memory, 4, &[]));
        assert_eq!(memory.data(), [0, 0, 0, 0]);
        // The device still works afterwards.
        assert!(write_registers(&mut memory, 3, &[1]));
        assert_eq!(memory.data()[3], 1);
    }

    #[test]
    fn each_start_expects_a_register_number_again() {
        let mut memory = I2cMemory::new(0x57, 16);
        write_registers(&mut memory, 1, &[0x11]);
        // In the second transfer the first byte is a register number
        // again, not data for register 2.
        write_registers(&mut memory, 6, &[0x66]);
        assert_eq!(memory.data()[1], 0x11);
        assert_eq!(memory.data()[2], 0);
        assert_eq!(memory.data()[6], 0x66);
    }

    #[test]
    fn a_full_256_byte_memory_can_be_addressed() {
        let mut memory = I2cMemory::new(0x57, 256);
        assert!(write_registers(&mut memory, 255, &[0xFF, 0x01])); // 255, then round to 0
        assert_eq!(memory.data()[255], 0xFF);
        assert_eq!(memory.data()[0], 0x01);
    }

    #[test]
    fn devices_can_be_used_through_the_trait() {
        // The controller will hold devices of different kinds in one
        // list, so the trait has to work as a boxed object.
        let mut devices: Vec<Box<dyn I2cDevice>> = vec![
            Box::new(I2cMemory::new(0x29, 8)),
            Box::new(I2cMemory::new(0x57, 8)),
        ];
        let device = devices.iter_mut().find(|d| d.address() == 0x57).unwrap();
        assert!(write_registers(device.as_mut(), 0, &[0x42]));
        assert_eq!(read_registers(device.as_mut(), 0, 1), [0x42]);
        // The other device was not touched.
        assert_eq!(read_registers(devices[0].as_mut(), 0, 1), [0]);
        // `tick` and `stop` have defaults that do nothing.
        devices[0].tick(1000);
    }
}
