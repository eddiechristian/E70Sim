pub trait Peripheral {
    /// Read from the peripheral at a given byte offset.
    /// `size` will be 1 (byte), 2 (halfword), or 4 (word).
    fn read(&mut self, offset: u32, size: usize) -> u32;

    /// Write a value to the peripheral at a given byte offset.
    /// `size` will be 1, 2, or 4.
    fn write(&mut self, offset: u32, size: usize, value: u32);
}
