//! The `Peripheral` trait and the peripheral stubs that implement it, one
//! file per peripheral in `src/peripheral/`. The bus maps each stub to its
//! address range.

pub mod efc;
pub mod matrix;
pub mod pio;
pub mod pmc;
pub mod rstc;
pub mod scb;
pub mod supc;
pub mod systick;
pub mod uart;
pub mod wdt;

pub trait Peripheral {
    /// Read from the peripheral at a given byte offset.
    /// `size` will be 1 (byte), 2 (halfword), or 4 (word).
    fn read(&mut self, offset: u32, size: usize) -> u32;

    /// Write a value to the peripheral at a given byte offset.
    /// `size` will be 1, 2, or 4.
    fn write(&mut self, offset: u32, size: usize, value: u32);
}
