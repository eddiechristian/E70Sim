use crate::peripheral::Peripheral;

pub struct Uart0Stub;

impl Peripheral for Uart0Stub {
    fn read(&mut self, offset: u32, _size: usize) -> u32 {
        match offset {
            0x14 => {
                // UART_SR (Status Register)
                // Bit 1 = TXRDY (Transmit Ready)
                // Bit 9 = TXEMPTY (Transmit Empty)
                (1 << 1) | (1 << 9)
            }
            _ => {
                // Return 0 for other unhandled reads (like variant/version IDs)
                0
            }
        }
    }

    fn write(&mut self, offset: u32, _size: usize, value: u32) {
        match offset {
            0x1C => {
                // UART_THR (Transmit Holding Register)
                // Cast the written value to a character and print to stdout immediately
                let c = (value & 0xFF) as u8 as char;
                print!("{}", c);
                use std::io::Write;
                std::io::stdout().flush().unwrap(); // Ensure it prints instantly without buffering
            }
            _ => {
                // Store-and-readback simulation or quiet ignore for control registers
            }
        }
    }
}
