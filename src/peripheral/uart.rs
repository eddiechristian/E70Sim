//! UART0, base 0x400E_0800.
//!
//! Transmit: the transmitter is always ready, and every byte written to
//! `UART_THR` goes straight to the host's stdout.
//!
//! Receive: bytes come from an input channel, which `main` connects to the
//! host's stdin. One byte at a time is moved into `UART_RHR`, which sets
//! RXRDY in `UART_SR`; reading `UART_RHR` clears RXRDY and makes room for
//! the next byte.
//!
//! Interrupt: the UART asks for its interrupt (IRQ 7) for as long as any
//! status bit that is set in `UART_SR` is also enabled in `UART_IMR`. The
//! bus passes that request on to the NVIC.
//!
//! Baud rate, parity and the pins are not modelled: `UART_MR` and
//! `UART_BRGR` are ignored. Bytes are never lost: a byte that arrives
//! while `UART_RHR` is still full waits in the channel, where a real UART
//! would overwrite the old byte and set OVRE.
//!
//! Register offsets and bit positions are from `ATSAME70Q21.svd`.

use std::io::{Read, Write};
use std::sync::mpsc::{self, Receiver};

use crate::peripheral::Peripheral;

// Register offsets.
const UART_CR: u32 = 0x00; // Control (write-only)
const UART_IER: u32 = 0x08; // Interrupt Enable (write-only)
const UART_IDR: u32 = 0x0C; // Interrupt Disable (write-only)
const UART_IMR: u32 = 0x10; // Interrupt Mask (read-only)
const UART_SR: u32 = 0x14; // Status (read-only)
const UART_RHR: u32 = 0x18; // Receive Holding (read-only)
const UART_THR: u32 = 0x1C; // Transmit Holding (write-only)

// UART_CR bits.
const CR_RSTRX: u32 = 1 << 2; // reset and disable the receiver
const CR_RXEN: u32 = 1 << 4; // enable the receiver
const CR_RXDIS: u32 = 1 << 5; // disable the receiver

// UART_SR bits. UART_IER, UART_IDR and UART_IMR use the same positions.
const SR_RXRDY: u32 = 1 << 0; // a received byte is waiting in UART_RHR
const SR_TXRDY: u32 = 1 << 1; // UART_THR can take a byte
const SR_TXEMPTY: u32 = 1 << 9; // nothing is being sent

/// The interrupt sources that exist: RXRDY, TXRDY, OVRE, FRAME, PARE,
/// TXEMPTY and CMP.
const INTERRUPT_BITS: u32 =
    (1 << 0) | (1 << 1) | (1 << 5) | (1 << 6) | (1 << 7) | (1 << 9) | (1 << 15);

pub struct Uart0 {
    /// Where received bytes come from. `None` means nothing is connected,
    /// so nothing is ever received.
    input: Option<Receiver<u8>>,
    /// Set by RXEN, cleared by RXDIS and RSTRX. The receiver is off after
    /// reset.
    rx_enabled: bool,
    /// UART_RHR: the last byte received.
    rhr: u8,
    /// RXRDY: `rhr` holds a byte the firmware has not read yet.
    rx_ready: bool,
    /// UART_IMR: which status bits ask for the interrupt.
    interrupt_mask: u32,
}

impl Uart0 {
    pub fn new() -> Self {
        Uart0 {
            input: None,
            rx_enabled: false,
            rhr: 0,
            rx_ready: false,
            interrupt_mask: 0,
        }
    }

    /// Connect the receiver to a source of bytes.
    pub fn set_input(&mut self, input: Receiver<u8>) {
        self.input = Some(input);
    }

    /// Move the next waiting input byte into UART_RHR, if the receiver is
    /// on and UART_RHR is free. The bus calls this as emulated time passes.
    pub fn poll_input(&mut self) {
        if !self.rx_enabled || self.rx_ready {
            return;
        }
        if let Some(input) = &self.input
            && let Ok(byte) = input.try_recv()
        {
            self.rhr = byte;
            self.rx_ready = true;
        }
    }

    fn status(&self) -> u32 {
        let rxrdy = if self.rx_ready { SR_RXRDY } else { 0 };
        SR_TXRDY | SR_TXEMPTY | rxrdy
    }

    /// True while the UART is asking for its interrupt: some status bit is
    /// set and enabled in UART_IMR.
    pub fn interrupt_requested(&self) -> bool {
        self.status() & self.interrupt_mask != 0
    }
}

impl Peripheral for Uart0 {
    fn read(&mut self, offset: u32, _size: usize) -> u32 {
        match offset {
            UART_IMR => self.interrupt_mask,
            UART_SR => self.status(),
            UART_RHR => {
                // Reading the byte clears RXRDY.
                self.rx_ready = false;
                self.rhr as u32
            }
            // Everything else (the write-only registers, UART_MR,
            // UART_BRGR) reads 0.
            _ => 0,
        }
    }

    fn write(&mut self, offset: u32, _size: usize, value: u32) {
        match offset {
            UART_CR => {
                if value & CR_RSTRX != 0 {
                    self.rx_enabled = false;
                    self.rx_ready = false;
                }
                if value & CR_RXEN != 0 {
                    self.rx_enabled = true;
                }
                // Written together with RXEN, RXDIS wins.
                if value & CR_RXDIS != 0 {
                    self.rx_enabled = false;
                }
            }
            UART_IER => self.interrupt_mask |= value & INTERRUPT_BITS,
            UART_IDR => self.interrupt_mask &= !value,
            UART_THR => {
                // Send the byte to stdout at once, without buffering.
                let mut stdout = std::io::stdout();
                let _ = stdout.write_all(&[value as u8]);
                let _ = stdout.flush();
            }
            // UART_MR and UART_BRGR are ignored; the rest is read-only.
            _ => {}
        }
    }
}

/// Start a thread that reads the host's stdin and returns the channel its
/// bytes arrive on. The thread ends when stdin reaches end of file.
///
/// A terminal hands over input a line at a time, so typed characters
/// arrive when Enter is pressed.
pub fn stdin_bytes() -> Receiver<u8> {
    let (sender, receiver) = mpsc::channel();
    std::thread::spawn(move || {
        for byte in std::io::stdin().lock().bytes() {
            match byte {
                // The emulator stopped listening: nothing more to do.
                Ok(byte) if sender.send(byte).is_err() => break,
                Ok(_) => {}
                Err(_) => break,
            }
        }
    });
    receiver
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::mpsc::Sender;

    /// A UART with its receiver switched on and an input channel attached.
    fn receiving() -> (Uart0, Sender<u8>) {
        let (sender, receiver) = mpsc::channel();
        let mut uart = Uart0::new();
        uart.set_input(receiver);
        uart.write(UART_CR, 4, CR_RXEN);
        (uart, sender)
    }

    #[test]
    fn transmitter_is_always_ready() {
        let mut uart = Uart0::new();
        assert_eq!(uart.read(UART_SR, 4), SR_TXRDY | SR_TXEMPTY);
    }

    #[test]
    fn received_byte_sets_rxrdy_and_reading_it_clears_rxrdy() {
        let (mut uart, sender) = receiving();
        sender.send(b'a').unwrap();
        uart.poll_input();
        assert_ne!(uart.read(UART_SR, 4) & SR_RXRDY, 0);
        assert_eq!(uart.read(UART_RHR, 4), b'a' as u32);
        assert_eq!(uart.read(UART_SR, 4) & SR_RXRDY, 0);
    }

    #[test]
    fn bytes_are_handed_over_one_at_a_time_in_order() {
        let (mut uart, sender) = receiving();
        sender.send(b'a').unwrap();
        sender.send(b'b').unwrap();
        uart.poll_input();
        uart.poll_input(); // 'b' has to wait: UART_RHR is still full
        assert_eq!(uart.read(UART_RHR, 4), b'a' as u32);
        uart.poll_input();
        assert_eq!(uart.read(UART_RHR, 4), b'b' as u32);
        uart.poll_input();
        assert_eq!(uart.read(UART_SR, 4) & SR_RXRDY, 0);
    }

    #[test]
    fn nothing_is_received_while_the_receiver_is_off() {
        let (sender, receiver) = mpsc::channel();
        let mut uart = Uart0::new();
        uart.set_input(receiver);
        sender.send(b'a').unwrap();
        uart.poll_input(); // off after reset
        assert_eq!(uart.read(UART_SR, 4) & SR_RXRDY, 0);

        // The byte was kept, and arrives once the receiver is on.
        uart.write(UART_CR, 4, CR_RXEN);
        uart.poll_input();
        assert_eq!(uart.read(UART_RHR, 4), b'a' as u32);

        uart.write(UART_CR, 4, CR_RXDIS);
        sender.send(b'b').unwrap();
        uart.poll_input();
        assert_eq!(uart.read(UART_SR, 4) & SR_RXRDY, 0);
    }

    #[test]
    fn resetting_the_receiver_drops_the_waiting_byte() {
        let (mut uart, sender) = receiving();
        sender.send(b'a').unwrap();
        uart.poll_input();
        uart.write(UART_CR, 4, CR_RSTRX);
        assert_eq!(uart.read(UART_SR, 4) & SR_RXRDY, 0);
    }

    #[test]
    fn uart_with_no_input_connected_receives_nothing() {
        let mut uart = Uart0::new();
        uart.write(UART_CR, 4, CR_RXEN);
        uart.poll_input();
        assert_eq!(uart.read(UART_SR, 4) & SR_RXRDY, 0);
    }

    #[test]
    fn interrupt_enable_and_disable_show_in_the_mask() {
        let mut uart = Uart0::new();
        assert_eq!(uart.read(UART_IMR, 4), 0);
        uart.write(UART_IER, 4, SR_RXRDY | SR_TXEMPTY);
        assert_eq!(uart.read(UART_IMR, 4), SR_RXRDY | SR_TXEMPTY);
        uart.write(UART_IDR, 4, SR_TXEMPTY);
        assert_eq!(uart.read(UART_IMR, 4), SR_RXRDY);
        // Bits that are not interrupt sources do not stick.
        uart.write(UART_IER, 4, 0xFFFF_FFFF);
        assert_eq!(uart.read(UART_IMR, 4), INTERRUPT_BITS);
    }

    #[test]
    fn interrupt_is_requested_while_an_enabled_status_bit_is_set() {
        let (mut uart, sender) = receiving();
        sender.send(b'a').unwrap();
        uart.poll_input();
        // RXRDY is set but not enabled.
        assert!(!uart.interrupt_requested());

        uart.write(UART_IER, 4, SR_RXRDY);
        assert!(uart.interrupt_requested());

        // Reading the byte clears RXRDY and so withdraws the request.
        uart.read(UART_RHR, 4);
        assert!(!uart.interrupt_requested());
    }

    #[test]
    fn enabled_txrdy_requests_the_interrupt_all_the_time() {
        let mut uart = Uart0::new();
        uart.write(UART_IER, 4, SR_TXRDY);
        assert!(uart.interrupt_requested());
        uart.write(UART_IDR, 4, SR_TXRDY);
        assert!(!uart.interrupt_requested());
    }
}
