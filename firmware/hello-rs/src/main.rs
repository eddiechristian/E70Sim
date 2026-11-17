//! Bare-metal firmware for the ATSAME70Q21: set up UART0 and print a line.
//!
//! See `../firmware.md` for what each step is for. Register addresses and
//! bit positions are from `ATSAME70Q21.svd`.

#![no_std] // no standard library: it needs an operating system
#![no_main] // no normal main: cortex-m-rt calls our #[entry] function

use core::ptr::{read_volatile, write_volatile};

use cortex_m_rt::entry;
use panic_halt as _; // on panic, loop forever

// Watchdog Timer.
const WDT_MR: *mut u32 = 0x400E_1854 as *mut u32;
const WDT_MR_WDDIS: u32 = 1 << 15; // watchdog disable

// Power Management Controller.
const PMC_PCER0: *mut u32 = 0x400E_0610 as *mut u32;
const ID_UART0: u32 = 7; // peripheral ID of UART0

// PIOA: UART0 uses PA9 (receive) and PA10 (transmit), peripheral function A.
const PIOA_PDR: *mut u32 = 0x400E_0E04 as *mut u32;
const PIOA_ABCDSR0: *mut u32 = 0x400E_0E70 as *mut u32;
const PIOA_ABCDSR1: *mut u32 = 0x400E_0E74 as *mut u32;
const UART0_PINS: u32 = (1 << 9) | (1 << 10);

// UART0.
const UART0_CR: *mut u32 = 0x400E_0800 as *mut u32;
const UART0_MR: *mut u32 = 0x400E_0804 as *mut u32;
const UART0_SR: *mut u32 = 0x400E_0814 as *mut u32;
const UART0_THR: *mut u32 = 0x400E_081C as *mut u32;
const UART0_BRGR: *mut u32 = 0x400E_0820 as *mut u32;

const UART_CR_RSTRX: u32 = 1 << 2; // reset receiver
const UART_CR_RSTTX: u32 = 1 << 3; // reset transmitter
const UART_CR_RXEN: u32 = 1 << 4; // enable receiver
const UART_CR_RXDIS: u32 = 1 << 5; // disable receiver
const UART_CR_TXEN: u32 = 1 << 6; // enable transmitter
const UART_CR_TXDIS: u32 = 1 << 7; // disable transmitter
const UART_MR_PAR_NO: u32 = 4 << 9; // no parity
const UART_SR_TXRDY: u32 = 1 << 1; // transmitter can take another byte

// The chip starts on its internal 12 MHz oscillator.
const CLOCK_HZ: u32 = 12_000_000;
const BAUD: u32 = 9600;
/// Clock divisor for UART_BRGR: clock / (16 x baud), rounded to nearest.
const UART_CD: u32 = (CLOCK_HZ + 8 * BAUD) / (16 * BAUD);

/// Write a hardware register.
fn write_reg(reg: *mut u32, value: u32) {
    // SAFETY: every `reg` passed in this file is the address of a real,
    // 32-bit register on the ATSAME70Q21.
    unsafe { write_volatile(reg, value) }
}

/// Read a hardware register.
fn read_reg(reg: *mut u32) -> u32 {
    // SAFETY: as for `write_reg`.
    unsafe { read_volatile(reg) }
}

fn uart_init() {
    // Peripherals have no clock until it is switched on.
    write_reg(PMC_PCER0, 1 << ID_UART0);

    // Hand the two pins from the PIO controller to peripheral function A
    // (a 0 bit in both select registers).
    write_reg(PIOA_ABCDSR0, read_reg(PIOA_ABCDSR0) & !UART0_PINS);
    write_reg(PIOA_ABCDSR1, read_reg(PIOA_ABCDSR1) & !UART0_PINS);
    write_reg(PIOA_PDR, UART0_PINS);

    // Reset and disable both directions while the settings change.
    write_reg(
        UART0_CR,
        UART_CR_RSTRX | UART_CR_RSTTX | UART_CR_RXDIS | UART_CR_TXDIS,
    );
    write_reg(UART0_MR, UART_MR_PAR_NO);
    write_reg(UART0_BRGR, UART_CD);
    write_reg(UART0_CR, UART_CR_RXEN | UART_CR_TXEN);
}

fn uart_write_byte(byte: u8) {
    // Wait until the transmitter can take another byte.
    while read_reg(UART0_SR) & UART_SR_TXRDY == 0 {}
    write_reg(UART0_THR, byte as u32);
}

fn uart_write_str(text: &str) {
    for byte in text.bytes() {
        uart_write_byte(byte);
    }
}

#[entry]
fn main() -> ! {
    // The watchdog is running at reset and would reset the chip after
    // about 16 seconds.
    write_reg(WDT_MR, WDT_MR_WDDIS);

    uart_init();
    uart_write_str("Hello from Rust on SAME70\n");

    loop {}
}
