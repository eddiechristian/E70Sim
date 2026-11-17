//! Bare-metal firmware for the ATSAME70Q21: set up UART0, print a line,
//! start a 1 ms SysTick, then echo every byte received on UART0.
//!
//! The SysTick exception handler only counts milliseconds; `millis` and
//! `delay_ms` are built on that count. The UART0 interrupt handler does
//! the echoing, so the main loop has nothing to do but sleep.
//!
//! See `../firmware.md` for what each step is for. Register addresses and
//! bit positions are from `ATSAME70Q21.svd`.

#![no_std] // no standard library: it needs an operating system
#![no_main] // no normal main: cortex-m-rt calls our #[entry] function

use core::ptr::{read_volatile, write_volatile};
use core::sync::atomic::{AtomicU32, Ordering};

use cortex_m_rt::{entry, exception};
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
const UART0_IER: *mut u32 = 0x400E_0808 as *mut u32; // interrupt enable
const UART0_SR: *mut u32 = 0x400E_0814 as *mut u32;
const UART0_RHR: *mut u32 = 0x400E_0818 as *mut u32; // received byte
const UART0_THR: *mut u32 = 0x400E_081C as *mut u32;
const UART0_BRGR: *mut u32 = 0x400E_0820 as *mut u32;

const UART_CR_RSTRX: u32 = 1 << 2; // reset receiver
const UART_CR_RSTTX: u32 = 1 << 3; // reset transmitter
const UART_CR_RXEN: u32 = 1 << 4; // enable receiver
const UART_CR_RXDIS: u32 = 1 << 5; // disable receiver
const UART_CR_TXEN: u32 = 1 << 6; // enable transmitter
const UART_CR_TXDIS: u32 = 1 << 7; // disable transmitter
const UART_MR_PAR_NO: u32 = 4 << 9; // no parity
const UART_SR_RXRDY: u32 = 1 << 0; // a received byte is waiting in UART0_RHR
const UART_SR_TXRDY: u32 = 1 << 1; // transmitter can take another byte

// NVIC, the Arm core's interrupt controller.
const NVIC_ISER0: *mut u32 = 0xE000_E100 as *mut u32; // enable IRQ 0 to 31
const IRQ_UART0: u32 = 7; // interrupt number of UART0

// SysTick, the Arm core's system timer.
const SYST_CSR: *mut u32 = 0xE000_E010 as *mut u32; // control and status
const SYST_RVR: *mut u32 = 0xE000_E014 as *mut u32; // reload value
const SYST_CVR: *mut u32 = 0xE000_E018 as *mut u32; // current value

const SYST_CSR_ENABLE: u32 = 1 << 0; // counter runs
const SYST_CSR_TICKINT: u32 = 1 << 1; // reaching zero raises the exception
const SYST_CSR_CLKSOURCE: u32 = 1 << 2; // count the processor clock

// The chip starts on its internal 12 MHz oscillator.
const CLOCK_HZ: u32 = 12_000_000;

/// SysTick exceptions per second: one every millisecond.
const TICK_HZ: u32 = 1_000;
/// Clock cycles between SysTick exceptions.
const TICK_CYCLES: u32 = CLOCK_HZ / TICK_HZ;

/// Milliseconds since `systick_init`, counted by the SysTick handler.
/// It wraps round to 0 after about 49.7 days.
///
/// An atomic, because it is shared between the handler, which writes it,
/// and the main code, which reads it.
static TICKS: AtomicU32 = AtomicU32::new(0);
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

/// Have UART0 raise its interrupt whenever a byte has been received.
/// There are two switches: one in the UART, one in the NVIC.
fn uart_enable_rx_interrupt() {
    write_reg(UART0_IER, UART_SR_RXRDY);
    write_reg(NVIC_ISER0, 1 << IRQ_UART0);
}

/// Runs when UART0 has received a byte: send it straight back.
///
/// Reading UART0_RHR is what makes the UART stop asking for the
/// interrupt. Without that read the handler would run again as soon as
/// it returned, for ever.
///
/// `no_mangle` keeps the name `UART0` in the compiled program, and
/// `extern "C"` makes it an ordinary function that the processor can call
/// through the vector table.
#[unsafe(no_mangle)]
#[allow(non_snake_case)]
extern "C" fn UART0() {
    if read_reg(UART0_SR) & UART_SR_RXRDY != 0 {
        let byte = read_reg(UART0_RHR) as u8;
        uart_write_byte(byte);
    }
}

unsafe extern "C" {
    /// `cortex-m-rt`'s handler for anything unexpected: it loops for ever.
    fn DefaultHandler();
}

/// The interrupt part of the vector table: entry n is the handler for
/// IRQ n. `cortex-m-rt` places this section straight after the 16 system
/// exception entries, so entry 7 lands at word 23 (`0x0040_005C`).
///
/// The table stops after the last interrupt this firmware uses.
#[unsafe(link_section = ".vector_table.interrupts")]
#[unsafe(no_mangle)]
#[used]
pub static __INTERRUPTS: [unsafe extern "C" fn(); 8] = [
    DefaultHandler, // IRQ 0
    DefaultHandler, // IRQ 1
    DefaultHandler, // IRQ 2
    DefaultHandler, // IRQ 3
    DefaultHandler, // IRQ 4
    DefaultHandler, // IRQ 5
    DefaultHandler, // IRQ 6
    UART0,          // IRQ 7
];

fn systick_init() {
    // The counter runs from the reload value down to zero, so a period of
    // N cycles needs a reload value of N - 1.
    write_reg(SYST_RVR, TICK_CYCLES - 1);
    // Writing any value clears the counter, so the first period is a full one.
    write_reg(SYST_CVR, 0);
    write_reg(
        SYST_CSR,
        SYST_CSR_ENABLE | SYST_CSR_TICKINT | SYST_CSR_CLKSOURCE,
    );
}

/// Runs every time SysTick reaches zero, once a millisecond. `cortex-m-rt`
/// puts this function in the vector table as the handler for exception 15.
#[exception]
fn SysTick() {
    // A separate load and store is enough: this handler is the only code
    // that writes TICKS, and it cannot interrupt itself.
    TICKS.store(TICKS.load(Ordering::Relaxed).wrapping_add(1), Ordering::Relaxed);
}

/// Milliseconds since `systick_init`.
fn millis() -> u32 {
    TICKS.load(Ordering::Relaxed)
}

/// Stop the processor until the next exception (the WFI instruction).
/// Nothing is lost by sleeping: the exception that wakes it is handled
/// first, and execution then continues after this call.
fn wait_for_interrupt() {
    // SAFETY: WFI reads and writes no memory and changes no register.
    unsafe { core::arch::asm!("wfi", options(nomem, nostack, preserves_flags)) }
}

/// Wait for at least `ms` milliseconds.
fn delay_ms(ms: u32) {
    let start = millis();
    // Comparing the elapsed time, rather than against a deadline, keeps
    // working when the counter wraps round.
    while millis().wrapping_sub(start) < ms {
        // The count can only change when SysTick fires, so sleep until
        // then instead of spinning.
        wait_for_interrupt();
    }
}

#[entry]
fn main() -> ! {
    // The watchdog is running at reset and would reset the chip after
    // about 16 seconds.
    write_reg(WDT_MR, WDT_MR_WDDIS);

    uart_init();
    uart_write_str("Hello from Rust on SAME70\n");

    systick_init();
    delay_ms(500);
    uart_write_str("Type something:\n");

    uart_enable_rx_interrupt();

    // Everything from here on happens in the interrupt handlers.
    loop {
        wait_for_interrupt();
    }
}
