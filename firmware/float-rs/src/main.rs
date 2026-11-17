//! Hard-float firmware for the ATSAME70Q21: floating-point arithmetic
//! done by the FPU, with the results printed over UART0 as hex bit
//! patterns.
//!
//! The second half checks that an interrupt handler which uses the FPU
//! does not disturb floating-point work in the main code.
//!
//! See `../firmware.md`. Register addresses and bit positions are from
//! `ATSAME70Q21.svd`.

#![no_std] // no standard library: it needs an operating system
#![no_main] // no normal main: cortex-m-rt calls our #[entry] function

use core::hint::black_box;
use core::ptr::{read_volatile, write_volatile};
use core::sync::atomic::{AtomicU32, Ordering};

use cortex_m_rt::{entry, exception};
use panic_halt as _; // on panic, loop forever

const WDT_MR: *mut u32 = 0x400E_1854 as *mut u32;
const WDT_MR_WDDIS: u32 = 1 << 15; // watchdog disable

const PMC_PCER0: *mut u32 = 0x400E_0610 as *mut u32;
const ID_UART0: u32 = 7;

const UART0_CR: *mut u32 = 0x400E_0800 as *mut u32;
const UART0_SR: *mut u32 = 0x400E_0814 as *mut u32;
const UART0_THR: *mut u32 = 0x400E_081C as *mut u32;
const UART_CR_TXEN: u32 = 1 << 6;
const UART_SR_TXRDY: u32 = 1 << 1;

const SYST_CSR: *mut u32 = 0xE000_E010 as *mut u32;
const SYST_RVR: *mut u32 = 0xE000_E014 as *mut u32;
const SYST_CVR: *mut u32 = 0xE000_E018 as *mut u32;

/// Clock cycles between SysTick exceptions. Short, so that the handler
/// interrupts the main loop's arithmetic at many different points.
const TICK_CYCLES: u32 = 997;

/// How many SysTick exceptions the second half of the test waits for.
const TICKS_WANTED: u32 = 200;

/// Number of SysTick exceptions so far.
static TICKS: AtomicU32 = AtomicU32::new(0);

/// A running total kept by the SysTick handler, stored as the bits of
/// an `f32`. The handler adds 0.5 to it each time.
static HANDLER_TOTAL: AtomicU32 = AtomicU32::new(0);

fn write_reg(reg: *mut u32, value: u32) {
    // SAFETY: every `reg` passed in this file is the address of a real,
    // 32-bit register on the ATSAME70Q21.
    unsafe { write_volatile(reg, value) }
}

fn read_reg(reg: *mut u32) -> u32 {
    // SAFETY: as for `write_reg`.
    unsafe { read_volatile(reg) }
}

fn uart_write_byte(byte: u8) {
    while read_reg(UART0_SR) & UART_SR_TXRDY == 0 {}
    write_reg(UART0_THR, byte as u32);
}

fn uart_write_str(text: &str) {
    for byte in text.bytes() {
        uart_write_byte(byte);
    }
}

/// Print a 32-bit value as 8 hex digits.
fn uart_write_hex(value: u32) {
    for shift in (0..8).rev() {
        let digit = ((value >> (4 * shift)) & 0xF) as u8;
        uart_write_byte(if digit < 10 { b'0' + digit } else { b'A' + digit - 10 });
    }
}

/// Print one result line: a label, then the bits of an `f32`.
fn report32(label: &str, value: f32) {
    uart_write_str(label);
    uart_write_str(" = ");
    uart_write_hex(value.to_bits());
    uart_write_byte(b'\n');
}

/// Print one result line: a label, then the bits of an `f64`.
fn report64(label: &str, value: f64) {
    let bits = value.to_bits();
    uart_write_str(label);
    uart_write_str(" = ");
    uart_write_hex((bits >> 32) as u32);
    uart_write_hex(bits as u32);
    uart_write_byte(b'\n');
}

/// Print one result line: a label, then an integer in hex.
fn report_int(label: &str, value: u32) {
    uart_write_str(label);
    uart_write_str(" = ");
    uart_write_hex(value);
    uart_write_byte(b'\n');
}

/// 1/1 + 1/2 + ... + 1/n, in single precision.
fn harmonic(n: u32) -> f32 {
    let mut sum = 0.0f32;
    let mut k = 1u32;
    while k <= n {
        sum += 1.0 / k as f32;
        k += 1;
    }
    sum
}

/// Runs every `TICK_CYCLES` clock cycles, and uses the FPU.
#[exception]
fn SysTick() {
    let total = f32::from_bits(HANDLER_TOTAL.load(Ordering::Relaxed));
    // black_box keeps the compiler from working the sum out in advance.
    let total = total + black_box(0.5f32);
    HANDLER_TOTAL.store(total.to_bits(), Ordering::Relaxed);
    TICKS.store(TICKS.load(Ordering::Relaxed).wrapping_add(1), Ordering::Relaxed);
}

#[entry]
fn main() -> ! {
    write_reg(WDT_MR, WDT_MR_WDDIS);
    write_reg(PMC_PCER0, 1 << ID_UART0);
    write_reg(UART0_CR, UART_CR_TXEN);

    uart_write_str("FPU test\n");

    // black_box hides the values from the compiler, so the arithmetic is
    // done by the FPU at run time and not folded away at build time.
    let a = black_box(1.5f32);
    let b = black_box(2.25f32);
    report32("f32 add", a + b);
    report32("f32 sub", a - b);
    report32("f32 mul", a * b);
    report32("f32 div", a / b);
    report32("f32 0.1+0.2", black_box(0.1f32) + black_box(0.2f32));

    let x = black_box(0.1f64);
    let y = black_box(0.2f64);
    report64("f64 add", x + y);
    report64("f64 mul", x * y);
    report64("f64 div", x / y);

    report64("f32 to f64", black_box(0.1f32) as f64);
    report32("f64 to f32", black_box(1.0f64 / 3.0) as f32);
    report32("i32 to f32", black_box(-7i32) as f32);
    report64("u32 to f64", black_box(4_000_000_000u32) as f64);
    report_int("f32 to i32", black_box(-2.9f32) as i32 as u32);
    report_int("f64 to u32", black_box(3_000_000_000.7f64) as u32);
    report_int("f32 a<b", (a < b) as u32);
    report_int("f32 a>b", (a > b) as u32);
    report32("harmonic(10)", harmonic(black_box(10)));

    // Second half: do floating-point work in the main code while an
    // interrupt handler that also uses the FPU keeps cutting in.
    write_reg(SYST_RVR, TICK_CYCLES - 1);
    write_reg(SYST_CVR, 0);
    write_reg(SYST_CSR, 0b111); // ENABLE | TICKINT | CLKSOURCE

    let expected = harmonic(black_box(50));
    let mut rounds = 0u32;
    let mut mismatches = 0u32;
    while TICKS.load(Ordering::Relaxed) < TICKS_WANTED {
        if harmonic(black_box(50)).to_bits() != expected.to_bits() {
            mismatches += 1;
        }
        rounds += 1;
    }
    write_reg(SYST_CSR, 0); // stop the timer

    report_int("ticks", TICKS.load(Ordering::Relaxed));
    report32("handler total", f32::from_bits(HANDLER_TOTAL.load(Ordering::Relaxed)));
    report_int("rounds > 0", (rounds > 0) as u32);
    report_int("mismatches", mismatches);
    uart_write_str("done\n");

    loop {
        // SAFETY: WFI reads and writes no memory and changes no register.
        unsafe { core::arch::asm!("wfi", options(nomem, nostack, preserves_flags)) }
    }
}
