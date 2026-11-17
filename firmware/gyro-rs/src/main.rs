//! Bare-metal firmware for the ATSAME70Q21: read a BNO055 gyro sensor
//! over I2C and print the readings on UART0.
//!
//! The sensor is on the BNO055 Xplained Pro card, plugged into the EXT1
//! header of a SAM E70 Xplained Ultra board. EXT1's I2C pins are PA3
//! (data) and PA4 (clock), which belong to the chip's first I2C
//! controller, TWIHS0.
//!
//! The program:
//!
//! 1. checks that the sensor answers and reads its chip ID;
//! 2. switches the sensor to its gyro-only mode;
//! 3. reads the three turn rates several times, waiting for each byte
//!    in a loop ("polled"), and prints them;
//! 4. reads them once more with the I2C interrupt doing the work, and
//!    prints that too;
//! 5. asks the sensor to raise its INT line when the board turns
//!    quickly, and reports each time it does. The INT line is on EXT1
//!    pin 9, which is PD28.
//!
//! See `../../gyro.md`. SAME70 register addresses and bit positions are
//! from `ATSAME70Q21.svd` and the datasheet in `reference/same70/`; the
//! sensor's are from `reference/bno055/`.

#![no_std] // no standard library: it needs an operating system
#![no_main] // no normal main: cortex-m-rt calls our #[entry] function

use core::ptr::{read_volatile, write_volatile};
use core::sync::atomic::{AtomicBool, AtomicU8, AtomicU32, Ordering};

use cortex_m_rt::{entry, exception};
use panic_halt as _; // on panic, loop forever

// Watchdog Timer.
const WDT_MR: *mut u32 = 0x400E_1854 as *mut u32;
const WDT_MR_WDDIS: u32 = 1 << 15;

// Power Management Controller.
const PMC_PCER0: *mut u32 = 0x400E_0610 as *mut u32;
const ID_UART0: u32 = 7;
const ID_PIOD: u32 = 16;
const ID_TWIHS0: u32 = 19;

// PIOA: hand pins to a peripheral (function A is a 0 in both select
// registers).
const PIOA_PDR: *mut u32 = 0x400E_0E04 as *mut u32;
const PIOA_ABCDSR0: *mut u32 = 0x400E_0E70 as *mut u32;
const PIOA_ABCDSR1: *mut u32 = 0x400E_0E74 as *mut u32;
const TWIHS0_PINS: u32 = (1 << 3) | (1 << 4); // PA3 = TWD0, PA4 = TWCK0

// PIOD: the sensor's INT line comes in on PD28.
const PIOD_PER: *mut u32 = 0x400E_1400 as *mut u32; // pin controlled by the PIO
const PIOD_ODR: *mut u32 = 0x400E_1414 as *mut u32; // pin is an input
const PIOD_IER: *mut u32 = 0x400E_1440 as *mut u32; // interrupt enable
const PIOD_ISR: *mut u32 = 0x400E_144C as *mut u32; // interrupt status
const PIOD_AIMER: *mut u32 = 0x400E_14B0 as *mut u32; // pick one edge or level
const PIOD_ESR: *mut u32 = 0x400E_14C0 as *mut u32; // ... an edge
const PIOD_REHLSR: *mut u32 = 0x400E_14D4 as *mut u32; // ... the rising one
const INT_PIN: u32 = 1 << 28;

// UART0.
const UART0_CR: *mut u32 = 0x400E_0800 as *mut u32;
const UART0_SR: *mut u32 = 0x400E_0814 as *mut u32;
const UART0_THR: *mut u32 = 0x400E_081C as *mut u32;
const UART_CR_TXEN: u32 = 1 << 6;
const UART_SR_TXRDY: u32 = 1 << 1;

// TWIHS0, the I2C controller.
const TWIHS0_CR: *mut u32 = 0x4001_8000 as *mut u32; // control
const TWIHS0_MMR: *mut u32 = 0x4001_8004 as *mut u32; // host mode
const TWIHS0_IADR: *mut u32 = 0x4001_800C as *mut u32; // internal address
const TWIHS0_CWGR: *mut u32 = 0x4001_8010 as *mut u32; // clock waveform
const TWIHS0_SR: *mut u32 = 0x4001_8020 as *mut u32; // status
const TWIHS0_IER: *mut u32 = 0x4001_8024 as *mut u32; // interrupt enable
const TWIHS0_IDR: *mut u32 = 0x4001_8028 as *mut u32; // interrupt disable
const TWIHS0_RHR: *mut u32 = 0x4001_8030 as *mut u32; // received byte
const TWIHS0_THR: *mut u32 = 0x4001_8034 as *mut u32; // byte to send

const TWIHS_CR_START: u32 = 1 << 0;
const TWIHS_CR_STOP: u32 = 1 << 1;
const TWIHS_CR_MSEN: u32 = 1 << 2; // host mode on
const TWIHS_CR_SVDIS: u32 = 1 << 5; // client mode off
const TWIHS_CR_SWRST: u32 = 1 << 7; // reset the controller

const TWIHS_MMR_IADRSZ_1: u32 = 1 << 8; // send one internal address byte
const TWIHS_MMR_MREAD: u32 = 1 << 12; // direction: read
const TWIHS_MMR_DADR_SHIFT: u32 = 16; // device address

const TWIHS_SR_TXCOMP: u32 = 1 << 0; // transfer finished
const TWIHS_SR_RXRDY: u32 = 1 << 1; // a byte is waiting in TWIHS0_RHR
const TWIHS_SR_TXRDY: u32 = 1 << 2; // TWIHS0_THR can take a byte
const TWIHS_SR_NACK: u32 = 1 << 8; // nothing answered

// NVIC and SysTick, in the Arm core.
const NVIC_ISER0: *mut u32 = 0xE000_E100 as *mut u32;
const IRQ_PIOD: u32 = 16;
const IRQ_TWIHS0: u32 = 19;
const SYST_CSR: *mut u32 = 0xE000_E010 as *mut u32;
const SYST_RVR: *mut u32 = 0xE000_E014 as *mut u32;
const SYST_CVR: *mut u32 = 0xE000_E018 as *mut u32;

/// The chip starts on its internal 12 MHz oscillator.
const CLOCK_HZ: u32 = 12_000_000;

// The BNO055 sensor.
const BNO055: u32 = 0x29; // its I2C address
const BNO055_CHIP_ID: u32 = 0x00;
const BNO055_GYR_DATA: u32 = 0x14; // X, Y, Z: six bytes
const BNO055_PAGE_ID: u32 = 0x07; // which of the two register pages
const BNO055_INT_STA: u32 = 0x37; // which interrupts have happened
const BNO055_OPR_MODE: u32 = 0x3D;
const BNO055_SYS_TRIGGER: u32 = 0x3F;
// On page 1:
const BNO055_INT_MSK: u32 = 0x0F; // which interrupts drive the INT line
const BNO055_INT_EN: u32 = 0x10; // which interrupts are watched for
const BNO055_GYR_INT_SETTING: u32 = 0x17; // which axes
const BNO055_GYR_AM_THRES: u32 = 0x1E; // any-motion threshold
const BNO055_INT_GYRO_AM: u8 = 1 << 2; // the gyro any-motion interrupt
const BNO055_RST_INT: u8 = 1 << 6; // in SYS_TRIGGER: lower the INT line
const BNO055_MODE_CONFIG: u8 = 0x00;
const BNO055_CHIP_ID_VALUE: u8 = 0xA0;
const BNO055_MODE_GYROONLY: u8 = 0x03;
const BNO055_RST_SYS: u8 = 1 << 5;

/// "Turning quickly" means faster than this, in degrees per second.
const MOTION_THRESHOLD_DPS: u8 = 50;
/// How long to watch for it, in milliseconds.
const MOTION_WATCH_MS: u32 = 3000;

/// Set by the PIOD interrupt handler when the sensor raises its INT
/// line.
static MOTION: AtomicBool = AtomicBool::new(false);

/// Milliseconds since start-up, counted by the SysTick handler.
static TICKS: AtomicU32 = AtomicU32::new(0);

// State shared with the TWIHS0 interrupt handler, for the read it does.
/// The bytes received so far.
static RX_BUFFER: [AtomicU8; 6] = [const { AtomicU8::new(0) }; 6];
/// How many bytes the read is for, and how many have arrived.
static RX_WANTED: AtomicU8 = AtomicU8::new(0);
static RX_COUNT: AtomicU8 = AtomicU8::new(0);
/// Set by the handler when the transfer has ended.
static RX_DONE: AtomicBool = AtomicBool::new(false);
/// Set by the handler if the device did not answer.
static RX_FAILED: AtomicBool = AtomicBool::new(false);

fn write_reg(reg: *mut u32, value: u32) {
    // SAFETY: every `reg` passed in this file is the address of a real,
    // 32-bit register on the ATSAME70Q21.
    unsafe { write_volatile(reg, value) }
}

fn read_reg(reg: *mut u32) -> u32 {
    // SAFETY: as for `write_reg`.
    unsafe { read_volatile(reg) }
}

// ---------------------------------------------------------------- UART

fn uart_write_byte(byte: u8) {
    while read_reg(UART0_SR) & UART_SR_TXRDY == 0 {}
    write_reg(UART0_THR, byte as u32);
}

fn uart_write_str(text: &str) {
    for byte in text.bytes() {
        uart_write_byte(byte);
    }
}

/// Print a byte as two hex digits.
fn uart_write_hex8(value: u8) {
    for shift in [4, 0] {
        let digit = (value >> shift) & 0xF;
        uart_write_byte(if digit < 10 { b'0' + digit } else { b'A' + digit - 10 });
    }
}

/// Print a number in decimal, with at least `min_digits` digits.
fn uart_write_decimal(mut value: u32, min_digits: usize) {
    let mut digits = [b'0'; 10];
    let mut count = 0;
    while value > 0 || count < min_digits {
        digits[count] = b'0' + (value % 10) as u8;
        value /= 10;
        count += 1;
    }
    for i in (0..count).rev() {
        uart_write_byte(digits[i]);
    }
}

/// Print a raw gyro value as degrees per second with two decimals. The
/// sensor's unit is 1/16 of a degree per second.
fn uart_write_rate(raw: i16) {
    if raw < 0 {
        uart_write_byte(b'-');
    }
    // In hundredths of a degree per second: raw * 100 / 16.
    let hundredths = (raw as i32).unsigned_abs() * 100 / 16;
    uart_write_decimal(hundredths / 100, 1);
    uart_write_byte(b'.');
    uart_write_decimal(hundredths % 100, 2);
}

/// Print one line with the three rates.
fn report(label: &str, rates: [i16; 3]) {
    uart_write_str(label);
    for (name, raw) in ["x", "y", "z"].iter().zip(rates) {
        uart_write_byte(b' ');
        uart_write_str(name);
        uart_write_byte(b'=');
        uart_write_rate(raw);
    }
    uart_write_str(" dps\n");
}

// ---------------------------------------------------------------- time

#[exception]
fn SysTick() {
    TICKS.store(TICKS.load(Ordering::Relaxed).wrapping_add(1), Ordering::Relaxed);
}

fn millis() -> u32 {
    TICKS.load(Ordering::Relaxed)
}

fn wait_for_interrupt() {
    // SAFETY: WFI reads and writes no memory and changes no register.
    unsafe { core::arch::asm!("wfi", options(nomem, nostack, preserves_flags)) }
}

fn delay_ms(ms: u32) {
    let start = millis();
    while millis().wrapping_sub(start) < ms {
        wait_for_interrupt();
    }
}

// ----------------------------------------------------------------- I2C

fn i2c_init() {
    // Clock for the controller, and its two pins.
    write_reg(PMC_PCER0, 1 << ID_TWIHS0);
    write_reg(PIOA_ABCDSR0, read_reg(PIOA_ABCDSR0) & !TWIHS0_PINS);
    write_reg(PIOA_ABCDSR1, read_reg(PIOA_ABCDSR1) & !TWIHS0_PINS);
    write_reg(PIOA_PDR, TWIHS0_PINS);

    write_reg(TWIHS0_CR, TWIHS_CR_SWRST);
    // Bus speed 100 kHz. Each half of the clock period lasts
    // (DIV * 2^CKDIV + 3) clock cycles, so with CKDIV = 0:
    // DIV = 12 MHz / (2 * 100 kHz) - 3 = 57.
    let div = CLOCK_HZ / (2 * 100_000) - 3;
    write_reg(TWIHS0_CWGR, (div << 8) | div); // CHDIV and CLDIV
    write_reg(TWIHS0_CR, TWIHS_CR_SVDIS | TWIHS_CR_MSEN);
}

/// Write one byte to register `register` of the device at `address`.
/// Returns false if the device did not answer.
fn i2c_write(address: u32, register: u32, value: u8) -> bool {
    // The register number goes as the "internal address".
    write_reg(TWIHS0_MMR, (address << TWIHS_MMR_DADR_SHIFT) | TWIHS_MMR_IADRSZ_1);
    write_reg(TWIHS0_IADR, register);
    write_reg(TWIHS0_THR, value as u32);
    // Wait until the byte has gone, or was refused.
    loop {
        let status = read_reg(TWIHS0_SR);
        if status & TWIHS_SR_NACK != 0 {
            return false;
        }
        if status & TWIHS_SR_TXRDY != 0 {
            break;
        }
    }
    write_reg(TWIHS0_CR, TWIHS_CR_STOP);
    while read_reg(TWIHS0_SR) & TWIHS_SR_TXCOMP == 0 {}
    true
}

/// Read `buffer.len()` bytes starting at register `register` of the
/// device at `address`, waiting for each one. Returns false if the
/// device did not answer.
fn i2c_read(address: u32, register: u32, buffer: &mut [u8]) -> bool {
    write_reg(
        TWIHS0_MMR,
        (address << TWIHS_MMR_DADR_SHIFT) | TWIHS_MMR_IADRSZ_1 | TWIHS_MMR_MREAD,
    );
    write_reg(TWIHS0_IADR, register);
    let count = buffer.len();
    // For a single byte, STOP goes with START.
    if count == 1 {
        write_reg(TWIHS0_CR, TWIHS_CR_START | TWIHS_CR_STOP);
    } else {
        write_reg(TWIHS0_CR, TWIHS_CR_START);
    }
    for (i, slot) in buffer.iter_mut().enumerate() {
        loop {
            let status = read_reg(TWIHS0_SR);
            if status & TWIHS_SR_NACK != 0 {
                return false;
            }
            if status & TWIHS_SR_RXRDY != 0 {
                break;
            }
        }
        // STOP has to be asked for while the next-to-last byte is
        // waiting: the controller then takes one more byte and ends.
        if count > 1 && i == count - 2 {
            write_reg(TWIHS0_CR, TWIHS_CR_STOP);
        }
        *slot = read_reg(TWIHS0_RHR) as u8;
    }
    while read_reg(TWIHS0_SR) & TWIHS_SR_TXCOMP == 0 {}
    true
}

/// Start a read of `count` bytes (2 to 6) that the TWIHS0 interrupt
/// handler will carry out. `RX_DONE` is set when it has finished.
fn i2c_read_start_by_interrupt(address: u32, register: u32, count: u8) {
    RX_WANTED.store(count, Ordering::Relaxed);
    RX_COUNT.store(0, Ordering::Relaxed);
    RX_DONE.store(false, Ordering::Relaxed);
    RX_FAILED.store(false, Ordering::Relaxed);
    write_reg(
        TWIHS0_MMR,
        (address << TWIHS_MMR_DADR_SHIFT) | TWIHS_MMR_IADRSZ_1 | TWIHS_MMR_MREAD,
    );
    write_reg(TWIHS0_IADR, register);
    // Ask for an interrupt for each received byte, and for "no answer".
    write_reg(TWIHS0_IER, TWIHS_SR_RXRDY | TWIHS_SR_NACK);
    write_reg(TWIHS0_CR, TWIHS_CR_START);
}

/// Runs when TWIHS0 has received a byte, the device has not answered,
/// or the transfer has ended.
#[unsafe(no_mangle)]
#[allow(non_snake_case)]
extern "C" fn TWIHS0() {
    let status = read_reg(TWIHS0_SR);
    let wanted = RX_WANTED.load(Ordering::Relaxed);
    let count = RX_COUNT.load(Ordering::Relaxed);

    if status & TWIHS_SR_NACK != 0 {
        RX_FAILED.store(true, Ordering::Relaxed);
        write_reg(TWIHS0_IDR, !0);
        RX_DONE.store(true, Ordering::Relaxed);
        return;
    }
    if status & TWIHS_SR_RXRDY != 0 && count < wanted {
        // The same rule as in `i2c_read`: STOP while the next-to-last
        // byte is waiting.
        if count + 2 == wanted {
            write_reg(TWIHS0_CR, TWIHS_CR_STOP);
        }
        RX_BUFFER[count as usize].store(read_reg(TWIHS0_RHR) as u8, Ordering::Relaxed);
        RX_COUNT.store(count + 1, Ordering::Relaxed);
        if count + 1 == wanted {
            // All bytes are in: now wait only for the end of the transfer.
            write_reg(TWIHS0_IDR, TWIHS_SR_RXRDY);
            write_reg(TWIHS0_IER, TWIHS_SR_TXCOMP);
        }
        return;
    }
    if status & TWIHS_SR_TXCOMP != 0 && count == wanted {
        write_reg(TWIHS0_IDR, !0);
        RX_DONE.store(true, Ordering::Relaxed);
    }
}

/// Runs when a pin of PIOD that has its interrupt enabled changes:
/// here, when the sensor's INT line goes high.
///
/// It only notes that it happened. Asking the sensor what it wants
/// takes I2C transfers, which the main code does.
#[unsafe(no_mangle)]
#[allow(non_snake_case)]
extern "C" fn PIOD() {
    // Reading the status is what stops the port asking again.
    if read_reg(PIOD_ISR) & INT_PIN != 0 {
        MOTION.store(true, Ordering::Relaxed);
    }
}

unsafe extern "C" {
    /// `cortex-m-rt`'s handler for anything unexpected: it loops for ever.
    fn DefaultHandler();
}

/// The interrupt part of the vector table: entry n is the handler for
/// IRQ n. PIOD is IRQ 16 and TWIHS0 is IRQ 19.
#[unsafe(link_section = ".vector_table.interrupts")]
#[unsafe(no_mangle)]
#[used]
pub static __INTERRUPTS: [unsafe extern "C" fn(); 20] = {
    let mut table: [unsafe extern "C" fn(); 20] = [DefaultHandler; 20];
    table[IRQ_PIOD as usize] = PIOD;
    table[IRQ_TWIHS0 as usize] = TWIHS0;
    table
};

// --------------------------------------------------------------- gyro

/// Turn six data bytes into the three rates. Each is a signed 16-bit
/// number with its low byte first.
fn rates_from(bytes: [u8; 6]) -> [i16; 3] {
    [
        i16::from_le_bytes([bytes[0], bytes[1]]),
        i16::from_le_bytes([bytes[2], bytes[3]]),
        i16::from_le_bytes([bytes[4], bytes[5]]),
    ]
}

/// Have the sensor raise its INT line when any axis turns faster than
/// `MOTION_THRESHOLD_DPS`, and have that line interrupt the processor.
fn motion_interrupt_init() {
    // The sensor's settings can only be changed in its configuration
    // mode. The waiting times are the datasheet's.
    i2c_write(BNO055, BNO055_OPR_MODE, BNO055_MODE_CONFIG);
    delay_ms(19);
    i2c_write(BNO055, BNO055_PAGE_ID, 1);
    i2c_write(BNO055, BNO055_GYR_INT_SETTING, 0b111); // X, Y and Z
    // At the gyro's default range one unit is 1 degree per second.
    i2c_write(BNO055, BNO055_GYR_AM_THRES, MOTION_THRESHOLD_DPS);
    i2c_write(BNO055, BNO055_INT_MSK, BNO055_INT_GYRO_AM);
    i2c_write(BNO055, BNO055_INT_EN, BNO055_INT_GYRO_AM);
    i2c_write(BNO055, BNO055_PAGE_ID, 0);
    i2c_write(BNO055, BNO055_OPR_MODE, BNO055_MODE_GYROONLY);
    delay_ms(7);

    // PD28 as an input that interrupts on its rising edge.
    write_reg(PMC_PCER0, 1 << ID_PIOD);
    write_reg(PIOD_PER, INT_PIN);
    write_reg(PIOD_ODR, INT_PIN);
    write_reg(PIOD_AIMER, INT_PIN);
    write_reg(PIOD_ESR, INT_PIN);
    write_reg(PIOD_REHLSR, INT_PIN);
    // Forget any change seen on the pin before now.
    read_reg(PIOD_ISR);
    write_reg(PIOD_IER, INT_PIN);
    write_reg(NVIC_ISER0, 1 << IRQ_PIOD);
}

fn stop(message: &str) -> ! {
    uart_write_str(message);
    loop {
        wait_for_interrupt();
    }
}

#[entry]
fn main() -> ! {
    write_reg(WDT_MR, WDT_MR_WDDIS);
    write_reg(PMC_PCER0, 1 << ID_UART0);
    write_reg(UART0_CR, UART_CR_TXEN);

    // A 1 ms tick, for the delays the sensor needs.
    write_reg(SYST_RVR, CLOCK_HZ / 1000 - 1);
    write_reg(SYST_CVR, 0);
    write_reg(SYST_CSR, 0b111);

    uart_write_str("Gyro test\n");
    i2c_init();

    // 1. Is the sensor there?
    let mut id = [0u8; 1];
    if !i2c_read(BNO055, BNO055_CHIP_ID, &mut id) {
        stop("no answer from the BNO055 at I2C address 0x29\n");
    }
    uart_write_str("BNO055 chip id = ");
    uart_write_hex8(id[0]);
    uart_write_byte(b'\n');
    if id[0] != BNO055_CHIP_ID_VALUE {
        stop("unexpected chip id\n");
    }

    // 2. Reset the sensor so it starts from a known state, then switch
    //    on the gyro. The datasheet's waiting times: 650 ms after a
    //    reset, 7 ms after leaving the configuration mode.
    i2c_write(BNO055, BNO055_SYS_TRIGGER, BNO055_RST_SYS);
    delay_ms(650);
    i2c_write(BNO055, BNO055_OPR_MODE, BNO055_MODE_GYROONLY);
    delay_ms(7);

    // 3. Read the three rates a few times, a tenth of a second apart.
    for _ in 0..5 {
        let mut data = [0u8; 6];
        if !i2c_read(BNO055, BNO055_GYR_DATA, &mut data) {
            stop("gyro read failed\n");
        }
        report("gyro", rates_from(data));
        delay_ms(100);
    }

    // 4. Once more, with the interrupt handler collecting the bytes
    //    while the processor sleeps.
    write_reg(NVIC_ISER0, 1 << IRQ_TWIHS0);
    i2c_read_start_by_interrupt(BNO055, BNO055_GYR_DATA, 6);
    while !RX_DONE.load(Ordering::Relaxed) {
        wait_for_interrupt();
    }
    if RX_FAILED.load(Ordering::Relaxed) {
        stop("gyro read by interrupt failed\n");
    }
    let mut data = [0u8; 6];
    for (slot, byte) in data.iter_mut().zip(&RX_BUFFER) {
        *slot = byte.load(Ordering::Relaxed);
    }
    report("gyro by interrupt", rates_from(data));

    // 5. Let the sensor say when the board is turning quickly.
    motion_interrupt_init();
    uart_write_str("watching for motion\n");
    let start = millis();
    let mut detections = 0u32;
    while millis().wrapping_sub(start) < MOTION_WATCH_MS {
        if !MOTION.swap(false, Ordering::Relaxed) {
            wait_for_interrupt();
            continue;
        }
        // The sensor raised its INT line. Ask it why, and what the
        // rates are now.
        let mut status = [0u8; 1];
        let mut data = [0u8; 6];
        i2c_read(BNO055, BNO055_INT_STA, &mut status);
        i2c_read(BNO055, BNO055_GYR_DATA, &mut data);
        uart_write_str("motion at ");
        uart_write_decimal(millis(), 1);
        uart_write_str(" ms, INT_STA = ");
        uart_write_hex8(status[0]);
        uart_write_byte(b',');
        report("", rates_from(data));
        detections += 1;
        // Lower the INT line so that the next rise can be seen.
        i2c_write(BNO055, BNO055_SYS_TRIGGER, BNO055_RST_INT);
    }
    uart_write_str("motion detected ");
    uart_write_decimal(detections, 1);
    uart_write_str(" times\n");

    stop("done\n");
}
