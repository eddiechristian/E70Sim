pub mod bus;
pub mod cpu;
pub mod efc;
pub mod helpers;
pub mod matrix;
pub mod peripheral;
pub mod pio;
pub mod pmc;
pub mod rstc;
pub mod scb;
pub mod supc;
pub mod uart;
pub mod wdt;

use bus::Bus;
use cpu::Cpu;

fn main() {
    let mut bus = Bus::new();
    bus.load_firmware_from_bin("firmware/firmware.bin".to_string());
    let mut cpu = Cpu::new();
    // `--trace` logs every instruction and the registers to stderr.
    cpu.trace = std::env::args().any(|arg| arg == "--trace");
    cpu.reset(&mut bus);
    // Runs until step() panics on an unhandled instruction.
    loop {
        cpu.step(&mut bus);
    }
}
