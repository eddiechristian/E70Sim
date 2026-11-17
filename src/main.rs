pub mod bus;
pub mod cpu;
pub mod helpers;
pub mod peripheral;

use bus::Bus;
use cpu::Cpu;

fn main() {
    let mut bus = Bus::new();
    // Usage: E70_emulator [--trace] [firmware.bin]
    // `--trace` logs every instruction and the registers to stderr.
    // The first argument that is not a flag is the firmware image to load.
    let args: Vec<String> = std::env::args().skip(1).collect();
    let firmware = args
        .iter()
        .find(|arg| !arg.starts_with("--"))
        .cloned()
        .unwrap_or_else(|| "firmware/hello-rs.bin".to_string());
    bus.load_firmware_from_bin(firmware);
    // What is typed on stdin arrives at UART0's receiver.
    bus.set_uart0_input(peripheral::uart::stdin_bytes());
    let mut cpu = Cpu::new();
    cpu.trace = args.iter().any(|arg| arg == "--trace");
    cpu.reset(&mut bus);
    // Runs until step() panics on an unhandled instruction.
    loop {
        cpu.step(&mut bus);
    }
}
