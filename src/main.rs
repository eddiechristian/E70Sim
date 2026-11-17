pub mod bus;
pub mod cpu;
pub mod helpers;
pub mod peripheral;

use bus::Bus;
use cpu::Cpu;

fn main() {
    let mut bus = Bus::new();
    // Usage: E70_emulator [--trace] [--bus-faults] [--pin=...] [firmware.bin]
    // `--trace` logs every instruction and the registers to stderr.
    // `--bus-faults` makes an access to an unmapped address a BusFault,
    // as on the real chip, instead of a warning.
    // `--pin=PA11=0@5000000` drives an input pin (here PA11) to a level
    // (0) at a moment given in clock cycles since reset. It can be given
    // several times.
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
    bus.set_bus_faults(args.iter().any(|arg| arg == "--bus-faults"));
    for arg in &args {
        if let Some(text) = arg.strip_prefix("--pin=") {
            match bus::PinEvent::parse(text) {
                Ok(event) => bus.schedule_pin_event(event),
                Err(message) => {
                    eprintln!("{}", message);
                    std::process::exit(2);
                }
            }
        }
    }
    let mut cpu = Cpu::new();
    cpu.trace = args.iter().any(|arg| arg == "--trace");
    cpu.reset(&mut bus);
    // Runs until step() panics on an unhandled instruction.
    loop {
        cpu.step(&mut bus);
    }
}
