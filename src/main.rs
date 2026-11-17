pub mod bno055;
pub mod bus;
pub mod cpu;
pub mod helpers;
pub mod i2c;
pub mod peripheral;

use bus::Bus;
use cpu::Cpu;

fn main() {
    let mut bus = Bus::new();
    // Usage: E70_emulator [--trace] [--bus-faults] [--pin=...] [--gyro=...] [--gyro-int=...] [firmware.bin]
    // `--trace` logs every instruction and the registers to stderr.
    // `--bus-faults` makes an access to an unmapped address a BusFault,
    // as on the real chip, instead of a warning.
    // `--pin=PA11=0@5000000` drives an input pin (here PA11) to a level
    // (0) at a moment given in clock cycles since reset. It can be given
    // several times.
    // `--gyro=1.0,-2.0,0.5` sets how fast the gyro sensor is turning
    // around X, Y and Z, in degrees per second. `--gyro=pattern` gives a
    // changing pattern and `--gyro=file:PATH` reads samples from a file.
    // `--gyro-int=PD28` names the pin the sensor's interrupt output is
    // wired to (PD28 is the default); `--gyro-int=none` leaves it
    // unconnected.
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
    // The BNO055 gyro card is plugged into the EXT1 header, which is
    // wired to TWIHS0.
    let mut motion = bno055::Motion::Constant([1.0, -2.0, 0.5]);
    for arg in &args {
        if let Some(text) = arg.strip_prefix("--gyro=") {
            match bno055::Motion::parse_option(text) {
                Ok(chosen) => motion = chosen,
                Err(message) => {
                    eprintln!("{}", message);
                    std::process::exit(2);
                }
            }
        }
    }
    bus.attach_i2c_device(0, Box::new(bno055::Bno055::new(motion)));
    // The card's INT line is on EXT1 pin 9, which is PD28 on the SAM E70
    // Xplained Ultra.
    let mut interrupt_pin = Some((3, 28));
    for arg in &args {
        if let Some(text) = arg.strip_prefix("--gyro-int=") {
            interrupt_pin = match text {
                "none" => None,
                name => match bus::parse_pin_name(name) {
                    Ok(pin) => Some(pin),
                    Err(message) => {
                        eprintln!("{}", message);
                        std::process::exit(2);
                    }
                },
            };
        }
    }
    if let Some((port, pin)) = interrupt_pin {
        bus.connect_i2c_interrupt(0, bno055::ADDRESS, port, pin);
    }
    let mut cpu = Cpu::new();
    cpu.trace = args.iter().any(|arg| arg == "--trace");
    cpu.reset(&mut bus);
    // Runs until step() panics on an unhandled instruction.
    loop {
        cpu.step(&mut bus);
    }
}
