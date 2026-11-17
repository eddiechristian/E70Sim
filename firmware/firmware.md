# Rust firmware for the emulator

This describes the first compiled firmware for Phase 3 of `PLAN.md`: a bare-metal Rust program that sets up UART0 and prints a line. The project is in `firmware/hello-rs/` and builds to `firmware/hello-rs.bin`. The hand-written `hello.s` and its `firmware.bin` are kept as a known-good test.

"Bare-metal" means the program runs directly on the chip with no operating system. Nothing prepares memory or calls `main` for it, so the program has to bring its own startup code.

## Pieces

| Piece | What it is | Who provides it |
|---|---|---|
| Vector table | The first words of flash: initial stack pointer, then the address of the reset code and every exception handler | `cortex-m-rt` crate |
| Startup code | Runs at reset, prepares RAM, then calls `main` | `cortex-m-rt` crate |
| Memory layout | Where flash and RAM are on this chip | `memory.x`, written by us |
| Panic handler | What to do if the program panics | `panic-halt` crate |
| `main` | Disable the watchdog, set up UART0, print | written by us |

## Minimal startup

### What the chip does at reset

The Cortex-M7 reads two words from the start of flash: word 0 goes into the stack pointer and word 1 into the program counter. Those two words are the start of the vector table. This is the same thing `hello.s` does by hand with its `.vectors` section, and it is what `Cpu::reset` already emulates.

### What `cortex-m-rt` does

The `cortex-m-rt` crate supplies the vector table and a `Reset` function that word 1 points to. `Reset` does three things before any Rust code of ours runs:

1. **Zero `.bss`.** Variables with no initial value (`static mut COUNT: u32 = 0`) live in a RAM section called `.bss`. RAM holds garbage at power-up, so it is filled with zeros.
2. **Copy `.data`.** Variables with an initial value (`static mut SPEED: u32 = 9600`) live in RAM, but their starting values are stored in flash. They are copied from flash to RAM.
3. **Call `main`.** The function we mark with `#[entry]`. It must never return.

On a hard-float target `Reset` would also switch the FPU on. We build soft-float, so it does not.

### Memory layout: `memory.x`

`cortex-m-rt` needs to know where flash and RAM are. The values match `src/bus.rs`:

```
MEMORY
{
    FLASH : ORIGIN = 0x00400000, LENGTH = 2M
    RAM   : ORIGIN = 0x20400000, LENGTH = 384K
}
```

From this the linker places the vector table at `0x0040_0000` and sets the initial stack pointer to the end of RAM, `0x2046_0000`. The stack grows downwards from there.

### Program skeleton

```rust
#![no_std]   // no standard library: it needs an operating system
#![no_main]  // no normal main: cortex-m-rt calls our #[entry] function

use cortex_m_rt::entry;
use panic_halt as _; // on panic, loop forever

#[entry]
fn main() -> ! {
    // set-up and printing go here
    loop {}
}
```

### Project layout

The firmware is its own Cargo project, separate from the emulator, because it is built for a different processor:

```
firmware/hello-rs/
    Cargo.toml          dependencies: cortex-m-rt, panic-halt
    memory.x            flash and RAM layout
    build.rs            puts memory.x where the linker looks for it
    .cargo/config.toml  build target and linker script
    build.sh            builds and writes ../hello-rs.bin
    src/main.rs
```

`.cargo/config.toml`:

```toml
[build]
target = "thumbv7em-none-eabi"   # Cortex-M7, soft-float (no "hf")

[target.thumbv7em-none-eabi]
rustflags = ["-C", "link-arg=-Tlink.x"]  # link.x comes from cortex-m-rt
```

### Building

One-time set-up:

```
rustup target add thumbv7em-none-eabi
rustup component add llvm-tools
```

Build, and turn the result into the raw image the emulator loads:

```
firmware/hello-rs/build.sh
```

The script runs `cargo build --release`, which produces an ELF file: the program plus symbols and section information. The emulator's loader copies raw bytes to the start of flash, so the script then uses `llvm-objcopy` (part of `llvm-tools`) to strip the ELF down to just those bytes, in `firmware/hello-rs.bin`.

### Running

From the repo root. Without an argument the emulator loads `firmware/hello-rs.bin`; pass a path to load another image, such as the assembly version.

```
cargo run
cargo run -- --trace
cargo run -- firmware/firmware.bin
```

### Disassembling

When the emulator stops on an unknown instruction it prints the address. To see what the instruction is, disassemble the ELF and look that address up:

```
OBJDUMP=$(find "$(rustc --print sysroot)" -name llvm-objdump | head -n 1)
"$OBJDUMP" -d firmware/hello-rs/target/thumbv7em-none-eabi/release/hello-rs
```

## UART initialisation

All registers are written with `core::ptr::write_volatile` and read with `core::ptr::read_volatile` on the raw address. "Volatile" tells the compiler the access has a side effect and must not be removed or reordered.

Register offsets and bits below are from `ATSAME70Q21.svd`.

### Step 1: disable the watchdog

The watchdog is running at reset and would reset a real chip after about 16 seconds.

| Register | Address | Value |
|---|---|---|
| `WDT_MR` | `0x400E_1854` | `1 << 15` (WDDIS) |

### Step 2: enable the UART0 clock

Peripherals are unclocked at reset to save power. UART0 is peripheral ID 7.

| Register | Address | Value |
|---|---|---|
| `PMC_PCER0` | `0x400E_0610` | `1 << 7` |

### Step 3: hand the pins to UART0

UART0 uses PA9 (receive) and PA10 (transmit), as peripheral function A. `PIO_PDR` takes the pins away from the PIO controller; clearing the pin's bit in both `PIO_ABCDSR` registers selects function A.

| Register | Address | Value |
|---|---|---|
| `PIO_PDR` (PIOA) | `0x400E_0E04` | `(1 << 9) \| (1 << 10)` |
| `PIO_ABCDSR[0]` (PIOA) | `0x400E_0E70` | clear bits 9 and 10 |
| `PIO_ABCDSR[1]` (PIOA) | `0x400E_0E74` | clear bits 9 and 10 |

### Step 4: configure UART0

UART0 base is `0x400E_0800`.

| Order | Register | Offset | Value | Meaning |
|---|---|---|---|---|
| 1 | `UART_CR` | `+0x00` | `(1 << 2) \| (1 << 3) \| (1 << 5) \| (1 << 7)` | RSTRX, RSTTX, RXDIS, TXDIS: reset and disable both directions |
| 2 | `UART_MR` | `+0x04` | `4 << 9` | PAR = no parity, normal mode |
| 3 | `UART_BRGR` | `+0x20` | clock divisor `CD` | sets the baud rate |
| 4 | `UART_CR` | `+0x00` | `(1 << 4) \| (1 << 6)` | RXEN, TXEN: enable both directions |

The UART always sends 8 data bits and 1 stop bit; only parity is configurable.

Baud rate: `CD = clock / (16 × baud)`. The chip starts on its internal 12 MHz oscillator, so 9600 baud needs `CD = 12_000_000 / (16 × 9600) = 78`. 115200 baud would need 6.5, which cannot be set accurately; a real board needs a faster clock from the PMC first.

### Step 5: print

For each byte:

1. Read `UART_SR` (`+0x14`) until TXRDY (bit 1) is set.
2. Write the byte to `UART_THR` (`+0x1C`).

This is the same loop as `hello.s`.

## What the emulator does with this

| Step | Emulator behaviour |
|---|---|
| Watchdog disable | Stored; the emulated watchdog never expires anyway |
| Clock enable | Stored in `PMC_PCSR0`; the UART stub works with or without it |
| Pin set-up | Stored; no pins exist |
| `UART_CR`, `UART_MR`, `UART_BRGR` | Ignored by the UART stub |
| `UART_SR` | Always reports TXRDY |
| `UART_THR` | Byte goes to stdout |

So only step 5 is needed for output in the emulator. Steps 1 to 4 are there so the same firmware would work on a real board.

Runs are expected to stop with an "unknown instruction" panic until the CPU core catches up. The compiler emits instructions the core does not implement yet, and each panic names the next one to add. The first is the `BL` at `0x0040_0400`, the very first instruction of the startup code.

## To verify

These are from memory, not from a document in the repo:

- UART0 on PA9 / PA10, peripheral function A (SAME70 datasheet, pin multiplexing table)
- 12 MHz internal oscillator as the clock at reset (SAME70 datasheet, PMC chapter)
- The order in which `cortex-m-rt` prepares `.bss` and `.data` (check the version actually used)
