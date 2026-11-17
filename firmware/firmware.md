# Rust firmware for the emulator

This describes the first compiled firmware for Phase 3 of `PLAN.md`: a bare-metal Rust program that sets up UART0 and prints a line, starts a 1 ms SysTick that keeps a millisecond counter, waits 500 ms, and from then on echoes every byte it receives on UART0 from its receive interrupt handler. The project is in `firmware/hello-rs/` and builds to `firmware/hello-rs.bin`. The hand-written `hello.s` and its `firmware.bin` are kept as a known-good test.

"Bare-metal" means the program runs directly on the chip with no operating system. Nothing prepares memory or calls `main` for it, so the program has to bring its own startup code.

## Pieces

| Piece | What it is | Who provides it |
|---|---|---|
| Vector table | The first words of flash: initial stack pointer, then the address of the reset code and every exception handler | `cortex-m-rt` crate |
| Startup code | Runs at reset, prepares RAM, then calls `main` | `cortex-m-rt` crate |
| Memory layout | Where flash and RAM are on this chip | `memory.x`, written by us |
| Panic handler | What to do if the program panics | `panic-halt` crate |
| Interrupt vector table | The words after the first 16: the address of every peripheral interrupt handler | `__INTERRUPTS` in `main.rs`, written by us |
| `main` | Disable the watchdog, set up UART0, print, start SysTick, enable the receive interrupt | written by us |

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
    device.x            empty; see "UART echo" below
    build.rs            puts memory.x and device.x where the linker looks for them
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
cargo run -- --bus-faults
cargo run -- firmware/firmware.bin
```

`--bus-faults` makes a read or write of an unmapped address a BusFault, as on the real chip, instead of a warning. When firmware faults, the emulator prints a dump of the registers; `exceptionInterrupt.md` explains it.

The emulator's stdin is UART0's receive line. Type a line and press Enter and the firmware echoes it; the terminal hands input over a line at a time, so nothing arrives before Enter. Input can also be piped in:

```
printf 'hello\n' | cargo run
```

Stop the emulator with Ctrl-C.

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

## SysTick, `millis` and `delay_ms`

After printing its line, the firmware starts the Arm core's system timer with a period of 1 ms. This is the usual way firmware keeps time: the handler only counts, and everything else is built on the count. `exceptionInterrupt.md` in the repo root explains exceptions and how the emulator counts time.

### Set-up

| Order | Register | Address | Value | Meaning |
|---|---|---|---|---|
| 1 | `SYST_RVR` | `0xE000_E014` | `11_999` | Reload value: a period of 12,000 clock cycles, 1 ms at 12 MHz |
| 2 | `SYST_CVR` | `0xE000_E018` | `0` | Any write clears the counter, so the first period is a full one |
| 3 | `SYST_CSR` | `0xE000_E010` | `0b111` | ENABLE, TICKINT (raise the exception at zero), CLKSOURCE (processor clock) |

### The handler and the counter

```rust
static TICKS: AtomicU32 = AtomicU32::new(0);

#[exception]
fn SysTick() {
    TICKS.store(TICKS.load(Ordering::Relaxed).wrapping_add(1), Ordering::Relaxed);
}
```

`#[exception]` comes from `cortex-m-rt`. It puts the function's address in the vector table at word 15 (`0x0040_003C`), the entry for the SysTick exception. The function itself is ordinary code: the processor saves and restores the interrupted registers around it.

`TICKS` is the number of milliseconds since the timer started. It is an atomic because the handler and the main code share it. A separate load and store is enough here, because the handler is the only writer and cannot interrupt itself.

`TICKS` is the firmware's first global variable. It lives in RAM in the `.bss` section, which the startup code fills with zeros before `main` runs, so the zeroing loop described under "Minimal startup" now has real work to do.

### Using the count

```rust
fn millis() -> u32 {
    TICKS.load(Ordering::Relaxed)
}

fn delay_ms(ms: u32) {
    let start = millis();
    while millis().wrapping_sub(start) < ms {
        wait_for_interrupt();
    }
}
```

`delay_ms` compares elapsed time (`now - start`) with the wanted delay, not the current time with a deadline. That keeps working when the 32-bit counter wraps round to 0, which happens after about 49.7 days.

`main` uses it once, between the greeting and the prompt:

```rust
systick_init();
delay_ms(500);
uart_write_str("Type something:\n");
```

`wait_for_interrupt` is the `WFI` instruction, written with `core::arch::asm!`. It stops the processor until the next exception. The count can only change when SysTick fires, so there is no point checking it more often than that; on a real chip sleeping in between saves power.

The emulator skips the sleep: when the CPU is asleep it jumps emulated time straight to the next SysTick. So the prompt appears almost at once on the host, while being 6,000,000 emulated cycles (500 ms at 12 MHz) after the timer started.

## UART echo: the receive interrupt

After the prompt, the firmware turns on UART0's receive interrupt and the main loop only sleeps. Each received byte runs the UART0 handler, which sends the byte back.

### Set-up

An interrupt has two switches, and both must be on:

| Order | Register | Address | Value | Meaning |
|---|---|---|---|---|
| 1 | `UART_IER` | `0x400E_0808` | `1 << 0` (RXRDY) | In the UART: ask for the interrupt when a byte has been received |
| 2 | `NVIC_ISER0` | `0xE000_E100` | `1 << 7` | In the Arm core's interrupt controller: let IRQ 7 (UART0) run its handler |

The receiver itself was already switched on by RXEN in `uart_init`.

### The handler

```rust
#[unsafe(no_mangle)]
#[allow(non_snake_case)]
extern "C" fn UART0() {
    if read_reg(UART0_SR) & UART_SR_RXRDY != 0 {
        let byte = read_reg(UART0_RHR) as u8;
        uart_write_byte(byte);
    }
}
```

Reading `UART_RHR` (`0x400E_0818`) fetches the byte and clears RXRDY, which makes the UART stop asking for the interrupt. A handler that returned without reading it would be called again at once, for ever.

Calling `uart_write_byte` from a handler is acceptable here because the transmitter is ready almost immediately. On a real board at 9600 baud each byte takes about 1 ms to send, so firmware that receives a lot usually stores the byte in a buffer and lets the main loop do the sending.

### The interrupt vector table

`#[exception]` works for `SysTick` because `cortex-m-rt` knows the 16 system exceptions, which are the same on every Cortex-M. It does not know which peripheral interrupts a particular chip has, so that part of the vector table is ours to supply:

```rust
#[unsafe(link_section = ".vector_table.interrupts")]
#[unsafe(no_mangle)]
#[used]
pub static __INTERRUPTS: [unsafe extern "C" fn(); 8] = [
    DefaultHandler, // IRQ 0
    // ... IRQ 1 to 6 the same
    UART0,          // IRQ 7
];
```

- Entry n is the handler for IRQ n. `cortex-m-rt`'s linker script places the section straight after the 16 system entries, so `UART0` ends up at word 23 (`0x0040_005C`), where the processor looks for exception 16 + 7.
- `DefaultHandler` is `cortex-m-rt`'s catch-all, an endless loop.
- The `device` feature of `cortex-m-rt` (in `Cargo.toml`) is what tells it not to supply a table of its own. With that feature its linker script also includes a file called `device.x`, so an empty one is provided.

Normally a "peripheral access crate" generated from the chip's SVD file supplies this table, with all the interrupts by name. Writing it by hand shows what such a crate does.

## What the emulator does with this

| Step | Emulator behaviour |
|---|---|
| Watchdog disable | Stored; the emulated watchdog never expires anyway |
| Clock enable | Stored in `PMC_PCSR0`; the UART stub works with or without it |
| Pin set-up | Stored; no pins exist |
| `UART_CR` | RXEN switches the emulated receiver on; the transmitter works regardless |
| `UART_MR`, `UART_BRGR` | Ignored |
| `UART_SR` | Always reports TXRDY |
| `UART_THR` | Byte goes to stdout |
| SysTick set-up | The emulated timer starts counting clock cycles |
| SysTick reaching zero | The handler runs and adds 1 to `TICKS`, every 12,000 emulated cycles |
| `WFI` | The CPU sleeps; emulated time jumps to the next SysTick |
| `delay_ms(500)` | Takes 6,000,000 emulated cycles, which the host gets through almost at once |
| `UART_IER`, `NVIC_ISER0` | The emulated UART and NVIC pass the receive interrupt through |
| A byte on stdin | Goes into `UART_RHR`; the UART0 handler runs and echoes it |

So only step 5 is needed for UART output in the emulator, plus RXEN from step 4 for input. The rest is there so the same firmware would work on a real board.

When the firmware uses an instruction the CPU core does not implement yet, the emulator stops with an "unhandled instruction" panic that names the next one to add. That is different from a fault, which is the firmware doing something illegal: the emulator then prints a dump and runs the firmware's HardFault handler, which in `hello-rs` is `cortex-m-rt`'s default endless loop.

## The hard-float firmware: `float-rs`

`firmware/float-rs/` is a second firmware, built to use the FPU (the floating-point unit: hardware that does arithmetic on `f32` and `f64` values). It builds to `firmware/float-rs.bin`.

```
firmware/float-rs/build.sh
cargo run -- firmware/float-rs.bin
```

One-time set-up: `rustup target add thumbv7em-none-eabihf`.

### Soft-float and hard-float

| | `hello-rs` | `float-rs` |
|---|---|---|
| Target | `thumbv7em-none-eabi` | `thumbv7em-none-eabihf` |
| Floating-point arithmetic | Done by library code using integer instructions | Done by FPU instructions (`vadd`, `vmul`, ...) |
| Floating-point arguments | Passed in R0–R3 | Passed in S0–S15 / D0–D7 |

`.cargo/config.toml` also sets `-C target-cpu=cortex-m7`. Without it the target assumes an FPU with single precision only, and `f64` arithmetic would still be done in software.

### Turning the FPU on

The FPU is off after reset, and any FPU instruction is then a UsageFault. Firmware switches it on by setting bits 20–23 of `CPACR` (`0xE000_ED88`). On a hard-float target `cortex-m-rt`'s startup code does this before `main` runs, so `float-rs` has no code for it.

### What it does

1. Prints the results of `f32` and `f64` arithmetic, conversions between the two and to and from integers, and comparisons. Each result is printed as the hex bit pattern of the value, because printing a decimal number needs far more code. `core::hint::black_box` hides the inputs from the compiler; otherwise it would work the answers out at build time and no FPU instruction would run.
2. Starts SysTick with a short period and a handler that does floating-point arithmetic of its own. Meanwhile the main code repeats one calculation and checks that the answer never changes. This tests that the FPU registers are saved and restored around the handler (see "The FPU and the exception frame" in `exceptionInterrupt.md`).

Expected output:

```
FPU test
f32 add = 40700000
...
ticks = 000000C8
handler total = 42C80000
rounds > 0 = 00000001
mismatches = 00000000
done
```

`40700000` is the bit pattern of 3.75 (1.5 + 2.25). `mismatches = 00000000` is the line that matters in the second half. The whole output matches, bit for bit, the same program run natively on the host.

### What the emulator does

The FPU is in `src/cpu/fpu.rs`. The arithmetic is done with the host's own `f32` and `f64`, which follow the same standard (IEEE 754) as the chip's FPU. The file's header lists the differences: arithmetic always rounds to nearest, the exception flags in `FPSCR` are never set, and a few special modes are ignored.

Building this firmware also needed several dozen ordinary integer instructions that `hello-rs` never used. They are in `src/cpu/data.rs`.

## To verify

These are from memory, not from a document in the repo:

- UART0 on PA9 / PA10, peripheral function A (SAME70 datasheet, pin multiplexing table)
- 12 MHz internal oscillator as the clock at reset (SAME70 datasheet, PMC chapter)
- The order in which `cortex-m-rt` prepares `.bss` and `.data` (check the version actually used)
- That the SAME70's FPU has double precision. The emulator reports and implements it (`MVFR0` = `0x10110221`), but the SVD's reset value for `MVFR0` (`0x10110021`) describes a single-precision FPU (SAME70 datasheet, processor features)
- The cycle costs of FPU instructions (Cortex-M7 documentation)
