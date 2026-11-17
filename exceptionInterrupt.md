# Exceptions and interrupts (Phase 4)

This explains what Phase 4 of `PLAN.md` is about: what exceptions and interrupts are on the Cortex-M7, what the processor does when one happens, and what the emulator needs in order to do the same.

The authoritative source is the ARMv7-M Architecture Reference Manual in the repo root (`DDI0403E_e_armv7m_arm.pdf`), chapter B1 for the exception model and chapter B3 for the NVIC and SysTick registers.

## The idea

So far the emulator runs one instruction after another, and the only thing that changes the order is the program itself (branches and calls). An **exception** is the processor changing the order without being asked:

1. Something happens: a timer runs out, a byte arrives on the UART, the program does something illegal.
2. The processor finishes the current instruction, saves where it was, and jumps to a function written for that event, called a **handler**.
3. When the handler returns, the processor restores what it saved and carries on as if nothing had happened.

The interrupted code does not know it was interrupted. That is the whole point: a program can be in the middle of a calculation and still react to a UART byte within microseconds.

An **interrupt** is one kind of exception: one that comes from a peripheral outside the processor core (UART, timer, pin change). Arm uses "exception" for the whole family and "interrupt" or "IRQ" for the peripheral ones.

## The kinds of exception

Every exception has a number. The number decides which handler runs.

| Number | Name | Cause |
|---|---|---|
| 1 | Reset | Power-up or reset. Already emulated by `Cpu::reset`. |
| 2 | NMI | Non-maskable interrupt: cannot be switched off |
| 3 | HardFault | A fault with no more specific handler available |
| 4 | MemManage | Access the MPU forbids |
| 5 | BusFault | Access to memory that does not exist |
| 6 | UsageFault | Undefined instruction, divide by zero, and similar |
| 11 | SVCall | The `SVC` instruction: a deliberate call into an operating system |
| 14 | PendSV | Requested by software, used by operating systems to switch tasks |
| 15 | SysTick | The system timer reached zero |
| 16 + n | IRQ n | Peripheral interrupt n. UART0 is IRQ 7, so exception 23. |

The IRQ numbers come from the SVD file (the `<interrupt>` entries). The SAME70 has about 70 of them.

## The vector table

The vector table is a list of addresses at the start of flash. The emulator already uses the first two entries at reset:

| Word | Address | Contents |
|---|---|---|
| 0 | `0x0040_0000` | Initial stack pointer |
| 1 | `0x0040_0004` | Reset handler |
| 2 | `0x0040_0008` | NMI handler |
| 3 | `0x0040_000C` | HardFault handler |
| ... | | |
| 15 | `0x0040_003C` | SysTick handler |
| 16 + n | `0x0040_0040 + 4n` | Handler for IRQ n |

To find the handler for exception number `k`, the processor reads the word at `VTOR + 4 × k`. `VTOR` is the Vector Table Offset Register in the SCB (`0xE000_ED08`), which lets firmware move the table. The SCB stub already stores it.

`cortex-m-rt` builds this table for the Rust firmware. Handlers you do not write yourself point at a default handler that loops forever.

## What happens on entry

When the processor takes an exception it does this, with no instructions involved (ARM manual B1.5.6):

**1. Push eight registers onto the current stack.** This block is called the exception frame.

| Offset from new SP | Register |
|---|---|
| `+0x00` | R0 |
| `+0x04` | R1 |
| `+0x08` | R2 |
| `+0x0C` | R3 |
| `+0x10` | R12 |
| `+0x14` | LR |
| `+0x18` | Return address (the instruction to resume at) |
| `+0x1C` | xPSR (the flags and other status) |

These eight are exactly the registers a normal function is allowed to overwrite. Because the processor saves them, a handler can be an ordinary compiled function: it saves R4–R11 itself if it uses them, like any function does.

The stack is also aligned to 8 bytes first (B1.5.7). If that needed 4 bytes of padding, bit 9 of the saved xPSR is set so the return knows to remove it.

**2. Put a special value in LR**, called `EXC_RETURN`. It is not a real address. It records how to get back (see the next section).

**3. Switch to Handler mode** and record the exception number in the low 9 bits of xPSR (a field called IPSR).

**4. Load the PC from the vector table.**

## What happens on return

A handler ends like any function: `bx lr` or `pop {..., pc}`. That loads the `EXC_RETURN` value into the PC. No real code lives at `0xFFFF_FFxx`, and the processor recognises the value as "end of exception" rather than as an address (ARM manual B1.5.8).

| `EXC_RETURN` | Return to | Using stack |
|---|---|---|
| `0xFFFF_FFF1` | Handler mode (this exception interrupted another handler) | Main |
| `0xFFFF_FFF9` | Thread mode | Main |
| `0xFFFF_FFFD` | Thread mode | Process |

The processor then pops the eight-word frame, which restores R0–R3, R12, LR, the PC and xPSR, and the interrupted code continues.

This is the second job of LR. The check is made by the instructions that can return from a function: `BX`, a `POP` that includes the PC, and a load (`LDR`, `LDM`) into the PC. In Handler mode, a value starting with `0xF` written to the PC by one of these is an exception return, not a branch target. A plain `MOV pc, lr` does not count.

## Modes and the two stacks

| | Thread mode | Handler mode |
|---|---|---|
| Runs | Normal program code | Exception handlers |
| Entered | At reset, and on return from the last handler | On exception entry |
| Stack | Main or Process, chosen by `CONTROL` bit 1 | Always Main |

There are two stack pointers, **MSP** (main) and **PSP** (process). Register R13 (SP) is whichever one is currently selected. Simple firmware, including `hello-rs`, uses only MSP. An operating system gives each task its own stack through PSP and keeps MSP for handlers, so a task with a full stack cannot break interrupt handling.

`Cpu` already has `msp`, `psp` and `control` fields; nothing uses them yet.

## Priorities: which exception wins

Each exception has a priority (ARM manual B1.5.4). **A lower number is a higher priority.**

- Reset (-3), NMI (-2) and HardFault (-1) are fixed and beat everything else.
- All others have a priority the firmware sets. The SAME70 implements 3 priority bits, giving 8 levels, held in the top 3 bits of an 8-bit field (so the usable values are `0x00`, `0x20`, ... `0xE0`).

An exception can be in these states:

- **Pending**: it has been requested and is waiting.
- **Active**: its handler is running (or was interrupted by a higher-priority one).

A pending exception is taken when its priority is higher than the priority of whatever is running now. So a handler can itself be interrupted by a more urgent one; this is **nesting**, and it is why `0xFFFF_FFF1` (return to Handler mode) exists. An exception of equal or lower priority stays pending until the current handler finishes.

Firmware can also hold exceptions back with three mask registers:

| Register | Effect when set |
|---|---|
| `PRIMASK` | Blocks everything except NMI and HardFault. This is "interrupts off". |
| `FAULTMASK` | Blocks everything except NMI |
| `BASEPRI` | Blocks exceptions at or below a chosen priority level |

`PRIMASK` is set and cleared by the `CPSID i` and `CPSIE i` instructions. All three can be read and written with `MRS` and `MSR`. `Cpu` already has the fields.

## The NVIC

The Nested Vectored Interrupt Controller is the part of the core that keeps track of the peripheral interrupts. It is a block of registers starting at `0xE000_E100` (ARM manual B3.4). Each has one bit per IRQ (32 IRQs per register), except the priorities, which have one byte per IRQ.

| Register | Address | Purpose |
|---|---|---|
| `ISER` | `0xE000_E100` | Write 1 to enable an IRQ |
| `ICER` | `0xE000_E180` | Write 1 to disable an IRQ |
| `ISPR` | `0xE000_E200` | Write 1 to make an IRQ pending |
| `ICPR` | `0xE000_E280` | Write 1 to clear a pending IRQ |
| `IABR` | `0xE000_E300` | Read-only: which IRQs are active |
| `IPR` | `0xE000_E400` | Priority, one byte per IRQ |

An IRQ's handler runs only if the IRQ is **pending**, **enabled**, and of high enough priority. A peripheral can make its IRQ pending while it is disabled; it then simply waits.

There are usually two switches to turn on for an interrupt to work: one in the peripheral (for UART0, the RXRDY bit in `UART_IER`) and one in the NVIC (`ISER` bit 7). The system exceptions (SysTick, PendSV, the faults) are not in the NVIC; they are controlled from the SCB.

## SysTick

SysTick is a 24-bit countdown timer built into the core, at `0xE000_E010` (ARM manual B3.3). Almost every firmware uses it for a regular "tick", typically every millisecond.

| Register | Address | Purpose |
|---|---|---|
| `SYST_CSR` | `0xE000_E010` | ENABLE (bit 0), TICKINT (bit 1: raise the exception at zero), COUNTFLAG (bit 16: reached zero since last read) |
| `SYST_RVR` | `0xE000_E014` | Reload value |
| `SYST_CVR` | `0xE000_E018` | Current value |
| `SYST_CALIB` | `0xE000_E01C` | Calibration (read-only) |

While enabled it counts down once per clock cycle. On reaching zero it reloads from `SYST_RVR`, sets COUNTFLAG, and, if TICKINT is set, makes exception 15 pending.

The emulator has no clock yet. The simplest stand-in is to count down once per emulated instruction. Real timing (cycle counting) is a Phase 5 item.

## Faults

A fault is an exception caused by the program doing something wrong (ARM manual B1.5.14). Today the emulator panics in those places. On a real chip:

- An undefined instruction raises **UsageFault**.
- An access to unmapped memory raises **BusFault**.
- If that specific fault is not enabled (they are off at reset), it **escalates** to **HardFault**.

Firmware often has a HardFault handler that prints diagnostics, so emulating the fault properly lets that code run. The plan also asks for the emulator itself to print a dump of the faulting state (registers, the stacked frame, the fault status registers in the SCB), which is far more useful for debugging than the firmware's own output.

One caution: unimplemented instructions should keep panicking. A panic means "the emulator is missing something"; a fault means "the firmware did something wrong". Mixing them up would hide the to-do list.

## UART receive interrupt

The last Phase 4 item makes input work. Typed characters on the host's stdin become received bytes on UART0:

1. A byte arrives from stdin. The UART stub stores it in `UART_RHR` (`+0x18`) and sets RXRDY (bit 0) in `UART_SR`.
2. If the firmware enabled the RXRDY interrupt in `UART_IER` (`+0x08`), the stub makes IRQ 7 pending.
3. If IRQ 7 is enabled in the NVIC, the processor takes exception 23 and runs the firmware's UART0 handler.
4. The handler reads `UART_RHR`, which clears RXRDY.

## What the emulator needs

| Piece | Where | Notes |
|---|---|---|
| Pending and active state for every exception | new | Shared by the CPU and the registers below |
| NVIC registers | new stub at `0xE000_E100` | Enable, pending, priority |
| SysTick registers and countdown | new stub at `0xE000_E010` | Counted per instruction for now |
| `ICSR`, `SHPR`, `SHCSR` behaviour | `src/peripheral/scb.rs` | Currently store-and-readback or logged |
| Check for a pending exception | `Cpu::step` | Before fetching each instruction |
| Exception entry | `src/cpu/exception.rs` | Push the frame, set LR, load the PC |
| Exception return | `bx_write_pc` in `src/cpu/exception.rs` | Used by `BX` and `POP`; loads into the PC will need it too |
| MSP / PSP switching | `src/cpu/exception.rs` | Done for entry and return; writing `CONTROL` with `MSR` is still to come |
| A way for a peripheral to raise an IRQ | `Peripheral` trait or the bus | The UART stub has no route to the CPU today |

The last row is the main design question. At present a peripheral can only answer reads and writes; it cannot tell anyone that something happened. The SCB stub ran into the same limit ("logged but not carried out"). One workable shape is for the bus to own the exception state, for peripherals to set a pending bit there, and for the CPU to ask the bus once per step whether anything is waiting.

New instructions will be needed as well, as real firmware hits them: `BX`, `CPSID` / `CPSIE`, `MRS` / `MSR`, `SVC`, and `WFI` (wait for interrupt, which firmware uses to idle).

## A suggested order

Each step can be tested before the next one starts.

1. **Entry and return, triggered by hand.** Write the frame push and pop, and test them by making an exception pending directly from a unit test.
2. **SysTick.** It needs no peripheral and no NVIC, so it is the smallest real source. Firmware test: a SysTick handler that prints a character on every tick.
3. **PRIMASK and priorities.** `CPSID` / `CPSIE`, then priority comparison and nesting.
4. **NVIC and the UART receive interrupt.** The first peripheral interrupt, and the first use of stdin.
5. **Faults.** HardFault with the state dump.
6. **MSP / PSP.** Only matters once firmware uses the process stack, which mostly means an operating system.

## Left out on purpose

- **Tail-chaining and late arrival.** Real hardware skips the pop-then-push when one handler ends and another is waiting. The result is the same as returning and immediately re-entering, so the emulator can do it the simple way.
- **The FPU's extended frame.** With the FPU on, entry pushes 26 words instead of 8 and `EXC_RETURN` has other values (`0xFFFF_FFE1` and so on). That belongs with the FPU in Phase 5.
- **MPU faults.** MemManage only matters once the MPU is emulated.
