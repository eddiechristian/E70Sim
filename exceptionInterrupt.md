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

In the emulator `regs[SP]` is always the stack in use, and the other one is kept in `Cpu`'s `msp` or `psp` field. Exception entry and return switch between them, and firmware can with `MSR` (see "Changing the masks" below).

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

`PRIMASK` is set and cleared by the `CPSID i` and `CPSIE i` instructions. All three can be read and written with `MRS` and `MSR`.

### Where the priorities are set

| Exception | Register | Address |
|---|---|---|
| System exceptions 4 to 15 | `SHPR1`–`SHPR3` in the SCB, one byte each | `0xE000_ED18 + (number - 4)`; SysTick is `0xE000_ED23` |
| IRQ n | `IPR` in the NVIC, one byte each | `0xE000_E400 + n` |

Every priority is 0 after reset, the highest a configurable exception can have. So until firmware sets priorities, nothing can interrupt a handler. When two waiting exceptions have the same priority, the lower exception number goes first.

### Group priority and subpriority

`AIRCR.PRIGROUP` (SCB, `0xE000_ED0C`) can split the priority value in two. The upper part, the **group priority**, decides whether one exception can interrupt another. The lower part, the **subpriority**, only decides the order among exceptions that are waiting. With the reset value all 3 implemented bits are group priority, which is what most firmware uses.

### How the emulator does it

This follows the `ExecutionPriority` pseudocode in B1.5.4.

- The bus keeps the set of **active** exceptions (`activate_exception` on entry, `deactivate_exception` on return). The priority of the running code is the most urgent group priority among them, or 256 in Thread mode, a value that anything can beat (`Bus::active_priority`).
- `Cpu::execution_priority` then applies `BASEPRI`, `PRIMASK` and `FAULTMASK`.
- Before each instruction the CPU asks the bus for a pending exception more urgent than that (`Bus::take_pending_exception`). The bus picks the most urgent one waiting and hands it over only if its group priority is strictly higher. The CPU enters it whether it was in Thread mode or in a handler.

Nesting needs nothing special on the stack: the second frame is simply pushed below the first, and its `EXC_RETURN` is `0xFFFF_FFF1`, "back to a handler".

### Pending an exception from firmware: `ICSR` and `STIR`

Firmware can ask for an exception itself, with no timer or peripheral involved.

| Register | Address | Write | Read |
|---|---|---|---|
| `ICSR` | `0xE000_ED04` | Bit 31 raises NMI. Bit 28 pends PendSV and bit 27 withdraws it. Bit 26 pends SysTick and bit 25 withdraws it. | The same "set" bits show what is pending. Bits 8–0 (`VECTACTIVE`) are the exception running now, bits 20–12 (`VECTPENDING`) the most urgent one waiting, bit 22 says some IRQ is pending, and bit 11 (`RETTOBASE`) says the running handler did not interrupt another one. |
| `STIR` | `0xE000_EF00` | The number of an IRQ to make pending | — |

The main user is an operating system. **PendSV** exists for this: it is given the lowest priority, and any handler that decides a task switch is needed writes `PENDSVSET`. PendSV then runs only after every other handler has finished, and does the switch there.

The bus handles both registers (`read_icsr` and `write_icsr` in `src/bus.rs`), because it is what keeps the pending and active state. To report `VECTACTIVE` it needs to know which handler is running, so the CPU tells it on every exception entry and return (`set_current_exception`).

### Changing the masks: `CPSID` / `CPSIE`, `MRS` and `MSR`

These are in `src/cpu/system.rs` (ARM manual B5.2).

| Instruction | What it does |
|---|---|
| `cpsid i` / `cpsie i` | Set / clear `PRIMASK`: interrupts off / on |
| `cpsid f` / `cpsie f` | Set / clear `FAULTMASK` |
| `mrs r0, <register>` | Read a special-purpose register into R0 |
| `msr <register>, r0` | Write R0 to a special-purpose register |

The special-purpose registers `MRS` and `MSR` can name are `PRIMASK`, `FAULTMASK`, `BASEPRI`, `BASEPRI_MAX`, `CONTROL`, `MSP`, `PSP` and the parts of the xPSR.

The usual use is a **critical section**: a few instructions that must not be interrupted, for example main code updating a variable that a handler also uses.

```
mrs   r0, primask   ; remember whether interrupts were already off
cpsid i             ; interrupts off
...                 ; the protected instructions
msr   primask, r0   ; put it back the way it was
```

An exception that arrives in between is not lost. It stays pending and is taken right after interrupts come back on.

Details worth knowing:

- `BASEPRI_MAX` is `BASEPRI` with a safety catch: a write only takes effect if it masks more than before, so nested critical sections cannot accidentally unmask.
- `FAULTMASK` is cleared automatically when a handler returns (other than NMI's). `PRIMASK` is not.
- Writing `CONTROL` bit 1 in Thread mode switches to the process stack. Bit 0 makes Thread mode **unprivileged**: from then on its writes to the masks, `CONTROL` and the stack pointers are ignored and its reads of them return 0. Only an exception handler can undo that.

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

### How the emulator counts time: clock cycles

A real SysTick counts clock cycles. The emulator has no oscillator, so it keeps a count of **emulated clock cycles** instead. Every instruction reports how many cycles it took, `Cpu::step` adds that to a running total and passes it to `Bus::tick`, and SysTick moves by that amount.

So a tick lands every N + 1 emulated cycles (N being the reload value), whatever mix of instructions ran in between. Firmware that sets up a 1 ms tick for a 12 MHz clock writes a reload value of 11,999 and gets its handler every 12,000 cycles, which is what its author meant.

The costs are in `src/cpu.rs`:

| What | Cycles | Constant |
|---|---|---|
| Arithmetic, logic, moves, shifts, compares | 1 | `CYCLES_DATA` |
| One load or store | 2 | `CYCLES_MEMORY` |
| `PUSH` / `POP` | 1 + one per register | |
| Anything that changes the PC (taken branch, `BL`, `BX`, `POP` with PC, `MOV pc`) | 2 extra | `CYCLES_REFILL` |
| Conditional branch not taken | 1 | |
| Exception entry | 12 | `CYCLES_EXCEPTION_ENTRY` |
| Exception return | 10 extra, in place of the 2 | `CYCLES_EXCEPTION_RETURN` |

Every new instruction has to return its cost; the compiler enforces that, because the instruction methods return `u32`.

Three things to keep in mind:

- **The costs are estimates.** They follow the timings Arm publishes for the Cortex-M4. The Cortex-M7 runs two instructions at once where it can, predicts branches and has caches, so the true cost of an instruction depends on its neighbours and on history, and Arm publishes no simple table for it. Ratios between pieces of code come out roughly right; absolute numbers do not match the real chip.
- **It is emulated time, not real time.** A "1 second" delay takes however long the host needs to emulate that many cycles, and almost no time at all if the firmware sleeps in `WFI` meanwhile. Pacing the emulator against the host's clock, so that 1 ms takes 1 ms, would build on the cycle count but is not implemented.
- **It is exactly repeatable.** The same firmware takes its SysTick exception at the same instruction on every run.

The running total is also what `DWT_CYCCNT` (the core's cycle counter register, a Phase 5 item) will report. With `--trace`, each step prints the total and the cycles that step took.

The CLKSOURCE bit of `SYST_CSR` is ignored. On the real chip it selects between the processor clock and a slower reference clock.

## Faults

A fault is an exception caused by the program doing something wrong (ARM manual B1.5.14). The code for this is in `src/cpu/fault.rs`.

### What raises a fault

| The firmware does | Fault | Bit set in `CFSR` |
|---|---|---|
| Executes `udf`, the "permanently undefined" instruction. Compilers emit it to stop on a failed check | UsageFault | UNDEFINSTR |
| Branches to an address without bit 0 set (the Thumb bit), or has a vector table entry without it | UsageFault | INVSTATE |
| Returns from an exception with a bad `EXC_RETURN` value, or one for the wrong mode | UsageFault | INVPC |
| Runs off into an address where there is no memory | BusFault | IBUSERR |
| Runs off into the peripheral or system area (`0x4000_0000` and up) | MemManage | IACCVIOL |
| Reads or writes an unmapped address, **only with `--bus-faults`** | BusFault | PRECISERR |

For the Thumb-bit case the branch itself works; the fault comes when the instruction at the target tries to execute. This is how the real chip behaves, and it means the saved return address is the bad target.

### The `--bus-faults` switch

On the real chip an access to an address where nothing exists is a BusFault. In the emulator "nothing exists there" often means "that peripheral is not emulated yet", and a warning that lets the firmware carry on is more useful: the warnings are the list of peripherals the firmware wants.

So this is a switch:

| | Unmapped read or write |
|---|---|
| Default | Prints `WARNING: Unmapped ...`, reads 0, carries on |
| `cargo run -- --bus-faults` | BusFault, as on the real chip |

Use the switch when the firmware runs with no warnings and you want wild pointers caught.

### Escalation to HardFault

MemManage, BusFault and UsageFault are switched off after reset. Firmware enables them with bits 16, 17 and 18 of `SHCSR` (`0xE000_ED24`). A fault uses its own exception only if that is enabled and has a priority high enough to interrupt the running code. Otherwise it **escalates**: it is delivered as HardFault, which is always on, and bit 30 (FORCED) is set in `HFSR`.

So in firmware that enables nothing, such as `hello-rs`, every fault arrives at the HardFault handler. `cortex-m-rt`'s default one loops for ever.

### What the firmware's handler sees

A fault is taken like any other exception: the 8-word frame is pushed and the handler runs. The saved return address is the instruction that faulted, and the registers are as they were before it.

| Register | Address | Holds |
|---|---|---|
| `CFSR` | `0xE000_ED28` | Which fault: MemManage bits 7–0, BusFault bits 15–8, UsageFault bits 31–16 |
| `HFSR` | `0xE000_ED2C` | Bit 30: this HardFault is an escalated fault |
| `BFAR` | `0xE000_ED38` | The address of a failed data access, when `CFSR` bit 15 (BFARVALID) is set |

The status bits stay set until firmware writes 1 to them.

### The emulator's dump

Whenever a fault is raised the emulator prints the state to stderr, before the handler runs:

```
==== FAULT at PC 0x00400100 ====
  cause: BusFault: read of unmapped address 0x60000000 (CFSR.PRECISERR, BFAR valid)
  taken as: HardFault (BusFault is disabled or cannot interrupt the running code; HFSR.FORCED)
  instruction: 0x6801
  r0  = 60000000  r1  = FFFFFFFF  r2  = 00000000  r3  = 00000000
  ...
  r12 = 00000000  sp  = 20460000  lr  = 00000000  pc  = 00400100
  xpsr = 01000000  flags ----
  mode: Thread, main stack
  masks: primask = 0, faultmask = 0, basepri = 0x00
  cycles since reset: 0
====
```

Look the PC up in the disassembly (see `firmware/firmware.md`) to find the line of code.

### Lockup

A fault while HardFault or NMI is already running, or with `FAULTMASK` set, has no handler that can take it. The real chip stops in a state called **lockup**. The emulator prints the dump and stops with a `LOCKUP` panic.

This is also what happens to firmware with no HardFault handler at all. The assembly `hello.s` has only two vector table entries, so its HardFault "vector" is whatever bytes come next; a fault there ends in lockup.

### Not a fault: unimplemented instructions

An instruction the emulator has not implemented still panics with "unhandled instruction". A panic means "the emulator is missing something"; a fault means "the firmware did something wrong". Only `udf`, which is undefined by design, raises UNDEFINSTR.

### Not emulated

- A fault while pushing or popping an exception frame (a stack that has run off the end of RAM). With `--bus-faults` the emulator stops with a message; without it, the usual warnings appear.
- Divide by zero and unaligned access traps; there is no divide instruction yet.
- `MMFAR`, and the "active" and "pended" bits of `SHCSR`.

## UART receive interrupt

The last Phase 4 item makes input work. Typed characters on the host's stdin become received bytes on UART0:

1. A byte arrives from stdin. If the firmware has switched the receiver on (RXEN in `UART_CR`), the UART stores it in `UART_RHR` (`+0x18`) and sets RXRDY (bit 0) in `UART_SR`.
2. If the firmware enabled the RXRDY interrupt in `UART_IER` (`+0x08`), the UART asks for IRQ 7, which makes it pending in the NVIC.
3. If IRQ 7 is enabled in the NVIC, the processor takes exception 23 and runs the firmware's UART0 handler.
4. The handler reads `UART_RHR`, which clears RXRDY, so the UART stops asking.

Step 4 matters. A peripheral's interrupt request is a **level**: it stays on until the cause is dealt with. A handler that returns without reading `UART_RHR` is called again at once, for ever. This is true on the real chip too, and it is the usual reason for firmware that "hangs" in an interrupt.

How this is done in the emulator:

- A thread reads the host's stdin and passes the bytes down a channel (`stdin_bytes` in `src/peripheral/uart.rs`). After every instruction the bus lets the UART take one byte from the channel, if `UART_RHR` is free.
- The bus then asks the UART whether it wants its interrupt and tells the NVIC (`update_irq_lines` in `src/bus.rs`). This is the route from a peripheral to the CPU.
- When the CPU asks the bus for the next exception, the bus considers the pending system exceptions and the IRQs that are pending and enabled in the NVIC, and picks by priority (see "Priorities" above).

Two things differ from a real board:

- **A terminal sends a line at a time.** Typed characters reach the emulator when Enter is pressed, then arrive one after another. Piped input (`printf 'abc' | cargo run`) arrives straight away.
- **Bytes are never lost.** A real UART overwrites `UART_RHR` if the firmware is too slow, and sets the OVRE error bit. The emulator keeps the next byte back until the last one has been read, because the speed of the host and the speed of emulated time are unrelated.

When a byte arrives is not tied to the emulated clock, so two runs can see it at different cycle counts. Everything else in the emulator repeats exactly.

## What the emulator needs

| Piece | Where | Notes |
|---|---|---|
| Pending and active state for every exception | `Bus` (`pend_exception`, `take_pending_exception`, `activate_exception`) | Done |
| Priorities, pre-emption and nesting | `Bus` (`exception_priority`, `active_priority`), `Cpu::execution_priority` | Done, including priority grouping |
| `PRIMASK`, `FAULTMASK`, `BASEPRI` | `Cpu::execution_priority`, `src/cpu/system.rs` | Done, with `CPSID` / `CPSIE`, `MRS` and `MSR` |
| NVIC registers | `src/peripheral/nvic.rs` | Done: enable, pending, active and priority |
| SysTick registers and countdown | `src/peripheral/systick.rs` | Done. Counts emulated clock cycles |
| `ICSR` and `STIR` | `read_icsr`, `write_icsr` in `src/bus.rs` | Done. Firmware can pend NMI, PendSV, SysTick or any IRQ, and read which exception is running and which is waiting |
| `SHPR` (priorities) | `src/peripheral/scb.rs` | Done |
| `SHCSR` (fault enables), `CFSR`, `HFSR`, `BFAR` | `src/peripheral/scb.rs` | Done: the enables are honoured and the status registers are filled in |
| Faults and the state dump | `src/cpu/fault.rs` | Done; see "Faults" above |
| Check for a pending exception | `Cpu::step` | Done. Before fetching each instruction, in Thread mode or in a handler |
| Exception entry | `src/cpu/exception.rs` | Push the frame, set LR, load the PC |
| Exception return | `bx_write_pc` in `src/cpu/exception.rs` | Used by `BX` and `POP`; loads into the PC will need it too |
| MSP / PSP switching | `src/cpu/exception.rs`, `src/cpu/system.rs` | Done: entry, return, and writing `CONTROL`, `MSP` and `PSP` with `MSR` |
| A way for a peripheral to raise an IRQ | `update_irq_lines` in `src/bus.rs` | Done. The bus asks each peripheral whether it wants its interrupt and tells the NVIC |
| UART0 receive from stdin | `src/peripheral/uart.rs` | Done |

A peripheral can only answer reads and writes; it cannot call anyone to say that something happened. So the bus does the asking: it owns the exception state, checks each peripheral's interrupt request after every instruction, and the CPU asks the bus once per step whether anything is waiting. Two SCB registers act on that state, `ICSR` and `STIR`, so the bus answers those itself instead of passing them to the SCB stub.

New instructions are needed as well, as real firmware hits them. `BX`, `WFI`, `CPSID` / `CPSIE` and `MRS` / `MSR` exist; `SVC` is still to come.

`WFI` (wait for interrupt) is what firmware uses to idle. After it, the CPU is asleep: `Cpu::step` executes nothing until an exception is pending. Instead of counting through the wait one cycle at a time, it asks the bus how many cycles remain until the next exception will be raised and jumps emulated time forward by that much. If nothing is set up to raise one, the CPU stays asleep and time creeps on a cycle per step.

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
