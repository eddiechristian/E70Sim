# SAME70 Emulator Technical Reference & Phase 2 Guide

This reference manual documents the architectural maps, reset behaviors, milestone instruction breakdowns, and comprehensive peripheral stub behaviors required to emulate an ATSAME70Q21 (Cortex-M7, ARMv7-M) microcontroller.

---

## 1. System Memory Map & Vector Rules

The ATSAME70Q21 utilizes a 4 GB (32-bit) address space. To minimize memory consumption on the host machine and accurately capture hardware mechanics, memory must be implemented as an address router or Bus rather than a single flat slice of data bytes.

### Memory Regions
*   **Boot Alias (`0x0000_0000` to `0x001F_FFFF`):** Mirrors the Flash region. At boot time, the CPU automatically reads addresses `0x0000_0000` and `0x0000_0004` to fetch the initial Stack Pointer (SP) and Program Counter (PC).
*   **Flash (`0x0040_0000` to `0x005F_FFFF`):** 2 MB of physical non-volatile storage holding the compiled binary machine instructions and static read-only data.
*   **SRAM (`0x2040_0000` to `0x2045_FFFF`):** 384 KB of volatile general-purpose RAM used for execution stacks, heaps, and global variables.
*   **Peripherals (`0x4000_0000` onwards):** Memory-mapped I/O blocks where read/write operations execute logic and mutate peripheral state instead of acting as passive storage cells.
*   **System Control Space (`0xE000_E000` onwards):** Internal ARM-native peripherals such as the System Control Block (SCB), SysTick timer, and the Nested Vectored Interrupt Controller (NVIC).

### Hardware Reset Sequence
Upon triggering a system reset, the Cortex-M7 core hardware strictly executes the following micro-architectural rules:
1.  **Initialize SP (`SP = [0x0000_0000]`):** The core reads the 4-byte word residing at address `0x0000_0000` and forces it directly into the Stack Pointer (SP / R13). This points to the top of volatile SRAM (`0x2046_0000` for a 384 KB configuration).
2.  **Initialize PC (`PC = [0x0000_0004] & !1`):** The core reads the next 4-byte word at address `0x0000_0004` to locate the Reset Vector. The hardware forces bit 0 of this pointer to `0` via a bitwise `AND NOT 1` mask to guarantee strict halfword or word memory alignment for instruction fetching.
3.  **Set Thumb Execution Mode (`xpsr.T = 1`):** Compilers intentionally set bit 0 of the reset vector address inside the vector table to `1` to act as an execution mode flag. While the physical `PC` skips this bit during the mask operation, the hardware interprets the incoming raw `1` to flip the Thumb state bit (bit 24) of the Execution Program Status Register (EPSR) to `1`. Cortex-M7 cores do not support traditional ARM 32-bit execution mode and will immediately hard fault if this bit is zero.

---

## 2. Phase 1 Milestone Instructions

To pass Milestone 1 and verify the loop structure using manual assembly, the decoder only requires implementations for the following 9 distinct instructions:

1.  **`LDR` (literal / PC-relative):** Used to load large 32-bit constant values (like peripheral base addresses) out of a local literal pool placed by the compiler at the end of the text segment.
2.  **`LDRB` (immediate / register-offset):** Fetches a single byte from a string array pointed to by a source register and writes it into a destination target register.
3.  **`ADDS` (immediate):** Adds an immediate value to a register, writes the output back to the destination register, and updates the processor's APSR status flags (`N`, `Z`, `C`, `V`).
4.  **`CMP` (immediate):** Subtracts an immediate value from a register value to evaluate data equality without modifying the register contents, intentionally updating status flags.
5.  **`BEQ` (conditional branch):** Inspects the Zero flag (`Z`). If `Z == 1` (indicating equality from a previous `CMP`), it branches to a designated target offset.
6.  **`LDR` (immediate / register-offset):** Reads a full 32-bit word out of a memory-mapped peripheral register utilizing a base register plus an explicit offset.
7.  **`LSLS` (immediate):** Shifts bits within a register leftward by an immediate value. Shifting a single status bit into the most significant slot (bit 31) allows quick routing of hardware flags straight into the **Negative (`N`)** flag.
8.  **`BPL` (conditional branch):** Inspects the Negative flag (`N`). If `N == 0` (indicating a positive or zero outcome), it branches to the designated target loop offset.
9.  **`STR` (immediate / register-offset):** Writes a 32-bit register value or sub-byte component out to a specific memory-mapped peripheral address (e.g., transmitting data bytes out of the UART Transmit Holding Register).
10. **`B` (unconditional branch):** Forces execution flow directly to a designated instruction offset unconditionally, forming main loops or blocking terminal traps.

---

## 3. Phase 2 Peripheral Compliance Stubs

When executing real, compiled C or Rust firmware frameworks rather than hand-crafted assembly, the firmware initializes physical microchip properties before starting processing loops. Stubs behave as compliance layers; they do not require deep emulation engines, but must provide predictable, hardcoded values to satisfy boot checking loops.

### UART0 — Universal Asynchronous Receiver Transmitter
*   **Base Address:** `0x400E_0800`
*   **Why It's There:** Acts as the primary external communication channel. Firmware utilizes it as a stdout pipe to print boot diagnostic telemetry, panic info, and user tracking lines.
*   **The Problem:** Firmware checks the Status Register (`UART_SR`, offset `+0x14`) to ensure the peripheral is idle and has room to buffer data. If it reads a busy flag or empty buffers aren't signaled, the software spins indefinitely inside a transmission blocking loop.
*   **Stub Logic:** Hardcode reads to `UART_SR` (`+0x14`) to always report `TXRDY` (bit 1) and `TXEMPTY` (bit 9) as set (`1`). Intercept any writes targeting the Transmit Holding Register (`UART_THR`, offset `+0x1C`), strip out the bottom byte, convert it to a host character, and write it out directly to your host platform's standard console output stream (`stdout`).

### PMC — Power Management Controller
*   **Base Address:** `0x400E_0600`
*   **Why It's There:** Manages the system distribution grids, clock prescalers, and phase-locked loop (PLL) multipliers across the chip core.
*   **The Problem:** At startup, the microchip operates on an extremely slow internal oscillator. The firmware shifts configurations to route execution through a fast external crystal oscillator. Real crystal tanks take milliseconds to safely settle and lock phase. The incoming firmware loops continuously over the Status Register (`PMC_SR`, offset `+0x68`) waiting for stabilizing bits to flip high. If they remain zero, execution freezes instantly.
*   **Stub Logic:** Intercept reads directed at `PMC_SR` (`+0x68`) and hardcode returning these key status bits as high: `MOSCXTS` (Main Oscillator Stabilization), `LOCKA` (PLL A Lock), `MCKRDY` (Master Clock Ready), and `MOSCSELS` (Main Clock Source Selection). For configuration registers, implement basic write-storage and readback so the firmware can verify its modifications were successfully written to memory.

### EFC — Embedded Flash Controller
*   **Base Address:** `0x400E_0C00`
*   **Why It's There:** Configures timing parameters, controller commands, and read wait-states for physical access to the flash array.
*   **The Problem:** Because the PMC clock speed just increased significantly, the flash memory layout requires explicit wait states inserted so the physical array can keep up with the fast CPU core. The firmware writes these tuning values to flash control paths and immediately blocks on the Flash Status Register (`EEFC_FSR`, offset `+0x08`), waiting for the controller to report a ready state.
*   **Stub Logic:** Hardcode the Flash Ready bit (`FRDY`, bit 0) inside `EEFC_FSR` (`+0x08`) to always evaluate as `1`. The software assumes the tuning configuration parameters took effect instantaneously and immediately advances past the check loop.

### WDT — Watchdog Timer
*   **Base Address:** `0x400E_1850`
*   **Why It's There:** Functions as a fail-safe supervisor timer designed to automatically force a hardware recovery reset if the firmware locks up or hangs.
*   **The Problem:** On the ATSAME70Q21 chip, the watchdog timer is active by default at power-on. Since software emulators run significantly slower than native bare-metal hardware silicon, the watchdog timer would theoretically expire and continually trip a hard reset before your emulator can complete basic initialization. Standard startup code writes a specialized sequence to disable the watchdog early in the boot cycle.
*   **Stub Logic:** Accept the explicit watchdog configuration and disable commands sent to the watchdog control registers. Treat them as quiet success events (no-ops) without executing real timer mechanics.

### SUPC / RSTC — Supply Controller & Reset Controller
*   **Base Addresses:** `0x400E_1810` (SUPC) / `0x400E_1800` (RSTC)
*   **Why They're There:** Monitor internal voltage monitors, brown-out detectors, and track the explicit sources of hardware reset triggers.
*   **The Problem:** Vendor runtime environments often inspect reset history or voltage levels during boot to determine whether the chip is coming out of a deep sleep cycle, a brownout event, or a standard cold power-on reset.
*   **Stub Logic:** Implement standard **Store-and-Readback**. Allocate a simple backing register variable map to hold values written by the software. If the firmware performs a confirmation validation read over these paths, hand back the exact data it previously committed.

### PIOx / MATRIX — Parallel I/O Controllers & Bus Matrix
*   **Base Addresses:** `0x400E_0E00` onwards (PIO blocks) / `0x4008_8000` (MATRIX)
*   **Why They're There:** PIOx manages physical pin routing, logic directions, and pull-up/pull-down resistors. MATRIX arbitrates access priorities between the high-speed bus masters (like the Cortex-M7 core and DMA channels).
*   **The Problem:** Boot code configures peripheral pin routing maps (e.g., assigning specific pins to UART0 mode) and sets up memory access priorites across the internal matrix lines. Real pin toggling or arbitration priorities do not impact a basic instruction emulator, but the configuration values must be persistent.
*   **Stub Logic:** Implement standard **Store-and-Readback**. You do not need to track pin logic levels or bus bandwidth limits. Simply capture any setup register writes inside basic variables and echo those identical configurations back to the guest software upon demand.

### SCB — System Control Block (Internal ARM Peripheral)
*   **Base Address:** `0xE000_ED00`
*   **Why It's There:** An internal architectural configuration space provided natively by the ARM Cortex-M7 core design to handle execution rules, vector placements, and system exceptions.
*   **The Problem:** Standard runtime systems query and modify these addresses early to configure the Vector Table Offset Register (`VTOR`), activate the Floating Point Unit via the Coprocessor Access Control Register (`CPACR`), or execute structural instruction and data cache clearing procedures.
*   **Stub Logic:** Implement **Store-and-Readback** for structural tracking registers like `VTOR` and `CPACR`. For cache control operations (such as clearing or invalidating I-Cache or D-Cache), intercept the requests and treat them as successful operations without implementing complex cache layer mechanics.