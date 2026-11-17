### DWT — Data Watchpoint and Trace
*   **Base Address:** `0xE000_1000`
*   **Why It's There:** Debugging hardware in the Arm core. Firmware uses one part of it on its own: `DWT_CYCCNT` (`+0x004`), a 32-bit count of clock cycles, as a stopwatch.
*   **The Problem:** Timing code reads the counter before and after and subtracts. With the register unmapped both reads return 0, and delay loops built on it never end.
*   **Logic:** The counter runs while `TRCENA` (bit 24 of `DEMCR`, `0xE000_EDFC`, in the SCB's range) and `CYCCNTENA` (bit 0 of `DWT_CTRL`, `+0x000`) are both set. It advances by the emulator's estimated cost of each instruction and wraps round after 2^32 cycles. Firmware can write it, usually to 0. `DWT_CTRL` reports four comparators.
*   **Not Modelled:** Watchpoints and the other counters; their registers read 0. Writes to the lock register are accepted and ignored.
