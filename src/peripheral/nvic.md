### NVIC — Nested Vectored Interrupt Controller
*   **Base Address:** `0xE000_E100`
*   **Why It's There:** The part of the Arm core that keeps track of the peripheral interrupts (IRQs). A peripheral can ask for its interrupt at any time; the NVIC decides whether the handler may run. Firmware enables each IRQ it wants here, in addition to enabling the cause inside the peripheral.
*   **The Problem:** Without it no peripheral interrupt can be delivered: firmware that enables an IRQ and waits for its handler would wait forever.
*   **Logic:** For each IRQ there is an enable bit (`ISER` `+0x000` sets, `ICER` `+0x080` clears) and a pending state. An IRQ is pending while its peripheral is asking for it, or after firmware writes its bit in `ISPR` (`+0x100`) until the handler starts or the bit is written in `ICPR` (`+0x180`). The CPU is handed the lowest-numbered IRQ that is both pending and enabled, as exception number 16 + IRQ number.
*   **Not Implemented Yet:** Priorities (`IPR`, `+0x300`, one byte per IRQ) are stored with their 3 real bits but not used. `IABR` (`+0x200`, the "active" bits) always reads 0.
