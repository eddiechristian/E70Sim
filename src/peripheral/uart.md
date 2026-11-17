### UART0 — Universal Asynchronous Receiver Transmitter
*   **Base Address:** `0x400E_0800`
*   **Why It's There:** Acts as the primary external communication channel. Firmware utilizes it as a stdout pipe to print boot diagnostic telemetry, panic info, and user tracking lines.
*   **The Problem:** Firmware checks the Status Register (`UART_SR`, offset `+0x14`) to ensure the peripheral is idle and has room to buffer data. If it reads a busy flag or empty buffers aren't signaled, the software spins indefinitely inside a transmission blocking loop.
*   **Stub Logic:** Hardcode reads to `UART_SR` (`+0x14`) to always report `TXRDY` (bit 1) and `TXEMPTY` (bit 9) as set (`1`). Intercept any writes targeting the Transmit Holding Register (`UART_THR`, offset `+0x1C`), strip out the bottom byte, convert it to a host character, and write it out directly to your host platform's standard console output stream (`stdout`).
