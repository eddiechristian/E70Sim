### MATRIX — AHB Bus Matrix
*   **Base Address:** `0x4008_8000`
*   **Why It's There:** Arbitrates between the bus masters and slaves inside the chip, and holds a few chip configuration registers.
*   **The Problem:** Startup code writes the System I/O register (`CCFG_SYSIO`, offset `+0x114`) to release pins such as PB4–PB7 and PB12 from their debug and erase functions, usually by reading the register, setting bits and writing it back. On unmapped memory each access logs a warning and the readback is lost.
*   **Stub Logic:** Store-and-readback for every register; nothing has any effect. The Write Protection Mode Register (`MATRIX_WPMR`, offset `+0x1E4`) accepts `WPEN` (bit 0) only with key `0x4D4154` in bits 31–8, and protects nothing. The Write Protection Status Register (`MATRIX_WPSR`, offset `+0x1E8`) always reads 0.
