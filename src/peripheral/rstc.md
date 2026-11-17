### RSTC — Reset Controller
*   **Base Address:** `0x400E_1800`
*   **Why It's There:** Records why the chip was last reset, controls how the NRST pin behaves, and lets firmware reset the chip by itself.
*   **The Problem:** Startup code reads the Status Register (`RSTC_SR`, offset `+0x04`) to decide between a cold start and a restart after a watchdog or software reset, and writes the Mode Register (`RSTC_MR`, offset `+0x08`) to configure the NRST pin. On unmapped memory each access logs a warning and the readback is wrong.
*   **Stub Logic:** `RSTC_SR` always reports a power-up reset (`RSTTYP` = 0) with the NRST pin high (`NRSTL`, bit 16). `RSTC_MR` starts at `0x0000_0001` (`URSTEN`) and is store-and-readback, accepting writes only with the `0xA5` key in bits 31–24. A software reset written to the Control Register (`RSTC_CR`, offset `+0x00`, `PROCRST` bit 0) is logged but not carried out.
