### SUPC — Supply Controller
*   **Base Address:** `0x400E_1810`
*   **Why It's There:** Looks after the chip's power: the voltage regulator, brownout detector, supply monitor, wake-up from backup mode, and the choice of 32 kHz slow clock source.
*   **The Problem:** Firmware that wants the accurate 32 kHz crystal writes `XTALSEL` (bit 3) to the Control Register (`SUPC_CR`, offset `+0x00`) and then loops on the Status Register (`SUPC_SR`, offset `+0x14`) until `OSCSEL` (bit 7) is set. If it stays zero, execution hangs. Other code writes the Mode Register (`SUPC_MR`, offset `+0x08`) and expects to read its settings back.
*   **Stub Logic:** A `SUPC_CR` write with `XTALSEL` and the `0xA5` key in bits 31–24 sets `OSCSEL` in `SUPC_SR` at once. All other status bits read 0 (no brownout, no wake-up event). `SUPC_SMMR`, `SUPC_MR` (key `0xA5` required), `SUPC_WUMR` and `SUPC_WUIR` are store-and-readback. Backup mode (`VROFF`) is logged but not carried out.
