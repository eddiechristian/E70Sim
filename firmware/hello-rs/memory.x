/* ATSAME70Q21 memory layout. Must match src/bus.rs in the emulator. */
MEMORY
{
    FLASH : ORIGIN = 0x00400000, LENGTH = 2M
    RAM   : ORIGIN = 0x20400000, LENGTH = 384K
}
