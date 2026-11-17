SECTIONS
{
    . = 0x00400000;
    .text : { KEEP(*(.vectors)) *(.text*) *(.rodata*) }
}
