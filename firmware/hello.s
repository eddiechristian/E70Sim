    .syntax unified
    .thumb
    .section .vectors
    .word 0x20460000          @ initial SP = top of 384 KB SRAM
    .word reset               @ reset vector (.thumb_func sets bit 0)

    .text
    .thumb_func
reset:
    ldr   r4, =0x400E0600     @ PMC base
wait_mck:
    ldr   r5, [r4, #0x68]     @ PMC_SR
    lsls  r5, r5, #28         @ move MCKRDY (bit 3) into the N flag
    bpl   wait_mck
    ldr   r0, =0x400E0800     @ UART0 base
    ldr   r1, =msg
next:
    ldrb  r2, [r1]
    adds  r1, r1, #1
    cmp   r2, #0
    beq   done
wait:
    ldr   r3, [r0, #0x14]     @ UART_SR
    lsls  r3, r3, #30         @ move TXRDY (bit 1) into the N flag
    bpl   wait
    str   r2, [r0, #0x1C]     @ UART_THR
    b     next
done:
    b     done
msg: .asciz "Hello from SAME70\n"
