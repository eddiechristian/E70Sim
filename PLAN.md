# SAME70 Emulator Plan

Goal: emulate an ATSAME70Q21 (Cortex-M7, ARMv7-M) well enough to run real firmware, using UART0 output as the primary debug channel.

Strategy: get a CPU core running, reach UART output with the smallest possible instruction set, then grow the instruction set on demand by running real firmware and implementing whatever it hits next.

Reference docs (repo root):
- `ATSAME70Q21.svd` — peripheral base addresses, register offsets, bitfields, IRQ numbers.
- `DDI0403E_e_armv7m_arm.pdf` — ARMv7-M ARM. Key chapters: A2 (programmers' model), A5 (Thumb encoding tables), A6 (FP encoding), A7 (instruction details + pseudocode), B1 (exception model), B3 (system address map / SCS).

---

## Phase 0 — Skeleton

- [ ] Memory map
  - Flash `0x0040_0000` (2 MB), aliased at `0x0000_0000` for boot
  - SRAM `0x2040_0000` (384 KB)
  - Peripherals `0x4000_0000+`
  - System Control Space `0xE000_E000`
- [x] Bus with `read8/16/32` / `write8/16/32`; dispatch to a `Peripheral` trait (`read(offset, size)`, `write(offset, size, val)`) by address range
- [ ] Unmapped accesses: log `PC, addr, value`, return 0 — do not crash. This tells you which peripheral firmware wants next.
- [ ] Loader: raw `.bin` at `0x0040_0000` first; ELF later (`object` or `goblin` crate) for symbols
- [x] Reset: SP = `[0x0]`, PC = `[0x4] & !1`

## Phase 1 — CPU core

- [x] State: R0–R15, APSR (N Z C V), stub XPSR/PRIMASK/CONTROL
- [x] Fetch/decode: a halfword whose top 5 bits are `0b11101`, `0b11110` or `0b11111` starts a 32-bit instruction
- [x] Shared helpers from the ARM pseudocode, with unit tests:
  - `AddWithCarry`
  - `LSL_C`, `LSR_C`, `ASR_C`, `ROR_C`, `RRX_C`
  - `ThumbExpandImm_C`
  - `ConditionPassed`
- [x] Unknown instruction → panic with PC and raw encoding (this is the to-do list)
- [x] `--trace` flag: PC, decoded instruction, changed registers
- [x] Minimal instruction set (all 16-bit encodings):
  `LDR` (literal), `LDR` (imm), `LDRB` (imm), `STR` (imm), `ADDS` (imm), `CMP` (imm), `LSLS` (imm), `B` / `B<cond>`
- [x] Minimal UART0 stub: `UART_SR` always reports TXRDY; `UART_THR` writes go to stdout

### ✅ Milestone 1: hand-written assembly prints over UART

UART0 base `0x400E0800`: `UART_SR` = `+0x14` (TXRDY = bit 1), `UART_THR` = `+0x1C`.

```asm
    .syntax unified
    .thumb
    .section .vectors
    .word 0x20460000          @ initial SP = top of 384 KB SRAM
    .word reset               @ reset vector (.thumb_func sets bit 0)

    .text
    .thumb_func
reset:
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
```

Build: `arm-none-eabi-as` → `arm-none-eabi-ld -Ttext=0x400000` (vectors first) → `arm-none-eabi-objcopy -O binary`.

## Phase 2 — Peripheral stubs for real startup code

Startup code busy-waits on "ready" bits; stubs must report ready immediately.

| Peripheral | Base | Stub behaviour |
|---|---|---|
| UART0 | `0x400E0800` | ✅ SR: TXRDY + TXEMPTY (bits 1, 9) always set; THR → stdout; RHR (`+0x18`) ← stdin with RXRDY (bit 0); IER / IDR / IMR; raises IRQ 7 |
| PMC | `0x400E0600` | `PMC_SR` (`+0x68`): MOSCXTS, LOCKA, MCKRDY, MOSCSELS set; store other writes |
| EFC | `0x400E0C00` | ✅ `EEFC_FSR` (`+0x08`): FRDY (bit 0) set; `EEFC_FMR` (`+0x00`) store-and-readback; `EEFC_FCR` (`+0x04`) commands logged, not executed |
| WDT | `0x400E1850` | ✅ Never expires. `WDT_MR` (`+0x04`) resets to `0x3FFF2FFF` and is write-once, so the disable write (WDDIS, bit 15) reads back; `WDT_CR` (`+0x00`) restart is a no-op; `WDT_SR` (`+0x08`) reads 0 |
| SUPC | `0x400E1810` | ✅ Store-and-readback; `SUPC_CR` (`+0x00`) XTALSEL sets OSCSEL (bit 7) in `SUPC_SR` (`+0x14`) at once |
| RSTC | `0x400E1800` | ✅ `RSTC_SR` (`+0x04`) reports a power-up reset with NRST high; `RSTC_MR` (`+0x08`) store-and-readback; software reset logged, not carried out |
| PIOA–PIOE | `0x400E0E00` + `0x200` per port | ✅ Enable / disable / status register groups; outputs read back in `PIO_PDSR` (`+0x3C`), inputs read 1; no interrupts |
| MATRIX | `0x40088000` | ✅ Store-and-readback (including `CCFG_SYSIO`, `+0x114`) |
| SCB | `0xE000ED00` | ✅ Store-and-readback: VTOR, CPACR, CCR, SHPR, MPU; fixed CPUID and cache ID registers; cache maintenance writes ignored; exception and reset requests logged, not carried out |

All offsets and bit names in this table are verified against the SVD.

## Phase 3 — Compiled firmware, instruction set on demand

- [x] Write bare-metal firmware that inits UART and prints: Rust (`cortex-m-rt`, target `thumbv7em-none-eabi`) in `firmware/hello-rs/`, described in `firmware/firmware.md`
- [x] Build **soft-float** (no `hf`) so the FPU is not needed yet
- [ ] Loop: run → hit unimplemented instruction → implement → repeat. Expected set (~60–80 instructions):
  - Stack/moves: `PUSH/POP`, `MOV`, `MOVW/MOVT`, `ADD/SUB` (incl. SP-relative)
  - Logic/shifts: `AND/ORR/EOR/BIC/MVN`, `LSL/LSR/ASR/ROR`
  - Branches: `BL`, `BX`, `BLX`, `CBZ/CBNZ`, `IT` blocks, `TBB/TBH`
  - Loads/stores: `LDRB/STRB/LDRH/STRH`, `LDRD/STRD`, `LDM/STM`
  - Misc: `UXTB/SXTB`, `MUL/MLA`, `UDIV/SDIV`, `CLZ`
  - No-ops: `DSB/ISB/DMB/NOP`, `CPSID/CPSIE` (update PRIMASK)

### ✅ Milestone 2: compiled firmware prints over UART — `printf` debugging available from here

## Phase 4 — Exceptions and interrupts (ARM ARM B1, B3)

- [x] Exception entry: push 8-word frame; return via `EXC_RETURN` (`0xFFFFFFF1` / `9` / `D`)
- [x] MSP/PSP switching, PRIMASK, BASEPRI, FAULTMASK, CONTROL; changed with `CPSID` / `CPSIE` and `MRS` / `MSR`
- [x] SysTick (`0xE000E010`), counting emulated clock cycles
- [x] NVIC (`0xE000E100`): enable and pending bits
- [x] Exception priorities and nesting (priority grouping included)
- [x] HardFault with a useful dump of the faulting state: UsageFault, BusFault and MemManage with escalation and lockup; `--bus-faults` switch for unmapped accesses
- [x] UART0 RX interrupt (IRQ 7) fed from stdin

## Phase 5 — As needed

- [x] FPU (ARM ARM A6) for hard-float firmware: S/D registers, FPSCR, CPACR enable, extended exception frame with lazy saving, and the instructions `firmware/float-rs/` uses. FPv5 additions (`VSEL`, `VRINT*`, `VCVTA` ..., `VMAXNM`) are still to come
- [ ] GDB remote stub (`target remote`)
- [ ] TC timers, PIO interrupts, other peripherals
- [x] Cycle counting: every instruction reports an estimated cost in clock cycles (see `exceptionInterrupt.md`)
- [ ] `DWT_CYCCNT` register, reading the cycle count

## Phase 6 — I2C gyro add-on card (BNO055 Xplained Pro)

Emulates the ATBNO055-XPRO extension card: a Bosch BNO055 9-axis sensor on I2C. Two layers: the SAME70's I2C controller (TWIHS), and the sensor as a device attached to it. Needs Phase 3 (compiled firmware) to run a real driver; interrupt-driven drivers also need Phase 4.

References: `reference/bno055/BST-BNO055-DS000-18.pdf` (register map chapter 4, I2C protocol section 4.6); TWIHS registers and bitfields from the SVD.

- [ ] `I2cDevice` trait: `address()`, `write(reg, value)`, `read(reg) -> u8`
- [ ] TWIHS controller as a `Peripheral` (master mode, polled), holding a list of `I2cDevice`s

  | Instance | Base | IRQ |
  |---|---|---|
  | TWIHS0 | `0x40018000` | 19 |
  | TWIHS1 | `0x4001C000` | 20 |
  | TWIHS2 | `0x40060000` | 41 |

  - `TWIHS_CR` (`+0x00`): START (bit 0), STOP (bit 1), MSEN (bit 2), SWRST (bit 7)
  - `TWIHS_MMR` (`+0x04`): DADR (bits 22–16), MREAD (bit 12), IADRSZ (bits 9–8)
  - `TWIHS_IADR` (`+0x0C`): register number inside the device
  - `TWIHS_CWGR` (`+0x10`): store-and-readback (bus speed is irrelevant)
  - `TWIHS_SR` (`+0x20`): TXCOMP (bit 0), RXRDY (bit 1), TXRDY (bit 2), NACK (bit 8)
  - `TWIHS_RHR` (`+0x30`) / `TWIHS_THR` (`+0x34`): one byte in / out; register number auto-increments across a multi-byte transfer
  - No device at DADR → set NACK in `TWIHS_SR`
- [ ] BNO055 device model at I2C address `0x29` (card default; `0x28` if COM3 is low)
  - ID registers: `CHIP_ID` (`0x00`) = `0xA0`, plus `ACC_ID` (`0x01`), `MAG_ID` (`0x02`), `GYR_ID` (`0x03`)
  - `PAGE_ID` (`0x07`): selects register page 0 or 1
  - Store-and-readback config: `UNIT_SEL` (`0x3B`), `OPR_MODE` (`0x3D`, starts in CONFIGMODE), `PWR_MODE` (`0x3E`), `SYS_TRIGGER` (`0x3F`, reset bit restores defaults)
  - Status reads "all good": `CALIB_STAT` (`0x35`), `ST_RESULT` (`0x36`), `SYS_STATUS` (`0x39`)
  - Gyro data: `GYR_DATA_X_LSB` (`0x14`) through Z MSB (`0x19`), little-endian 16-bit, 16 LSB per degree/second
  - Later: accelerometer (`0x08`), Euler angles (`0x1A`), temperature (`0x34`)
- [ ] Motion source for the data registers: constants first, then a generated pattern or values from a file
- [ ] Hand-written assembly test: read `CHIP_ID` over TWIHS and print it over UART

To verify before or during implementation:
- Which TWIHS instance the EXT1/EXT2 header pins use on the SAME70 Xplained (card schematic / board user guide — not in the repo)
- TWIHS transfer sequencing (when TXRDY/RXRDY/TXCOMP set and clear) against the SAME70 datasheet — the SVD gives register layout only, not behaviour
- RGB LED wiring on the card (PIO pins)

### ✅ Milestone 3: firmware reads gyro values over I2C and prints them over UART

---

## Tips

- Differential testing: run random Thumb-2 encodings through the `unicorn-engine` crate and compare registers/flags. Catches most flag and carry bugs.
- The A7 pseudocode is the spec — translate it nearly line-for-line.
- Semihosting (`BKPT 0xAB` intercepted by the emulator) is an even simpler debug channel than UART if needed.
