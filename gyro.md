# Simulating the gyro (Phase 6)

This is the plan for Phase 6 of `PLAN.md`: make firmware running in the emulator able to read a gyro sensor, as it would on a SAM E70 Xplained Ultra board with the BNO055 Xplained Pro card plugged in.

Sources: the sensor's datasheet in `reference/bno055/BST-BNO055-DS000-18.pdf` (register map in chapter 4, I2C in section 4.6); `ATSAME70Q21.svd` for the register layout of the SAME70's I2C controller; and the SAM E70 datasheet in `reference/same70/` for how that controller behaves (chapter 43, with host mode in section 43.7.3). Values below are from those two unless marked "from memory".

## What is being simulated

A gyro measures how fast something is turning, in degrees per second, around three axes (X, Y, Z). The BNO055 is a chip that contains a gyro, an accelerometer and a magnetometer, plus a small processor that can combine the three into an orientation.

The firmware never touches the sensor directly. It talks to it over **I2C**, a two-wire bus (clock and data) shared by one controller and any number of devices, each with a 7-bit address. So there are two things to build:

| Layer | What it is | Where it lives |
|---|---|---|
| TWIHS | The SAME70's I2C controller: a peripheral with registers, like UART0 | New `Peripheral`, mapped by the bus |
| BNO055 | The sensor: a device on the I2C bus with its own table of registers | New "I2C device", attached to the TWIHS |

A read of the gyro's X rate then goes: firmware writes TWIHS registers → TWIHS addresses device `0x29` → BNO055 model returns the bytes of its `GYR_DATA_X` register → firmware reads them from the TWIHS.

## Prerequisites: what is in place

Almost everything outside the two new pieces already exists.

| Needed for | Requirement | Status |
|---|---|---|
| Mapping the controller | `Peripheral` trait and address dispatch in the bus | Done |
| Turning the controller on | PMC peripheral clock enables (`PMC_PCER0` / `PMC_PCER1`) | Done; stored, not enforced |
| Routing the two I2C pins | PIO pin selection (`PIO_PDR`, `PIO_ABCDSR`) | Done; stored |
| Interrupt-driven drivers | NVIC, priorities, nesting, `PRIMASK`, exception entry and return | Done |
| The delays the sensor needs | SysTick, TC timers, `WFI` with time skipping | Done |
| Timing transfers | Emulated cycle count on the bus | Done |
| Printing results | UART0 output | Done |
| Scaling raw values to degrees per second | FPU, or soft-float | Done |
| The sensor's "data ready" interrupt line | PIO pin-change interrupts | Done |
| Catching driver bugs | Faults with a dump, `--bus-faults` | Done |
| Knowing the sensor | BNO055 datasheet in the repo | Done |
| Knowing the controller's registers | TWIHS in the SVD | Done |
| Knowing how the controller behaves | SAM E70 datasheet, chapter 43, in `reference/same70/` | Done |
| Knowing which controller and pins the card uses | SAM E70 Xplained Ultra user guide in `reference/same70/` | Done: TWIHS0 on PA3 / PA4 |

## Board wiring

The board is the **SAM E70 Xplained Ultra**. Its user guide is `reference/same70/SAM-E70-Xplained-Ultra-User-Guide-70005389A.pdf`.

The guide's text says only that the "system I2C" is I2C0, shared by the debugger, EXT1, EXT2 and the mikroBUS socket (section 2). The pin-by-pin wiring is in the schematic on page 33:

| EXT1 pin | Role | SAM E70 pin | Function |
|---|---|---|---|
| 11 | I2C data | PA3 | TWD0, so TWIHS0 |
| 12 | I2C clock | PA4 | TWCK0 |
| 9 | Interrupt / GPIO | PD28 | WKUP5. A candidate for the sensor's INT line |
| 5, 6 | GPIO | PB3, PB2 | Candidates for the sensor's reset line |
| 10 | SPI select / GPIO | PC17 | Another candidate |

So the firmware uses **TWIHS0** (base `0x4001_8000`, IRQ 19) on PA3 and PA4, peripheral function A.

Two more things the guide shows:

- **The sensor is not alone on that bus.** The same two wires go to EXT2, the mikroBUS socket, the debugger, and an EEPROM soldered on the board: an AT24MAC402 holding a unique serial number and MAC address (section 5.13). Its address pins are all tied high by default; by that chip's addressing scheme this should put it at `0x57`, but that number is from memory, not from the guide. The emulator does not need to model it, but it is a ready-made second device for testing that the controller picks the right one by address.
- **The board's debug serial port is probably not UART0.** The USB debugger provides a virtual COM port "connected to a UART on the ATSAME70Q21" (section 1), but the text does not say which. From memory it is USART1 (PB4 transmit, PA21 receive). The firmware in this repo prints on UART0, which is fine in the emulator; on the real board its output would then not appear on the USB serial port.

## Prerequisites: what is missing

None of these blocks a start. The first should be dealt with during the work.

| Gap | Why it matters | What to do |
|---|---|---|
| **Divide and multiply instructions** (`UDIV`, `SDIV`, 32-bit `MUL`, `UMULL`) | Printing a number in decimal needs division, and a gyro reading is only readable in decimal. The emulator has no divide yet, and one existing test asserts that `UDIV` is unhandled | Add them when the firmware hits them, as usual. Small |
| **A device cannot drive a pin** | The sensor has an interrupt output (INT) and a reset input (nRESET). Today only the `--pin=` option changes an input pin, and nothing is told when firmware changes an output pin | Only needed for step 6 below. Give the bus a way to connect a device's output to a PIO input |
| **The assembly test in `PLAN.md` cannot be built here** | `arm-none-eabi` binutils are not installed, so no new `.s` file can be assembled | Write the test firmware in Rust instead |
| **Other compiled-code instructions** | A real driver is more code than `hello-rs`, and will hit instructions not implemented yet (`TBB`, bit-field operations, and so on) | On demand |

## Design

### The I2C device interface

`PLAN.md` sketches `write(reg, value)` and `read(reg)`. I suggest one level lower, bytes on the bus, because that is what I2C carries and it keeps "which register" inside the device where it belongs:

```rust
pub trait I2cDevice {
    /// The device's 7-bit address.
    fn address(&self) -> u8;
    /// A transfer begins (the START condition).
    fn start(&mut self);
    /// The controller sends a byte. Returns false to refuse it (NACK).
    fn write(&mut self, byte: u8) -> bool;
    /// The controller asks for a byte.
    fn read(&mut self) -> u8;
    /// Let emulated time pass, for devices whose behaviour depends on it.
    fn tick(&mut self, cycles: u32) {}
}
```

For the BNO055, as for most I2C sensors, the first byte written after a start is the register number, any further bytes written go into that register and the ones after it, and bytes read come from that register onwards. The register number moves up by one after each byte.

### The TWIHS controller

Master mode only; that is the role the SAME70 plays towards a sensor.

| Instance | Base | IRQ |
|---|---|---|
| TWIHS0 | `0x4001_8000` | 19 |
| TWIHS1 | `0x4001_C000` | 20 |
| TWIHS2 | `0x4006_0000` | 41 |

| Register | Offset | Behaviour in the model |
|---|---|---|
| `TWIHS_CR` | `+0x00` | START (bit 0), STOP (bit 1), MSEN (bit 2), MSDIS (bit 3), SWRST (bit 7) |
| `TWIHS_MMR` | `+0x04` | DADR (bits 22–16): device address. MREAD (bit 12): direction. IADRSZ (bits 9–8): how many "internal address" bytes to send first |
| `TWIHS_IADR` | `+0x0C` | The internal address: for a sensor, the register number |
| `TWIHS_CWGR` | `+0x10` | Bus speed. Stored; transfers take no emulated time at first |
| `TWIHS_SR` | `+0x20` | TXCOMP (bit 0), RXRDY (bit 1), TXRDY (bit 2), NACK (bit 8) |
| `TWIHS_IER` / `IDR` / `IMR` | `+0x24` / `+0x28` / `+0x2C` | Interrupt mask, as on UART0 |
| `TWIHS_RHR` | `+0x30` | Next received byte; reading it fetches the following one |
| `TWIHS_THR` | `+0x34` | Byte to send |

Two ways firmware reads a sensor register, and the model must handle both:

- **Internal address**: put the register number in `TWIHS_IADR`, set IADRSZ to 1 and MREAD, write START. The controller sends the register number and then reads.
- **Write then read**: send the register number as a data byte through `TWIHS_THR`, then start a read without a STOP in between (a "repeated start").

If no device has the address in DADR, the controller sets NACK and TXCOMP. That is also how firmware scans the bus for devices.

### The BNO055 model

At I2C address `0x29`, the card's default (`0x28` if the COM3 pin is low).

| Group | Registers | Behaviour |
|---|---|---|
| Identity | `CHIP_ID` `0x00` = `0xA0`, `ACC_ID` `0x01` = `0xFB`, `MAG_ID` `0x02` = `0x32`, `GYR_ID` `0x03` = `0x0F` | Fixed. A driver checks `CHIP_ID` first |
| Page select | `PAGE_ID` `0x07` | Chooses between two register pages. Page 1 holds sensor configuration |
| Mode | `OPR_MODE` `0x3D` (reset `0x10`: low four bits 0 = CONFIGMODE), `PWR_MODE` `0x3E`, `UNIT_SEL` `0x3B` (reset `0x80`) | Stored. Data registers only change outside CONFIGMODE |
| Reset | `SYS_TRIGGER` `0x3F` | Bit 5 (RST_SYS) restores every register |
| Status | `CALIB_STAT` `0x35`, `ST_RESULT` `0x36` (`0x0F`: all self-tests passed), `SYS_STATUS` `0x39`, `SYS_ERR` `0x3A` | Report "all good" |
| Gyro data | `GYR_DATA_X_LSB` `0x14` to `GYR_DATA_Z_MSB` `0x19` | Three little-endian signed 16-bit values. 16 units per degree per second, or 900 per radian per second if `UNIT_SEL` says so |
| Later | Accelerometer `0x08`, Euler angles `0x1A`, temperature `0x34` | Not at first |

Operating modes that produce gyro data include GYROONLY (`0011`), AMG (`0111`), IMU (`1000`) and NDOF (`1100`).

**Timing.** The real chip is not ready immediately: 650 ms from reset to CONFIGMODE, 7 ms to leave CONFIGMODE and 19 ms to return to it. A driver that waits the right time works whether or not the model enforces this. I suggest the model enforces it (the chip does not answer during those times) but behind a switch, so that it can catch drivers that do not wait.

### The motion source

Where the numbers come from, in order of effort:

1. **Constants**, for example X = 1.0, Y = −2.0, Z = 0.5 degrees per second. Enough to prove the whole path.
2. **A generated pattern**, such as a slow sine wave on each axis, driven by emulated time. Shows that successive readings differ and are read consistently.
3. **A file** of timestamped samples, given on the command line. Lets a recorded motion be replayed.

## Steps

Each step can be tested before the next begins.

| # | Step | Test |
|---|---|---|
| 1 | `I2cDevice` trait and a trivial test device (a few bytes of memory) | Unit tests |
| 2 | TWIHS, polled, both ways of reading a register, NACK for an absent address. Attach to the bus at all three bases | Unit tests driving the registers as firmware would |
| 3 | BNO055 with identity, page, mode, reset and status registers, and constant gyro data | Unit tests against datasheet values |
| 4 | Rust firmware `firmware/gyro-rs/`: set up TWIHS0, read `CHIP_ID`, print it | Prints `A0`. This is the `PLAN.md` "read CHIP_ID and print it" item |
| 5 | Firmware: switch to a gyro mode, read the three axes in a loop, print them. Add the divide instructions if not already there | Printed values match the motion source. **Milestone 3** |
| 6 | TWIHS interrupts (IRQ 19), and optionally the sensor's INT pin through a PIO interrupt | An interrupt-driven read gives the same values |
| 7 | Motion pattern, then motion from a file | Readings follow the pattern over time |
| 8 | Sensor timing behind a switch; transfer time taken from `TWIHS_CWGR` | A driver without the delays fails with the switch on |

Steps 1 to 5 are the core. Steps 6 to 8 are refinements and can be done in any order, or not at all.

## Verification

There is no real board to compare against, so correctness rests on:

- **The datasheet**: reset values, identity values, data format and units are checked in unit tests against the PDF.
- **Both ends agreeing**: the firmware is written from the datasheets, the models from the same datasheets. A misreading of the datasheet would be made twice, so this proves less than it seems.
- **An existing driver**, if one is wanted: running a published BNO055 driver unchanged would be a far stronger test than firmware written alongside the model. For Rust that means a HAL for the SAME70's I2C underneath it, which is a larger piece of work.

## Left out on purpose

- **The sensor's fusion output** (orientation as Euler angles or a quaternion). It would need a model of the fusion algorithm, or values supplied directly.
- **Calibration.** `CALIB_STAT` simply reports "calibrated".
- **I2C slave mode, high-speed mode, SMBus and the FIFO** on the TWIHS.
- **Bus-level faults**: clock stretching, arbitration loss, a stuck bus.
- **The card's RGB LED.**

## To verify

- Which EXT header pins the card uses for the sensor's INT and nRESET lines. That is in the BNO055 Xplained Pro card's own user guide, which is not in the repo. The board side is known: see "Board wiring" above.
