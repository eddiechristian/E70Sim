# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project status

Early-stage Rust project (edition 2024, no dependencies yet) intended to become an emulator for the Microchip **ATSAME70Q21** microcontroller (Arm Cortex-M7, ARMv7-M architecture). Currently `src/main.rs` is only the `cargo new` hello-world stub — there is no architecture yet. Update this file as the design takes shape.

## Commands

- Build: `cargo build`
- Run: `cargo run`
- Test: `cargo test`; single test: `cargo test <test_name>`
- Lint/format: `cargo clippy`, `cargo fmt`

## Reference material in the repo root

These are large files — search them (grep, or read the PDF with page ranges) rather than reading them whole.

- `ATSAME70Q21.svd` (~2.3 MB, CMSIS-SVD XML, Microchip): authoritative source for the memory map and all ~62 peripherals — base addresses, register offsets, reset values, bitfields, and interrupt numbers. CPU section: CM7 r1p1, little-endian, MPU and FPU present, 3 NVIC priority bits.
- `DDI0403E_e_armv7m_arm.pdf` (~6 MB): Arm ARMv7-M Architecture Reference Manual — instruction encodings/semantics (Thumb/Thumb-2), exception model, system control space (NVIC, SysTick, SCB), and memory model.
- `reference/bno055/BST-BNO055-DS000-18.pdf` (~2 MB, 117 pages): Bosch BNO055 9-axis sensor datasheet (rev 1.8), for a possible I2C gyro model. Register map is chapter 4; I2C protocol is section 4.6. The BNO055 Xplained Pro card uses I2C address `0x29`.

## Roadmap

`PLAN.md` holds the phased build plan (CPU core → UART output → peripheral stubs → on-demand instruction set → exceptions). Check it before starting new work and tick off items as they are completed.
