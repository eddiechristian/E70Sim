#!/usr/bin/env bash
# Build firmware.bin from hello.s (needs the arm-none-eabi binutils).
set -euo pipefail
cd "$(dirname "$0")"

arm-none-eabi-as -mcpu=cortex-m7 hello.s -o hello.o
arm-none-eabi-ld -T link.ld hello.o -o hello.elf
arm-none-eabi-objcopy -O binary hello.elf firmware.bin

echo "Built $(pwd)/firmware.bin ($(stat -c %s firmware.bin) bytes)"
