#!/usr/bin/env bash
# Build the Rust firmware and turn it into the raw image the emulator loads.
#
# One-time set-up:
#   rustup target add thumbv7em-none-eabi
#   rustup component add llvm-tools
set -euo pipefail
cd "$(dirname "$0")"

cargo build --release

ELF=target/thumbv7em-none-eabi/release/gyro-rs
BIN=../gyro-rs.bin

# llvm-objcopy comes with the llvm-tools component, inside the toolchain.
OBJCOPY=$(find "$(rustc --print sysroot)" -name llvm-objcopy | head -n 1)
if [ -z "$OBJCOPY" ]; then
    echo "llvm-objcopy not found: run 'rustup component add llvm-tools'" >&2
    exit 1
fi

# The ELF holds the program plus symbols; the emulator wants only the bytes
# that go into flash.
"$OBJCOPY" -O binary "$ELF" "$BIN"

echo "Built $(realpath "$BIN") ($(stat -c %s "$BIN") bytes)"
