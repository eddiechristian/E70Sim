use crate::peripheral::Peripheral;
use crate::pmc::PmcStub;
use crate::uart::Uart0Stub;

// Memory map (see the SVD and the SAME70 datasheet).
const BOOT_ALIAS_END: u32 = 0x001F_FFFF; // flash is also visible at 0x0
const FLASH_BASE: u32 = 0x0040_0000;
const FLASH_SIZE: u32 = 2 * 1024 * 1024; // 2 MB
const SRAM_BASE: u32 = 0x2040_0000;
const SRAM_SIZE: u32 = 384 * 1024; // 384 KB
const PMC_BASE: u32 = 0x400E_0600;
const PMC_SIZE: u32 = 0x200;
const UART0_BASE: u32 = 0x400E_0800;
const UART0_SIZE: u32 = 0x200;

const FLASH_END: u32 = FLASH_BASE + FLASH_SIZE - 1;
const SRAM_END: u32 = SRAM_BASE + SRAM_SIZE - 1;
const PMC_END: u32 = PMC_BASE + PMC_SIZE - 1;
const UART0_END: u32 = UART0_BASE + UART0_SIZE - 1;

/// Read `size` bytes (1, 2 or 4) at `offset` as a little-endian value. A
/// read that runs off the end of the region is logged and returns 0 instead
/// of panicking.
fn read_le(mem: &[u8], offset: usize, size: usize, addr: u32) -> u32 {
    match mem.get(offset..offset + size) {
        // Fold from the highest address down, so the lowest byte ends up
        // in the low bits.
        Some(bytes) => bytes.iter().rev().fold(0, |acc, &b| (acc << 8) | b as u32),
        None => {
            eprintln!(
                "WARNING: {}-byte read past end of region at Addr: {:#010X}",
                size, addr
            );
            0
        }
    }
}

/// Write the low `size` bytes (1, 2 or 4) of `value` at `offset`,
/// little-endian. A write that runs off the end of the region is logged and
/// ignored.
fn write_le(mem: &mut [u8], offset: usize, size: usize, value: u32, addr: u32) {
    match mem.get_mut(offset..offset + size) {
        Some(bytes) => bytes.copy_from_slice(&value.to_le_bytes()[..size]),
        None => eprintln!(
            "WARNING: {}-byte write past end of region at Addr: {:#010X} = {:#010X}",
            size, addr, value
        ),
    }
}

pub struct Bus {
    flash: Vec<u8>,
    sram: Vec<u8>,
    // Peripheral stubs, each implementing the Peripheral trait.
    pmc: PmcStub,
    uart0: Uart0Stub,
}

impl Bus {
    pub fn new() -> Self {
        Bus {
            flash: vec![0xFF; FLASH_SIZE as usize], // erased flash reads as 0xFF
            sram: vec![0; SRAM_SIZE as usize],
            pmc: PmcStub::new(),
            uart0: Uart0Stub,
        }
    }
    pub fn load_firmware_from_bin(&mut self, bin_filename: String) {
        let data = std::fs::read(bin_filename).expect("Failed to read firmware");
        self.load_firmware(&data);
    }

    /// Copy a raw firmware image to the start of flash.
    pub fn load_firmware(&mut self, data: &[u8]) {
        self.flash[..data.len()].copy_from_slice(data);
    }

    pub fn read8(&mut self, addr: u32) -> u8 {
        self.read(addr, 1) as u8
    }

    pub fn read16(&mut self, addr: u32) -> u16 {
        self.read(addr, 2) as u16
    }

    pub fn read32(&mut self, addr: u32) -> u32 {
        self.read(addr, 4)
    }

    pub fn write8(&mut self, addr: u32, value: u8) {
        self.write(addr, 1, value as u32);
    }

    pub fn write16(&mut self, addr: u32, value: u16) {
        self.write(addr, 2, value as u32);
    }

    pub fn write32(&mut self, addr: u32, value: u32) {
        self.write(addr, 4, value);
    }

    /// Read `size` bytes (1, 2 or 4) from whatever is mapped at `addr`.
    /// All of read8/16/32 go through here, so the memory map is in one place.
    fn read(&mut self, addr: u32, size: usize) -> u32 {
        match addr {
            // Boot alias: flash is also visible at address 0.
            0..=BOOT_ALIAS_END => read_le(&self.flash, addr as usize, size, addr),
            FLASH_BASE..=FLASH_END => {
                read_le(&self.flash, (addr - FLASH_BASE) as usize, size, addr)
            }
            SRAM_BASE..=SRAM_END => read_le(&self.sram, (addr - SRAM_BASE) as usize, size, addr),
            PMC_BASE..=PMC_END => self.pmc.read(addr - PMC_BASE, size),
            UART0_BASE..=UART0_END => self.uart0.read(addr - UART0_BASE, size),
            // Unmapped: log instead of panicking. The log shows which
            // peripheral the firmware wants next.
            _ => {
                eprintln!(
                    "WARNING: Unmapped {}-byte read at Addr: {:#010X}",
                    size, addr
                );
                0
            }
        }
    }

    /// Write the low `size` bytes (1, 2 or 4) of `value` to whatever is
    /// mapped at `addr`. All of write8/16/32 go through here.
    fn write(&mut self, addr: u32, size: usize, value: u32) {
        match addr {
            // Flash cannot be changed by a plain store; on the real chip it
            // is programmed through the EEFC peripheral.
            0..=BOOT_ALIAS_END | FLASH_BASE..=FLASH_END => {
                eprintln!(
                    "WARNING: Ignored {}-byte write to flash at Addr: {:#010X} = {:#010X}",
                    size, addr, value
                );
            }
            SRAM_BASE..=SRAM_END => write_le(
                &mut self.sram,
                (addr - SRAM_BASE) as usize,
                size,
                value,
                addr,
            ),
            PMC_BASE..=PMC_END => self.pmc.write(addr - PMC_BASE, size, value),
            UART0_BASE..=UART0_END => self.uart0.write(addr - UART0_BASE, size, value),
            _ => {
                eprintln!(
                    "WARNING: Unmapped {}-byte write at Addr: {:#010X} = {:#010X}",
                    size, addr, value
                );
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn sram_round_trips_every_size() {
        let mut bus = Bus::new();
        bus.write32(SRAM_BASE, 0x1234_5678);
        assert_eq!(bus.read32(SRAM_BASE), 0x1234_5678);
        bus.write16(SRAM_BASE + 8, 0xBEEF);
        assert_eq!(bus.read16(SRAM_BASE + 8), 0xBEEF);
        bus.write8(SRAM_BASE + 12, 0xA5);
        assert_eq!(bus.read8(SRAM_BASE + 12), 0xA5);
    }

    #[test]
    fn memory_is_little_endian() {
        let mut bus = Bus::new();
        bus.write32(SRAM_BASE, 0x1234_5678);
        assert_eq!(bus.read8(SRAM_BASE), 0x78);
        assert_eq!(bus.read8(SRAM_BASE + 3), 0x12);
        assert_eq!(bus.read16(SRAM_BASE), 0x5678);
        assert_eq!(bus.read16(SRAM_BASE + 2), 0x1234);
    }

    #[test]
    fn narrow_writes_leave_neighbours_alone() {
        let mut bus = Bus::new();
        bus.write32(SRAM_BASE, 0xFFFF_FFFF);
        bus.write8(SRAM_BASE + 1, 0x00);
        assert_eq!(bus.read32(SRAM_BASE), 0xFFFF_00FF);
        bus.write16(SRAM_BASE + 2, 0x1234);
        assert_eq!(bus.read32(SRAM_BASE), 0x1234_00FF);
    }

    #[test]
    fn last_word_of_sram_is_usable() {
        let mut bus = Bus::new();
        bus.write32(SRAM_END - 3, 0xCAFE_F00D);
        assert_eq!(bus.read32(SRAM_END - 3), 0xCAFE_F00D);
    }

    #[test]
    fn boot_alias_mirrors_flash() {
        let mut bus = Bus::new();
        bus.load_firmware(&[0x00, 0x00, 0x46, 0x20, 0x09, 0x00, 0x40, 0x00]);
        assert_eq!(bus.read32(0x0000_0000), 0x2046_0000);
        assert_eq!(bus.read32(FLASH_BASE), 0x2046_0000);
        assert_eq!(bus.read32(0x0000_0004), 0x0040_0009);
        assert_eq!(bus.read16(FLASH_BASE + 4), 0x0009);
        assert_eq!(bus.read8(FLASH_BASE + 3), 0x20);
    }

    #[test]
    fn erased_flash_reads_as_ff() {
        let mut bus = Bus::new();
        assert_eq!(bus.read32(FLASH_END - 3), 0xFFFF_FFFF);
    }

    #[test]
    fn writes_to_flash_are_ignored() {
        let mut bus = Bus::new();
        bus.write32(FLASH_BASE, 0);
        bus.write8(0x0000_0010, 0);
        assert_eq!(bus.read32(FLASH_BASE), 0xFFFF_FFFF);
        assert_eq!(bus.read8(0x0000_0010), 0xFF);
    }

    #[test]
    fn unmapped_access_returns_zero_without_panicking() {
        let mut bus = Bus::new();
        bus.write32(0x6000_0000, 0x1234);
        assert_eq!(bus.read32(0x6000_0000), 0);
        assert_eq!(bus.read16(0x6000_0000), 0);
        assert_eq!(bus.read8(0x6000_0000), 0);
    }

    #[test]
    fn access_straddling_the_end_of_a_region_does_not_panic() {
        let mut bus = Bus::new();
        bus.write32(SRAM_END - 1, 0xFFFF_FFFF);
        assert_eq!(bus.read32(SRAM_END - 1), 0);
        // The bytes that are inside the region were not partially written.
        assert_eq!(bus.read16(SRAM_END - 1), 0);
    }

    #[test]
    fn pmc_is_mapped_at_its_base_address() {
        let mut bus = Bus::new();
        // PMC_SR: MOSCXTS, LOCKA, MCKRDY and MOSCSELS all set.
        let sr = bus.read32(PMC_BASE + 0x68);
        assert_eq!(sr & 0x0001_000B, 0x0001_000B);
        // PMC_MCKR is store-and-readback.
        bus.write32(PMC_BASE + 0x30, 0x0000_0102);
        assert_eq!(bus.read32(PMC_BASE + 0x30), 0x0000_0102);
    }

    #[test]
    fn uart_status_reads_ready_at_every_size() {
        let mut bus = Bus::new();
        assert_eq!(bus.read32(UART0_BASE + 0x14), 0x202);
        assert_eq!(bus.read16(UART0_BASE + 0x14), 0x202);
        assert_eq!(bus.read8(UART0_BASE + 0x14), 0x02);
    }
}
