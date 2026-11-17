use std::sync::mpsc::Receiver;

use crate::i2c::I2cDevice;
use crate::peripheral::Peripheral;
use crate::peripheral::dwt::Dwt;
use crate::peripheral::efc::EfcStub;
use crate::peripheral::matrix::MatrixStub;
use crate::peripheral::nvic::Nvic;
use crate::peripheral::pio::PioStub;
use crate::peripheral::pmc::PmcStub;
use crate::peripheral::rstc::RstcStub;
use crate::peripheral::scb::ScbStub;
use crate::peripheral::supc::SupcStub;
use crate::peripheral::systick::SysTick;
use crate::peripheral::tc::Tc;
use crate::peripheral::twihs::Twihs;
use crate::peripheral::uart::Uart0;
use crate::peripheral::wdt::WdtStub;

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
const EFC_BASE: u32 = 0x400E_0C00;
const EFC_SIZE: u32 = 0x200;
// Four Timer Counter modules, TC0 to TC3, of three channels each.
const TC_BASES: [u32; 4] = [0x4000_C000, 0x4001_0000, 0x4001_4000, 0x4005_4000];
const TC_SIZE: u32 = 0x100;
// Three I2C controllers, TWIHS0 to TWIHS2.
const TWIHS_BASES: [u32; 3] = [0x4001_8000, 0x4001_C000, 0x4006_0000];
const TWIHS_SIZE: u32 = 0x100;
const MATRIX_BASE: u32 = 0x4008_8000;
const MATRIX_SIZE: u32 = 0x200;
// Five PIO ports, PIOA to PIOE, one after another.
const PIO_BASE: u32 = 0x400E_0E00;
const PIO_SIZE: u32 = 0x200; // per port
const PIO_COUNT: u32 = 5;
// RSTC, SUPC, RTT, WDT and RTC share one 0x200 block, so each gets only
// its own few bytes (the RTT starts at 0x400E_1830, the RTC at 0x400E_1860).
const RSTC_BASE: u32 = 0x400E_1800;
const RSTC_SIZE: u32 = 0x10;
const SUPC_BASE: u32 = 0x400E_1810;
const SUPC_SIZE: u32 = 0x20;
const WDT_BASE: u32 = 0x400E_1850;
const WDT_SIZE: u32 = 0x10;
// The DWT, debugging hardware that includes the cycle counter.
const DWT_BASE: u32 = 0xE000_1000;
const DWT_SIZE: u32 = 0x1000;
const DWT_END: u32 = DWT_BASE + DWT_SIZE - 1;
// System Control Space: SysTick, the NVIC, then the System Control Block
// and everything above it.
const SYSTICK_BASE: u32 = 0xE000_E010;
const SYSTICK_SIZE: u32 = 0x10;
const NVIC_BASE: u32 = 0xE000_E100;
const NVIC_SIZE: u32 = 0x400;
const SCB_BASE: u32 = 0xE000_ED00;
const SCB_SIZE: u32 = 0x300;

const FLASH_END: u32 = FLASH_BASE + FLASH_SIZE - 1;
const SRAM_END: u32 = SRAM_BASE + SRAM_SIZE - 1;
const PMC_END: u32 = PMC_BASE + PMC_SIZE - 1;
const UART0_END: u32 = UART0_BASE + UART0_SIZE - 1;
const EFC_END: u32 = EFC_BASE + EFC_SIZE - 1;
const MATRIX_END: u32 = MATRIX_BASE + MATRIX_SIZE - 1;
const PIO_END: u32 = PIO_BASE + PIO_SIZE * PIO_COUNT - 1;
const RSTC_END: u32 = RSTC_BASE + RSTC_SIZE - 1;
const SUPC_END: u32 = SUPC_BASE + SUPC_SIZE - 1;
const WDT_END: u32 = WDT_BASE + WDT_SIZE - 1;
const SYSTICK_END: u32 = SYSTICK_BASE + SYSTICK_SIZE - 1;
const NVIC_END: u32 = NVIC_BASE + NVIC_SIZE - 1;
const SCB_END: u32 = SCB_BASE + SCB_SIZE - 1;

// Two System Control Block registers that the bus answers itself, because
// they act on the pending and active exceptions (B3.2.4 and B3.2.21 of the
// ARM manual).
const ICSR: u32 = SCB_BASE + 0x04; // Interrupt Control and State
const STIR: u32 = SCB_BASE + 0x200; // Software Triggered Interrupt

// ICSR bits.
const ICSR_NMIPENDSET: u32 = 1 << 31; // write 1: raise NMI; reads 1 while pending
const ICSR_PENDSVSET: u32 = 1 << 28; // write 1: pend PendSV; reads 1 while pending
const ICSR_PENDSVCLR: u32 = 1 << 27; // write 1: PendSV no longer pending
const ICSR_PENDSTSET: u32 = 1 << 26; // write 1: pend SysTick; reads 1 while pending
const ICSR_PENDSTCLR: u32 = 1 << 25; // write 1: SysTick no longer pending
const ICSR_ISRPENDING: u32 = 1 << 22; // some IRQ is pending
const ICSR_VECTPENDING_SHIFT: u32 = 12; // bits 20-12: most urgent pending exception
const ICSR_RETTOBASE: u32 = 1 << 11; // no handler is active but the running one
// Bits 8-0, VECTACTIVE: the exception whose handler is running.

const ICSR_END: u32 = ICSR + 3;

const EXCEPTION_NMI: u32 = 2;
const EXCEPTION_PENDSV: u32 = 14;
/// Exception number of the SysTick exception.
const EXCEPTION_SYSTICK: u32 = 15;
/// Exception number of IRQ 0. IRQ n is exception 16 + n.
const EXCEPTION_IRQ0: u32 = 16;

/// Whether code can run from an address.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Executable {
    /// Flash or RAM.
    Yes,
    /// Nothing is there at all.
    NoMemory,
    /// The peripheral and system regions, from 0x4000_0000 up, can never
    /// hold code ("execute never" in the architecture's memory map,
    /// B3.1), except for the external RAM region, which is not used here.
    Never,
}

// SHCSR bits that enable the three configurable fault exceptions.
const SHCSR_MEMFAULTENA: u32 = 1 << 16;
const SHCSR_BUSFAULTENA: u32 = 1 << 17;
const SHCSR_USGFAULTENA: u32 = 1 << 18;

/// The priority of code running with no exception active (Thread mode):
/// one below the lowest priority an exception can have, so that anything
/// can interrupt it.
pub const PRIORITY_THREAD: i32 = 256;

// Interrupt numbers (from the SVD).
const IRQ_UART0: u32 = 7;
/// One per I2C controller, TWIHS0 to TWIHS2.
const IRQ_TWIHS: [u32; 3] = [19, 20, 41];
/// One per PIO port, PIOA to PIOE.
const IRQ_PIO: [u32; PIO_COUNT as usize] = [10, 11, 12, 16, 17];
/// One per Timer Counter channel: `IRQ_TC[module][channel]`.
const IRQ_TC: [[u32; 3]; 4] = [[23, 24, 25], [26, 27, 28], [47, 48, 49], [50, 51, 52]];

/// A change to an input pin, due at a given moment: what the `--pin=`
/// option sets up.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PinEvent {
    /// When, in clock cycles since reset.
    pub cycle: u64,
    /// PIO port, 0 (PIOA) to 4 (PIOE).
    pub port: usize,
    /// Pin within the port, 0 to 31.
    pub pin: u32,
    /// The level the pin is driven to.
    pub high: bool,
}

/// Parse a pin's name, such as `PA11` or `PD28`, into its port (0 for
/// PIOA to 4 for PIOE) and its number within the port (0 to 31).
pub fn parse_pin_name(name: &str) -> Result<(usize, u32), String> {
    let error = || format!("bad pin name '{}': expected something like PA11", name);
    let mut letters = name.chars();
    if letters.next() != Some('P') {
        return Err(error());
    }
    let port = match letters.next() {
        Some(letter @ 'A'..='E') => letter as usize - 'A' as usize,
        _ => return Err(error()),
    };
    let pin: u32 = letters.as_str().parse().map_err(|_| error())?;
    if pin > 31 {
        return Err(error());
    }
    Ok((port, pin))
}

/// A wire from an I2C device's interrupt output to an input pin.
struct InterruptWire {
    /// Which I2C controller's bus the device is on, and its address.
    controller: usize,
    address: u8,
    /// The pin the wire goes to.
    port: usize,
    pin: u32,
    /// The level last put on the pin.
    level: bool,
}

impl PinEvent {
    /// Parse the text of a `--pin=` option: port and pin, level, and the
    /// cycle to apply it at, for example `PA11=0@5000000` for "drive
    /// PA11 low at cycle 5,000,000". Underscores in the number are
    /// ignored.
    pub fn parse(text: &str) -> Result<PinEvent, String> {
        let error = || {
            format!(
                "bad pin event '{}': expected something like PA11=0@5000000",
                text
            )
        };
        let (name, rest) = text.split_once('=').ok_or_else(error)?;
        let (level, cycle) = rest.split_once('@').ok_or_else(error)?;
        let (port, pin) = parse_pin_name(name).map_err(|_| error())?;
        let high = match level {
            "0" => false,
            "1" => true,
            _ => return Err(error()),
        };
        let cycle = cycle.replace('_', "").parse().map_err(|_| error())?;
        Ok(PinEvent {
            cycle,
            port,
            pin,
            high,
        })
    }
}

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

/// Which I2C controller `addr` belongs to, if any.
fn twihs_index(addr: u32) -> Option<usize> {
    TWIHS_BASES
        .iter()
        .position(|&base| (base..base + TWIHS_SIZE).contains(&addr))
}

/// Which Timer Counter module `addr` belongs to, if any.
fn tc_module(addr: u32) -> Option<usize> {
    TC_BASES
        .iter()
        .position(|&base| (base..base + TC_SIZE).contains(&addr))
}

pub struct Bus {
    flash: Vec<u8>,
    sram: Vec<u8>,
    // Peripheral stubs, each implementing the Peripheral trait.
    pmc: PmcStub,
    uart0: Uart0,
    efc: EfcStub,
    wdt: WdtStub,
    matrix: MatrixStub,
    pio: [PioStub; PIO_COUNT as usize], // PIOA to PIOE
    rstc: RstcStub,
    supc: SupcStub,
    scb: ScbStub,
    systick: SysTick,
    nvic: Nvic,
    tc: [Tc; 4],       // TC0 to TC3
    twihs: [Twihs; 3], // TWIHS0 to TWIHS2
    dwt: Dwt,
    /// Clock cycles since reset, as counted by `tick`.
    cycles: u64,
    /// Changes to input pins still to come, latest first.
    pin_events: Vec<PinEvent>,
    /// Interrupt outputs of I2C devices that are wired to input pins.
    interrupt_wires: Vec<InterruptWire>,
    /// System exceptions (numbers below 16) that have been requested and
    /// are waiting for the CPU to take them: bit n is set while exception
    /// number n is pending. Peripheral interrupts are kept by the NVIC.
    pending_exceptions: u128,
    /// Exceptions whose handler has started and not yet returned: bit n
    /// for exception number n. More than one bit is set while handlers
    /// are nested.
    active_exceptions: u128,
    /// The exception whose handler is running now (the CPU's IPSR), or 0
    /// in Thread mode. Kept here so that ICSR can report it.
    current_exception: u32,
    /// The `--bus-faults` switch. Off: an access to an unmapped address
    /// prints a warning and carries on, which shows which peripheral the
    /// firmware wants next. On: it is a BusFault, as on the real chip.
    bus_faults: bool,
    /// With the switch on: the first unmapped data access since the CPU
    /// last asked, as (address, was it a write).
    bus_fault: Option<(u32, bool)>,
}

impl Bus {
    pub fn new() -> Self {
        Bus {
            flash: vec![0xFF; FLASH_SIZE as usize], // erased flash reads as 0xFF
            sram: vec![0; SRAM_SIZE as usize],
            pmc: PmcStub::new(),
            uart0: Uart0::new(),
            efc: EfcStub::new(),
            wdt: WdtStub::new(),
            matrix: MatrixStub::new(),
            pio: std::array::from_fn(|_| PioStub::new()),
            rstc: RstcStub::new(),
            supc: SupcStub::new(),
            scb: ScbStub::new(),
            systick: SysTick::new(),
            nvic: Nvic::new(),
            tc: std::array::from_fn(|_| Tc::new()),
            twihs: std::array::from_fn(|_| Twihs::new()),
            dwt: Dwt::new(),
            cycles: 0,
            pin_events: Vec::new(),
            interrupt_wires: Vec::new(),
            pending_exceptions: 0,
            active_exceptions: 0,
            current_exception: 0,
            bus_faults: false,
            bus_fault: None,
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

    /// Let emulated time move on by `cycles` clock cycles. The CPU calls
    /// this after every instruction with the number of cycles it took;
    /// that is the only clock there is.
    pub fn tick(&mut self, cycles: u32) {
        if self.systick.tick(cycles) {
            self.pend_exception(EXCEPTION_SYSTICK);
        }
        for tc in &mut self.tc {
            tc.tick(cycles);
        }
        self.dwt.tick(cycles, self.scb.trace_enabled());
        for twihs in &mut self.twihs {
            twihs.tick(cycles);
        }
        self.uart0.poll_input();

        // Apply the pin changes whose time has come.
        self.cycles += cycles as u64;
        while let Some(event) = self.pin_events.last().copied() {
            if event.cycle > self.cycles {
                break;
            }
            self.pin_events.pop();
            self.set_pin(event.port, event.pin, event.high);
        }
        self.update_interrupt_wires();
        self.update_irq_lines();
    }

    /// Plug a device into one of the I2C buses: `controller` is 0
    /// (TWIHS0) to 2 (TWIHS2).
    pub fn attach_i2c_device(&mut self, controller: usize, device: Box<dyn I2cDevice>) {
        self.twihs[controller].attach(device);
    }

    /// Wire the interrupt output of the I2C device at `address` on
    /// controller `controller`'s bus to an input pin. From then on the
    /// pin follows the device's output.
    pub fn connect_i2c_interrupt(&mut self, controller: usize, address: u8, port: usize, pin: u32) {
        // An unconnected pin reads high.
        self.interrupt_wires.push(InterruptWire {
            controller,
            address,
            port,
            pin,
            level: true,
        });
        self.update_interrupt_wires();
    }

    /// Put each wired device's interrupt output on its pin, if it has
    /// changed.
    fn update_interrupt_wires(&mut self) {
        for i in 0..self.interrupt_wires.len() {
            let wire = &self.interrupt_wires[i];
            let level = self.twihs[wire.controller].device_interrupt_line(wire.address);
            if let Some(level) = level
                && level != wire.level
            {
                let (port, pin) = (wire.port, wire.pin);
                self.interrupt_wires[i].level = level;
                self.set_pin(port, pin, level);
            }
        }
    }

    /// The outside world drives a pin high or low. `port` is 0 (PIOA)
    /// to 4 (PIOE) and `pin` 0 to 31.
    pub fn set_pin(&mut self, port: usize, pin: u32, high: bool) {
        self.pio[port].set_input(pin, high);
    }

    /// Arrange for a pin to change at a later moment.
    pub fn schedule_pin_event(&mut self, event: PinEvent) {
        self.pin_events.push(event);
        // Latest first, so the next one due is at the end.
        self.pin_events.sort_by(|a, b| b.cycle.cmp(&a.cycle));
    }

    /// Connect UART0's receiver to a source of bytes, such as stdin.
    pub fn set_uart0_input(&mut self, input: Receiver<u8>) {
        self.uart0.set_input(input);
    }

    /// Pass each peripheral's interrupt request on to the NVIC.
    fn update_irq_lines(&mut self) {
        self.nvic
            .set_line(IRQ_UART0, self.uart0.interrupt_requested());
        for (port, &irq) in IRQ_PIO.iter().enumerate() {
            self.nvic
                .set_line(irq, self.pio[port].interrupt_requested());
        }
        for (index, &irq) in IRQ_TWIHS.iter().enumerate() {
            self.nvic
                .set_line(irq, self.twihs[index].interrupt_requested());
        }
        for (module, irqs) in IRQ_TC.iter().enumerate() {
            for (channel, &irq) in irqs.iter().enumerate() {
                self.nvic
                    .set_line(irq, self.tc[module].interrupt_requested(channel));
            }
        }
    }

    /// Ask for exception `number` to be taken. It stays pending until the
    /// CPU takes it. For an IRQ (16 and up) this sets its pending bit in
    /// the NVIC, where it also has to be enabled.
    pub fn pend_exception(&mut self, number: u32) {
        if number < EXCEPTION_IRQ0 {
            self.pending_exceptions |= 1 << number;
        } else {
            self.nvic.set_pending(number - EXCEPTION_IRQ0);
        }
    }

    /// The priority of exception `number`. A lower value is a higher
    /// priority (B1.5.4). Reset, NMI and HardFault have fixed negative
    /// priorities; firmware sets the rest, in the SCB for the system
    /// exceptions and in the NVIC for the IRQs.
    fn exception_priority(&self, number: u32) -> i32 {
        match number {
            1 => -3, // Reset
            2 => -2, // NMI
            3 => -1, // HardFault
            4..EXCEPTION_IRQ0 => self.scb.system_priority(number) as i32,
            _ => self.nvic.priority(number - EXCEPTION_IRQ0) as i32,
        }
    }

    /// The part of `priority` that decides whether one exception can
    /// interrupt another. AIRCR.PRIGROUP splits a priority value in two:
    /// the upper bits are the group priority and the lower bits, the
    /// subpriority, only settle the order among exceptions that are
    /// waiting. With the reset value of PRIGROUP every implemented bit is
    /// group priority.
    pub fn group_priority(&self, priority: i32) -> i32 {
        if priority < 0 {
            return priority; // the fixed priorities are not split
        }
        let group_value = 2 << self.scb.priority_group();
        priority - priority % group_value
    }

    /// The priority the active handlers give the running code: the most
    /// urgent group priority among them, or `PRIORITY_THREAD` if no
    /// exception is active.
    pub fn active_priority(&self) -> i32 {
        let mut priority = PRIORITY_THREAD;
        let mut active = self.active_exceptions;
        while active != 0 {
            let number = active.trailing_zeros();
            active &= active - 1; // clear the lowest set bit
            priority = priority.min(self.group_priority(self.exception_priority(number)));
        }
        priority
    }

    /// Exception `number`'s handler is starting.
    pub fn activate_exception(&mut self, number: u32) {
        self.active_exceptions |= 1 << number;
        if number >= EXCEPTION_IRQ0 {
            self.nvic.set_active(number - EXCEPTION_IRQ0, true);
        }
    }

    /// Exception `number`'s handler has returned.
    pub fn deactivate_exception(&mut self, number: u32) {
        self.active_exceptions &= !(1 << number);
        if number >= EXCEPTION_IRQ0 {
            self.nvic.set_active(number - EXCEPTION_IRQ0, false);
        }
    }

    /// Turn the `--bus-faults` switch on or off.
    pub fn set_bus_faults(&mut self, on: bool) {
        self.bus_faults = on;
    }

    /// The unmapped data access the last instruction made, if the switch
    /// is on and there was one. Asking clears it.
    pub fn take_bus_fault(&mut self) -> Option<(u32, bool)> {
        self.bus_fault.take()
    }

    /// Whether code can run from `addr`.
    pub fn executable(&self, addr: u32) -> Executable {
        match addr {
            0..=BOOT_ALIAS_END | FLASH_BASE..=FLASH_END | SRAM_BASE..=SRAM_END => Executable::Yes,
            0x4000_0000..=0x5FFF_FFFF | 0xA000_0000..=0xFFFF_FFFF => Executable::Never,
            _ => Executable::NoMemory,
        }
    }

    /// True if fault exception `number` (4 MemManage, 5 BusFault, 6
    /// UsageFault) can be used for a fault in code running at priority
    /// `current`: firmware has enabled it in SHCSR, and it is urgent
    /// enough to interrupt. If not, the fault becomes a HardFault.
    pub fn fault_exception_usable(&self, number: u32, current: i32) -> bool {
        let enable = match number {
            4 => SHCSR_MEMFAULTENA,
            5 => SHCSR_BUSFAULTENA,
            6 => SHCSR_USGFAULTENA,
            _ => return false,
        };
        self.scb.shcsr() & enable != 0
            && self.group_priority(self.exception_priority(number)) < current
    }

    /// Record a fault in the SCB's status registers: bits for CFSR and
    /// HFSR, and the address of a failed data access for BFAR.
    pub fn record_fault(&mut self, cfsr_bits: u32, hfsr_bits: u32, address: Option<u32>) {
        self.scb.record_fault(cfsr_bits, hfsr_bits, address);
    }

    /// The CPU reports which exception's handler is running (0 for
    /// Thread mode) whenever that changes.
    pub fn set_current_exception(&mut self, number: u32) {
        self.current_exception = number;
    }

    /// Read ICSR: which exceptions are pending and active.
    fn read_icsr(&mut self) -> u32 {
        let pending = |number: u32| self.pending_exceptions & (1 << number) != 0;
        let mut value = self.current_exception; // VECTACTIVE
        if pending(EXCEPTION_NMI) {
            value |= ICSR_NMIPENDSET;
        }
        if pending(EXCEPTION_PENDSV) {
            value |= ICSR_PENDSVSET;
        }
        if pending(EXCEPTION_SYSTICK) {
            value |= ICSR_PENDSTSET;
        }
        // In a handler: set if returning from it would go back to Thread
        // mode, that is, if it did not interrupt another handler.
        if self.current_exception != 0
            && self.active_exceptions & !(1 << self.current_exception) == 0
        {
            value |= ICSR_RETTOBASE;
        }
        self.update_irq_lines();
        if self.nvic.any_pending() {
            value |= ICSR_ISRPENDING;
        }
        if let Some(number) = self.next_pending_exception() {
            value |= number << ICSR_VECTPENDING_SHIFT;
        }
        value
    }

    /// Write ICSR: firmware asks for NMI, PendSV or SysTick, or withdraws
    /// a PendSV or SysTick that has not been taken yet. Bits written as 0
    /// do nothing.
    fn write_icsr(&mut self, bits: u32) {
        if bits & ICSR_NMIPENDSET != 0 {
            self.pend_exception(EXCEPTION_NMI);
        }
        if bits & ICSR_PENDSVSET != 0 {
            self.pend_exception(EXCEPTION_PENDSV);
        }
        if bits & ICSR_PENDSTSET != 0 {
            self.pend_exception(EXCEPTION_SYSTICK);
        }
        if bits & ICSR_PENDSVCLR != 0 {
            self.pending_exceptions &= !(1 << EXCEPTION_PENDSV);
        }
        if bits & ICSR_PENDSTCLR != 0 {
            self.pending_exceptions &= !(1 << EXCEPTION_SYSTICK);
        }
    }

    /// The most urgent exception that is waiting, if any: the one with
    /// the lowest priority value, and of those the lowest number.
    fn next_pending_exception(&mut self) -> Option<u32> {
        self.update_irq_lines();
        let mut best = self.nvic.next_irq().map(|irq| EXCEPTION_IRQ0 + irq);
        // System exceptions have the lower numbers, so they win a tie.
        // Checked from the highest number down, each one that is at least
        // as urgent as the best so far replaces it.
        let mut pending = self.pending_exceptions;
        while pending != 0 {
            let number = 127 - pending.leading_zeros();
            pending &= !(1 << number);
            if best.is_none_or(|b| self.exception_priority(number) <= self.exception_priority(b)) {
                best = Some(number);
            }
        }
        best
    }

    /// True if an exception is waiting that is urgent enough to interrupt
    /// code running at priority `current`.
    pub fn has_pending_exception(&mut self, current: i32) -> bool {
        self.next_pending_exception()
            .is_some_and(|n| self.group_priority(self.exception_priority(n)) < current)
    }

    /// How many clock cycles from now the next exception will be raised,
    /// if nothing else changes. `None` if nothing is set up to raise one.
    /// A sleeping CPU uses this to skip the wait.
    pub fn cycles_until_next_exception(&self) -> Option<u32> {
        // The sources that work to a timetable: SysTick, the Timer
        // Counters, and pin changes that have been scheduled. UART input
        // arrives whenever the host delivers it, which cannot be
        // predicted; it is noticed on the next tick.
        // Also the next moment an I2C device does something on its own,
        // which may change its interrupt output.
        let devices = self
            .twihs
            .iter()
            .filter_map(Twihs::cycles_until_device_event);
        let timers = self
            .tc
            .iter()
            .filter_map(Tc::cycles_until_interrupt)
            .chain(devices);
        let pin = self.pin_events.last().map(|event| {
            let wait = event.cycle.saturating_sub(self.cycles).max(1);
            u32::try_from(wait).unwrap_or(u32::MAX)
        });
        self.systick
            .cycles_until_exception()
            .into_iter()
            .chain(timers)
            .chain(pin)
            .min()
    }

    /// Hand the CPU a pending exception to take, if one is urgent enough
    /// to interrupt code running at priority `current`, and stop it being
    /// pending. An exception interrupts only if its group priority is
    /// strictly higher (a lower value) than `current`; otherwise it stays
    /// pending.
    pub fn take_pending_exception(&mut self, current: i32) -> Option<u32> {
        let number = self.next_pending_exception()?;
        if self.group_priority(self.exception_priority(number)) >= current {
            return None;
        }
        if number < EXCEPTION_IRQ0 {
            self.pending_exceptions &= !(1 << number);
        } else {
            self.nvic.acknowledge(number - EXCEPTION_IRQ0);
        }
        Some(number)
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
            EFC_BASE..=EFC_END => self.efc.read(addr - EFC_BASE, size),
            WDT_BASE..=WDT_END => self.wdt.read(addr - WDT_BASE, size),
            MATRIX_BASE..=MATRIX_END => self.matrix.read(addr - MATRIX_BASE, size),
            PIO_BASE..=PIO_END => {
                let offset = addr - PIO_BASE;
                self.pio[(offset / PIO_SIZE) as usize].read(offset % PIO_SIZE, size)
            }
            RSTC_BASE..=RSTC_END => self.rstc.read(addr - RSTC_BASE, size),
            SUPC_BASE..=SUPC_END => self.supc.read(addr - SUPC_BASE, size),
            SYSTICK_BASE..=SYSTICK_END => self.systick.read(addr - SYSTICK_BASE, size),
            NVIC_BASE..=NVIC_END => self.nvic.read(addr - NVIC_BASE, size),
            DWT_BASE..=DWT_END => self.dwt.read(addr - DWT_BASE, size),
            _ if tc_module(addr).is_some() => {
                let module = tc_module(addr).unwrap();
                self.tc[module].read(addr - TC_BASES[module], size)
            }
            _ if twihs_index(addr).is_some() => {
                let index = twihs_index(addr).unwrap();
                self.twihs[index].read(addr - TWIHS_BASES[index], size)
            }
            // Any size of access to ICSR: pick the bytes asked for.
            ICSR..=ICSR_END => {
                let word = self.read_icsr() >> ((addr & 3) * 8);
                match size {
                    1 => word & 0xFF,
                    2 => word & 0xFFFF,
                    _ => word,
                }
            }
            SCB_BASE..=SCB_END => self.scb.read(addr - SCB_BASE, size),
            // Unmapped. With the bus-faults switch on, note it for the
            // CPU to raise a BusFault. Otherwise log instead of
            // panicking: the log shows which peripheral the firmware
            // wants next.
            _ => {
                if self.bus_faults {
                    self.bus_fault.get_or_insert((addr, false));
                } else {
                    eprintln!(
                        "WARNING: Unmapped {}-byte read at Addr: {:#010X}",
                        size, addr
                    );
                }
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
            EFC_BASE..=EFC_END => self.efc.write(addr - EFC_BASE, size, value),
            WDT_BASE..=WDT_END => self.wdt.write(addr - WDT_BASE, size, value),
            MATRIX_BASE..=MATRIX_END => self.matrix.write(addr - MATRIX_BASE, size, value),
            PIO_BASE..=PIO_END => {
                let offset = addr - PIO_BASE;
                self.pio[(offset / PIO_SIZE) as usize].write(offset % PIO_SIZE, size, value)
            }
            RSTC_BASE..=RSTC_END => self.rstc.write(addr - RSTC_BASE, size, value),
            SUPC_BASE..=SUPC_END => self.supc.write(addr - SUPC_BASE, size, value),
            SYSTICK_BASE..=SYSTICK_END => self.systick.write(addr - SYSTICK_BASE, size, value),
            NVIC_BASE..=NVIC_END => self.nvic.write(addr - NVIC_BASE, size, value),
            DWT_BASE..=DWT_END => self.dwt.write(addr - DWT_BASE, size, value),
            _ if tc_module(addr).is_some() => {
                let module = tc_module(addr).unwrap();
                self.tc[module].write(addr - TC_BASES[module], size, value)
            }
            _ if twihs_index(addr).is_some() => {
                let index = twihs_index(addr).unwrap();
                self.twihs[index].write(addr - TWIHS_BASES[index], size, value)
            }
            // Writing 1 acts and writing 0 does nothing, so a narrow write
            // only needs its bits moved to their place in the register.
            ICSR..=ICSR_END => self.write_icsr(value << ((addr & 3) * 8)),
            // STIR: the value written is the number of an IRQ to make
            // pending, the same as setting its bit in the NVIC's ISPR.
            STIR => self.nvic.set_pending(value & 0x1FF),
            SCB_BASE..=SCB_END => self.scb.write(addr - SCB_BASE, size, value),
            _ => {
                if self.bus_faults {
                    self.bus_fault.get_or_insert((addr, true));
                } else {
                    eprintln!(
                        "WARNING: Unmapped {}-byte write at Addr: {:#010X} = {:#010X}",
                        size, addr, value
                    );
                }
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
    fn efc_is_mapped_at_its_base_address() {
        let mut bus = Bus::new();
        // EEFC_FSR: FRDY set.
        assert_eq!(bus.read32(EFC_BASE + 0x08), 1);
        // EEFC_FMR is store-and-readback (FWS = 6).
        bus.write32(EFC_BASE, 6 << 8);
        assert_eq!(bus.read32(EFC_BASE), 6 << 8);
    }

    #[test]
    fn wdt_is_mapped_at_its_base_address() {
        let mut bus = Bus::new();
        // WDT_MR: the disable write (WDDIS, bit 15) reads back.
        bus.write32(WDT_BASE + 0x04, 1 << 15);
        assert_eq!(bus.read32(WDT_BASE + 0x04), 1 << 15);
        // The neighbours on either side (RTT, RTC) are not part of the WDT.
        assert_eq!(bus.read32(WDT_BASE - 0x0C), 0);
        assert_eq!(bus.read32(WDT_BASE + 0x14), 0);
    }

    #[test]
    fn rstc_and_supc_are_mapped_side_by_side() {
        let mut bus = Bus::new();
        // RSTC_SR: NRST pin high. RSTC_MR resets with URSTEN set.
        assert_eq!(bus.read32(RSTC_BASE + 0x04), 1 << 16);
        assert_eq!(bus.read32(RSTC_BASE + 0x08), 1);
        // SUPC_CR with key: select the 32 kHz crystal, then SUPC_SR shows
        // OSCSEL (bit 7).
        bus.write32(SUPC_BASE, 0xA500_0008);
        assert_eq!(bus.read32(SUPC_BASE + 0x14), 1 << 7);
        // The SUPC write did not land in the RSTC.
        assert_eq!(bus.read32(RSTC_BASE + 0x08), 1);
    }

    #[test]
    fn each_pio_port_has_its_own_registers() {
        let mut bus = Bus::new();
        const PIOA: u32 = 0x400E_0E00;
        const PIOC: u32 = 0x400E_1200;
        const PIOE: u32 = 0x400E_1600;
        // PIO_PER (+0x00) sets bits in PIO_PSR (+0x08).
        bus.write32(PIOA, 0x0000_0001);
        bus.write32(PIOC, 0x0000_0100);
        bus.write32(PIOE, 0x0000_0020);
        assert_eq!(bus.read32(PIOA + 0x08), 0x0000_0001);
        assert_eq!(bus.read32(PIOC + 0x08), 0x0000_0100);
        assert_eq!(bus.read32(PIOE + 0x08), 0x0000_0020);
        // PIOB and PIOD were not touched.
        assert_eq!(bus.read32(0x400E_1000 + 0x08), 0);
        assert_eq!(bus.read32(0x400E_1400 + 0x08), 0);
    }

    #[test]
    fn matrix_is_mapped_at_its_base_address() {
        let mut bus = Bus::new();
        // CCFG_SYSIO is store-and-readback.
        bus.write32(MATRIX_BASE + 0x114, 0x0000_10F0);
        assert_eq!(bus.read32(MATRIX_BASE + 0x114), 0x0000_10F0);
    }

    #[test]
    fn scb_is_mapped_at_its_base_address() {
        let mut bus = Bus::new();
        assert_eq!(bus.read32(SCB_BASE), 0x411F_C271); // CPUID
        bus.write32(SCB_BASE + 0x08, 0x0040_0000); // VTOR
        assert_eq!(bus.read32(SCB_BASE + 0x08), 0x0040_0000);
        // Byte write to SHPR3 (SysTick priority) goes through.
        bus.write8(SCB_BASE + 0x23, 0xE0);
        assert_eq!(bus.read32(SCB_BASE + 0x20), 0xE000_0000);
    }

    #[test]
    fn nvic_is_mapped_at_its_base_address() {
        let mut bus = Bus::new();
        bus.write32(NVIC_BASE, 1 << 7); // ISER0: enable IRQ 7
        assert_eq!(bus.read32(NVIC_BASE), 1 << 7);
        bus.write8(NVIC_BASE + 0x307, 0xE0); // IPR byte for IRQ 7
        assert_eq!(bus.read32(NVIC_BASE + 0x304), 0xE000_0000);
    }

    /// A bus with UART0's receiver on and an input channel attached.
    fn bus_with_uart_input() -> (Bus, std::sync::mpsc::Sender<u8>) {
        let (sender, receiver) = std::sync::mpsc::channel();
        let mut bus = Bus::new();
        bus.set_uart0_input(receiver);
        bus.write32(UART0_BASE, 1 << 4); // UART_CR: RXEN
        (bus, sender)
    }

    #[test]
    fn uart_byte_raises_irq_7_when_both_switches_are_on() {
        let (mut bus, sender) = bus_with_uart_input();
        bus.write32(UART0_BASE + 0x08, 1 << 0); // UART_IER: RXRDY
        bus.write32(NVIC_BASE, 1 << IRQ_UART0); // ISER0
        bus.tick(1);
        assert!(!bus.has_pending_exception(PRIORITY_THREAD));

        sender.send(b'x').unwrap();
        bus.tick(1);
        assert!(bus.has_pending_exception(PRIORITY_THREAD));
        assert_eq!(bus.take_pending_exception(PRIORITY_THREAD), Some(23));
        // Still asking: the byte has not been read.
        assert_eq!(bus.take_pending_exception(PRIORITY_THREAD), Some(23));
        // Reading UART_RHR withdraws the request.
        assert_eq!(bus.read32(UART0_BASE + 0x18), b'x' as u32);
        assert_eq!(bus.take_pending_exception(PRIORITY_THREAD), None);
        assert!(!bus.has_pending_exception(PRIORITY_THREAD));
    }

    #[test]
    fn uart_byte_raises_nothing_with_either_switch_off() {
        // Enabled in the UART but not in the NVIC.
        let (mut bus, sender) = bus_with_uart_input();
        bus.write32(UART0_BASE + 0x08, 1 << 0);
        sender.send(b'x').unwrap();
        bus.tick(1);
        assert_eq!(bus.take_pending_exception(PRIORITY_THREAD), None);
        // The NVIC shows it pending all the same (ISPR0).
        assert_eq!(bus.read32(NVIC_BASE + 0x100), 1 << IRQ_UART0);

        // Enabled in the NVIC but not in the UART.
        let (mut bus, sender) = bus_with_uart_input();
        bus.write32(NVIC_BASE, 1 << IRQ_UART0);
        sender.send(b'x').unwrap();
        bus.tick(1);
        assert_eq!(bus.take_pending_exception(PRIORITY_THREAD), None);
        // The byte can still be read by polling UART_SR.
        assert_eq!(bus.read32(UART0_BASE + 0x14) & 1, 1);
    }

    #[test]
    fn system_exceptions_are_taken_before_irqs() {
        let (mut bus, sender) = bus_with_uart_input();
        bus.write32(UART0_BASE + 0x08, 1 << 0);
        bus.write32(NVIC_BASE, 1 << IRQ_UART0);
        sender.send(b'x').unwrap();
        bus.tick(1);
        bus.pend_exception(EXCEPTION_SYSTICK);
        assert_eq!(bus.take_pending_exception(PRIORITY_THREAD), Some(15));
        assert_eq!(bus.take_pending_exception(PRIORITY_THREAD), Some(23));
    }

    #[test]
    fn systick_counts_bus_ticks_and_pends_its_exception() {
        let mut bus = Bus::new();
        bus.write32(SYSTICK_BASE + 0x04, 2); // SYST_RVR: reload 2
        bus.write32(SYSTICK_BASE + 0x08, 0); // SYST_CVR: clear
        bus.write32(SYSTICK_BASE, 0b011); // SYST_CSR: ENABLE | TICKINT
        // Load 2, then 1, then 0: the third cycle reaches zero.
        bus.tick(2);
        assert_eq!(bus.take_pending_exception(PRIORITY_THREAD), None);
        bus.tick(1);
        assert_eq!(bus.take_pending_exception(PRIORITY_THREAD), Some(15));
        // Taking it stops it being pending.
        assert_eq!(bus.take_pending_exception(PRIORITY_THREAD), None);
    }

    #[test]
    fn next_exception_time_follows_systick() {
        let mut bus = Bus::new();
        assert_eq!(bus.cycles_until_next_exception(), None);
        assert!(!bus.has_pending_exception(PRIORITY_THREAD));

        bus.write32(SYSTICK_BASE + 0x04, 99); // SYST_RVR: period 100
        bus.write32(SYSTICK_BASE + 0x08, 0); // SYST_CVR: clear
        bus.write32(SYSTICK_BASE, 0b011); // SYST_CSR: ENABLE | TICKINT
        assert_eq!(bus.cycles_until_next_exception(), Some(100));

        bus.tick(30);
        assert_eq!(bus.cycles_until_next_exception(), Some(70));

        // Ticking exactly that far raises it.
        bus.tick(70);
        assert!(bus.has_pending_exception(PRIORITY_THREAD));
        assert_eq!(bus.cycles_until_next_exception(), Some(100));
    }

    #[test]
    fn pending_exceptions_are_taken_lowest_number_first() {
        let mut bus = Bus::new();
        bus.write32(NVIC_BASE, 1 << 7); // ISER0: enable IRQ 7
        bus.pend_exception(23);
        bus.pend_exception(15);
        bus.pend_exception(23); // pending twice is still pending once
        assert_eq!(bus.take_pending_exception(PRIORITY_THREAD), Some(15));
        assert_eq!(bus.take_pending_exception(PRIORITY_THREAD), Some(23));
        assert_eq!(bus.take_pending_exception(PRIORITY_THREAD), None);
    }

    #[test]
    fn uart_status_reads_ready_at_every_size() {
        let mut bus = Bus::new();
        assert_eq!(bus.read32(UART0_BASE + 0x14), 0x202);
        assert_eq!(bus.read16(UART0_BASE + 0x14), 0x202);
        assert_eq!(bus.read8(UART0_BASE + 0x14), 0x02);
    }

    const SHPR3_SYSTICK: u32 = SCB_BASE + 0x23; // SysTick's priority byte
    const NVIC_IPR: u32 = NVIC_BASE + 0x300; // one priority byte per IRQ
    const AIRCR: u32 = SCB_BASE + 0x0C;

    /// A bus with IRQ 7 and IRQ 9 enabled in the NVIC.
    fn bus_with_irqs_enabled() -> Bus {
        let mut bus = Bus::new();
        bus.write32(NVIC_BASE, (1 << 7) | (1 << 9));
        bus
    }

    #[test]
    fn most_urgent_pending_exception_is_taken_first() {
        let mut bus = bus_with_irqs_enabled();
        bus.write8(SHPR3_SYSTICK, 0xE0);
        bus.write8(NVIC_IPR + 7, 0x40);
        bus.write8(NVIC_IPR + 9, 0x20);
        bus.pend_exception(15);
        bus.pend_exception(23);
        bus.pend_exception(25);
        assert_eq!(bus.take_pending_exception(PRIORITY_THREAD), Some(25));
        assert_eq!(bus.take_pending_exception(PRIORITY_THREAD), Some(23));
        assert_eq!(bus.take_pending_exception(PRIORITY_THREAD), Some(15));
        assert_eq!(bus.take_pending_exception(PRIORITY_THREAD), None);
    }

    #[test]
    fn exception_is_taken_only_if_more_urgent_than_the_running_code() {
        let mut bus = bus_with_irqs_enabled();
        bus.write8(NVIC_IPR + 7, 0x40);
        bus.pend_exception(23);
        // Equal priority does not interrupt, nor does anything more urgent.
        assert!(!bus.has_pending_exception(0x40));
        assert_eq!(bus.take_pending_exception(0x40), None);
        assert_eq!(bus.take_pending_exception(0x20), None);
        // It is still waiting, and gets in below less urgent code.
        assert!(bus.has_pending_exception(0x60));
        assert_eq!(bus.take_pending_exception(0x60), Some(23));
    }

    #[test]
    fn active_exceptions_set_the_running_priority() {
        let mut bus = bus_with_irqs_enabled();
        bus.write8(SHPR3_SYSTICK, 0xE0);
        bus.write8(NVIC_IPR + 7, 0x40);
        assert_eq!(bus.active_priority(), PRIORITY_THREAD);

        bus.activate_exception(15);
        assert_eq!(bus.active_priority(), 0xE0);
        bus.activate_exception(23); // nested inside SysTick
        assert_eq!(bus.active_priority(), 0x40);
        assert_eq!(bus.read32(NVIC_BASE + 0x200), 1 << 7); // IABR0

        bus.deactivate_exception(23);
        assert_eq!(bus.active_priority(), 0xE0);
        assert_eq!(bus.read32(NVIC_BASE + 0x200), 0);
        bus.deactivate_exception(15);
        assert_eq!(bus.active_priority(), PRIORITY_THREAD);
    }

    #[test]
    fn fixed_priorities_beat_every_configurable_one() {
        let mut bus = Bus::new();
        bus.pend_exception(15); // SysTick, priority 0
        bus.pend_exception(3); // HardFault, priority -1
        bus.pend_exception(2); // NMI, priority -2
        // Nothing configurable gets in above priority 0, but these do.
        assert_eq!(bus.take_pending_exception(0), Some(2));
        assert_eq!(bus.take_pending_exception(0), Some(3));
        assert_eq!(bus.take_pending_exception(0), None);
        // HardFault cannot interrupt NMI or itself.
        bus.pend_exception(3);
        assert_eq!(bus.take_pending_exception(-1), None);
        assert_eq!(bus.take_pending_exception(-2), None);
    }

    #[test]
    fn only_three_priority_bits_exist() {
        let mut bus = Bus::new();
        bus.write8(SHPR3_SYSTICK, 0xFF);
        assert_eq!(bus.read8(SHPR3_SYSTICK), 0xE0);
        assert_eq!(bus.exception_priority(15), 0xE0);
    }

    #[test]
    fn priority_grouping_stops_subpriority_from_interrupting() {
        let mut bus = bus_with_irqs_enabled();
        bus.write8(NVIC_IPR + 7, 0x40);
        bus.write8(NVIC_IPR + 9, 0x20);
        // PRIGROUP = 6: only bit 7 is group priority, so 0x20 and 0x40
        // are in the same group.
        bus.write32(AIRCR, 0x05FA_0000 | (6 << 8));
        assert_eq!(bus.group_priority(0x40), 0);
        assert_eq!(bus.group_priority(0xE0), 0x80);

        bus.activate_exception(23);
        bus.pend_exception(25);
        assert_eq!(bus.take_pending_exception(bus.active_priority()), None);
        // Among waiting exceptions the subpriority still orders them.
        bus.deactivate_exception(23);
        bus.pend_exception(23);
        assert_eq!(bus.take_pending_exception(PRIORITY_THREAD), Some(25));

        // With the reset grouping, 0x20 does interrupt 0x40.
        bus.write32(AIRCR, 0x05FA_0000);
        bus.activate_exception(23);
        bus.pend_exception(25);
        assert_eq!(bus.take_pending_exception(bus.active_priority()), Some(25));
    }

    #[test]
    fn icsr_write_pends_systick_pendsv_and_nmi() {
        let mut bus = Bus::new();
        bus.write32(ICSR, ICSR_PENDSTSET | ICSR_PENDSVSET | ICSR_NMIPENDSET);
        // They read back as pending.
        let icsr = bus.read32(ICSR);
        assert_eq!(
            icsr & (ICSR_PENDSTSET | ICSR_PENDSVSET | ICSR_NMIPENDSET),
            ICSR_PENDSTSET | ICSR_PENDSVSET | ICSR_NMIPENDSET
        );
        // NMI first (priority -2), then PendSV (14) before SysTick (15).
        assert_eq!(bus.take_pending_exception(PRIORITY_THREAD), Some(2));
        assert_eq!(bus.take_pending_exception(PRIORITY_THREAD), Some(14));
        assert_eq!(bus.take_pending_exception(PRIORITY_THREAD), Some(15));
        assert_eq!(bus.read32(ICSR), 0);
    }

    #[test]
    fn icsr_write_can_withdraw_pendsv_and_systick() {
        let mut bus = Bus::new();
        bus.write32(ICSR, ICSR_PENDSTSET | ICSR_PENDSVSET);
        bus.write32(ICSR, ICSR_PENDSVCLR);
        assert_eq!(bus.read32(ICSR) & ICSR_PENDSVSET, 0);
        assert_ne!(bus.read32(ICSR) & ICSR_PENDSTSET, 0);
        bus.write32(ICSR, ICSR_PENDSTCLR);
        assert_eq!(bus.take_pending_exception(PRIORITY_THREAD), None);
    }

    #[test]
    fn icsr_byte_write_reaches_the_right_bits() {
        let mut bus = Bus::new();
        // PENDSVSET is bit 28: bit 4 of the top byte.
        bus.write8(ICSR + 3, 1 << 4);
        assert_eq!(bus.take_pending_exception(PRIORITY_THREAD), Some(14));
        bus.write32(ICSR, ICSR_PENDSTSET);
        assert_eq!(bus.read8(ICSR + 3), (ICSR_PENDSTSET >> 24) as u8);
    }

    #[test]
    fn icsr_reports_the_most_urgent_pending_exception() {
        let mut bus = bus_with_irqs_enabled();
        assert_eq!(bus.read32(ICSR), 0);
        bus.write8(SHPR3_SYSTICK, 0xE0);
        bus.write8(NVIC_IPR + 7, 0x20);
        bus.write32(ICSR, ICSR_PENDSTSET);
        assert_eq!((bus.read32(ICSR) >> 12) & 0x1FF, 15);
        assert_eq!(bus.read32(ICSR) & ICSR_ISRPENDING, 0);

        bus.pend_exception(23); // IRQ 7, more urgent
        assert_eq!((bus.read32(ICSR) >> 12) & 0x1FF, 23);
        assert_ne!(bus.read32(ICSR) & ICSR_ISRPENDING, 0);
    }

    #[test]
    fn icsr_reports_the_running_handler_and_whether_it_is_nested() {
        let mut bus = Bus::new();
        bus.activate_exception(15);
        bus.set_current_exception(15);
        assert_eq!(bus.read32(ICSR) & 0x1FF, 15);
        assert_ne!(bus.read32(ICSR) & ICSR_RETTOBASE, 0);

        bus.activate_exception(23); // interrupts the SysTick handler
        bus.set_current_exception(23);
        assert_eq!(bus.read32(ICSR) & 0x1FF, 23);
        assert_eq!(bus.read32(ICSR) & ICSR_RETTOBASE, 0);

        bus.deactivate_exception(23);
        bus.set_current_exception(15);
        assert_ne!(bus.read32(ICSR) & ICSR_RETTOBASE, 0);
    }

    #[test]
    fn stir_write_pends_an_irq() {
        let mut bus = bus_with_irqs_enabled();
        bus.write32(STIR, 9);
        assert_eq!(bus.read32(NVIC_BASE + 0x100), 1 << 9); // ISPR0
        assert_eq!(bus.take_pending_exception(PRIORITY_THREAD), Some(25));
        // A number with no IRQ behind it is ignored.
        bus.write32(STIR, 300);
        assert_eq!(bus.take_pending_exception(PRIORITY_THREAD), None);
    }

    #[test]
    fn unmapped_access_is_noted_only_with_the_switch_on() {
        let mut bus = Bus::new();
        bus.read32(0x6000_0000);
        assert_eq!(bus.take_bus_fault(), None);

        bus.set_bus_faults(true);
        bus.write8(0x6000_0010, 1);
        bus.read32(0x6000_0020); // only the first is kept
        assert_eq!(bus.take_bus_fault(), Some((0x6000_0010, true)));
        assert_eq!(bus.take_bus_fault(), None);
        // Mapped addresses are unaffected.
        bus.read32(SRAM_BASE);
        bus.read32(UART0_BASE + 0x14);
        assert_eq!(bus.take_bus_fault(), None);
    }

    #[test]
    fn only_flash_and_ram_can_hold_code() {
        let bus = Bus::new();
        assert_eq!(bus.executable(0x0000_0100), Executable::Yes);
        assert_eq!(bus.executable(FLASH_BASE), Executable::Yes);
        assert_eq!(bus.executable(SRAM_END - 1), Executable::Yes);
        assert_eq!(bus.executable(0x1000_0000), Executable::NoMemory);
        assert_eq!(bus.executable(SRAM_END + 1), Executable::NoMemory);
        assert_eq!(bus.executable(UART0_BASE), Executable::Never);
        assert_eq!(bus.executable(0xFFFF_FFF8), Executable::Never);
    }

    #[test]
    fn fault_exception_needs_its_enable_bit_and_enough_priority() {
        let mut bus = Bus::new();
        const SHCSR: u32 = SCB_BASE + 0x24;
        assert!(!bus.fault_exception_usable(6, PRIORITY_THREAD));
        bus.write32(SHCSR, SHCSR_USGFAULTENA);
        assert!(bus.fault_exception_usable(6, PRIORITY_THREAD));
        assert!(!bus.fault_exception_usable(5, PRIORITY_THREAD)); // BusFault is still off
        // UsageFault has priority 0 by default: it cannot interrupt code
        // already running at 0.
        assert!(!bus.fault_exception_usable(6, 0));
    }

    const TC0: u32 = TC_BASES[0];
    const TC_CMR_INTERVAL: u32 = (1 << 15) | (1 << 14); // waveform, restart at RC, PCK6

    /// Start channel `channel` of TC `module` as an interval timer with
    /// its RC interrupt on, and enable that IRQ in the NVIC.
    fn start_timer(bus: &mut Bus, module: usize, channel: u32, period: u32) {
        let base = TC_BASES[module] + 0x40 * channel;
        bus.write32(base + 0x04, TC_CMR_INTERVAL);
        bus.write32(base + 0x1C, period); // TC_RC
        bus.write32(base + 0x24, 1 << 4); // TC_IER: CPCS
        bus.write32(base, 0b101); // TC_CCR: CLKEN | SWTRG
        let irq = IRQ_TC[module][channel as usize];
        bus.write32(NVIC_BASE + 4 * (irq / 32), 1 << (irq % 32));
    }

    #[test]
    fn timer_counter_raises_its_own_irq() {
        let mut bus = Bus::new();
        start_timer(&mut bus, 0, 0, 100);
        bus.tick(99);
        assert_eq!(bus.take_pending_exception(PRIORITY_THREAD), None);
        bus.tick(1);
        assert_eq!(bus.take_pending_exception(PRIORITY_THREAD), Some(16 + 23));
        // Reading TC_SR withdraws the request.
        bus.read32(TC0 + 0x20);
        assert_eq!(bus.take_pending_exception(PRIORITY_THREAD), None);
    }

    #[test]
    fn every_timer_channel_has_its_own_address_and_irq() {
        for (module, irqs) in IRQ_TC.iter().enumerate() {
            for (channel, &irq) in irqs.iter().enumerate() {
                let mut bus = Bus::new();
                start_timer(&mut bus, module, channel as u32, 10);
                bus.tick(10);
                assert_eq!(
                    bus.take_pending_exception(PRIORITY_THREAD),
                    Some(16 + irq),
                    "TC{} channel {}",
                    module,
                    channel
                );
            }
        }
    }

    #[test]
    fn next_exception_time_is_the_earliest_of_the_timers() {
        let mut bus = Bus::new();
        bus.write32(SYSTICK_BASE + 0x04, 999); // SysTick: period 1000
        bus.write32(SYSTICK_BASE + 0x08, 0);
        bus.write32(SYSTICK_BASE, 0b011);
        assert_eq!(bus.cycles_until_next_exception(), Some(1000));
        start_timer(&mut bus, 1, 2, 300);
        assert_eq!(bus.cycles_until_next_exception(), Some(300));
        bus.tick(250);
        assert_eq!(bus.cycles_until_next_exception(), Some(50));
        // A pin change due sooner comes first.
        bus.schedule_pin_event(PinEvent::parse("PA11=0@270").unwrap());
        assert_eq!(bus.cycles_until_next_exception(), Some(20));
    }

    #[test]
    fn pin_change_raises_the_ports_irq() {
        let mut bus = Bus::new();
        const PIOA: u32 = 0x400E_0E00;
        bus.write32(PIOA + 0x40, 1 << 11); // PIO_IER
        bus.write32(NVIC_BASE, 1 << 10); // ISER0: PIOA is IRQ 10
        bus.tick(1);
        assert_eq!(bus.take_pending_exception(PRIORITY_THREAD), None);

        bus.set_pin(0, 11, false);
        assert_eq!(bus.read32(PIOA + 0x3C) & (1 << 11), 0); // PIO_PDSR
        assert_eq!(bus.take_pending_exception(PRIORITY_THREAD), Some(16 + 10));
        // Reading PIO_ISR withdraws the request.
        assert_eq!(bus.read32(PIOA + 0x4C), 1 << 11);
        assert_eq!(bus.take_pending_exception(PRIORITY_THREAD), None);
    }

    #[test]
    fn each_port_has_its_own_irq() {
        for (port, &irq) in IRQ_PIO.iter().enumerate() {
            let mut bus = Bus::new();
            let base = PIO_BASE + PIO_SIZE * port as u32;
            bus.write32(base + 0x40, 1); // PIO_IER, pin 0
            bus.write32(NVIC_BASE, 1 << irq);
            bus.set_pin(port, 0, false);
            assert_eq!(bus.take_pending_exception(PRIORITY_THREAD), Some(16 + irq));
        }
    }

    #[test]
    fn scheduled_pin_events_happen_at_their_cycle_in_order() {
        let mut bus = Bus::new();
        const PIOB_PDSR: u32 = 0x400E_1000 + 0x3C;
        // Given out of order.
        bus.schedule_pin_event(PinEvent::parse("PB3=1@200").unwrap());
        bus.schedule_pin_event(PinEvent::parse("PB3=0@100").unwrap());
        bus.tick(99);
        assert_ne!(bus.read32(PIOB_PDSR) & (1 << 3), 0);
        bus.tick(1);
        assert_eq!(bus.read32(PIOB_PDSR) & (1 << 3), 0);
        bus.tick(150); // passes cycle 200 in the middle of a step
        assert_ne!(bus.read32(PIOB_PDSR) & (1 << 3), 0);
        assert_eq!(bus.cycles_until_next_exception(), None);
    }

    #[test]
    fn pin_event_text_is_parsed() {
        assert_eq!(
            PinEvent::parse("PA11=0@5_000_000"),
            Ok(PinEvent {
                cycle: 5_000_000,
                port: 0,
                pin: 11,
                high: false
            })
        );
        assert_eq!(
            PinEvent::parse("PE31=1@0"),
            Ok(PinEvent {
                cycle: 0,
                port: 4,
                pin: 31,
                high: true
            })
        );
        for bad in [
            "PA11", "PA11=0", "PF1=0@5", "PA32=0@5", "PA1=2@5", "A1=0@5", "PA1=0@x",
        ] {
            assert!(PinEvent::parse(bad).is_err(), "{}", bad);
        }
    }

    #[test]
    fn cycle_counter_counts_once_both_enables_are_set() {
        let mut bus = Bus::new();
        const DEMCR: u32 = 0xE000_EDFC;
        const DWT_CTRL: u32 = DWT_BASE;
        const DWT_CYCCNT: u32 = DWT_BASE + 4;
        bus.tick(100);
        assert_eq!(bus.read32(DWT_CYCCNT), 0);
        bus.write32(DEMCR, 1 << 24); // TRCENA
        bus.write32(DWT_CTRL, 1); // CYCCNTENA
        bus.tick(100);
        bus.tick(23);
        assert_eq!(bus.read32(DWT_CYCCNT), 123);
    }

    /// A 16-byte I2C memory at address 0x57 holding 0xA0, 0xA1, ...
    fn test_memory() -> Box<dyn I2cDevice> {
        let mut memory = crate::i2c::I2cMemory::new(0x57, 16);
        for (i, byte) in memory.data_mut().iter_mut().enumerate() {
            *byte = 0xA0 + i as u8;
        }
        Box::new(memory)
    }

    /// Read one register of the device at 0x57 through the controller
    /// at `base`, as firmware would. `None` if nothing answered.
    fn i2c_read_register(bus: &mut Bus, base: u32, register: u32) -> Option<u8> {
        bus.write32(base, 1 << 2); // TWIHS_CR: MSEN
        bus.write32(base + 0x04, (0x57 << 16) | (1 << 12) | (1 << 8)); // TWIHS_MMR
        bus.write32(base + 0x0C, register); // TWIHS_IADR
        bus.write32(base, 0b11); // TWIHS_CR: START | STOP
        if bus.read32(base + 0x20) & (1 << 8) != 0 {
            return None; // TWIHS_SR: NACK
        }
        Some(bus.read32(base + 0x30) as u8) // TWIHS_RHR
    }

    #[test]
    fn firmware_can_read_an_i2c_device_through_twihs0() {
        let mut bus = Bus::new();
        bus.attach_i2c_device(0, test_memory());
        assert_eq!(i2c_read_register(&mut bus, TWIHS_BASES[0], 3), Some(0xA3));
    }

    #[test]
    fn each_i2c_controller_has_its_own_address_and_its_own_bus() {
        for attached in 0..3 {
            let mut bus = Bus::new();
            bus.attach_i2c_device(attached, test_memory());
            for (index, &base) in TWIHS_BASES.iter().enumerate() {
                let expected = if index == attached { Some(0xA0) } else { None };
                assert_eq!(
                    i2c_read_register(&mut bus, base, 0),
                    expected,
                    "device on TWIHS{}, read through TWIHS{}",
                    attached,
                    index
                );
            }
        }
    }

    #[test]
    fn i2c_controller_with_nothing_attached_reports_no_answer() {
        let mut bus = Bus::new();
        assert_eq!(i2c_read_register(&mut bus, TWIHS_BASES[1], 0), None);
    }

    #[test]
    fn each_i2c_controller_raises_its_own_irq() {
        for (index, &irq) in IRQ_TWIHS.iter().enumerate() {
            let mut bus = Bus::new();
            bus.attach_i2c_device(index, test_memory());
            let base = TWIHS_BASES[index];
            bus.write32(base + 0x24, 1 << 1); // TWIHS_IER: RXRDY
            bus.write32(NVIC_BASE + 4 * (irq / 32), 1 << (irq % 32));
            assert_eq!(bus.take_pending_exception(PRIORITY_THREAD), None);

            bus.write32(base, 1 << 2); // MSEN
            bus.write32(base + 0x04, (0x57 << 16) | (1 << 12));
            bus.write32(base, 0b11); // START | STOP: a byte arrives
            assert_eq!(bus.take_pending_exception(PRIORITY_THREAD), Some(16 + irq));
            bus.read32(base + 0x30); // reading it withdraws the request
            assert_eq!(bus.take_pending_exception(PRIORITY_THREAD), None);
        }
    }

    /// An I2C device that is nothing but an interrupt output, which the
    /// test sets through a shared flag.
    struct Alarm(std::rc::Rc<std::cell::Cell<bool>>);
    impl I2cDevice for Alarm {
        fn address(&self) -> u8 {
            0x29
        }
        fn start(&mut self) {}
        fn write(&mut self, _byte: u8) -> bool {
            true
        }
        fn read(&mut self) -> u8 {
            0
        }
        fn interrupt_line(&self) -> Option<bool> {
            Some(self.0.get())
        }
    }

    #[test]
    fn device_interrupt_output_drives_the_pin_it_is_wired_to() {
        const PIOD: u32 = 0x400E_1400;
        let line = std::rc::Rc::new(std::cell::Cell::new(false));
        let mut bus = Bus::new();
        bus.attach_i2c_device(0, Box::new(Alarm(line.clone())));
        bus.connect_i2c_interrupt(0, 0x29, 3, 28); // to PD28
        // The device holds the line low from the start.
        assert_eq!(bus.read32(PIOD + 0x3C) & (1 << 28), 0); // PIO_PDSR
        bus.read32(PIOD + 0x4C); // PIO_ISR: forget that first edge

        // Rising edge only, with the interrupt on.
        bus.write32(PIOD + 0xB0, 1 << 28); // PIO_AIMER
        bus.write32(PIOD + 0xC0, 1 << 28); // PIO_ESR
        bus.write32(PIOD + 0xD4, 1 << 28); // PIO_REHLSR
        bus.write32(PIOD + 0x40, 1 << 28); // PIO_IER
        bus.write32(NVIC_BASE, 1 << 16); // PIOD is IRQ 16
        bus.tick(1);
        assert_eq!(bus.take_pending_exception(PRIORITY_THREAD), None);

        line.set(true);
        bus.tick(1);
        assert_ne!(bus.read32(PIOD + 0x3C) & (1 << 28), 0);
        assert_eq!(bus.take_pending_exception(PRIORITY_THREAD), Some(16 + 16));
        assert_eq!(bus.read32(PIOD + 0x4C), 1 << 28);

        // Lowering it again is not the edge asked for.
        line.set(false);
        bus.tick(1);
        assert_eq!(bus.take_pending_exception(PRIORITY_THREAD), None);
    }

    #[test]
    fn unwired_device_interrupt_output_changes_no_pin() {
        let line = std::rc::Rc::new(std::cell::Cell::new(false));
        let mut bus = Bus::new();
        bus.attach_i2c_device(0, Box::new(Alarm(line)));
        bus.tick(1);
        assert_eq!(bus.read32(0x400E_1400 + 0x3C), 0xFFFF_FFFF);
    }

    #[test]
    fn pin_names_are_parsed() {
        assert_eq!(parse_pin_name("PD28"), Ok((3, 28)));
        assert_eq!(parse_pin_name("PA0"), Ok((0, 0)));
        for bad in ["D28", "PF1", "PA32", "PA", "PAx"] {
            assert!(parse_pin_name(bad).is_err(), "{}", bad);
        }
    }
}
