//! TC (Timer Counter): one module of three independent timer channels.
//! The SAME70 has four modules, TC0 to TC3, so twelve channels in all.
//!
//! What is modelled is a channel used as an interval timer: it counts
//! up at a chosen fraction of the clock, compares the count with three
//! compare registers (RA, RB, RC), sets status bits when they match or
//! when the counter overflows, and asks for its interrupt for as long
//! as a status bit that firmware enabled is set. The usual set-up is
//! "restart from 0 whenever the count reaches RC", which gives one
//! interrupt every RC counts.
//!
//! Not modelled: the pins. There is no waveform output (PWM), no
//! capture of an input signal, no external clock or trigger, and no
//! up-down counting.
//!
//! Register offsets and bit positions are from `ATSAME70Q21.svd`, which
//! describes TC_CMR only in its capture-mode layout. The waveform-mode
//! bits (WAVSEL, CPCSTOP, CPCDIS), the five clock choices, the 16-bit
//! counter width and the counting behaviour are from chapter 50 of the
//! SAM E70 datasheet in `reference/same70/`.

use crate::peripheral::Peripheral;

// Register offsets within a channel. Channel n is at n * 0x40.
const TC_CCR: u32 = 0x00; // Channel Control (write-only)
const TC_CMR: u32 = 0x04; // Channel Mode
const TC_CV: u32 = 0x10; // Counter Value (read-only)
const TC_RA: u32 = 0x14; // Register A
const TC_RB: u32 = 0x18; // Register B
const TC_RC: u32 = 0x1C; // Register C
const TC_SR: u32 = 0x20; // Status (read-only; reading clears it)
const TC_IER: u32 = 0x24; // Interrupt Enable (write-only)
const TC_IDR: u32 = 0x28; // Interrupt Disable (write-only)
const TC_IMR: u32 = 0x2C; // Interrupt Mask (read-only)

const CHANNEL_SIZE: u32 = 0x40;
const CHANNELS: usize = 3;

// Registers of the whole module.
const TC_BCR: u32 = 0xC0; // Block Control (write-only)
const TC_BMR: u32 = 0xC4; // Block Mode

// TC_CCR bits.
const CCR_CLKEN: u32 = 1 << 0; // enable the channel's clock
const CCR_CLKDIS: u32 = 1 << 1; // disable it
const CCR_SWTRG: u32 = 1 << 2; // trigger: reset the counter and start it

// TC_CMR bits.
const CMR_TCCLKS: u32 = 0x7; // which clock to count
const CMR_CPCSTOP: u32 = 1 << 6; // waveform mode: stop at an RC match
const CMR_CPCDIS: u32 = 1 << 7; // waveform mode: disable the clock at an RC match
const CMR_WAVSEL_RC: u32 = 1 << 14; // waveform mode: restart at an RC match
const CMR_CPCTRG: u32 = 1 << 14; // capture mode: restart at an RC match
const CMR_WAVE: u32 = 1 << 15; // 1 = waveform mode, 0 = capture mode

// TC_SR bits. TC_IER, TC_IDR and TC_IMR use the same positions.
const SR_COVFS: u32 = 1 << 0; // the counter overflowed
const SR_CPAS: u32 = 1 << 2; // the count matched RA
const SR_CPBS: u32 = 1 << 3; // the count matched RB
const SR_CPCS: u32 = 1 << 4; // the count matched RC
const SR_CLKSTA: u32 = 1 << 16; // the channel's clock is enabled

/// The interrupt sources: bits 0 to 7 of TC_SR.
const INTERRUPT_BITS: u32 = 0xFF;

/// TC_BCR bit 0: trigger all three channels at once.
const BCR_SYNC: u32 = 1 << 0;

/// The counter is 16 bits wide: it overflows after this value.
const COUNTER_MAX: u32 = 0xFFFF;

/// The clock the emulator's cycles are counted in, in Hz. Changes that
/// firmware makes to the clock through the PMC are not modelled.
const MCK_HZ: u64 = 12_000_000;

/// The slow clock: a 32.768 kHz oscillator.
const SLCK_HZ: u64 = 32_768;

/// How many counts one clock cycle is worth for each TCCLKS setting, as
/// a fraction (numerator, denominator).
fn clock_ratio(tcclks: u32) -> (u64, u64) {
    match tcclks {
        // PCK6, a programmable clock from the PMC. Not modelled: taken
        // to be the full clock rate.
        0 => (1, 1),
        1 => (1, 8),   // MCK / 8
        2 => (1, 32),  // MCK / 32
        3 => (1, 128), // MCK / 128
        4 => (SLCK_HZ, MCK_HZ),
        // 5 to 7 are external clock pins, which do not exist here.
        _ => (0, 1),
    }
}

#[derive(Clone)]
struct Channel {
    mode: u32,
    counter: u32,
    ra: u32,
    rb: u32,
    rc: u32,
    /// The status bits of TC_SR that are set, other than CLKSTA.
    status: u32,
    interrupt_mask: u32,
    /// CLKSTA: the clock is enabled (CLKEN), so a trigger can start it.
    enabled: bool,
    /// The counter has been triggered and is counting.
    running: bool,
    /// Clock cycles not yet turned into counts, times the numerator of
    /// the clock ratio.
    remainder: u64,
}

impl Channel {
    fn new() -> Self {
        Channel {
            mode: 0,
            counter: 0,
            ra: 0,
            rb: 0,
            rc: 0,
            status: 0,
            interrupt_mask: 0,
            enabled: false,
            running: false,
            remainder: 0,
        }
    }

    fn counting(&self) -> bool {
        self.enabled && self.running
    }

    fn waveform(&self) -> bool {
        self.mode & CMR_WAVE != 0
    }

    /// True if reaching RC sends the counter back to 0.
    fn restarts_at_rc(&self) -> bool {
        // The same bit position in both modes, under two names.
        let bit = if self.waveform() {
            CMR_WAVSEL_RC
        } else {
            CMR_CPCTRG
        };
        self.mode & bit != 0 && self.rc != 0
    }

    /// Reset the counter and start it, if the clock is enabled.
    fn trigger(&mut self) {
        if self.enabled {
            self.counter = 0;
            self.remainder = 0;
            self.running = true;
        }
    }

    /// How many counts until something next happens: a match with one
    /// of the compare registers, or the counter going back to 0.
    fn counts_to_next_event(&self) -> u32 {
        // Where the counter goes back to 0 from.
        let wrap = if self.restarts_at_rc() && self.counter < self.rc {
            self.rc
        } else {
            COUNTER_MAX + 1
        };
        let mut distance = wrap - self.counter;
        let compares: &[u32] = if self.waveform() {
            &[self.ra, self.rb, self.rc]
        } else {
            &[self.rc]
        };
        for &value in compares {
            if value > self.counter && value < wrap {
                distance = distance.min(value - self.counter);
            }
        }
        distance
    }

    /// Count `counts` times, setting status bits on the way.
    fn advance(&mut self, mut counts: u64) {
        while counts > 0 && self.counting() {
            let distance = self.counts_to_next_event();
            if counts < distance as u64 {
                self.counter += counts as u32;
                return;
            }
            counts -= distance as u64;
            self.counter += distance;

            if self.waveform() {
                if self.counter == self.ra {
                    self.status |= SR_CPAS;
                }
                if self.counter == self.rb {
                    self.status |= SR_CPBS;
                }
            }
            if self.counter == self.rc {
                self.status |= SR_CPCS;
                if self.restarts_at_rc() {
                    self.counter = 0;
                }
                if self.waveform() {
                    if self.mode & CMR_CPCSTOP != 0 {
                        self.running = false;
                    }
                    if self.mode & CMR_CPCDIS != 0 {
                        self.enabled = false;
                    }
                }
            }
            if self.counter > COUNTER_MAX {
                self.status |= SR_COVFS;
                self.counter = 0;
            }
        }
    }

    /// Let `cycles` clock cycles pass.
    fn tick(&mut self, cycles: u32) {
        if !self.counting() {
            return;
        }
        let (numerator, denominator) = clock_ratio(self.mode & CMR_TCCLKS);
        let total = self.remainder + cycles as u64 * numerator;
        self.remainder = total % denominator;
        self.advance(total / denominator);
    }

    fn interrupt_requested(&self) -> bool {
        self.status & self.interrupt_mask != 0
    }

    /// How many clock cycles from now this channel will next ask for
    /// its interrupt, if it is not asking already.
    fn cycles_until_interrupt(&self) -> Option<u32> {
        if !self.counting() || self.interrupt_mask == 0 {
            return None;
        }
        let (numerator, denominator) = clock_ratio(self.mode & CMR_TCCLKS);
        if numerator == 0 {
            return None;
        }
        // Run a copy forward, one event at a time, until an enabled
        // status bit appears. Every enabled source comes round within
        // one trip of the counter, which is at most four events.
        let mut copy = self.clone();
        copy.status = 0;
        let mut counts = 0u64;
        for _ in 0..8 {
            if !copy.counting() {
                return None;
            }
            let distance = copy.counts_to_next_event() as u64;
            counts += distance;
            copy.advance(distance);
            if copy.status & self.interrupt_mask != 0 {
                // The smallest number of cycles that makes that many counts.
                let needed = counts * denominator - self.remainder;
                return u32::try_from(needed.div_ceil(numerator)).ok();
            }
        }
        None
    }

    fn read(&mut self, offset: u32) -> u32 {
        match offset {
            TC_CMR => self.mode,
            TC_CV => self.counter,
            TC_RA => self.ra,
            TC_RB => self.rb,
            TC_RC => self.rc,
            TC_SR => {
                let clksta = if self.enabled { SR_CLKSTA } else { 0 };
                let value = self.status | clksta;
                self.status = 0; // reading clears the status bits
                value
            }
            TC_IMR => self.interrupt_mask,
            _ => 0,
        }
    }

    fn write(&mut self, offset: u32, value: u32) {
        match offset {
            TC_CCR => {
                if value & CCR_CLKEN != 0 {
                    self.enabled = true;
                }
                // Written together with CLKEN, CLKDIS wins.
                if value & CCR_CLKDIS != 0 {
                    self.enabled = false;
                    self.running = false;
                }
                if value & CCR_SWTRG != 0 {
                    self.trigger();
                }
            }
            TC_CMR => self.mode = value,
            TC_RA => self.ra = value & COUNTER_MAX,
            TC_RB => self.rb = value & COUNTER_MAX,
            TC_RC => self.rc = value & COUNTER_MAX,
            TC_IER => self.interrupt_mask |= value & INTERRUPT_BITS,
            TC_IDR => self.interrupt_mask &= !value,
            _ => {}
        }
    }
}

pub struct Tc {
    channels: [Channel; CHANNELS],
    /// TC_BMR, stored only.
    block_mode: u32,
}

impl Tc {
    pub fn new() -> Self {
        Tc {
            channels: std::array::from_fn(|_| Channel::new()),
            block_mode: 0,
        }
    }

    /// Let `cycles` clock cycles pass on all three channels.
    pub fn tick(&mut self, cycles: u32) {
        for channel in &mut self.channels {
            channel.tick(cycles);
        }
    }

    /// True while channel `channel` (0 to 2) is asking for its interrupt.
    pub fn interrupt_requested(&self, channel: usize) -> bool {
        self.channels[channel].interrupt_requested()
    }

    /// How many clock cycles from now a channel will next ask for its
    /// interrupt, or `None` if none is going to.
    pub fn cycles_until_interrupt(&self) -> Option<u32> {
        self.channels
            .iter()
            .filter_map(Channel::cycles_until_interrupt)
            .min()
    }
}

impl Peripheral for Tc {
    /// All registers are 32 bits wide, so every access is treated as a
    /// whole-word access to the register containing `offset`.
    fn read(&mut self, offset: u32, _size: usize) -> u32 {
        let offset = offset & !3;
        let channel = (offset / CHANNEL_SIZE) as usize;
        if channel < CHANNELS {
            self.channels[channel].read(offset % CHANNEL_SIZE)
        } else if offset == TC_BMR {
            self.block_mode
        } else {
            0
        }
    }

    fn write(&mut self, offset: u32, _size: usize, value: u32) {
        let offset = offset & !3;
        let channel = (offset / CHANNEL_SIZE) as usize;
        if channel < CHANNELS {
            self.channels[channel].write(offset % CHANNEL_SIZE, value);
        } else if offset == TC_BCR {
            if value & BCR_SYNC != 0 {
                for channel in &mut self.channels {
                    channel.trigger();
                }
            }
        } else if offset == TC_BMR {
            self.block_mode = value;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const CLOCK_DIV8: u32 = 1;
    /// Waveform mode, restarting at RC, counting MCK / 8.
    const INTERVAL_MODE: u32 = CMR_WAVE | CMR_WAVSEL_RC | CLOCK_DIV8;

    /// Channel 0 set up as an interval timer and started.
    fn interval_timer(rc: u32) -> Tc {
        let mut tc = Tc::new();
        tc.write(TC_CMR, 4, INTERVAL_MODE);
        tc.write(TC_RC, 4, rc);
        tc.write(TC_CCR, 4, CCR_CLKEN | CCR_SWTRG);
        tc
    }

    #[test]
    fn channel_is_stopped_after_reset() {
        let mut tc = Tc::new();
        tc.tick(1000);
        assert_eq!(tc.read(TC_CV, 4), 0);
        assert_eq!(tc.read(TC_SR, 4), 0);
    }

    #[test]
    fn clock_must_be_enabled_and_then_triggered() {
        let mut tc = Tc::new();
        tc.write(TC_CMR, 4, CLOCK_DIV8);
        tc.write(TC_CCR, 4, CCR_SWTRG); // trigger without the clock
        tc.tick(80);
        assert_eq!(tc.read(TC_CV, 4), 0);
        tc.write(TC_CCR, 4, CCR_CLKEN); // clock without a trigger
        tc.tick(80);
        assert_eq!(tc.read(TC_CV, 4), 0);
        assert_eq!(tc.read(TC_SR, 4), SR_CLKSTA);
        tc.write(TC_CCR, 4, CCR_SWTRG);
        tc.tick(80);
        assert_eq!(tc.read(TC_CV, 4), 10);
    }

    #[test]
    fn counts_at_the_selected_fraction_of_the_clock() {
        for (tcclks, divisor) in [(0, 1), (1, 8), (2, 32), (3, 128)] {
            let mut tc = Tc::new();
            tc.write(TC_CMR, 4, tcclks);
            tc.write(TC_CCR, 4, CCR_CLKEN | CCR_SWTRG);
            // In uneven steps, as instructions of different cost give.
            for cycles in [1, 3, 2, 12, 110] {
                tc.tick(cycles);
            }
            assert_eq!(tc.read(TC_CV, 4), 128 / divisor, "TCCLKS = {}", tcclks);
        }
    }

    #[test]
    fn slow_clock_counts_32768_times_a_second() {
        let mut tc = Tc::new();
        tc.write(TC_CMR, 4, 4); // SLCK
        tc.write(TC_CCR, 4, CCR_CLKEN | CCR_SWTRG);
        // One tenth of a second at 12 MHz, in pieces.
        for _ in 0..1200 {
            tc.tick(1000);
        }
        assert_eq!(tc.read(TC_CV, 4), 3276);
    }

    #[test]
    fn interval_timer_restarts_at_rc_and_sets_cpcs() {
        let mut tc = interval_timer(10);
        tc.tick(79); // 9 counts and most of the tenth
        assert_eq!(tc.read(TC_CV, 4), 9);
        assert_eq!(tc.read(TC_SR, 4), SR_CLKSTA);
        tc.tick(1);
        assert_eq!(tc.read(TC_CV, 4), 0); // reached 10, back to 0
        assert_eq!(tc.read(TC_SR, 4), SR_CPCS | SR_CLKSTA);
        // Reading the status cleared it.
        assert_eq!(tc.read(TC_SR, 4), SR_CLKSTA);
        // The period is exactly RC counts.
        tc.tick(80);
        assert_eq!(tc.read(TC_SR, 4), SR_CPCS | SR_CLKSTA);
    }

    #[test]
    fn interrupt_is_requested_until_the_status_is_read() {
        let mut tc = interval_timer(10);
        tc.tick(80);
        assert!(!tc.interrupt_requested(0)); // CPCS set but not enabled
        tc.read(TC_SR, 4);

        tc.write(TC_IER, 4, SR_CPCS);
        assert_eq!(tc.read(TC_IMR, 4), SR_CPCS);
        tc.tick(79);
        assert!(!tc.interrupt_requested(0));
        tc.tick(1);
        assert!(tc.interrupt_requested(0));
        assert!(!tc.interrupt_requested(1));
        tc.tick(40); // still asking: nobody has dealt with it
        assert!(tc.interrupt_requested(0));
        tc.read(TC_SR, 4);
        assert!(!tc.interrupt_requested(0));

        tc.write(TC_IDR, 4, SR_CPCS);
        tc.tick(80);
        assert!(!tc.interrupt_requested(0));
    }

    #[test]
    fn a_long_step_passes_several_periods() {
        let mut tc = interval_timer(10);
        tc.tick(8 * (35)); // 35 counts: three periods and 5 more
        assert_eq!(tc.read(TC_CV, 4), 5);
        assert_eq!(tc.read(TC_SR, 4), SR_CPCS | SR_CLKSTA);
    }

    #[test]
    fn free_running_counter_overflows_after_65536_counts() {
        let mut tc = Tc::new();
        tc.write(TC_CMR, 4, CMR_WAVE); // waveform, no restart at RC, PCK6
        tc.write(TC_CCR, 4, CCR_CLKEN | CCR_SWTRG);
        tc.tick(65_535);
        assert_eq!(tc.read(TC_CV, 4), 0xFFFF);
        assert_eq!(tc.read(TC_SR, 4) & SR_COVFS, 0);
        tc.tick(1);
        assert_eq!(tc.read(TC_CV, 4), 0);
        assert_ne!(tc.read(TC_SR, 4) & SR_COVFS, 0);
    }

    #[test]
    fn ra_and_rb_matches_are_noted_in_waveform_mode() {
        let mut tc = Tc::new();
        tc.write(TC_CMR, 4, CMR_WAVE | CMR_WAVSEL_RC); // PCK6: one count per cycle
        tc.write(TC_RA, 4, 3);
        tc.write(TC_RB, 4, 6);
        tc.write(TC_RC, 4, 10);
        tc.write(TC_CCR, 4, CCR_CLKEN | CCR_SWTRG);
        tc.tick(3);
        assert_eq!(tc.read(TC_SR, 4), SR_CPAS | SR_CLKSTA);
        tc.tick(3);
        assert_eq!(tc.read(TC_SR, 4), SR_CPBS | SR_CLKSTA);
        tc.tick(4);
        assert_eq!(tc.read(TC_SR, 4), SR_CPCS | SR_CLKSTA);
    }

    #[test]
    fn capture_mode_can_restart_at_rc_too() {
        let mut tc = Tc::new();
        tc.write(TC_CMR, 4, CMR_CPCTRG); // WAVE clear
        tc.write(TC_RA, 4, 3); // RA is not a compare register here
        tc.write(TC_RC, 4, 10);
        tc.write(TC_CCR, 4, CCR_CLKEN | CCR_SWTRG);
        tc.tick(25);
        assert_eq!(tc.read(TC_CV, 4), 5);
        assert_eq!(tc.read(TC_SR, 4), SR_CPCS | SR_CLKSTA);
    }

    #[test]
    fn one_shot_stops_at_rc() {
        let mut tc = Tc::new();
        tc.write(TC_CMR, 4, CMR_WAVE | CMR_WAVSEL_RC | CMR_CPCSTOP);
        tc.write(TC_RC, 4, 10);
        tc.write(TC_CCR, 4, CCR_CLKEN | CCR_SWTRG);
        tc.tick(100);
        assert_eq!(tc.read(TC_CV, 4), 0);
        assert_eq!(tc.read(TC_SR, 4), SR_CPCS | SR_CLKSTA); // once, clock still enabled
        tc.tick(100);
        assert_eq!(tc.read(TC_SR, 4), SR_CLKSTA);
        // A new trigger runs it again.
        tc.write(TC_CCR, 4, CCR_SWTRG);
        tc.tick(10);
        assert_eq!(tc.read(TC_SR, 4), SR_CPCS | SR_CLKSTA);

        // CPCDIS switches the clock off as well.
        let mut tc = Tc::new();
        tc.write(TC_CMR, 4, CMR_WAVE | CMR_WAVSEL_RC | CMR_CPCDIS);
        tc.write(TC_RC, 4, 10);
        tc.write(TC_CCR, 4, CCR_CLKEN | CCR_SWTRG);
        tc.tick(100);
        assert_eq!(tc.read(TC_SR, 4), SR_CPCS);
    }

    #[test]
    fn disabling_the_clock_stops_the_count() {
        let mut tc = interval_timer(1000);
        tc.tick(80);
        tc.write(TC_CCR, 4, CCR_CLKDIS);
        tc.tick(800);
        assert_eq!(tc.read(TC_CV, 4), 10);
        assert_eq!(tc.read(TC_SR, 4), 0);
    }

    #[test]
    fn the_three_channels_are_independent() {
        let mut tc = Tc::new();
        tc.write(CHANNEL_SIZE + TC_CMR, 4, 0); // channel 1, PCK6
        tc.write(CHANNEL_SIZE + TC_CCR, 4, CCR_CLKEN | CCR_SWTRG);
        tc.tick(7);
        assert_eq!(tc.read(TC_CV, 4), 0);
        assert_eq!(tc.read(CHANNEL_SIZE + TC_CV, 4), 7);
        assert_eq!(tc.read(2 * CHANNEL_SIZE + TC_CV, 4), 0);
    }

    #[test]
    fn sync_triggers_every_enabled_channel() {
        let mut tc = Tc::new();
        for channel in 0..2 {
            tc.write(channel * CHANNEL_SIZE + TC_CCR, 4, CCR_CLKEN);
        }
        tc.write(TC_BCR, 4, BCR_SYNC);
        tc.tick(5);
        assert_eq!(tc.read(TC_CV, 4), 5);
        assert_eq!(tc.read(CHANNEL_SIZE + TC_CV, 4), 5);
        assert_eq!(tc.read(2 * CHANNEL_SIZE + TC_CV, 4), 0); // clock not enabled
    }

    #[test]
    fn predicted_time_to_the_interrupt_is_when_it_is_requested() {
        let mut tc = interval_timer(10);
        assert_eq!(tc.cycles_until_interrupt(), None); // nothing enabled
        tc.write(TC_IER, 4, SR_CPCS);
        for _ in 0..3 {
            let cycles = tc.cycles_until_interrupt().unwrap();
            tc.tick(cycles - 1);
            assert!(!tc.interrupt_requested(0));
            tc.tick(1);
            assert!(tc.interrupt_requested(0));
            tc.read(TC_SR, 4);
        }
        // From partway through a period, and through a count.
        tc.tick(27);
        assert_eq!(tc.cycles_until_interrupt(), Some(53));
    }

    #[test]
    fn prediction_skips_events_that_are_not_enabled() {
        let mut tc = Tc::new();
        tc.write(TC_CMR, 4, CMR_WAVE | CMR_WAVSEL_RC);
        tc.write(TC_RA, 4, 3);
        tc.write(TC_RC, 4, 10);
        tc.write(TC_CCR, 4, CCR_CLKEN | CCR_SWTRG);
        tc.write(TC_IER, 4, SR_CPCS);
        assert_eq!(tc.cycles_until_interrupt(), Some(10)); // not 3
        tc.write(TC_IDR, 4, SR_CPCS);
        tc.write(TC_IER, 4, SR_COVFS); // never happens: restarts at RC
        assert_eq!(tc.cycles_until_interrupt(), None);
    }

    #[test]
    fn compare_registers_are_16_bits_wide() {
        let mut tc = Tc::new();
        tc.write(TC_RC, 4, 0xFFFF_FFFF);
        assert_eq!(tc.read(TC_RC, 4), 0xFFFF);
    }
}
