//! A model of the Bosch BNO055, the sensor on the BNO055 Xplained Pro
//! card: a gyro, an accelerometer and a magnetometer in one chip, read
//! over I2C.
//!
//! What is modelled is the gyro. A gyro measures how fast the chip is
//! turning around each of three axes (X, Y, Z), in degrees per second.
//! The turning itself comes from a `Motion`: a constant rate, a
//! generated pattern, or samples from a file.
//!
//! The chip is a table of numbered one-byte registers, in two pages
//! selected by the PAGE_ID register. Firmware:
//!
//! 1. reads CHIP_ID to check that the chip is there (0xA0);
//! 2. writes OPR_MODE to leave CONFIGMODE, where the chip starts and
//!    measures nothing, for a mode that has the gyro on;
//! 3. reads the six gyro data registers: X, Y and Z as signed 16-bit
//!    numbers, low byte first, in units of 1/16 degree per second.
//!
//! Reading several data registers in one transfer gives values that
//! belong together: they are taken at the start of the transfer (the
//! datasheet calls this data register shadowing, section 3.7).
//!
//! Not modelled: the accelerometer, the magnetometer and the fused
//! orientation outputs (their data registers read 0); calibration
//! (always reported complete); the sensor's interrupts; the start-up
//! and mode-switching delays (the chip is ready at once); and the rule
//! that most settings can only be changed in CONFIGMODE.
//!
//! Register numbers, reset values and units are from the datasheet,
//! `reference/bno055/BST-BNO055-DS000-18.pdf`, chapter 4.

use crate::i2c::I2cDevice;

/// The card's I2C address. (0x28 if the chip's COM3 pin is tied low.)
pub const ADDRESS: u8 = 0x29;

/// The clock the emulator's cycles are counted in, in Hz.
const CYCLES_PER_SECOND: f64 = 12_000_000.0;

// Page 0 registers.
const CHIP_ID: u8 = 0x00;
const ACC_ID: u8 = 0x01;
const MAG_ID: u8 = 0x02;
const GYR_ID: u8 = 0x03;
const SW_REV_ID_LSB: u8 = 0x04;
const SW_REV_ID_MSB: u8 = 0x05;
const BL_REV_ID: u8 = 0x06;
const PAGE_ID: u8 = 0x07; // on both pages
const GYR_DATA_X_LSB: u8 = 0x14; // X, Y, Z: six bytes up to 0x19
const GYR_DATA_Z_MSB: u8 = 0x19;
const TEMP: u8 = 0x34;
const CALIB_STAT: u8 = 0x35;
const ST_RESULT: u8 = 0x36;
const SYS_STATUS: u8 = 0x39;
const UNIT_SEL: u8 = 0x3B;
const OPR_MODE: u8 = 0x3D;
const PWR_MODE: u8 = 0x3E;
const SYS_TRIGGER: u8 = 0x3F;
const AXIS_MAP_CONFIG: u8 = 0x41;

// Page 1 registers: settings of the individual sensors.
const ACC_CONFIG: u8 = 0x08;
const MAG_CONFIG: u8 = 0x09;
const GYR_CONFIG_0: u8 = 0x0A;

/// Registers are numbered 0x00 to 0x7F on each page.
const REGISTERS: usize = 0x80;

/// SYS_TRIGGER bit 5 (RST_SYS): reset the chip.
const SYS_TRIGGER_RST_SYS: u8 = 1 << 5;

/// UNIT_SEL bit 1 (GYR_Unit): 0 = degrees per second, 1 = radians per
/// second.
const UNIT_SEL_GYR_RPS: u8 = 1 << 1;

/// Raw units per degree per second, and per radian per second.
const LSB_PER_DPS: f64 = 16.0;
const LSB_PER_RPS: f64 = 900.0;

/// OPR_MODE: the low four bits choose the operating mode. 0 is
/// CONFIGMODE; 1 to 7 are plain sensor modes; 8 to 12 are fusion modes,
/// where the chip also works out an orientation.
const OPR_MODE_MASK: u8 = 0x0F;
const LAST_PLAIN_MODE: u8 = 7;
const LAST_MODE: u8 = 12;

/// True if the gyro is switched on in operating mode `mode`: GYROONLY
/// (3), ACCGYRO (5), MAGGYRO (6), AMG (7), IMU (8), NDOF_FMC_OFF (11)
/// and NDOF (12).
fn gyro_on(mode: u8) -> bool {
    matches!(mode, 3 | 5 | 6 | 7 | 8 | 11 | 12)
}

/// Where the gyro's readings come from: how fast the imaginary board is
/// turning around X, Y and Z, in degrees per second, at any moment.
#[derive(Debug)]
pub enum Motion {
    /// Always the same rates.
    Constant([f64; 3]),
    /// A smooth repeating pattern: each axis swings back and forth at
    /// its own speed. X goes to +/-90 once a second, Y to +/-45 every
    /// two seconds, Z to +/-10 every four seconds.
    Pattern,
    /// Samples as (time in seconds, rates), in time order. A sample
    /// holds until the next one's time.
    Recorded(Vec<(f64, [f64; 3])>),
}

impl Motion {
    /// The rates in degrees per second, `seconds` after reset.
    pub fn rates_at(&self, seconds: f64) -> [f64; 3] {
        match self {
            Motion::Constant(rates) => *rates,
            Motion::Pattern => {
                let turn = 2.0 * std::f64::consts::PI * seconds;
                [
                    90.0 * turn.sin(),
                    45.0 * (turn / 2.0).sin(),
                    10.0 * (turn / 4.0).sin(),
                ]
            }
            Motion::Recorded(samples) => {
                // The last sample at or before `seconds`; before the
                // first sample, the first one.
                let index = samples.partition_point(|&(time, _)| time <= seconds);
                match samples.get(index.saturating_sub(1)) {
                    Some(&(_, rates)) => rates,
                    None => [0.0; 3],
                }
            }
        }
    }

    /// Read samples from text: one per line, as four numbers, the time
    /// in seconds and then the X, Y and Z rates in degrees per second.
    /// The numbers may be separated by spaces, tabs or commas. Blank
    /// lines and lines starting with `#` are ignored.
    pub fn parse_samples(text: &str) -> Result<Motion, String> {
        let mut samples = Vec::new();
        for (number, line) in text.lines().enumerate() {
            let line = line.trim();
            if line.is_empty() || line.starts_with('#') {
                continue;
            }
            let fields: Result<Vec<f64>, _> = line
                .split(|c: char| c == ',' || c.is_whitespace())
                .filter(|field| !field.is_empty())
                .map(str::parse)
                .collect();
            match fields.as_deref() {
                Ok(&[time, x, y, z]) => samples.push((time, [x, y, z])),
                _ => {
                    return Err(format!(
                        "line {}: expected 'seconds x y z', found '{}'",
                        number + 1,
                        line
                    ));
                }
            }
        }
        if samples.is_empty() {
            return Err("no samples".to_string());
        }
        samples.sort_by(|a, b| a.0.total_cmp(&b.0));
        Ok(Motion::Recorded(samples))
    }

    /// Understand the text of a `--gyro=` option:
    ///
    /// - `1.0,-2.0,0.5`: constant rates for X, Y and Z;
    /// - `pattern`: the generated pattern;
    /// - `file:PATH`: samples from a file (see `parse_samples`).
    pub fn parse_option(text: &str) -> Result<Motion, String> {
        if text == "pattern" {
            return Ok(Motion::Pattern);
        }
        if let Some(path) = text.strip_prefix("file:") {
            let contents = std::fs::read_to_string(path)
                .map_err(|error| format!("cannot read gyro file '{}': {}", path, error))?;
            return Motion::parse_samples(&contents)
                .map_err(|error| format!("gyro file '{}': {}", path, error));
        }
        let rates: Result<Vec<f64>, _> =
            text.split(',').map(|field| field.trim().parse()).collect();
        match rates.as_deref() {
            Ok(&[x, y, z]) => Ok(Motion::Constant([x, y, z])),
            _ => Err(format!(
                "bad gyro option '{}': expected X,Y,Z (degrees per second), 'pattern' or 'file:PATH'",
                text
            )),
        }
    }
}

pub struct Bno055 {
    /// The stored registers of page 0 and page 1.
    pages: [[u8; REGISTERS]; 2],
    /// The register the next byte is read from or written to.
    position: u8,
    /// True after a START, until a byte is written: that byte will be a
    /// register number, not data.
    expecting_register: bool,
    motion: Motion,
    /// Clock cycles since reset.
    cycles: u64,
    /// The gyro data registers as they were when the current transfer
    /// began.
    gyro_data: [u8; 6],
}

impl Bno055 {
    pub fn new(motion: Motion) -> Self {
        let mut sensor = Bno055 {
            pages: [[0; REGISTERS]; 2],
            position: 0,
            expecting_register: false,
            motion,
            cycles: 0,
            gyro_data: [0; 6],
        };
        sensor.reset_registers();
        sensor
    }

    /// Put every register back to its value after power-on.
    fn reset_registers(&mut self) {
        self.pages = [[0; REGISTERS]; 2];
        let page0 = &mut self.pages[0];
        page0[CHIP_ID as usize] = 0xA0;
        page0[ACC_ID as usize] = 0xFB;
        page0[MAG_ID as usize] = 0x32;
        page0[GYR_ID as usize] = 0x0F;
        // Software version 3.11.
        page0[SW_REV_ID_LSB as usize] = 0x11;
        page0[SW_REV_ID_MSB as usize] = 0x03;
        page0[BL_REV_ID as usize] = 0x15;
        page0[ST_RESULT as usize] = 0x0F; // all four self-tests passed
        page0[UNIT_SEL as usize] = 0x80;
        page0[OPR_MODE as usize] = 0x10; // low four bits 0: CONFIGMODE
        page0[AXIS_MAP_CONFIG as usize] = 0x24;
        let page1 = &mut self.pages[1];
        page1[ACC_CONFIG as usize] = 0x0D;
        page1[MAG_CONFIG as usize] = 0x0B;
        page1[GYR_CONFIG_0 as usize] = 0x38;
    }

    fn page(&self) -> usize {
        (self.pages[0][PAGE_ID as usize] & 1) as usize
    }

    fn mode(&self) -> u8 {
        self.pages[0][OPR_MODE as usize] & OPR_MODE_MASK
    }

    /// Take a reading: work out the six gyro data bytes for this
    /// moment.
    fn sample_gyro(&mut self) {
        self.gyro_data = [0; 6];
        if !gyro_on(self.mode()) {
            return;
        }
        let seconds = self.cycles as f64 / CYCLES_PER_SECOND;
        let rates = self.motion.rates_at(seconds);
        let radians = self.pages[0][UNIT_SEL as usize] & UNIT_SEL_GYR_RPS != 0;
        for (axis, &rate) in rates.iter().enumerate() {
            let scaled = if radians {
                rate.to_radians() * LSB_PER_RPS
            } else {
                rate * LSB_PER_DPS
            };
            // A rate too large for 16 bits reads as the largest value.
            let raw = scaled.round().clamp(i16::MIN as f64, i16::MAX as f64) as i16;
            self.gyro_data[2 * axis..2 * axis + 2].copy_from_slice(&raw.to_le_bytes());
        }
    }

    fn read_register(&self, register: u8) -> u8 {
        if register == PAGE_ID {
            return self.pages[0][PAGE_ID as usize];
        }
        if self.page() == 1 {
            return self.pages[1][register as usize];
        }
        let mode = self.mode();
        match register {
            GYR_DATA_X_LSB..=GYR_DATA_Z_MSB => self.gyro_data[(register - GYR_DATA_X_LSB) as usize],
            // The chip's own temperature, in degrees Celsius.
            TEMP if mode != 0 => 25,
            // Every part reports "fully calibrated".
            CALIB_STAT => 0xFF,
            // 0 idle, 6 running without fusion, 5 fusion running.
            SYS_STATUS => match mode {
                0 => 0,
                1..=LAST_PLAIN_MODE => 6,
                _ => 5,
            },
            _ => self.pages[0][register as usize],
        }
    }

    fn write_register(&mut self, register: u8, value: u8) {
        if register == PAGE_ID {
            self.pages[0][PAGE_ID as usize] = value & 1;
            return;
        }
        if self.page() == 1 {
            self.pages[1][register as usize] = value;
            return;
        }
        match register {
            OPR_MODE => {
                // A mode that does not exist is ignored.
                let mode = value & OPR_MODE_MASK;
                if mode <= LAST_MODE {
                    self.pages[0][OPR_MODE as usize] = 0x10 | mode;
                }
            }
            SYS_TRIGGER => {
                if value & SYS_TRIGGER_RST_SYS != 0 {
                    self.reset_registers();
                } else {
                    // The other bits (clock source, self-test,
                    // interrupt reset) act and read back as 0, except
                    // the clock source, bit 7, which is kept.
                    self.pages[0][SYS_TRIGGER as usize] = value & 0x80;
                }
            }
            UNIT_SEL | PWR_MODE => self.pages[0][register as usize] = value,
            // Identification, data and status registers are read-only.
            0x00..=0x3A => {}
            // The rest are settings: axis mapping, calibration offsets
            // and so on. Stored and read back.
            _ => self.pages[0][register as usize] = value,
        }
    }

    /// Move on to the next register, going round at the end.
    fn advance(&mut self) {
        self.position = (self.position + 1) % REGISTERS as u8;
    }
}

impl I2cDevice for Bno055 {
    fn address(&self) -> u8 {
        ADDRESS
    }

    fn start(&mut self) {
        self.expecting_register = true;
        self.sample_gyro();
    }

    fn write(&mut self, byte: u8) -> bool {
        if self.expecting_register {
            if byte as usize >= REGISTERS {
                return false;
            }
            self.position = byte;
            self.expecting_register = false;
        } else {
            self.write_register(self.position, byte);
            self.advance();
        }
        true
    }

    fn read(&mut self) -> u8 {
        let byte = self.read_register(self.position);
        self.advance();
        byte
    }

    fn tick(&mut self, cycles: u32) {
        self.cycles += cycles as u64;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const GYROONLY: u8 = 0x03;
    const NDOF: u8 = 0x0C;

    fn sensor() -> Bno055 {
        Bno055::new(Motion::Constant([1.0, -2.0, 0.5]))
    }

    fn write(sensor: &mut Bno055, register: u8, value: u8) {
        sensor.start();
        assert!(sensor.write(register));
        assert!(sensor.write(value));
        sensor.stop();
    }

    fn read(sensor: &mut Bno055, register: u8, count: usize) -> Vec<u8> {
        sensor.start();
        assert!(sensor.write(register));
        sensor.start();
        let bytes = (0..count).map(|_| sensor.read()).collect();
        sensor.stop();
        bytes
    }

    /// The three gyro axes as signed raw values.
    fn gyro(sensor: &mut Bno055) -> [i16; 3] {
        let bytes = read(sensor, GYR_DATA_X_LSB, 6);
        [0, 2, 4].map(|i| i16::from_le_bytes([bytes[i], bytes[i + 1]]))
    }

    #[test]
    fn answers_at_the_cards_address() {
        assert_eq!(sensor().address(), 0x29);
    }

    #[test]
    fn identification_registers_match_the_datasheet() {
        let mut sensor = sensor();
        // CHIP_ID, ACC_ID, MAG_ID, GYR_ID, software 3.11, bootloader.
        assert_eq!(
            read(&mut sensor, CHIP_ID, 7),
            [0xA0, 0xFB, 0x32, 0x0F, 0x11, 0x03, 0x15]
        );
    }

    #[test]
    fn reset_values_match_the_datasheet() {
        let mut sensor = sensor();
        assert_eq!(read(&mut sensor, PAGE_ID, 1), [0x00]);
        assert_eq!(read(&mut sensor, ST_RESULT, 1), [0x0F]);
        assert_eq!(read(&mut sensor, UNIT_SEL, 1), [0x80]);
        assert_eq!(read(&mut sensor, OPR_MODE, 1), [0x10]);
        assert_eq!(read(&mut sensor, PWR_MODE, 2), [0x00, 0x00]); // and SYS_TRIGGER
        assert_eq!(read(&mut sensor, SYS_STATUS, 1), [0]); // idle
    }

    #[test]
    fn identification_registers_cannot_be_written() {
        let mut sensor = sensor();
        write(&mut sensor, CHIP_ID, 0x55);
        assert_eq!(read(&mut sensor, CHIP_ID, 1), [0xA0]);
    }

    #[test]
    fn no_gyro_data_in_configmode() {
        let mut sensor = sensor();
        assert_eq!(gyro(&mut sensor), [0, 0, 0]);
    }

    #[test]
    fn gyro_data_is_16_units_per_degree_per_second() {
        let mut sensor = sensor(); // 1.0, -2.0, 0.5 degrees per second
        write(&mut sensor, OPR_MODE, GYROONLY);
        assert_eq!(gyro(&mut sensor), [16, -32, 8]);
        // Low byte first, negative values in two's complement.
        assert_eq!(read(&mut sensor, GYR_DATA_X_LSB, 4), [16, 0, 0xE0, 0xFF]);
    }

    #[test]
    fn gyro_is_on_in_the_modes_that_include_it() {
        for mode in 0..=12u8 {
            let mut sensor = sensor();
            write(&mut sensor, OPR_MODE, mode);
            let expected = matches!(mode, 3 | 5 | 6 | 7 | 8 | 11 | 12);
            assert_eq!(gyro(&mut sensor) != [0, 0, 0], expected, "mode {}", mode);
        }
    }

    #[test]
    fn mode_reads_back_and_shows_in_the_status() {
        let mut sensor = sensor();
        write(&mut sensor, OPR_MODE, GYROONLY);
        assert_eq!(read(&mut sensor, OPR_MODE, 1)[0] & 0x0F, GYROONLY);
        assert_eq!(read(&mut sensor, SYS_STATUS, 1), [6]); // running, no fusion
        write(&mut sensor, OPR_MODE, NDOF);
        assert_eq!(read(&mut sensor, SYS_STATUS, 1), [5]); // fusion running
        // A mode number that does not exist is ignored.
        write(&mut sensor, OPR_MODE, 0x0F);
        assert_eq!(read(&mut sensor, OPR_MODE, 1)[0] & 0x0F, NDOF);
    }

    #[test]
    fn radians_per_second_can_be_selected() {
        let mut sensor = Bno055::new(Motion::Constant([180.0, -90.0, 0.0]));
        write(&mut sensor, OPR_MODE, GYROONLY);
        write(&mut sensor, UNIT_SEL, 0x80 | UNIT_SEL_GYR_RPS);
        // 180 degrees is pi radians; 900 units per radian per second.
        let pi_raw = (std::f64::consts::PI * 900.0).round() as i16;
        assert_eq!(gyro(&mut sensor), [pi_raw, -(pi_raw + 1) / 2, 0]);
    }

    #[test]
    fn rates_too_large_for_16_bits_read_as_the_largest_value() {
        let mut sensor = Bno055::new(Motion::Constant([5000.0, -5000.0, 0.0]));
        write(&mut sensor, OPR_MODE, GYROONLY);
        assert_eq!(gyro(&mut sensor), [i16::MAX, i16::MIN, 0]);
    }

    #[test]
    fn values_read_in_one_transfer_belong_together() {
        let mut sensor = Bno055::new(Motion::Pattern);
        write(&mut sensor, OPR_MODE, GYROONLY);
        sensor.tick(3_000_000); // a quarter of a second: X at its peak
        sensor.start();
        sensor.write(GYR_DATA_X_LSB);
        sensor.start();
        let low = sensor.read();
        // Time passes in the middle of the transfer.
        sensor.tick(3_000_000);
        let high = sensor.read();
        sensor.stop();
        assert_eq!(i16::from_le_bytes([low, high]), 90 * 16);
        // A new transfer sees the new moment: X back at zero.
        assert_eq!(gyro(&mut sensor)[0], 0);
    }

    #[test]
    fn system_reset_restores_every_register() {
        let mut sensor = sensor();
        write(&mut sensor, OPR_MODE, GYROONLY);
        write(&mut sensor, UNIT_SEL, 0x82);
        write(&mut sensor, SYS_TRIGGER, SYS_TRIGGER_RST_SYS);
        assert_eq!(read(&mut sensor, OPR_MODE, 1), [0x10]);
        assert_eq!(read(&mut sensor, UNIT_SEL, 1), [0x80]);
        assert_eq!(read(&mut sensor, SYS_TRIGGER, 1), [0x00]);
        assert_eq!(gyro(&mut sensor), [0, 0, 0]);
    }

    #[test]
    fn second_page_holds_the_sensor_settings() {
        let mut sensor = sensor();
        write(&mut sensor, PAGE_ID, 1);
        assert_eq!(read(&mut sensor, PAGE_ID, 1), [1]); // visible on both pages
        assert_eq!(read(&mut sensor, ACC_CONFIG, 3), [0x0D, 0x0B, 0x38]);
        write(&mut sensor, GYR_CONFIG_0, 0x3B);
        assert_eq!(read(&mut sensor, GYR_CONFIG_0, 1), [0x3B]);
        // Register 0 on this page is not CHIP_ID.
        assert_eq!(read(&mut sensor, 0x00, 1), [0x00]);
        write(&mut sensor, PAGE_ID, 0);
        assert_eq!(read(&mut sensor, CHIP_ID, 1), [0xA0]);
        // Page 0's register 0x0A was not touched by the page 1 write.
        assert_eq!(read(&mut sensor, 0x0A, 1), [0x00]);
    }

    #[test]
    fn status_registers_report_all_good() {
        let mut sensor = sensor();
        assert_eq!(read(&mut sensor, CALIB_STAT, 1), [0xFF]);
        write(&mut sensor, OPR_MODE, GYROONLY);
        assert_eq!(read(&mut sensor, TEMP, 1), [25]);
        assert_eq!(read(&mut sensor, 0x3A, 1), [0]); // SYS_ERR: no error
    }

    #[test]
    fn register_number_out_of_range_is_refused() {
        let mut sensor = sensor();
        sensor.start();
        assert!(!sensor.write(0x80));
    }

    #[test]
    fn pattern_swings_each_axis_at_its_own_speed() {
        let near = |a: f64, b: f64| (a - b).abs() < 1e-9;
        let motion = Motion::Pattern;
        assert_eq!(motion.rates_at(0.0), [0.0, 0.0, 0.0]);
        let [x, y, z] = motion.rates_at(0.25); // X at its peak
        assert!(near(x, 90.0) && y > 0.0 && z > 0.0);
        let [x, y, _] = motion.rates_at(0.5); // X back at zero, Y at its peak
        assert!(near(x, 0.0) && near(y, 45.0));
        let [x, _, z] = motion.rates_at(1.0);
        assert!(near(x, 0.0) && near(z, 10.0));
        assert!(near(motion.rates_at(0.75)[0], -90.0));
    }

    #[test]
    fn recorded_samples_hold_until_the_next_one() {
        let motion =
            Motion::parse_samples("# time x y z\n0.0 1 2 3\n\n1.0, 4, 5, 6\n2.5\t-7\t-8\t-9\n")
                .unwrap();
        assert_eq!(motion.rates_at(0.0), [1.0, 2.0, 3.0]);
        assert_eq!(motion.rates_at(0.99), [1.0, 2.0, 3.0]);
        assert_eq!(motion.rates_at(1.0), [4.0, 5.0, 6.0]);
        assert_eq!(motion.rates_at(2.0), [4.0, 5.0, 6.0]);
        assert_eq!(motion.rates_at(100.0), [-7.0, -8.0, -9.0]);
    }

    #[test]
    fn recorded_samples_are_put_in_time_order() {
        let motion = Motion::parse_samples("2 20 0 0\n1 10 0 0\n").unwrap();
        assert_eq!(motion.rates_at(0.5)[0], 10.0); // before the first: the first
        assert_eq!(motion.rates_at(1.5)[0], 10.0);
        assert_eq!(motion.rates_at(2.5)[0], 20.0);
    }

    #[test]
    fn bad_sample_text_is_reported_with_its_line() {
        let error = Motion::parse_samples("0 1 2 3\n1 2 3\n").unwrap_err();
        assert!(error.starts_with("line 2"), "{}", error);
        assert!(Motion::parse_samples("0 1 2 x\n").is_err());
        assert!(Motion::parse_samples("# nothing\n").is_err());
    }

    #[test]
    fn gyro_option_text_is_understood() {
        assert!(matches!(
            Motion::parse_option("1.5,-2,0"),
            Ok(Motion::Constant(rates)) if rates == [1.5, -2.0, 0.0]
        ));
        assert!(matches!(
            Motion::parse_option("pattern"),
            Ok(Motion::Pattern)
        ));
        assert!(Motion::parse_option("1,2").is_err());
        assert!(Motion::parse_option("fast").is_err());
        assert!(Motion::parse_option("file:/no/such/file").is_err());
    }

    #[test]
    fn sensor_follows_the_motion_over_time() {
        let mut sensor = Bno055::new(Motion::parse_samples("0 10 0 0\n1 20 0 0\n").unwrap());
        write(&mut sensor, OPR_MODE, GYROONLY);
        assert_eq!(gyro(&mut sensor)[0], 160);
        sensor.tick(12_000_000); // one second
        assert_eq!(gyro(&mut sensor)[0], 320);
    }
}
