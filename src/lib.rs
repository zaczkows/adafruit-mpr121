//! Rust version of access to adafruit MPR121 capacitive touch sensor HAT under Linux.
//! Completely inspired by
//! [this](https://github.com/adafruit/Adafruit_CircuitPython_MPR121)
//! and [that](https://github.com/adafruit/Adafruit_MPR121) original adafruit repos.
//! It only works with 12 input touch, numbered from 0 to 11 [product info](https://www.adafruit.com/product/2340).
//!
//! Default initialization:
//! ```rust,no_run
//! use adafruit_mpr121::Mpr121;
//! let mut touch_sensor = Mpr121::new_default(1).expect("Failed to initialize sensor");
//! let status = touch_sensor.touch_status().unwrap();
//! println!("Touch status: {}", status);

use i2cdev::{
    core::*,
    linux::{LinuxI2CDevice, LinuxI2CError},
};

/// Manages adafruit MPR121 capacitive sensor HAT I2C device.
pub struct Mpr121 {
    dev: LinuxI2CDevice,
}

/// Basic error type, mostly I2C errors
pub type Mpr121Error = LinuxI2CError;

/// Touch status for all pins
#[derive(Debug)]
pub struct Mpr121TouchStatus {
    status: u16,
}

/// Convenient iterator for pins in `Mpr121TouchStatus`
pub struct Mpr121TouchStatusIterator<'a> {
    status: &'a Mpr121TouchStatus,
    count: u8,
}

/// Default I2C address for MPR121
pub const MPR121_I2CADDR_DEFAULT: u16 = 0x5A;

/// Default touch threshold set for MPR121
pub const MPR121_TOUCH_THRESHOLD_DEFAULT: u8 = 12;

/// Default release threshold set for MPR121
pub const MPR121_RELEASE_THRESHOLD_DEFAULT: u8 = 6;

impl Mpr121 {
    // Register addresses.
    const REG_TOUCHSTATUS_L: u8 = 0x00;
    // const REG_TOUCHSTATUS_H: u8 = 0x01;
    // const REG_FILTDATA_0L: u8 = 0x04;
    // const REG_FILTDATA_0H: u8 = 0x05;
    // const REG_BASELINE_0: u8 = 0x1E;
    const REG_MHDR: u8 = 0x2B;
    const REG_NHDR: u8 = 0x2C;
    const REG_NCLR: u8 = 0x2D;
    const REG_FDLR: u8 = 0x2E;
    const REG_MHDF: u8 = 0x2F;
    const REG_NHDF: u8 = 0x30;
    const REG_NCLF: u8 = 0x31;
    const REG_FDLF: u8 = 0x32;
    const REG_NHDT: u8 = 0x33;
    const REG_NCLT: u8 = 0x34;
    const REG_FDLT: u8 = 0x35;
    const REG_TOUCHTH_0: u8 = 0x41;
    const REG_RELEASETH_0: u8 = 0x42;
    const REG_DEBOUNCE: u8 = 0x5B;
    const REG_CONFIG1: u8 = 0x5C;
    const REG_CONFIG2: u8 = 0x5D;
    // const REG_CHARGECURR_0: u8 = 0x5F;
    // const REG_CHARGETIME_1: u8 = 0x6C;
    const REG_ECR: u8 = 0x5E;
    // const REG_AUTOCONFIG0: u8 = 0x7B;
    // const REG_AUTOCONFIG1: u8 = 0x7C;
    // const REG_UPLIMIT: u8 = 0x7D;
    // const REG_LOWLIMIT: u8 = 0x7E;
    // const REG_TARGETLIMIT: u8 = 0x7F;
    // const REG_GPIODIR: u8 = 0x76;
    // const REG_GPIOEN: u8 = 0x77;
    // const REG_GPIOSET: u8 = 0x78;
    // const REG_GPIOCLR: u8 = 0x79;
    // const REG_GPIOTOGGLE: u8 = 0x7A;
    const REG_SOFTRESET: u8 = 0x80;

    /// Opens MPR121 with default I2C address (see `MPR121_I2CADDR_DEFAULT`)
    pub fn new_default(device_id: u8) -> Result<Self, Mpr121Error> {
        Mpr121::new(device_id, MPR121_I2CADDR_DEFAULT)
    }

    /// Opens MPR121 with default I2C address (0x5a)
    pub fn new(device_id: u8, slave_addr: u16) -> Result<Self, Mpr121Error> {
        let dev = LinuxI2CDevice::new(format!("/dev/i2c-{}", device_id), slave_addr)?;
        Ok(Mpr121 { dev })
    }

    /// Reset the MPR121 into a default state ready to detect touch inputs, with
    /// default thresholds for touch and release
    pub fn reset(&mut self) -> Result<(), Mpr121Error> {
        self.reset_with_thresholds(
            MPR121_TOUCH_THRESHOLD_DEFAULT,
            MPR121_RELEASE_THRESHOLD_DEFAULT,
        )
    }

    /// Reset the MPR121 into a default state ready to detect touch inputs
    pub fn reset_with_thresholds(&mut self, touch: u8, release: u8) -> Result<(), Mpr121Error> {
        // Write to the reset register.
        self.dev
            .smbus_write_byte_data(Mpr121::REG_SOFTRESET, 0x63)?;
        // This 1ms delay here probably isn't necessary but can't hurt.
        std::thread::sleep(std::time::Duration::from_millis(1));
        // Set electrode configuration to default values.
        self.dev.smbus_write_byte_data(Mpr121::REG_ECR, 0x00)?;
        // Check CDT, SFI, ESI configuration is at default values.
        if self.dev.smbus_read_byte_data(Mpr121::REG_CONFIG2)? != 0x24 {
            panic!("Failed to find MPR121 in expected config state!");
        }
        // Default touch and release thresholds
        for i in 0..12 {
            self.dev
                .smbus_write_byte_data(Mpr121::REG_TOUCHTH_0 + 2 * i, touch)?;
            self.dev
                .smbus_write_byte_data(Mpr121::REG_RELEASETH_0 + 2 * i, release)?;
        }
        // Configure baseline filtering control registers.
        self.dev.smbus_write_byte_data(Mpr121::REG_MHDR, 0x01)?;
        self.dev.smbus_write_byte_data(Mpr121::REG_NHDR, 0x01)?;
        self.dev.smbus_write_byte_data(Mpr121::REG_NCLR, 0x0E)?;
        self.dev.smbus_write_byte_data(Mpr121::REG_FDLR, 0x00)?;
        self.dev.smbus_write_byte_data(Mpr121::REG_MHDF, 0x01)?;
        self.dev.smbus_write_byte_data(Mpr121::REG_NHDF, 0x05)?;
        self.dev.smbus_write_byte_data(Mpr121::REG_NCLF, 0x01)?;
        self.dev.smbus_write_byte_data(Mpr121::REG_FDLF, 0x00)?;
        self.dev.smbus_write_byte_data(Mpr121::REG_NHDT, 0x00)?;
        self.dev.smbus_write_byte_data(Mpr121::REG_NCLT, 0x00)?;
        self.dev.smbus_write_byte_data(Mpr121::REG_FDLT, 0x00)?;
        // Set other configuration registers.
        self.dev.smbus_write_byte_data(Mpr121::REG_DEBOUNCE, 0)?;
        // default, 16uA charge current
        self.dev.smbus_write_byte_data(Mpr121::REG_CONFIG1, 0x10)?;
        // 0.5uS encoding, 1ms period
        self.dev.smbus_write_byte_data(Mpr121::REG_CONFIG2, 0x20)?;
        // Enable all electrodes.
        self.dev.smbus_write_byte_data(Mpr121::REG_ECR, 0x8F)?;
        // start with first 5 bits of baseline tracking

        Ok(())
    }

    /// Reads the touch status of MPR121. In order to detect if something was really
    /// touched, old and new status must be compared.
    pub fn touch_status(&mut self) -> Result<Mpr121TouchStatus, Mpr121Error> {
        let status = self.dev.smbus_read_word_data(Mpr121::REG_TOUCHSTATUS_L)?;
        Ok(Mpr121TouchStatus::new(status))
    }
}

impl Mpr121TouchStatus {
    /// Creates new touch status
    fn new(touch_status: u16) -> Self {
        Self {
            status: touch_status,
        }
    }

    /// Returns if specific pin was touched
    pub fn touched(&self, item: u8) -> bool {
        if item <= Mpr121TouchStatus::last() {
            return self.status >> item & 0x1 != 0;
        }

        false
    }

    /// Returns if *any* pin was touched
    pub fn was_touched(&self) -> bool {
        self.status > 0
    }

    /// Number of the first pin
    pub fn first() -> u8 {
        0
    }

    /// Number of the last pin
    pub fn last() -> u8 {
        11
    }

    pub fn iter(&self) -> Mpr121TouchStatusIterator {
        Mpr121TouchStatusIterator::new(self)
    }
}

impl std::fmt::Display for Mpr121TouchStatus {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "Touch status: [")?;
        for i in Mpr121TouchStatus::first()..=Mpr121TouchStatus::last() {
            if i > 0 {
                write!(f, ", ")?;
            }
            write!(
                f,
                "{} is {}",
                i,
                if (self.status >> i) & 0x1 != 0 {
                    "on"
                } else {
                    "off"
                }
            )?;
        }
        write!(f, "]")
    }
}

impl<'a> Mpr121TouchStatusIterator<'a> {
    fn new(status: &'a Mpr121TouchStatus) -> Self {
        Self { status, count: 0 }
    }
}

impl<'a> Iterator for Mpr121TouchStatusIterator<'a> {
    type Item = bool;

    fn next(&mut self) -> Option<Self::Item> {
        self.count += 1;

        if self.count <= 12 {
            Some(self.status.touched(self.count - 1))
        } else {
            None
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn touch_status() {
        assert_eq!(Mpr121TouchStatus::first(), 0);
        assert_eq!(Mpr121TouchStatus::last(), 11);
        {
            let ts = Mpr121TouchStatus::new(0b101010101010);
            let mut tsi = ts.iter();
            for i in Mpr121TouchStatus::first()..=Mpr121TouchStatus::last() {
                let v = tsi.next();
                assert!(v.is_some());
                let v = v.unwrap();
                if i % 2 == 0 {
                    assert!(!ts.touched(i));
                    assert!(!v);
                } else {
                    assert!(ts.touched(i));
                    assert!(v);
                }
            }
            assert!(tsi.next().is_none());
        }
        {
            let ts = Mpr121TouchStatus::new(0b010101010101);
            let mut tsi = ts.iter();
            for i in Mpr121TouchStatus::first()..=Mpr121TouchStatus::last() {
                let v = tsi.next();
                assert!(v.is_some());
                let v = v.unwrap();
                if i % 2 == 1 {
                    assert!(!ts.touched(i));
                    assert!(!v);
                } else {
                    assert!(ts.touched(i));
                    assert!(v);
                }
            }
            assert!(tsi.next().is_none());
        }
    }
}
