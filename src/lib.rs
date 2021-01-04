use i2cdev::{
    core::*,
    linux::{LinuxI2CDevice, LinuxI2CError},
};

pub struct Mpr121 {
    dev: LinuxI2CDevice,
}

pub type Mpr121Error = LinuxI2CError;

pub const MPR121_I2CADDR_DEFAULT: u16 = 0x5A;
pub const MPR121_TOUCH_THRESHOLD_DEFAULT: u8 = 12;
pub const MPR121_RELEASE_THRESHOLD_DEFAULT: u8 = 6;

// TODO: temporary - remove it!
#[allow(dead_code)]
/// How MPR121 works is inspired by
/// https://github.com/adafruit/Adafruit_CircuitPython_MPR121 and https://github.com/adafruit/Adafruit_MPR121
impl Mpr121 {
    // Register addresses.
    const REG_TOUCHSTATUS_L: u8 = 0x00;
    const REG_TOUCHSTATUS_H: u8 = 0x01;
    const REG_FILTDATA_0L: u8 = 0x04;
    const REG_FILTDATA_0H: u8 = 0x05;
    const REG_BASELINE_0: u8 = 0x1E;
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
    const REG_CHARGECURR_0: u8 = 0x5F;
    const REG_CHARGETIME_1: u8 = 0x6C;
    const REG_ECR: u8 = 0x5E;
    const REG_AUTOCONFIG0: u8 = 0x7B;
    const REG_AUTOCONFIG1: u8 = 0x7C;
    const REG_UPLIMIT: u8 = 0x7D;
    const REG_LOWLIMIT: u8 = 0x7E;
    const REG_TARGETLIMIT: u8 = 0x7F;
    const REG_GPIODIR: u8 = 0x76;
    const REG_GPIOEN: u8 = 0x77;
    const REG_GPIOSET: u8 = 0x78;
    const REG_GPIOCLR: u8 = 0x79;
    const REG_GPIOTOGGLE: u8 = 0x7A;
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
        // self.dev.smbus_write_byte_data(Mpr121::REG_ECR, 0x8F)?;
        // start with first 5 bits of baseline tracking

        Ok(())
    }

    /// Reads the touch status of MPR121. In order to detect if something was really
    /// touched, old and new status must be compared.
    pub fn touch_status(&mut self) -> Result<u16, Mpr121Error> {
        self.dev.smbus_read_word_data(Mpr121::REG_TOUCHSTATUS_L)
    }
}

#[cfg(test)]
mod tests {
    #[test]
    fn it_works() {
        assert_eq!(2 + 2, 4);
    }
}
