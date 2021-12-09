//! Device electronic signature
//!
//! (stored in flash memory)
//!
//! Based on the STM32F4xx HAL.

use core::str::from_utf8_unchecked;

/// This is the test voltage, in millivolts of the calibration done at the factory
pub const VDDA_CALIB_MV: u32 = 3000;

macro_rules! define_ptr_type {
    ($name: ident, $ptr: expr) => {
        impl $name {
            fn ptr() -> *const Self {
                $ptr as *const _
            }

            /// Returns a wrapped reference to the value in flash memory
            pub fn get() -> &'static Self {
                unsafe { &*Self::ptr() }
            }
        }
    };
}

/// Uniqure Device ID register
#[derive(Hash, Debug)]
#[repr(C)]
pub struct Uid {
    x: u16,
    y: u16,
    waf_lot: [u8; 8],
}
define_ptr_type!(Uid, 0x1FFF_7590);

impl Uid {
    /// X coordinate on wafer
    pub fn x(&self) -> u16 {
        self.x
    }

    /// Y coordinate on wafer
    pub fn y(&self) -> u16 {
        self.y
    }

    /// Wafer number
    pub fn waf_num(&self) -> u8 {
        self.waf_lot[0]
    }

    /// Lot number
    pub fn lot_num(&self) -> &str {
        unsafe { from_utf8_unchecked(&self.waf_lot[1..]) }
    }

    /// As a byte array
    pub fn as_bytes() -> &'static [u8; 12] {
        unsafe { &*(Self::ptr() as *const _) }
    }
}

/// Size of integrated flash
#[derive(Debug)]
#[repr(C)]
pub struct FlashSize(u16);
define_ptr_type!(FlashSize, 0x1FFF_75E0);

impl FlashSize {
    /// Read flash size in kilobytes
    pub fn kilo_bytes(&self) -> u16 {
        self.0
    }

    /// Read flash size in bytes
    pub fn bytes(&self) -> usize {
        usize::from(self.kilo_bytes()) * 1024
    }
}

/// ADC VREF calibration value is stored in at the factory
#[derive(Debug)]
#[repr(C)]
pub struct VrefCal(u16);
define_ptr_type!(VrefCal, 0x1FFF_75AA);

impl VrefCal {
    /// Read calibration value
    pub fn read(&self) -> u16 {
        self.0
    }
}

/// A temperature reading taken at 30°C stored at the factory
/// aka TS_CAL1 in reference manual
#[derive(Debug)]
#[repr(C)]
pub struct VtempCalLow(u16);
define_ptr_type!(VtempCalLow, 0x1FFF_75A8);

impl VtempCalLow {
    /// aka TS_CAL1_TEMP in reference manual
    pub const TEMP_DEGREES: u16 = 30;
    /// Read calibration value
    pub fn read(&self) -> u16 {
        self.0
    }
}

/// A temperature reading taken at 130°C stored at the factory
/// aka TS_CAL2 in reference manual
#[derive(Debug)]
#[repr(C)]
pub struct VtempCalHigh(u16);
define_ptr_type!(VtempCalHigh, 0x1FFF_75CA);

impl VtempCalHigh {
    /// aka TS_CAL2_TEMP in reference manual
    /// Feature gate Required: this is 110 for L47x/L48x, 130 for other L4s according to
    /// https://github.com/STMicroelectronics/STM32CubeL4/blob/5e1553e07706491bd11f4edd304e093b6e4b83a4/Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_ll_adc.h#L352-L356

    // L47/L48
    #[cfg(any(
        feature = "stm32l471",
        feature = "stm32l475",
        feature = "stm32l476",
        feature = "stm32l486"
    ))]
    pub const TEMP_DEGREES: u16 = 110;
    // else
    #[cfg(not(any(
        feature = "stm32l471",
        feature = "stm32l475",
        feature = "stm32l476",
        feature = "stm32l486"
    )))]
    pub const TEMP_DEGREES: u16 = 130;
    /// Read calibration value
    pub fn read(&self) -> u16 {
        self.0
    }
}
