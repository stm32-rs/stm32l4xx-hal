//! STM32L4 HAL implementation
//!
//! NOTE: This HAL implementation is under active development (as is the underlying
//! `embedded_hal` itself, together with its traits, some of which are unproven).
//! We follow the HAL implementation in <https://github.com/therealprof/stm32f4xx-hal>
//! and pull in individual devices behind features - the goal is for this implementation
//! to become ubiquitous for the STM32L4 family of devices (well-tested, feature-complete,
//! well-documented). However! At this time, actual testing has only been performed for
//! the STM32L432KC microcontroller. Participation is of course very welcome!

#![no_std]

pub use embedded_hal as hal;

pub use stm32l4;
#[cfg(condition = "family_L4x1")]
pub use stm32l4::stm32l4x1 as pac;

#[cfg(condition = "family_L41_42")]
pub use stm32l4::stm32l412 as pac;
#[cfg(all(condition = "family_L4x2", not(condition = "family_L41_42")))]
pub use stm32l4::stm32l4x2 as pac;

#[cfg(condition = "family_L4x3")]
pub use stm32l4::stm32l4x3 as pac;

#[cfg(condition = "family_L4x5")]
pub use stm32l4::stm32l4x5 as pac;

#[cfg(condition = "family_L4x6")]
pub use stm32l4::stm32l4x6 as pac;

#[cfg(condition = "family_L4+x9")]
pub use stm32l4::stm32l4r9 as pac;

#[cfg(feature = "rt")]
pub use self::pac::interrupt;
// aliases for crate::pac
pub use crate::pac as device;
pub use crate::pac as stm32;

pub mod traits;

#[cfg(not(any(condition = "family_L4+x9",)))]
pub mod adc;
#[cfg(not(any(condition = "family_L4+x9",)))]
#[cfg(condition = "peripheral_can1")]
pub mod can;
#[cfg(not(any(condition = "family_L4+x9",)))]
pub mod crc;
pub mod datetime;
#[cfg(not(any(condition = "family_L4+x9",)))]
pub mod delay;
#[cfg(not(any(condition = "family_L4+x9",)))]
pub mod dma;
#[cfg(not(any(condition = "family_L4+x9",)))]
pub mod flash;
#[cfg(not(any(condition = "family_L4+x9",)))]
pub mod gpio;
#[cfg(not(any(condition = "family_L4+x9",)))]
pub mod i2c;
#[cfg(not(any(condition = "family_L4+x9",)))]
pub mod lptimer;
#[cfg(all(feature = "otg_fs", condition = "peripheral_usb_otg_fs"))]
pub mod otg_fs;
#[cfg(not(any(condition = "family_L4+x9",)))]
pub mod prelude;
#[cfg(not(any(condition = "family_L4+x9",)))]
pub mod pwm;
#[cfg(not(any(condition = "family_L4+x9",)))]
pub mod pwr;
#[cfg(not(any(condition = "family_L4+x9",)))]
#[cfg(condition = "peripheral_qspi")]
pub mod qspi;
#[cfg(not(any(condition = "family_L4+x9",)))]
pub mod rcc;
#[cfg(not(any(condition = "family_L4+x9",)))]
pub mod rng;
#[cfg(not(any(condition = "family_L4+x9",)))]
pub mod rtc;
#[cfg(not(any(condition = "family_L4+x9",)))]
pub mod serial;
#[cfg(not(any(condition = "family_L4+x9",)))]
pub mod signature;
#[cfg(not(any(condition = "family_L4+x9",)))]
pub mod spi;
#[cfg(not(any(condition = "family_L4+x9",)))]
pub mod time;
#[cfg(not(any(condition = "family_L4+x9",)))]
pub mod timer;
#[cfg(not(any(condition = "family_L4+x9",)))]
pub mod tsc;
#[cfg(all(feature = "stm32-usbd", condition = "peripheral_usb_device_fs"))]
pub mod usb;
#[cfg(not(any(condition = "family_L4+x9",)))]
pub mod watchdog;

mod sealed {
    pub trait Sealed {}
}
pub(crate) use sealed::Sealed;
