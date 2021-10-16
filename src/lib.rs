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

#[cfg(not(any(
    feature = "stm32l431",
    feature = "stm32l451",
    feature = "stm32l471",
    feature = "stm32l412",
    feature = "stm32l422",
    feature = "stm32l432",
    feature = "stm32l442",
    feature = "stm32l452",
    feature = "stm32l462",
    feature = "stm32l433",
    feature = "stm32l443",
    feature = "stm32l475",
    feature = "stm32l476",
    feature = "stm32l486",
    feature = "stm32l496",
    feature = "stm32l4A6",
)))]
compile_error!("This crate requires one of the following features enabled: stm32l431, stm32l451, stm32l471, stm32l412, stm32l422, stm32l432, stm32l442, stm32l452, stm32l462, stm32l433, stm32l443, stm32l475, stm32l476, stm32l486, stm32l496, stm32l4A6, ");

pub use embedded_hal as hal;

pub use stm32l4;
#[cfg(feature = "private_pac_stm32l4x1")]
pub use stm32l4::stm32l4x1 as pac;

#[cfg(feature = "private_pac_stm32l4x2")]
pub use stm32l4::stm32l4x2 as pac;

#[cfg(feature = "private_pac_stm32l4x3")]
pub use stm32l4::stm32l4x3 as pac;

#[cfg(feature = "private_pac_stm32l4x5")]
pub use stm32l4::stm32l4x5 as pac;

#[cfg(feature = "private_pac_stm32l4x6")]
pub use stm32l4::stm32l4x6 as pac;

#[cfg(feature = "rt")]
pub use self::pac::interrupt;

#[cfg(any(
    feature = "private_pac_stm32l4x1",
    feature = "private_pac_stm32l4x2",
    feature = "private_pac_stm32l4x3",
    feature = "private_pac_stm32l4x5",
    feature = "private_pac_stm32l4x6"
))]
pub use crate::pac as device;
#[cfg(any(
    feature = "private_pac_stm32l4x1",
    feature = "private_pac_stm32l4x2",
    feature = "private_pac_stm32l4x3",
    feature = "private_pac_stm32l4x5",
    feature = "private_pac_stm32l4x6"
))]
pub use crate::pac as stm32;

pub mod traits;

#[cfg(any(
    feature = "private_pac_stm32l4x1",
    feature = "private_pac_stm32l4x2",
    feature = "private_pac_stm32l4x3",
    feature = "private_pac_stm32l4x5",
    feature = "private_pac_stm32l4x6"
))]
pub mod adc;
#[cfg(any(feature = "private_pac_stm32l4x1", feature = "private_pac_stm32l4x5",))]
pub mod can;
#[cfg(any(
    feature = "private_pac_stm32l4x1",
    feature = "private_pac_stm32l4x2",
    feature = "private_pac_stm32l4x3",
    feature = "private_pac_stm32l4x5",
    feature = "private_pac_stm32l4x6"
))]
pub mod crc;
pub mod datetime;
#[cfg(any(
    feature = "private_pac_stm32l4x1",
    feature = "private_pac_stm32l4x2",
    feature = "private_pac_stm32l4x3",
    feature = "private_pac_stm32l4x5",
    feature = "private_pac_stm32l4x6"
))]
pub mod delay;
#[cfg(any(
    feature = "private_pac_stm32l4x1",
    feature = "private_pac_stm32l4x2",
    feature = "private_pac_stm32l4x3",
    feature = "private_pac_stm32l4x5",
    feature = "private_pac_stm32l4x6"
))]
pub mod dma;
#[cfg(any(
    feature = "private_pac_stm32l4x1",
    feature = "private_pac_stm32l4x2",
    feature = "private_pac_stm32l4x3",
    feature = "private_pac_stm32l4x5",
    feature = "private_pac_stm32l4x6"
))]
pub mod flash;
#[cfg(any(
    feature = "private_pac_stm32l4x1",
    feature = "private_pac_stm32l4x2",
    feature = "private_pac_stm32l4x3",
    feature = "private_pac_stm32l4x5",
    feature = "private_pac_stm32l4x6"
))]
pub mod gpio;
#[cfg(any(
    feature = "private_pac_stm32l4x1",
    feature = "private_pac_stm32l4x2",
    feature = "private_pac_stm32l4x3",
    feature = "private_pac_stm32l4x5",
    feature = "private_pac_stm32l4x6"
))]
pub mod i2c;
#[cfg(all(
    feature = "otg_fs",
    any(feature = "private_pac_stm32l4x5", feature = "private_pac_stm32l4x6")
))]
pub mod otg_fs;
#[cfg(any(
    feature = "private_pac_stm32l4x1",
    feature = "private_pac_stm32l4x2",
    feature = "private_pac_stm32l4x3",
    feature = "private_pac_stm32l4x5",
    feature = "private_pac_stm32l4x6"
))]
pub mod prelude;
#[cfg(any(
    feature = "private_pac_stm32l4x1",
    feature = "private_pac_stm32l4x2",
    feature = "private_pac_stm32l4x3",
    feature = "private_pac_stm32l4x5",
    feature = "private_pac_stm32l4x6"
))]
pub mod pwm;
#[cfg(any(
    feature = "private_pac_stm32l4x1",
    feature = "private_pac_stm32l4x2",
    feature = "private_pac_stm32l4x3",
    feature = "private_pac_stm32l4x5",
    feature = "private_pac_stm32l4x6"
))]
pub mod pwr;

#[cfg(any(
    feature = "private_pac_stm32l4x1",
    feature = "private_pac_stm32l4x2",
    feature = "private_pac_stm32l4x3",
    feature = "private_pac_stm32l4x5",
    feature = "private_pac_stm32l4x6"
))]
pub mod lptimer;
#[cfg(any(
    feature = "private_pac_stm32l4x1",
    feature = "private_pac_stm32l4x2",
    feature = "private_pac_stm32l4x5",
    feature = "private_pac_stm32l4x6"
))]
pub mod qspi;
#[cfg(any(
    feature = "private_pac_stm32l4x1",
    feature = "private_pac_stm32l4x2",
    feature = "private_pac_stm32l4x3",
    feature = "private_pac_stm32l4x5",
    feature = "private_pac_stm32l4x6"
))]
pub mod rcc;
#[cfg(any(
    feature = "private_pac_stm32l4x1",
    feature = "private_pac_stm32l4x2",
    feature = "private_pac_stm32l4x3",
    feature = "private_pac_stm32l4x5",
    feature = "private_pac_stm32l4x6"
))]
pub mod rng;
#[cfg(any(
    feature = "private_pac_stm32l4x1",
    feature = "private_pac_stm32l4x2",
    feature = "private_pac_stm32l4x3",
    feature = "private_pac_stm32l4x5",
    feature = "private_pac_stm32l4x6"
))]
pub mod rtc;
#[cfg(any(
    feature = "private_pac_stm32l4x1",
    feature = "private_pac_stm32l4x2",
    feature = "private_pac_stm32l4x3",
    feature = "private_pac_stm32l4x5",
    feature = "private_pac_stm32l4x6"
))]
pub mod serial;
#[cfg(any(
    feature = "private_pac_stm32l4x1",
    feature = "private_pac_stm32l4x2",
    feature = "private_pac_stm32l4x3",
    feature = "private_pac_stm32l4x5",
    feature = "private_pac_stm32l4x6"
))]
pub mod signature;
#[cfg(any(
    feature = "private_pac_stm32l4x1",
    feature = "private_pac_stm32l4x2",
    feature = "private_pac_stm32l4x3",
    feature = "private_pac_stm32l4x5",
    feature = "private_pac_stm32l4x6"
))]
pub mod spi;
#[cfg(any(
    feature = "private_pac_stm32l4x1",
    feature = "private_pac_stm32l4x2",
    feature = "private_pac_stm32l4x3",
    feature = "private_pac_stm32l4x5",
    feature = "private_pac_stm32l4x6"
))]
pub mod time;
#[cfg(any(
    feature = "private_pac_stm32l4x1",
    feature = "private_pac_stm32l4x2",
    feature = "private_pac_stm32l4x3",
    feature = "private_pac_stm32l4x5",
    feature = "private_pac_stm32l4x6"
))]
pub mod timer;
#[cfg(any(
    feature = "private_pac_stm32l4x1",
    feature = "private_pac_stm32l4x2",
    feature = "private_pac_stm32l4x3",
    feature = "private_pac_stm32l4x5",
    feature = "private_pac_stm32l4x6"
))]
pub mod tsc;

#[cfg(all(
    feature = "stm32-usbd",
    any(feature = "private_pac_stm32l4x2", feature = "private_pac_stm32l4x3")
))]
pub mod usb;
#[cfg(any(
    feature = "private_pac_stm32l4x1",
    feature = "private_pac_stm32l4x2",
    feature = "private_pac_stm32l4x3",
    feature = "private_pac_stm32l4x5",
    feature = "private_pac_stm32l4x6"
))]
pub mod watchdog;
