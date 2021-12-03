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
    feature = "stm32l4a6",
    // note L4+ PAC support is mostly missing so other than r9/s9 these features don't actually exist yet
    //feature = "stm32l4p5",
    //feature = "stm32l4q5",
    //feature = "stm32l4r5",
    //feature = "stm32l4s5",
    //feature = "stm32l4r7",
    //feature = "stm32l4s7",
    // these have PAC support. Hal integration is very slim though
    feature = "stm32l4r9",
    feature = "stm32l4s9"
)))]
compile_error!(
    "\
This crate requires one of the following features enabled:
    stm32l431, stm32l451, stm32l471
    stm32l412, stm32l422, stm32l432, stm32l442, stm32l452, stm32l462
    stm32l433, stm32l443
    stm32l475, 
    stm32l476, stm32l486, stm32l496, stm32l4a6
    stm32l4r9, stm32l4s9
"
);

pub use embedded_hal as hal;

pub use stm32l4;
#[cfg(any(feature = "stm32l431", feature = "stm32l451", feature = "stm32l471"))]
pub use stm32l4::stm32l4x1 as pac;

#[cfg(any(
    feature = "stm32l412",
    feature = "stm32l422",
    feature = "stm32l432",
    feature = "stm32l442",
    feature = "stm32l452",
    feature = "stm32l462"
))]
pub use stm32l4::stm32l4x2 as pac;

#[cfg(any(feature = "stm32l433", feature = "stm32l443"))]
pub use stm32l4::stm32l4x3 as pac;

#[cfg(any(feature = "stm32l475"))]
pub use stm32l4::stm32l4x5 as pac;

#[cfg(any(
    feature = "stm32l476",
    feature = "stm32l486",
    feature = "stm32l496",
    feature = "stm32l4a6"
))]
pub use stm32l4::stm32l4x6 as pac;

#[cfg(any(feature = "stm32l4r9", feature = "stm32l4s9",))]
pub use stm32l4::stm32l4r9 as pac;

#[cfg(feature = "rt")]
pub use self::pac::interrupt;
// aliases for crate::pac
pub use crate::pac as device;
pub use crate::pac as stm32;

pub mod traits;

#[cfg(not(any(feature = "stm32l4r9", feature = "stm32l4s9",)))]
pub mod adc;
#[cfg(any(
    feature = "stm32l431",
    feature = "stm32l451",
    feature = "stm32l471",
    feature = "stm32l475"
))]
pub mod can;
#[cfg(not(any(feature = "stm32l4r9", feature = "stm32l4s9",)))]
pub mod crc;
pub mod datetime;
#[cfg(not(any(feature = "stm32l4r9", feature = "stm32l4s9",)))]
pub mod delay;
#[cfg(not(any(feature = "stm32l4r9", feature = "stm32l4s9",)))]
pub mod dma;
#[cfg(not(any(feature = "stm32l4r9", feature = "stm32l4s9",)))]
pub mod flash;
#[cfg(not(any(feature = "stm32l4r9", feature = "stm32l4s9",)))]
pub mod gpio;
#[cfg(not(any(feature = "stm32l4r9", feature = "stm32l4s9",)))]
pub mod i2c;
#[cfg(not(any(feature = "stm32l4r9", feature = "stm32l4s9",)))]
pub mod lptimer;
#[cfg(all(
    feature = "otg_fs",
    any(
        feature = "stm32l475",
        feature = "stm32l476",
        feature = "stm32l486",
        feature = "stm32l496",
        feature = "stm32l4a6",
    )
))]
pub mod otg_fs;
#[cfg(not(any(feature = "stm32l4r9", feature = "stm32l4s9",)))]
pub mod prelude;
#[cfg(not(any(feature = "stm32l4r9", feature = "stm32l4s9",)))]
pub mod pwm;
#[cfg(not(any(feature = "stm32l4r9", feature = "stm32l4s9",)))]
pub mod pwr;
#[cfg(not(any(
    feature = "stm32l433",
    feature = "stm32l443",
    feature = "stm32l4r9",
    feature = "stm32l4s9",
)))]
pub mod qspi;
#[cfg(not(any(feature = "stm32l4r9", feature = "stm32l4s9",)))]
pub mod rcc;
#[cfg(not(any(feature = "stm32l4r9", feature = "stm32l4s9",)))]
pub mod rng;
#[cfg(not(any(feature = "stm32l4r9", feature = "stm32l4s9",)))]
pub mod rtc;
#[cfg(not(any(feature = "stm32l4r9", feature = "stm32l4s9",)))]
pub mod serial;
#[cfg(not(any(feature = "stm32l4r9", feature = "stm32l4s9",)))]
pub mod signature;
#[cfg(not(any(feature = "stm32l4r9", feature = "stm32l4s9",)))]
pub mod spi;
#[cfg(not(any(feature = "stm32l4r9", feature = "stm32l4s9",)))]
pub mod time;
#[cfg(not(any(feature = "stm32l4r9", feature = "stm32l4s9",)))]
pub mod timer;
#[cfg(not(any(feature = "stm32l4r9", feature = "stm32l4s9",)))]
pub mod tsc;
#[cfg(all(
    feature = "stm32-usbd",
    any(
        feature = "stm32l412",
        feature = "stm32l422",
        feature = "stm32l432",
        feature = "stm32l442",
        feature = "stm32l452",
        feature = "stm32l462",
        feature = "stm32l433",
        feature = "stm32l443"
    )
))]
pub mod usb;
#[cfg(not(any(feature = "stm32l4r9", feature = "stm32l4s9",)))]
pub mod watchdog;

mod sealed {
    pub trait Sealed {}
}
pub(crate) use sealed::Sealed;
