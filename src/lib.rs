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
    // feature = "stm32l4p5",
    // feature = "stm32l4q5",
    feature = "stm32l4r5",
    feature = "stm32l4s5",
    // feature = "stm32l4r7",
    // feature = "stm32l4s7",
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
    stm32l4r5, stm32l4s5
    stm32l4r9, stm32l4s9
"
);

// the common lists of features to spec what a MCU is capable of
// lists are split by 3rd digit and 2nd digit groups

// L4x1
// any(feature = "stm32l431", feature = "stm32l451", feature = "stm32l471")
// L4x2
// any(feature = "stm32l412", feature = "stm32l422", feature = "stm32l432", feature = "stm32l442", feature = "stm32l452", feature = "stm32l462")
// L4x3
// any(feature = "stm32l433", feature = "stm32l443")
// L4x5
// any(feature = "stm32l475")
// L4x6
// any(feature = "stm32l476", feature = "stm32l486", feature = "stm32l496", , feature = "stm32l4a6")
// L4+x9
// any(feature = "stm32l4r9", feature = "stm32l4s9")

// NOTE: The even member of the pair has additional hashing peripheral(s)
// L41 / L42
// any(feature = "stm32l412", feature = "stm32l422")
// L43 / L44
// any(feature = "stm32l431", feature = "stm32l432", feature = "stm32l433", feature = "stm32l442", feature = "stm32l443")
// L45 / L46
// any(feature = "stm32l451", feature = "stm32l452", feature = "stm32l462")
// L47 / L48
// any(feature = "stm32l471", feature = "stm32l475", feature = "stm32l476", feature = "stm32l486")
// L49 / L4a
// any(feature = "stm32l496", feature = "stm32l4a6")
// L4r / L4s
// any(feature = "stm32l4r9", feature = "stm32l4s9")

pub use embedded_hal as hal;

pub use stm32l4;
#[cfg(any(feature = "stm32l431", feature = "stm32l451", feature = "stm32l471"))]
pub use stm32l4::stm32l4x1 as pac;

#[cfg(any(feature = "stm32l412", feature = "stm32l422"))]
pub use stm32l4::stm32l412 as pac;
#[cfg(any(
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

#[cfg(any(feature = "stm32l4r5", feature = "stm32l4s5"))]
pub use stm32l4::stm32l4r5 as pac;

#[cfg(any(feature = "stm32l4r9", feature = "stm32l4s9",))]
pub use stm32l4::stm32l4r9 as pac;

#[cfg(feature = "rt")]
pub use self::pac::interrupt;
// aliases for crate::pac
pub use crate::pac as device;
pub use crate::pac as stm32;

pub mod traits;

#[cfg(not(any(
    feature = "stm32l4r9",
    feature = "stm32l4s9",
    feature = "stm32l4r5",
    feature = "stm32l4s5",
)))]
pub mod adc;
#[cfg(not(any(feature = "stm32l4r9", feature = "stm32l4s9",)))]
#[cfg(not(any(feature = "stm32l412",)))]
pub mod can;
pub mod crc;
pub mod delay;
pub mod dma;
pub mod dmamux;
pub mod flash;
pub mod gpio;
pub mod i2c;
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
pub mod prelude;
pub mod pwm;
pub mod pwr;
#[cfg(not(any(
    feature = "stm32l433",
    feature = "stm32l443",
    feature = "stm32l4r5",
    feature = "stm32l4s5",
    feature = "stm32l4r9",
    feature = "stm32l4s9",
)))]
pub mod qspi;
pub mod rcc;
pub mod rng;
pub mod rtc;
pub mod serial;
pub mod signature;
pub mod spi;
pub mod time;
pub mod timer;
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
pub mod watchdog;

mod sealed {
    pub trait Sealed {}
}
pub(crate) use sealed::Sealed;
