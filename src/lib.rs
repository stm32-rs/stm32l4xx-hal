//! STM32L4 Hardware abstraction layer

#![no_std]

// TODO, remove this feature (currently required in dma.rs)
#![feature(unsize)]

use embedded_hal as hal;

pub use stm32l4;

#[cfg(feature = "stm32l4x1")]
pub use stm32l4::stm32l4x1 as stm32;

#[cfg(feature = "stm32l4x2")]
pub use stm32l4::stm32l4x2 as stm32;

#[cfg(feature = "stm32l4x3")]
pub use stm32l4::stm32l4x3 as stm32;

#[cfg(feature = "stm32l4x5")]
pub use stm32l4::stm32l4x5 as stm32;

#[cfg(feature = "stm32l4x6")]
pub use stm32l4::stm32l4x6 as stm32;

#[cfg(feature = "rt")]
pub use stm32l4::interrupt;

pub mod dma;
pub mod prelude;
pub mod serial;
pub mod time;
pub mod rcc;
pub mod flash;
pub mod gpio;
pub mod delay;
pub mod timer;
pub mod spi;
pub mod rtc;
pub mod pwr;
pub mod datetime;
pub mod tsc;

