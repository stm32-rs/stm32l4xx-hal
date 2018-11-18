//! STM32l432xx Hardware abstraction layer

#![no_std]

// TODO, remove this feature (currently required in dma.rs)
#![feature(unsize)]



use embedded_hal as hal;

pub use stm32l4;



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
pub mod i2c;

