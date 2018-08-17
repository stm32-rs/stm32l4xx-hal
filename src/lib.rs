//! STM32l432xx Hardware abstraction layer

#![no_std]

// TODO, remove this feature (currently required in dma.rs)
#![feature(unsize)]

extern crate cortex_m;
extern crate cast;
extern crate embedded_hal as hal;
extern crate nb;
pub extern crate stm32l4;
extern crate void;


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


#[cfg(test)]
mod tests {
    #[test]
    fn it_works() {
        assert_eq!(2 + 2, 4);
    }
}
