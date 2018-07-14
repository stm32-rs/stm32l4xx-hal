#![no_std]

// currently these are the only 2 unstable features, never_typoe can replaced with void, nto sure what unsize can be repalced with
#![feature(unsize)]
#![feature(never_type)]

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


#[cfg(test)]
mod tests {
    #[test]
    fn it_works() {
        assert_eq!(2 + 2, 4);
    }
}
