#![no_std]
#![feature(unsize)]
#![feature(never_type)]

extern crate cortex_m;
extern crate cast;
extern crate embedded_hal as hal;
extern crate nb;
pub extern crate stm32l4;


mod dma;
// mod serial;
mod time;
mod rcc;
// mod gpio;


#[cfg(test)]
mod tests {
    #[test]
    fn it_works() {
        assert_eq!(2 + 2, 4);
    }
}
