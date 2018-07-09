#![no_std]

#![feature(unsize)]

extern crate cortex_m;
extern crate embedded_hal as hal;
extern crate nb;
pub extern crate stm32l4;


mod dma;


#[cfg(test)]
mod tests {
    #[test]
    fn it_works() {
        assert_eq!(2 + 2, 4);
    }
}
