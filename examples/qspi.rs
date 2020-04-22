//! Test the serial interface
//!
//! This example requires you to short (connect) the TX and RX pins.
// #![deny(unsafe_code)]
// #![deny(warnings)]
#![no_main]
#![no_std]

extern crate cortex_m;
#[macro_use(entry, exception)]
extern crate cortex_m_rt as rt;
// #[macro_use(block)]
extern crate nb;
extern crate panic_semihosting;

extern crate stm32l4xx_hal as hal;
// #[macro_use(block)]
// extern crate nb;

use cortex_m::asm;
use crate::hal::prelude::*;
use crate::hal::qspi::{Qspi, QspiReadCommand, QspiConfig};
use crate::rt::ExceptionFrame;

#[entry]
fn main() -> ! {
    let p = hal::stm32::Peripherals::take().unwrap();

    let mut flash = p.FLASH.constrain();
    let mut rcc = p.RCC.constrain();
    let mut gpioa = p.GPIOA.split(&mut rcc.ahb2);
    // let mut gpiob = p.GPIOB.split(&mut rcc.ahb2);
    let mut pwr = p.PWR.constrain(&mut rcc.apb1r1);

    // clock configuration using the default settings (all clocks run at 8 MHz)
    // let clocks = rcc.cfgr.freeze(&mut flash.acr);
    // TRY this alternate clock configuration (clocks run at nearly the maximum frequency)
    let clocks = rcc.cfgr.sysclk(80.mhz()).pclk1(80.mhz()).pclk2(80.mhz()).freeze(&mut flash.acr, &mut pwr);

    let command = QspiReadCommand{
        instruction : Some(0x9f),
        address : None,
        alternative_bytes : None,
        dummy_cycles : 0,
        recive_lenght : 3,
    };
    let mut arr : [u8; 3];

    let qspi = Qspi::new(p.QUADSPI, &mut rcc.ahb3, QspiConfig::default());

    qspi.transfer(command, &mut arr);

    // if all goes well you should reach this breakpoint
    asm::bkpt();

    loop {}
}

#[exception]
fn HardFault(ef: &ExceptionFrame) -> ! {
    panic!("{:#?}", ef);
}
