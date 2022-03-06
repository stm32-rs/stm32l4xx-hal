//! Interfacing the on-board L3GD20 (gyroscope)
#![no_main]
#![no_std]

use cortex_m_rt::entry;
use defmt::println;
use embedded_hal::spi::{Mode, Phase, Polarity};
use stm32l4xx_hal::{self as hal, prelude::*, spi::Spi};

/// SPI mode
pub const MODE: Mode = Mode {
    phase: Phase::CaptureOnFirstTransition,
    polarity: Polarity::IdleLow,
};

#[entry]
fn main() -> ! {
    let p = hal::stm32::Peripherals::take().unwrap();

    let mut flash = p.FLASH.constrain();
    let mut rcc = p.RCC.constrain();
    let mut pwr = p.PWR.constrain(&mut rcc.apb1r1);

    // TRY the other clock configuration
    // let clocks = rcc.cfgr.freeze(&mut flash.acr);
    let _clocks = rcc
        .cfgr
        .sysclk(80.MHz())
        .pclk1(80.MHz())
        .pclk2(80.MHz())
        .freeze(&mut flash.acr, &mut pwr);

    let mut gpioa = p.GPIOA.split(&mut rcc.ahb2);

    // The `L3gd20` abstraction exposed by the `f3` crate requires a specific pin configuration to
    // be used and won't accept any configuration other than the one used here. Trying to use a
    // different pin configuration will result in a compiler error.
    let sck = gpioa
        .pa5
        .into_alternate(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrl);
    let miso = gpioa
        .pa6
        .into_alternate(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrl);
    let mosi = gpioa
        .pa7
        .into_alternate(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrl);

    // clock speed is determined by the master
    let mut spi = Spi::spi1_slave(p.SPI1, (sck, miso, mosi), MODE, &mut rcc.apb2);

    let mut data = [0x1];
    // this will block until the master starts the clock
    spi.transfer(&mut data).unwrap();

    // when you reach this breakpoint you'll be able to inspect the variable `data` which contains the
    // data sent by the master
    println!("example complete");

    loop {
        continue;
    }
}
