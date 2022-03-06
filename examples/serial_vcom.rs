//! Test the serial interface
//!
#![no_main]
#![no_std]

use cortex_m_rt::entry;
use defmt::println;
use nb::block;
use panic_probe as _;
use stm32l4xx_hal::{
    self as hal,
    prelude::*,
    serial::{Config, Serial},
};

#[entry]
fn main() -> ! {
    let p = hal::stm32::Peripherals::take().unwrap();

    let mut flash = p.FLASH.constrain();
    let mut rcc = p.RCC.constrain();
    let mut pwr = p.PWR.constrain(&mut rcc.apb1r1);
    // let mut gpioa = p.GPIOA.split(&mut rcc.ahb2);
    // let mut gpiob = p.GPIOB.split(&mut rcc.ahb2);
    let mut gpiod = p.GPIOD.split(&mut rcc.ahb2);

    // clock configuration using the default settings (all clocks run at 8 MHz)
    let clocks = rcc.cfgr.freeze(&mut flash.acr, &mut pwr);
    // TRY this alternate clock configuration (clocks run at nearly the maximum frequency)
    // let clocks = rcc.cfgr.sysclk(64.MHz()).pclk1(32.MHz()).freeze(&mut flash.acr);

    //let tx = gpioa.pa2.into_af7(&mut gpioa.moder, &mut gpioa.afrl);
    // let tx = gpiob.pb6.into_af7(&mut gpiob.moder, &mut gpiob.afrl);
    let tx = gpiod
        .pd5
        .into_alternate(&mut gpiod.moder, &mut gpiod.otyper, &mut gpiod.afrl);

    // let rx = gpioa.pa3.into_af7(&mut gpioa.moder, &mut gpioa.afrl);
    // let rx = gpiob.pb7.into_af7(&mut gpiob.moder, &mut gpiob.afrl);
    let rx = gpiod
        .pd6
        .into_alternate(&mut gpiod.moder, &mut gpiod.otyper, &mut gpiod.afrl);

    // TRY using a different USART peripheral here
    let serial = Serial::usart2(
        p.USART2,
        (tx, rx),
        Config::default().baudrate(115_200.bps()),
        clocks,
        &mut rcc.apb1r1,
    );
    let (mut tx, mut rx) = serial.split();

    let sent = b'X';

    // The `block!` macro makes an operation block until it finishes
    // NOTE the error type is `!`

    block!(tx.write(sent)).unwrap();
    block!(tx.write(sent)).unwrap();
    block!(tx.write(sent)).unwrap();
    block!(tx.write(sent)).unwrap();
    block!(tx.write(sent)).unwrap();

    // when using virtual com port for recieve can causes a framing error
    // On the stm32l476 discovery it is working fine at 115200 baud
    let received = block!(rx.read()).unwrap();

    assert_eq!(received, sent);

    // if all goes well you should reach this breakpoint
    println!("example complete");

    loop {
        continue;
    }
}
