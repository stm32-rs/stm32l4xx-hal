//! Test the serial interface
//!
//! This example requires you to short (connect) the TX and RX pins.
#![no_main]
#![no_std]

use cortex_m_rt::entry;
use defmt::println;
use defmt_rtt as _;
use nb::block;
use panic_probe as _;
use stm32l4xx_hal::{self as hal, prelude::*, serial::Serial};

#[entry]
fn main() -> ! {
    let p = hal::stm32::Peripherals::take().unwrap();

    let mut flash = p.FLASH.constrain();
    let mut rcc = p.RCC.constrain();
    let mut pwr = p.PWR.constrain(&mut rcc.apb1r1);

    let mut gpioa = p.GPIOA.split(&mut rcc.ahb2);
    // let mut gpiob = p.GPIOB.split(&mut rcc.ahb2);

    // clock configuration using the default settings (all clocks run at 8 MHz)
    // let clocks = rcc.cfgr.freeze(&mut flash.acr);
    // TRY this alternate clock configuration (clocks run at nearly the maximum frequency)
    let clocks = rcc
        .cfgr
        .sysclk(80.MHz())
        .pclk1(80.MHz())
        .pclk2(80.MHz())
        .freeze(&mut flash.acr, &mut pwr);

    // The Serial API is highly generic
    // TRY the commented out, different pin configurations
    // let tx = gpioa.pa9.into_af7_pushpull(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrh);
    let tx = gpioa
        .pa2
        .into_alternate(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrl);
    // let tx = gpiob.pb6.into_alternate(&mut gpiob.moder, &mut gpiob.otyper, &mut gpiob.afrl);

    // let rx = gpioa.pa10.into_alternate(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrh);
    let rx = gpioa
        .pa3
        .into_alternate(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrl);
    // let rx = gpiob.pb7.into_alternate(&mut gpiob.moder, &mut gpiob.otyper, &mut gpiob.afrl);

    // TRY using a different USART peripheral here
    let serial = Serial::usart2(p.USART2, (tx, rx), 9_600.bps(), clocks, &mut rcc.apb1r1);
    let (mut tx, mut rx) = serial.split();

    let sent = b'X';

    // The `block!` macro makes an operation block until it finishes
    // NOTE the error type is `!`

    block!(tx.write(sent)).unwrap();
    let received = block!(rx.read()).unwrap();

    assert_eq!(received, sent);
    println!("example complete");

    loop {
        continue;
    }
}
