//! Test the serial interface in Half-Duplex mode.
//!
//! This example requires you to hook-up a pullup resistor on the TX pin. RX pin is not used.
//! Resistor value depends on the baurate and line caracteristics, 1KOhms works well in most cases.
//! Half-Duplex mode internally connect TX to RX, meaning that bytes sent will also be received.
#![deny(unsafe_code)]
#![deny(warnings)]
#![no_main]
#![no_std]

extern crate cortex_m;
#[macro_use(entry, exception)]
extern crate cortex_m_rt as rt;
#[macro_use(block)]
extern crate nb;
extern crate panic_semihosting;

extern crate stm32l4xx_hal as hal;
// #[macro_use(block)]
// extern crate nb;

use crate::hal::prelude::*;
use crate::hal::serial::{Config, Serial};
use crate::rt::ExceptionFrame;
use cortex_m::asm;

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
        .sysclk(80.mhz())
        .pclk1(80.mhz())
        .pclk2(80.mhz())
        .freeze(&mut flash.acr, &mut pwr);

    // The Serial API is highly generic
    // TRY the commented out, different pin configurations
    // let tx = gpioa.pa9.into_af7(&mut gpioa.moder, &mut gpioa.afrh).set_open_drain();
    let tx = gpioa
        .pa2
        .into_af7(&mut gpioa.moder, &mut gpioa.afrl)
        .set_open_drain();
    // let tx = gpiob.pb6.into_af7(&mut gpiob.moder, &mut gpiob.afrl).set_open_drain();

    // TRY using a different USART peripheral here
    let serial = Serial::usart2(
        p.USART2,
        (tx,),
        Config::default().baudrate(9_600.bps()),
        clocks,
        &mut rcc.apb1r1,
    );
    let (mut tx, mut rx) = serial.split();

    let sent = b'X';

    // The `block!` macro makes an operation block until it finishes
    // NOTE the error type is `!`

    block!(tx.write(sent)).ok();

    let received = block!(rx.read()).unwrap();

    assert_eq!(received, sent);

    // if all goes well you should reach this breakpoint
    asm::bkpt();

    loop {
        continue;
    }
}

#[exception]
fn HardFault(ef: &ExceptionFrame) -> ! {
    panic!("{:#?}", ef);
}
