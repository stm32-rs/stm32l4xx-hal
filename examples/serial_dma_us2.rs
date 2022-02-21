//! Test the serial interface with the DMA engine
//!
//! This example requires you to short (connect) the TX and RX pins.
#![no_main]
#![no_std]

#[macro_use(singleton)]
extern crate cortex_m;
#[macro_use(entry, exception)]
extern crate cortex_m_rt as rt;
#[macro_use(block)]
extern crate nb;
extern crate panic_semihosting;

extern crate stm32l4xx_hal as hal;
// #[macro_use(block)]
// extern crate nb;

use crate::hal::dma::CircReadDma;
use crate::hal::prelude::*;
use crate::hal::serial::Serial;
use crate::rt::ExceptionFrame;
use cortex_m::asm;

#[entry]
fn main() -> ! {
    let p = hal::stm32::Peripherals::take().unwrap();

    let mut flash = p.FLASH.constrain();
    let mut rcc = p.RCC.constrain();
    let mut pwr = p.PWR.constrain(&mut rcc.apb1r1);

    let mut gpioa = p.GPIOA.split(&mut rcc.ahb2);
    let channels = p.DMA1.split(&mut rcc.ahb1);
    // let mut gpiob = p.GPIOB.split(&mut rcc.ahb2);

    // clock configuration using the default settings (all clocks run at 8 MHz)
    let clocks = rcc.cfgr.freeze(&mut flash.acr, &mut pwr);
    // TRY this alternate clock configuration (clocks run at nearly the maximum frequency)
    // let clocks = rcc.cfgr.sysclk(64.MHz()).pclk1(32.MHz()).freeze(&mut flash.acr);

    let tx = gpioa
        .pa2
        .into_alternate(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrl);
    // let tx = gpiob.pb6.into_af7_pushpull(&mut gpiob.moder, &mut gpiob.otyper, &mut gpiob.afrl);

    // let rx = gpioa.pa10.into_af7_pushpull(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrh);
    let rx = gpioa
        .pa3
        .into_alternate(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrl);
    // let rx = gpiob.pb7.into_af7_pushpull(&mut gpiob.moder, &mut gpiob.otyper, &mut gpiob.afrl);

    // TRY using a different USART peripheral here
    let serial = Serial::usart2(p.USART2, (tx, rx), 115_200.bps(), clocks, &mut rcc.apb1r1);
    let (mut tx, rx) = serial.split();

    let sent = b'X';

    // The `block!` macro makes an operation block until it finishes
    // NOTE the error type is `!`

    block!(tx.write(sent)).ok();

    let buf = singleton!(: [u8; 8] = [0; 8]).unwrap();

    let mut circ_buffer = rx.with_dma(channels.6).circ_read(buf);

    let mut rx_buf = [0; 8];
    let rx_len = circ_buffer.read(&mut rx_buf).unwrap();

    let _received = &rx_buf[..rx_len];

    // let received = block!(rx.read()).unwrap();

    // assert_eq!(received, sent);

    // if all goes well you should reach this breakpoint
    asm::bkpt();

    loop {
        continue;
    }
}

#[exception]
unsafe fn HardFault(ef: &ExceptionFrame) -> ! {
    panic!("{:#?}", ef);
}
