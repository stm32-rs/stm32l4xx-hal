//! Test the serial interface with the DMA engine
//!
//! This example requires you to short (connect) the TX and RX pins.
#![no_main]
#![no_std]

use cortex_m::singleton;
use cortex_m_rt::entry;
use defmt::println;
use defmt_rtt as _;
use nb::block;
use panic_probe as _;
use stm32l4xx_hal::{self as hal, dma::CircReadDma, prelude::*, serial::Serial};

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
    block!(tx.write(sent)).unwrap();
    let buf = singleton!(: [u8; 8] = [0; 8]).unwrap();
    let mut circ_buffer = rx.with_dma(channels.6).circ_read(buf);
    let mut rx_buf = [0; 8];
    let rx_len = circ_buffer.read(&mut rx_buf).unwrap();

    let received = &rx_buf[..rx_len];
    defmt::assert_eq!([sent], received);

    println!("echo received");

    loop {
        continue;
    }
}
