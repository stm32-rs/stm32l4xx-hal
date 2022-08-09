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
    let channels = p.DMA1.split(&mut rcc.ahb1);
    // let mut gpiob = p.GPIOB.split(&mut rcc.ahb2);

    // clock configuration using the default settings (all clocks run at 8 MHz)
    let clocks = rcc.cfgr.freeze(&mut flash.acr, &mut pwr);
    // TRY this alternate clock configuration (clocks run at nearly the maximum frequency)
    // let clocks = rcc.cfgr.sysclk(64.MHz()).pclk1(32.MHz()).freeze(&mut flash.acr);

    // The Serial API is highly generic
    // TRY the commented out, different pin configurations
    let tx = gpioa
        .pa9
        .into_alternate(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrh);
    // let tx = gpiob.pb6.into_alternate(&mut gpiob.moder, &mut gpiob.otyper, &mut gpiob.afrl);

    let rx = gpioa
        .pa10
        .into_alternate(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrh);
    // let rx = gpiob.pb7.into_alternate(&mut gpiob.moder, &mut gpiob.otyper, &mut gpiob.afrl);

    // TRY using a different USART peripheral here
    let serial = Serial::usart1(
        p.USART1,
        (tx, rx),
        Config::default().baudrate(9_600.bps()),
        clocks,
        &mut rcc.apb2,
    );
    let (mut tx, rx) = serial.split();

    let buf = singleton!(: [u8; 9] = [0; 9]).unwrap();
    let mut circ_buffer = rx.with_dma(channels.5).circ_read(buf);
    let mut rx_buf = [0; 9];

    // single byte send/receive
    send(&mut tx, b"x");
    let rx_len = circ_buffer.read(&mut rx_buf).unwrap();
    assert_eq!(rx_len, 1);
    assert_eq!(&rx_buf[..1], b"x");

    // multi byte send/receive
    send(&mut tx, b"12345678");
    let rx_len = circ_buffer.read(&mut rx_buf).unwrap();
    assert_eq!(rx_len, 8);
    assert_eq!(&rx_buf[..8], b"12345678");

    // Checking three types of overflow detection
    // 1. write pointer passes read pointer
    send(&mut tx, b"12345678"); // write-pointer -> 8
    let rx_len = circ_buffer.read(&mut rx_buf[..1]).unwrap(); // read-pointer -> 1
    assert_eq!(rx_len, 1);
    send(&mut tx, b"12"); // write-pointer -> 1 (catches up with read-pointer)
    let rx_res = circ_buffer.read(&mut rx_buf[..1]);
    if let Err(hal::dma::Error::Overrun) = rx_res {
    } else {
        panic!("An overrun should have been detected");
    }

    // 2. transfer complete flag set but it looks like the write-pointer did not pass 0
    send(&mut tx, b"123456789"); // write-pointer stays 1, transfer complete flag set
    send(&mut tx, b"1234"); // write-pointer -> 5
    let rx_res = circ_buffer.read(&mut rx_buf[..]);
    if let Err(hal::dma::Error::Overrun) = rx_res {
    } else {
        panic!("An overrun should have been detected");
    }

    // 3a. half complete flag set but it looks like the write-ptr did not pass ceil(capacity/2) = 5
    send(&mut tx, b"123456789"); // write-pointer stays 5, all flags set
    send(&mut tx, b"12345678"); // write-pointer -> 4
    let rx_res = circ_buffer.read(&mut rx_buf[..]);
    if let Err(hal::dma::Error::Overrun) = rx_res {
    } else {
        panic!("An overrun should have been detected");
    }

    // 3b. check that the half complete flag is not yet set at write-pointer = floor(capacity/2) = 4
    send(&mut tx, b"1234"); // write-pointer -> 0
    circ_buffer.read(&mut rx_buf[..]).unwrap(); // read something to prevent overrun
    send(&mut tx, b"12345"); // write-pointer -> 4
    circ_buffer
        .read(&mut rx_buf[..])
        .expect("No overrun should be detected here");

    // Undetectable overrun
    send(&mut tx, b"123456789");
    send(&mut tx, b"abcdefgh"); // overrun but it looks like only 8 bytes have been written
    let rx_len = circ_buffer.read(&mut rx_buf[..]).unwrap();
    assert_eq!(rx_len, 8);
    assert_eq!(&rx_buf[..8], b"abcdefgh");

    // if all goes well you should reach this breakpoint
    asm::bkpt();

    loop {
        continue;
    }
}

fn send(tx: &mut impl embedded_hal::serial::Write<u8>, data: &[u8]) {
    for byte in data {
        if let Err(_) = block!(tx.write(*byte)) {
            panic!("serial tx failed");
        }
    }

    // waste some time so that the data will be received completely
    for _ in 0..10000 {
        cortex_m::asm::nop();
    }
}

#[exception]
unsafe fn HardFault(ef: &ExceptionFrame) -> ! {
    panic!("{:#?}", ef);
}
