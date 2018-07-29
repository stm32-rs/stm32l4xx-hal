//! Test the serial interface with the DMA engine
//!
//! This example requires you to short (connect) the TX and RX pins.
#![deny(unsafe_code)]
// #![deny(warnings)]
#![no_main]
#![no_std]

#[macro_use(singleton)]
extern crate cortex_m;
#[macro_use(entry, exception)]
extern crate cortex_m_rt as rt;
#[macro_use(block)]
extern crate nb;
extern crate panic_semihosting;

extern crate stm32l432xx_hal as hal;
// #[macro_use(block)]
// extern crate nb;

use cortex_m::asm;
use hal::dma::Half;
use hal::prelude::*;
use hal::serial::Serial;
use hal::stm32l4::stm32l4x2;
use rt::ExceptionFrame;
use hal::delay::Delay;

entry!(main);

fn main() -> ! {
    let p = stm32l4x2::Peripherals::take().unwrap();
    let cp = cortex_m::Peripherals::take().unwrap();
    
    let mut flash = p.FLASH.constrain();
    let mut rcc = p.RCC.constrain();
    let mut gpioa = p.GPIOA.split(&mut rcc.ahb2);
    let channels = p.DMA1.split(&mut rcc.ahb1);
    // let mut gpiob = p.GPIOB.split(&mut rcc.ahb2);

    // clock configuration using the default settings (all clocks run at 8 MHz)
    let clocks = rcc.cfgr.freeze(&mut flash.acr);
    // TRY this alternate clock configuration (clocks run at nearly the maximum frequency)
    // let clocks = rcc.cfgr.sysclk(64.mhz()).pclk1(32.mhz()).freeze(&mut flash.acr);

    let mut timer = Delay::new(cp.SYST, clocks);
    // The Serial API is highly generic
    // TRY the commented out, different pin configurations
    let tx = gpioa.pa9.into_af7(&mut gpioa.moder, &mut gpioa.afrh);
    // let tx = gpiob.pb6.into_af7(&mut gpiob.moder, &mut gpiob.afrl);

    let rx = gpioa.pa10.into_af7(&mut gpioa.moder, &mut gpioa.afrh);
    // let rx = gpiob.pb7.into_af7(&mut gpiob.moder, &mut gpiob.afrl);

    // TRY using a different USART peripheral here
    let serial = Serial::usart1(p.USART1, (tx, rx), 9_600.bps(), clocks, &mut rcc.apb2);
    let (mut tx, mut rx) = serial.split();

    let sent = b'S';

    // The `block!` macro makes an operation block until it finishes
    // NOTE the error type is `!`

    block!(tx.write(sent)).ok();

    let buf = singleton!(: [[u8; 8]; 2] = [[0; 8]; 2]).unwrap();

    let mut circ_buffer = rx.circ_read(channels.5, buf);

    // wait for 3 seconds, enter data on serial
    timer.delay_ms(1000_u32);
    timer.delay_ms(1000_u32);
    timer.delay_ms(1000_u32);

    // then view the partial buffer
    circ_buffer.partial_peek(|half, _| {
        Ok( (0, 0) )
    }).unwrap();

    // for _ in 0..2 {
    //     while circ_buffer.readable_half().unwrap() != Half::First {}

    //     let _first_half = circ_buffer.peek(|half, _| {
    //         *half
    //     }).unwrap();
        
    //     // asm::bkpt();

    //     while circ_buffer.readable_half().unwrap() != Half::Second {}

    //     // asm::bkpt();

    //     let _second_half = circ_buffer.peek(|half, _| *half).unwrap();
    // }
    
    asm::bkpt();

    loop {}
}

exception!(HardFault, hard_fault);

fn hard_fault(ef: &ExceptionFrame) -> ! {
    panic!("{:#?}", ef);
}

exception!(*, default_handler);

fn default_handler(irqn: i16) {
    panic!("Unhandled exception (IRQn = {})", irqn);
}
