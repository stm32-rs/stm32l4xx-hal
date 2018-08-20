//! Test the serial interface
//!
//! This example requires you to short (connect) the TX and RX pins.
#![deny(unsafe_code)]
// #![deny(warnings)]
#![no_main]
#![no_std]

extern crate cortex_m;
#[macro_use(entry, exception)]
extern crate cortex_m_rt as rt;
extern crate panic_semihosting;

extern crate stm32l432xx_hal as hal;


use hal::prelude::*;
use hal::stm32l4::stm32l4x2;
use hal::tsc::Tsc;
use rt::ExceptionFrame;
use hal::delay::Delay;

entry!(main);

fn main() -> ! {
    let p = stm32l4x2::Peripherals::take().unwrap();
    let cp = cortex_m::Peripherals::take().unwrap();

    let mut flash = p.FLASH.constrain();
    let mut rcc = p.RCC.constrain();
    // let mut gpioa = p.GPIOA.split(&mut rcc.ahb2);
    let mut gpiob = p.GPIOB.split(&mut rcc.ahb2);

    // clock configuration using the default settings (all clocks run at 8 MHz)
    let clocks = rcc.cfgr.freeze(&mut flash.acr);
    // TRY this alternate clock configuration (clocks run at nearly the maximum frequency)
    // let clocks = rcc.cfgr.sysclk(64.mhz()).pclk1(32.mhz()).freeze(&mut flash.acr);

    // let mut delay = Delay::new(cp.SYST, clocks);

    let sample_pin = gpiob.pb4.into_touch_sample(&mut gpiob.moder, &mut gpiob.otyper, &mut gpiob.afrl);
    let c1 = gpiob.pb5.into_touch_channel(&mut gpiob.moder, &mut gpiob.otyper, &mut gpiob.afrl);
    let c2 = gpiob.pb6.into_touch_channel(&mut gpiob.moder, &mut gpiob.otyper, &mut gpiob.afrl);
    let c3 = gpiob.pb7.into_touch_channel(&mut gpiob.moder, &mut gpiob.otyper, &mut gpiob.afrl);
    
    let tsc = Tsc::tsc(p.TSC, sample_pin, (c1, c2, c3) , &mut rcc.ahb1);

    tsc.start();
    let baseline = tsc.wait().unwrap();

    tsc.start();
    let touched = tsc.wait().unwrap();

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
