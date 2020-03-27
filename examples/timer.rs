//! Blinks an LED

#![deny(unsafe_code)]
// #![deny(warnings)]
#![no_std]
#![no_main]

extern crate cortex_m;
#[macro_use]
extern crate cortex_m_rt as rt;
extern crate cortex_m_semihosting as sh;
extern crate panic_semihosting;
extern crate stm32l4xx_hal as hal;

use crate::hal::interrupt;
use crate::hal::prelude::*;
use crate::hal::timer::{Event, Timer};
use crate::rt::entry;
use crate::rt::ExceptionFrame;

use crate::sh::hio;
use core::fmt::Write;

#[entry]
fn main() -> ! {
    let mut hstdout = hio::hstdout().unwrap();
    writeln!(hstdout, "Hello, world!").unwrap();

    let cp = cortex_m::Peripherals::take().unwrap();
    let dp = hal::stm32::Peripherals::take().unwrap();

    let mut flash = dp.FLASH.constrain(); // .constrain();
    let mut rcc = dp.RCC.constrain();

    // Try a different clock configuration
    let clocks = rcc.cfgr.freeze(&mut flash.acr);

    // let mut gpiob = dp.GPIOB.split(&mut rcc.ahb2);
    // let mut led = gpiob.pb3.into_push_pull_output(&mut gpiob.moder, &mut gpiob.otyper);

    let mut nvic = cp.NVIC;
    nvic.enable(hal::stm32::Interrupt::TIM7);
    let mut timer = Timer::tim7(dp.TIM7, 1.hz(), clocks, &mut rcc.apb1r1);
    timer.listen(Event::TimeOut);

    loop {}
}

#[interrupt]
fn TIM7() {
    let mut p = 0;
    p += 1;
    // let mut hstdout = hio::hstdout().unwrap();
    // writeln!(hstdout, "Hello, TIM!").unwrap();
}

#[exception]
fn HardFault(ef: &ExceptionFrame) -> ! {
    panic!("{:#?}", ef);
}
