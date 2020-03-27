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
// #[macro_use(block)]
// extern crate nb;

use crate::hal::datetime::{Date, Time};
use crate::hal::delay::Delay;
use crate::hal::prelude::*;
use crate::hal::rtc::Rtc;
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

    let mut timer = Delay::new(cp.SYST, clocks);
    let mut pwr = dp.PWR.constrain(&mut rcc.apb1r1);
    let rtc = Rtc::rtc(dp.RTC, &mut rcc.apb1r1, &mut rcc.bdcr, &mut pwr.cr1, clocks);

    let mut time = Time::new(21.hours(), 57.minutes(), 32.seconds(), false);
    let mut date = Date::new(1.day(), 24.date(), 4.month(), 2018.year());

    rtc.set_time(&time);
    rtc.set_date(&date);

    timer.delay_ms(1000_u32);
    timer.delay_ms(1000_u32);
    timer.delay_ms(1000_u32);

    time = rtc.get_time();
    date = rtc.get_date();

    writeln!(hstdout, "Time: {:?}", time).unwrap();
    writeln!(hstdout, "Date: {:?}", date).unwrap();
    writeln!(hstdout, "Good bye!").unwrap();
    loop {}
}

#[exception]
fn HardFault(ef: &ExceptionFrame) -> ! {
    panic!("{:#?}", ef);
}
