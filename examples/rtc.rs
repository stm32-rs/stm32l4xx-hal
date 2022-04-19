//! Blinks an LED

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
use time::{Date, Time};

use crate::hal::delay::Delay;
use crate::hal::prelude::*;
use crate::hal::rcc::{ClockSecuritySystem, CrystalBypass};
use crate::hal::rtc::{Rtc, RtcClockSource, RtcConfig};
use crate::rt::ExceptionFrame;

use crate::sh::hio;
use core::convert::TryInto;
use core::fmt::Write;

#[entry]
fn main() -> ! {
    let mut hstdout = hio::hstdout().unwrap();

    writeln!(hstdout, "Hello, world!").unwrap();

    let cp = cortex_m::Peripherals::take().unwrap();
    let dp = hal::stm32::Peripherals::take().unwrap();

    let mut flash = dp.FLASH.constrain();
    let mut rcc = dp.RCC.constrain();
    let mut pwr = dp.PWR.constrain(&mut rcc.apb1r1);

    // Try a different clock configuration
    let clocks = rcc
        .cfgr
        .lse(CrystalBypass::Disable, ClockSecuritySystem::Disable)
        .freeze(&mut flash.acr, &mut pwr);

    let mut timer = Delay::new(cp.SYST, clocks);

    let mut rtc = Rtc::rtc(
        dp.RTC,
        &mut rcc.apb1r1,
        &mut rcc.bdcr,
        &mut pwr.cr1,
        RtcConfig::default().clock_config(RtcClockSource::LSE),
    );

    let time = Time::from_hms(21, 57, 32).unwrap();
    let date = Date::from_calendar_date(2018, 4.try_into().unwrap(), 24).unwrap();

    rtc.set_datetime(&date.with_time(time));

    timer.delay_ms(1000_u32);
    timer.delay_ms(1000_u32);
    timer.delay_ms(1000_u32);

    let rtc_datetime = rtc.get_datetime();

    writeln!(hstdout, "Time: {:?}", rtc_datetime.time()).unwrap();
    writeln!(hstdout, "Date: {:?}", rtc_datetime.date()).unwrap();
    writeln!(hstdout, "Good bye!").unwrap();
    loop {
        continue;
    }
}

#[exception]
unsafe fn HardFault(ef: &ExceptionFrame) -> ! {
    panic!("{:#?}", ef);
}
