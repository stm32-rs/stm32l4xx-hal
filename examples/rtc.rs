//! Blinks an LED

#![no_std]
#![no_main]

use cortex_m_rt::entry;
use defmt::println;
use panic_probe as _;
use stm32l4xx_hal::{
    self as hal,
    datetime::{Date, Time},
    delay::Delay,
    prelude::*,
    rcc::{ClockSecuritySystem, CrystalBypass},
    rtc::{Rtc, RtcClockSource, RtcConfig},
};

#[entry]
fn main() -> ! {
    println!("Hello, world!");

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

    let time = Time::new(21.hours(), 57.minutes(), 32.secs(), 0.micros(), false);
    let date = Date::new(1.day(), 24.date(), 4.month(), 2018.year());

    rtc.set_date_time(date, time);

    timer.delay_ms(1000_u32);
    timer.delay_ms(1000_u32);
    timer.delay_ms(1000_u32);

    let (rtc_date, rtc_time) = rtc.get_date_time();

    println!("Time: {:?}", defmt::Debug2Format(&rtc_time));
    println!("Date: {:?}", defmt::Debug2Format(&rtc_date));
    println!("Good bye!");
    loop {
        continue;
    }
}
