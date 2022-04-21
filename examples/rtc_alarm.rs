//! Sets an RTC alarm

#![no_std]
#![no_main]

extern crate cortex_m;
#[macro_use]
extern crate cortex_m_rt as rt;
extern crate cortex_m_semihosting as sh;
extern crate panic_semihosting;
extern crate stm32l4xx_hal as hal;

use crate::hal::prelude::*;
use crate::hal::rcc::{ClockSecuritySystem, CrystalBypass};
use crate::hal::rtc::{Event, Rtc, RtcClockSource, RtcConfig};
use crate::rt::ExceptionFrame;
use cortex_m::interrupt::{free, Mutex};
use time::{Date, Time};

use crate::sh::hio;
use core::{cell::RefCell, convert::TryInto, fmt::Write, ops::DerefMut};
use hal::interrupt;
use hal::pac;
use pac::NVIC;

static RTC: Mutex<RefCell<Option<Rtc>>> = Mutex::new(RefCell::new(None));

#[entry]
fn main() -> ! {
    let mut hstdout = hio::hstdout().unwrap();

    writeln!(hstdout, "Hello, world!").unwrap();

    let mut dp = hal::stm32::Peripherals::take().unwrap();
    dp.RCC.apb2enr.write(|w| w.syscfgen().set_bit());

    let mut flash = dp.FLASH.constrain();
    let mut rcc = dp.RCC.constrain();
    let mut pwr = dp.PWR.constrain(&mut rcc.apb1r1);

    // Try a different clock configuration
    rcc.cfgr
        .lse(CrystalBypass::Disable, ClockSecuritySystem::Disable)
        .freeze(&mut flash.acr, &mut pwr);

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

    // Set alarm A for 1 minute
    // let alarm_time = Time::new(21.hours(), 57.minutes(), 37.secs(), 0.micros(), false);
    // let alarm_date = date;
    // rtc.set_alarm(Alarm::AlarmA, alarm_date, alarm_time);
    let mut wkp = rtc.wakeup_timer();
    wkp.start(15_u32);
    rtc.listen(&mut dp.EXTI, Event::WakeupTimer);

    unsafe {
        NVIC::unmask(pac::Interrupt::RTC_WKUP);
    }

    free(|cs| {
        RTC.borrow(cs).replace(Some(rtc));
    });

    loop {
        continue;
    }
}

#[interrupt]
fn RTC_WKUP() {
    let mut hstdout = hio::hstdout().unwrap();
    free(|cs| {
        let mut rtc_ref = RTC.borrow(cs).borrow_mut();
        if let Some(ref mut rtc) = rtc_ref.deref_mut() {
            if rtc.check_interrupt(Event::WakeupTimer, true) {
                writeln!(hstdout, "RTC Wakeup!").unwrap();
            }
        }
    });
}

#[exception]
unsafe fn HardFault(ef: &ExceptionFrame) -> ! {
    panic!("{:#?}", ef);
}
