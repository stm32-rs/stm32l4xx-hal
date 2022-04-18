//! Sets an RTC alarm

#![no_std]
#![no_main]

use core::{cell::RefCell, ops::DerefMut};
use cortex_m::interrupt::{free, Mutex};
use cortex_m_rt::entry;
use defmt::println;
use defmt_rtt as _;
use panic_probe as _;
use stm32l4xx_hal::{
    self as hal,
    datetime::{Date, Time},
    device::NVIC,
    interrupt,
    prelude::*,
    rcc::{ClockSecuritySystem, CrystalBypass},
    rtc::{Event, Rtc, RtcClockSource, RtcConfig},
};

static RTC: Mutex<RefCell<Option<Rtc>>> = Mutex::new(RefCell::new(None));

#[entry]
fn main() -> ! {
    println!("Hello, world!");

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

    let time = Time::new(21.hours(), 57.minutes(), 32.secs(), 0.micros(), false);
    let date = Date::new(1.day(), 24.date(), 4.month(), 2018.year());

    rtc.set_date_time(date, time);

    // Set alarm A for 1 minute
    // let alarm_time = Time::new(21.hours(), 57.minutes(), 37.secs(), 0.micros(), false);
    // let alarm_date = date;
    // rtc.set_alarm(Alarm::AlarmA, alarm_date, alarm_time);
    let mut wkp = rtc.wakeup_timer();
    wkp.start(15_u32);
    rtc.listen(&mut dp.EXTI, Event::WakeupTimer);

    unsafe {
        NVIC::unmask(hal::pac::Interrupt::RTC_WKUP);
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
    free(|cs| {
        let mut rtc_ref = RTC.borrow(cs).borrow_mut();
        if let Some(ref mut rtc) = rtc_ref.deref_mut() {
            if rtc.check_interrupt(Event::WakeupTimer, true) {
                println!("RTC Wakeup!");
            }
        }
    });
}
