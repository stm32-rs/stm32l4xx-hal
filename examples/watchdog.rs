//! Example of watchdog timer
#![deny(unsafe_code)]
// #![deny(warnings)]
#![no_std]
#![no_main]

use crate::hal::delay::Delay;
use crate::hal::prelude::*;
use crate::hal::time::MilliSeconds;
use crate::hal::watchdog::IndependentWatchdog;
use cortex_m_rt::{entry, exception, ExceptionFrame};
use cortex_m_semihosting as sh;
use panic_semihosting as _;
use stm32l4xx_hal as hal;

use crate::sh::hio;
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
    let clocks = rcc.cfgr.lsi(true).freeze(&mut flash.acr, &mut pwr);

    let mut timer = Delay::new(cp.SYST, clocks);

    // Initiate the independent watchdog timer
    let mut watchdog = IndependentWatchdog::new(dp.IWDG);
    watchdog.stop_on_debug(&dp.DBGMCU, true);

    // Start the independent watchdog timer
    watchdog.start(MilliSeconds(1020));
    timer.delay_ms(1000_u32);

    // Feed the independent watchdog timer
    watchdog.feed();
    timer.delay_ms(1000_u32);

    watchdog.feed();
    timer.delay_ms(1000_u32);

    watchdog.feed();
    writeln!(hstdout, "Good bye!").unwrap();
    loop {
        continue;
    }
}

#[exception]
fn HardFault(ef: &ExceptionFrame) -> ! {
    panic!("{:#?}", ef);
}
