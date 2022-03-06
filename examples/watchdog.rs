//! Example of watchdog timer
#![no_std]
#![no_main]

use cortex_m_rt::entry;
use defmt::println;
use panic_probe as _;
use stm32l4xx_hal::{self as hal, delay::Delay, prelude::*, watchdog::IndependentWatchdog};

#[entry]
fn main() -> ! {
    println!("Hello, world!");

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
    watchdog.start(1020.millis());
    timer.delay_ms(1000_u32);

    // Feed the independent watchdog timer
    watchdog.feed();
    timer.delay_ms(1000_u32);

    watchdog.feed();
    timer.delay_ms(1000_u32);

    watchdog.feed();
    println!("Good bye!");
    // watchdog will reset after 1020 milliseconds
    loop {
        continue;
    }
}
