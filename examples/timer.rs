//! Blinks an LED

#![no_std]
#![no_main]

use cortex_m::peripheral::NVIC;
use cortex_m_rt::entry;
use defmt::println;
use defmt_rtt as _;
use panic_probe as _;
use stm32l4xx_hal::{
    self as hal, interrupt,
    prelude::*,
    timer::{Event, Timer},
};

#[entry]
fn main() -> ! {
    println!("Hello, world!");

    // let cp = cortex_m::Peripherals::take().unwrap();
    let dp = hal::stm32::Peripherals::take().unwrap();

    let mut flash = dp.FLASH.constrain(); // .constrain();
    let mut rcc = dp.RCC.constrain();
    let mut pwr = dp.PWR.constrain(&mut rcc.apb1r1);

    // Try a different clock configuration
    let clocks = rcc.cfgr.freeze(&mut flash.acr, &mut pwr);

    // let mut gpiob = dp.GPIOB.split(&mut rcc.ahb2);
    // let mut led = gpiob.pb3.into_push_pull_output(&mut gpiob.moder, &mut gpiob.otyper);

    unsafe { NVIC::unmask(hal::stm32::Interrupt::TIM7) };
    let mut timer = Timer::tim6(dp.TIM6, 1.Hz(), clocks, &mut rcc.apb1r1);
    timer.listen(Event::TimeOut);

    loop {
        continue;
    }
}

#[interrupt]
fn TIM7() {
    static mut COUNT: u32 = 0;
    *COUNT += 1;
    println!("TIM7 interrupt");
}
