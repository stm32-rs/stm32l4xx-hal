//! Blinks an LED
#![no_std]
#![no_main]

use cortex_m_rt::{entry, exception, ExceptionFrame};
use defmt::println;
use defmt_rtt as _;
use panic_probe as _;
use stm32l4xx_hal as hal; // hal
use stm32l4xx_hal::{delay::Delay, prelude::*};

#[entry]
fn main() -> ! {
    println!("Hello, world!");

    let cp = cortex_m::Peripherals::take().unwrap();
    let dp = hal::stm32::Peripherals::take().unwrap();

    let mut flash = dp.FLASH.constrain();
    let mut rcc = dp.RCC.constrain();
    let mut pwr = dp.PWR.constrain(&mut rcc.apb1r1);

    // Try a different clock configuration
    let clocks = rcc.cfgr.freeze(&mut flash.acr, &mut pwr);
    // let clocks = rcc.cfgr
    //     .sysclk(64.MHz())
    //     .pclk1(32.MHz())
    //     .freeze(&mut flash.acr);

    let mut gpiob = dp.GPIOB.split(&mut rcc.ahb2);
    let mut led = gpiob
        .pb3
        .into_push_pull_output(&mut gpiob.moder, &mut gpiob.otyper);

    // simple blink loop counting clock cycles for delays
    let mut timer = Delay::new(cp.SYST, clocks);
    loop {
        timer.delay_ms(1000_u32);
        led.set_high();
        println!("LED on");
        timer.delay_ms(1000_u32);
        led.set_low();
        println!("LED off");
    }
}

#[exception]
unsafe fn HardFault(ef: &ExceptionFrame) -> ! {
    panic!("{:#?}", ef);
}
