//! Blinks an LED
#![no_std]
#![no_main]

use cortex_m_rt::{entry, exception, ExceptionFrame};
// Using RTT for debug + panic handler
use panic_rtt_target as _; // panic handler
use rtt_target::{rprintln, rtt_init_print}; // RTT functions
use stm32l4xx_hal as hal; // hal
use stm32l4xx_hal::{delay::Delay, prelude::*};

#[entry]
fn main() -> ! {
    rtt_init_print!();
    rprintln!("Hello, world!");

    let cp = cortex_m::Peripherals::take().unwrap();
    let dp = hal::stm32::Peripherals::take().unwrap();

    let mut flash = dp.FLASH.constrain();
    let mut rcc = dp.RCC.constrain();
    let mut pwr = dp.PWR.constrain(&mut rcc.apb1r1);

    // Try a different clock configuration
    let clocks = rcc.cfgr.hclk(8.MHz()).freeze(&mut flash.acr, &mut pwr);
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
        timer.delay_ms(1000_u32);
        led.set_low();
    }
}

#[exception]
unsafe fn HardFault(ef: &ExceptionFrame) -> ! {
    panic!("{:#?}", ef);
}
