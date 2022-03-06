//! Test the serial interface
//!
//! This example requires you to short (connect) the TX and RX pins.
#![no_main]
#![no_std]

use cortex_m_rt::entry;
use panic_probe as _;
use stm32l4xx_hal::{self as hal, prelude::*, tsc::Tsc};

#[entry]
fn main() -> ! {
    let p = hal::stm32::Peripherals::take().unwrap();
    // let cp = cortex_m::Peripherals::take().unwrap();

    let mut flash = p.FLASH.constrain();
    let mut rcc = p.RCC.constrain();
    let mut pwr = p.PWR.constrain(&mut rcc.apb1r1);
    // let mut gpioa = p.GPIOA.split(&mut rcc.ahb2);
    let mut gpiob = p.GPIOB.split(&mut rcc.ahb2);

    // clock configuration using the default settings (all clocks run at 8 MHz)
    let _clocks = rcc.cfgr.freeze(&mut flash.acr, &mut pwr);

    // let mut delay = Delay::new(cp.SYST, clocks);
    let mut led = gpiob
        .pb3
        .into_push_pull_output(&mut gpiob.moder, &mut gpiob.otyper);

    let sample_pin =
        gpiob
            .pb4
            .into_alternate_open_drain(&mut gpiob.moder, &mut gpiob.otyper, &mut gpiob.afrl);
    let mut c1 = gpiob
        .pb5
        .into_alternate(&mut gpiob.moder, &mut gpiob.otyper, &mut gpiob.afrl);
    let mut c2 = gpiob
        .pb6
        .into_alternate(&mut gpiob.moder, &mut gpiob.otyper, &mut gpiob.afrl);
    // let mut c3 = gpiob.pb7.into_alternate(&mut gpiob.moder, &mut gpiob.otyper, &mut gpiob.afrl);

    // , (c1, c2, c3)
    let tsc = Tsc::tsc(p.TSC, sample_pin, &mut rcc.ahb1, None);

    let baseline = tsc.acquire(&mut c1).unwrap();
    let threshold = (baseline / 100) * 60;

    loop {
        let touched = tsc.acquire(&mut c1).unwrap();
        let _touched_c2 = tsc.acquire(&mut c2).unwrap();
        // try and pass c1, it will detect an error!
        let _touched_c2_again = tsc.read(&mut c2).unwrap();
        if touched < threshold {
            led.set_high();
        } else {
            led.set_low();
        }
    }
}
