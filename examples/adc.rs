#![no_main]
#![no_std]

use cortex_m_rt::entry;
use defmt::println;
use defmt_rtt as _;
use panic_probe as _;
use stm32l4xx_hal::{adc::ADC, delay::Delay, pac, prelude::*};

#[entry]
fn main() -> ! {
    println!("Initializing...");

    let cp = pac::CorePeripherals::take().unwrap();
    let dp = pac::Peripherals::take().unwrap();

    let mut rcc = dp.RCC.constrain();
    let mut flash = dp.FLASH.constrain();
    let mut pwr = dp.PWR.constrain(&mut rcc.apb1r1);

    let clocks = rcc.cfgr.freeze(&mut flash.acr, &mut pwr);

    let mut delay = Delay::new(cp.SYST, clocks);
    let mut adc = ADC::new(
        dp.ADC1,
        dp.ADC_COMMON,
        &mut rcc.ahb2,
        &mut rcc.ccipr,
        &mut delay,
    );

    let mut gpioc = dp.GPIOC.split(&mut rcc.ahb2);
    let mut a1 = gpioc.pc0.into_analog(&mut gpioc.moder, &mut gpioc.pupdr);

    println!("Initializing done.");

    loop {
        let value = adc.read(&mut a1).unwrap();
        println!("Value: {}", value);
    }
}
