#![no_main]
#![no_std]

use panic_rtt_target as _;

use cortex_m_rt::entry;
use rtt_target::{rprint, rprintln};
use stm32l4xx_hal::{adc::ADC, delay::Delay, pac, prelude::*};

#[entry]
fn main() -> ! {
    rtt_target::rtt_init_print!();
    rprint!("Initializing...");

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

    rprintln!(" done.");

    loop {
        let value = adc.read(&mut a1).unwrap();
        rprintln!("Value: {}", value);
    }
}
