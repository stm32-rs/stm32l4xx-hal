#![deny(warnings)]
#![deny(unsafe_code)]
#![no_main]
#![no_std]

// use rtt_target::{rprintln, rtt_init_print};

// currently only works with these devices
// #[cfg(any(feature = "stm32l476", feature = "stm32l486", feature = "stm32l496", feature = "stm32l4a6"))]

extern crate cortex_m;
extern crate cortex_m_rt as rt;
extern crate panic_halt;
extern crate stm32l4xx_hal as hal;

use hal::dac::GeneratorConfig;
use hal::delay::Delay;
use hal::hal::Direction;
use hal::prelude::*;
// use hal::rcc::Config;
use hal::stm32;
use rt::entry;

use crate::hal::dac::DacExt;
use crate::hal::dac::DacOut;

#[entry]
fn main() -> ! {
    // rtt_init_print!();

    let dp = stm32::Peripherals::take().expect("cannot take peripherals");
    let cp = cortex_m::Peripherals::take().expect("cannot take core peripherals");

    let mut rcc = dp.RCC.constrain();
    let mut flash = dp.FLASH.constrain();
    let mut pwr = dp.PWR.constrain(&mut rcc.apb1r1);
    let clocks = rcc.cfgr.freeze(&mut flash.acr, &mut pwr);
    let mut delay = Delay::new(cp.SYST, clocks);

    let mut gpioa = dp.GPIOA.split(&mut rcc.ahb2);
    let pa4 = gpioa.pa4.into_analog(&mut gpioa.moder, &mut gpioa.pupdr);
    let pa5 = gpioa.pa5.into_analog(&mut gpioa.moder, &mut gpioa.pupdr);

    let (dac0, dac1) = dp.DAC.constrain((pa4, pa5), &mut rcc.apb1r1);

    let mut dac = dac0.calibrate_buffer(&mut delay).enable();
    let mut generator = dac1.enable_generator(GeneratorConfig::noise(11));

    let mut dir = Direction::Upcounting;
    let mut val = 0;

    loop {
        generator.trigger();
        dac.set_value(val);
        match val {
            0 => dir = Direction::Upcounting,
            4095 => dir = Direction::Downcounting,
            _ => (),
        };

        match dir {
            Direction::Upcounting => val += 1,
            Direction::Downcounting => val -= 1,
        }
    }
}
