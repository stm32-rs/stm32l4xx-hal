//! Testing comparator.
#![deny(unsafe_code)]
#![deny(warnings)]
#![no_main]
#![no_std]

extern crate panic_halt;

use cortex_m_rt::entry;
use rtt_target::rprintln;
use stm32l4xx_hal::{
    comp::{self, Comp, CompConfig, CompDevice},
    delay::Delay,
    pac,
    prelude::*,
};

#[entry]
fn main() -> ! {
    // Set Up RTT
    rtt_target::rtt_init_print!();

    // Set up ARM Cortex-M peripherals. These are common to many MCUs, including all STM32 ones.
    let cp = cortex_m::Peripherals::take().unwrap();
    // Set up peripherals specific to the microcontroller you're using.
    let dp = pac::Peripherals::take().unwrap();

    // Setting Up Clock
    let mut rcc = dp.RCC.constrain();
    let mut flash = dp.FLASH.constrain();
    let mut pwr = dp.PWR.constrain(&mut rcc.apb1r1);

    let clocks = rcc.cfgr.freeze(&mut flash.acr, &mut pwr);

    // Setting Up GPIO (Not really needed)
    let mut gpiob = dp.GPIOB.split(&mut rcc.ahb2);
    gpiob.pb2.into_analog(&mut gpiob.moder, &mut gpiob.pupdr);

    // Setting Up Delay
    let mut delay = Delay::new(cp.SYST, clocks);

    // Setting Up Comparator
    // Comparator Configuration
    let cfg = CompConfig {
        // No Hysterysis
        hyst: comp::Hysterisis::NoHysterisis,
        // Using internal Vref as negative input
        // e.g. (1.22V) in STM32L47xx, STM32L48xx, STM32L49xx and STM32L4Axx.
        // Consult Reference Manual for all negative input.
        inmsel: comp::InvertingInput::Vref,
        // Using Io2 as positive input
        // e.g. (PB2) for COMP1 in STM32L47xx, STM32L48xx, STM32L49xx and STM32L4Axx.
        // Consult Reference Manual for all positive input.
        inpsel: comp::NonInvertingInput::Io2,
        // Don't invert output high when inverting input < noninverting and etc.
        polarity: comp::OutputPolarity::NotInverted,
        // High Power Consumption (lowest propagation delay)
        pwrmode: comp::PowerMode::HighSpeed,
    };
    // Creating Comparator device using COMP1
    let mut comparator = Comp::new(CompDevice::One, cfg, &mut rcc.apb2);
    // Starting Comparator
    comparator.start().unwrap();

    loop {
        // Reading and Printing Output
        let output = comparator.get_output_level();
        rprintln!("{}", output);
        delay.delay_ms(1000u32);
    }
}
