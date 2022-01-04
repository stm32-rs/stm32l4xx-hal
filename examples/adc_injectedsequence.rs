#![no_main]
#![no_std]

extern crate stm32l4;

use panic_rtt_target as _;
use rtt_target::{rprintln, rtt_init_print};
use stm32l4xx_hal::{
    adc::{Jsequence, SampleTime, ADC},
    delay::DelayCM,
    pac::TIM2,
    prelude::*,
    time::Hertz,
    timer::Timer,
};

use rtic::app;

#[app(device = stm32l4xx_hal::stm32, peripherals = true, monotonic = rtic::cyccnt::CYCCNT)]
const APP: () = {
    // RTIC app is written in here!

    struct Resources {
        adc: stm32l4xx_hal::adc::ADC,
    }

    #[init]
    fn init(cx: init::Context) -> init::LateResources {
        rtt_init_print!();

        rprintln!("Hello from init!");

        let cp = cx.core;
        let mut dcb = cp.DCB;
        let mut dwt = cp.DWT;

        dcb.enable_trace();
        dwt.enable_cycle_counter();

        let pac = cx.device;

        let mut rcc = pac.RCC.constrain();
        let mut flash = pac.FLASH.constrain();
        let mut pwr = pac.PWR.constrain(&mut rcc.apb1r1);

        //
        // Initialize the clocks
        //
        let clocks = rcc
            .cfgr
            .sysclk(Hertz(80_000_000))
            .freeze(&mut flash.acr, &mut pwr);

        let mut delay = DelayCM::new(clocks);

        let mut adc = ADC::new(
            pac.ADC1,
            pac.ADC_COMMON,
            &mut rcc.ahb2,
            &mut rcc.ccipr,
            &mut delay,
        );

        let mut temp_pin = adc.enable_temperature(&mut delay);

        let mut gpioc = pac.GPIOC.split(&mut rcc.ahb2);
        let mut pc0 = gpioc.pc0.into_analog(&mut gpioc.moder, &mut gpioc.pupdr);

        adc.configure_jsequence(&mut temp_pin, Jsequence::One, SampleTime::Cycles247_5);
        adc.configure_jsequence(&mut pc0, Jsequence::Two, SampleTime::Cycles247_5);
        adc.configure_jsequence(&mut temp_pin, Jsequence::Three, SampleTime::Cycles640_5);
        adc.configure_jsequence(&mut temp_pin, Jsequence::Four, SampleTime::Cycles12_5);

        // optional oversampling settings  (added, otherwise the first sampling is wrong due to adc errata in L4)
        adc.set_oversampling_ratio(7);
        adc.set_oversampling_shift(8);
        adc.inject_oversampling_enable();

        // set injection trigger source to timer2 TRGO and rising edge
        adc.set_inject_channel(2_u8, 1_u8);

        adc.start_injected_sequence();

        // start the timer
        let mut _timer = Timer::tim2(pac.TIM2, 1.hz(), clocks, &mut rcc.apb1r1);

        // Set timer output to trigger signal to ADC for start of sampling sequence
        unsafe {
            // get pointer of timer 2
            let tim = &(*TIM2::ptr());
            // config master mode selection to TRGO to Compare Pulse of timer2
            tim.cr2.modify(|_, w| w.mms().bits(3_u8));
            tim.dier.write(|w| w.ude().set_bit());
        }

        init::LateResources { adc }
    }

    #[idle]
    fn idle(_cx: idle::Context) -> ! {
        loop {
            cortex_m::asm::nop();
        }
    }

    // when sequence of adc has finished, the data can be fetched from the
    // injected data registers
    #[task(binds = ADC1_2, resources = [adc] )]
    fn adc1_irg(cx: adc1_irg::Context) {
        let adc1 = cx.resources.adc;

        let jdr1_val = adc1.get_injected_jdr(1_u8);
        let jdr2_val = adc1.get_injected_jdr(2_u8);
        let jdr3_val = adc1.get_injected_jdr(3_u8);
        let jdr4_val = adc1.get_injected_jdr(4_u8);
        rprintln!("jdr1: {}", jdr1_val);
        rprintln!("jdr2: {}", jdr2_val);
        rprintln!("jdr3: {}", jdr3_val);
        rprintln!("jdr4: {}", jdr4_val);

        adc1.set_jeos();
    }
};
