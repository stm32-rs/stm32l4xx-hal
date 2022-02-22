//! Test with
//! cargo flash --release --features stm32l4x2,rt --chip STM32L452RETx --example lptim_rtic --target thumbv7em-none-eabi
//! on Nucleo-L452-P board
#![deny(unsafe_code)]
#![no_main]
#![no_std]
extern crate panic_rtt_target;

use rtt_target::rprintln;
use stm32l4xx_hal::{
    flash::ACR,
    gpio::{gpiob::PB13, Output, PinState, PushPull},
    lptimer::{ClockSource, Event, LowPowerTimer, LowPowerTimerConfig, PreScaler},
    pac::LPTIM1,
    prelude::*,
    pwr::Pwr,
    rcc::{ClockSecuritySystem, Clocks, CrystalBypass, RccExt, CFGR},
};

// this is the LD4 on Nucleo-L452-P
type Led = PB13<Output<PushPull>>;
type Timer = LowPowerTimer<LPTIM1>;

pub fn configure_clock_tree(cfgr: CFGR, acr: &mut ACR, pwr: &mut Pwr) -> Clocks {
    cfgr.lse(CrystalBypass::Disable, ClockSecuritySystem::Disable)
        .sysclk(80.MHz())
        .freeze(acr, pwr)
}

#[rtic::app(device = stm32l4xx_hal::pac, peripherals = true)]
const APP: () = {
    struct Resources {
        led: Led,
        lptim: Timer,
    }

    #[init]
    fn init(ctx: init::Context) -> init::LateResources {
        rtt_target::rtt_init_print!();

        rprintln!("Init start");
        let device = ctx.device;
        // Configure the clock.
        let mut rcc = device.RCC.constrain();
        let mut flash = device.FLASH.constrain();
        let mut pwr = device.PWR.constrain(&mut rcc.apb1r1);
        let mut gpiob = device.GPIOB.split(&mut rcc.ahb2);
        let clocks = configure_clock_tree(rcc.cfgr, &mut flash.acr, &mut pwr);

        // PB13 is a user led on Nucleo-L452-P board
        let led = gpiob.pb13.into_push_pull_output_in_state(
            &mut gpiob.moder,
            &mut gpiob.otyper,
            PinState::Low,
        );
        rprintln!("Clocks = {:#?}", clocks);
        let lptim_config = LowPowerTimerConfig::default()
            .clock_source(ClockSource::LSE)
            .prescaler(PreScaler::U1)
            .arr_value(32_768u16); // roughly 1s
        let mut lptim = LowPowerTimer::lptim1(
            device.LPTIM1,
            lptim_config,
            &mut rcc.apb1r1,
            &mut rcc.ccipr,
            clocks,
        );
        lptim.listen(Event::AutoReloadMatch);
        init::LateResources { lptim, led }
    }

    #[task(binds = LPTIM1, resources = [lptim, led])]
    fn timer_tick(ctx: timer_tick::Context) {
        let timer_tick::Resources { lptim, led } = ctx.resources;
        if lptim.is_event_triggered(Event::AutoReloadMatch) {
            lptim.clear_event_flag(Event::AutoReloadMatch);
            rprintln!("LPTIM1 tick");

            led.toggle();
        }
    }

    #[idle]
    fn idle(_ctx: idle::Context) -> ! {
        loop {
            // See https://github.com/probe-rs/probe-rs/issues/350
            core::hint::spin_loop();
        }
    }

    extern "C" {
        fn LCD();
    }
};
