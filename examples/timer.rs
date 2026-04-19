//! Blinks an LED

#![no_std]
#![no_main]

extern crate cortex_m;
#[macro_use]
extern crate cortex_m_rt as rt;
extern crate cortex_m_semihosting as sh;
extern crate panic_semihosting;
extern crate stm32l4xx_hal as hal;

use crate::hal::interrupt;
use crate::hal::prelude::*;
use crate::hal::timer::{Event, Timer};
use crate::rt::entry;
use crate::rt::ExceptionFrame;
use cortex_m::interrupt::free;
use cortex_m::interrupt::Mutex;
use cortex_m::peripheral::NVIC;
use hal::gpio::Output;
use hal::gpio::PushPull;
use hal::gpio::PB3;
use hal::pac::TIM6;

use crate::sh::hio;
use core::cell::RefCell;
use core::fmt::Write;
use core::ops::DerefMut;

struct LedAndTimer {
    led: PB3<Output<PushPull>>,
    timer: Timer<TIM6>,
}

static LED_AND_TIMER: Mutex<RefCell<Option<LedAndTimer>>> = Mutex::new(RefCell::new(None));

#[entry]
fn main() -> ! {
    let mut hstdout = hio::hstdout().unwrap();
    writeln!(hstdout, "Hello, world!").unwrap();

    // let cp = cortex_m::Peripherals::take().unwrap();
    let dp = hal::stm32::Peripherals::take().unwrap();

    let mut flash = dp.FLASH.constrain(); // .constrain();
    let mut rcc = dp.RCC.constrain();
    let mut pwr = dp.PWR.constrain(&mut rcc.apb1r1);

    // Try a different clock configuration
    let clocks = rcc.cfgr.freeze(&mut flash.acr, &mut pwr);

    unsafe { NVIC::unmask(hal::stm32::Interrupt::TIM6_DACUNDER) };
    let mut timer = Timer::tim6(dp.TIM6, 1.Hz(), clocks, &mut rcc.apb1r1);
    timer.listen(Event::TimeOut);

    let mut gpiob = dp.GPIOB.split(&mut rcc.ahb2);
    let led = gpiob
        .pb3
        .into_push_pull_output(&mut gpiob.moder, &mut gpiob.otyper);

    free(|cs| {
        LED_AND_TIMER
            .borrow(cs)
            .replace(Some(LedAndTimer { led, timer }))
    });

    loop {
        continue;
    }
}

#[interrupt]
fn TIM6_DACUNDER() {
    free(|cs| {
        let mut led_and_timer_ref = LED_AND_TIMER.borrow(cs).borrow_mut();

        if let Some(led_and_timer) = led_and_timer_ref.deref_mut() {
            led_and_timer.led.toggle();
            led_and_timer.timer.clear_interrupt(Event::TimeOut);
        }
    });
}

#[exception]
unsafe fn HardFault(ef: &ExceptionFrame) -> ! {
    panic!("{:#?}", ef);
}
