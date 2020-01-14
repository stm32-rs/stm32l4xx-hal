#![no_std]
#![no_main]

extern crate panic_semihosting; // logs messages to the host stderr; requires a debugger
extern crate stm32l4xx_hal as hal;

use crate::hal::{
    gpio::{gpioc::PC13, Edge, ExtiPin, Input, PullUp},
    interrupt,
    prelude::*,
    rcc::{Clocks, Rcc},
    stm32,
};
use core::cell::{Cell, RefCell};
use core::fmt;
use core::ops::DerefMut;
use cortex_m::interrupt::{free, CriticalSection, Mutex};
use cortex_m_rt::entry;

// Set up global state. It's all mutexed up for concurrency safety.
static BUTTON: Mutex<RefCell<Option<PC13<Input<PullUp>>>>> = Mutex::new(RefCell::new(None));

#[entry]
fn main() -> ! {
    if let (Some(mut dp), Some(cp)) = (stm32::Peripherals::take(), cortex_m::Peripherals::take()) {
        dp.RCC.apb2enr.write(|w| w.syscfgen().enabled());

        let rcc = dp.RCC.constrain();
        let clocks = rcc
            .cfgr
            .hclk(48.mhz())
            .sysclk(48.mhz())
            .pclk1(24.mhz())
            .pclk2(24.mhz())
            .freeze();

        // Create a button input with an interrupt
        let gpioc = dp.GPIOC.split();
        let mut board_btn = gpioc
            .pc13
            .into_pull_up_input(&mut gpioc.moder, &mut gpioc.pupdr);
        board_btn.make_interrupt_source(&mut dp.SYSCFG);
        board_btn.enable_interrupt(&mut dp.EXTI);
        board_btn.trigger_on_edge(&mut dp.EXTI, Edge::FALLING);

        // Enable interrupts
        stm32::NVIC::unpend(hal::interrupt::EXTI15_10);
        unsafe {
            stm32::NVIC::unmask(hal::interrupt::EXTI15_10);
        };

        free(|cs| {
            BUTTON.borrow(cs).replace(Some(board_btn));
        });

        loop {}
    }

    loop {}
}

#[interrupt]
fn EXTI15_10() {
    free(|cs| {
        let mut btn_ref = BUTTON.borrow(cs).borrow_mut();
        if let Some(ref mut btn) = btn_ref.deref_mut() {
            if btn.check_interrupt() {

                // if we don't clear this bit, the ISR would trigger indefinitely
                btn.clear_interrupt_pending_bit();
            }
        }
    });
}
