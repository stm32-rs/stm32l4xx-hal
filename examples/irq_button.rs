#![no_std]
#![no_main]

extern crate cortex_m;
extern crate cortex_m_rt as rt;
extern crate panic_semihosting;
extern crate stm32l4xx_hal as hal;

use crate::hal::{
    gpio::{gpioc::PC13, Edge, ExtiPin, Input},
    interrupt,
    prelude::*,
    stm32,
};
use core::cell::RefCell;
use core::ops::DerefMut;
use cortex_m::{
    interrupt::{free, Mutex},
    peripheral::NVIC,
};
use rt::entry;

// Set up global state. It's all mutexed up for concurrency safety.
static BUTTON: Mutex<RefCell<Option<PC13<Input>>>> = Mutex::new(RefCell::new(None));

#[entry]
fn main() -> ! {
    if let Some(mut dp) = stm32::Peripherals::take() {
        dp.RCC.apb2enr.write(|w| w.syscfgen().set_bit());

        let mut rcc = dp.RCC.constrain();
        let mut flash = dp.FLASH.constrain(); // .constrain();
        let mut pwr = dp.PWR.constrain(&mut rcc.apb1r1);

        rcc.cfgr
            .hclk(48.MHz())
            .sysclk(80.MHz())
            .pclk1(24.MHz())
            .pclk2(24.MHz())
            .freeze(&mut flash.acr, &mut pwr);

        // Create a button input with an interrupt
        let mut gpioc = dp.GPIOC.split(&mut rcc.ahb2);
        let mut board_btn = gpioc
            .pc13
            .into_pull_up_input(&mut gpioc.moder, &mut gpioc.pupdr);
        board_btn.make_interrupt_source(&mut dp.SYSCFG, &mut rcc.apb2);
        board_btn.enable_interrupt(&mut dp.EXTI);
        board_btn.trigger_on_edge(&mut dp.EXTI, Edge::Falling);

        // Enable interrupts
        unsafe {
            NVIC::unmask(stm32::Interrupt::EXTI15_10);
        }

        free(|cs| {
            BUTTON.borrow(cs).replace(Some(board_btn));
        });

        loop {
            continue;
        }
    }

    loop {
        continue;
    }
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
