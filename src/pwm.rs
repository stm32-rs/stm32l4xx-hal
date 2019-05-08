//! # Pulse Width Modulation

use cast::{u16, u32};
use core::marker::PhantomData;
use core::mem;

use crate::hal;
use crate::stm32::TIM2;

use crate::gpio::gpioa::{PA0, PA1, PA2, PA3};
use crate::gpio::{Alternate, Output, PushPull, AF1};
use crate::rcc::{Clocks, APB1R1};
use crate::time::Hertz;

// NB: REMAP is not implemented!
pub trait Pins<TIM> {
    // const REMAP: u8;
    const C1: bool;
    const C2: bool;
    const C3: bool;
    const C4: bool;
    type Channels;
}

impl Pins<TIM2>
    for (
        PA0<Alternate<AF1, Output<PushPull>>>,
        PA1<Alternate<AF1, Output<PushPull>>>,
        PA2<Alternate<AF1, Output<PushPull>>>,
        PA3<Alternate<AF1, Output<PushPull>>>,
    )
{
    const C1: bool = true;
    const C2: bool = true;
    const C3: bool = true;
    const C4: bool = true;
    type Channels = (Pwm<TIM2, C1>, Pwm<TIM2, C2>, Pwm<TIM2, C3>, Pwm<TIM2, C4>);
}

// useful for RGB LED
impl Pins<TIM2>
    for (
        PA2<Alternate<AF1, Output<PushPull>>>,
        PA3<Alternate<AF1, Output<PushPull>>>,
        PA1<Alternate<AF1, Output<PushPull>>>,
    )
{
    const C1: bool = false;
    const C2: bool = true;
    const C3: bool = true;
    const C4: bool = true;
    type Channels = (Pwm<TIM2, C3>, Pwm<TIM2, C4>, Pwm<TIM2, C2>);
}

impl Pins<TIM2> for PA0<Alternate<AF1, Output<PushPull>>> {
    const C1: bool = true;
    const C2: bool = false;
    const C3: bool = false;
    const C4: bool = false;
    type Channels = Pwm<TIM2, C1>;
}

impl Pins<TIM2> for PA1<Alternate<AF1, Output<PushPull>>> {
    const C1: bool = false;
    const C2: bool = true;
    const C3: bool = false;
    const C4: bool = false;
    type Channels = Pwm<TIM2, C2>;
}

impl Pins<TIM2> for PA2<Alternate<AF1, Output<PushPull>>> {
    const C1: bool = false;
    const C2: bool = false;
    const C3: bool = true;
    const C4: bool = false;
    type Channels = Pwm<TIM2, C3>;
}

impl Pins<TIM2> for PA3<Alternate<AF1, Output<PushPull>>> {
    const C1: bool = false;
    const C2: bool = false;
    const C3: bool = false;
    const C4: bool = true;
    type Channels = Pwm<TIM2, C4>;
}

pub trait PwmExt: Sized {
    fn pwm<PINS, T>(
        self,
        _: PINS,
        frequency: T,
        clocks: Clocks,
        apb: &mut APB1R1,
    ) -> PINS::Channels
    where
        PINS: Pins<Self>,
        T: Into<Hertz>;
}

impl PwmExt for TIM2 {
    fn pwm<PINS, T>(self, _pins: PINS, freq: T, clocks: Clocks, apb: &mut APB1R1) -> PINS::Channels
    where
        PINS: Pins<Self>,
        T: Into<Hertz>,
    {
        // TODO: check if this is really not needed (in the f1xx examples value
        //       of remap is 0x0). if so, what's afio.mapr on l4xx?
        //
        // mapr.mapr()
        //     .modify(|_, w| unsafe { w.tim2_remap().bits(PINS::REMAP) });

        tim2(self, _pins, freq.into(), clocks, apb)
    }
}

pub struct Pwm<TIM, CHANNEL> {
    _channel: PhantomData<CHANNEL>,
    _tim: PhantomData<TIM>,
}

pub struct C1;
pub struct C2;
pub struct C3;
pub struct C4;

macro_rules! hal {
    ($($TIMX:ident: ($timX:ident, $timXen:ident, $timXrst:ident, $apb:ident),)+) => {
        $(
            fn $timX<PINS>(
                tim: $TIMX,
                _pins: PINS,
                freq: Hertz,
                clocks: Clocks,
                apb: &mut $apb,
            ) -> PINS::Channels
            where
                PINS: Pins<$TIMX>,
            {
                apb.enr().modify(|_, w| w.$timXen().set_bit());
                apb.rstr().modify(|_, w| w.$timXrst().set_bit());
                apb.rstr().modify(|_, w| w.$timXrst().clear_bit());

                if PINS::C1 {
                    tim.ccmr1_output
                        .modify(|_, w| unsafe { w.oc1pe().set_bit().oc1m().bits(6) });
                }

                if PINS::C2 {
                    tim.ccmr1_output
                        .modify(|_, w| unsafe { w.oc2pe().set_bit().oc2m().bits(6) });
                }

                if PINS::C3 {
                    tim.ccmr2_output
                        .modify(|_, w| unsafe { w.oc3pe().set_bit().oc3m().bits(6) });
                }

                if PINS::C4 {
                    tim.ccmr2_output
                        .modify(|_, w| unsafe { w.oc4pe().set_bit().oc4m().bits(6) });
                }
                let clk = clocks.pclk1().0;
                let freq = freq.0;
                let ticks = clk / freq;

                // maybe this is all u32? also, why no `- 1` vs `timer.rs`?
                let psc = u16(ticks / (1 << 16)).unwrap();
                tim.psc.write(|w| unsafe { w.psc().bits(psc) });
                let arr = u16(ticks / u32(psc + 1)).unwrap();
                tim.arr.write(|w| { w.arr().bits(u32(arr)) });

                tim.cr1.write(|w| unsafe {
                    w.cms()
                        .bits(0b00)
                        .dir().clear_bit()
                        .opm().clear_bit()
                        .cen().set_bit()
                });

                unsafe { mem::uninitialized() }
            }

            impl hal::PwmPin for Pwm<$TIMX, C1> {
                type Duty = u32;

                fn disable(&mut self) {
                    unsafe { (*$TIMX::ptr()).ccer.write(|w| w.cc1e().clear_bit()) }
                }

                fn enable(&mut self) {
                    unsafe { (*$TIMX::ptr()).ccer.write(|w| w.cc1e().set_bit()) }
                }

                fn get_duty(&self) -> Self::Duty {
                    unsafe { (*$TIMX::ptr()).ccr1.read().ccr().bits() }
                }

                fn get_max_duty(&self) -> Self::Duty {
                    unsafe { (*$TIMX::ptr()).arr.read().arr().bits() }
                }

                fn set_duty(&mut self, duty: Self::Duty) {
                    unsafe { (*$TIMX::ptr()).ccr1.write(|w| w.ccr().bits(duty)) }
                }
            }

            impl hal::PwmPin for Pwm<$TIMX, C2> {
                type Duty = u32;

                fn disable(&mut self) {
                    unsafe { (*$TIMX::ptr()).ccer.write(|w| w.cc2e().clear_bit()) }
                }

                fn enable(&mut self) {
                    unsafe { (*$TIMX::ptr()).ccer.write(|w| w.cc2e().set_bit()) }
                }

                fn get_duty(&self) -> Self::Duty {
                    unsafe { (*$TIMX::ptr()).ccr2.read().ccr().bits() }
                }

                fn get_max_duty(&self) -> Self::Duty {
                    unsafe { (*$TIMX::ptr()).arr.read().arr().bits() }
                }

                fn set_duty(&mut self, duty: Self::Duty) {
                    unsafe { (*$TIMX::ptr()).ccr2.write(|w| w.ccr().bits(duty)) }
                }
            }

            impl hal::PwmPin for Pwm<$TIMX, C3> {
                type Duty = u32;

                fn disable(&mut self) {
                    unsafe { (*$TIMX::ptr()).ccer.write(|w| w.cc3e().clear_bit()) }
                }

                fn enable(&mut self) {
                    unsafe { (*$TIMX::ptr()).ccer.write(|w| w.cc3e().set_bit()) }
                }

                fn get_duty(&self) -> Self::Duty {
                    unsafe { (*$TIMX::ptr()).ccr3.read().ccr().bits() }
                }

                fn get_max_duty(&self) -> Self::Duty {
                    unsafe { (*$TIMX::ptr()).arr.read().arr().bits() }
                }

                fn set_duty(&mut self, duty: Self::Duty) {
                    unsafe { (*$TIMX::ptr()).ccr3.write(|w| w.ccr().bits(duty)) }
                }
            }

            impl hal::PwmPin for Pwm<$TIMX, C4> {
                type Duty = u32;

                fn disable(&mut self) {
                    unsafe { (*$TIMX::ptr()).ccer.write(|w| w.cc4e().clear_bit()) }
                }

                fn enable(&mut self) {
                    unsafe { (*$TIMX::ptr()).ccer.write(|w| w.cc4e().set_bit()) }
                }

                fn get_duty(&self) -> Self::Duty {
                    unsafe { (*$TIMX::ptr()).ccr4.read().ccr().bits() }
                }

                fn get_max_duty(&self) -> Self::Duty {
                    unsafe { (*$TIMX::ptr()).arr.read().arr().bits() }
                }

                fn set_duty(&mut self, duty: Self::Duty) {
                    unsafe { (*$TIMX::ptr()).ccr4.write(|w| w.ccr().bits(duty)) }
                }
            }
        )+
    }
}

hal! {
    TIM2: (tim2, tim2en, tim2rst, APB1R1),
}
