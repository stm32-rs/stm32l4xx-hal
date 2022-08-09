//! # Pulse Width Modulation

use core::marker::PhantomData;
use core::mem;

use crate::hal;
use crate::stm32::{TIM1, TIM15, TIM2};

use crate::gpio::gpioa::{PA0, PA1, PA10, PA11, PA15, PA2, PA3, PA8, PA9};
use crate::gpio::gpiob::{PB10, PB11, PB14, PB3};
use crate::gpio::Alternate;
use crate::rcc::{Clocks, Enable, Reset, APB1R1, APB2};
use crate::time::Hertz;

// NB: REMAP is not implemented!
pub trait Pins<TIM> {
    // const REMAP: u8;
    const C1: bool = false;
    const C2: bool = false;
    const C3: bool = false;
    const C4: bool = false;
    type Channels;
}

macro_rules! pins_to_channels_mapping {
    ( $( $TIMX:ident: ( $($PINX:ident),+ ), ( $($ENCHX:ident),+ ), ( $($AF:literal),+ ); )+ ) => {
        $(
            #[allow(unused_parens)]
            impl<OTYPE> Pins<$TIMX> for ($($PINX<Alternate<OTYPE, $AF>>),+)
            {
                $(const $ENCHX: bool = true;)+
                type Channels = ($(Pwm<$TIMX, $ENCHX>),+);
            }
        )+
    };
}

pins_to_channels_mapping! {
    // TIM1
    TIM1: (PA8, PA9, PA10, PA11), (C1, C2, C3, C4), (1, 1, 1, 1);
    TIM1: (PA9, PA10, PA11), (C2, C3, C4), (1, 1, 1);
    TIM1: (PA8, PA10, PA11), (C1, C3, C4), (1, 1, 1);
    TIM1: (PA8, PA9, PA11), (C1, C2, C4), (1, 1, 1);
    TIM1: (PA8, PA9, PA10), (C1, C2, C3), (1, 1, 1);
    TIM1: (PA10, PA11), (C3, C4), (1, 1);
    TIM1: (PA9, PA11), (C2, C4), (1, 1);
    TIM1: (PA9, PA10), (C2, C3), (1, 1);
    TIM1: (PA8, PA11), (C1, C4), (1, 1);
    TIM1: (PA8, PA10), (C1, C3), (1, 1);
    TIM1: (PA8, PA9), (C1, C2), (1, 1);
    TIM1: (PA8), (C1), (1);
    TIM1: (PA9), (C2), (1);
    TIM1: (PA10), (C3), (1);
    TIM1: (PA11), (C4), (1);

    // TIM2
    TIM2: (PA0, PA1, PA2, PA3), (C1, C2, C3, C4), (1, 1, 1, 1);
    TIM2: (PA0, PA1, PA2, PB11), (C1, C2, C3, C4), (1, 1, 1, 1);
    TIM2: (PA15, PB3, PB10, PB11), (C1, C2, C3, C4), (1, 1, 1, 1);

    TIM2: (PA1, PA2, PA3), (C2, C3, C4), (1, 1, 1);
    TIM2: (PA0, PA2, PA3), (C1, C3, C4), (1, 1, 1);
    TIM2: (PA0, PA1, PA3), (C1, C2, C4), (1, 1, 1);
    TIM2: (PA0, PA1, PA2), (C1, C2, C3), (1, 1, 1);

    TIM2: (PB3, PB10, PB11), (C2, C3, C4), (1, 1, 1);
    TIM2: (PA15, PB10, PB11), (C1, C3, C4), (1, 1, 1);
    TIM2: (PA15, PB3, PB11), (C1, C2, C4), (1, 1, 1);
    TIM2: (PA15, PB3, PB10), (C1, C2, C3), (1, 1, 1);

    TIM2: (PA2, PA3), (C3, C4), (1, 1);
    TIM2: (PA1, PA3), (C2, C4), (1, 1);
    TIM2: (PA1, PA2), (C2, C3), (1, 1);
    TIM2: (PA0, PA3), (C1, C4), (1, 1);
    TIM2: (PA0, PA2), (C1, C3), (1, 1);
    TIM2: (PA0, PA1), (C1, C2), (1, 1);

    TIM2: (PB10, PB11), (C3, C4), (1, 1);
    TIM2: (PB3, PB11), (C2, C4), (1, 1);
    TIM2: (PB3, PB10), (C2, C3), (1, 1);
    TIM2: (PA15, PB11), (C1, C4), (1, 1);
    TIM2: (PA15, PB10), (C1, C3), (1, 1);
    TIM2: (PA15, PB3), (C1, C2), (1, 1);

    TIM2: (PA0), (C1), (1);
    TIM2: (PA1), (C2), (1);
    TIM2: (PA2), (C3), (1);
    TIM2: (PA3), (C4), (1);

    TIM2: (PA15), (C1), (1);
    TIM2: (PB3), (C2), (1);
    TIM2: (PB10), (C3), (1);
    TIM2: (PB11), (C4), (1);

    // TIM15 - TODO: The uncommented lines are awaiting PAC updates to be valid.
    TIM15: (PB14), (C1), (14);
    // TIM15: (PB15), (C2), (14);
    TIM15: (PA2), (C1), (14);
    // TIM15: (PA3), (C2), (14);
    // TIM15: (PB14, PB15), (C1, C2), (14, 14);
    // TIM15: (PB14, PA3), (C1, C2), (14, 14);
    // TIM15: (PA2, PB15), (C1, C2), (14, 14);
    // TIM15: (PA2, PA3), (C1, C2), (14, 14);
}

pub trait PwmExt1: Sized {
    fn pwm<PINS>(self, _: PINS, frequency: Hertz, clocks: Clocks, apb: &mut APB2) -> PINS::Channels
    where
        PINS: Pins<Self>;
}

pub trait PwmExt2: Sized {
    fn pwm<PINS>(
        self,
        _: PINS,
        frequency: Hertz,
        clocks: Clocks,
        apb: &mut APB1R1,
    ) -> PINS::Channels
    where
        PINS: Pins<Self>;
}

impl PwmExt1 for TIM1 {
    fn pwm<PINS>(self, _pins: PINS, freq: Hertz, clocks: Clocks, apb: &mut APB2) -> PINS::Channels
    where
        PINS: Pins<Self>,
    {
        tim1(self, _pins, freq, clocks, apb)
    }
}

impl PwmExt1 for TIM15 {
    fn pwm<PINS>(self, _pins: PINS, freq: Hertz, clocks: Clocks, apb: &mut APB2) -> PINS::Channels
    where
        PINS: Pins<Self>,
    {
        tim15(self, _pins, freq, clocks, apb)
    }
}

impl PwmExt2 for TIM2 {
    fn pwm<PINS>(self, _pins: PINS, freq: Hertz, clocks: Clocks, apb: &mut APB1R1) -> PINS::Channels
    where
        PINS: Pins<Self>,
    {
        // TODO: check if this is really not needed (in the f1xx examples value
        //       of remap is 0x0). if so, what's afio.mapr on l4xx?
        //
        // mapr.mapr()
        //     .modify(|_, w| unsafe { w.tim2_remap().bits(PINS::REMAP) });

        tim2(self, _pins, freq, clocks, apb)
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

macro_rules! advanced_timer {
    ($($TIMX:ident: ($timX:ident, $apb:ident, $psc_width:ident, $arr_width:ident),)+) => {
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
                <$TIMX>::enable(apb);
                <$TIMX>::reset(apb);

                if PINS::C1 {
                    tim.ccmr1_output().modify(|_, w| w.oc1pe().set_bit().oc1m().bits(6));
                }

                if PINS::C2 {
                    tim.ccmr1_output().modify(|_, w| w.oc2pe().set_bit().oc2m().bits(6));
                }

                if PINS::C3 {
                    tim.ccmr2_output().modify(|_, w| w.oc3pe().set_bit().oc3m().bits(6));
                }

                if PINS::C4 {
                    tim.ccmr2_output().modify(|_, w| w.oc4pe().set_bit().oc4m().bits(6));
                }

                let clk = clocks.pclk2();
                let ticks = clk / freq;

                // maybe this is all u32? also, why no `- 1` vs `timer.rs`?
                let psc = ticks / (1 << 16);
                tim.psc.write(|w| { w.psc().bits(psc as $psc_width) });
                let arr = ticks / (psc + 1);
                tim.arr.write(|w| { w.arr().bits(arr as $arr_width) });

                // Only for the advanced control timer
                tim.bdtr.write(|w| w.moe().set_bit());
                tim.egr.write(|w| w.ug().set_bit());

                tim.cr1.write(|w| {
                    w.cms()
                        .bits(0b00)
                        .dir().clear_bit()
                        .opm().clear_bit()
                        .cen().set_bit()
                        .arpe().set_bit()
                });

                unsafe { mem::MaybeUninit::uninit().assume_init() }
            }

            pwm_channels! {
                $TIMX:  (C1, $arr_width, cc1e, ccr1, ccr),
                        (C2, $arr_width, cc2e, ccr2, ccr),
                        (C3, $arr_width, cc3e, ccr3, ccr),
                        (C4, $arr_width, cc4e, ccr4, ccr),
            }

        )+
    }
}

macro_rules! standard_timer {
    ($($TIMX:ident: ($timX:ident, $apb:ident, $psc_width:ident, $arr_width:ident),)+) => {
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
                <$TIMX>::enable(apb);
                <$TIMX>::reset(apb);

                if PINS::C1 {
                    tim.ccmr1_output().modify(|_, w| w.oc1pe().set_bit().oc1m().bits(6));
                }

                if PINS::C2 {
                    tim.ccmr1_output().modify(|_, w| w.oc2pe().set_bit().oc2m().bits(6));
                }

                if PINS::C3 {
                    tim.ccmr2_output().modify(|_, w| w.oc3pe().set_bit().oc3m().bits(6));
                }

                if PINS::C4 {
                    tim.ccmr2_output().modify(|_, w| w.oc4pe().set_bit().oc4m().bits(6));
                }

                let clk = clocks.pclk1();
                let ticks = clk / freq;

                // maybe this is all u32? also, why no `- 1` vs `timer.rs`?
                let psc = ticks / (1 << 16);
                tim.psc.write(|w| { w.psc().bits(psc as $psc_width) });
                let arr = ticks / (psc + 1);
                tim.arr.write(|w| { w.arr().bits(arr as $arr_width) });

                tim.cr1.write(|w| {
                    w.cms()
                        .bits(0b00)
                        .dir().clear_bit()
                        .opm().clear_bit()
                        .cen().set_bit()
                        .arpe().set_bit()
                });

                unsafe { mem::MaybeUninit::uninit().assume_init() }
            }

            pwm_channels! {
                $TIMX:  (C1, $arr_width, cc1e, ccr1, ccr),
                        (C2, $arr_width, cc2e, ccr2, ccr),
                        (C3, $arr_width, cc3e, ccr3, ccr),
                        (C4, $arr_width, cc4e, ccr4, ccr),
            }

        )+
    }
}

macro_rules! small_timer {
    ($($TIMX:ident: ($timX:ident, $apb:ident, $psc_width:ident, $arr_width:ident),)+) => {
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
                <$TIMX>::enable(apb);
                <$TIMX>::reset(apb);

                if PINS::C1 {
                    tim.ccmr1_output().modify(|_, w| w.oc1pe().set_bit().oc1m().bits(6));
                }

                // TODO: The uncommented lines are awaiting PAC updates to be valid.
                // if PINS::C2 {
                //     tim.ccmr1_output().modify(|_, w| w.oc2pe().set_bit().oc2m().bits(6));
                // }

                let clk = clocks.pclk1();
                let ticks = clk / freq;

                // maybe this is all u32? also, why no `- 1` vs `timer.rs`?
                let psc = ticks / (1 << 16);
                tim.psc.write(|w| { w.psc().bits(psc as $psc_width) });
                let arr = ticks / (psc + 1);
                unsafe { tim.arr.write(|w| { w.arr().bits(arr as $arr_width) }); }

                tim.bdtr.write(|w| w.moe().set_bit());
                tim.egr.write(|w| w.ug().set_bit());

                tim.cr1.write(|w| {
                    w.opm().clear_bit()
                        .cen().set_bit()
                        .arpe().set_bit()
                });

                unsafe { mem::MaybeUninit::uninit().assume_init() }
            }

            pwm_channels! {
                $TIMX:  (C1, $arr_width, cc1e, ccr1, ccr),
                // TODO: The uncommented line is awaiting PAC updates to be valid.
                //        (C2, $arr_width, cc2e, ccr2, ccr2),
            }

        )+
    }
}

macro_rules! pwm_channels {
    ($TIMX:ident: $(($channel:ident, $arr_width:ident, $ccXe:ident, $ccrX:ident, $ccr:ident),)+) => {
        $(
            impl hal::PwmPin for Pwm<$TIMX, $channel> {
                type Duty = $arr_width;

                #[inline(always)]
                fn disable(&mut self) {
                    unsafe { (*$TIMX::ptr()).ccer.modify(|_, w| w.$ccXe().clear_bit()) }
                }

                #[inline(always)]
                fn enable(&mut self) {
                    unsafe { (*$TIMX::ptr()).ccer.modify(|_, w| w.$ccXe().set_bit()) }
                }

                #[inline(always)]
                fn get_duty(&self) -> Self::Duty {
                    unsafe { (*$TIMX::ptr()).$ccrX.read().$ccr().bits() }
                }

                #[inline(always)]
                fn get_max_duty(&self) -> Self::Duty {
                    unsafe { (*$TIMX::ptr()).arr.read().arr().bits() }
                }

                #[inline(always)]
                fn set_duty(&mut self, duty: Self::Duty) {
                    unsafe { (*$TIMX::ptr()).$ccrX.write(|w| w.$ccr().bits(duty)) }
                }
            }
        )+
    }
}

advanced_timer! {
    TIM1: (tim1, APB2, u16, u16),
}

standard_timer! {
    TIM2: (tim2, APB1R1, u16, u32),
}

small_timer! {
    TIM15: (tim15, APB2, u16, u16),
}
