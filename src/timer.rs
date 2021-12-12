//! Timers

use crate::hal::timer::{CountDown, Periodic};
// missing PAC support
/*
#[cfg(any(
    feature = "stm32l451",
    feature = "stm32l452",
    feature = "stm32l462",
    feature = "stm32l471",
    feature = "stm32l475",
    feature = "stm32l476",
    feature = "stm32l485",
    feature = "stm32l486",
    feature = "stm32l496",
    feature = "stm32l4a6",
    // feature = "stm32l4p5",
    // feature = "stm32l4q5",
    // feature = "stm32l4r5",
    // feature = "stm32l4s5",
    // feature = "stm32l4r7",
    // feature = "stm32l4s7",
    feature = "stm32l4r9",
    feature = "stm32l4s9",
))]
use crate::stm32::TIM3;
*/
#[cfg(not(any(
    feature = "stm32l412",
    feature = "stm32l422",
    feature = "stm32l451",
    feature = "stm32l452",
    feature = "stm32l462",
)))]
use crate::stm32::TIM7;
use crate::stm32::{TIM15, TIM16, TIM2, TIM6};
#[cfg(any(
    // feature = "stm32l471", // missing PAC support
    feature = "stm32l475",
    feature = "stm32l476",
    feature = "stm32l485",
    feature = "stm32l486",
    feature = "stm32l496",
    feature = "stm32l4a6",
    // feature = "stm32l4p5",
    // feature = "stm32l4q5",
    // feature = "stm32l4r5",
    // feature = "stm32l4s5",
    // feature = "stm32l4r7",
    // feature = "stm32l4s7",
    feature = "stm32l4r9",
    feature = "stm32l4s9",
))]
use crate::stm32::{TIM17, TIM4, TIM5};

// TIM1/TIM8 ("Advcanced Control Timers") -> no impl
// TIM2/TIM3/TIM4/TIM5 ("General Purpose Timers")
// TIM15/TIM16/TIM17 ("General Purpose Timers")
// TIM6/TIM7 ("Basic Timers")
// LPTIM ("Low power Timer") -> no impl

use cast::{u16, u32};
use void::Void;

use crate::rcc::{Clocks, Enable, Reset, APB1R1, APB2};
use crate::time::Hertz;
use fugit::RateExtU32;

/// Hardware timers
pub struct Timer<TIM> {
    clocks: Clocks,
    tim: TIM,
    timeout: Hertz,
}

/// Interrupt events
pub enum Event {
    /// Timer timed out / count down ended
    TimeOut,
}

macro_rules! hal {
    ($($TIM:ident: ($tim:ident, $frname:ident, $apb:ident, $width:ident),)+) => {
        $(
            impl Periodic for Timer<$TIM> {}

            impl CountDown for Timer<$TIM> {
                type Time = Hertz;

                // NOTE(allow) `w.psc().bits()` is safe for TIM{6,7} but not for TIM{2,3,4} due to
                // some SVD omission
                #[allow(unused_unsafe)]
                fn start<T>(&mut self, timeout: T)
                where
                    T: Into<Hertz>,
                {
                    // pause
                    self.tim.cr1.modify(|_, w| w.cen().clear_bit());

                    self.timeout = timeout.into();
                    let ticks = self.clocks.pclk1() / self.timeout; // TODO check pclk that timer is on
                    let psc = u16((ticks - 1) / (1 << 16)).unwrap();

                    self.tim.psc.write(|w| unsafe { w.psc().bits(psc) });

                    let arr = u16(ticks / u32(psc + 1)).unwrap();

                    self.tim.arr.write(|w| unsafe { w.bits(u32(arr)) });

                    // Trigger an update event to load the prescaler value to the clock
                    self.tim.egr.write(|w| w.ug().set_bit());
                    // The above line raises an update event which will indicate
                    // that the timer is already finished. Since this is not the case,
                    // it should be cleared
                    self.clear_update_interrupt_flag();

                    // start counter
                    self.tim.cr1.modify(|_, w| w.cen().set_bit());
                }

                fn wait(&mut self) -> nb::Result<(), Void> {
                    if self.tim.sr.read().uif().bit_is_clear() {
                        Err(nb::Error::WouldBlock)
                    } else {
                        self.clear_update_interrupt_flag();
                        Ok(())
                    }
                }
            }

            impl Timer<$TIM> {
                // XXX(why not name this `new`?) bummer: constructors need to have different names
                // even if the `$TIM` are non overlapping (compare to the `free` function below
                // which just works)
                /// Configures a TIM peripheral as a periodic count down timer
                pub fn $tim(tim: $TIM, timeout: Hertz, clocks: Clocks, apb: &mut $apb) -> Self {
                    // enable and reset peripheral to a clean slate state
                    <$TIM>::enable(apb);
                    <$TIM>::reset(apb);

                    let mut timer = Timer {
                        clocks,
                        tim,
                        timeout: 0.Hz(),
                    };
                    timer.start(timeout);

                    timer
                }

                /// Start a free running, monotonic, timer running at some specific frequency.
                ///
                /// May generate events on overflow of the timer.
                pub fn $frname(
                    tim: $TIM,
                    clocks: Clocks,
                    frequency: Hertz,
                    event_on_overflow: bool,
                    apb: &mut $apb,
                ) -> Self {
                    <$TIM>::enable(apb);
                    <$TIM>::reset(apb);

                    let psc = clocks.pclk1() / frequency - 1;

                    debug_assert!(clocks.pclk1() >= frequency);
                    debug_assert!(frequency.raw() > 0);
                    debug_assert!(psc <= core::u16::MAX.into());

                    tim.psc.write(|w| w.psc().bits((psc as u16).into()) );
                    let max = core::$width::MAX;
                    tim.arr.write(|w| unsafe { w.bits(max.into()) });

                    // Trigger an update event to load the prescaler value to the clock
                    tim.egr.write(|w| w.ug().set_bit());


                    // The above line raises an update event which will indicate
                    // that the timer is already finished. Since this is not the case,
                    // it should be cleared
                    tim.sr.modify(|_, w| w.uif().clear_bit());

                    // start counter
                    tim.cr1.modify(|_, w| {
                        w.cen().set_bit();

                        if event_on_overflow {
                            w.udis().clear_bit();
                        } else {
                            w.udis().set_bit();
                        }

                        w
                    });

                    Timer {
                        clocks,
                        tim,
                        timeout: frequency,
                    }
                }

                /// Starts listening for an `event`
                pub fn listen(&mut self, event: Event) {
                    match event {
                        Event::TimeOut => {
                            // Enable update event interrupt
                            self.tim.dier.write(|w| w.uie().set_bit());
                        }
                    }
                }


                /// Clears interrupt associated with `event`.
                ///
                /// If the interrupt is not cleared, it will immediately retrigger after
                /// the ISR has finished.
                pub fn clear_interrupt(&mut self, event: Event) {
                    match event {
                        Event::TimeOut => {
                            // Clear interrupt flag
                            self.tim.sr.write(|w| w.uif().clear_bit());
                        }
                    }
                }


                /// Stops listening for an `event`
                pub fn unlisten(&mut self, event: Event) {
                    match event {
                        Event::TimeOut => {
                            // Enable update event interrupt
                            self.tim.dier.write(|w| w.uie().clear_bit());
                        }
                    }
                }

                /// Clears Update Interrupt Flag
                pub fn clear_update_interrupt_flag(&mut self) {
                    self.tim.sr.modify(|_, w| w.uif().clear_bit());
                }

                /// Get the count of the timer.
                pub fn count() -> $width {
                    let cnt = unsafe { (*$TIM::ptr()).cnt.read() };
                    cnt.cnt().bits()
                }

                /// Releases the TIM peripheral
                pub fn free(self) -> $TIM {
                    // pause counter
                    self.tim.cr1.modify(|_, w| w.cen().clear_bit());
                    self.tim
                }
            }
        )+
    }
}

hal! {
    TIM2:  (tim2, free_running_tim2, APB1R1, u32),
    TIM6:  (tim6, free_running_tim6, APB1R1, u16),
    //TIM7:  (tim7, free_running_tim7, APB1R1, u16),
    TIM15: (tim15, free_running_tim15, APB2, u16),
    TIM16: (tim16, free_running_tim16, APB2, u16),
}

// missing PAC support
// RCC_APB1RSTR1->TIM3RST not defined
/*
#[cfg(any(
    feature = "stm32l451",
    feature = "stm32l452",
    feature = "stm32l462",
    feature = "stm32l471",
    feature = "stm32l475",
    feature = "stm32l476",
    feature = "stm32l485",
    feature = "stm32l486",
    feature = "stm32l496",
    feature = "stm32l4a6",
    // feature = "stm32l4p5",
    // feature = "stm32l4q5",
    // feature = "stm32l4r5",
    // feature = "stm32l4s5",
    // feature = "stm32l4r7",
    // feature = "stm32l4s7",
    feature = "stm32l4r9",
    feature = "stm32l4s9",
))]
hal! {
    TIM3:  (tim3, free_running_tim3, tim3en, tim3rst, APB1R1, u32),
}
*/

#[cfg(not(any(
    feature = "stm32l412",
    feature = "stm32l422",
    feature = "stm32l451",
    feature = "stm32l452",
    feature = "stm32l462",
)))]
hal! {
    TIM7:  (tim7, free_running_tim7, APB1R1, u16),
}

#[cfg(any(
    feature = "stm32l475",
    feature = "stm32l476",
    feature = "stm32l485",
    feature = "stm32l486",
    feature = "stm32l496",
    feature = "stm32l4a6",
    // feature = "stm32l4p5",
    // feature = "stm32l4q5",
    // feature = "stm32l4r5",
    // feature = "stm32l4s5",
    // feature = "stm32l4r7",
    // feature = "stm32l4s7",
    // feature = "stm32l4r9",
    // feature = "stm32l4s9",
))]
hal! {
    TIM4:  (tim4, free_running_tim4, APB1R1, u16),
    TIM5:  (tim5, free_running_tim5, APB1R1, u32),
    TIM17: (tim17, free_running_tim17, APB2, u16),
}
