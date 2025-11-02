//! Timers

use core::convert::Infallible;

use crate::hal::timer::{Cancel, CountDown, Periodic};

#[cfg(any(
    // feature = "stm32l451",
    feature = "stm32l452",
    feature = "stm32l462",
    // feature = "stm32l471",
    feature = "stm32l475",
    feature = "stm32l476",
    feature = "stm32l485",
    feature = "stm32l486",
    feature = "stm32l496",
    feature = "stm32l4a6",
    // feature = "stm32l4p5",
    // feature = "stm32l4q5",
    feature = "stm32l4r5",
    feature = "stm32l4s5",
    // feature = "stm32l4r7",
    // feature = "stm32l4s7",
    // feature = "stm32l4r9",
    // feature = "stm32l4s9",
))]
use crate::stm32::TIM3;

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
    feature = "stm32l4r5",
    feature = "stm32l4s5",
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
    clock: Hertz,
    tim: TIM,
    timeout: Hertz,
}

/// Interrupt events
pub enum Event {
    /// Timer timed out / count down ended
    TimeOut,
}

macro_rules! hal {
    ($($TIM:ident: ($tim:ident, $frname:ident, $apb:ident, $width:ident, $timclk:ident),)+) => {
        $(
            impl Periodic for Timer<$TIM> {}

            impl CountDown for Timer<$TIM> {
                type Time = Hertz;

                // NOTE(allow) `w.psc().bits()` is safe for TIM{6,7} but not for TIM{2,3,4} due to
                // some SVD omission.
                #[allow(unused_unsafe)]
                fn start<T>(&mut self, timeout: T)
                where
                    T: Into<Hertz>,
                {
                    self.pause();

                    self.timeout = timeout.into();
                    let ticks = self.clock / self.timeout; // TODO check pclk that timer is on
                    let psc = u16((ticks - 1) / (1 << 16)).unwrap();

                    self.tim.psc.write(|w| unsafe { w.psc().bits(psc) });

                    let arr = u16(ticks / u32(psc + 1)).unwrap();

                    self.tim.arr.write(|w| unsafe { w.bits(u32(arr)) });

                    // Trigger an update event to load the prescaler value to the clock.
                    self.tim.egr.write(|w| w.ug().set_bit());

                    // The above line raises an update event which will indicate
                    // that the timer is already finished. Since this is not the case,
                    // it should be cleared.
                    self.clear_update_interrupt_flag();

                    // Start counter.
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

            impl Cancel for Timer<$TIM> {
                type Error = Infallible;

                fn cancel(&mut self) -> Result<(), Self::Error> {
                    self.pause();
                    self.reset();

                    Ok(())
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

                    let clock = clocks.$timclk();

                    let mut timer = Timer {
                        clock,
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

                    let clock = clocks.$timclk();

                    let psc = clock / frequency - 1;

                    debug_assert!(clock >= frequency);
                    debug_assert!(frequency.raw() > 0);
                    debug_assert!(psc <= core::u16::MAX.into());

                    tim.psc.write(|w| w.psc().bits((psc as u16).into()) );
                    let max = core::$width::MAX;
                    tim.arr.write(|w| unsafe { w.bits(max.into()) });

                    // Trigger an update event to load the prescaler value to the clock.
                    tim.egr.write(|w| w.ug().set_bit());


                    // The above line raises an update event which will indicate
                    // that the timer is already finished. Since this is not the case,
                    // it should be cleared
                    tim.sr.modify(|_, w| w.uif().clear_bit());

                    // Start counter.
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
                        clock,
                        tim,
                        timeout: frequency,
                    }
                }

                /// Starts listening for an `event`
                pub fn listen(&mut self, event: Event) {
                    match event {
                        Event::TimeOut => {
                            // Enable update event interrupt.
                            self.tim.dier.write(|w| w.uie().set_bit());
                        }
                    }
                }

                /// Clears interrupt associated with `event`.
                ///
                /// If the interrupt is not cleared, it will immediately
                /// retrigger after the ISR has finished.
                pub fn clear_interrupt(&mut self, event: Event) {
                    match event {
                        Event::TimeOut => {
                            // Clear interrupt flag
                            self.tim.sr.write(|w| w.uif().clear_bit());
                        }
                    }
                }

                /// Stops listening for an `event`.
                pub fn unlisten(&mut self, event: Event) {
                    match event {
                        Event::TimeOut => {
                            // Enable update event interrupt
                            self.tim.dier.write(|w| w.uie().clear_bit());
                        }
                    }
                }

                /// Clear the update interrupt flag.
                pub fn clear_update_interrupt_flag(&mut self) {
                    self.tim.sr.modify(|_, w| w.uif().clear_bit());
                }

                /// Get the count of the timer.
                pub fn count() -> $width {
                    let cnt = unsafe { (*$TIM::ptr()).cnt.read() };
                    cnt.cnt().bits()
                }

                /// Pause the counter.
                pub fn pause(&mut self) {
                    self.tim.cr1.modify(|_, w| w.cen().clear_bit());
                }

                /// Reset the counter.
                pub fn reset(&mut self) {
                    self.tim.cnt.modify(|_, w| unsafe { w.bits(0) });
                }

                /// Releases the TIM peripheral.
                pub fn free(mut self) -> $TIM {
                    self.pause();
                    self.tim
                }
            }
        )+
    }
}

hal! {
    TIM2:  (tim2, free_running_tim2, APB1R1, u32, timclk1),
    TIM6:  (tim6, free_running_tim6, APB1R1, u16, timclk1),
    //TIM7:  (tim7, free_running_tim7, APB1R1, u16, timclk1),
    TIM15: (tim15, free_running_tim15, APB2, u16, timclk2),
    TIM16: (tim16, free_running_tim16, APB2, u16, timclk2),
}

#[cfg(any(
    // feature = "stm32l451",
    feature = "stm32l452",
    feature = "stm32l462",
    // feature = "stm32l471",
    feature = "stm32l475",
    feature = "stm32l476",
    feature = "stm32l485",
    feature = "stm32l486",
    feature = "stm32l496",
    feature = "stm32l4a6",
    // feature = "stm32l4p5",
    // feature = "stm32l4q5",
    feature = "stm32l4r5",
    feature = "stm32l4s5",
    // feature = "stm32l4r7",
    // feature = "stm32l4s7",
    // feature = "stm32l4r9",
    // feature = "stm32l4s9",
))]
hal! {
    TIM3:  (tim3, free_running_tim3, APB1R1, u16, timclk1),
}

#[cfg(not(any(
    feature = "stm32l412",
    feature = "stm32l422",
    feature = "stm32l451",
    feature = "stm32l452",
    feature = "stm32l462",
)))]
hal! {
    TIM7:  (tim7, free_running_tim7, APB1R1, u16, timclk1),
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
    feature = "stm32l4r5",
    feature = "stm32l4s5",
    // feature = "stm32l4r7",
    // feature = "stm32l4s7",
    // feature = "stm32l4r9",
    // feature = "stm32l4s9",
))]
hal! {
    TIM4:  (tim4, free_running_tim4, APB1R1, u16, timclk1),
    TIM5:  (tim5, free_running_tim5, APB1R1, u32, timclk1),
    TIM17: (tim17, free_running_tim17, APB2, u16, timclk2),
}
