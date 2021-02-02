//! Timers

use crate::hal::timer::{CountDown, Periodic};
use crate::stm32::{TIM15, TIM16, TIM2, TIM6, TIM7};
#[cfg(any(feature = "stm32l4x5", feature = "stm32l4x6",))]
use crate::stm32::{TIM17, TIM4, TIM5};
use cast::{u16, u32};
use void::Void;

use crate::rcc::{Clocks, APB1R1, APB2};
use crate::time::Hertz;

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
    ($($TIM:ident: ($tim:ident, $frname:ident, $timXen:ident, $timXrst:ident, $apb:ident, $width:ident),)+) => {
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
                    let frequency = self.timeout.0;
                    let ticks = self.clocks.pclk1().0 / frequency; // TODO check pclk that timer is on
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
                pub fn $tim<T>(tim: $TIM, timeout: T, clocks: Clocks, apb: &mut $apb) -> Self
                where
                    T: Into<Hertz>,
                {
                    // enable and reset peripheral to a clean slate state
                    apb.enr().modify(|_, w| w.$timXen().set_bit());
                    apb.rstr().modify(|_, w| w.$timXrst().set_bit());
                    apb.rstr().modify(|_, w| w.$timXrst().clear_bit());

                    let mut timer = Timer {
                        clocks,
                        tim,
                        timeout: Hertz(0),
                    };
                    timer.start(timeout);

                    timer
                }

                /// Start a free running, monotonic, timer running at some specific frequency.
                ///
                /// May generate events on overflow of the timer.
                pub fn $frname<T>(
                    tim: $TIM,
                    clocks: Clocks,
                    frequency: T,
                    event_on_overflow: bool,
                    apb: &mut $apb,
                ) -> Self
                where
                    T: Into<Hertz>,
                {
                    apb.enr().modify(|_, w| w.$timXen().set_bit());
                    apb.rstr().modify(|_, w| w.$timXrst().set_bit());
                    apb.rstr().modify(|_, w| w.$timXrst().clear_bit());

                    let frequency = frequency.into();
                    let psc = clocks.pclk1().0 / frequency.0 - 1;

                    debug_assert!(clocks.pclk1().0 >= frequency.0);
                    debug_assert!(frequency.0 > 0);
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
    TIM2:  (tim2, free_running_tim2, tim2en, tim2rst, APB1R1, u32),
    TIM6:  (tim6, free_running_tim6, tim6en, tim6rst, APB1R1, u16),
    TIM7:  (tim7, free_running_tim7, tim7en, tim7rst, APB1R1, u16),
    TIM15: (tim15, free_running_tim15, tim15en, tim15rst, APB2, u16),
    TIM16: (tim16, free_running_tim16, tim16en, tim16rst, APB2, u16),
}

#[cfg(any(feature = "stm32l4x5", feature = "stm32l4x6",))]
hal! {
    TIM4:  (tim4, free_running_tim4, tim4en, tim4rst, APB1R1, u16),
    TIM5:  (tim5, free_running_tim5, tim5en, tim5rst, APB1R1, u32),
    TIM17: (tim17, free_running_tim17, tim17en, tim17rst, APB2, u16),
}
