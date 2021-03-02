//! Timers

use crate::hal::timer::{CountDown, Periodic};
use crate::rcc::{Clocks, APB1R1, APB2};
use crate::stm32::{TIM15, TIM16, TIM2, TIM6, TIM7};
#[cfg(any(feature = "stm32l4x5", feature = "stm32l4x6",))]
use crate::stm32::{TIM17, TIM4, TIM5};
use crate::time::Hertz;
use cast::{u16, u32};
use rtic_monotonic::{embedded_time, Clock, Fraction, Instant, Monotonic};
use void::Void;

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
                            self.tim.dier.modify(|_, w| w.uie().set_bit());
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
                            self.tim.sr.modify(|_, w| w.uif().clear_bit());
                        }
                    }
                }


                /// Stops listening for an `event`
                pub fn unlisten(&mut self, event: Event) {
                    match event {
                        Event::TimeOut => {
                            // Enable update event interrupt
                            self.tim.dier.modify(|_, w| w.uie().clear_bit());
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

/// Extended TIM15/16 to 64 bits
pub struct ExtendedTimer<TIM> {
    tim: Timer<TIM>,
    ovf: u64,
}

impl ExtendedTimer<TIM15> {
    fn is_overflow(&self) -> bool {
        self.tim.tim.sr.read().uif().bit_is_set()
    }
}

impl Clock for ExtendedTimer<TIM15> {
    const SCALING_FACTOR: Fraction = Fraction::new(1, 1_000_000);
    type T = u64;

    fn try_now(&self) -> Result<Instant<Self>, embedded_time::clock::Error> {
        let cnt = Timer::<TIM15>::count();

        // If the overflow bit is set, we add this to the timer value. It means the `on_interrupt`
        // has not yet happened, and we need to compensate here.
        let ovf = if self.is_overflow() { 0x10000 } else { 0 };

        Ok(Instant::new(cnt as u64 + ovf as u64 + self.ovf))
    }
}

/// Use Compare channel 1 for Monotonic
impl Monotonic for ExtendedTimer<TIM15> {
    // Since we are counting overflows we can't let RTIC disable the interrupt.
    const DISABLE_INTERRUPT_ON_EMPTY_QUEUE: bool = false;

    unsafe fn reset(&mut self) {
        // Since reset is only called once, we use it to enable the interrupt generation bit.
        self.tim.tim.dier.modify(|_, w| w.cc1ie().set_bit());
        // self.tim.tim.cnt.write(|w| w.bits(0));
    }

    fn set_compare(&mut self, instant: &Instant<Self>) {
        let now = self.try_now().unwrap();

        // Since the timer may or may not overflow based on the requested compare val, we check
        // how many ticks are left.
        let val = match instant.checked_duration_since(&now) {
            None => Timer::<TIM15>::count().wrapping_add(1), // In the past
            Some(x) if *x.integer() <= 0xffff => *instant.duration_since_epoch().integer() as u16, // Will not overflow
            Some(_x) => Timer::<TIM15>::count().wrapping_add(0xffff), // Will overflow
        };

        unsafe { self.tim.tim.ccr1.write(|w| w.ccr1().bits(val)) };
    }

    fn clear_compare_flag(&mut self) {
        self.tim.tim.sr.modify(|_, w| w.cc1if().clear_bit());
    }

    fn on_interrupt(&mut self) {
        // If there was an overflow, increment the overflow counter.
        if self.is_overflow() {
            self.tim.clear_update_interrupt_flag();

            self.ovf += 0x10000;
        }
    }
}

// ------------

/// Implement Clock for TIM2 as it is the only timer with 32 bits.
impl Clock for Timer<TIM2> {
    const SCALING_FACTOR: Fraction = Fraction::new(1, 80_000_000);
    type T = u32;

    #[inline(always)]
    fn try_now(&self) -> Result<Instant<Self>, embedded_time::clock::Error> {
        Ok(Instant::new(Self::count()))
    }
}

/// Use Compare channel 1 for Monotonic
impl Monotonic for Timer<TIM2> {
    unsafe fn reset(&mut self) {
        // Since reset is only called once, we use it to enable the interrupt generation bit.
        self.tim.dier.modify(|_, w| w.cc1ie().set_bit());
        // self.tim.cnt.write(|w| w.bits(0));
    }

    fn set_compare(&mut self, instant: &Instant<Self>) {
        self.tim
            .ccr1
            .write(|w| w.ccr().bits(*instant.duration_since_epoch().integer()));
    }

    fn clear_compare_flag(&mut self) {
        self.tim.sr.modify(|_, w| w.cc1if().clear_bit());
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
