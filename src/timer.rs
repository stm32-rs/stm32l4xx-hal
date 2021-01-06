//! Timers

use crate::hal::timer::{CountDown, Periodic};
use crate::stm32::{TIM15, TIM16, TIM2, TIM6, TIM7};
#[cfg(any(feature = "stm32l4x5", feature = "stm32l4x6",))]
use crate::stm32::{TIM17, TIM3, TIM4, TIM5};
use cast::{u16, u32};
use num_traits::float::Float;
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

#[derive(Clone, Copy)]
pub struct ValueError {}

macro_rules! hal {
    ($($TIM:ident: ($tim:ident, $timXen:ident, $timXrst:ident, $apb:ident),)+) => {
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

                /// Releases the TIM peripheral
                pub fn free(self) -> $TIM {
                    // pause counter
                    self.tim.cr1.modify(|_, w| w.cen().clear_bit());
                    self.tim
                }

                /// Set the timer period, in seconds. Overrides the period or frequency set
                /// in the constructor.
                /// This allows you to set periods greater than 1hz.
                pub fn set_period(&mut self, period: f32, timer_clock_speed: f32) -> Result<(), ValueError> {
                    // PSC and ARR range: 0 to 65535
                    // (PSC+1)*(ARR+1) = TIMclk/Updatefrequency = TIMclk * period
                    // APB1 (pclk1) is used by Tim2, 3, 4, 6, 7.
                    // APB2 (pclk2) is used by Tim8, 15-20 etc.
                    // todo: It appears there's a (fixed?) 2x multiplier on APB1
                    // timers; it's twice `pclk1`. See clocks diagram in RM, or `Clock Configuration`
                    // tool in STM32CubeIDE.

                    // todo: Accept a suitable clocks struct once you figure out how it's layed out.
                    // `timer_clock_speed` is in Mhz.
                    // let tim_clk = clocks.calc_speeds().pclk1 * 1_000_000. * 2.;
                    let tim_clk = timer_clock_speed;

                    // We need to factor the right-hand-side of the above equation (`rhs` variable)
                    // into integers. There are likely clever algorithms available to do this.
                    // Some examples: https://cp-algorithms.com/algebra/factorization.html
                    // We've chosen something quick to write, and with sloppy precision;
                    // should be good enough for most cases.

                    // - If you work with pure floats, there are an infinite number of solutions: Ie for any value of PSC, you can find an ARR to solve the equation.
                    // - The actual values are integers that must be between 0 and 65_536
                    // - Different combinations will result in different amounts of rounding errors. Ideally, we pick the one with the lowest rounding error.
                    // - The aboveapproach sets PSC and ARR always equal to each other.
                    // This results in concise code, is computationally easy, and doesn't limit
                    // the maximum period. There will usually be solutions that have a smaller rounding error.

                    let max_val = 65_535;
                    let rhs = tim_clk * period;

                    // todo: Round instead of cast?
                    let arr = (rhs.sqrt() - 1.) as u16;
                    let psc = arr;

                    if arr > max_val || psc > max_val {
                        return Err(ValueError {})
                    }

                    self.tim.arr.write(|w| unsafe { w.bits(u32::from(arr)) });
                    self.tim.psc.write(|w| unsafe { w.bits(u32::from(psc)) });

                    Ok(())
                }
            }
        )+
    }
}

hal! {
    TIM2: (tim2, tim2en, tim2rst, APB1R1),
    TIM6: (tim6, tim6en, tim6rst, APB1R1),
    TIM7: (tim7, tim7en, tim7rst, APB1R1),
    TIM15: (tim15, tim15en, tim15rst, APB2),
    TIM16: (tim16, tim16en, tim16rst, APB2),
}

#[cfg(any(feature = "stm32l4x5", feature = "stm32l4x6",))]
hal! {
    TIM3: (tim3, tim3en, tim3rst, APB1R1), // todo: Confirm this exists. Why did I have to add it?
    TIM4: (tim4, tim4en, tim4rst, APB1R1),
    TIM5: (tim5, tim5en, tim5rst, APB1R1),
    TIM17: (tim17, tim17en, tim17rst, APB2),
}
