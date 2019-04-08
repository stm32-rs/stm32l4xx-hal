//! Delays

use cast::{u64, u32};
use core::time::Duration;
use cortex_m::peripheral::syst::SystClkSource;
use cortex_m::peripheral::SYST;

use ticklock::timer::{Timer, TimerInstant};
use crate::rcc::Clocks;

/// System timer (SysTick) as a delay provider
pub struct Delay {
    clocks: Clocks,
    syst: SYST,
}

impl Delay {
    /// Configures the system timer (SysTick) as a delay provider
    pub fn new(mut syst: SYST, clocks: Clocks) -> Self {
        syst.set_clock_source(SystClkSource::Core);

        Delay { syst, clocks }
    }

    /// Releases the system timer (SysTick) resource
    pub fn free(self) -> SYST {
        self.syst
    }
}

impl Timer for Delay {

    type U = u32;

    fn delay(&mut self, d: Duration) {
        let mut ticks = self.clocks.sysclk().ticks_in(d);
        while ticks != 0 {
            let current_rvr = if ticks <= u64(self.limit_value()) {
                u32(ticks).unwrap()
            } else {
                self.limit_value()
            };

            self.syst.set_reload(current_rvr);
            self.syst.clear_current();
            self.syst.enable_counter();

            // Update the tracking variable while we are waiting...
            ticks -= u64(current_rvr);

            while !self.has_wrapped() {}

            self.syst.disable_counter();
        }
    }

    fn start(mut self) ->  TimerInstant<Self> {
        self.syst.set_reload(self.limit_value());
        self.syst.clear_current();
        self.syst.enable_counter();
        TimerInstant::now(self)
    }

    /// Stop the counting timer.
    /// This method is only used by `TimerInstant` to release the timer.
    fn stop(mut self) -> Self {
        self.syst.disable_counter();
        self
    }

    /// Test if the counter has wrapped to its initial value
    fn has_wrapped(&mut self) -> bool {
        self.syst.has_wrapped()
    }

    /// The maximum / minimum value.
    /// For count down timer this should be the maximum value. Or the reload value.
    /// For count up limit_value should return 0.
    fn limit_value(&self) ->  u32 {
        // The SysTick Reload Value register supports values between 1 and 0x00FFFFFF.
        0x00FF_FFFF
    }

    /// Return the current counter value.
    fn get_current(&mut self) -> u32 {
        SYST::get_current()
    }

    /// Return the duration between 2 counted value.
    fn tick(&mut self) -> Duration {
        self.clocks.sysclk().tick()
    }
}
