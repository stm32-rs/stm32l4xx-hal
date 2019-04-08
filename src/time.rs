//! Time units

use cast::u64;
use core::time::Duration;
use cortex_m::peripheral::DWT;
use ticklock::clock::Frequency;
use ticklock::timer::{Timer, TimerInstant};

use crate::rcc::Clocks;

/// Bits per second
#[derive(Clone, Copy, Debug)]
pub struct Bps(pub u32);

/// Timer base on DWT feature. This is highly not recommended to use as it
/// depend on the debugger being attached or not.
#[derive(Clone, Copy, Debug)]
pub struct MonoTimer {
    frequency: Frequency,
    last_count: u32,
}

impl MonoTimer {
    /// Creates a new `Monotonic` timer
    pub fn new(mut dwt: DWT, clocks: Clocks) -> Self {
        dwt.enable_cycle_counter();

        // now the CYCCNT counter can't be stopped or resetted
        drop(dwt);

        MonoTimer {
            frequency: clocks.sysclk(),
            last_count: DWT::get_cycle_count()
        }
    }

    fn update_count(&mut self) {
        self.last_count = self.get_current();
    }
}

impl Timer for MonoTimer {

    type U = u32;

    /// Pause the execution for Duration.
    fn delay(&mut self, d: Duration) {
        let mut ticks = self.frequency.ticks_in(d);
        self.update_count();
        while ticks != 0 {
            let remaining = u32::max_value() - self.last_count;
            if ticks > u64(remaining) {
                // Wait for a full cycle.
                while !self.has_wrapped() {}
                self.update_count();
                ticks -= u64(remaining);

            } else {
                while ticks < u64(self.get_current() - self.last_count) {}
            }
        }
    }

    /// Start a timer counter
    /// The timer is being move and dedicated
    /// to the instant needs.
    fn start(mut self) ->  TimerInstant<Self> {
        TimerInstant::now(MonoTimer {
            frequency: self.frequency,
            last_count: self.get_current()
        })
    }

    /// Stop the counting timer.
    /// This method is only used by `TimerInstant` to release the timer.
    fn stop(mut self) -> Self {
        MonoTimer {
            frequency: self.frequency,
            last_count: self.get_current()
        }
    }

    /// Test if the counter has wrapped to its initial value
    fn has_wrapped(&mut self) -> bool {
        // TODO if wrapped twice it does not work.
        self.get_current() < self.last_count
    }

    /// The maximum / minimum value.
    /// For count down timer this should be the maximum value. Or the reload value.
    /// For count up limit_value should return 0.
    fn limit_value(&self) -> Self::U {
        0
    }

    /// Return the current counter value.
    fn get_current(&mut self) -> Self::U {
        DWT::get_cycle_count()
    }

    /// Return the duration between 2 counted value.
    fn tick(&mut self) -> Duration {
        self.frequency.tick()
    }
}
