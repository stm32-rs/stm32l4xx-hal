//! Time units

pub use fugit::{
    HertzU32 as Hertz, KilohertzU32 as KiloHertz, MegahertzU32 as MegaHertz,
    MicrosDurationU32 as MicroSeconds, MillisDurationU32 as MilliSeconds,
};

use crate::rcc::Clocks;
use cortex_m::peripheral::DWT;

/// Bits per second
#[derive(Clone, Copy, Debug)]
pub struct Bps(pub u32);

/// Extension trait that adds convenience methods to the `u32` type
pub trait U32Ext {
    /// Wrap in `Bps`
    fn bps(self) -> Bps;
}

impl U32Ext for u32 {
    fn bps(self) -> Bps {
        Bps(self)
    }
}

/// A monotonic nondecreasing timer
#[derive(Clone, Copy, Debug)]
pub struct MonoTimer {
    frequency: Hertz,
}

impl MonoTimer {
    /// Creates a new `Monotonic` timer
    pub fn new(mut dwt: DWT, clocks: Clocks) -> Self {
        dwt.enable_cycle_counter();

        // now the CYCCNT counter can't be stopped or resetted
        drop(dwt);

        MonoTimer {
            frequency: clocks.sysclk(),
        }
    }

    /// Returns the frequency at which the monotonic timer is operating at
    pub fn frequency(&self) -> Hertz {
        self.frequency
    }

    /// Returns an `Instant` corresponding to "now"
    pub fn now(&self) -> Instant {
        Instant {
            now: DWT::cycle_count(),
        }
    }
}

/// A measurement of a monotonically nondecreasing clock
#[derive(Clone, Copy, Debug)]
pub struct Instant {
    now: u32,
}

impl Instant {
    /// Ticks elapsed since the `Instant` was created
    pub fn elapsed(&self) -> u32 {
        DWT::cycle_count().wrapping_sub(self.now)
    }
}
