//! Delays

use cast::u32;
use cortex_m::asm;
use cortex_m::peripheral::syst::SystClkSource;
use cortex_m::peripheral::SYST;

use crate::hal::blocking::delay::{DelayMs, DelayUs};
use crate::rcc::Clocks;
use crate::time::Hertz;

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

/// System timer (SysTick) as a delay provider.
impl DelayMs<u32> for Delay {
    fn delay_ms(&mut self, ms: u32) {
        self.delay_us(ms * 1_000);
    }
}

impl DelayMs<u16> for Delay {
    fn delay_ms(&mut self, ms: u16) {
        self.delay_ms(u32(ms));
    }
}

impl DelayMs<u8> for Delay {
    fn delay_ms(&mut self, ms: u8) {
        self.delay_ms(u32(ms));
    }
}

impl DelayUs<u32> for Delay {
    fn delay_us(&mut self, us: u32) {
        // The SysTick Reload Value register supports values between 1 and 0x00FFFFFF.
        const MAX_RVR: u32 = 0x00FF_FFFF;

        let mut total_rvr = us * (self.clocks.sysclk().0 / 1_000_000);

        while total_rvr != 0 {
            let current_rvr = if total_rvr <= MAX_RVR {
                total_rvr
            } else {
                MAX_RVR
            };

            self.syst.set_reload(current_rvr);
            self.syst.clear_current();
            self.syst.enable_counter();

            // Update the tracking variable while we are waiting...
            total_rvr -= current_rvr;

            while !self.syst.has_wrapped() {}

            self.syst.disable_counter();
        }
    }
}

impl DelayUs<u16> for Delay {
    fn delay_us(&mut self, us: u16) {
        self.delay_us(u32(us))
    }
}

impl DelayUs<u8> for Delay {
    fn delay_us(&mut self, us: u8) {
        self.delay_us(u32(us))
    }
}

/// Cortex-M `asm::delay` as provider
#[derive(Clone, Copy)]
pub struct DelayCM {
    sysclk: Hertz,
}

impl DelayCM {
    /// Create a new delay
    pub fn new(clocks: Clocks) -> Self {
        DelayCM {
            sysclk: clocks.sysclk(),
        }
    }

    /// Create a new delay that is unchecked. The user needs to know the `sysclk`.
    ///
    /// # Safety
    /// Sysclk must be the same as the actual clock frequency of the chip
    pub unsafe fn new_unchecked(sysclk: Hertz) -> Self {
        DelayCM { sysclk }
    }
}

impl DelayMs<u32> for DelayCM {
    fn delay_ms(&mut self, ms: u32) {
        self.delay_us(ms * 1_000);
    }
}

impl DelayMs<u16> for DelayCM {
    fn delay_ms(&mut self, ms: u16) {
        self.delay_ms(u32(ms));
    }
}

impl DelayMs<u8> for DelayCM {
    fn delay_ms(&mut self, ms: u8) {
        self.delay_ms(u32(ms));
    }
}

impl DelayUs<u32> for DelayCM {
    fn delay_us(&mut self, us: u32) {
        // Max delay is 53_687_091 us at 80 MHz
        let ticks = self.sysclk.0 / 1_000_000;

        asm::delay(ticks * us);
    }
}

impl DelayUs<u16> for DelayCM {
    fn delay_us(&mut self, us: u16) {
        self.delay_us(u32(us))
    }
}

impl DelayUs<u8> for DelayCM {
    fn delay_us(&mut self, us: u8) {
        self.delay_us(u32(us))
    }
}
