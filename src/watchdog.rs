//! Watchdog peripherals

use crate::{
    hal::watchdog::{Watchdog, WatchdogEnable},
    stm32::{DBGMCU, IWDG},
    time::MilliSeconds,
};

/// Wraps the Independent Watchdog (IWDG) peripheral
pub struct IndependentWatchdog {
    iwdg: IWDG,
}

const LSI_KHZ: u32 = 32;
const MAX_PR: u32 = 0b110;
const MAX_RL: u16 = 0xFFF;
const KR_ACCESS: u16 = 0x5555;
const KR_RELOAD: u16 = 0xAAAA;
const KR_START: u16 = 0xCCCC;

impl IndependentWatchdog {
    /// Creates a new `IndependentWatchDog` without starting it. Call `start` to start the watchdog.
    /// See `WatchdogEnable` and `Watchdog` for more info.
    pub fn new(iwdg: IWDG) -> Self {
        IndependentWatchdog { iwdg }
    }

    /// Debug independent watchdog stopped when core is halted
    pub fn stop_on_debug(&self, dbgmcu: &DBGMCU, stop: bool) {
        #[cfg(any(
            feature = "stm32l431",
            feature = "stm32l451",
            feature = "stm32l471",
            feature = "stm32l412",
            feature = "stm32l422",
            feature = "stm32l432",
            feature = "stm32l442",
            feature = "stm32l452",
            feature = "stm32l462",
            feature = "stm32l433",
            feature = "stm32l443",
            feature = "stm32l475",
            feature = "stm32l476",
            feature = "stm32l486",
            feature = "stm32l496",
            feature = "stm32l4a6",
            feature = "stm32l4r5",
            feature = "stm32l4s5",
            feature = "stm32l4r9",
            feature = "stm32l4s9",
        ))]
        dbgmcu.apb1fzr1.modify(|_, w| w.dbg_iwdg_stop().bit(stop));
        #[cfg(not(any(
            feature = "stm32l431",
            feature = "stm32l451",
            feature = "stm32l471",
            feature = "stm32l412",
            feature = "stm32l422",
            feature = "stm32l432",
            feature = "stm32l442",
            feature = "stm32l452",
            feature = "stm32l462",
            feature = "stm32l433",
            feature = "stm32l443",
            feature = "stm32l475",
            feature = "stm32l476",
            feature = "stm32l486",
            feature = "stm32l496",
            feature = "stm32l4a6",
            feature = "stm32l4r5",
            feature = "stm32l4s5",
            feature = "stm32l4r9",
            feature = "stm32l4s9"
        )))]
        dbgmcu.apb1_fzr1.modify(|_, w| w.dbg_iwdg_stop().bit(stop));
    }

    /// Sets the watchdog timer timout period. Max: 32768 ms
    fn setup(&self, timeout_ms: MilliSeconds) {
        assert!(timeout_ms.ticks() < (1 << 15), "Watchdog timeout to high");
        let pr = match timeout_ms.ticks() {
            t if t == 0 => 0b000, // <= (MAX_PR + 1) * 4 / LSI_KHZ => 0b000,
            t if t <= (MAX_PR + 1) * 8 / LSI_KHZ => 0b001,
            t if t <= (MAX_PR + 1) * 16 / LSI_KHZ => 0b010,
            t if t <= (MAX_PR + 1) * 32 / LSI_KHZ => 0b011,
            t if t <= (MAX_PR + 1) * 64 / LSI_KHZ => 0b100,
            t if t <= (MAX_PR + 1) * 128 / LSI_KHZ => 0b101,
            _ => 0b110,
        };

        let max_period = Self::timeout_period(pr, MAX_RL);
        let max_rl = u32::from(MAX_RL);
        let rl = (timeout_ms.ticks() * max_rl / max_period).min(max_rl) as u16;

        self.access_registers(|iwdg| {
            iwdg.pr.modify(|_, w| w.pr().bits(pr));
            iwdg.rlr.modify(|_, w| w.rl().bits(rl));
        });
    }

    fn is_pr_updating(&self) -> bool {
        self.iwdg.sr.read().pvu().bit()
    }

    /// Returns the interval in ms
    pub fn interval(&self) -> MilliSeconds {
        while self.is_pr_updating() {}

        let pr = self.iwdg.pr.read().pr().bits();
        let rl = self.iwdg.rlr.read().rl().bits();
        let ms = Self::timeout_period(pr, rl);
        MilliSeconds::from_ticks(ms)
    }

    /// pr: Prescaler divider bits, rl: reload value
    ///
    /// Returns timeout period in ms
    fn timeout_period(pr: u8, rl: u16) -> u32 {
        let divider: u32 = match pr {
            0b000 => 4,
            0b001 => 8,
            0b010 => 16,
            0b011 => 32,
            0b100 => 64,
            0b101 => 128,
            0b110 => 256,
            0b111 => 256,
            _ => unreachable!(),
        };
        (u32::from(rl) + 1) * divider / LSI_KHZ
    }

    fn access_registers<A, F: FnMut(&IWDG) -> A>(&self, mut f: F) -> A {
        // Unprotect write access to registers
        self.iwdg.kr.write(|w| unsafe { w.key().bits(KR_ACCESS) });
        let a = f(&self.iwdg);

        // Protect again
        self.iwdg.kr.write(|w| unsafe { w.key().bits(KR_RELOAD) });
        a
    }
}

impl WatchdogEnable for IndependentWatchdog {
    type Time = MilliSeconds;

    fn start<T: Into<Self::Time>>(&mut self, period: T) {
        self.setup(period.into());

        self.iwdg.kr.write(|w| unsafe { w.key().bits(KR_START) });
    }
}

impl Watchdog for IndependentWatchdog {
    fn feed(&mut self) {
        self.iwdg.kr.write(|w| unsafe { w.key().bits(KR_RELOAD) });
    }
}
