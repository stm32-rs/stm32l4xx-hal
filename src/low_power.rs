//! This module contains code used to place the STM32L4 in low power modes.
//! Reference section 5.3.3: `Low power modes` of the Reference Manual.

use crate::pac::{PWR, RCC};
use cortex_m::{asm::wfi, peripheral::SCB};

// These enums are better suited for a clocks or rcc module.
#[derive(Clone, Copy)]
#[repr(u8)]
pub enum PllSrc {
    Msi = 0b00, // todo: check bit values
    Hsi16 = 0b01,
    Hse = 0b10,
}

#[derive(Clone, Copy)]
pub enum InputSrc {
    Msi,
    Hsi16,
    Hse,
    Pll(PllSrc),
}

impl InputSrc {
    /// Required due to numerical value on non-uniform discrim being experimental.
    /// (ie, can't set on `Pll(Pllsrc)`.
    pub fn bits(&self) -> u8 {
        match self {
            Self::Msi => 0b00,  // todo check bit values
            Self::Hsi16 => 0b00,
            Self::Hse => 0b01,
            Self::Pll(_) => 0b10,
        }
    }
}

// See L4 Reference Manual section 5.3.6. The values correspond
// todo PWR_CR1, LPMS field.
#[derive(Clone, Copy)]
#[repr(u8)]
pub enum StopMode {
    Zero = 0b000,
    One = 0b001,
    Two = 0b010,
}

/// Re-select innput source; used on Stop and Standby modes, where the system reverts
/// to HSI after wake.
fn re_select_input(input_src: InputSrc) {
    // Re-select the input source; it will revert to HSI during `Stop` or `Standby` mode.

    // Note: It would save code repetition to pass the `Clocks` struct in and re-run setup
    // todo: But this saves a few reg writes.
    match input_src {
        InputSrc::Hse => unsafe {
            (*RCC::ptr()).cr.modify(|_, w| w.hseon().set_bit());
            while (*RCC::ptr()).cr.read().hserdy().bit().is_clear() {}

            (*RCC::ptr())
                .cfgr
                .modify(|_, w| w.sw().bits(input_src.bits()));
        },
        InputSrc::Pll(_) => unsafe {
            // todo: DRY with above.
            (*RCC::ptr()).cr.modify(|_, w| w.hseon().set_bit());
            while (*RCC::ptr()).cr.read().hserdy().is_not_ready() {}

            (*RCC::ptr()).cr.modify(|_, w| w.pllon().off());
            while (*RCC::ptr()).cr.read().pllrdy().bit().is_set() {}

            (*RCC::ptr())
                .cfgr
                .modify(|_, w| w.sw().bits(input_src.bits()));

            (*RCC::ptr()).cr.modify(|_, w| w.pllon().on());
            while (*RCC::ptr()).cr.read().pllrdy().bit().is_clear() {}
        },
        InputSrc::Hsi => (), // Already reset to this.
    }
}

/// Ref man, table 24
/// Note that this assumes you've already reduced clock frequency below 2 Mhz.
pub fn low_power_run(scb: &mut SCB) {
    // Decrease the system clock frequency below 2 MHz
    // LPR = 1
    unsafe { (*PWR::ptr()) }
        .cr1
        .modify(|_, w| w.lpr().bit.set());
}

/// Ref man, table 24
/// Return to normal run mode from low-power run. Requires you to increase the clock speed
/// manually after running this.
pub fn return_from_low_power_run(scb: &mut SCB) {
    // LPR = 0
    unsafe { (*PWR::ptr()) }
        .cr1
        .modify(|_, w| w.lpr().bit.clear());

    // Wait until REGLPF = 0
    while unsafe { (*PWR::ptr()) }.sr2.read().regplf().bit_is_set() {}

    // Increase the system clock frequency
}

/// Place the system in sleep now mode. To enter `low-power sleep now`, enter low power mode
/// (eg `low_power_mode()`) before running this. Ref man, table 25 and 26
pub fn sleep_now(scb: &mut SCB) {
    // WFI (Wait for Interrupt) (eg `cortext_m::asm::wfi()) or WFE (Wait for Event) while:
    // – SLEEPDEEP = 0
    // – No interrupt (for WFI) or event (for WFE) is pending
    scb.clear_sleepdeep();

    // todo: Table 25 has a line through it, with the following below. Do we need it?
    // On return from ISR while:
    // // SLEEPDEEP = 0 and SLEEPONEXIT = 1
    // scb.clear_sleepdeep();
    // scb.set_sleeponexit();

    wfi();
}

/// Enter Stop 0, Stop 1, or Stop 2 modes. Reference manual, section 5.3.6. Tables 27, 28, and 29.
pub fn stop(scb: &mut SCB, pwr: &mut PWR, mode: StopMode, input_src: InputSrc) {
    // WFI (Wait for Interrupt) or WFE (Wait for Event) while:
    // – SLEEPDEEP bit is set in Cortex®-M4 System Control register
    scb.set_sleepdeep(); // 0
                         // – No interrupt (for WFI) or event (for WFE) is pending
                         // – LPMS = “000” in PWR_CR1
    unsafe { (*PWR::ptr()).cr1.modify(|_, w| w.lpms().bits(mode as u8)) }; // 0

    // On Return from ISR while:
    // – SLEEPDEEP bit is set in Cortex®-M4 System Control register
    // – SLEEPONEXIT = 1
    // – No interrupt is pending
    // – LPMS = “000” in PWR_CR1

    wfi();

    re_select_input(input_src); // todo?
}

/// Enter `Standby` mode: the lowest-power of the 3 low-power states avail on the
/// STM32f3.
/// To exit: WKUP pin rising edge, RTC alarm event’s rising edge, external Reset in
/// NRST pin, IWDG Reset.
/// Ref man, table 21.
pub fn standby(scb: &mut SCB, pwr: &mut PWR, input_src: InputSrc) {
    // WFI (Wait for Interrupt) or WFE (Wait for Event) while:

    // Set SLEEPDEEP bit in ARM® Cortex®-M4 System Control register
    scb.set_sleepdeep();

    // Set PDDS bit in Power Control register (PWR_CR)
    // This bit is set and cleared by software. It works together with the LPDS bit.
    // 0: Enter Stop mode when the CPU enters Deepsleep. The regulator status
    // depends on the LPDS bit.
    // 1: Enter Standby mode when the CPU enters Deepsleep.
    pwr.cr.modify(|_, w| w.pdds().set_bit());

    // Clear WUF bit in Power Control/Status register (PWR_CSR) (Must do this by setting CWUF bit in
    // PWR_CR.)
    pwr.cr.modify(|_, w| w.cwuf().set_bit());

    wfi();

    re_select_input(input_src);
}
