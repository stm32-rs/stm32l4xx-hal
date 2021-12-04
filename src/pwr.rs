//! Power management

use crate::rcc::{Enable, APB1R1, Clocks};
use crate::stm32::{pwr, PWR};
use crate::time::U32Ext;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum VosRange {
    #[doc = "High-Performance range, 1.2V, up to 80 MHz"]
    HighPerformance = 0b01,
    #[doc = "Low-power range, 1.0V, up to 26MHz"]
    LowPower = 0b10
}

pub struct Pwr {
    pub cr1: CR1,
    pub cr2: CR2,
    pub cr3: CR3,
    pub cr4: CR4,
}

impl Pwr {
    /// Configures dynamic voltage regulator range
    ///
    /// Will panic if low-power range is selected for higher system clock
    pub fn set_power_range(&mut self, range: VosRange, clocks: &Clocks) {
        match range {
            VosRange::HighPerformance => {unsafe {self.cr1.reg().modify(|_,w| w.vos().bits(VosRange::HighPerformance as u8))}}
            VosRange::LowPower => {if clocks.sysclk() > 26.mhz().into() {
                panic!("Unable to switch power regulator to low-power voltage due to the too high system clock frequency")
            } else {
                unsafe {self.cr1.reg().modify(|_,w| w.vos().bits(VosRange::LowPower as u8))}
            }}
        }
    }

    /// Switches the system into low power run mode
    pub fn low_power_run(&mut self, clocks: &Clocks) {
        if clocks.sysclk() > 2.mhz().into() {
            panic!("Unable to switch to lower power run due to the too high system clock frequency")
        }
        self.cr1.reg().modify(|_, w| w.lpr().set_bit())
    }
}

/// Extension trait that constrains the `PWR` peripheral
pub trait PwrExt {
    /// Constrains the `PWR` peripheral so it plays nicely with the other abstractions
    fn constrain(self, _: &mut APB1R1) -> Pwr;
}

impl PwrExt for PWR {
    fn constrain(self, apb1r1: &mut APB1R1) -> Pwr {
        // Enable the peripheral clock
        PWR::enable(apb1r1);
        Pwr {
            cr1: CR1 { _0: () },
            cr2: CR2 { _0: () },
            cr3: CR3 { _0: () },
            cr4: CR4 { _0: () },
        }
    }
}

/// CR1
pub struct CR1 {
    _0: (),
}

impl CR1 {
    // TODO remove `allow`
    #[allow(dead_code)]
    pub(crate) fn reg(&mut self) -> &pwr::CR1 {
        // NOTE(unsafe) this proxy grants exclusive access to this register
        unsafe { &(*PWR::ptr()).cr1 }
    }
}
/// CR2
pub struct CR2 {
    _0: (),
}

impl CR2 {
    // TODO remove `allow`
    #[allow(dead_code)]
    pub(crate) fn reg(&mut self) -> &pwr::CR2 {
        // NOTE(unsafe) this proxy grants exclusive access to this register
        unsafe { &(*PWR::ptr()).cr2 }
    }
}
/// CR3
pub struct CR3 {
    _0: (),
}

impl CR3 {
    // TODO remove `allow`
    #[allow(dead_code)]
    pub(crate) fn reg(&mut self) -> &pwr::CR3 {
        // NOTE(unsafe) this proxy grants exclusive access to this register
        unsafe { &(*PWR::ptr()).cr3 }
    }
}
/// CR4
pub struct CR4 {
    _0: (),
}

impl CR4 {
    // TODO remove `allow`
    #[allow(dead_code)]
    pub(crate) fn reg(&mut self) -> &pwr::CR4 {
        // NOTE(unsafe) this proxy grants exclusive access to this register
        unsafe { &(*PWR::ptr()).cr4 }
    }
}
