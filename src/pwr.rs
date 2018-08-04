// TODO impl power module so we enable backup domain for rtc etc
// apb1r1.enr().modify(|_, w| w.pwren().set_bit());

use rcc::{APB1R1};
use stm32l4::stm32l4x2::{pwr, PWR};


pub struct Pwr {
    pub cr1: CR1,
    pub cr2: CR2,
    pub cr3: CR3,
    pub cr4: CR4,
}

impl Pwr {
    pub fn pwr(apb1r1: &mut APB1R1) -> Self {
        apb1r1.enr().modify(|_, w| w.pwren().set_bit());

        Self {
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