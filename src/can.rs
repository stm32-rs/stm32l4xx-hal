//! # Controller Area Network (CAN) Interface
//!
//! Based on STM32F4xx HAL.

use crate::pac::CAN1;

mod sealed {
    pub trait Sealed {}
}

/// A pair of (TX, RX) pins configured for CAN communication
pub trait Pins: sealed::Sealed {
    /// The CAN peripheral that uses these pins
    type Instance;
}

/// Implements sealed::Sealed and Pins for a (TX, RX) pair of pins associated with a CAN peripheral
/// The alternate function number can be specified after each pin name.
macro_rules! pins {
    ($($PER:ident => ($tx:ident<$txaf:ident>, $rx:ident<$rxaf:ident>),)+) => {
        $(
            impl crate::can::sealed::Sealed for ($tx<crate::gpio::Alternate<$txaf, Input<Floating>>>, $rx<crate::gpio::Alternate<$rxaf, Input<Floating>>>) {}
            impl crate::can::Pins for ($tx<crate::gpio::Alternate<$txaf, Input<Floating>>>, $rx<crate::gpio::Alternate<$rxaf, Input<Floating>>>) {
                type Instance = $PER;
            }
        )+
    }
}

mod common_pins {
    use crate::gpio::{
        gpioa::{PA11, PA12},
        gpiob::{PB8, PB9},
        gpiod::{PD0, PD1},
        Floating, Input, AF9,
    };
    use crate::pac::CAN1;

    // All STM32L4 models with CAN support these pins
    pins! {
        CAN1 => (PA12<AF9>, PA11<AF9>),
        CAN1 => (PD1<AF9>, PD0<AF9>),
        CAN1 => (PB9<AF9>, PB8<AF9>),
    }
}

#[cfg(feature = "stm32l4x1")]
mod pb13_pb12_af10 {
    use crate::gpio::{
        gpiob::{PB12, PB13},
        Floating, Input, AF10,
    };
    use crate::pac::CAN1;
    pins! { CAN1 => (PB13<AF10>, PB12<AF10>), }
}

/// Enable/disable peripheral
pub trait Enable: sealed::Sealed {
    /// Enables this peripheral by setting the associated enable bit in an RCC enable register
    fn enable();
}

impl crate::can::sealed::Sealed for crate::pac::CAN1 {}

impl crate::can::Enable for crate::pac::CAN1 {
    #[inline(always)]
    fn enable() {
        unsafe {
            // NOTE(unsafe) this reference will only be used for atomic writes with no side effects.
            let rcc = &(*crate::pac::RCC::ptr());
            // Enable peripheral clock
            rcc.apb1enr1.modify(|_, w| w.can1en().set_bit());
            rcc.apb1rstr1.modify(|_, w| w.can1rst().set_bit());
            rcc.apb1rstr1.modify(|_, w| w.can1rst().clear_bit());
        };
    }
}

/// Interface to the CAN peripheral.
pub struct Can<Instance> {
    _peripheral: Instance,
}

impl<Instance> Can<Instance>
where
    Instance: Enable,
{
    /// Creates a CAN interface.
    pub fn new<P>(can: Instance, _pins: P) -> Can<Instance>
    where
        P: Pins<Instance = Instance>,
    {
        Instance::enable();
        Can { _peripheral: can }
    }

    pub fn new_unchecked(can: Instance) -> Can<Instance> {
        Instance::enable();
        Can { _peripheral: can }
    }
}

unsafe impl bxcan::Instance for Can<CAN1> {
    const REGISTERS: *mut bxcan::RegisterBlock = CAN1::ptr() as *mut _;
}

unsafe impl bxcan::FilterOwner for Can<CAN1> {
    const NUM_FILTER_BANKS: u8 = 14;
}

unsafe impl bxcan::MasterInstance for Can<CAN1> {}
