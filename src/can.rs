//! # Controller Area Network (CAN) Interface
//!
//! Based on STM32F4xx HAL.

use crate::pac::CAN1;
use crate::rcc::APB1R1;

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
            impl crate::can::sealed::Sealed for ($tx<crate::gpio::Alternate<$txaf, PushPull>>, $rx<crate::gpio::Alternate<$rxaf, PushPull>>) {}
            impl crate::can::Pins for ($tx<crate::gpio::Alternate<$txaf, PushPull>>, $rx<crate::gpio::Alternate<$rxaf, PushPull>>) {
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
        PushPull, AF9,
    };
    use crate::pac::CAN1;

    // All STM32L4 models with CAN support these pins
    pins! {
        CAN1 => (PA12<AF9>, PA11<AF9>),
        CAN1 => (PD1<AF9>, PD0<AF9>),
        CAN1 => (PB9<AF9>, PB8<AF9>),
    }
}

#[cfg(feature = "private_pac_stm32l4x1")]
mod pb13_pb12_af10 {
    use crate::gpio::{
        gpiob::{PB12, PB13},
        PushPull, AF10,
    };
    use crate::pac::CAN1;
    pins! { CAN1 => (PB13<AF10>, PB12<AF10>), }
}

/// Enable/disable peripheral
pub trait Enable: sealed::Sealed {
    /// Enables this peripheral by setting the associated enable bit in an RCC enable register
    fn enable(apb: &mut APB1R1);
}

impl crate::can::sealed::Sealed for crate::pac::CAN1 {}

impl crate::can::Enable for crate::pac::CAN1 {
    #[inline(always)]
    fn enable(apb: &mut APB1R1) {
        // Enable peripheral clock
        apb.enr().modify(|_, w| w.can1en().set_bit());
        apb.rstr().modify(|_, w| w.can1rst().set_bit());
        apb.rstr().modify(|_, w| w.can1rst().clear_bit());
    }
}

/// Interface to the CAN peripheral.
pub struct Can<Instance, Pins> {
    can: Instance,
    pins: Pins,
}

impl<Instance, P> Can<Instance, P>
where
    Instance: Enable,
    P: Pins<Instance = Instance>,
{
    /// Creates a CAN interface.
    pub fn new(apb: &mut APB1R1, can: Instance, pins: P) -> Can<Instance, P> {
        Instance::enable(apb);
        Can { can, pins }
    }

    // Split the peripheral back into its components.
    pub fn split(self) -> (Instance, P) {
        (self.can, self.pins)
    }
}

unsafe impl<Pins> bxcan::Instance for Can<CAN1, Pins> {
    const REGISTERS: *mut bxcan::RegisterBlock = CAN1::ptr() as *mut _;
}

unsafe impl<Pins> bxcan::FilterOwner for Can<CAN1, Pins> {
    const NUM_FILTER_BANKS: u8 = 14;
}

unsafe impl<Pins> bxcan::MasterInstance for Can<CAN1, Pins> {}
