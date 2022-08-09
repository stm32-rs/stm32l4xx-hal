//! # Controller Area Network (CAN) Interface
//!
//! Based on STM32F4xx HAL.

use crate::pac::CAN1;
use crate::rcc::{Enable, RccBus, Reset};

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
    ($($PER:ident => ($tx:ident<$txaf:literal>, $rx:ident<$rxaf:literal>),)+) => {
        $(
            impl crate::can::sealed::Sealed for ($tx<crate::gpio::Alternate<PushPull, $txaf, >>, $rx<crate::gpio::Alternate<PushPull, $rxaf>>) {}
            impl crate::can::Pins for ($tx<crate::gpio::Alternate<PushPull, $txaf, >>, $rx<crate::gpio::Alternate<PushPull, $rxaf>>) {
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
        PushPull,
    };
    use crate::pac::CAN1;

    // All STM32L4 models with CAN support these pins
    pins! {
        CAN1 => (PA12<9>, PA11<9>),
        CAN1 => (PD1<9>, PD0<9>),
        CAN1 => (PB9<9>, PB8<9>),
    }
}

// L4x1
#[cfg(any(feature = "stm32l431", feature = "stm32l451", feature = "stm32l471"))]
mod pb13_pb12_af10 {
    use crate::gpio::{
        gpiob::{PB12, PB13},
        PushPull,
    };
    use crate::pac::CAN1;
    pins! { CAN1 => (PB13<10>, PB12<10>), }
}

impl crate::can::sealed::Sealed for crate::pac::CAN1 {}

/// Interface to the CAN peripheral.
pub struct Can<CAN, Pins> {
    can: CAN,
    pins: Pins,
}

impl<CAN, P> Can<CAN, P>
where
    CAN: Enable + Reset,
    P: Pins<Instance = CAN>,
{
    /// Creates a CAN interface.
    pub fn new(apb: &mut <CAN as RccBus>::Bus, can: CAN, pins: P) -> Can<CAN, P> {
        CAN::enable(apb);
        CAN::reset(apb);
        Can { can, pins }
    }

    // Split the peripheral back into its components.
    pub fn split(self) -> (CAN, P) {
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
