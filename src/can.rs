//! # Controller Area Network (CAN) Interface
//!
//! Based on STM32F4xx HAL.

use crate::alternate_functions::{CanRxPin, CanTxPin};
use crate::gpio::AlternatePP;
use crate::pac::CAN1;
use crate::rcc::{Enable, RccBus, Reset};

mod sealed {
    pub trait Sealed {}
}

impl crate::can::sealed::Sealed for crate::pac::CAN1 {}

/// Interface to the CAN peripheral.
pub struct Can<CAN, PINS> {
    can: CAN,
    pins: PINS,
}

impl<CAN, TX, RX> Can<CAN, (TX, RX)>
where
    CAN: Enable + Reset,
    TX: CanTxPin<CAN> + AlternatePP,
    RX: CanRxPin<CAN> + AlternatePP,
{
    /// Creates a CAN interface.
    pub fn new(apb: &mut <CAN as RccBus>::Bus, can: CAN, pins: (TX, RX)) -> Can<CAN, (TX, RX)> {
        CAN::enable(apb);
        CAN::reset(apb);
        Can { can, pins }
    }

    // Split the peripheral back into its components.
    pub fn split(self) -> (CAN, (TX, RX)) {
        (self.can, self.pins)
    }
}

unsafe impl<PINS> bxcan::Instance for Can<CAN1, PINS> {
    const REGISTERS: *mut bxcan::RegisterBlock = CAN1::ptr() as *mut _;
}

unsafe impl<PINS> bxcan::FilterOwner for Can<CAN1, PINS> {
    const NUM_FILTER_BANKS: u8 = 14;
}

unsafe impl<PINS> bxcan::MasterInstance for Can<CAN1, PINS> {}
