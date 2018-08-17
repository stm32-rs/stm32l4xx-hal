//! Touch sense controller

use rcc::AHB1;
use stm32l4::stm32l4x2::{TSC};

pub enum Event {
    /// Max count error
    MaxCountError,
    /// End of acquisition
    EndOfAcquisition
}

pub trait Pins<TSC> {
    const REMAP: u8;
}

pub struct Tsc<PINS> {
    pins: PINS,
    tsc: TSC
}

impl<PINS> Tsc<PINS> {
    pub fn tsc(tsc: TSC, pins: PINS, ahb: &mut AHB1) -> Self
        where PINS: Pins<TSC>
    {
        /* Enable the peripheral clock */
        ahb.enr().modify(|_, w| w.tscen().set_bit());
        ahb.rstr().modify(|_, w| w.tscrst().set_bit());
        ahb.rstr().modify(|_, w| w.tscrst().clear_bit());



        Tsc {
            tsc,
            pins
        }
    }

    /// Starts a charge acquisition
    pub fn start(&self) {

    }

    /// Blocks waiting for a acquisition to complete
    pub fn wait(&self) -> Result<(), ()> {
        Ok(())
    }

    /// Enables an interrupt event
    pub fn listen(&mut self, event: Event){
        match event {
            Event::EndOfAcquisition => {
                self.tsc.ier.modify(|_, w| w.eoaie().set_bit());
            },
            Event::MaxCountError => {
                self.tsc.ier.modify(|_, w| w.mceie().set_bit());
            },
        }
    }

    /// Disables an interrupt event
    pub fn unlisten(&self, event: Event) {
        match event {
            Event::EndOfAcquisition => {
                self.tsc.ier.modify(|_, w| w.eoaie().clear_bit());
            },
            Event::MaxCountError => {
                self.tsc.ier.modify(|_, w| w.mceie().clear_bit());
            },
        }
    }

    /// Releases the TSC peripheral and associated pins
    pub fn free(self) -> (TSC, PINS) {
        (self.tsc, self.pins)
    }
}