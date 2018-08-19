//! Touch sense controller

use rcc::AHB1;
use stm32l4::stm32l4x2::{TSC};
use gpio::gpiob::{PB4, PB5, PB6, PB7};
use gpio::{AF9};

pub enum Event {
    /// Max count error
    MaxCountError,
    /// End of acquisition
    EndOfAcquisition
}

// TODO currently requires all the pins even if a user wants one channel, fix
pub trait Pins<TSC> {
    const REMAP: bool;
}

impl Pins<TSC> for (PB4<AF9>, PB5<AF9>, PB6<AF9>, PB7<AF9>) {
    const REMAP: bool = false; // TODO REMAP
}

pub struct Tsc<PINS> {
    pins: PINS,
    // num_channels: usize,
    // channel_pins: [PINS; 3], // upto 3 channels on the stm32l432xx
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

        tsc.cr.write(|w| unsafe {
            w.ctph()
                .bits(1 << 28)
                .ctpl()
                .bits(1 << 24)
                .sse()
                .clear_bit()
                .pgpsc()
                .bits(2 << 12)
                .mcv()
                // 000: 255
                // 001: 511
                // 010: 1023
                // 011: 2047
                // 100: 4095
                // 101: 8191
                // 110: 16383
                .bits(0b101) // TODO make this value configurable
        });

        // TODO this may require the pin index, maybe make into_touch_pin in gpio.rs

        // TODO allow configuration
        // Set the sampling pin
        tsc.ioscr.write(|w| { w.g2_io1().set_bit() });

        // Set the channel pin(s)
        tsc.ioccr.write(|w| {
            w.g2_io2().set_bit()
                .g2_io3().set_bit()
                .g2_io4().set_bit()
        });
        
        // set the acquisitiuon groups based of the channel pins, stm32l432xx only has group 2
        tsc.iogcsr.write(|w| { w.g2e().set_bit() });

        Tsc {
            tsc: tsc,
            pins: pins,
            // num_channels: 0,
            // channel_pins: [PINS; 3]
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