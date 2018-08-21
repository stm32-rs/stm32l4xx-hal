//! Touch sense controller

use rcc::AHB1;
use stm32l4::stm32l4x2::{TSC};
use gpio::gpiob::{PB4, PB5, PB6, PB7};
use gpio::{AF9, Alternate, Output, OpenDrain, PushPull};

#[derive(Debug)]
pub enum Event {
    /// Max count error
    MaxCountError,
    /// End of acquisition
    EndOfAcquisition
}

// TODO macro to impl all possible channel/sample pin combinations
pub trait SamplePin<TSC> {}
impl SamplePin<TSC> for PB4<Alternate<AF9, Output<OpenDrain>>> {}

pub trait ChannelPin<TSC> {
    const OFFSET: u32;
}
impl ChannelPin<TSC> for PB5<Alternate<AF9, Output<PushPull>>> {
    const OFFSET: u32 = 1 << 1 + (1 * 4);
}
impl ChannelPin<TSC> for PB6<Alternate<AF9, Output<PushPull>>> {
    const OFFSET: u32 = 1 << 2 + (1 * 4);
}
impl ChannelPin<TSC> for PB7<Alternate<AF9, Output<PushPull>>> {
    const OFFSET: u32 = 1 << 3 + (1 * 4);
}


// TODO currently requires all the pins even if a user wants one channel, fix
pub struct Tsc<SPIN> {
    sample_pin: SPIN,
    // pins: PINS,
    tsc: TSC
}

impl<SPIN> Tsc<SPIN> {
    pub fn tsc(tsc: TSC, sample_pin: SPIN, ahb: &mut AHB1) -> Self
        where SPIN: SamplePin<TSC> // PINS: ChannelPins<TSC>,
    {
        /* Enable the peripheral clock */
        ahb.enr().modify(|_, w| w.tscen().set_bit());
        ahb.rstr().modify(|_, w| w.tscrst().set_bit());
        ahb.rstr().modify(|_, w| w.tscrst().clear_bit());

        tsc.cr.write(|w| unsafe {
            w.ctph()
                .bits((1 << 28) as u8)
                .ctpl()
                .bits((1 << 24) as u8)
                .sse()
                .clear_bit()
                .pgpsc()
                .bits((2 << 12) as u8)
                .mcv()
                // 000: 255
                // 001: 511
                // 010: 1023
                // 011: 2047
                // 100: 4095
                // 101: 8191
                // 110: 16383
                .bits(0b101) // TODO make this value configurable
                .tsce()
                .set_bit()
        });

        // TODO allow configuration
        
        // Schmitt trigger hysteresis on all used TSC IOs
        tsc.iohcr.write(|w| {
            w.g2_io1().set_bit()
                .g2_io2().set_bit()
                .g2_io3().set_bit()
                .g2_io4().set_bit()
        });

        // Set the sampling pin
        tsc.ioscr.write(|w| { w.g2_io1().set_bit() });
        
        // set the acquisitiuon groups based of the channel pins, stm32l432xx only has group 2
        tsc.iogcsr.write(|w| { w.g2e().set_bit() });

        // clear interrupt & flags
        tsc.icr.write(|w| { 
            w.eoaic().set_bit()
                .mceic().set_bit()
        });

        Tsc {
            tsc: tsc,
            sample_pin: sample_pin,
            // pins: pins,
        }
    }

    /// Starts a charge acquisition
    pub fn start<PIN>(&self, _input: &mut PIN) 
        where PIN: ChannelPin<TSC>
    {
        // clear interrupt & flags
        self.tsc.icr.write(|w| { 
            w.eoaic().set_bit()
                .mceic().set_bit()
        });

        // discharge the caps ready for a new reading
        self.tsc.cr.modify(|_, w| {
            w.iodef().clear_bit()
        });

        // Set the channel pin
        self.tsc.ioccr.write(|w| unsafe {
            w.bits(PIN::OFFSET)
        });

        self.tsc.cr.modify(|_, w| { w.start().set_bit() });
    }

    /// Blocks waiting for a acquisition to complete or for a Max Count Error
    pub fn wait(&self) -> Result<u16, Event> {
        loop {
            let isr = self.tsc.isr.read();
            if isr.eoaf().bit_is_set() {
                break Ok(self.tsc.iog2cr.read().cnt().bits());
            } else if isr.mcef().bit_is_set() {
                break Err(Event::MaxCountError);
            }
        }
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
    pub fn free(self) -> (TSC, SPIN) {
        (self.tsc, self.sample_pin)
    }
}