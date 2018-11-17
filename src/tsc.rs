//! Touch sense controller
//! 
//! From STM32 (https://www.st.com/content/ccc/resource/technical/document/application_note/9d/be/03/8c/5d/8c/49/50/DM00088471.pdf/files/DM00088471.pdf/jcr:content/translations/en.DM00088471.pdf): 
//! 
//! The Cs capacitance is a key parameter for sensitivity. For touchkey sensors, the Cs value is
//! usually comprised between 8.7nF to 22nF. For linear and rotary touch sensors, the value is
//! usually comprised between 47nF and 100nF. These values are given as reference for an
//! electrode fitting a human finger tip size across a few millimeters dielectric panel.

use crate::rcc::AHB1;
use stm32l4::stm32l4x2::{TSC};
use crate::gpio::gpiob::{PB4, PB5, PB6, PB7};
use crate::gpio::{AF9, Alternate, Output, OpenDrain, PushPull};

#[derive(Debug)]
pub enum Event {
    /// Max count error
    MaxCountError,
    /// End of acquisition
    EndOfAcquisition
}

#[derive(Debug)]
pub enum Error {
    /// Max count error
    MaxCountError,
    /// Wrong GPIO for reading
    InvalidPin
}

pub trait SamplePin<TSC> {
    const GROUP: u32;
    const OFFSET: u32;
}
impl SamplePin<TSC> for PB4<Alternate<AF9, Output<OpenDrain>>> {
    const GROUP: u32 = 2;
    const OFFSET: u32 = 0;
}
impl SamplePin<TSC> for PB5<Alternate<AF9, Output<OpenDrain>>> {
    const GROUP: u32 = 2;
    const OFFSET: u32 = 1;
}
impl SamplePin<TSC> for PB6<Alternate<AF9, Output<OpenDrain>>> {
    const GROUP: u32 = 2;
    const OFFSET: u32 = 2;
}
impl SamplePin<TSC> for PB7<Alternate<AF9, Output<OpenDrain>>> {
    const GROUP: u32 = 2;
    const OFFSET: u32 = 3;
}

pub trait ChannelPin<TSC> {
    const GROUP: u32;
    const OFFSET: u32;
}
impl ChannelPin<TSC> for PB4<Alternate<AF9, Output<PushPull>>> {
    const GROUP: u32 = 2;
    const OFFSET: u32 = 0;
}
impl ChannelPin<TSC> for PB5<Alternate<AF9, Output<PushPull>>> {
    const GROUP: u32 = 2;
    const OFFSET: u32 = 1;
}
impl ChannelPin<TSC> for PB6<Alternate<AF9, Output<PushPull>>> {
    const GROUP: u32 = 2;
    const OFFSET: u32 = 2;
}
impl ChannelPin<TSC> for PB7<Alternate<AF9, Output<PushPull>>> {
    const GROUP: u32 = 2;
    const OFFSET: u32 = 3;
}


pub struct Tsc<SPIN> {
    sample_pin: SPIN,
    tsc: TSC
}

pub struct Config {
    pub clock_prescale: Option<ClockPrescaler>,
    pub max_count_error: Option<MaxCountError>,
}

pub enum ClockPrescaler {
    Hclk = 0b000,
    HclkDiv2 = 0b001,
    HclkDiv4 = 0b010,
    HclkDiv8 = 0b011,
    HclkDiv16 = 0b100,
    HclkDiv32 = 0b101,
    HclkDiv64 = 0b110,
    HclkDiv128 = 0b111,
}

pub enum MaxCountError {
    /// 000: 255
    U255 = 000,
    /// 001: 511
    U511 = 001,
    /// 010: 1023
    U1023 = 010,
    /// 011: 2047
    U2047 = 011,
    /// 100: 4095
    U4095 = 100,
    /// 101: 8191
    U8191 = 101,
    /// 110: 16383
    U16383 = 110
}

impl<SPIN> Tsc<SPIN> {
    pub fn tsc(tsc: TSC, sample_pin: SPIN, ahb: &mut AHB1, cfg: Option<Config>) -> Self
        where SPIN: SamplePin<TSC>
    {
        /* Enable the peripheral clock */
        ahb.enr().modify(|_, w| w.tscen().set_bit());
        ahb.rstr().modify(|_, w| w.tscrst().set_bit());
        ahb.rstr().modify(|_, w| w.tscrst().clear_bit());

        let config = cfg.unwrap_or(Config {
            clock_prescale: None,
            max_count_error: None
        });

        tsc.cr.write(|w| unsafe {
            w.ctph()
                .bits((1 << 28) as u8)
                .ctpl()
                .bits((1 << 24) as u8)
                // TODO configure sse?
                .sse()
                .set_bit()
                .ssd()
                .bits(16)
                .pgpsc()
                .bits(config.clock_prescale.unwrap_or(ClockPrescaler::Hclk) as u8)
                .mcv()
                .bits(config.max_count_error.unwrap_or(MaxCountError::U8191) as u8)
                .tsce()
                .set_bit()
        });
        
        let bit_pos = SPIN::OFFSET + (4 * (SPIN::GROUP - 1));
        
        // Schmitt trigger hysteresis on sample IOs
        tsc.iohcr.write(|w| unsafe {
            w.bits(1 << bit_pos)
        });

        // Set the sampling pin
        tsc.ioscr.write(|w| unsafe { w.bits(1 << bit_pos) });
        
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

        let bit_pos = PIN::OFFSET + (4 * (PIN::GROUP - 1));

        // Set the channel pin
        self.tsc.ioccr.write(|w| unsafe {
            w.bits(1 << bit_pos)
        });

        self.tsc.cr.modify(|_, w| { w.start().set_bit() });
    }

    /// Blocks waiting for a acquisition to complete or for a Max Count Error
    pub fn acquire<PIN>(&self, input: &mut PIN) -> Result<u16, Error>
        where PIN: ChannelPin<TSC>
    {
        // start the acq
        self.start(input);

        let result = loop {
            let isr = self.tsc.isr.read();
            if isr.eoaf().bit_is_set() {
                self.tsc.icr.write(|w| { w.eoaic().set_bit() });
                break Ok(self.read_unchecked())
            } else if isr.mcef().bit_is_set() {
                self.tsc.icr.write(|w| { w.mceic().set_bit() });
                break Err(Error::MaxCountError)
            }
        };

        result
    }

    /// Reads the tsc group 2 count register
    pub fn read<PIN>(&self, _input: &mut PIN) -> Result<u16, Error>
        where PIN: ChannelPin<TSC>
    {
        let bit_pos = PIN::OFFSET + (4 * (PIN::GROUP - 1));
        // Read the current channel config
        let channel = self.tsc.ioccr.read().bits();
        // if they are equal we have the right pin
        if channel == (1 << bit_pos) {
            Ok(self.read_unchecked())
        } else {
            Err(Error::InvalidPin)
        }
    }

    /// Reads the tsc group 2 count register
    /// WARNING, just returns the contents of the register! No validation of the correct pin 
    pub fn read_unchecked(&self) -> u16 {
        self.tsc.iog2cr.read().cnt().bits()
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