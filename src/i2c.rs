//! Inter-Integrated Circuit (I2C) bus

use crate::gpio::{Alternate, OpenDrain, Output, AF4};
use crate::hal::blocking::i2c::{Read, Write, WriteRead};
use crate::rcc::{Clocks, APB1R1};
use crate::stm32::{i2c1, I2C1, I2C2};
use crate::time::Hertz;
use cast::u8;
use core::ops::Deref;

const MAX_NBYTE_SIZE: usize = 255;

/// I2C error
#[non_exhaustive]
#[derive(Debug)]
pub enum Error {
    /// Bus error
    Bus,
    /// Arbitration loss
    Arbitration,
    /// NACK
    Nack,
    // Overrun, // slave mode only
    // Pec, // SMBUS mode only
    // Timeout, // SMBUS mode only
    // Alert, // SMBUS mode only
}

#[doc(hidden)]
mod private {
    pub trait Sealed {}
}

/// SCL pin. This trait is sealed and cannot be implemented.
pub trait SclPin<I2C>: private::Sealed {}

/// SDA pin. This trait is sealed and cannot be implemented.
pub trait SdaPin<I2C>: private::Sealed {}

macro_rules! pins {
    ($spi:ident, $af:ident, SCL: [$($scl:ident),*], SDA: [$($sda:ident),*]) => {
        $(
            impl private::Sealed for $scl<Alternate<$af, Output<OpenDrain>>> {}
            impl SclPin<$spi> for $scl<Alternate<$af, Output<OpenDrain>>> {}
        )*
        $(
            impl private::Sealed for $sda<Alternate<$af, Output<OpenDrain>>> {}
            impl SdaPin<$spi> for $sda<Alternate<$af, Output<OpenDrain>>> {}
        )*
    }
}

/// I2C peripheral operating in master mode
pub struct I2c<I2C, PINS> {
    i2c: I2C,
    pins: PINS,
}

macro_rules! busy_wait {
    ($i2c:expr, $flag:ident) => {
        loop {
            let isr = $i2c.isr.read();

            if isr.berr().bit_is_set() {
                return Err(Error::Bus);
            } else if isr.arlo().bit_is_set() {
                return Err(Error::Arbitration);
            } else if isr.nackf().bit_is_set() {
                return Err(Error::Nack);
            } else if isr.$flag().bit_is_set() {
                break;
            } else {
                // try again
            }
        }
    };
}

#[derive(Debug, Clone)]
enum StartCondition {
    FirstAndLast,
    First,
    Middle,
    Last,
}

impl StartCondition {
    fn config<'a>(&self, w: &'a mut i2c1::cr2::W) -> &'a mut i2c1::cr2::W {
        use StartCondition::*;
        match self {
            FirstAndLast => w.start().start().autoend().automatic(),
            First => w.start().start().reload().not_competed(),
            Middle => w
                .start()
                .no_start()
                .stop()
                .no_stop()
                .reload()
                .not_competed(),
            Last => w.start().no_start().stop().no_stop().autoend().automatic(),
        }
    }
}

#[derive(Debug, Default, Clone)]
struct State {
    total_length: usize,
    current_length: Option<usize>,
}

impl State {
    fn new(total_length: usize) -> Self {
        Self {
            total_length,
            current_length: None,
        }
    }

    fn start_condition(&mut self, chunk_len: usize) -> StartCondition {
        use StartCondition::*;
        let total_len = self.total_length;
        match self.current_length.as_mut() {
            None if chunk_len == total_len => FirstAndLast,
            None => {
                self.current_length.replace(chunk_len);
                First
            }
            Some(len) if *len + chunk_len < total_len => {
                *len += chunk_len;
                Middle
            }
            Some(_) => Last,
        }
    }
}

/// I2C sender for master mode
struct Tx<'a> {
    cr2: &'a i2c1::CR2,
    isr: &'a i2c1::ISR,
    icr: &'a i2c1::ICR,
    txdr: &'a i2c1::TXDR,
    aborted: bool,
}

impl<'a> Tx<'a> {
    // Creation succeeds only if busy wait is happy.
    fn new(i2c: &'a i2c1::RegisterBlock) -> Result<Self, Error> {
        // Instead of this potentially infinte loop, ideally set up a timer and raise a timeout error.
        while i2c.isr.read().busy().is_busy() {}

        Ok(Self {
            cr2: &i2c.cr2,
            isr: &i2c.isr,
            icr: &i2c.icr,
            txdr: &i2c.txdr,
            aborted: true,
        })
    }

    fn check_acknowledge_failed(&self) -> Result<(), Error> {
        if self.isr.read().nackf().bit_is_set() {
            // Wait until STOP Flag is reset
            // AutoEnd should be initiate after AF
            while self.isr.read().stopf().is_no_stop() {}
            return Err(Error::Nack);
        }
        Ok(())
    }

    fn send_byte(&mut self, byte: u8) -> Result<(), Error> {
        while self.isr.read().txis().is_not_empty() {
            self.check_acknowledge_failed()?;
        }
        // Write data to TXDR
        self.txdr.write(|w| w.txdata().bits(byte));
        Ok(())
    }
}

/// Clean up register state.
impl<'a> Drop for Tx<'a> {
    fn drop(&mut self) {
        if self.aborted {
            // **ABORTED** PostProcessing from I2C_IsAcknowledgeFailed
            // Clear NACKF Flag
            self.icr.write(|w| w.nackcf().clear());
            // Flush TX register
            self.txdr.reset();
        }
        // **Graceful shutdown**
        // Clear STOP Flag
        self.icr.write(|w| w.stopcf().clear());
        // Clear Configuration Register 2
        self.cr2.reset();
    }
}

impl<'a> Write for Tx<'a> {
    type Error = Error;

    fn write(&mut self, addr: u8, bytes: &[u8]) -> Result<(), Error> {
        while self.isr.read().busy().is_busy() {}
        let total_len = bytes.len();
        bytes
            .chunks(MAX_NBYTE_SIZE)
            .scan(State::new(total_len), |st, chunk| {
                let sc = st.start_condition(chunk.len());
                (sc, chunk).into()
            })
            .try_for_each(|(sc, chunk)| {
                use StartCondition::*;
                // Wait until TCR flag is set
                if let Middle | Last = sc {
                    while self.isr.read().tcr().is_not_complete() {}
                }

                self.cr2.write_with_zero(|w| {
                    sc.config(w)
                        .sadd()
                        .bits((addr as u16) << 1)
                        .rd_wrn()
                        .write()
                        .nbytes()
                        .bits(chunk.len() as u8)
                });

                chunk.iter().try_for_each(|byte| self.send_byte(*byte))
            })?;

        // Let the caller drop self.
        self.aborted = false;
        Ok(())
    }
}

impl<SCL, SDA> I2c<I2C1, (SCL, SDA)> {
    pub fn i2c1<F>(i2c: I2C1, pins: (SCL, SDA), freq: F, clocks: Clocks, apb1: &mut APB1R1) -> Self
    where
        F: Into<Hertz>,
        SCL: SclPin<I2C1>,
        SDA: SdaPin<I2C1>,
    {
        apb1.enr().modify(|_, w| w.i2c1en().set_bit());
        apb1.rstr().modify(|_, w| w.i2c1rst().set_bit());
        apb1.rstr().modify(|_, w| w.i2c1rst().clear_bit());
        Self::new(i2c, pins, freq, clocks)
    }
}

impl<SCL, SDA> I2c<I2C2, (SCL, SDA)> {
    pub fn i2c2<F>(i2c: I2C2, pins: (SCL, SDA), freq: F, clocks: Clocks, apb1: &mut APB1R1) -> Self
    where
        F: Into<Hertz>,
        SCL: SclPin<I2C2>,
        SDA: SdaPin<I2C2>,
    {
        apb1.enr().modify(|_, w| w.i2c2en().set_bit());
        apb1.rstr().modify(|_, w| w.i2c2rst().set_bit());
        apb1.rstr().modify(|_, w| w.i2c2rst().clear_bit());
        Self::new(i2c, pins, freq, clocks)
    }
}

impl<SCL, SDA, I2C> I2c<I2C, (SCL, SDA)>
where
    I2C: Deref<Target = i2c1::RegisterBlock>,
{
    /// Configures the I2C peripheral to work in master mode
    fn new<F>(i2c: I2C, pins: (SCL, SDA), freq: F, clocks: Clocks) -> Self
    where
        F: Into<Hertz>,
        SCL: SclPin<I2C>,
        SDA: SdaPin<I2C>,
    {
        let freq = freq.into().0;
        assert!(freq <= 1_000_000);
        // Make sure the I2C unit is disabled so we can configure it
        i2c.cr1.modify(|_, w| w.pe().clear_bit());

        // TODO review compliance with the timing requirements of I2C
        // t_I2CCLK = 1 / PCLK1
        // t_PRESC  = (PRESC + 1) * t_I2CCLK
        // t_SCLL   = (SCLL + 1) * t_PRESC
        // t_SCLH   = (SCLH + 1) * t_PRESC
        //
        // t_SYNC1 + t_SYNC2 > 4 * t_I2CCLK
        // t_SCL ~= t_SYNC1 + t_SYNC2 + t_SCLL + t_SCLH
        let i2cclk = clocks.pclk1().0;
        let ratio = i2cclk / freq - 4;
        let (presc, scll, sclh, sdadel, scldel) = if freq >= 100_000 {
            // fast-mode or fast-mode plus
            // here we pick SCLL + 1 = 2 * (SCLH + 1)
            let presc = ratio / 387;

            let sclh = ((ratio / (presc + 1)) - 3) / 3;
            let scll = 2 * (sclh + 1) - 1;

            let (sdadel, scldel) = if freq > 400_000 {
                // fast-mode plus
                let sdadel = 0;
                let scldel = i2cclk / 4_000_000 / (presc + 1) - 1;

                (sdadel, scldel)
            } else {
                // fast-mode
                let sdadel = i2cclk / 8_000_000 / (presc + 1);
                let scldel = i2cclk / 2_000_000 / (presc + 1) - 1;

                (sdadel, scldel)
            };

            (presc, scll, sclh, sdadel, scldel)
        } else {
            // standard-mode
            // here we pick SCLL = SCLH
            let presc = ratio / 514;

            let sclh = ((ratio / (presc + 1)) - 2) / 2;
            let scll = sclh;

            let sdadel = i2cclk / 2_000_000 / (presc + 1);
            let scldel = i2cclk / 800_000 / (presc + 1) - 1;

            (presc, scll, sclh, sdadel, scldel)
        };

        let presc = u8(presc).unwrap();
        assert!(presc < 16);
        let scldel = u8(scldel).unwrap();
        assert!(scldel < 16);
        let sdadel = u8(sdadel).unwrap();
        assert!(sdadel < 16);
        let sclh = u8(sclh).unwrap();
        let scll = u8(scll).unwrap();

        // Configure for "fast mode" (400 KHz)
        i2c.timingr.write(|w| {
            w.presc()
                .bits(presc)
                .scll()
                .bits(scll)
                .sclh()
                .bits(sclh)
                .sdadel()
                .bits(sdadel)
                .scldel()
                .bits(scldel)
        });

        // Enable the peripheral
        i2c.cr1.write(|w| w.pe().set_bit());

        I2c { i2c, pins }
    }

    /// Releases the I2C peripheral and associated pins
    pub fn free(self) -> (I2C, (SCL, SDA)) {
        (self.i2c, self.pins)
    }
}

impl<PINS, I2C> Write for I2c<I2C, PINS>
where
    I2C: Deref<Target = i2c1::RegisterBlock>,
{
    type Error = Error;

    fn write(&mut self, addr: u8, bytes: &[u8]) -> Result<(), Error> {
        Tx::new(&self.i2c)?.write(addr, bytes)
    }
}

impl<PINS, I2C> Read for I2c<I2C, PINS>
where
    I2C: Deref<Target = i2c1::RegisterBlock>,
{
    type Error = Error;

    fn read(&mut self, addr: u8, buffer: &mut [u8]) -> Result<(), Error> {
        self.i2c.cr2.write(|w| {
            w.sadd()
                .bits((addr as u16) << 1)
                .rd_wrn()
                .set_bit()
                .nbytes()
                .bits(buffer.len() as u8)
                .start()
                .set_bit()
                .autoend()
                .set_bit()
        });

        for byte in buffer {
            // Wait until we have received something
            busy_wait!(self.i2c, rxne);

            *byte = self.i2c.rxdr.read().rxdata().bits();
        }

        Ok(())
    }
}

impl<PINS, I2C> WriteRead for I2c<I2C, PINS>
where
    I2C: Deref<Target = i2c1::RegisterBlock>,
{
    type Error = Error;

    fn write_read(&mut self, addr: u8, bytes: &[u8], buffer: &mut [u8]) -> Result<(), Error> {
        // TODO support transfers of more than 255 bytes
        assert!(bytes.len() < 256 && bytes.len() > 0);
        assert!(buffer.len() < 256 && buffer.len() > 0);

        // TODO do we have to explicitly wait here if the bus is busy (e.g. another
        // master is communicating)?

        // START and prepare to send `bytes`
        self.i2c.cr2.write(|w| {
            w.sadd()
                .bits((addr as u16) << 1)
                .rd_wrn()
                .clear_bit()
                .nbytes()
                .bits(bytes.len() as u8)
                .start()
                .set_bit()
                .autoend()
                .clear_bit()
        });

        for byte in bytes {
            // Wait until we are allowed to send data (START has been ACKed or last byte
            // when through)
            busy_wait!(self.i2c, txis);

            // put byte on the wire
            self.i2c.txdr.write(|w| w.txdata().bits(*byte));
        }

        // Wait until the last transmission is finished
        busy_wait!(self.i2c, tc);

        // reSTART and prepare to receive bytes into `buffer`
        self.i2c.cr2.write(|w| {
            w.sadd()
                .bits((addr as u16) << 1)
                .rd_wrn()
                .set_bit()
                .nbytes()
                .bits(buffer.len() as u8)
                .start()
                .set_bit()
                .autoend()
                .set_bit()
        });

        for byte in buffer {
            // Wait until we have received something
            busy_wait!(self.i2c, rxne);

            *byte = self.i2c.rxdr.read().rxdata().bits();
        }

        // automatic STOP - due to autoend

        Ok(())
    }
}

use crate::gpio::gpioa::{PA10, PA9};
use crate::gpio::gpiob::{PB10, PB11, PB6, PB7};

pins!(I2C1, AF4,
    SCL: [PA9, PB6],
    SDA: [PA10, PB7]);

pins!(I2C2, AF4, SCL: [PB10], SDA: [PB11]);

#[cfg(any(feature = "stm32l4x1", feature = "stm32l4x6"))]
use crate::gpio::gpiob::{PB13, PB14, PB8, PB9};

#[cfg(any(feature = "stm32l4x1", feature = "stm32l4x6"))]
pins!(I2C1, AF4, SCL: [PB8], SDA: [PB9]);

#[cfg(any(feature = "stm32l4x1", feature = "stm32l4x6"))]
pins!(I2C2, AF4, SCL: [PB13], SDA: [PB14]);
