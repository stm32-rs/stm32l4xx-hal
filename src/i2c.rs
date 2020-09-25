//! Inter-Integrated Circuit (I2C) bus

use crate::gpio::{Alternate, OpenDrain, Output, AF4};
use crate::hal::blocking::i2c::{Read, Write, WriteRead};
use crate::rcc::{Clocks, APB1R1};
use crate::stm32::{i2c1, I2C1, I2C2};
use crate::time::Hertz;
use cast::u8;
use core::ops::Deref;

const COUNTDOWN_TIMER: u32 = 10_000;

/// I2C error
#[non_exhaustive]
#[derive(Debug)]
pub enum Error {
    /// RXNE error
    Rxne,
    /// TIMEOUT error
    Timeout,
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

/// *COMPARE* with the `busy_wait` macro below. All the methods are taken from
/// the authentic STM32 HAL library, which is confirmed to be working. Not meant
/// to be merged as-is, though.
impl<PINS, I2C> I2c<I2C, PINS>
where
    I2C: Deref<Target = i2c1::RegisterBlock>,
{
    /// I2C_IsAcknowledgeFailed(_, _, _)
    fn check_acknowledge_failed(&self) -> Result<(), Error> {
        if self.i2c.isr.read().nackf().bit_is_set() {
            /* Wait until STOP Flag is reset */
            /* AutoEnd should be initiate after AF */
            let mut clock = (0..COUNTDOWN_TIMER).into_iter();
            while self.i2c.isr.read().stopf().is_no_stop() {
                /* Check for the Timeout */
                clock.next().ok_or(Error::Timeout)?;
            }

            /* Clear NACKF Flag (Not available) */
            /* Clear STOP Flag (Not available) */

            /* If a pending TXIS flag is set */
            /* Write a dummy data in TXDR to clear it */
            if self.i2c.isr.read().txis().is_empty() {
                self.i2c.txdr.modify(|_, w| w.txdata().bits(0x00u8));
            }
            /* Flush TX register if not empty */
            if self.i2c.isr.read().txe().is_not_empty() {
                self.i2c.isr.modify(|_, w| w.txe().clear_bit())
            }

            /* Clear Configuration Register 2 */
            self.i2c.cr2.reset();
        }
        Ok(())
    }
}

/// FIXME: Come up with a nice way, neither the wordy C derivative nor the
/// malfunctioning macro.
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

/// Take a closer look and find that each register needs slightly different treatment resp.
impl<PINS, I2C> I2c<I2C, PINS>
where
    I2C: Deref<Target = i2c1::RegisterBlock>,
{
    /// Handles I2C communication timeout.
    /// I2C_WaitOnFlagUntilTimeout(_, I2C_FLAG_BUSY, SET, _, _)
    fn wait_on_busy_until_timeout(&self) -> Result<(), Error> {
        let mut clock = (0..COUNTDOWN_TIMER).into_iter();
        while self.i2c.isr.read().busy().is_busy() {
            /* Check for the Timeout */
            clock.next().ok_or(Error::Timeout)?;
        }
        Ok(())
    }

    /// Handles I2C communication timeout.
    /// I2C_WaitOnTXISFlagUntilTimeout(_, _, _)
    fn wait_on_txis_until_timeout(&self) -> Result<(), Error> {
        let mut clock = (0..COUNTDOWN_TIMER).into_iter();
        while self.i2c.isr.read().txis().is_empty() {
            /* Check if a NACK is detected */
            self.check_acknowledge_failed()?;
            /* Check for the Timeout */
            clock.next().ok_or(Error::Timeout)?;
        }
        Ok(())
    }

    /// I2C_WaitOnSTOPFlagUntilTimeout
    fn wait_on_stopf_until_timeout(&self) -> Result<(), Error> {
        let mut clock = (0..COUNTDOWN_TIMER).into_iter();
        while self.i2c.isr.read().stopf().is_no_stop() {
            /* Check if a NACK is detected */
            self.check_acknowledge_failed()?;
            /* Check for the Timeout */
            clock.next().ok_or(Error::Timeout)?;
        }
        Ok(())
    }

    /// Handles I2C communication timeout.
    /// I2C_WaitOnRXNEFlagUntilTimeout(_, _, _)
    fn wait_on_rxne_until_timeout(&self) -> Result<(), Error> {
        let mut clock = (0..COUNTDOWN_TIMER).into_iter();
        while self.i2c.isr.read().rxne().is_empty() {
            /* Check if a NACK is detected */
            self.check_acknowledge_failed()?;

            /* Check if a STOPF is detected */
            if self.i2c.isr.read().stopf().is_stop() {
                /* The Reading of data from RXDR will be done in caller function */
                return Ok(());
            }
            /* Check for the Timeout */
            clock.next().ok_or(Error::Timeout)?;
        }
        Ok(())
    }

    /// Basic building block for Master mode transmittion. A payload size over
    /// MAX_NBYTE_SIZE is not supported.
    pub fn master_transmit(&mut self, addr: u8, bytes: &[u8]) -> Result<(), Error> {
        self.wait_on_busy_until_timeout()?;
        /* Send Slave Address and set NBYTES to write */
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
                .set_bit()
        });

        for byte in bytes {
            self.wait_on_txis_until_timeout()?;

            /* Write data to TXDR */
            self.i2c.txdr.write(|w| w.txdata().bits(*byte));
        }

        /* No need to Check TC flag, with AUTOEND mode the stop is automatically generated */
        /* Wait until STOPF flag is set */
        self.wait_on_stopf_until_timeout()?;

        /* Clear STOP Flag (Not avaialble) */
        /* Clear Configuration Register 2 */
        self.i2c.cr2.reset();

        Ok(())
    }

    /// Basic building block for Master mode data receiving.
    pub fn master_receive(&mut self, addr: u8, buffer: &mut [u8]) -> Result<(), Error> {
        self.wait_on_busy_until_timeout()?;

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
            /* Wait until RXNE flag is set */
            self.wait_on_rxne_until_timeout()?;
            *byte = self.i2c.rxdr.read().rxdata().bits();
        }

        /* Wait until STOPF flag is set */
        self.wait_on_stopf_until_timeout()?;

        /* Clear STOP Flag (Not avaialble) */
        /* Clear Configuration Register 2 */
        self.i2c.cr2.reset();

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

        // Configure for "Standard mode" (100 KHz)
        let _ = clocks;
        
        const B_L475E_IOT01A_I2C_TIMING: u32 = 0x00702681u32;
        const TIMING_CLEAR_MASK: u32 = 0xF0FFFFFFu32;
        i2c.timingr.write(|w| {
            unsafe { w.bits(B_L475E_IOT01A_I2C_TIMING & TIMING_CLEAR_MASK) }
        });

        // Check if the value matches.
        assert_eq!(B_L475E_IOT01A_I2C_TIMING & TIMING_CLEAR_MASK, i2c.timingr.read().bits());

        // Enable the peripheral
        i2c.cr1.write(|w| w.pe().set_bit().anfoff().enabled());

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
        self.master_transmit(addr, bytes)
    }
}

impl<PINS, I2C> Read for I2c<I2C, PINS>
where
    I2C: Deref<Target = i2c1::RegisterBlock>,
{
    type Error = Error;

    fn read(&mut self, addr: u8, buffer: &mut [u8]) -> Result<(), Error> {
        self.master_receive(addr, buffer)
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
