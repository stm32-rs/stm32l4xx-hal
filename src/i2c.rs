//! Inter-Integrated Circuit (I2C) bus. Synchronized with the
//! [stm32h7xx-hal](https://github.com/stm32-rs/stm32h7xx-hal) implementation,
//! as of 2021-02-25.

use crate::hal::blocking::i2c::{Read, Write, WriteRead};
use crate::pac::{i2c1, I2C1, I2C2, I2C3, I2C4};
use crate::rcc::{Clocks, APB1R1};
use crate::time::Hertz;
use cast::{u16, u8};
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
pub(self) mod private {
    pub trait Sealed {}
}

/// SCL pin. This trait is sealed and cannot be implemented.
pub trait SclPin<I2C>: private::Sealed {}

/// SDA pin. This trait is sealed and cannot be implemented.
pub trait SdaPin<I2C>: private::Sealed {}

macro_rules! pins {
    ($spi:ident, $af:ident, SCL: [$($scl:ident),*], SDA: [$($sda:ident),*]) => {
        $(
            impl super::private::Sealed for $scl<Alternate<$af, Output<OpenDrain>>> {}
            impl super::SclPin<$spi> for $scl<Alternate<$af, Output<OpenDrain>>> {}
        )*
        $(
            impl super::private::Sealed for $sda<Alternate<$af, Output<OpenDrain>>> {}
            impl super::SdaPin<$spi> for $sda<Alternate<$af, Output<OpenDrain>>> {}
        )*
    }
}

/// I2C peripheral operating in master mode
pub struct I2c<I2C, PINS> {
    i2c: I2C,
    pins: PINS,
}

// TODO review why StartCondition and State structs exists. Last
// update to them was november 2020
/// Start conditions that govern repeated sequential transfer
#[derive(Debug, Clone)]
enum StartCondition {
    /// Non-sequential transfer. Generate both start and stop.
    FirstAndLast,
    /// First transfer in a sequence. Generate start and reload for the next.
    First,
    /// In the middle of sequential transfer. No start/stop generation.
    Middle,
    /// Generate stop to terminate the sequence.
    Last,
}

impl StartCondition {
    /// Configures the I2C sequential transfer
    fn config<'a>(&self, w: &'a mut i2c1::cr2::W) -> &'a mut i2c1::cr2::W {
        use StartCondition::*;
        match self {
            FirstAndLast => w.start().start().autoend().automatic(),
            First => w.start().start().reload().not_completed(),
            Middle => w.start().no_start().reload().not_completed(),
            Last => w.start().no_start().autoend().automatic(),
        }
    }
}

/// Yields start/stop direction from chunks of a payload
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

macro_rules! hal {
    ($i2c_type: ident, $enr: ident, $rstr: ident, $i2cX: ident, $i2cXen: ident, $i2cXrst: ident) => {
        impl<SCL, SDA> I2c<$i2c_type, (SCL, SDA)> {
            pub fn $i2cX<F>(
                i2c: $i2c_type,
                pins: (SCL, SDA),
                freq: F,
                clocks: Clocks,
                apb1: &mut APB1R1,
            ) -> Self
            where
                F: Into<Hertz>,
                SCL: SclPin<$i2c_type>,
                SDA: SdaPin<$i2c_type>,
            {
                apb1.$enr().modify(|_, w| w.$i2cXen().set_bit());
                apb1.$rstr().modify(|_, w| w.$i2cXrst().set_bit());
                apb1.$rstr().modify(|_, w| w.$i2cXrst().clear_bit());
                Self::new(i2c, pins, freq, clocks)
            }
        }
    };
}

hal!(I2C1, enr, rstr, i2c1, i2c1en, i2c1rst);
hal!(I2C2, enr, rstr, i2c2, i2c2en, i2c2rst);

#[cfg(any(feature = "stm32l4x3", feature = "stm32l4x5", feature = "stm32l4x6"))]
hal!(I2C3, enr, rstr, i2c3, i2c3en, i2c3rst);

// Warning: available on stm32l4x5 according to reference manual,
// but stm32l475rc (only one available) does not have this peripheral listed,
#[cfg(any(feature = "stm32l4x5", feature = "stm32l4x6"))]
hal!(I2C4, enr2, rstr2, i2c4, i2c4en, i2c4rst);

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

        macro_rules! u8_or_panic {
            ($value: expr, $message: literal) => {
                match u8($value) {
                    Ok(value) => value,
                    Err(_) => panic!($message),
                }
            };
        }

        let presc = u8_or_panic!(presc, "I2C pres");
        assert!(presc < 16);

        let scldel = u8_or_panic!(scldel, "I2C scldel");
        assert!(scldel < 16);

        let sdadel = u8_or_panic!(sdadel, "I2C sdadel");
        assert!(sdadel < 16);

        let sclh = u8_or_panic!(sclh, "I2C sclh");
        let scll = u8_or_panic!(scll, "I2C scll");

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

/// Sequence to flush the TXDR register. This resets the TXIS and TXE
// flags
macro_rules! flush_txdr {
    ($i2c:expr) => {
        // If a pending TXIS flag is set, write dummy data to TXDR
        if $i2c.isr.read().txis().bit_is_set() {
            $i2c.txdr.write(|w| w.txdata().bits(0));
        }

        // If TXDR is not flagged as empty, write 1 to flush it
        if $i2c.isr.read().txe().is_not_empty() {
            $i2c.isr.write(|w| w.txe().set_bit());
        }
    };
}

macro_rules! busy_wait {
    ($i2c:expr, $flag:ident, $variant:ident) => {
        loop {
            let isr = $i2c.isr.read();

            if isr.$flag().$variant() {
                break;
            } else if isr.berr().is_error() {
                $i2c.icr.write(|w| w.berrcf().set_bit());
                return Err(Error::Bus);
            } else if isr.arlo().is_lost() {
                $i2c.icr.write(|w| w.arlocf().set_bit());
                return Err(Error::Arbitration);
            } else if isr.nackf().bit_is_set() {
                $i2c.icr.write(|w| w.stopcf().set_bit().nackcf().set_bit());
                flush_txdr!($i2c);
                return Err(Error::Nack);
            } else {
                // try again
            }
        }
    };
}

impl<PINS, I2C> Write for I2c<I2C, PINS>
where
    I2C: Deref<Target = i2c1::RegisterBlock>,
{
    type Error = Error;

    fn write(&mut self, addr: u8, bytes: &[u8]) -> Result<(), Error> {
        // TODO support transfers of more than 255 bytes
        assert!(bytes.len() < 256 && bytes.len() > 0);

        // Wait for any previous address sequence to end
        // automatically. This could be up to 50% of a bus
        // cycle (ie. up to 0.5/freq)
        while self.i2c.cr2.read().start().bit_is_set() {}

        // Set START and prepare to send `bytes`. The
        // START bit can be set even if the bus is BUSY or
        // I2C is in slave mode.
        self.i2c.cr2.write(|w| {
            w.start()
                .set_bit()
                .sadd()
                .bits(u16(addr << 1 | 0))
                .add10()
                .clear_bit()
                .rd_wrn()
                .write()
                .nbytes()
                .bits(bytes.len() as u8)
                .autoend()
                .software()
        });

        for byte in bytes {
            // Wait until we are allowed to send data
            // (START has been ACKed or last byte when
            // through)
            busy_wait!(self.i2c, txis, is_empty);

            // Put byte on the wire
            self.i2c.txdr.write(|w| w.txdata().bits(*byte));
        }

        // Wait until the write finishes
        busy_wait!(self.i2c, tc, is_complete);

        // Stop
        self.i2c.cr2.write(|w| w.stop().set_bit());

        Ok(())
        // Tx::new(&self.i2c)?.write(addr, bytes)
    }
}

impl<PINS, I2C> Read for I2c<I2C, PINS>
where
    I2C: Deref<Target = i2c1::RegisterBlock>,
{
    type Error = Error;

    fn read(&mut self, addr: u8, buffer: &mut [u8]) -> Result<(), Error> {
        // TODO support transfers of more than 255 bytes
        assert!(buffer.len() < 256 && buffer.len() > 0);

        // Wait for any previous address sequence to end
        // automatically. This could be up to 50% of a bus
        // cycle (ie. up to 0.5/freq)
        while self.i2c.cr2.read().start().bit_is_set() {}

        // Set START and prepare to receive bytes into
        // `buffer`. The START bit can be set even if the bus
        // is BUSY or I2C is in slave mode.
        self.i2c.cr2.write(|w| {
            w.sadd()
                .bits((addr << 1 | 0) as u16)
                .rd_wrn()
                .read()
                .nbytes()
                .bits(buffer.len() as u8)
                .start()
                .set_bit()
                .autoend()
                .automatic()
        });

        for byte in buffer {
            // Wait until we have received something
            busy_wait!(self.i2c, rxne, is_not_empty);

            *byte = self.i2c.rxdr.read().rxdata().bits();
        }

        // automatic STOP

        Ok(())
        // Rx::new(&self.i2c)?.read(addr, buffer)
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

        // Wait for any previous address sequence to end
        // automatically. This could be up to 50% of a bus
        // cycle (ie. up to 0.5/freq)
        while self.i2c.cr2.read().start().bit_is_set() {}

        // Set START and prepare to send `bytes`. The
        // START bit can be set even if the bus is BUSY or
        // I2C is in slave mode.
        self.i2c.cr2.write(|w| {
            w.start()
                .set_bit()
                .sadd()
                .bits(u16(addr << 1 | 0))
                .add10()
                .clear_bit()
                .rd_wrn()
                .write()
                .nbytes()
                .bits(bytes.len() as u8)
                .autoend()
                .software()
        });

        for byte in bytes {
            // Wait until we are allowed to send data
            // (START has been ACKed or last byte went through)
            busy_wait!(self.i2c, txis, is_empty);

            // Put byte on the wire
            self.i2c.txdr.write(|w| w.txdata().bits(*byte));
        }

        // Wait until the write finishes before beginning to read.
        busy_wait!(self.i2c, tc, is_complete);

        // reSTART and prepare to receive bytes into `buffer`
        self.i2c.cr2.write(|w| {
            w.sadd()
                .bits(u16(addr << 1 | 1))
                .add10()
                .clear_bit()
                .rd_wrn()
                .read()
                .nbytes()
                .bits(buffer.len() as u8)
                .start()
                .set_bit()
                .autoend()
                .automatic()
        });

        for byte in buffer {
            // Wait until we have received something
            busy_wait!(self.i2c, rxne, is_not_empty);

            *byte = self.i2c.rxdr.read().rxdata().bits();
        }

        Ok(())
    }
}

/// **Warning**: A9 and A10 in AF4 don't have this function on stm32l471qe,
/// but do on stm32l431cb and stm32l451cc
mod i2c_pins_default {
    use super::{I2C1, I2C2};
    use crate::gpio::gpioa::{PA10, PA9};
    use crate::gpio::gpiob::{PB10, PB11, PB6, PB7};
    use crate::gpio::{Alternate, OpenDrain, Output, AF4};

    pins!(I2C1, AF4,
        SCL: [PA9, PB6],
        SDA: [PA10, PB7]);

    pins!(I2C2, AF4, SCL: [PB10], SDA: [PB11]);
}

#[cfg(any(feature = "stm32l4x1", feature = "stm32l4x2", feature = "stm32l4x6"))]
mod i2c_pins_pb8_pb9 {
    use super::{I2C1, I2C2};
    use crate::gpio::gpiob::{PB13, PB14, PB8, PB9};
    use crate::gpio::{Alternate, OpenDrain, Output, AF4};

    pins!(I2C1, AF4, SCL: [PB8], SDA: [PB9]);
    pins!(I2C2, AF4, SCL: [PB13], SDA: [PB14]);
}

#[cfg(any(
    feature = "stm32l4x1",
    feature = "stm32l4x3",
    feature = "stm32l4x5",
    feature = "stm32l4x6"
))]
mod i2c_pins_pbc0_pc1 {
    use super::I2C3;
    use crate::gpio::gpioc::{PC0, PC1};
    use crate::gpio::{Alternate, OpenDrain, Output, AF4};

    pins!(I2C3, AF4, SCL: [PC0], SDA: [PC1]);
}

/// **Warning**: PA7 and PB4 don't have this function on stm32l486jg and stm32l476je,
/// but do on stm32l496ae and stm32l4a6ag
///
/// **Warning**: PA7 and PB4 don't have this function on stm32l471qe and stm32l475rc,
/// but do on stm32l431cb and stm32l451cc
#[cfg(any(
    feature = "stm32l4x1",
    feature = "stm32l4x2",
    feature = "stm32l4x3",
    feature = "stm32l4x6"
))]
mod i2c_pins_pa7_pb4 {
    use super::I2C3;
    use crate::gpio::gpioa::PA7;
    use crate::gpio::gpiob::PB4;
    use crate::gpio::{Alternate, OpenDrain, Output, AF4};

    // These pins aren't as nicely consistent
    pins!(I2C3, AF4, SCL: [PA7], SDA: [PB4]);
}

/// **Warning**: these pins don't have this function on stm32l486jg and stm32l476je,
/// but do on stm32l496ae and stm32l4a6ag
#[cfg(feature = "stm32l4x6")]
mod i2c_pins_pd12_pd13_pf0_pf1_pf14_pf15 {
    use super::{I2C2, I2C4};
    use crate::gpio::gpiod::{PD12, PD13};
    use crate::gpio::gpiof::{PF0, PF1, PF14, PF15};
    use crate::gpio::{Alternate, OpenDrain, Output, AF4};

    // SCL and SDA are "reversed", correct according to reference manual
    pins!(I2C2, AF4, SCL: [PF1], SDA: [PF0]);

    pins!(I2C4, AF4, SCL: [PD12], SDA: [PD13]);
    pins!(I2C4, AF4, SCL: [PF14], SDA: [PF15]);
}
