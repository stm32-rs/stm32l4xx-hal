//! Serial Peripheral Interface (SPI) bus
//!
//! The PACs and SVDs are not set up granularity enough to handle all peripheral configurations.
//! SPI2 is enabled for stm32l4x2 feature at a HAL level even though some variants do and some
//! don't have it (L432xx and L442xx don't, L452xx does). Users of this MCU variant that
//! don't have it shouldn't attempt to use it. Relevant info is on user-manual level.

use core::ptr;
use core::sync::atomic;
use core::sync::atomic::Ordering;

use crate::dma::{self, dma1, dma2, TransferPayload};
use crate::gpio::{Alternate, Floating, Input, AF5};
use crate::hal::spi::{FullDuplex, Mode, Phase, Polarity};
use crate::rcc::{Clocks, APB1R1, APB2};
use crate::time::Hertz;

use embedded_dma::{StaticReadBuffer, StaticWriteBuffer};

/// SPI error
#[non_exhaustive]
#[derive(Debug)]
pub enum Error {
    /// Overrun occurred
    Overrun,
    /// Mode fault occurred
    ModeFault,
    /// CRC error
    Crc,
}

#[doc(hidden)]
mod private {
    pub trait Sealed {}
}

/// SCK pin. This trait is sealed and cannot be implemented.
pub trait SckPin<SPI>: private::Sealed {}
/// MISO pin. This trait is sealed and cannot be implemented.
pub trait MisoPin<SPI>: private::Sealed {}
/// MOSI pin. This trait is sealed and cannot be implemented.
pub trait MosiPin<SPI>: private::Sealed {}

macro_rules! pins {
    ($spi:ident, $af:ident, SCK: [$($sck:ident),*], MISO: [$($miso:ident),*], MOSI: [$($mosi:ident),*]) => {
        $(
            impl private::Sealed for $sck<Alternate<$af, Input<Floating>>> {}
            impl SckPin<$spi> for $sck<Alternate<$af, Input<Floating>>> {}
        )*
        $(
            impl private::Sealed for $miso<Alternate<$af, Input<Floating>>> {}
            impl MisoPin<$spi> for $miso<Alternate<$af, Input<Floating>>> {}
        )*
        $(
            impl private::Sealed for $mosi<Alternate<$af, Input<Floating>>> {}
            impl MosiPin<$spi> for $mosi<Alternate<$af, Input<Floating>>> {}
        )*
    }
}

/// SPI peripheral operating in full duplex master mode
pub struct Spi<SPI, PINS> {
    spi: SPI,
    pins: PINS,
}

macro_rules! hal {
    ($($SPIX:ident: ($spiX:ident, $spiX_slave:ident, $APBX:ident, $spiXen:ident, $spiXrst:ident, $pclkX:ident),)+) => {
        $(
            impl<SCK, MISO, MOSI> Spi<$SPIX, (SCK, MISO, MOSI)> {
                /// Configures the SPI peripheral to operate in full duplex master mode
                pub fn $spiX<F>(
                    spi: $SPIX,
                    pins: (SCK, MISO, MOSI),
                    mode: Mode,
                    freq: F,
                    clocks: Clocks,
                    apb2: &mut $APBX,
                ) -> Self
                where
                    F: Into<Hertz>,
                    SCK: SckPin<$SPIX>,
                    MISO: MisoPin<$SPIX>,
                    MOSI: MosiPin<$SPIX>,
                {
                    // enable or reset $SPIX
                    apb2.enr().modify(|_, w| w.$spiXen().set_bit());
                    apb2.rstr().modify(|_, w| w.$spiXrst().set_bit());
                    apb2.rstr().modify(|_, w| w.$spiXrst().clear_bit());

                    // FRXTH: RXNE event is generated if the FIFO level is greater than or equal to
                    //        8-bit
                    // DS: 8-bit data size
                    // SSOE: Slave Select output disabled
                    spi.cr2
                        .write(|w| unsafe {
                            w.frxth().set_bit().ds().bits(0b111).ssoe().clear_bit()
                        });

                    let br = Self::compute_baud_rate(clocks.$pclkX(), freq.into());

                    // CPHA: phase
                    // CPOL: polarity
                    // MSTR: master mode
                    // BR: 1 MHz
                    // SPE: SPI disabled
                    // LSBFIRST: MSB first
                    // SSM: enable software slave management (NSS pin free for other uses)
                    // SSI: set nss high = master mode
                    // CRCEN: hardware CRC calculation disabled
                    // BIDIMODE: 2 line unidirectional (full duplex)
                    spi.cr1.write(|w| unsafe {
                        w.cpha()
                            .bit(mode.phase == Phase::CaptureOnSecondTransition)
                            .cpol()
                            .bit(mode.polarity == Polarity::IdleHigh)
                            .mstr()
                            .set_bit()
                            .br()
                            .bits(br)
                            .spe()
                            .set_bit()
                            .lsbfirst()
                            .clear_bit()
                            .ssi()
                            .set_bit()
                            .ssm()
                            .set_bit()
                            .crcen()
                            .clear_bit()
                            .bidimode()
                            .clear_bit()
                    });

                    Spi { spi, pins }
                }

                pub fn $spiX_slave(spi: $SPIX, pins: (SCK, MISO, MOSI), mode: Mode, apb2: &mut $APBX,) -> Self
                where
                    SCK: SckPin<$SPIX>,
                    MISO: MisoPin<$SPIX>,
                    MOSI: MosiPin<$SPIX>,
                {
                    // enable or reset $SPIX
                    apb2.enr().modify(|_, w| w.$spiXen().set_bit());
                    apb2.rstr().modify(|_, w| w.$spiXrst().set_bit());
                    apb2.rstr().modify(|_, w| w.$spiXrst().clear_bit());

                    // CPOL: polarity
                    // CPHA: phase
                    // BIDIMODE: 2 line unidirectional (full duplex)
                    // LSBFIRST: MSB first
                    // CRCEN: hardware CRC calculation disabled
                    // MSTR: master mode
                    // SSM: disable software slave management (NSS pin not free for other uses)
                    // SPE: SPI disabled
                    spi.cr1.write(|w| {
                        w.cpol()
                            .bit(mode.polarity == Polarity::IdleHigh)
                            .cpha()
                            .bit(mode.phase == Phase::CaptureOnSecondTransition)
                            .bidimode()
                            .clear_bit()
                            .lsbfirst()
                            .clear_bit()
                            .crcen()
                            .clear_bit()
                            .ssm()
                            .clear_bit()
                            .mstr()
                            .clear_bit()
                    });

                    // DS: 8-bit data size
                    // FRXTH: RXNE event is generated if the FIFO level is greater than or equal to
                    //        8-bit
                    spi.cr2
                        .write(|w| unsafe { w.ds().bits(0b111).frxth().set_bit() });

                    // SPE: SPI enabled
                    spi.cr1.write(|w| w.spe().set_bit());

                    Spi { spi, pins }
                }

                pub fn clear_overrun(&mut self) {
                    self.spi.dr.read().dr();
                    self.spi.sr.read().ovr();
                }

                /// Change the baud rate of the SPI
                pub fn reclock<F>(&mut self, freq: F, clocks: Clocks)
                    where F: Into<Hertz>
                {
                    self.spi.cr1.modify(|_, w| w.spe().clear_bit());
                    self.spi.cr1.modify(|_, w| {
                        unsafe {w.br().bits(Self::compute_baud_rate(clocks.$pclkX(), freq.into()));}
                        w.spe().set_bit()
                    });
                }

                fn compute_baud_rate(clocks: Hertz, freq: Hertz) -> u8 {
                    match clocks.0 / freq.0 {
                        0 => unreachable!(),
                        1..=2 => 0b000,
                        3..=5 => 0b001,
                        6..=11 => 0b010,
                        12..=23 => 0b011,
                        24..=39 => 0b100,
                        40..=95 => 0b101,
                        96..=191 => 0b110,
                        _ => 0b111,
                    }
                }

                /// Releases the SPI peripheral and associated pins
                pub fn free(self) -> ($SPIX, (SCK, MISO, MOSI)) {
                    (self.spi, self.pins)
                }
            }

            impl<PINS> FullDuplex<u8> for Spi<$SPIX, PINS> {
                type Error = Error;

                fn read(&mut self) -> nb::Result<u8, Error> {
                    let sr = self.spi.sr.read();

                    Err(if sr.ovr().bit_is_set() {
                        nb::Error::Other(Error::Overrun)
                    } else if sr.modf().bit_is_set() {
                        nb::Error::Other(Error::ModeFault)
                    } else if sr.crcerr().bit_is_set() {
                        nb::Error::Other(Error::Crc)
                    } else if sr.rxne().bit_is_set() {
                        // NOTE(read_volatile) read only 1 byte (the svd2rust API only allows
                        // reading a half-word)
                        return Ok(unsafe {
                            ptr::read_volatile(&self.spi.dr as *const _ as *const u8)
                        });
                    } else {
                        nb::Error::WouldBlock
                    })
                }

                fn send(&mut self, byte: u8) -> nb::Result<(), Error> {
                    let sr = self.spi.sr.read();

                    Err(if sr.ovr().bit_is_set() {
                        nb::Error::Other(Error::Overrun)
                    } else if sr.modf().bit_is_set() {
                        nb::Error::Other(Error::ModeFault)
                    } else if sr.crcerr().bit_is_set() {
                        nb::Error::Other(Error::Crc)
                    } else if sr.txe().bit_is_set() {
                        // NOTE(write_volatile) see note above
                        unsafe { ptr::write_volatile(&self.spi.dr as *const _ as *mut u8, byte) }
                        return Ok(());
                    } else {
                        nb::Error::WouldBlock
                    })
                }
            }

            impl<PINS> crate::hal::blocking::spi::transfer::Default<u8> for Spi<$SPIX, PINS> {}

            impl<PINS> crate::hal::blocking::spi::write::Default<u8> for Spi<$SPIX, PINS> {}
        )+
    }
}

#[cfg(any(
    feature = "stm32l4x1",
    feature = "stm32l4x2",
    feature = "stm32l4x3",
    feature = "stm32l4x5",
    feature = "stm32l4x6",
))]
use crate::gpio::gpiod::*;
#[cfg(any(feature = "stm32l4x5", feature = "stm32l4x6"))]
use crate::gpio::gpiog::*;
use crate::gpio::{gpioa::*, gpiob::*, gpioc::*, gpioe::*};

use crate::stm32::SPI1;
#[cfg(any(
    feature = "stm32l4x1",
    feature = "stm32l4x2",
    feature = "stm32l4x3",
    feature = "stm32l4x5",
    feature = "stm32l4x6"
))]
hal! {
    SPI1: (spi1, spi1_slave, APB2, spi1en, spi1rst, pclk2),
}

#[cfg(any(
    feature = "stm32l4x1",
    feature = "stm32l4x2",
    feature = "stm32l4x3",
    feature = "stm32l4x5",
    feature = "stm32l4x6"
))]
pins!(SPI1, AF5,
    SCK: [PA5, PB3, PE13],
    MISO: [PA6, PB4, PE14],
    MOSI: [PA7, PB5, PE15]);

#[cfg(any(feature = "stm32l4x5", feature = "stm32l4x6"))]
pins!(SPI1, AF5, SCK: [PG2], MISO: [PG3], MOSI: [PG4]);

#[cfg(any(
    feature = "stm32l4x1",
    feature = "stm32l4x2",
    feature = "stm32l4x5",
    feature = "stm32l4x6",
))]
use crate::{gpio::AF6, stm32::SPI3};

#[cfg(any(
    feature = "stm32l4x1",
    feature = "stm32l4x2",
    feature = "stm32l4x5",
    feature = "stm32l4x6",
))]
hal! {
    SPI3: (spi3, spi3_slave, APB1R1, spi3en, spi3rst, pclk1),
}

#[cfg(any(
    feature = "stm32l4x1",
    feature = "stm32l4x2",
    feature = "stm32l4x5",
    feature = "stm32l4x6",
))]
pins!(SPI3, AF6,
    SCK: [PB3, PC10],
    MISO: [PB4, PC11],
    MOSI: [PB5, PC12]);

#[cfg(any(feature = "stm32l4x5", feature = "stm32l4x6",))]
pins!(SPI3, AF6, SCK: [PG9], MISO: [PG10], MOSI: [PG11]);

#[cfg(any(
    feature = "stm32l4x1",
    feature = "stm32l4x2",
    feature = "stm32l4x3",
    feature = "stm32l4x5",
    feature = "stm32l4x6",
))]
use crate::stm32::SPI2;

#[cfg(any(
    feature = "stm32l4x1",
    feature = "stm32l4x2",
    feature = "stm32l4x3",
    feature = "stm32l4x5",
    feature = "stm32l4x6",
))]
hal! {
    SPI2: (spi2, spi2_slave, APB1R1, spi2en, spi2rst, pclk1),
}

#[cfg(any(
    feature = "stm32l4x1",
    feature = "stm32l4x2",
    feature = "stm32l4x3",
    feature = "stm32l4x5",
    feature = "stm32l4x6",
))]
pins!(SPI2, AF5,
    SCK: [PB13, PB10, PD1],
    MISO: [PB14, PC2, PD3],
    MOSI: [PB15, PC3, PD4]);

pub struct SpiPayload<SPI, PINS> {
    spi: Spi<SPI, PINS>,
}

pub type SpiRxDma<SPI, PINS, CHANNEL> = dma::RxDma<SpiPayload<SPI, PINS>, CHANNEL>;

pub type SpiTxDma<SPI, PINS, CHANNEL> = dma::TxDma<SpiPayload<SPI, PINS>, CHANNEL>;

pub type SpiRxTxDma<SPI, PINS, RXCH, TXCH> = dma::RxTxDma<SpiPayload<SPI, PINS>, RXCH, TXCH>;

macro_rules! spi_dma {
    ($SPIX:ident, $RX_CH:path, $RX_CHX:ident, $RX_MAPX:ident, $TX_CH:path, $TX_CHX:ident, $TX_MAPX:ident) => {
        impl<PINS> dma::Receive for SpiRxDma<$SPIX, PINS, $RX_CH> {
            type RxChannel = $RX_CH;
            type TransmittedWord = u8;
        }

        impl<PINS> dma::Transmit for SpiTxDma<$SPIX, PINS, $TX_CH> {
            type TxChannel = $TX_CH;
            type ReceivedWord = u8;
        }

        impl<PINS> dma::ReceiveTransmit for SpiRxTxDma<$SPIX, PINS, $RX_CH, $TX_CH> {
            type RxChannel = $RX_CH;
            type TxChannel = $TX_CH;
            type TransferedWord = u8;
        }

        impl<PINS> Spi<$SPIX, PINS> {
            pub fn with_rx_dma(self, channel: $RX_CH) -> SpiRxDma<$SPIX, PINS, $RX_CH> {
                let payload = SpiPayload { spi: self };
                SpiRxDma { payload, channel }
            }

            pub fn with_tx_dma(self, channel: $TX_CH) -> SpiTxDma<$SPIX, PINS, $TX_CH> {
                let payload = SpiPayload { spi: self };
                SpiTxDma { payload, channel }
            }

            pub fn with_rxtx_dma(
                self,
                rx_channel: $RX_CH,
                tx_channel: $TX_CH,
            ) -> SpiRxTxDma<$SPIX, PINS, $RX_CH, $TX_CH> {
                let payload = SpiPayload { spi: self };
                SpiRxTxDma {
                    payload,
                    rx_channel,
                    tx_channel,
                }
            }
        }

        impl<PINS> SpiRxDma<$SPIX, PINS, $RX_CH> {
            pub fn split(mut self) -> (Spi<$SPIX, PINS>, $RX_CH) {
                self.stop();
                (self.payload.spi, self.channel)
            }
        }

        impl<PINS> SpiTxDma<$SPIX, PINS, $TX_CH> {
            pub fn split(mut self) -> (Spi<$SPIX, PINS>, $TX_CH) {
                self.stop();
                (self.payload.spi, self.channel)
            }
        }

        impl<PINS> SpiRxTxDma<$SPIX, PINS, $RX_CH, $TX_CH> {
            pub fn split(mut self) -> (Spi<$SPIX, PINS>, $RX_CH, $TX_CH) {
                self.stop();
                (self.payload.spi, self.rx_channel, self.tx_channel)
            }
        }

        impl<PINS> dma::TransferPayload for SpiRxDma<$SPIX, PINS, $RX_CH> {
            fn start(&mut self) {
                self.payload
                    .spi
                    .spi
                    .cr2
                    .modify(|_, w| w.rxdmaen().set_bit());
                self.channel.start();
            }

            fn stop(&mut self) {
                self.channel.stop();
                self.payload
                    .spi
                    .spi
                    .cr2
                    .modify(|_, w| w.rxdmaen().clear_bit());
            }
        }

        impl<PINS> dma::TransferPayload for SpiTxDma<$SPIX, PINS, $TX_CH> {
            fn start(&mut self) {
                self.payload
                    .spi
                    .spi
                    .cr2
                    .modify(|_, w| w.txdmaen().set_bit());
                self.channel.start();
            }

            fn stop(&mut self) {
                self.channel.stop();
                self.payload
                    .spi
                    .spi
                    .cr2
                    .modify(|_, w| w.txdmaen().clear_bit());
            }
        }

        impl<PINS> dma::TransferPayload for SpiRxTxDma<$SPIX, PINS, $RX_CH, $TX_CH> {
            fn start(&mut self) {
                self.payload
                    .spi
                    .spi
                    .cr2
                    .modify(|_, w| w.rxdmaen().set_bit().txdmaen().set_bit());
                self.rx_channel.start();
                self.tx_channel.start();
            }

            fn stop(&mut self) {
                self.tx_channel.stop();
                self.rx_channel.stop();
                self.payload
                    .spi
                    .spi
                    .cr2
                    .modify(|_, w| w.rxdmaen().clear_bit().txdmaen().clear_bit());
            }
        }

        impl<B, PINS> dma::ReadDma<B, u8> for SpiRxDma<$SPIX, PINS, $RX_CH>
        where
            B: StaticWriteBuffer<Word = u8>,
        {
            fn read(mut self, mut buffer: B) -> dma::Transfer<dma::W, B, Self> {
                // NOTE(unsafe) We own the buffer now and we won't call other `&mut` on it
                // until the end of the transfer.
                let (ptr, len) = unsafe { buffer.static_write_buffer() };
                self.channel.set_peripheral_address(
                    unsafe { &(*$SPIX::ptr()).dr as *const _ as u32 },
                    false,
                );
                self.channel.set_memory_address(ptr as u32, true);
                self.channel.set_transfer_length(len as u16);

                self.channel.cselr().modify(|_, w| w.$RX_CHX().$RX_MAPX());

                atomic::compiler_fence(Ordering::Release);
                self.channel.ccr().modify(|_, w| {
                    w
                        // memory to memory mode disabled
                        .mem2mem()
                        .clear_bit()
                        // medium channel priority level
                        .pl()
                        .medium()
                        // 8-bit memory size
                        .msize()
                        .bits8()
                        // 8-bit peripheral size
                        .psize()
                        .bits8()
                        // circular mode disabled
                        .circ()
                        .clear_bit()
                        // write to memory
                        .dir()
                        .clear_bit()
                });
                atomic::compiler_fence(Ordering::Release);
                self.start();

                dma::Transfer::w(buffer, self)
            }
        }

        impl<B, PINS> dma::WriteDma<B, u8> for SpiTxDma<$SPIX, PINS, $TX_CH>
        where
            B: StaticReadBuffer<Word = u8>,
        {
            fn write(mut self, buffer: B) -> dma::Transfer<dma::R, B, Self> {
                // NOTE(unsafe) We own the buffer now and we won't call other `&mut` on it
                // until the end of the transfer.
                let (ptr, len) = unsafe { buffer.static_read_buffer() };
                self.channel.set_peripheral_address(
                    unsafe { &(*$SPIX::ptr()).dr as *const _ as u32 },
                    false,
                );
                self.channel.set_memory_address(ptr as u32, true);
                self.channel.set_transfer_length(len as u16);

                self.channel.cselr().modify(|_, w| w.$TX_CHX().$TX_MAPX());

                atomic::compiler_fence(Ordering::Release);
                self.channel.ccr().modify(|_, w| {
                    w
                        // memory to memory mode disabled
                        .mem2mem()
                        .clear_bit()
                        // medium channel priority level
                        .pl()
                        .medium()
                        // 8-bit memory size
                        .msize()
                        .bits8()
                        // 8-bit peripheral size
                        .psize()
                        .bits8()
                        // circular mode disabled
                        .circ()
                        .clear_bit()
                        // write to peripheral
                        .dir()
                        .set_bit()
                });
                atomic::compiler_fence(Ordering::Release);
                self.start();

                dma::Transfer::r(buffer, self)
            }
        }

        impl<B, PINS> dma::TransferDma<B, u8> for SpiRxTxDma<$SPIX, PINS, $RX_CH, $TX_CH>
        where
            B: StaticWriteBuffer<Word = u8>,
        {
            fn transfer(mut self, mut buffer: B) -> dma::Transfer<dma::RW, B, Self> {
                // Transfer: we use the same buffer for RX and TX

                // NOTE(unsafe) We own the buffer now and we won't call other `&mut` on it
                // until the end of the transfer.
                let (ptr, len) = unsafe { buffer.static_write_buffer() };

                //
                // Setup RX channel
                //
                self.rx_channel.set_peripheral_address(
                    unsafe { &(*$SPIX::ptr()).dr as *const _ as u32 },
                    false,
                );
                self.rx_channel.set_memory_address(ptr as u32, true);
                self.rx_channel.set_transfer_length(len as u16);

                self.rx_channel
                    .cselr()
                    .modify(|_, w| w.$RX_CHX().$RX_MAPX());

                atomic::compiler_fence(Ordering::Release);
                self.rx_channel.ccr().modify(|_, w| {
                    w
                        // memory to memory mode disabled
                        .mem2mem()
                        .clear_bit()
                        // medium channel priority level
                        .pl()
                        .medium()
                        // 8-bit memory size
                        .msize()
                        .bits8()
                        // 8-bit peripheral size
                        .psize()
                        .bits8()
                        // circular mode disabled
                        .circ()
                        .clear_bit()
                        // write to memory
                        .dir()
                        .clear_bit()
                });

                //
                // Setup TX channel
                //
                self.tx_channel.set_peripheral_address(
                    unsafe { &(*$SPIX::ptr()).dr as *const _ as u32 },
                    false,
                );
                self.tx_channel.set_memory_address(ptr as u32, true);
                self.tx_channel.set_transfer_length(len as u16);

                self.tx_channel
                    .cselr()
                    .modify(|_, w| w.$TX_CHX().$TX_MAPX());

                atomic::compiler_fence(Ordering::Release);
                self.tx_channel.ccr().modify(|_, w| {
                    w
                        // memory to memory mode disabled
                        .mem2mem()
                        .clear_bit()
                        // medium channel priority level
                        .pl()
                        .medium()
                        // 8-bit memory size
                        .msize()
                        .bits8()
                        // 8-bit peripheral size
                        .psize()
                        .bits8()
                        // circular mode disabled
                        .circ()
                        .clear_bit()
                        // write to peripheral
                        .dir()
                        .set_bit()
                });

                //
                // Fences and start
                //
                atomic::compiler_fence(Ordering::Release);
                self.start();

                dma::Transfer::rw(buffer, self)
            }
        }
    };
}

spi_dma!(SPI1, dma1::C2, c2s, map1, dma1::C3, c3s, map1);
#[cfg(any(
    feature = "stm32l4x1",
    feature = "stm32l4x3",
    feature = "stm32l4x5",
    feature = "stm32l4x6",
))]
spi_dma!(SPI2, dma1::C4, c4s, map1, dma1::C5, c5s, map1);
// spi_dma!(SPI1, dma2::C3, c3s, map4, dma2::C4, c4s, map4);
#[cfg(any(
    feature = "stm32l4x1",
    feature = "stm32l4x2",
    feature = "stm32l4x5",
    feature = "stm32l4x6",
))]
spi_dma!(SPI3, dma2::C1, c1s, map3, dma2::C2, c2s, map3);
