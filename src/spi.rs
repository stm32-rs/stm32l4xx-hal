//! Serial Peripheral Interface (SPI) bus
//!
//! The PACs and SVDs are not set up granularity enough to handle all peripheral configurations.
//! SPI2 is enabled for stm32l4x2 feature at a HAL level even though some variants do and some
//! don't have it (L432xx and L442xx don't, L452xx does). Users of this MCU variant that
//! don't have it shouldn't attempt to use it. Relevant info is on user-manual level.

use core::ptr;
use core::sync::atomic;
use core::sync::atomic::Ordering;

#[cfg(not(any(feature = "stm32l433", feature = "stm32l443",)))]
use crate::dma::dma2;
use crate::dma::{self, dma1, TransferPayload};
use crate::dmamux::{DmaInput, DmaMux};
use crate::gpio::{Alternate, PushPull};
use crate::hal::spi::{FullDuplex, Mode, Phase, Polarity};
use crate::rcc::{Clocks, Enable, RccBus, Reset};
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
    ($spi:ident, $af:literal, SCK: [$($sck:ident),*], MISO: [$($miso:ident),*], MOSI: [$($mosi:ident),*]) => {
        $(
            impl private::Sealed for $sck<Alternate<PushPull, $af>> {}
            impl SckPin<$spi> for $sck<Alternate<PushPull, $af>> {}
        )*
        $(
            impl private::Sealed for $miso<Alternate<PushPull, $af>> {}
            impl MisoPin<$spi> for $miso<Alternate<PushPull, $af>> {}
        )*
        $(
            impl private::Sealed for $mosi<Alternate<PushPull, $af>> {}
            impl MosiPin<$spi> for $mosi<Alternate<PushPull, $af>> {}
        )*
    }
}

/// SPI peripheral operating in full duplex master mode
pub struct Spi<SPI, PINS> {
    spi: SPI,
    pins: PINS,
}

macro_rules! hal {
    ($($SPIX:ident: ($spiX:ident, $spiX_slave:ident, $pclkX:ident),)+) => {
        $(
            impl<PINS> Spi<$SPIX, PINS> {
                /// Enable the SPI peripheral.
                #[allow(unused)] // Only used for DMA.
                #[inline]
                fn enable(&mut self) {
                    self.spi.cr1.modify(|_, w| w.spe().set_bit());
                }

                /// Disable the SPI peripheral.
                #[inline]
                fn disable(&mut self) {
                    self.spi.cr1.modify(|_, w| w.spe().clear_bit());
                }
            }

            impl<SCK, MISO, MOSI> Spi<$SPIX, (SCK, MISO, MOSI)> {
                /// Configures the SPI peripheral to operate in full duplex master mode
                #[allow(unused_unsafe)]  // Necessary for stm32l4r9
                pub fn $spiX(
                    spi: $SPIX,
                    pins: (SCK, MISO, MOSI),
                    mode: Mode,
                    freq: Hertz,
                    clocks: Clocks,
                    apb2: &mut <$SPIX as RccBus>::Bus,
                ) -> Self
                where
                    SCK: SckPin<$SPIX>,
                    MISO: MisoPin<$SPIX>,
                    MOSI: MosiPin<$SPIX>,
                {
                    // enable or reset $SPIX
                    <$SPIX>::enable(apb2);
                    <$SPIX>::reset(apb2);

                    // FRXTH: RXNE event is generated if the FIFO level is greater than or equal to
                    //        8-bit
                    // DS: 8-bit data size
                    // SSOE: Slave Select output disabled
                    spi.cr2
                        .write(|w| unsafe {
                            w.frxth().set_bit().ds().bits(0b111).ssoe().clear_bit()
                        });

                    let br = Self::compute_baud_rate(clocks.$pclkX(), freq);

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

                pub fn $spiX_slave(spi: $SPIX, pins: (SCK, MISO, MOSI), mode: Mode, apb2: &mut <$SPIX as RccBus>::Bus) -> Self
                where
                    SCK: SckPin<$SPIX>,
                    MISO: MisoPin<$SPIX>,
                    MOSI: MosiPin<$SPIX>,
                {
                    // enable or reset $SPIX
                    <$SPIX>::enable(apb2);
                    <$SPIX>::reset(apb2);

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
                #[allow(unused_unsafe)]  // Necessary for stm32l4r9
                pub fn reclock(&mut self, freq: Hertz, clocks: Clocks) {
                    self.disable();
                    self.spi.cr1.modify(|_, w| unsafe {
                        w.br().bits(Self::compute_baud_rate(clocks.$pclkX(), freq));
                        w.spe().set_bit()
                    });
                }

                fn compute_baud_rate(clocks: Hertz, freq: Hertz) -> u8 {
                    match clocks / freq {
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

use crate::gpio::gpiod::*;
#[cfg(any(
    // feature = "stm32l471",  // missing PAC support for Port G
    feature = "stm32l475",
    feature = "stm32l476",
    feature = "stm32l485",
    feature = "stm32l486",
    feature = "stm32l496",
    feature = "stm32l4a6",
    // feature = "stm32l4p5",
    // feature = "stm32l4q5",
    feature = "stm32l4r5",
    feature = "stm32l4s5",
    // feature = "stm32l4r7",
    // feature = "stm32l4s7",
    feature = "stm32l4r9",
    feature = "stm32l4s9",
))]
use crate::gpio::gpiog::*;
use crate::gpio::{gpioa::*, gpiob::*, gpioc::*, gpioe::*};

use crate::stm32::SPI1;
hal! {
    SPI1: (spi1, spi1_slave, pclk2),
}

pins!(SPI1, 5,
    SCK: [PA5, PB3, PE13],
    MISO: [PA6, PB4, PE14],
    MOSI: [PA7, PB5, PE15]);

#[cfg(any(
    // feature = "stm32l471", // missing PAC support for Port G
    feature = "stm32l475",
    feature = "stm32l476",
    feature = "stm32l485",
    feature = "stm32l486",
    feature = "stm32l496",
    feature = "stm32l4a6",
    // feature = "stm32l4p5",
    // feature = "stm32l4q5",
    feature = "stm32l4r5",
    feature = "stm32l4s5",
    // feature = "stm32l4r7",
    // feature = "stm32l4s7",
    feature = "stm32l4r9",
    feature = "stm32l4s9",
))]
pins!(SPI1, 5, SCK: [PG2], MISO: [PG3], MOSI: [PG4]);

#[cfg(not(any(feature = "stm32l433", feature = "stm32l443",)))]
use crate::stm32::SPI3;

#[cfg(not(any(feature = "stm32l433", feature = "stm32l443",)))]
hal! {
    SPI3: (spi3, spi3_slave, pclk1),
}

#[cfg(not(any(feature = "stm32l433", feature = "stm32l443",)))]
pins!(SPI3, 6,
    SCK: [PB3, PC10],
    MISO: [PB4, PC11],
    MOSI: [PB5, PC12]);

#[cfg(any(
    // feature = "stm32l471", // missing PAC support for Port G
    feature = "stm32l475",
    feature = "stm32l476",
    feature = "stm32l485",
    feature = "stm32l486",
    feature = "stm32l496",
    feature = "stm32l4a6",
    // feature = "stm32l4p5",
    // feature = "stm32l4q5",
    feature = "stm32l4r5",
    feature = "stm32l4s5",
    // feature = "stm32l4r7",
    // feature = "stm32l4s7",
    feature = "stm32l4r9",
    feature = "stm32l4s9",
))]
pins!(SPI3, 6, SCK: [PG9], MISO: [PG10], MOSI: [PG11]);

use crate::stm32::SPI2;

hal! {
    SPI2: (spi2, spi2_slave, pclk1),
}

pins!(SPI2, 5,
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
    ($SPIX:ident, $RX_CH:path, $RX_CHSEL:path, $TX_CH:path, $TX_CHSEL:path) => {
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
            pub fn with_rx_dma(self, mut channel: $RX_CH) -> SpiRxDma<$SPIX, PINS, $RX_CH> {
                let payload = SpiPayload { spi: self };

                // Perform one-time setup actions to keep the work minimal when using the driver.

                channel.set_peripheral_address(
                    unsafe { &(*$SPIX::ptr()).dr as *const _ as u32 },
                    false,
                );
                channel.set_request_line($RX_CHSEL).unwrap();
                channel.ccr().modify(|_, w| {
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

                SpiRxDma { payload, channel }
            }

            pub fn with_tx_dma(self, mut channel: $TX_CH) -> SpiTxDma<$SPIX, PINS, $TX_CH> {
                let payload = SpiPayload { spi: self };

                // Perform one-time setup actions to keep the work minimal when using the driver.

                channel.set_peripheral_address(
                    unsafe { &(*$SPIX::ptr()).dr as *const _ as u32 },
                    false,
                );
                channel.set_request_line($TX_CHSEL).unwrap();
                channel.ccr().modify(|_, w| {
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

                SpiTxDma { payload, channel }
            }

            pub fn with_rxtx_dma(
                self,
                mut rx_channel: $RX_CH,
                mut tx_channel: $TX_CH,
            ) -> SpiRxTxDma<$SPIX, PINS, $RX_CH, $TX_CH> {
                let payload = SpiPayload { spi: self };

                // Perform one-time setup actions to keep the work minimal when using the driver.

                //
                // Setup RX channel
                //
                rx_channel.set_peripheral_address(
                    unsafe { &(*$SPIX::ptr()).dr as *const _ as u32 },
                    false,
                );
                rx_channel.set_request_line($RX_CHSEL).unwrap();

                rx_channel.ccr().modify(|_, w| {
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
                tx_channel.set_peripheral_address(
                    unsafe { &(*$SPIX::ptr()).dr as *const _ as u32 },
                    false,
                );
                tx_channel.set_request_line($TX_CHSEL).unwrap();

                tx_channel.ccr().modify(|_, w| {
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
                let mut spi = self.payload.spi;
                // Keep the peripheral itself enabled after stopping DMA.
                spi.enable();
                (spi, self.channel)
            }
        }

        impl<PINS> SpiTxDma<$SPIX, PINS, $TX_CH> {
            pub fn split(mut self) -> (Spi<$SPIX, PINS>, $TX_CH) {
                self.stop();
                let mut spi = self.payload.spi;
                // Keep the peripheral itself enabled after stopping DMA.
                spi.enable();
                (spi, self.channel)
            }
        }

        impl<PINS> SpiRxTxDma<$SPIX, PINS, $RX_CH, $TX_CH> {
            pub fn split(mut self) -> (Spi<$SPIX, PINS>, $RX_CH, $TX_CH) {
                self.stop();
                let mut spi = self.payload.spi;
                // Keep the peripheral itself enabled after stopping DMA.
                spi.enable();
                (spi, self.rx_channel, self.tx_channel)
            }
        }

        impl<PINS> dma::TransferPayload for SpiRxDma<$SPIX, PINS, $RX_CH> {
            fn start(&mut self) {
                // Setup DMA channels in accordance with RM 40.4.9, subheading "Communication using
                // DMA (direct memory addressing)".
                // It is mandatory to follow these steps in order:
                //
                // 0. SPI disabled during setup.
                // 1. Enable DMA Rx buffer in the RXDMAEN bit in the SPI_CR2 register, if DMA Rx is used.
                // 2. Enable DMA streams for Tx and Rx in DMA registers, if the streams are used.
                // 3. Enable DMA Tx buffer in the TXDMAEN bit in the SPI_CR2 register, if DMA Tx is used.
                // 4. Enable the SPI by setting the SPE bit.
                self.payload.spi.disable(); // 0.
                self.payload
                    .spi
                    .spi
                    .cr2
                    .modify(|_, w| w.rxdmaen().set_bit()); // 1.
                self.channel.start(); // 2.
                self.payload.spi.enable(); // 4.
            }

            fn stop(&mut self) {
                // Stop DMA channels in accordance with RM 40.4.9, subheading "Communication using
                // DMA (direct memory addressing)".
                // It is mandatory to follow these steps in order:
                //
                // 1. Disable DMA streams for Tx and Rx in the DMA registers, if the streams are used.
                // 2. Disable the SPI by following the SPI disable procedure.
                // 3. Disable DMA Tx and Rx buffers by clearing the TXDMAEN and RXDMAEN bits in the
                //    SPI_CR2 register, if DMA Tx and/or DMA Rx are used.
                self.channel.stop(); // 1.
                self.payload.spi.disable(); // 2.
                self.payload
                    .spi
                    .spi
                    .cr2
                    .modify(|_, w| w.rxdmaen().clear_bit()); // 3.
            }
        }

        impl<PINS> dma::TransferPayload for SpiTxDma<$SPIX, PINS, $TX_CH> {
            fn start(&mut self) {
                // Setup DMA channels in accordance with RM 40.4.9, subheading "Communication using
                // DMA (direct memory addressing)".
                // It is mandatory to follow these steps in order:
                //
                // 0. SPI disabled during setup.
                // 1. Enable DMA Rx buffer in the RXDMAEN bit in the SPI_CR2 register, if DMA Rx is used.
                // 2. Enable DMA streams for Tx and Rx in DMA registers, if the streams are used.
                // 3. Enable DMA Tx buffer in the TXDMAEN bit in the SPI_CR2 register, if DMA Tx is used.
                // 4. Enable the SPI by setting the SPE bit.
                self.payload.spi.disable(); // 0.
                self.channel.start(); // 2.
                self.payload
                    .spi
                    .spi
                    .cr2
                    .modify(|_, w| w.txdmaen().set_bit()); // 3.
                self.payload.spi.enable(); // 4.
            }

            fn stop(&mut self) {
                // Stop DMA channels in accordance with RM 40.4.9, subheading "Communication using
                // DMA (direct memory addressing)".
                // It is mandatory to follow these steps in order:
                //
                // 1. Disable DMA streams for Tx and Rx in the DMA registers, if the streams are used.
                // 2. Disable the SPI by following the SPI disable procedure.
                // 3. Disable DMA Tx and Rx buffers by clearing the TXDMAEN and RXDMAEN bits in the
                //    SPI_CR2 register, if DMA Tx and/or DMA Rx are used.
                self.channel.stop(); // 1.
                self.payload.spi.disable(); // 2.
                self.payload
                    .spi
                    .spi
                    .cr2
                    .modify(|_, w| w.txdmaen().clear_bit()); // 3.
            }
        }

        impl<PINS> dma::TransferPayload for SpiRxTxDma<$SPIX, PINS, $RX_CH, $TX_CH> {
            fn start(&mut self) {
                // Setup DMA channels in accordance with RM 40.4.9, subheading "Communication using
                // DMA (direct memory addressing)".
                // It is mandatory to follow these steps in order:
                //
                // 0. SPI disabled during setup.
                // 1. Enable DMA Rx buffer in the RXDMAEN bit in the SPI_CR2 register, if DMA Rx is used.
                // 2. Enable DMA streams for Tx and Rx in DMA registers, if the streams are used.
                // 3. Enable DMA Tx buffer in the TXDMAEN bit in the SPI_CR2 register, if DMA Tx is used.
                // 4. Enable the SPI by setting the SPE bit.
                self.payload.spi.disable(); // 0.
                self.payload
                    .spi
                    .spi
                    .cr2
                    .modify(|_, w| w.rxdmaen().set_bit()); // 1.
                self.rx_channel.start(); // 2.
                self.tx_channel.start(); // 2.
                self.payload
                    .spi
                    .spi
                    .cr2
                    .modify(|_, w| w.txdmaen().set_bit()); // 3.
                self.payload.spi.enable(); // 4.
            }

            fn stop(&mut self) {
                // Stop DMA channels in accordance with RM 40.4.9, subheading "Communication using
                // DMA (direct memory addressing)".
                // It is mandatory to follow these steps in order:
                //
                // 1. Disable DMA streams for Tx and Rx in the DMA registers, if the streams are used.
                // 2. Disable the SPI by following the SPI disable procedure.
                // 3. Disable DMA Tx and Rx buffers by clearing the TXDMAEN and RXDMAEN bits in the
                //    SPI_CR2 register, if DMA Tx and/or DMA Rx are used.
                self.tx_channel.stop(); // 1.
                self.rx_channel.stop(); // 1.
                self.payload.spi.disable(); // 2.
                self.payload
                    .spi
                    .spi
                    .cr2
                    .modify(|_, w| w.rxdmaen().clear_bit().txdmaen().clear_bit()); // 3.
            }
        }

        impl<B, PINS> dma::ReadDma<B, u8> for SpiRxDma<$SPIX, PINS, $RX_CH>
        where
            B: StaticWriteBuffer<Word = u8>,
        {
            fn read(mut self, mut buffer: B) -> dma::Transfer<dma::W, B, Self> {
                // Setup DMA channels in accordance with RM 40.4.9, subheading "Communication using
                // DMA (direct memory addressing)"

                // NOTE(unsafe) We own the buffer now and we won't call other `&mut` on it
                // until the end of the transfer.
                let (ptr, len) = unsafe { buffer.static_write_buffer() };

                // Setup RX channel addresses and length
                self.channel.set_memory_address(ptr as u32, true);
                self.channel.set_transfer_length(len as u16);

                // Fences and start
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
                // Setup DMA channels in accordance with RM 40.4.9, subheading "Communication using
                // DMA (direct memory addressing)"

                // NOTE(unsafe) We own the buffer now and we won't call other `&mut` on it
                // until the end of the transfer.
                let (ptr, len) = unsafe { buffer.static_read_buffer() };

                // Setup TX channel addresses and length
                self.channel.set_memory_address(ptr as u32, true);
                self.channel.set_transfer_length(len as u16);

                // Fences and start
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
                // Setup DMA channels in accordance with RM 40.4.9, subheading "Communication using
                // DMA (direct memory addressing)"

                // Transfer: we use the same buffer for RX and TX

                // NOTE(unsafe) We own the buffer now and we won't call other `&mut` on it
                // until the end of the transfer.
                let (ptr, len) = unsafe { buffer.static_write_buffer() };

                // Setup RX channel addresses and length
                self.rx_channel.set_memory_address(ptr as u32, true);
                self.rx_channel.set_transfer_length(len as u16);

                // Setup TX channel addresses and length
                self.tx_channel.set_memory_address(ptr as u32, true);
                self.tx_channel.set_transfer_length(len as u16);

                // Fences and start
                atomic::compiler_fence(Ordering::Release);
                self.start();

                dma::Transfer::rw(buffer, self)
            }
        }
    };
}

spi_dma!(SPI1, dma1::C2, DmaInput::Spi1Rx, dma1::C3, DmaInput::Spi1Tx);
#[cfg(not(any(
    feature = "stm32l412",
    feature = "stm32l422",
    feature = "stm32l432",
    feature = "stm32l442",
    feature = "stm32l452",
    feature = "stm32l462",
)))]
spi_dma!(SPI2, dma1::C4, DmaInput::Spi2Rx, dma1::C5, DmaInput::Spi2Tx);
// spi_dma!(SPI1, dma2::C3, c3s, map4, dma2::C4, c4s, map4);
#[cfg(not(any(feature = "stm32l433", feature = "stm32l443",)))]
spi_dma!(SPI3, dma2::C1, DmaInput::Spi3Rx, dma2::C2, DmaInput::Spi3Tx);
