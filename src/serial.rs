//! Serial module
//!
//! This module support both polling and interrupt based accesses to the serial peripherals.

use core::fmt;
use core::marker::PhantomData;
use core::ops::DerefMut;
use core::ptr;
use core::sync::atomic::{self, Ordering};
use embedded_dma::StaticWriteBuffer;
use stable_deref_trait::StableDeref;

use crate::hal::serial::{self, Write};

use crate::dma::{
    dma1, CircBuffer, DMAFrame, FrameReader, FrameSender, Receive, RxDma, TransferPayload,
    Transmit, TxDma,
};
use crate::gpio::{self, Alternate, OpenDrain, PushPull};
use crate::pac;
use crate::rcc::{Clocks, Enable, RccBus, Reset};
use crate::time::{Bps, U32Ext};

#[cfg(any(has_peripheral = "uart4", has_peripheral = "uart5",))]
use crate::dma::dma2;

/// Interrupt event
pub enum Event {
    /// New data has been received
    Rxne,
    /// New data can be sent
    Txe,
    /// The line has gone idle
    Idle,
    /// Character match
    CharacterMatch,
    /// Receiver timeout
    ReceiverTimeout,
}

/// Serial error
#[non_exhaustive]
#[derive(Debug)]
pub enum Error {
    /// Framing error
    Framing,
    /// Noise error
    Noise,
    /// RX buffer overrun
    Overrun,
    /// Parity check error
    Parity,
}

/// USART parity settings
pub enum Parity {
    /// No parity
    ParityNone,
    /// Even parity
    ParityEven,
    /// Odd parity
    ParityOdd,
}

/// USART stopbits settings
pub enum StopBits {
    /// 1 stop bit
    STOP1,
    /// 0.5 stop bits
    STOP0P5,
    /// 2 stop bits
    STOP2,
    // 1.5 stop bits
    STOP1P5,
}

/// USART oversampling settings
pub enum Oversampling {
    /// Oversample 8 times (allows for faster data rates)
    Over8,
    /// Oversample 16 times (higher stability)
    Over16,
}

/// USART Configuration structure
pub struct Config {
    baudrate: Bps,
    parity: Parity,
    stopbits: StopBits,
    oversampling: Oversampling,
    character_match: Option<u8>,
    receiver_timeout: Option<u32>,
    disable_overrun: bool,
    onebit_sampling: bool,
}

impl Config {
    /// Set the baudrate to a specific value
    pub fn baudrate(mut self, baudrate: Bps) -> Self {
        self.baudrate = baudrate;
        self
    }

    /// Set parity to none
    pub fn parity_none(mut self) -> Self {
        self.parity = Parity::ParityNone;
        self
    }

    /// Set parity to even
    pub fn parity_even(mut self) -> Self {
        self.parity = Parity::ParityEven;
        self
    }

    /// Set parity to odd
    pub fn parity_odd(mut self) -> Self {
        self.parity = Parity::ParityOdd;
        self
    }

    /// Set the number of stopbits
    pub fn stopbits(mut self, stopbits: StopBits) -> Self {
        self.stopbits = stopbits;
        self
    }

    /// Set the oversampling size
    pub fn oversampling(mut self, oversampling: Oversampling) -> Self {
        self.oversampling = oversampling;
        self
    }

    /// Set the character match character
    pub fn character_match(mut self, character_match: u8) -> Self {
        self.character_match = Some(character_match);
        self
    }

    /// Set the receiver timeout, the value is the number of bit durations
    ///
    /// Note that it only takes 24 bits, using more than this will cause a panic.
    pub fn receiver_timeout(mut self, receiver_timeout: u32) -> Self {
        assert!(receiver_timeout < 1 << 24);
        self.receiver_timeout = Some(receiver_timeout);
        self
    }

    /// Disable overrun detection
    pub fn with_overrun_disabled(mut self) -> Self {
        self.disable_overrun = true;
        self
    }

    /// Change to onebit sampling
    pub fn with_onebit_sampling(mut self) -> Self {
        self.onebit_sampling = true;
        self
    }
}

impl Default for Config {
    fn default() -> Config {
        let baudrate = 115_200_u32.bps();
        Config {
            baudrate,
            parity: Parity::ParityNone,
            stopbits: StopBits::STOP1,
            oversampling: Oversampling::Over16,
            character_match: None,
            receiver_timeout: None,
            disable_overrun: false,
            onebit_sampling: false,
        }
    }
}

/// Serial abstraction
pub struct Serial<USART, PINS> {
    usart: USART,
    pins: PINS,
}

/// Serial receiver
pub struct Rx<USART> {
    _usart: PhantomData<USART>,
}

/// Serial transmitter
pub struct Tx<USART> {
    _usart: PhantomData<USART>,
}

macro_rules! hal {
    ($(
        $(#[$meta:meta])*
        $USARTX:ident: (
            $usartX:ident,
            $pclkX:ident,
            tx: ($txdma:ident, $dmacst:ident, $dmatxch:path),
            rx: ($rxdma:ident, $dmacsr:ident, $dmarxch:path)
        ),
    )+) => {
        $(
            impl<PINS> Serial<pac::$USARTX, PINS> {
                /// Configures the serial interface and creates the interface
                /// struct.
                ///
                /// `Config` is a config struct that configures baud rate, stop bits and parity.
                ///
                /// `Clocks` passes information about the current frequencies of
                /// the clocks.  The existence of the struct ensures that the
                /// clock settings are fixed.
                ///
                /// The `serial` struct takes ownership over the `USARTX` device
                /// registers and the specified `PINS`
                ///
                /// `MAPR` and `APBX` are register handles which are passed for
                /// configuration. (`MAPR` is used to map the USART to the
                /// corresponding pins. `APBX` is used to reset the USART.)
                pub fn $usartX(
                    usart: pac::$USARTX,
                    pins: PINS,
                    config: Config,
                    clocks: Clocks,
                    apb: &mut <pac::$USARTX as RccBus>::Bus,
                ) -> Self
                where
                    PINS: Pins<pac::$USARTX>,
                {
                    // enable or reset $USARTX
                    <pac::$USARTX>::enable(apb);
                    <pac::$USARTX>::reset(apb);

                    // Reset other registers to disable advanced USART features
                    usart.cr1.reset();
                    usart.cr2.reset();
                    usart.cr3.reset();

                    // Configure baud rate
                    match config.oversampling {
                        Oversampling::Over8 => {
                            let uartdiv = 2 * clocks.$pclkX().0 / config.baudrate.0;
                            assert!(uartdiv >= 16, "impossible baud rate");

                            let lower = (uartdiv & 0xf) >> 1;
                            let brr = (uartdiv & !0xf) | lower;

                            usart.cr1.modify(|_, w| w.over8().set_bit());
                            usart.brr.write(|w| unsafe { w.bits(brr) });
                        }
                        Oversampling::Over16 => {
                            let brr = clocks.$pclkX().0 / config.baudrate.0;
                            assert!(brr >= 16, "impossible baud rate");

                            usart.brr.write(|w| unsafe { w.bits(brr) });
                        }
                    }

                    if let Some(val) = config.receiver_timeout {
                        usart.rtor.modify(|_, w| w.rto().bits(val));
                    }

                    // enable DMA transfers
                    usart.cr3.modify(|_, w| w.dmat().set_bit().dmar().set_bit());

                    // Configure hardware flow control (CTS/RTS or RS485 Driver Enable)
                    if PINS::FLOWCTL {
                        usart.cr3.modify(|_, w| w.rtse().set_bit().ctse().set_bit());
                    } else if PINS::DEM {
                        usart.cr3.modify(|_, w| w.dem().set_bit());

                        // Pre/post driver enable set conservative to the max time
                        usart.cr1.modify(|_, w| w.deat().bits(0b1111).dedt().bits(0b1111));
                    } else {
                        usart.cr3.modify(|_, w| w.rtse().clear_bit().ctse().clear_bit());
                    }

                    // Enable One bit sampling method
                    usart.cr3.modify(|_, w| {
                        if config.onebit_sampling {
                            w.onebit().set_bit();
                        }

                        if config.disable_overrun {
                            w.ovrdis().set_bit();
                        }

                        // configure Half Duplex
                        if PINS::HALF_DUPLEX {
                            w.hdsel().set_bit();
                        }

                        w
                    });

                    // Configure parity and word length
                    // Unlike most uart devices, the "word length" of this usart device refers to
                    // the size of the data plus the parity bit. I.e. "word length"=8, parity=even
                    // results in 7 bits of data. Therefore, in order to get 8 bits and one parity
                    // bit, we need to set the "word" length to 9 when using parity bits.
                    let (word_length, parity_control_enable, parity) = match config.parity {
                        Parity::ParityNone => (false, false, false),
                        Parity::ParityEven => (true, true, false),
                        Parity::ParityOdd => (true, true, true),
                    };
                    usart.cr1.modify(|_r, w| {
                        w
                            .m0().bit(word_length)
                            .ps().bit(parity)
                            .pce().bit(parity_control_enable)
                    });

                    // Configure stop bits
                    let stop_bits = match config.stopbits {
                        StopBits::STOP1 => 0b00,
                        StopBits::STOP0P5 => 0b01,
                        StopBits::STOP2 => 0b10,
                        StopBits::STOP1P5 => 0b11,
                    };
                    usart.cr2.modify(|_r, w| {
                        w.stop().bits(stop_bits);

                        // Setup character match (if requested)
                        if let Some(c) = config.character_match {
                            w.add().bits(c);
                        }

                        if config.receiver_timeout.is_some() {
                            w.rtoen().set_bit();
                        }

                        w
                    });


                    // UE: enable USART
                    // RE: enable receiver
                    // TE: enable transceiver
                    usart
                        .cr1
                        .modify(|_, w| w.ue().set_bit().re().set_bit().te().set_bit());

                    Serial { usart, pins }
                }

                /// Starts listening for an interrupt event
                pub fn listen(&mut self, event: Event) {
                    match event {
                        Event::Rxne => {
                            self.usart.cr1.modify(|_, w| w.rxneie().set_bit())
                        },
                        Event::Txe => {
                            self.usart.cr1.modify(|_, w| w.txeie().set_bit())
                        },
                        Event::Idle => {
                            self.usart.cr1.modify(|_, w| w.idleie().set_bit())
                        },
                        Event::CharacterMatch => {
                            self.usart.cr1.modify(|_, w| w.cmie().set_bit())
                        },
                        Event::ReceiverTimeout => {
                            self.usart.cr1.modify(|_, w| w.rtoie().set_bit())
                        },
                    }
                }

                /// Check for, and return, any errors
                ///
                /// See [`Rx::check_for_error`].
                pub fn check_for_error() -> Result<(), Error> {
                    let mut rx: Rx<pac::$USARTX> = Rx {
                        _usart: PhantomData,
                    };
                    rx.check_for_error()
                }

                /// Stops listening for an interrupt event
                pub fn unlisten(&mut self, event: Event) {
                    match event {
                        Event::Rxne => {
                            self.usart.cr1.modify(|_, w| w.rxneie().clear_bit())
                        },
                        Event::Txe => {
                            self.usart.cr1.modify(|_, w| w.txeie().clear_bit())
                        },
                        Event::Idle => {
                            self.usart.cr1.modify(|_, w| w.idleie().clear_bit())
                        },
                        Event::CharacterMatch => {
                            self.usart.cr1.modify(|_, w| w.cmie().clear_bit())
                        },
                        Event::ReceiverTimeout => {
                            self.usart.cr1.modify(|_, w| w.rtoie().clear_bit())
                        },
                    }
                }

                /// Splits the `Serial` abstraction into a transmitter and a receiver half
                pub fn split(self) -> (Tx<pac::$USARTX>, Rx<pac::$USARTX>) {
                    (
                        Tx {
                            _usart: PhantomData,
                        },
                        Rx {
                            _usart: PhantomData,
                        },
                    )
                }

                /// Frees the USART peripheral
                pub fn release(self) -> (pac::$USARTX, PINS) {
                    (self.usart, self.pins)
                }
            }

            impl<PINS> serial::Read<u8> for Serial<pac::$USARTX, PINS> {
                type Error = Error;

                fn read(&mut self) -> nb::Result<u8, Error> {
                    let mut rx: Rx<pac::$USARTX> = Rx {
                        _usart: PhantomData,
                    };
                    rx.read()
                }
            }

            impl serial::Read<u8> for Rx<pac::$USARTX> {
                type Error = Error;

                fn read(&mut self) -> nb::Result<u8, Error> {
                    self.check_for_error()?;

                    // NOTE(unsafe) atomic read with no side effects
                    let isr = unsafe { (*pac::$USARTX::ptr()).isr.read() };

                    if isr.rxne().bit_is_set() {
                        // NOTE(read_volatile) see `write_volatile` below
                        return Ok(unsafe {
                            ptr::read_volatile(&(*pac::$USARTX::ptr()).rdr as *const _ as *const _)
                        });
                    }

                    Err(nb::Error::WouldBlock)
                }
            }

            impl<PINS> serial::Write<u8> for Serial<pac::$USARTX, PINS> {
                type Error = Error;

                fn flush(&mut self) -> nb::Result<(), Error> {
                    let mut tx: Tx<pac::$USARTX> = Tx {
                        _usart: PhantomData,
                    };
                    tx.flush()
                }

                fn write(&mut self, byte: u8) -> nb::Result<(), Error> {
                    let mut tx: Tx<pac::$USARTX> = Tx {
                        _usart: PhantomData,
                    };
                    tx.write(byte)
                }
            }

            impl serial::Write<u8> for Tx<pac::$USARTX> {
                // NOTE(Void) See section "29.7 USART interrupts"; the only possible errors during
                // transmission are: clear to send (which is disabled in this case) errors and
                // framing errors (which only occur in SmartCard mode); neither of these apply to
                // our hardware configuration
                type Error = Error;

                fn flush(&mut self) -> nb::Result<(), Error> {
                    // NOTE(unsafe) atomic read with no side effects
                    let isr = unsafe { (*pac::$USARTX::ptr()).isr.read() };

                    if isr.tc().bit_is_set() {
                        Ok(())
                    } else {
                        Err(nb::Error::WouldBlock)
                    }
                }

                fn write(&mut self, byte: u8) -> nb::Result<(), Error> {
                    // NOTE(unsafe) atomic read with no side effects
                    let isr = unsafe { (*pac::$USARTX::ptr()).isr.read() };

                    if isr.txe().bit_is_set() {
                        // NOTE(unsafe) atomic write to stateless register
                        // NOTE(write_volatile) 8-bit write that's not possible through the svd2rust API
                        unsafe {
                            ptr::write_volatile(&(*pac::$USARTX::ptr()).tdr as *const _ as *mut _, byte)
                        }
                        Ok(())
                    } else {
                        Err(nb::Error::WouldBlock)
                    }
                }
            }

            impl embedded_hal::blocking::serial::write::Default<u8>
                for Tx<pac::$USARTX> {}

            pub type $rxdma = RxDma<Rx<pac::$USARTX>, $dmarxch>;
            pub type $txdma = TxDma<Tx<pac::$USARTX>, $dmatxch>;

            impl Receive for $rxdma {
                type RxChannel = $dmarxch;
                type TransmittedWord = u8;
            }

            impl Transmit for $txdma {
                type TxChannel = $dmatxch;
                type ReceivedWord = u8;
            }

            impl TransferPayload for $rxdma {
                fn start(&mut self) {
                    self.channel.start();
                }
                fn stop(&mut self) {
                    self.channel.stop();
                }
            }

            impl TransferPayload for $txdma {
                fn start(&mut self) {
                    self.channel.start();
                }
                fn stop(&mut self) {
                    self.channel.stop();
                }
            }

            impl Rx<pac::$USARTX> {
                pub fn with_dma(self, channel: $dmarxch) -> $rxdma {
                    RxDma {
                        payload: self,
                        channel,
                    }
                }

                /// Check for, and return, any errors
                ///
                /// The `read` methods can only return one error at a time, but
                /// there might actually be multiple errors. This method will
                /// return and clear a currently active error. Once it returns
                /// `Ok(())`, it should be possible to proceed with the next
                /// `read` call unimpeded.
                pub fn check_for_error(&mut self) -> Result<(), Error> {
                    // NOTE(unsafe): Only used for atomic access.
                    let isr = unsafe { (*pac::$USARTX::ptr()).isr.read() };
                    let icr = unsafe { &(*pac::$USARTX::ptr()).icr };

                    if isr.pe().bit_is_set() {
                        icr.write(|w| w.pecf().clear());
                        return Err(Error::Parity);
                    }
                    if isr.fe().bit_is_set() {
                        icr.write(|w| w.fecf().clear());
                        return Err(Error::Framing);
                    }
                    if isr.nf().bit_is_set() {
                        icr.write(|w| w.ncf().clear());
                        return Err(Error::Noise);
                    }
                    if isr.ore().bit_is_set() {
                        icr.write(|w| w.orecf().clear());
                        return Err(Error::Overrun);
                    }

                    Ok(())
                }

                /// Checks to see if the USART peripheral has detected an idle line and clears
                /// the flag
                pub fn is_idle(&mut self, clear: bool) -> bool {
                    let isr = unsafe { &(*pac::$USARTX::ptr()).isr.read() };
                    let icr = unsafe { &(*pac::$USARTX::ptr()).icr };

                    if isr.idle().bit_is_set() {
                        if clear {
                            icr.write(|w| w.idlecf().set_bit() );
                        }
                        true
                    } else {
                        false
                    }
                }


                /// Checks to see if the USART peripheral has detected an receiver timeout and
                /// clears the flag
                pub fn is_receiver_timeout(&mut self, clear: bool) -> bool {
                    let isr = unsafe { &(*pac::$USARTX::ptr()).isr.read() };
                    let icr = unsafe { &(*pac::$USARTX::ptr()).icr };

                    if isr.rtof().bit_is_set() {
                        if clear {
                            icr.write(|w| w.rtocf().set_bit() );
                        }
                        true
                    } else {
                        false
                    }
                }

                /// Checks to see if the USART peripheral has detected an character match and
                /// clears the flag
                pub fn check_character_match(&mut self, clear: bool) -> bool {
                    let isr = unsafe { &(*pac::$USARTX::ptr()).isr.read() };
                    let icr = unsafe { &(*pac::$USARTX::ptr()).icr };

                    if isr.cmf().bit_is_set() {
                        if clear {
                            icr.write(|w| w.cmcf().set_bit() );
                        }
                        true
                    } else {
                        false
                    }
                }
            }

            impl crate::dma::CharacterMatch for Rx<pac::$USARTX> {
                /// Checks to see if the USART peripheral has detected an character match and
                /// clears the flag
                fn check_character_match(&mut self, clear: bool) -> bool {
                    self.check_character_match(clear)
                }
            }

            impl crate::dma::ReceiverTimeout for Rx<pac::$USARTX> {
                fn check_receiver_timeout(&mut self, clear: bool) -> bool {
                    self.is_receiver_timeout(clear)
                }
            }

            impl crate::dma::OperationError<(), Error> for Rx<pac::$USARTX>{
                fn check_operation_error(&mut self) -> Result<(), Error> {
                    self.check_for_error()
                }
            }

            impl Tx<pac::$USARTX> {
                pub fn with_dma(self, channel: $dmatxch) -> $txdma {
                    TxDma {
                        payload: self,
                        channel,
                    }
                }
            }

            impl $rxdma {
                pub fn split(mut self) -> (Rx<pac::$USARTX>, $dmarxch) {
                    self.stop();
                    let RxDma {payload, channel} = self;
                    (
                        payload,
                        channel
                    )
                }
            }

            impl $txdma {
                pub fn split(mut self) -> (Tx<pac::$USARTX>, $dmatxch) {
                    self.stop();
                    let TxDma {payload, channel} = self;
                    (
                        payload,
                        channel,
                    )
                }
            }

            impl<B> crate::dma::CircReadDma<B, u8> for $rxdma
            where
                &'static mut B: StaticWriteBuffer<Word = u8>,
                B: 'static,
                Self: core::marker::Sized,
            {
                fn circ_read(mut self, mut buffer: &'static mut B,
                ) -> CircBuffer<B, Self>
                {
                    let (ptr, len) = unsafe { buffer.static_write_buffer() };
                    self.channel.set_peripheral_address(
                        unsafe { &(*pac::$USARTX::ptr()).rdr as *const _ as u32 },
                        false,
                    );
                    self.channel.set_memory_address(ptr as u32, true);
                    self.channel.set_transfer_length(len as u16);

                    // Tell DMA to request from serial
                    self.channel.cselr().modify(|_, w| {
                        w.$dmacsr().map2()
                    });

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
                            .set_bit()
                            // write to memory
                            .dir()
                            .clear_bit()
                    });

                    // NOTE(compiler_fence) operations on `buffer` should not be reordered after
                    // the next statement, which starts the DMA transfer
                    atomic::compiler_fence(Ordering::Release);

                    self.start();

                    CircBuffer::new(buffer, self)
                }
            }

            impl $rxdma {
                /// Create a frame reader that can either react on the Character match interrupt or
                /// Transfer Complete from the DMA.
                pub fn frame_reader<BUFFER, const N: usize>(
                    mut self,
                    buffer: BUFFER,
                ) -> FrameReader<BUFFER, Self, N>
                    where
                        BUFFER: Sized + StableDeref<Target = DMAFrame<N>> + DerefMut + 'static,
                {
                    let usart = unsafe{ &(*pac::$USARTX::ptr()) };

                    // Setup DMA transfer
                    let buf = &*buffer;
                    self.channel.set_peripheral_address(&usart.rdr as *const _ as u32, false);
                    self.channel.set_memory_address(unsafe { buf.buffer_address_for_dma() } as u32, true);
                    self.channel.set_transfer_length(buf.max_len() as u16);

                    // Tell DMA to request from serial
                    self.channel.cselr().modify(|_, w| {
                        w.$dmacsr().map2()
                    });

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
                            // Peripheral -> Mem
                            .dir()
                            .clear_bit()
                    });

                    // NOTE(compiler_fence) operations on `buffer` should not be reordered after
                    // the next statement, which starts the DMA transfer
                    atomic::compiler_fence(Ordering::Release);

                    self.channel.start();

                    FrameReader::new(buffer, self, usart.cr2.read().add().bits())
                }
            }

            impl $txdma {
                /// Creates a new DMA frame sender
                pub fn frame_sender<BUFFER, const N: usize>(
                    mut self,
                ) -> FrameSender<BUFFER, Self, N>
                    where
                        BUFFER: Sized + StableDeref<Target = DMAFrame<N>> + DerefMut + 'static,
                {
                    let usart = unsafe{ &(*pac::$USARTX::ptr()) };

                    // Setup DMA
                    self.channel.set_peripheral_address(&usart.tdr as *const _ as u32, false);

                    // Tell DMA to request from serial
                    self.channel.cselr().modify(|_, w| {
                        w.$dmacst().map2()
                    });

                    self.channel.ccr().modify(|_, w| unsafe {
                        w.mem2mem()
                            .clear_bit()
                            // 00: Low, 01: Medium, 10: High, 11: Very high
                            .pl()
                            .bits(0b01)
                            // 00: 8-bits, 01: 16-bits, 10: 32-bits, 11: Reserved
                            .msize()
                            .bits(0b00)
                            // 00: 8-bits, 01: 16-bits, 10: 32-bits, 11: Reserved
                            .psize()
                            .bits(0b00)
                            // Mem -> Peripheral
                            .dir()
                            .set_bit()
                    });

                    FrameSender::new(self)
                }
            }
        )+
    }
}

hal! {
    USART1: (usart1, pclk2, tx: (TxDma1, c4s, dma1::C4), rx: (RxDma1, c5s, dma1::C5)),
    USART2: (usart2, pclk1, tx: (TxDma2, c7s, dma1::C7), rx: (RxDma2, c6s, dma1::C6)),
}

#[cfg(has_peripheral = "usart3")]
hal! {
    USART3: (usart3, pclk1, tx: (TxDma3, c2s, dma1::C2), rx: (RxDma3, c3s, dma1::C3)),
}

#[cfg(has_peripheral = "uart4")]
hal! {
    UART4: (uart4, pclk1, tx: (TxDma4, c3s, dma2::C3), rx: (RxDma4, c5s, dma2::C5)),
}

#[cfg(has_peripheral = "uart5")]
hal! {
    UART5: (uart5, pclk1, tx: (TxDma5, c1s, dma2::C1), rx: (RxDma5, c2s, dma2::C2)),
}

impl<USART, PINS> fmt::Write for Serial<USART, PINS>
where
    Serial<USART, PINS>: crate::hal::serial::Write<u8>,
{
    fn write_str(&mut self, s: &str) -> fmt::Result {
        let _ = s
            .as_bytes()
            .iter()
            .map(|c| nb::block!(self.write(*c)))
            .last();
        Ok(())
    }
}

impl<USART> fmt::Write for Tx<USART>
where
    Tx<USART>: crate::hal::serial::Write<u8>,
{
    fn write_str(&mut self, s: &str) -> fmt::Result {
        let _ = s
            .as_bytes()
            .iter()
            .map(|c| nb::block!(self.write(*c)))
            .last();
        Ok(())
    }
}

/// Marks pins as being as being TX pins for the given USART instance
pub trait TxPin<Instance>: private::SealedTx {}

/// Marks pins as being TX Half Duplex pins for the given USART instance
pub trait TxHalfDuplexPin<Instance>: private::SealedTxHalfDuplex {}

/// Marks pins as being as being RX pins for the given USART instance
pub trait RxPin<Instance>: private::SealedRx {}

/// Marks pins as being as being RTS pins for the given USART instance
pub trait RtsDePin<Instance>: private::SealedRtsDe {}

/// Marks pins as being as being CTS pins for the given USART instance
pub trait CtsPin<Instance>: private::SealedCts {}

macro_rules! impl_pin_traits {
    (
        $(
            $instance:ident: {
                $(
                    $af:literal: {
                        TX: $($tx:ident),*;
                        RX: $($rx:ident),*;
                        RTS_DE: $($rts_de:ident),*;
                        CTS: $($cts:ident),*;
                    }
                )*
            }
        )*
    ) => {
        $(
            $(
                $(
                    impl private::SealedTx for
                        gpio::$tx<Alternate<PushPull, $af>> {}
                    impl TxPin<pac::$instance> for
                        gpio::$tx<Alternate<PushPull, $af>> {}
                )*

                $(
                    impl private::SealedTxHalfDuplex for
                        gpio::$tx<Alternate<OpenDrain, $af>> {}
                    impl TxHalfDuplexPin<pac::$instance> for
                        gpio::$tx<Alternate<OpenDrain, $af>> {}
                )*

                $(
                    impl private::SealedRx for
                        gpio::$rx<Alternate<PushPull, $af>> {}
                    impl RxPin<pac::$instance> for
                        gpio::$rx<Alternate<PushPull, $af>> {}
                )*

                $(
                    impl private::SealedRtsDe for
                        gpio::$rts_de<Alternate<PushPull, $af>> {}
                    impl RtsDePin<pac::$instance> for
                        gpio::$rts_de<Alternate<PushPull, $af>> {}
                )*

                $(
                    impl private::SealedCts for
                        gpio::$cts<Alternate<PushPull, $af>> {}
                    impl CtsPin<pac::$instance> for
                        gpio::$cts<Alternate<PushPull, $af>> {}
                )*
            )*
        )*
    };
}

impl_pin_traits! {
    USART1: {
        7: {
            TX: PA9, PB6;
            RX: PA10, PB7;
            RTS_DE: PA12, PB3;
            CTS: PA11, PB4;
        }
    }
    USART2: {
        7: {
            TX: PA2, PD5;
            RX: PA3, PD6;
            RTS_DE: PA1, PD4;
            CTS: PA0, PD3;
        }
        3: {
            TX: ;
            RX: PA15;
            RTS_DE: ;
            CTS: ;
        }
    }
    USART3: {
        7: {
            TX: PB10, PC4, PC10, PD8;
            RX: PB11, PC5, PC11, PD9;
            RTS_DE: PB1, PB14, PD2, PD12;
            CTS: PA6, PB13, PD11;
        }
    }
}

#[cfg(has_peripheral = "uart4")]
impl_pin_traits! {
    UART4: {
        8: {
            TX: PA0, PC10;
            RX: PA1, PC11;
            RTS_DE: PA15;
            CTS: PB7;
        }
    }
}

#[cfg(has_peripheral = "uart5")]
impl_pin_traits! {
    UART5: {
        8: {
            TX: PC12;
            RX: PD2;
            RTS_DE: PB4;
            CTS: PB5;
        }
    }
}

/// Pins trait for detecting hardware flow control or RS485 mode.
pub trait Pins<USART> {
    const FLOWCTL: bool;
    const DEM: bool;
    const HALF_DUPLEX: bool;
}

// No flow control, just Rx+Tx
impl<Instance, Tx, Rx> Pins<Instance> for (Tx, Rx)
where
    Tx: TxPin<Instance>,
    Rx: RxPin<Instance>,
{
    const FLOWCTL: bool = false;
    const DEM: bool = false;
    const HALF_DUPLEX: bool = false;
}

// No flow control Half_duplex, just Tx
impl<Instance, Tx> Pins<Instance> for (Tx,)
where
    Tx: TxHalfDuplexPin<Instance>,
{
    const FLOWCTL: bool = false;
    const DEM: bool = false;
    const HALF_DUPLEX: bool = true;
}

// Hardware flow control, Rx+Tx+Rts+Cts
impl<Instance, Tx, Rx, Rts, Cts> Pins<Instance> for (Tx, Rx, Rts, Cts)
where
    Tx: TxPin<Instance>,
    Rx: RxPin<Instance>,
    Rts: RtsDePin<Instance>,
    Cts: CtsPin<Instance>,
{
    const FLOWCTL: bool = true;
    const DEM: bool = false;
    const HALF_DUPLEX: bool = false;
}

// DEM for RS485 mode
impl<Instance, Tx, Rx, De> Pins<Instance> for (Tx, Rx, De)
where
    Tx: TxPin<Instance>,
    Rx: RxPin<Instance>,
    De: RtsDePin<Instance>,
{
    const FLOWCTL: bool = false;
    const DEM: bool = true;
    const HALF_DUPLEX: bool = false;
}

/// Contains supertraits used to restrict which traits users can implement
mod private {
    pub trait SealedTx {}
    pub trait SealedTxHalfDuplex {}
    pub trait SealedRx {}
    pub trait SealedRtsDe {}
    pub trait SealedCts {}
}
