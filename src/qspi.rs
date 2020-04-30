//! Quad Serial Peripheral Interface (QSPI) bus

use crate::gpio::gpioe::{PE10, PE11, PE12, PE13, PE14, PE15};
use crate::gpio::{Alternate, Floating, Input, AF10};
use crate::rcc::AHB3;
use crate::stm32::QUADSPI;
use core::ptr;

#[doc(hidden)]
mod private {
    pub trait Sealed {}
}

/// CLK pin. This trait is sealed and cannot be implemented.
pub trait ClkPin<QSPI>: private::Sealed {}
/// nCS pin. This trait is sealed and cannot be implemented.
pub trait nCSPin<QSPI>: private::Sealed {}
/// IO0 pin. This trait is sealed and cannot be implemented.
pub trait IO0Pin<QSPI>: private::Sealed {}
/// IO1 pin. This trait is sealed and cannot be implemented.
pub trait IO1Pin<QSPI>: private::Sealed {}
/// IO2 pin. This trait is sealed and cannot be implemented.
pub trait IO2Pin<QSPI>: private::Sealed {}
/// IO3 pin. This trait is sealed and cannot be implemented.
pub trait IO3Pin<QSPI>: private::Sealed {}

macro_rules! pins {
    ($qspi:ident, $af:ident, CLK: [$($clk:ident),*], nCS: [$($ncs:ident),*],
        IO0: [$($io0:ident),*], IO1: [$($io1:ident),*], IO2: [$($io2:ident),*],
        IO3: [$($io3:ident),*]) => {
        $(
            impl private::Sealed for $clk<Alternate<$af, Input<Floating>>> {}
            impl ClkPin<$qspi> for $clk<Alternate<$af, Input<Floating>>> {}
        )*
        $(
            impl private::Sealed for $ncs<Alternate<$af, Input<Floating>>> {}
            impl nCSPin<$qspi> for $ncs<Alternate<$af, Input<Floating>>> {}
        )*
        $(
            impl private::Sealed for $io0<Alternate<$af, Input<Floating>>> {}
            impl IO0Pin<$qspi> for $io0<Alternate<$af, Input<Floating>>> {}
        )*
        $(
            impl private::Sealed for $io1<Alternate<$af, Input<Floating>>> {}
            impl IO1Pin<$qspi> for $io1<Alternate<$af, Input<Floating>>> {}
        )*
        $(
            impl private::Sealed for $io2<Alternate<$af, Input<Floating>>> {}
            impl IO2Pin<$qspi> for $io2<Alternate<$af, Input<Floating>>> {}
        )*
        $(
            impl private::Sealed for $io3<Alternate<$af, Input<Floating>>> {}
            impl IO3Pin<$qspi> for $io3<Alternate<$af, Input<Floating>>> {}
        )*
    }
}

#[cfg(feature = "stm32l4x5")]
pins!(
    QUADSPI,
    AF10,
    CLK: [PE10],
    nCS: [PE11],
    IO0: [PE12],
    IO1: [PE13],
    IO2: [PE14],
    IO3: [PE15]
);

#[derive(Copy, Clone, Debug, PartialEq)]
#[repr(u8)]
pub enum QspiMode {
    SingleChannel = 0b01,
    DualChannel = 0b10,
    QuadChannel = 0b11,
}

#[derive(Copy, Clone, Debug, PartialEq)]
#[repr(u8)]
pub enum AddressSize {
    Addr8Bit = 0b00,
    Addr16Bit = 0b01,
    Addr24Bit = 0b10,
    Addr32Bit = 0b11,
}

#[derive(Copy, Clone, Debug, PartialEq)]
pub enum SampleShift {
    None,
    HalfACycle,
}

#[derive(Copy, Clone, Debug, PartialEq)]
pub enum ClockMode {
    Mode0,
    Mode3,
}

#[derive(Copy, Clone, Debug, PartialEq)]
pub struct QspiConfig {
    /// This field defines the scaler factor for generating CLK based on the AHB clock
    /// (value+1).
    clock_prescaler: u8,
    /// Number of bytes in Flash memory = 2^[FSIZE+1]
    flash_size: u8,
    address_size: AddressSize,
    /// This bit indicates the level that CLK takes between commands Mode 0(low) / mode 3(high)
    clock_mode: ClockMode,
    /// FIFO threshold level (Activates FTF, QUADSPI_SR[2]) 0-15.
    fifo_threshold: u8,
    sample_shift: SampleShift,
    /// CSHT+1 defines the minimum number of CLK cycles which the chip select (nCS) must
    /// remain high between commands issued to the Flash memory.
    chip_select_high_time: u8,
    qpi_mode: bool,
}

impl Default for QspiConfig {
    fn default() -> QspiConfig {
        QspiConfig {
            clock_prescaler: 0,
            flash_size: 22, //8MB //26 = 128MB
            address_size: AddressSize::Addr24Bit,
            clock_mode: ClockMode::Mode0,
            fifo_threshold: 1,
            sample_shift: SampleShift::HalfACycle,
            chip_select_high_time: 1,
            qpi_mode: false,
        }
    }
}

impl QspiConfig {
    pub fn clock_prescaler(mut self, clk_pre: u8) -> Self {
        self.clock_prescaler = clk_pre;
        self
    }

    pub fn flash_size(mut self, fl_size: u8) -> Self {
        self.flash_size = fl_size;
        self
    }

    pub fn address_size(mut self, add_size: AddressSize) -> Self {
        self.address_size = add_size;
        self
    }

    pub fn clock_mode(mut self, clk_mode: ClockMode) -> Self {
        self.clock_mode = clk_mode;
        self
    }

    pub fn fifo_threshold(mut self, fifo_thres: u8) -> Self {
        self.fifo_threshold = fifo_thres;
        self
    }

    pub fn sample_shift(mut self, shift: SampleShift) -> Self {
        self.sample_shift = shift;
        self
    }

    pub fn chip_select_high_time(mut self, csht: u8) -> Self {
        self.chip_select_high_time = csht;
        self
    }

    pub fn qpi_mode(mut self, qpi: bool) -> Self {
        self.qpi_mode = qpi;
        self
    }
}

#[derive(Copy, Clone, Debug, PartialEq)]
pub struct QspiWriteCommand<'c> {
    pub instruction: Option<(u8, QspiMode)>,
    pub address: Option<(u32, QspiMode)>,
    pub alternative_bytes: Option<(&'c [u8], QspiMode)>,
    pub dummy_cycles: u8,
    pub data: Option<(&'c [u8], QspiMode)>,
    pub double_data_rate: bool,
}

#[derive(Copy, Clone, Debug, PartialEq)]
pub struct QspiReadCommand<'c> {
    pub instruction: Option<(u8, QspiMode)>,
    pub address: Option<(u32, QspiMode)>,
    pub alternative_bytes: Option<(&'c [u8], QspiMode)>,
    pub dummy_cycles: u8,
    pub data_mode: QspiMode,
    pub recive_lenght: u32,
    pub double_data_rate: bool,
}

impl<'c> QspiWriteCommand<'c> {
    pub fn address(self, addr: u32, mode: QspiMode) -> Self {
        QspiWriteCommand {
            instruction: self.instruction,
            address: Some((addr, mode)),
            alternative_bytes: self.alternative_bytes,
            dummy_cycles: self.dummy_cycles,
            data: self.data,
            double_data_rate: self.double_data_rate,
        }
    }

    pub fn alternative_bytes(self, bytes: &'c [u8], mode: QspiMode) -> Self {
        QspiWriteCommand {
            instruction: self.instruction,
            address: self.address,
            alternative_bytes: Some((bytes, mode)),
            dummy_cycles: self.dummy_cycles,
            data: self.data,
            double_data_rate: self.double_data_rate,
        }
    }

    pub fn dummy_cycles(self, n: u8) -> Self {
        QspiWriteCommand {
            instruction: self.instruction,
            address: self.address,
            alternative_bytes: self.alternative_bytes,
            dummy_cycles: n,
            data: self.data,
            double_data_rate: self.double_data_rate,
        }
    }

    pub fn data(self, bytes: &'c [u8], mode: QspiMode) -> Self {
        QspiWriteCommand {
            instruction: self.instruction,
            address: self.address,
            alternative_bytes: self.alternative_bytes,
            dummy_cycles: self.dummy_cycles,
            data: Some((bytes, mode)),
            double_data_rate: self.double_data_rate,
        }
    }
}

impl<'c> QspiReadCommand<'c> {
    pub fn address(self, addr: u32, mode: QspiMode) -> Self {
        QspiReadCommand {
            instruction: self.instruction,
            address: Some((addr, mode)),
            alternative_bytes: self.alternative_bytes,
            dummy_cycles: self.dummy_cycles,
            data_mode: self.data_mode,
            recive_lenght: self.recive_lenght,
            double_data_rate: self.double_data_rate,
        }
    }

    pub fn alternative_bytes(self, bytes: &'c [u8], mode: QspiMode) -> Self {
        QspiReadCommand {
            instruction: self.instruction,
            address: self.address,
            alternative_bytes: Some((bytes, mode)),
            dummy_cycles: self.dummy_cycles,
            data_mode: self.data_mode,
            recive_lenght: self.recive_lenght,
            double_data_rate: self.double_data_rate,
        }
    }

    pub fn dummy_cycles(self, n: u8) -> Self {
        QspiReadCommand {
            instruction: self.instruction,
            address: self.address,
            alternative_bytes: self.alternative_bytes,
            dummy_cycles: n,
            data_mode: self.data_mode,
            recive_lenght: self.recive_lenght,
            double_data_rate: self.double_data_rate,
        }
    }

    pub fn recive_lenght(self, length: u32) -> Self {
        QspiReadCommand {
            instruction: self.instruction,
            address: self.address,
            alternative_bytes: self.alternative_bytes,
            dummy_cycles: self.dummy_cycles,
            data_mode: self.data_mode,
            recive_lenght: length,
            double_data_rate: self.double_data_rate,
        }
    }
}

pub struct Qspi<PINS> {
    qspi: QUADSPI,
    pins: PINS,
    config: QspiConfig,
}

impl<CLK, NCS, IO0, IO1, IO2, IO3> Qspi<(CLK, NCS, IO0, IO1, IO2, IO3)> {
    pub fn new(
        qspi: QUADSPI,
        pins: (CLK, NCS, IO0, IO1, IO2, IO3),
        ahb3: &mut AHB3,
        config: QspiConfig,
    ) -> Self
    where
        CLK: ClkPin<QUADSPI>,
        NCS: nCSPin<QUADSPI>,
        IO0: IO0Pin<QUADSPI>,
        IO1: IO1Pin<QUADSPI>,
        IO2: IO2Pin<QUADSPI>,
        IO3: IO3Pin<QUADSPI>,
    {
        // Enable quad SPI in the clocks.
        ahb3.enr().modify(|_, w| w.qspien().bit(true));

        // Disable QUADSPI before configuring it.
        qspi.cr.modify(|_, w| w.en().clear_bit());

        // Clear all pending flags.
        qspi.fcr.write(|w| {
            w.ctof()
                .set_bit()
                .csmf()
                .set_bit()
                .ctcf()
                .set_bit()
                .ctef()
                .set_bit()
        });

        let mut unit = Qspi { qspi, pins, config };
        unit.apply_config(config);
        unit
    }

    pub fn is_busy(&self) -> bool {
        self.qspi.sr.read().busy().bit_is_set()
    }

    pub fn get_config(&self) -> QspiConfig {
        self.config
    }

    pub fn apply_config(&mut self, config: QspiConfig) {
        if self.qspi.sr.read().busy().bit_is_set() {
            //Todo: Handle error
            // return Err(QspiError::Busy);
        }

        self.qspi
            .cr
            .modify(|_, w| unsafe { w.fthres().bits(config.fifo_threshold as u8) });

        while self.qspi.sr.read().busy().bit_is_set() {}

        // modify the prescaler and select flash bank 2 - flash bank 1 is currently unsupported.
        self.qspi.cr.modify(|_, w| unsafe {
            w.prescaler()
                .bits(config.clock_prescaler as u8)
                .sshift()
                .bit(config.sample_shift == SampleShift::HalfACycle)
        });

        //Modify DCR with flash size, CSHT and clock mode
        self.qspi.dcr.modify(|_, w| unsafe {
            w.fsize()
                .bits(config.flash_size as u8)
                .csht()
                .bits(config.chip_select_high_time as u8)
                .ckmode()
                .bit(config.clock_mode == ClockMode::Mode3)
        });

        //Enable SPI
        self.qspi.cr.modify(|_, w| w.en().set_bit());

        self.config = config;
    }

    pub fn transfer(&self, command: QspiReadCommand, buffer: &mut [u8]) {
        if self.is_busy() {
            //Todo handle error
        }
        // Clear the transfer complete flag.
        self.qspi.fcr.modify(|_, w| w.ctcf().set_bit());

        let mut dmode: u8 = 0;
        let mut instruction: u8 = 0;
        let mut imode: u8 = 0;
        let mut admode: u8 = 0;
        let mut adsize: u8 = 0;
        let mut abmode: u8 = 0;
        let mut absize: u8 = 0;

        // Write the length and format of data
        if command.recive_lenght > 0 {
            self.qspi
                .dlr
                .write(|w| unsafe { w.dl().bits(command.recive_lenght as u32 - 1) });
            if self.config.qpi_mode {
                dmode = QspiMode::QuadChannel as u8;
            } else {
                dmode = command.data_mode as u8;
            }
        }

        //Write instruction mode
        if let Some((inst, mode)) = command.instruction {
            if self.config.qpi_mode {
                imode = QspiMode::QuadChannel as u8;
            } else {
                imode = mode as u8;
            }
            instruction = inst;
        }

        // Note Address mode
        if let Some((_, mode)) = command.address {
            if self.config.qpi_mode {
                admode = QspiMode::QuadChannel as u8;
            } else {
                admode = mode as u8;
            }
            adsize = self.config.address_size as u8;
        }

        // Write Alternative bytes
        if let Some((a_bytes, mode)) = command.alternative_bytes {
            if self.config.qpi_mode {
                abmode = QspiMode::QuadChannel as u8;
            } else {
                abmode = mode as u8;
            }

            absize = a_bytes.len() as u8 - 1;

            self.qspi.abr.write(|w| {
                let mut i = 0;
                let mut reg_byte: u32 = 0;
                for element in a_bytes.iter().rev() {
                    reg_byte = reg_byte | ((*element as u32) << i * 8);
                    i += 1;
                }
                unsafe { w.alternate().bits(reg_byte) }
            });
        }

        if command.double_data_rate {
            self.qspi.cr.modify(|_, w| w.sshift().bit(false));
        }

        //Write CCR register with instruction etc.
        self.qspi.ccr.modify(|_, w| unsafe {
            w.fmode()
                .bits(0b01)
                .admode()
                .bits(admode)
                .adsize()
                .bits(adsize)
                .abmode()
                .bits(abmode)
                .absize()
                .bits(absize)
                .ddrm()
                .bit(command.double_data_rate)
                .dcyc()
                .bits(command.dummy_cycles)
                .dmode()
                .bits(dmode)
                .imode()
                .bits(imode)
                .instruction()
                .bits(instruction)
        });

        //Write address, triggers send
        if let Some((addr, _)) = command.address {
            self.qspi.ar.write(|w| unsafe { w.address().bits(addr) });
        }

        //Read data from the buffer
        let mut b = buffer.iter_mut();
        while self.qspi.sr.read().tcf().bit_is_clear() {
            if self.qspi.sr.read().ftf().bit_is_set() {
                if let Some(v) = b.next() {
                    unsafe {
                        *v = ptr::read_volatile(&self.qspi.dr as *const _ as *const u8);
                    }
                } else {
                    // OVERFLOW
                }
            }
        }
        //When transfer complete, empty fifo buffer
        while self.qspi.sr.read().flevel().bits() > 0 {
            if let Some(v) = b.next() {
                unsafe {
                    *v = ptr::read_volatile(&self.qspi.dr as *const _ as *const u8);
                }
            } else {
                // OVERFLOW
            }
        }
        self.qspi.fcr.write(|w| w.ctcf().set_bit());

        if command.double_data_rate {
            self.qspi.cr.modify(|_, w| {
                w.sshift()
                    .bit(self.config.sample_shift == SampleShift::HalfACycle)
            });
        }
    }

    pub fn write(&self, command: QspiWriteCommand) {
        if self.is_busy() {
            //Todo handle error
        }
        // Clear the transfer complete flag.
        self.qspi.fcr.modify(|_, w| w.ctcf().set_bit());

        let mut dmode: u8 = 0;
        let mut instruction: u8 = 0;
        let mut imode: u8 = 0;
        let mut admode: u8 = 0;
        let mut adsize: u8 = 0;
        let mut abmode: u8 = 0;
        let mut absize: u8 = 0;

        // Write the length and format of data
        if let Some((data, mode)) = command.data {
            self.qspi
                .dlr
                .write(|w| unsafe { w.dl().bits(data.len() as u32 - 1) });
            if self.config.qpi_mode {
                dmode = QspiMode::QuadChannel as u8;
            } else {
                dmode = mode as u8;
            }
        }

        //Write instruction mode
        if let Some((inst, mode)) = command.instruction {
            if self.config.qpi_mode {
                imode = QspiMode::QuadChannel as u8;
            } else {
                imode = mode as u8;
            }
            instruction = inst;
        }

        // Note Address mode
        if let Some((_, mode)) = command.address {
            if self.config.qpi_mode {
                admode = QspiMode::QuadChannel as u8;
            } else {
                admode = mode as u8;
            }
            adsize = self.config.address_size as u8;
        }

        // Write Alternative bytes
        if let Some((a_bytes, mode)) = command.alternative_bytes {
            if self.config.qpi_mode {
                abmode = QspiMode::QuadChannel as u8;
            } else {
                abmode = mode as u8;
            }

            absize = a_bytes.len() as u8 - 1;

            self.qspi.abr.write(|w| {
                let mut i = 0;
                let mut reg_byte: u32 = 0;
                for element in a_bytes.iter().rev() {
                    reg_byte = reg_byte | ((*element as u32) << i * 8);
                    i += 1;
                }
                unsafe { w.alternate().bits(reg_byte) }
            });
        }

        if command.double_data_rate {
            self.qspi.cr.modify(|_, w| w.sshift().bit(false));
        }

        //Write CCR register with instruction etc.
        self.qspi.ccr.modify(|_, w| unsafe {
            w.fmode()
                .bits(0b00)
                .admode()
                .bits(admode)
                .adsize()
                .bits(adsize)
                .abmode()
                .bits(abmode)
                .absize()
                .bits(absize)
                .ddrm()
                .bit(command.double_data_rate)
                .dcyc()
                .bits(command.dummy_cycles)
                .dmode()
                .bits(dmode)
                .imode()
                .bits(imode)
                .instruction()
                .bits(instruction)
        });

        //Write address, triggers send
        if let Some((addr, _)) = command.address {
            self.qspi.ar.write(|w| unsafe { w.address().bits(addr) });
        }

        //Write data to the FIFO
        if let Some((data, _)) = command.data {
            for byte in data {
                while self.qspi.sr.read().ftf().bit_is_clear() {}
                unsafe {
                    ptr::write_volatile(&self.qspi.dr as *const _ as *mut u8, *byte);
                }
            }
        }

        while self.qspi.sr.read().tcf().bit_is_clear() {}

        self.qspi.fcr.write(|w| w.ctcf().set_bit());

        if command.double_data_rate {
            self.qspi.cr.modify(|_, w| {
                w.sshift()
                    .bit(self.config.sample_shift == SampleShift::HalfACycle)
            });
        }
    }
}
