//! Quad Serial Peripheral Interface (QSPI) bus

use crate::stm32::QUADSPI;
use crate::rcc::AHB3;
use core::ptr;


#[derive(Copy, Clone, Debug, PartialEq)]
#[repr(u8)]
pub enum QspiMode{
    SingleChannel = 0b01,
    DualChannel = 0b10,
    QuadChannel = 0b11,
}

#[derive(Copy, Clone, Debug, PartialEq)]
#[repr(u8)]
pub enum AddressSize{
    Addr8Bit = 0b00,
    Addr16Bit = 0b01,
    Addr24Bit = 0b10,
    Addr32Bit = 0b11,
}

#[derive(Copy, Clone, Debug, PartialEq)]
// #[repr(bool)]
pub enum SampleShift{
    None,
    HalfACycle,
}

#[derive(Copy, Clone, Debug, PartialEq)]
pub enum ClockMode {
    Mode0,
    Mode3,
}

#[derive(Copy, Clone, Debug, PartialEq)]
pub struct QspiConfig{
    /// This field defines the scaler factor for generating CLK based on the AHB clock
    /// (value+1).
    clock_prescaler : u8,
    /// Number of bytes in Flash memory = 2^[FSIZE+1]
    flash_size : u8,
    address_size : AddressSize,
    /// This bit indicates the level that CLK takes between commands Mode 0(low) / mode 3(high)
    clock_mode : ClockMode,
    /// FIFO threshold level (Activates FTF, QUADSPI_SR[2]) 0-15.
    fifo_threshold : u8,
    /// Single, dual og quad mode
    qspi_mode : QspiMode,
    sample_shift : SampleShift,
    /// CSHT+1 defines the minimum number of CLK cycles which the chip select (nCS) must
    /// remain high between commands issued to the Flash memory.
    chip_select_high_time: u8,
    double_data_rate : bool,
    qpi_mode : bool,
}

impl Default for QspiConfig{
    fn default() -> QspiConfig {
        QspiConfig {
            clock_prescaler : 0,
            flash_size : 22, //8MB //26 = 128MB
            address_size : AddressSize::Addr24Bit,
            clock_mode : ClockMode::Mode0,
            fifo_threshold : 1,
            qspi_mode : QspiMode::QuadChannel,
            sample_shift : SampleShift::None,
            chip_select_high_time : 0,
            double_data_rate : false,
            qpi_mode : false,
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

    pub fn qspi_mode(mut self, qspi: QspiMode) -> Self {
        self.qspi_mode = qspi;
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

    pub fn double_data_rate(mut self, ddr: bool) -> Self {
        self.double_data_rate = ddr;
        self 
    }

    pub fn qpi_mode(mut self, qpi: bool) -> Self {
        self.qpi_mode = qpi;
        self 
    }
}


pub struct QspiWriteCommand<'c>{
    pub instruction : Option<u8>,
    pub address : Option<u32>,
    pub alternative_bytes : Option<&'c[u8]>,
    pub dummy_cycles : u8,
    pub data: Option<&'c[u8]>,
}

pub struct QspiReadCommand<'c>{
    pub instruction : Option<u8>,
    pub address : Option<u32>,
    pub alternative_bytes : Option<&'c[u8]>,
    pub dummy_cycles : u8,
    pub recive_lenght : u32,
}


pub struct Qspi {
    qspi: QUADSPI,
    config: QspiConfig,
}

impl Qspi {
    pub fn new(qspi: QUADSPI, ahb3 : &mut AHB3, config : QspiConfig) -> Self {
        // Enable quad SPI in the clocks.
        ahb3.enr().modify(|_,w| w.qspien().bit(true));

        // Disable QUADSPI before configuring it.
        qspi.cr.write(|w| {
            w.en().clear_bit()
        });

        // Clear all pending flags.
        qspi.fcr.write(|w| {
            w
            .ctof().set_bit()
            .csmf().set_bit()
            .ctcf().set_bit()
            .ctef().set_bit()
        });

        let mut unit = Qspi{qspi, config};
        unit.apply_config(config);
        unit
    }

    pub fn is_busy(&self) -> bool {
        self.qspi.sr.read().busy().bit_is_set()
    }

    pub fn apply_config(&mut self, config : QspiConfig) {
        if self.qspi.sr.read().busy().bit_is_set() {
            //Todo: Handle error
            // return Err(QspiError::Busy);
        }

        // modify the prescaler and select flash bank 2 - flash bank 1 is currently unsupported.
        self.qspi.cr.write(|w| unsafe {
            w.prescaler().bits(config.clock_prescaler as u8)
                // .fsel().set_bit()
                .fthres().bits(config.fifo_threshold as u8)
                .sshift().bit(config.sample_shift == SampleShift::HalfACycle)
        });

        self.qspi.dcr.write(|w| unsafe{
            w.fsize().bits(config.flash_size as u8)
                .csht().bits(config.chip_select_high_time as u8)
                .ckmode().bit(config.clock_mode == ClockMode::Mode3)
        });

        /////////// TODO: find solution to "natural state" for now no natural state
        self.qspi.ccr.write(|w| unsafe{
            w.imode().bits(if config.qpi_mode {0b11} else {0b01})
                .admode().bits(config.qspi_mode as u8)
                .dmode().bits(config.qspi_mode as u8)
                .abmode().bits(config.qspi_mode as u8)
        });
        //////////
        self.config = config;
    }

    pub fn transfer(& self, command : QspiReadCommand, buffer : &mut [u8]) {
        if self.is_busy() {
            //Todo handle error
            // return Err(QspiError::Busy);
        }
        // Clear the transfer complete flag.
        self.qspi.fcr.modify(|_ ,w| w.ctcf().set_bit());

        // Write the length and format of data  
        if command.recive_lenght > 0 {
            self.qspi.dlr.write(|w| unsafe {w.dl().bits(command.recive_lenght as u32 - 1)});
            self.qspi.ccr.write(|w| unsafe {w.dmode().bits(self.config.qspi_mode as u8)});
        } else {
            self.qspi.ccr.write(|w| unsafe {w.dmode().bits(0b00)});
        }
        
        //Instruction mode
        if let Some(_) = command.instruction {
            if self.config.qpi_mode {
                self.qspi.ccr.write(|w| unsafe {w.imode().bits(self.config.qspi_mode as u8)});
            } else {
                self.qspi.ccr.write(|w| unsafe {w.imode().bits(0b01)});
            }
        } else {
            self.qspi.ccr.write(|w| unsafe {w.imode().bits(0b00)});
        }
        
        //Address mode
        if let Some(_) = command.address {
            self.qspi.ccr.write(|w| unsafe {w.admode().bits(self.config.qspi_mode as u8)});
        } else {
            self.qspi.ccr.write(|w| unsafe {w.admode().bits(0b00)});
        }

        //Number of dummycycles
        self.qspi.ccr.write(|w| unsafe {w.dcyc().bits(command.dummy_cycles)});
        
        //Write Alternative bytes
        if let Some(a_bytes) = command.alternative_bytes {
            self.qspi.ccr.write(|w| unsafe {
                w.abmode().bits(self.config.qspi_mode as u8)
                    .absize().bits(a_bytes.len() as u8 - 1)
            });
            self.qspi.abr.write(|w| {
                let mut i = 0;
                let mut reg_byte: u32 = 0;
                for element in a_bytes.iter().rev(){
                    reg_byte = reg_byte | ((*element as u32) << i*8);
                    i += 1;
                }
                unsafe {
                    w.alternate().bits(reg_byte)
                }
            });
        } else {
            self.qspi.ccr.write(|w| unsafe {w.abmode().bits(0b00)});
        }

        //Enable functional mode indirect read
        self.qspi.ccr.write(|w| unsafe {w.fmode().bits(0b01)});
        
        //Enable QSPI
        self.qspi.cr.write(|w| w.en().bit(true));

        //Write instruction, triggers send if no address required
        if let Some(inst) = command.instruction {
            self.qspi.ccr.write(|w| unsafe {w.instruction().bits(inst)});
        }

        //Write address, triggers send
        if let Some(addr) = command.address {
            self.qspi.ar.write(|w| unsafe {w.address().bits(addr)});
        }

        //Read data from the buffer
        let mut b = buffer.iter_mut();
        while self.qspi.sr.read().tcf().bit_is_clear() {
            if self.qspi.sr.read().ftf().bit_is_set(){
                if let Some(v) = b.next() {
                    unsafe{
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
                unsafe{
                    *v = ptr::read_volatile(&self.qspi.dr as *const _ as *const u8);
                }
            } else {
                // OVERFLOW
            }
        }

        unsafe {
            for location in buffer {
                *location = ptr::read_volatile(&self.qspi.dr as *const _ as *const u8);
            }
        }

        self.qspi.fcr.write(|w| w.ctcf().set_bit());        
    }
/*
    pub fn write(&mut self, addr: u8, data: &[u8]) -> Result<(), QspiError> {
        if self.is_busy() {
            return Err(QspiError::Busy);
        }

        // Clear the transfer complete flag.
        self.qspi.fcr.modify(|_ ,w| w.ctcf().set_bit());

        // Write the length
        self.qspi.dlr.write(|w| unsafe {w.dl().bits(data.len() as u32 - 1)});

        // Configure the mode to indirect write and configure the instruction byte.
        self.rb.ccr.write(|w| unsafe {
            w.fmode().bits(0b00)
             .instruction().bits(addr)
        });

        // Enable the transaction
        self.rb.cr.write(|w| {w.en().set_bit()});

        // Write data to the FIFO in a byte-wise manner.
        unsafe {
            for byte in data {
                ptr::write_volatile(&self.rb.dr as *const _ as *mut u8, *byte);
            }
        }

        // Wait for the transaction to complete
        while self.rb.sr.read().tcf().bit_is_clear() {}

        // Check that there is no more transaction pending.
        if self.is_busy() {
            return Err(QspiError::FifoData);
        }

        self.rb.cr.write(|w| {w.en().clear_bit()});

        // Clear the transfer complete flag.
        self.rb.fcr.write(|w| w.ctcf().set_bit());

        Ok(())
    }

    pub fn read(&mut self, addr: u8, dest: &mut [u8]) -> Result<(), QspiError> {
        if self.is_busy() {
            return Err(QspiError::Busy);
        }

        // Clear the transfer complete flag.
        self.rb.fcr.modify(|_ ,w| w.ctcf().set_bit());

        // Write the length that should be read.
        self.rb.dlr.write(|w| unsafe {
            w.dl().bits(dest.len() as u32 - 1)
        });

        // Configure the mode to indirect read and configure the instruction byte.
        self.rb.ccr.modify(|_, w| unsafe {
            w.fmode().bits(0b01)
             .instruction().bits(addr)
        });

        // Enable the transaction
        self.rb.cr.modify(|_, w| {w.en().set_bit()});

        // Write the instruction bits to force the read to start. This has to be done after the
        // transaction is enabled to indicate to the peripheral that we are ready to start the
        // transaction, even though these bits should already be set.
        self.rb.ccr.modify(|_, w| unsafe {
            w.instruction().bits(addr)
        });

        // Wait for the transaction to complete
        while self.rb.sr.read().tcf().bit_is_clear() {}

        // Check for underflow on the FIFO.
        if (self.rb.sr.read().flevel().bits() as usize) < dest.len() {
            return Err(QspiError::Underflow);
        }

        // Read data from the FIFO in a byte-wise manner.
        unsafe {
            for location in dest {
                *location = ptr::read_volatile(&self.rb.dr as *const _ as *const u8);
            }
        }

        // Check that there is no more transaction pending.
        if self.is_busy() {
            return Err(QspiError::FifoData);
        }

        self.rb.cr.modify(|_, w| {w.en().clear_bit()});

        // Clear the transfer complete flag.
        self.rb.fcr.modify(|_ ,w| w.ctcf().set_bit());

        Ok(())
    }
*/
    
}