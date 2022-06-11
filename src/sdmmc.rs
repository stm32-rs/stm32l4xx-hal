#![cfg(feature = "sdmmc")]

//! SDMMC peripheral abstraction

use core::{
    fmt,
    ops::{ControlFlow, Deref, DerefMut},
    ptr, slice,
    sync::atomic::{self, Ordering},
};

use fugit::HertzU32 as Hertz;
use sdio_host::{
    common_cmd::{self, ResponseLen},
    sd::{
        BusWidth, CardCapacity, CardStatus, CurrentState, SDStatus, CIC, CID, CSD, OCR, RCA, SCR,
        SD,
    },
    sd_cmd, Cmd,
};

use crate::{
    dma::dma2,
    dmamux::{DmaInput, DmaMux},
    gpio::{self, Alternate, PushPull},
    pac::{sdmmc1, SDMMC1},
    rcc::{Clocks, Enable, Reset, APB2},
};
pub trait PinClk {}
pub trait PinCmd {}
pub trait PinD0 {}
pub trait PinD1 {}
pub trait PinD2 {}
pub trait PinD3 {}
pub trait PinD4 {}
pub trait PinD5 {}
pub trait PinD6 {}
pub trait PinD7 {}

pub trait Pins {
    const BUS_WIDTH: BusWidth;
}

impl<CLK, CMD, D0> Pins for (CLK, CMD, D0)
where
    CLK: PinClk,
    CMD: PinCmd,
    D0: PinD0,
{
    const BUS_WIDTH: BusWidth = BusWidth::One;
}

impl<CLK, CMD, D0, D1, D2, D3> Pins for (CLK, CMD, D0, D1, D2, D3)
where
    CLK: PinClk,
    CMD: PinCmd,
    D0: PinD0,
    D1: PinD1,
    D2: PinD2,
    D3: PinD3,
{
    const BUS_WIDTH: BusWidth = BusWidth::Four;
}

impl<CLK, CMD, D0, D1, D2, D3, D4, D5, D6, D7> Pins for (CLK, CMD, D0, D1, D2, D3, D4, D5, D6, D7)
where
    CLK: PinClk,
    CMD: PinCmd,
    D0: PinD0,
    D1: PinD1,
    D2: PinD2,
    D3: PinD3,
    D4: PinD4,
    D5: PinD5,
    D6: PinD6,
    D7: PinD7,
{
    const BUS_WIDTH: BusWidth = BusWidth::Eight;
}

macro_rules! pins {
    ($(CLK: [$($CLK:ty),*] CMD: [$($CMD:ty),*] D0: [$($D0:ty),*] D1: [$($D1:ty),*] D2: [$($D2:ty),*] D3: [$($D3:ty),*] D4: [$($D4:ty),*] D5: [$($D5:ty),*] D6: [$($D6:ty),*] D7: [$($D7:ty),*])+) => {
        $(
            $(
                impl PinClk for $CLK {}
            )*
            $(
                impl PinCmd for $CMD {}
            )*
            $(
                impl PinD0 for $D0 {}
            )*
            $(
                impl PinD1 for $D1 {}
            )*
            $(
                impl PinD2 for $D2 {}
            )*
            $(
                impl PinD3 for $D3 {}
            )*
            $(
                impl PinD4 for $D4 {}
            )*
            $(
                impl PinD5 for $D5 {}
            )*
            $(
                impl PinD6 for $D6 {}
            )*
            $(
                impl PinD7 for $D7 {}
            )*
        )+
    }
}

#[cfg(any(
    feature = "stm32l476",
    feature = "stm32l486",
    feature = "stm32l496",
    feature = "stm32l4a6"
))]
pins! {
  CLK: [gpio::PC12<Alternate<PushPull, 12>>]
  CMD: [gpio::PD2<Alternate<PushPull, 12>>]
  D0: [gpio::PC8<Alternate<PushPull, 12>>]
  D1: [gpio::PC9<Alternate<PushPull, 12>>]
  D2: [gpio::PC10<Alternate<PushPull, 12>>]
  D3: [gpio::PC11<Alternate<PushPull, 12>>]
  D4: [gpio::PB8<Alternate<PushPull, 12>>]
  D5: [gpio::PB9<Alternate<PushPull, 12>>]
  D6: [gpio::PC6<Alternate<PushPull, 12>>]
  D7: [gpio::PC7<Alternate<PushPull, 12>>]
}

/// SDMMC clock frequency.
#[derive(Debug, Copy, Clone, Eq, PartialEq)]
#[repr(u8)]
pub enum ClockFreq {
    /// 24 MHz
    Freq24MHz = 0,
    /// 12 MHz
    Freq12MHz = 2,
    /// 8 MHz
    Freq8MHz = 4,
    /// 4 MHz
    Freq4MHz = 10,
    /// 1 MHz
    Freq1MHz = 46,
    /// 400 kHz
    Freq400KHz = 118,
}

/// Error during SDMMC operations.
#[derive(Debug, Clone, Copy)]
pub enum Error {
    /// No card detected.
    NoCard,
    /// Command CRC check failed.
    CommandCrc,
    /// Data block CRC check failed.
    DataCrc,
    /// Timeout in hardware.
    Timeout,
    /// Timeout in software.
    SoftwareTimeout(&'static str),
    /// Receive FIFO overrun.
    RxOverrun,
    /// Transmit FIFO underrun.
    TxUnderrun,
    /// Invalid input.
    InvalidInput,
}

impl fmt::Display for Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::NoCard => "no card",
            Self::CommandCrc => "command CRC check failed",
            Self::DataCrc => "data block CRC check failed",
            Self::Timeout => "timeout",
            Self::SoftwareTimeout(op) => return write!(f, "software timeout during {}", op),
            Self::RxOverrun => "receive FIFO overrun",
            Self::TxUnderrun => "transmit FIFO underrun",
            Self::InvalidInput => "invalid input",
        }
        .fmt(f)
    }
}

macro_rules! sta_rx_tx_err {
    ($sta:expr, $sdmmc:expr) => {
        if $sta.dcrcfail().bit() {
            clear_static_data_flags(&$sdmmc.icr);

            Err(Error::DataCrc)
        } else if $sta.dtimeout().bit() {
            clear_static_data_flags(&$sdmmc.icr);

            Err(Error::Timeout)
        } else {
            Ok($sta)
        }
    };
}

macro_rules! sta_rx {
    ($sdmmc:expr) => {{
        let sta = $sdmmc.sta.read();

        if sta.rxoverr().bit() {
            clear_static_data_flags(&$sdmmc.icr);

            Err(Error::RxOverrun)
        } else {
            sta_rx_tx_err!(sta, $sdmmc)
        }
    }};
}

macro_rules! sta_tx {
    ($sdmmc:expr) => {{
        let sta = $sdmmc.sta.read();

        if sta.txunderr().bit() {
            clear_static_data_flags(&$sdmmc.icr);

            Err(Error::TxUnderrun)
        } else {
            sta_rx_tx_err!(sta, $sdmmc)
        }
    }};
}

#[inline]
fn clear_static_command_flags(icr: &sdmmc1::ICR) {
    icr.write(|w| {
        w.cmdsentc()
            .set_bit()
            .cmdrendc()
            .set_bit()
            .ctimeoutc()
            .set_bit()
            .ccrcfailc()
            .set_bit()
    })
}

#[inline]
fn clear_static_data_flags(icr: &sdmmc1::ICR) {
    icr.write(|w| {
        w.dbckendc()
            .set_bit()
            .dataendc()
            .set_bit()
            .rxoverrc()
            .set_bit()
            .txunderrc()
            .set_bit()
            .dtimeoutc()
            .set_bit()
            .dcrcfailc()
            .set_bit()
    })
}

#[inline]
fn clear_all_interrupts(icr: &sdmmc1::ICR) {
    clear_static_command_flags(icr);
    clear_static_data_flags(icr);

    icr.modify(|_, w| w.sdioitc().set_bit());
}

/// An initialized SD card.
#[derive(Default)]
pub struct SdCard {
    ocr: OCR<SD>,
    cid: CID<SD>,
    rca: RCA<SD>,
    csd: CSD<SD>,
    scr: SCR,
}

// TODO:  Remove when https://github.com/jkristell/sdio-host/issues/11 is fixed.
impl fmt::Debug for SdCard {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("SdCard")
            .field("cid", &self.cid)
            .field("csd", &self.csd)
            .field("ocr", &self.ocr)
            .field("rca", &self.rca.address())
            .field("scr", &self.scr)
            .finish()
    }
}

impl SdCard {
    /// Get the card's address.
    pub fn address(&self) -> u16 {
        self.rca.address()
    }

    pub fn block_count(&self) -> u64 {
        self.csd.block_count()
    }

    /// Get the card size in bytes.
    pub fn size(&self) -> u64 {
        self.csd.card_size()
    }

    /// Get the card's capacity type.
    pub fn capacity(&self) -> CardCapacity {
        if self.ocr.high_capacity() {
            CardCapacity::HighCapacity
        } else {
            CardCapacity::StandardCapacity
        }
    }
}

#[derive(Debug, Clone, Copy)]
/// Indicates transfer direction
enum Dir {
    HostToCard = 0,
    CardToHost = 1,
}

/// SD card version.
#[derive(Debug, Clone, Copy, PartialEq)]
enum SdCardVersion {
    V1,
    V2,
}

/// An SDMMC controller.
#[derive(Debug)]
pub struct Sdmmc {
    sdmmc: SDMMC1,
    clock: Hertz,
    bus_width: BusWidth,
    card: Option<SdCard>,
}

impl Sdmmc {
    pub fn new<PINS: Pins>(sdmmc: SDMMC1, _pins: PINS, apb2: &mut APB2, clocks: &Clocks) -> Self {
        SDMMC1::enable(apb2);
        SDMMC1::reset(apb2);

        let clock = clocks.sysclk();

        sdmmc.clkcr.modify(|_, w| unsafe {
            w.negedge()
                .clear_bit() // Rising Edge
                .bypass()
                .clear_bit() // Disable bypass.
                .pwrsav()
                .clear_bit() // Disable power-save.
                .widbus()
                .bits(0) // Bus Width 1
                .hwfc_en()
                .clear_bit() // Disable hardware flow-control.
                .clkdiv()
                .bits(ClockFreq::Freq400KHz as u8) // Clock must be <= 400 kHz while in identification mode.
                .clken()
                .clear_bit() // Disable clock.
        });

        let mut host = Self {
            sdmmc,
            clock,
            bus_width: PINS::BUS_WIDTH,
            card: None,
        };

        host.power_card(false);

        host
    }

    #[inline]
    pub fn init(&mut self, freq: ClockFreq) -> Result<(), Error> {
        self.power_card(true);

        // Enable clock.
        self.sdmmc.clkcr.modify(|_, w| w.clken().set_bit());

        self.cmd(common_cmd::idle())?;

        let check_pattern = 0xaa;
        self.cmd(sd_cmd::send_if_cond(1, check_pattern))?;
        let cic = CIC::from(self.sdmmc.resp1.read().bits().to_le());

        let card_version = if cic.pattern() == 0xAA {
            SdCardVersion::V2
        } else {
            SdCardVersion::V1
        };

        let mut card = SdCard::default();

        let high_capacity = card_version == SdCardVersion::V2;
        let voltage_window = 1 << 5;

        let mut timeout = 0xffff;
        card.ocr = loop {
            if timeout == 0 {
                return Err(Error::SoftwareTimeout("init"));
            }

            timeout -= 1;

            match self.app_cmd(sd_cmd::sd_send_op_cond(
                high_capacity,
                false,
                false,
                voltage_window,
            )) {
                Ok(()) => (),
                Err(Error::CommandCrc) => (),
                Err(err) => return Err(err),
            }

            let ocr = OCR::from(self.sdmmc.resp1.read().bits().to_le());
            if !ocr.is_busy() {
                // Power up done
                break ocr;
            }
        };

        self.cmd(common_cmd::all_send_cid())?;
        card.cid = CID::from([
            self.sdmmc.resp4.read().bits().to_le(),
            self.sdmmc.resp3.read().bits().to_le(),
            self.sdmmc.resp2.read().bits().to_le(),
            self.sdmmc.resp1.read().bits().to_le(),
        ]);

        self.cmd(sd_cmd::send_relative_address())?;
        card.rca = RCA::from(self.sdmmc.resp1.read().bits().to_le());

        self.cmd(common_cmd::send_csd(card.rca.address()))?;
        card.csd = CSD::from([
            self.sdmmc.resp4.read().bits().to_le(),
            self.sdmmc.resp3.read().bits().to_le(),
            self.sdmmc.resp2.read().bits().to_le(),
            self.sdmmc.resp1.read().bits().to_le(),
        ]);

        self.select_card(card.rca.address())?;
        card.scr = self.get_scr(card.rca.address())?;

        //  `app_cmd` will select this card from now on.
        self.card = Some(card);

        // Wait before setting the bus witdth and frequency to avoid timeouts on SDSC cards
        while !self.card_ready()? {}

        self.set_bus(self.bus_width, freq)?;

        Ok(())
    }

    #[inline]
    pub fn read_block(&mut self, addr: u32, block: &mut DataBlock) -> Result<(), Error> {
        self.read_blocks(addr, slice::from_mut(block))
    }

    pub(crate) fn start_read_transfer(
        &mut self,
        addr: u32,
        words: &mut [u32],
        dma: bool,
    ) -> Result<bool, Error> {
        let card = self.card()?;

        let addr = match card.capacity() {
            CardCapacity::StandardCapacity => addr * 512,
            _ => addr,
        };

        self.cmd(common_cmd::set_block_length(512))?;

        let byte_count = words.len() * 4;
        self.start_datapath_transfer(byte_count as u32, 9, Dir::CardToHost, dma);

        let block_count = words.len() / 128;
        match block_count {
            0 => return Ok(false),
            1 => self.cmd(common_cmd::read_single_block(addr))?,
            _ => self.cmd(common_cmd::read_multiple_blocks(addr))?,
        }

        Ok(block_count > 1)
    }

    #[inline]
    pub fn read_blocks(&mut self, addr: u32, blocks: &mut [DataBlock]) -> Result<(), Error> {
        let words = DataBlock::blocks_to_words_mut(blocks);
        let mut word_count = words.len();

        let stop_transmission = self.start_read_transfer(addr, words, false)?;
        let mut it = words.into_iter();

        let timeout: u32 = 0xffff_ffff;
        for _ in 0..timeout {
            let sta = sta_rx!(self.sdmmc)?;

            // If we received all data, wait for transfer to end.
            if word_count == 0 {
                if sta.dbckend().bit() {
                    clear_static_data_flags(&self.sdmmc.icr);

                    if stop_transmission {
                        self.cmd(common_cmd::stop_transmission())?;
                    }

                    return Ok(());
                }
            }
            // If the FIFO is half-full, receive some data.
            else if sta.rxfifohf().bit() {
                for _ in 0..8 {
                    unsafe {
                        let word = it.next().unwrap_unchecked();
                        *word = self.sdmmc.fifo.read().bits();
                    }
                }

                word_count -= 8;
            }
        }

        clear_static_data_flags(&self.sdmmc.icr);
        Err(Error::SoftwareTimeout("read_blocks"))
    }

    #[inline]
    pub(crate) fn start_write_transfer(
        &mut self,
        addr: u32,
        words: &[u32],
        dma: bool,
    ) -> Result<bool, Error> {
        let card = self.card()?;

        let addr = match card.capacity() {
            CardCapacity::StandardCapacity => addr * 512,
            _ => addr,
        };

        let byte_count = words.len() * 4;
        self.cmd(common_cmd::set_block_length(512))?;
        self.start_datapath_transfer(byte_count as u32, 9, Dir::HostToCard, dma);

        let block_count = words.len() / 128;
        match block_count {
            0 => (),
            1 => self.cmd(common_cmd::write_single_block(addr))?,
            _ => self.cmd(common_cmd::write_multiple_blocks(addr))?,
        }

        Ok(block_count > 1)
    }

    #[inline]
    pub fn write_block(&mut self, addr: u32, block: &DataBlock) -> Result<(), Error> {
        self.write_blocks(addr, slice::from_ref(block))
    }

    #[inline]
    fn write_blocks(&mut self, addr: u32, blocks: &[DataBlock]) -> Result<(), Error> {
        let words = DataBlock::blocks_to_words(blocks);
        let mut word_count = words.len();

        let stop_transmission = self.start_write_transfer(addr, words, false)?;
        let mut it = words.into_iter();

        loop {
            let sta = sta_tx!(self.sdmmc)?;

            // If we sent all data, wait for transfer to end.
            if word_count == 0 {
                if sta.dbckend().bit() {
                    clear_static_data_flags(&self.sdmmc.icr);

                    if stop_transmission {
                        self.cmd(common_cmd::stop_transmission())?;
                    }

                    break;
                }
            }
            // If the FIFO is half-empty, send some data.
            else if sta.txfifohe().bit() {
                for _ in 0..8 {
                    self.sdmmc.fifo.write(|w| unsafe {
                        let word = it.next().unwrap_unchecked();
                        w.bits(*word)
                    });
                }

                word_count -= 8;
            }
        }

        clear_static_data_flags(&self.sdmmc.icr);

        let timeout: u32 = 0xffff_ffff;
        for _ in 0..timeout {
            if self.card_ready()? {
                return Ok(());
            }
        }

        Err(Error::SoftwareTimeout("write_blocks"))
    }

    #[inline]
    fn set_bus(&self, bus_width: BusWidth, freq: ClockFreq) -> Result<(), Error> {
        let card = self.card()?;

        let bus_width = match bus_width {
            BusWidth::Eight | BusWidth::Four if card.scr.bus_width_four() => BusWidth::Four,
            _ => BusWidth::One,
        };

        self.app_cmd(sd_cmd::set_bus_width(bus_width == BusWidth::Four))?;

        self.sdmmc.clkcr.modify(|_, w| unsafe {
            w.clkdiv()
                .bits(freq as u8)
                .widbus()
                .bits(match bus_width {
                    BusWidth::One => 0,
                    BusWidth::Four => 1,
                    BusWidth::Eight => 2,
                    _ => unimplemented!(),
                })
                .clken()
                .set_bit()
        });
        Ok(())
    }

    fn start_datapath_transfer(
        &self,
        length_bytes: u32,
        block_size: u8,
        direction: Dir,
        dma: bool,
    ) {
        // Block size up to 2^14 bytes
        assert!(block_size <= 14);

        loop {
            let sta = self.sdmmc.sta.read();

            if sta.cmdact().bit_is_clear()
                && sta.rxact().bit_is_clear()
                && sta.txact().bit_is_clear()
            {
                break;
            }
        }

        self.sdmmc
            .dtimer
            .modify(|_, w| unsafe { w.datatime().bits(0xFFFFFFFF) });

        self.sdmmc
            .dlen
            .modify(|_, w| unsafe { w.datalength().bits(length_bytes) });

        self.sdmmc.dctrl.modify(|_, w| unsafe {
            w.dblocksize()
                .bits(block_size)
                .dtdir()
                .bit(match direction {
                    Dir::HostToCard => false,
                    Dir::CardToHost => true,
                })
                .dtmode()
                .clear_bit()
                .dten()
                .set_bit()
                .dmaen()
                .bit(dma)
        });
    }

    fn get_scr(&self, rca: u16) -> Result<SCR, Error> {
        self.cmd(common_cmd::set_block_length(8))?;
        self.cmd(common_cmd::app_cmd(rca))?;
        self.start_datapath_transfer(8, 3, Dir::CardToHost, false);
        self.cmd(sd_cmd::send_scr())?;

        let mut scr = [0; 2];

        let mut i = scr.len();

        let timeout: u32 = 0xffff_ffff;
        for _ in 0..timeout {
            let sta = sta_rx!(self.sdmmc)?;

            // If we received all data, wait for transfer to end.
            if i == 0 {
                if sta.dbckend().bit() {
                    clear_static_data_flags(&self.sdmmc.icr);

                    return Ok(SCR::from(scr));
                }
            } else if sta.rxdavl().bit_is_set() {
                i -= 1;

                let bits = u32::from_be(self.sdmmc.fifo.read().bits());
                scr[i] = bits.to_le();

                continue;
            }
        }

        return Err(Error::SoftwareTimeout("get_scr"));
    }

    fn power_card(&mut self, on: bool) {
        self.sdmmc
            .power
            .modify(|_, w| unsafe { w.pwrctrl().bits(if on { 0b11 } else { 0b00 }) });

        // Wait for 2 ms after power mode change.
        cortex_m::asm::delay(2 * (self.clock.raw() / 1000));
    }

    fn read_status(&mut self) -> Result<CardStatus<SdCard>, Error> {
        let card = self.card()?;

        self.cmd(common_cmd::card_status(card.address(), false))?;

        Ok(CardStatus::from(self.sdmmc.resp1.read().bits().to_le()))
    }

    /// Check if card is done writing/reading and back in transfer state.
    fn card_ready(&mut self) -> Result<bool, Error> {
        Ok(self.read_status()?.state() == CurrentState::Transfer)
    }

    /// Read the [`SDStatus`](sdio_host::sd::SDStatus).
    pub fn read_sd_status(&mut self) -> Result<SDStatus, Error> {
        self.card()?;
        self.cmd(common_cmd::set_block_length(64))?;
        self.start_datapath_transfer(64, 6, Dir::CardToHost, false);
        self.app_cmd(sd_cmd::sd_status())?;

        let mut sd_status = [0u32; 16];

        let mut i = sd_status.len();
        let timeout: u32 = 0xffff_ffff;
        for _ in 0..timeout {
            let sta = sta_rx!(self.sdmmc)?;

            // If we received all data, wait for transfer to end.
            if i == 0 {
                if sta.dbckend().bit() {
                    clear_static_data_flags(&self.sdmmc.icr);

                    return Ok(SDStatus::from(sd_status));
                }
            } else if sta.rxfifohf().bit() {
                for _ in 0..8 {
                    i -= 1;
                    let bits = u32::from_be(self.sdmmc.fifo.read().bits());
                    unsafe {
                        *sd_status.get_unchecked_mut(i) = bits.to_le();
                    }
                }
            }
        }

        Err(Error::SoftwareTimeout("read_sd_status"))
    }

    /// Get the initialized card, if present.
    #[inline]
    pub fn card(&self) -> Result<&SdCard, Error> {
        self.card.as_ref().ok_or(Error::NoCard)
    }

    /// Select the card with the given address.
    #[inline]
    fn select_card(&self, rca: u16) -> Result<(), Error> {
        let r = self.cmd(common_cmd::select_card(rca));
        match (r, rca) {
            (Err(Error::Timeout), 0) => Ok(()),
            (r, _) => r,
        }
    }

    /// Send an app command to the card.
    #[inline]
    fn app_cmd<R: common_cmd::Resp>(&self, cmd: Cmd<R>) -> Result<(), Error> {
        let rca = self.card().map(|card| card.address()).unwrap_or(0);
        self.cmd(common_cmd::app_cmd(rca))?;
        self.cmd(cmd)
    }

    /// Send a command to the card.
    fn cmd<R: common_cmd::Resp>(&self, cmd: Cmd<R>) -> Result<(), Error> {
        while self.sdmmc.sta.read().cmdact().bit_is_set() {}

        self.sdmmc
            .arg
            .write(|w| unsafe { w.cmdarg().bits(cmd.arg) });

        let waitresp = match cmd.response_len() {
            ResponseLen::Zero => 0b00,
            ResponseLen::R48 => 0b01,
            ResponseLen::R136 => 0b11,
        };

        self.sdmmc.cmd.write(|w| unsafe {
            w.cmdindex()
                .bits(cmd.cmd)
                .waitresp()
                .bits(waitresp)
                .waitint()
                .clear_bit()
                .cpsmen()
                .set_bit()
        });

        let timeout = 5000 * (self.clock.raw() / 8 / 1000);
        let mut res = Err(Error::SoftwareTimeout("cmd"));
        for _ in 0..timeout {
            let sta = self.sdmmc.sta.read();

            // Command transfer still in progress.
            if sta.cmdact().bit_is_set() {
                continue;
            }

            if sta.ctimeout().bit() {
                res = Err(Error::Timeout);
                break;
            }

            if cmd.response_len() == ResponseLen::Zero {
                if sta.cmdsent().bit() {
                    res = Ok(());
                    break;
                }
            } else {
                if sta.ccrcfail().bit() {
                    res = Err(Error::CommandCrc);
                    break;
                }

                if sta.cmdrend().bit() {
                    res = Ok(());
                    break;
                }
            }
        }

        clear_static_command_flags(&self.sdmmc.icr);
        res
    }

    /// Create an [`SdmmcBlockDevice`], which implements the [`BlockDevice`](embedded-sdmmc::BlockDevice) trait.
    pub fn into_block_device(self) -> SdmmcBlockDevice<Sdmmc> {
        SdmmcBlockDevice {
            sdmmc: core::cell::RefCell::new(self),
        }
    }
}

#[derive(Debug, Clone)]
#[repr(align(4))]
pub struct DataBlock([u8; 512]);

impl DataBlock {
    pub const fn new() -> Self {
        Self([0; 512])
    }

    pub fn as_bytes(&self) -> &[u8] {
        &self.0
    }

    pub fn as_bytes_mut(&mut self) -> &mut [u8] {
        &mut self.0
    }

    pub(crate) fn blocks_to_words(blocks: &[DataBlock]) -> &[u32] {
        let word_count = blocks.len() * 128;
        // SAFETY: `DataBlock` is 4-byte aligned.
        unsafe { slice::from_raw_parts(blocks.as_ptr() as *mut u32, word_count) }
    }

    pub(crate) fn blocks_to_words_mut(blocks: &mut [DataBlock]) -> &mut [u32] {
        let word_count = blocks.len() * 128;
        // SAFETY: `DataBlock` is 4-byte aligned.
        unsafe { slice::from_raw_parts_mut(blocks.as_mut_ptr() as *mut u32, word_count) }
    }
}

#[derive(Debug)]
pub struct SdmmcDma {
    sdmmc: Sdmmc,
    channel: dma2::C5,
}

impl SdmmcDma {
    #[inline]
    pub fn new(sdmmc: Sdmmc, mut channel: dma2::C5) -> Self {
        channel.set_peripheral_address(sdmmc.sdmmc.fifo.as_ptr() as u32, false);
        channel.set_request_line(DmaInput::SdMmc1).unwrap();
        channel.ccr().modify(|_, w| {
            w
                // memory to memory mode disabled
                .mem2mem()
                .clear_bit()
                // medium channel priority level
                .pl()
                .very_high()
                // 32-bit memory size
                .msize()
                .bits32()
                // 32-bit peripheral size
                .psize()
                .bits32()
                // circular mode disabled
                .circ()
                .clear_bit()
        });

        Self { sdmmc, channel }
    }

    #[inline]
    pub fn read_block(&mut self, addr: u32, block: &mut DataBlock) -> Result<(), Error> {
        self.read_blocks(addr, slice::from_mut(block))
    }

    #[inline]
    pub fn read_blocks(&mut self, addr: u32, blocks: &mut [DataBlock]) -> Result<(), Error> {
        let words = DataBlock::blocks_to_words_mut(blocks);

        self.channel.ccr().modify(|_, w| {
            w
                // read from peripheral/write to memory
                .dir()
                .clear_bit()
        });
        self.channel
            .set_memory_address(words.as_mut_ptr() as u32, true);
        self.channel.set_transfer_length(words.len() as u16);
        atomic::compiler_fence(Ordering::Release);

        let stop_transmission = self.sdmmc.start_read_transfer(addr, words, true)?;
        self.channel.start();

        // self.sdmmc.sdmmc.mask.modify(|_, w| {
        //     w.dbckendie()
        //         .set_bit()
        //         .dataendie()
        //         .set_bit()
        //         .dcrcfailie()
        //         .set_bit()
        //         .dtimeoutie()
        //         .set_bit()
        //         .rxoverrie()
        //         .set_bit()
        // });

        let res = loop {
            match sta_rx!(self.sdmmc.sdmmc) {
                Ok(sta) => {
                    if sta.dbckend().bit_is_set() {
                        break Ok(());
                    }
                }
                Err(err) => break Err(err),
            }
        };

        clear_static_data_flags(&self.sdmmc.sdmmc.icr);
        self.channel.stop();

        if stop_transmission {
            res.or(self.sdmmc.cmd(common_cmd::stop_transmission()))
        } else {
            res
        }
    }

    #[inline]
    pub fn write_block(&mut self, addr: u32, block: &DataBlock) -> Result<(), Error> {
        self.write_blocks(addr, slice::from_ref(block))
    }

    #[inline]
    fn write_blocks(&mut self, addr: u32, blocks: &[DataBlock]) -> Result<(), Error> {
        let words = DataBlock::blocks_to_words(blocks);

        self.channel.ccr().modify(|_, w| {
            w
                // read from memory/write to peripheral
                .dir()
                .set_bit()
        });
        self.channel.set_memory_address(words.as_ptr() as u32, true);
        self.channel.set_transfer_length(words.len() as u16);
        atomic::compiler_fence(Ordering::Release);

        let stop_transmission = self.sdmmc.start_write_transfer(addr, words, true)?;
        self.channel.start();

        let res = loop {
            match sta_tx!(self.sdmmc.sdmmc) {
                Ok(sta) => {
                    if sta.dbckend().bit_is_set() {
                        break Ok(());
                    }
                }
                Err(err) => break Err(err),
            }
        };

        clear_static_data_flags(&self.sdmmc.sdmmc.icr);
        self.channel.stop();

        let res = if stop_transmission {
            res.or(self.sdmmc.cmd(common_cmd::stop_transmission()))
        } else {
            res
        };

        res?;

        let timeout: u32 = 0xffff_ffff;
        for _ in 0..timeout {
            if self.sdmmc.card_ready()? {
                return Ok(());
            }
        }

        Err(Error::SoftwareTimeout("write_blocks"))
    }

    #[inline]
    pub fn split(mut self) -> (Sdmmc, dma2::C5) {
        (self.sdmmc, self.channel)
    }

    /// Get the initialized card, if present.
    #[inline]
    pub fn card(&self) -> Result<&SdCard, Error> {
        self.sdmmc.card()
    }
}

/// Type implementing the [`BlockDevice`](embedded-sdmmc::BlockDevice) trait.
pub struct SdmmcBlockDevice<SDMMC> {
    sdmmc: core::cell::RefCell<SDMMC>,
}

#[cfg(feature = "embedded-sdmmc")]
impl embedded_sdmmc::BlockDevice for SdmmcBlockDevice<Sdmmc> {
    type Error = Error;

    fn read(
        &self,
        blocks: &mut [embedded_sdmmc::Block],
        start_block_idx: embedded_sdmmc::BlockIdx,
        _reason: &str,
    ) -> Result<(), Self::Error> {
        let mut sdmmc = self.sdmmc.borrow_mut();

        for block in blocks {
            // TODO: `embedded_sdmmc::Block` should be aligned to 4 bytes.
            let data_block = unsafe { &mut *(block.contents.as_mut_ptr() as *mut _) };
            sdmmc.read_block(start_block_idx.0, data_block)?;
        }

        Ok(())
    }

    fn write(
        &self,
        blocks: &[embedded_sdmmc::Block],
        start_block_idx: embedded_sdmmc::BlockIdx,
    ) -> Result<(), Self::Error> {
        let mut sdmmc = self.sdmmc.borrow_mut();

        for block in blocks {
            // TODO: `embedded_sdmmc::Block` should be aligned to 4 bytes.
            let data_block = unsafe { &*(block.contents.as_ptr() as *const _) };
            sdmmc.write_block(start_block_idx.0, data_block)?;
        }

        Ok(())
    }

    fn num_blocks(&self) -> Result<embedded_sdmmc::BlockCount, Self::Error> {
        let sdmmc = self.sdmmc.borrow_mut();
        let block_count = sdmmc.card()?.block_count();
        Ok(embedded_sdmmc::BlockCount(block_count as u32))
    }
}

#[cfg(feature = "fatfs")]
use fatfs::{IntoStorage, IoBase, IoError, Read, Seek, SeekFrom, Write};

impl IoError for Error {
    fn is_interrupted(&self) -> bool {
        false
    }

    fn new_unexpected_eof_error() -> Self {
        unimplemented!()
    }

    fn new_write_zero_error() -> Self {
        unimplemented!()
    }
}

#[derive(Debug, Clone, Copy)]
pub struct PartitionInfo {
    start: u32,
    len_bytes: u64,
}

impl PartitionInfo {
    pub const fn new(lba_start: u32, num_blocks: u32) -> Self {
        Self {
            start: lba_start,
            len_bytes: num_blocks as u64 * 512,
        }
    }

    #[inline]
    pub const fn start(&self) -> u32 {
        self.start
    }

    #[inline]
    pub const fn len_bytes(&self) -> u64 {
        self.len_bytes
    }
}

#[derive(Debug)]
pub struct FatFsCursor<SDMMC> {
    sdmmc: SDMMC,
    pos: u64,
    partition_index: u8,
    partition_info: Option<PartitionInfo>,
    block: DataBlock,
    current_block: Option<u32>,
    dirty: bool,
}

impl<SDMMC> FatFsCursor<SDMMC> {
    pub fn new(sdmmc: SDMMC) -> Self {
        Self {
            sdmmc,
            pos: 0,
            partition_index: 0,
            partition_info: None,
            block: DataBlock([0; 512]),
            current_block: None,
            dirty: false,
        }
    }

    pub fn sdmmc(&mut self) -> &SDMMC {
        &self.sdmmc
    }

    pub fn sdmmc_mut(&mut self) -> &mut SDMMC {
        &mut self.sdmmc
    }
}

pub trait SdmmcIo {
    fn read_block(&mut self, addr: u32, block: &mut DataBlock) -> Result<(), Error>;
    fn write_block(&mut self, addr: u32, block: &DataBlock) -> Result<(), Error>;
}

impl SdmmcIo for Sdmmc {
    #[inline(always)]
    fn read_block(&mut self, addr: u32, block: &mut DataBlock) -> Result<(), Error> {
        Self::read_block(self, addr, block)
    }

    #[inline(always)]
    fn write_block(&mut self, addr: u32, block: &DataBlock) -> Result<(), Error> {
        Self::write_block(self, addr, block)
    }
}

impl SdmmcIo for SdmmcDma {
    #[inline(always)]
    fn read_block(&mut self, addr: u32, block: &mut DataBlock) -> Result<(), Error> {
        Self::read_block(self, addr, block)
    }

    #[inline(always)]
    fn write_block(&mut self, addr: u32, block: &DataBlock) -> Result<(), Error> {
        Self::write_block(self, addr, block)
    }
}

impl<T> SdmmcIo for &mut T
where
    T: SdmmcIo,
{
    #[inline(always)]
    fn read_block(&mut self, addr: u32, block: &mut DataBlock) -> Result<(), Error> {
        (*self).read_block(addr, block)
    }

    #[inline(always)]
    fn write_block(&mut self, addr: u32, block: &DataBlock) -> Result<(), Error> {
        (*self).write_block(addr, block)
    }
}

impl<SDMMC> FatFsCursor<SDMMC>
where
    SDMMC: SdmmcIo,
{
    pub fn partition_info(&mut self) -> Result<PartitionInfo, Error> {
        if let Some(partition_info) = self.partition_info {
            return Ok(partition_info);
        }

        let mut block = DataBlock([0; 512]);
        self.sdmmc.read_block(0, &mut block)?;

        let start = self.partition_index as usize * 16;
        let partition_info = &block.0[446..][start..(start + 16)];
        let lba_start = u32::from_le_bytes([
            partition_info[8],
            partition_info[9],
            partition_info[10],
            partition_info[11],
        ]);

        let num_blocks = u32::from_le_bytes([
            partition_info[12],
            partition_info[13],
            partition_info[14],
            partition_info[15],
        ]);

        Ok(*self
            .partition_info
            .get_or_insert(PartitionInfo::new(lba_start, num_blocks)))
    }
}

#[cfg(feature = "fatfs")]
impl<'sdmmc> IntoStorage<FatFsCursor<&'sdmmc mut Sdmmc>> for &'sdmmc mut Sdmmc {
    #[inline]
    fn into_storage(self) -> FatFsCursor<&'sdmmc mut Sdmmc> {
        FatFsCursor::new(self)
    }
}

#[cfg(feature = "fatfs")]
impl<SDMMC> IoBase for FatFsCursor<SDMMC> {
    type Error = Error;
}

#[cfg(feature = "fatfs")]
impl<SDMMC> IoBase for &mut FatFsCursor<SDMMC> {
    type Error = Error;
}

impl<SDMMC> Seek for FatFsCursor<SDMMC>
where
    SDMMC: SdmmcIo,
{
    fn seek(&mut self, pos: SeekFrom) -> Result<u64, Self::Error> {
        // TODO: Replace when `u64::checked_add_signed(i64)` becomes stable.
        let checked_add_signed = |v: u64, o: i64| -> Result<u64, Self::Error> {
            if o.is_negative() {
                v.checked_sub(o.abs() as u64)
            } else {
                v.checked_add(o as u64)
            }
            .ok_or(Self::Error::InvalidInput)
        };
        let new_pos = match pos {
            SeekFrom::Start(offset) => Ok(offset),
            SeekFrom::End(offset) => {
                let end = self.partition_info()?.len_bytes();
                checked_add_signed(end as u64, offset)
            }
            SeekFrom::Current(offset) => checked_add_signed(self.pos, offset),
        }?;

        let new_pos = new_pos as u64;

        self.pos = new_pos;
        Ok(self.pos)
    }
}

#[cfg(feature = "fatfs")]
impl<SDMMC> Seek for &mut FatFsCursor<SDMMC>
where
    SDMMC: SdmmcIo,
{
    #[inline(always)]
    fn seek(&mut self, pos: SeekFrom) -> Result<u64, Self::Error> {
        (*self).seek(pos)
    }
}

#[cfg(feature = "fatfs")]
impl<SDMMC> Read for FatFsCursor<SDMMC>
where
    SDMMC: SdmmcIo,
{
    fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        let PartitionInfo { start, len_bytes } = self.partition_info()?;

        let pos = self.pos;
        if pos >= len_bytes {
            return Ok(0);
        }

        let addr = start + (pos / 512) as u32;
        let offset = (pos % 512) as usize;
        let len = buf.len().min(512 - offset);

        // Only read the block if we have not already read it.
        if self.current_block != Some(addr) {
            self.flush()?;
            self.current_block = None;
            self.sdmmc.read_block(addr, &mut self.block)?;
            self.current_block = Some(addr);
        }

        // SAFETY: `offset` and `len` are ensured to fit within the slices.
        unsafe {
            ptr::copy_nonoverlapping(self.block.0.as_ptr().add(offset), buf.as_mut_ptr(), len);
        }

        self.pos += len as u64;

        Ok(len)
    }

    // TODO: Add `read_exact` implementation which supports reading multiple blocks.
}

#[cfg(feature = "fatfs")]
impl<SDMMC> Read for &mut FatFsCursor<SDMMC>
where
    SDMMC: SdmmcIo,
{
    #[inline(always)]
    fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        (*self).read(buf)
    }
}

#[cfg(feature = "fatfs")]
impl<SDMMC> Write for FatFsCursor<SDMMC>
where
    SDMMC: SdmmcIo,
{
    fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        let PartitionInfo { start, len_bytes } = self.partition_info()?;

        let pos = self.pos;
        if pos >= len_bytes {
            return Ok(0);
        }

        let addr = start + (pos / 512) as u32;
        let offset = (pos % 512) as usize;
        let len = buf.len().min(512 - offset);

        // Flush if current block is not the one we're writing to.
        if self.current_block != Some(addr) {
            self.flush()?;

            // Read the block unless we write the full block.
            if len != 512 {
                self.current_block = None;
                self.sdmmc.read_block(addr, &mut self.block)?;
            }

            self.current_block = Some(addr);
        }

        // SAFETY: `offset` and `len` are ensured to fit within the slices.
        unsafe {
            ptr::copy_nonoverlapping(buf.as_ptr(), self.block.0.as_mut_ptr().add(offset), len);
        }

        self.dirty = true;
        self.pos += len as u64;

        Ok(len)
    }

    // TODO: Add `write_exact` implementation which supports writing multiple blocks.

    #[inline]
    fn flush(&mut self) -> Result<(), Self::Error> {
        if self.dirty {
            self.sdmmc
                .write_block(self.current_block.unwrap(), &self.block)?;
            self.dirty = false;
        }

        Ok(())
    }
}

#[cfg(feature = "fatfs")]
impl<SDMMC> Write for &mut FatFsCursor<SDMMC>
where
    SDMMC: SdmmcIo,
{
    #[inline(always)]
    fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        (*self).write(buf)
    }

    #[inline(always)]
    fn flush(&mut self) -> Result<(), Self::Error> {
        (*self).flush()
    }
}
