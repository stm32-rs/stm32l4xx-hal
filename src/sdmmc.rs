use fugit::HertzU32 as Hertz;
use sdio_host::{
    common_cmd::{self, ResponseLen},
    Cmd,
};

use crate::{
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
    const BUSWIDTH: BusWidth;
}

impl<CLK, CMD, D0> Pins for (CLK, CMD, D0)
where
    CLK: PinClk,
    CMD: PinCmd,
    D0: PinD0,
{
    const BUSWIDTH: BusWidth = BusWidth::BusWidth1;
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
    const BUSWIDTH: BusWidth = BusWidth::BusWidth4;
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
    const BUSWIDTH: BusWidth = BusWidth::BusWidth8;
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

#[cfg(any(feature = "stm32l496",))]
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

#[derive(Debug, Copy, Clone, Eq, PartialEq)]
#[repr(u8)]
pub enum BusWidth {
    BusWidth1 = 0,
    BusWidth4 = 1,
    BusWidth8 = 2,
}

#[derive(Debug, Copy, Clone, Eq, PartialEq)]
#[repr(u8)]
pub enum ClockFreq {
    Freq24MHz = 0,    // 48 MHz / (0 + 2) < 25 MHz
    Freq400KHz = 118, // 48 MHz / (118 + 2) < 400 kHz
}

#[derive(Debug)]
pub enum Error {
    NoCard,
    SoftwareTimeout,
    Crc,
    DataCrcFail,
    RxOverFlow,
    Timeout,
    TxUnderErr,
}

fn status_to_error(sta: sdmmc1::sta::R) -> Result<(), Error> {
    if sta.ctimeout().bit_is_set() {
        return Err(Error::Timeout);
    } else if sta.ccrcfail().bit() {
        return Err(Error::Crc);
    } else if sta.dcrcfail().bit() {
        return Err(Error::DataCrcFail);
    } else if sta.rxoverr().bit() {
        return Err(Error::RxOverFlow);
    } else if sta.dtimeout().bit() {
        return Err(Error::Timeout);
    } else if sta.txunderr().bit() {
        return Err(Error::TxUnderErr);
    }
    Ok(())
}

fn clear_all_interrupts(icr: &sdmmc1::ICR) {
    icr.modify(|_, w| {
        w.ccrcfailc()
            .set_bit()
            .ctimeoutc()
            .set_bit()
            .ceataendc()
            .set_bit()
            .cmdrendc()
            .set_bit()
            .cmdsentc()
            .set_bit()
            .dataendc()
            .set_bit()
            .dbckendc()
            .set_bit()
            .dcrcfailc()
            .set_bit()
            .dtimeoutc()
            .set_bit()
            .sdioitc()
            .set_bit()
            .stbiterrc()
            .set_bit()
            .rxoverrc()
            .set_bit()
            .txunderrc()
            .set_bit()
    });
}

pub struct SdMmc {
    sdmmc: SDMMC1,
    clock: Hertz,
}

impl SdMmc {
    pub fn new<PINS: Pins>(
        mut sdmmc: SDMMC1,
        _pins: PINS,
        apb2: &mut APB2,
        clocks: &Clocks,
    ) -> Self {
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
                .bits(0 as u8) // Clock must be <= 400 kHz while in identification mode.
                .clken()
                .clear_bit() // Disable clock.
        });

        let mut host = Self { sdmmc, clock };

        host.power_card(false);

        host
    }

    pub fn init(&mut self) -> Result<(), Error> {
        self.power_card(true);

        // Enable clock.
        self.sdmmc.clkcr.modify(|_, w| w.clken().set_bit());

        self.cmd(common_cmd::idle())
    }

    pub fn power_card(&mut self, on: bool) {
        self.sdmmc
            .power
            .modify(|_, w| unsafe { w.pwrctrl().bits(if on { 0b11 } else { 0b00 }) });

        // Wait for 2 ms after power mode change.
        cortex_m::asm::delay(2 * (self.clock.raw() / 1000));
    }

    pub fn cmd<R: common_cmd::Resp>(&self, cmd: Cmd<R>) -> Result<(), Error> {
        while self.sdmmc.sta.read().cmdact().bit_is_set() {}

        // Clear the interrupts before we start
        clear_all_interrupts(&self.sdmmc.icr);

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

        let mut timeout = 5000 * (self.clock.raw() / 8 / 1000);

        let sta = if cmd.response_len() == ResponseLen::Zero {

            loop {
                let sta = self.sdmmc.sta.read();

                if sta.cmdact().bit_is_clear()
                    && (sta.ctimeout().bit_is_set() || sta.cmdsent().bit_is_set())
                {
                    break sta;
                }
                if timeout == 0 {
                    return Err(Error::SoftwareTimeout);
                }

                timeout -= 1;
            }
        } else {

            let sta = loop {
                timeout -= 1;

                if timeout == 0 {
                    return Err(Error::SoftwareTimeout);
                }

                let sta = self.sdmmc.sta.read();

                if sta.ccrcfail().bit() || sta.cmdrend().bit() || sta.ctimeout().bit() {
                    break sta;
                }
            };

            if sta.ctimeout().bit() {
                self.sdmmc.icr.modify(|_, w| w.ctimeoutc().set_bit());

                return Err(Error::Timeout);
            }

            if sta.ccrcfail().bit() {
                self.sdmmc.icr.modify(|_, w| w.ccrcfailc().set_bit());

                return Err(Error::Crc);
            }

            if self.sdmmc.respcmd.read().respcmd().bits() != cmd.cmd {
                return Err(Error::Crc);
            }

            sta
        };

        status_to_error(sta)
    }
}
