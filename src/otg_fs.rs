//! USB OTG full-speed peripheral
//!
//! The STM32L4 series only supports the full-speed peripheral.

pub use synopsys_usb_otg::UsbBus;
use synopsys_usb_otg::UsbPeripheral;

use crate::{
    gpio::{
        gpioa::{PA11, PA12},
        Alternate, PushPull,
    },
    pac,
    pwr::Pwr,
    rcc::{Clocks, Enable, Reset},
    time::Hertz,
};

pub struct Usb {
    _global: pac::OTG_FS_GLOBAL,
    _device: pac::OTG_FS_DEVICE,
    _pwrclk: pac::OTG_FS_PWRCLK,
    // TODO: check type
    _dm: PA11<Alternate<PushPull, 10>>,
    _dp: PA12<Alternate<PushPull, 10>>,
    hclk: Hertz,
}

impl Usb {
    pub fn new(
        global: pac::OTG_FS_GLOBAL,
        device: pac::OTG_FS_DEVICE,
        pwrclk: pac::OTG_FS_PWRCLK,
        dm: PA11<Alternate<PushPull, 10>>,
        dp: PA12<Alternate<PushPull, 10>>,
        pwr: &mut Pwr,
        clocks: Clocks,
    ) -> Self {
        pwr.cr2.reg().modify(|_, w| w.usv().set_bit());

        Self {
            _global: global,
            _device: device,
            _pwrclk: pwrclk,
            _dm: dm,
            _dp: dp,
            hclk: clocks.hclk(),
        }
    }
}

unsafe impl Sync for Usb {}

unsafe impl UsbPeripheral for Usb {
    const REGISTERS: *const () = pac::OTG_FS_GLOBAL::ptr() as *const ();

    const HIGH_SPEED: bool = false;
    const FIFO_DEPTH_WORDS: usize = 320;

    const ENDPOINT_COUNT: usize = 6;

    fn enable() {
        cortex_m::interrupt::free(|_| unsafe {
            // Enable USB peripheral
            pac::OTG_FS_GLOBAL::enable_unchecked();

            // Reset USB peripheral
            pac::OTG_FS_GLOBAL::reset_unchecked();
        });
    }

    fn ahb_frequency_hz(&self) -> u32 {
        self.hclk.to_Hz()
    }
}

pub type UsbBusType = UsbBus<Usb>;
