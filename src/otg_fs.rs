//! USB OTG full-speed peripheral
//!
//! The STM32L4 series only supports the full-speed peripheral.

use crate::rcc::{Enable, Reset};
use crate::stm32;

use crate::gpio::{
    gpioa::{PA11, PA12},
    Alternate, PushPull,
};
use crate::time::Hertz;

pub use synopsys_usb_otg::UsbBus;
use synopsys_usb_otg::UsbPeripheral;

pub struct USB {
    pub usb_global: stm32::OTG_FS_GLOBAL,
    pub usb_device: stm32::OTG_FS_DEVICE,
    pub usb_pwrclk: stm32::OTG_FS_PWRCLK,
    // TODO: check type
    pub pin_dm: PA11<Alternate<PushPull, 10>>,
    pub pin_dp: PA12<Alternate<PushPull, 10>>,
    pub hclk: Hertz,
}

unsafe impl Sync for USB {}

unsafe impl UsbPeripheral for USB {
    const REGISTERS: *const () = stm32::OTG_FS_GLOBAL::ptr() as *const ();

    const HIGH_SPEED: bool = false;
    const FIFO_DEPTH_WORDS: usize = 320;

    const ENDPOINT_COUNT: usize = 6;

    fn enable() {
        cortex_m::interrupt::free(|_| unsafe {
            // Enable USB peripheral
            stm32::OTG_FS_GLOBAL::enable_unchecked();

            // Reset USB peripheral
            stm32::OTG_FS_GLOBAL::reset_unchecked();
        });
    }

    fn ahb_frequency_hz(&self) -> u32 {
        self.hclk.to_Hz()
    }
}

pub type UsbBusType = UsbBus<USB>;
