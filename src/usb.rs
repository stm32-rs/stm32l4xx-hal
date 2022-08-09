//! USB peripheral
//!
//! Requires the `stm32-usbd` feature.
//!
//! See <https://github.com/stm32-rs/stm32l4xx-hal/tree/master/examples>
//! for usage examples.

use crate::rcc::{Enable, Reset};
use crate::stm32::USB;
use stm32_usbd::UsbPeripheral;

use crate::gpio::gpioa::{PA11, PA12};
use crate::gpio::{Alternate, PushPull};
pub use stm32_usbd::UsbBus;

pub struct Peripheral {
    pub usb: USB,
    pub pin_dm: PA11<Alternate<PushPull, 10>>,
    pub pin_dp: PA12<Alternate<PushPull, 10>>,
}

unsafe impl Sync for Peripheral {}

unsafe impl UsbPeripheral for Peripheral {
    const REGISTERS: *const () = USB::ptr() as *const ();
    const DP_PULL_UP_FEATURE: bool = true;
    const EP_MEMORY: *const () = 0x4000_6c00 as _;
    const EP_MEMORY_SIZE: usize = 1024;
    const EP_MEMORY_ACCESS_2X16: bool = true;

    fn enable() {
        cortex_m::interrupt::free(|_| unsafe {
            // Enable USB peripheral
            USB::enable_unchecked();

            // Reset USB peripheral
            USB::reset_unchecked();
        });
    }

    fn startup_delay() {
        // There is a chip specific startup delay. For STM32F103xx it's 1µs and this should wait for
        // at least that long.
        cortex_m::asm::delay(72);
    }
}

pub type UsbBusType = UsbBus<Peripheral>;
