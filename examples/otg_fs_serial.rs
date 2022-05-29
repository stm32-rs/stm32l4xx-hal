//! OTG USB 2.0 FS serial port example using polling in a busy loop.
//!
//! Note: Must build with features "stm32l4x5 otg_fs" or "stm32l4x6 otg_fs".
#![no_main]
#![no_std]

extern crate panic_semihosting;

use cortex_m_rt::entry;
use stm32l4xx_hal::gpio::Speed;
use stm32l4xx_hal::otg_fs::{Usb, UsbBus};
use stm32l4xx_hal::prelude::*;
use stm32l4xx_hal::rcc::{
    ClockSecuritySystem, CrystalBypass, MsiFreq, PllConfig, PllDivider, PllSource,
};
use stm32l4xx_hal::stm32::{Peripherals, CRS, PWR, RCC};
use usb_device::prelude::*;

/// Enable CRS (Clock Recovery System)
fn enable_crs() {
    let rcc = unsafe { &(*RCC::ptr()) };
    rcc.apb1enr1.modify(|_, w| w.crsen().set_bit());
    let crs = unsafe { &(*CRS::ptr()) };
    // Initialize clock recovery
    // Set autotrim enabled.
    crs.cr.modify(|_, w| w.autotrimen().set_bit());
    // Enable CR
    crs.cr.modify(|_, w| w.cen().set_bit());
}

static mut EP_MEMORY: [u32; 1024] = [0; 1024];

#[entry]
unsafe fn main() -> ! {
    let dp = Peripherals::take().unwrap();

    let mut flash = dp.FLASH.constrain();
    let mut rcc = dp.RCC.constrain();
    let mut pwr = dp.PWR.constrain(&mut rcc.apb1r1);

    // Set to true if external 16 MHz high-speed resonator/crystal is used.
    const USE_HSE_CLK: bool = true;

    let clocks = {
        if !USE_HSE_CLK {
            // 48 MHz / 6 * 40 / 4 = 80 MHz
            let pll_cfg = PllConfig::new(6, 40, PllDivider::Div4);

            // Note: If program needs low-speed clocks, adjust this.
            rcc.cfgr
                .msi(MsiFreq::RANGE48M) // Set the MSI (multi-speed internal) clock to 48 MHz
                .pll_source(PllSource::MSI)
                .sysclk_with_pll(80.MHz(), pll_cfg)
                .pclk1(24.MHz())
                .pclk2(24.MHz())
                .freeze(&mut flash.acr, &mut pwr)
        } else {
            // Note: If program needs low-speed clocks, adjust this.
            //       Tested using a 16 MHz resonator.
            rcc.cfgr
                .msi(MsiFreq::RANGE48M)
                .hse(
                    16.MHz(),
                    CrystalBypass::Disable, // Bypass enabled when clock signals instead of crystals/resonators are used.
                    ClockSecuritySystem::Disable, // We have not set up interrupt routines handling clock drifts/errors.
                )
                .pll_source(PllSource::HSE)
                .sysclk(80.MHz())
                .freeze(&mut flash.acr, &mut pwr)
        }
    };

    // Enable clock recovery system.
    enable_crs();

    let mut gpioa = dp.GPIOA.split(&mut rcc.ahb2);

    let usb = Usb::new(
        dp.OTG_FS_GLOBAL,
        dp.OTG_FS_DEVICE,
        dp.OTG_FS_PWRCLK,
        gpioa
            .pa11
            .into_alternate(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrh)
            .set_speed(Speed::VeryHigh),
        gpioa
            .pa12
            .into_alternate(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrh)
            .set_speed(Speed::VeryHigh),
        &mut pwr,
        clocks.hclk(),
    );

    let usb_bus = UsbBus::new(usb, &mut EP_MEMORY);

    let mut usb_serial = usbd_serial::SerialPort::new(&usb_bus);

    let mut usb_dev = UsbDeviceBuilder::new(&usb_bus, UsbVidPid(0x16c0, 0x27dd))
        .manufacturer("Fake Company")
        .product("Serial port")
        .serial_number("TEST")
        .device_class(usbd_serial::USB_CLASS_CDC)
        .build();

    #[cfg(feature = "semihosting")]
    hprintln!("Polling!").ok();

    loop {
        if !usb_dev.poll(&mut [&mut usb_serial]) {
            continue;
        }

        let mut buf = [0u8; 64];

        match usb_serial.read(&mut buf) {
            Ok(count) if count > 0 => {
                // Echo back in upper case
                for c in buf[0..count].iter_mut() {
                    if 0x61 <= *c && *c <= 0x7a {
                        *c &= !0x20;
                    }
                }

                let mut write_offset = 0;
                while write_offset < count {
                    match usb_serial.write(&buf[write_offset..count]) {
                        Ok(len) if len > 0 => {
                            write_offset += len;
                        }
                        _ => {}
                    }
                }
            }
            _ => {}
        }
    }
}
