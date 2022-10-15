//! Example of using the Cyclic Redundancy Check calculation unit
//! (CRC).
//! This example is based on the following document from
//! STMicroelectronics:
//!
//! RM0394 Reference manual:
//! STM32L41xxx/42xxx/43xxx/44xxx/45xxx/46xxx advanced Arm®-based 32-bit MCUs
//! RM0394 Rev 4
//!
//! The CRC unit lets you generate CRC codes by specifying a generator
//! polynomial, various CRC configuration settings, and passing in
//! data.
//! The STM32L4XX class of devices have a much richer set of
//! functionality included with their CRC calculation units than the
//! STM32F1XX devices and earlier devices.  It allows specifying the
//! polynomial itself, along with bit reversal and other settings.
//!
//! This example shows how to convert from a common CRC configuration
//! model to the STM32 and stm32l4xx-hal configuration model.
#![no_std]
#![no_main]

use core::fmt;
use panic_halt as _;

extern crate cortex_m;
extern crate cortex_m_rt as rt;
extern crate stm32l4xx_hal as hal;

use crate::stm32::CRC;
use stm32l4xx_hal::{
    crc::{self, BitReversal},
    rcc::AHB1,
};

use cortex_m_rt::entry;

use crate::hal::prelude::*;
use crate::hal::serial::{self, Serial};
use crate::hal::stm32;

macro_rules! uprint {
    ($serial:expr, $($arg:tt)*) => {
        fmt::write($serial, format_args!($($arg)*)).ok()
    };
}

macro_rules! uprintln {
    ($serial:expr, $fmt:expr) => {
        uprint!($serial, concat!($fmt, "\n"))
    };
    ($serial:expr, $fmt:expr, $($arg:tt)*) => {
        uprint!($serial, concat!($fmt, "\n"), $($arg)*)
    };
}

/// Some examples of common CRC polynomials and settings
/// The STM32L4xxxx units support custom settings, this is meant to
/// show how to build some common or useful CRCs.
#[allow(dead_code)]
enum CRCPreset {
    /// Common CRC-32 used in Ethernet, gzip, PNG, MPEG-2 and others
    Crc32Mpeg2,
    /// CRC-32 used in zlib
    Crc32IsoHdlc,
    /// CRC-16-CCITT used in Bluetooth, XMODEM, and others
    Crc16Ccitt,
}

/// CRC settings
/// This is a commonly used parameter model to build CRCs
/// The settings model is from
/// [Catalogue of parametrised CRC algorithms](http://reveng.sourceforge.net/crc-catalogue).
/// and Williams, Ross N. "A Painless Guide to CRC Error Detection
/// Algorithms", Rocksoft Pty Ltd., 1993, crc_ross.pdf
struct CRCSettings {
    width: u8,
    poly: u32,
    init: u32,
    refin: bool,
    refout: bool,
    xorout: u32,
    check: u32,
}

/// CRC settings for the three examples
/// These settings are from the
/// [Catalogue of parametrised CRC algorithms](http://reveng.sourceforge.net/crc-catalogue).
fn crc_settings(preset: &CRCPreset) -> CRCSettings {
    match preset {
        // specify the settings directly
        // We can also use the default settings of the unit,
        // This is shown in the below code.
        CRCPreset::Crc32Mpeg2 => CRCSettings {
            width: 32,
            poly: 0x04c11db7,
            init: 0xffffffff,
            refin: false,
            refout: false,
            xorout: 0x00000000,
            check: 0x0376e6e7,
        },

        CRCPreset::Crc32IsoHdlc => CRCSettings {
            width: 32,
            poly: 0x04c11db7,
            init: 0xffffffff,
            refin: true,
            refout: true,
            xorout: 0xffffffff,
            check: 0xcbf43926,
        },

        CRCPreset::Crc16Ccitt => CRCSettings {
            width: 16,
            poly: 0x1021,
            init: 0x0000,
            refin: false,
            refout: false,
            xorout: 0x0000,
            check: 0x31c3,
        },
    }
}

/// CRC configuration selected to use in the computation
// You can select a different configuration by changing this constant
const CRC_CONFIGURATION: CRCPreset = CRCPreset::Crc32Mpeg2;

/// Generate a CRC using the stm32::CRC API
fn generate_crc(preset: &CRCPreset, crc: CRC, ahb1: &mut AHB1, data: &[u8]) -> u32 {
    let mut crc = match preset {
        // The default for the STM32L4XX devices is CRC-32 MPEG-2
        CRCPreset::Crc32Mpeg2 => crc.constrain(ahb1).freeze(),
        // Explicitly set CRC Computation Unit registers
        _ => {
            let settings = crc_settings(preset);

            let poly = match settings.width {
                16 => crc::Polynomial::L16(settings.poly as u16),
                32 => crc::Polynomial::L32(settings.poly),
                _ => panic!("This width doesn't currently have an example or is not supported"),
            };

            // Apply the settings, CRC polynomial, and initial value
            let crc = crc
                .constrain(ahb1)
                .polynomial(poly)
                .initial_value(settings.init);

            // Apply input bit reversal if enabled in the settings.
            let crc = if settings.refin {
                crc.input_bit_reversal(BitReversal::ByByte)
            } else {
                crc
            };

            // Apply output bit reversal if enabled in the settings
            // and freeze the configuration.
            crc.output_bit_reversal(settings.refout).freeze()
        }
    };

    crc.feed(data);
    crc.result()
}

#[entry]
fn main() -> ! {
    let device = stm32::Peripherals::take().unwrap();

    let mut flash = device.FLASH.constrain();
    let mut rcc = device.RCC.constrain();
    let mut pwr = device.PWR.constrain(&mut rcc.apb1r1);

    let clocks = rcc
        .cfgr
        .sysclk(64.MHz())
        .pclk1(32.MHz())
        .freeze(&mut flash.acr, &mut pwr);

    let mut gpioa = device.GPIOA.split(&mut rcc.ahb2);

    // setup usart
    let tx = gpioa
        .pa9
        .into_alternate(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrh);
    let rx = gpioa
        .pa10
        .into_alternate(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrh);

    // setup serial
    let baud_rate = 9_600;
    let serial = Serial::usart1(
        device.USART1,
        (tx, rx),
        serial::Config::default().baudrate(baud_rate.bps()),
        clocks,
        &mut rcc.apb2,
    );
    let (mut tx, _) = serial.split();

    let string = "123456789";
    let data = string.as_bytes();

    let mut result = generate_crc(&CRC_CONFIGURATION, device.CRC, &mut rcc.ahb1, data);

    let settings = crc_settings(&CRC_CONFIGURATION);

    // XOR the output if the CRC settings specify it
    result ^= settings.xorout;

    uprintln!(&mut tx, "CRC Result: 0x{:X}\r", result);
    if result == settings.check {
        uprintln!(&mut tx, "Result matches expected value\r");
    } else {
        uprintln!(
            &mut tx,
            "Result doesn't match expected value: 0x{:X} != 0x{:X}\r",
            result,
            settings.check
        );
    }

    loop {}
}
