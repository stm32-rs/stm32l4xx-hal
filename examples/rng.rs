#![no_std]
#![no_main]

extern crate panic_halt;
extern crate stm32l4xx_hal as hal;

use core::fmt;
use cortex_m_rt::entry;

use crate::hal::delay::Delay;
use crate::hal::prelude::*;
use crate::hal::serial::{Config, Serial};
use crate::hal::stm32;
use hal::hal::blocking::rng::Read;
use hal::rcc::Clk48Source;

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

#[entry]
fn main() -> ! {
    let core = cortex_m::Peripherals::take().unwrap();
    let device = stm32::Peripherals::take().unwrap();

    let mut flash = device.FLASH.constrain();
    let mut rcc = device.RCC.constrain();
    let mut pwr = device.PWR.constrain(&mut rcc.apb1r1);

    let clocks = rcc
        .cfgr
        // Needed for RNG.
        .clk48_source({
            #[cfg(any(
                feature = "stm32l476",
                feature = "stm32l486",
                feature = "stm32l496",
                feature = "stm32l4a6"
            ))]
            {
                Clk48Source::Hsi48
            }

            #[cfg(not(any(
                feature = "stm32l476",
                feature = "stm32l486",
                feature = "stm32l496",
                feature = "stm32l4a6"
            )))]
            {
                Clk48Source::Msi
            }
        })
        .sysclk(64.MHz())
        .pclk1(32.MHz())
        .freeze(&mut flash.acr, &mut pwr);

    // setup usart
    let mut gpioa = device.GPIOA.split(&mut rcc.ahb2);
    let tx = gpioa
        .pa9
        .into_alternate(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrh);
    let rx = gpioa
        .pa10
        .into_alternate(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrh);

    let baud_rate = 9_600; // 115_200;
    let serial = Serial::usart1(
        device.USART1,
        (tx, rx),
        Config::default().baudrate(baud_rate.bps()),
        clocks,
        &mut rcc.apb2,
    );
    let (mut tx, _) = serial.split();

    // get a timer
    let mut timer = Delay::new(core.SYST, clocks);

    // setup rng
    let mut rng = device.RNG.enable(&mut rcc.ahb2, clocks);

    uprintln!(&mut tx, "{:?}", clocks);

    let some_time: u32 = 500;
    loop {
        const N: usize = 5;
        let mut random_bytes = [0u8; N];
        rng.read(&mut random_bytes)
            .expect("missing random data for some reason");
        uprintln!(&mut tx, "{} random u8 values: {:?}", N, random_bytes);

        timer.delay_ms(some_time);
    }
}
