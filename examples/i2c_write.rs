//! Blinks an LED

#![deny(unsafe_code)]
// #![deny(warnings)]
#![no_std]
#![no_main]

extern crate cortex_m;
#[macro_use]
extern crate cortex_m_rt as rt;
extern crate cortex_m_semihosting as sh;
extern crate panic_semihosting;
extern crate stm32l432xx_hal as hal;
// #[macro_use(block)]
// extern crate nb;

use crate::hal::prelude::*;
use crate::hal::stm32l4::stm32l4x2;

use crate::hal::i2c::I2c;
use crate::rt::ExceptionFrame;
use crate::rt::entry;

use core::fmt::Write;
use crate::sh::hio;

#[entry]
fn main() -> ! {

    let mut hstdout = hio::hstdout().unwrap();

    writeln!(hstdout, "Hello, world!").unwrap();

    // let cp = cortex_m::Peripherals::take().unwrap();
    let dp = stm32l4x2::Peripherals::take().unwrap();

    let mut flash = dp.FLASH.constrain();
    let mut rcc = dp.RCC.constrain();

    let clocks = rcc.cfgr.freeze(&mut flash.acr);

    let mut gpioa = dp.GPIOA.split(&mut rcc.ahb2);

    let mut scl = gpioa.pa9.into_open_drain_output(&mut gpioa.moder, &mut gpioa.otyper);
    scl.internal_pull_up(&mut gpioa.pupdr, true);
    let scl = scl.into_af4(&mut gpioa.moder, &mut gpioa.afrh);

    let mut sda = gpioa.pa10.into_open_drain_output(&mut gpioa.moder, &mut gpioa.otyper);
    sda.internal_pull_up(&mut gpioa.pupdr, true);
    let sda = sda.into_af4(&mut gpioa.moder, &mut gpioa.afrh);
    
    let mut i2c = I2c::i2c1(dp.I2C1, (scl, sda), 100.khz(), clocks, &mut rcc.apb1r1);

    // i2c.write(0x3C, &[0xCC, 0xAA]).unwrap();
    let mut buffer = [0u8; 2];
    // 0x08 is version reg
    // i2c.write(0x36, &[0x08],).unwrap();
    // let val = i2c.read(0x36, &mut buffer).unwrap();
    i2c.write_read(0x36, &[0x08], &mut buffer).unwrap();
    

    loop {}
}

#[exception]
fn HardFault(ef: &ExceptionFrame) -> ! {
    panic!("{:#?}", ef);
}