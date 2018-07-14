//! Interfacing the on-board LSM303DLHC (accelerometer + compass)
#![deny(unsafe_code)]
// #![deny(warnings)]
#![no_std]
#![no_main]

#[macro_use(entry, exception)]
extern crate cortex_m_rt as rt;
extern crate cortex_m;
extern crate panic_semihosting;
extern crate stm32l432xx_hal as hal;
extern crate embedded_graphics;
extern crate ssd1306;

use cortex_m::asm;
use hal::i2c::I2c;
use hal::prelude::*;
use hal::stm32l4::stm32l4x2;
use rt::ExceptionFrame;

use embedded_graphics::prelude::*;
use embedded_graphics::primitives::{Circle, Line, Rect};
use ssd1306::prelude::*;
use ssd1306::Builder;


entry!(main);

fn main() -> ! {
    let p = stm32l4x2::Peripherals::take().unwrap();

    let mut flash = p.FLASH.constrain();
    let mut rcc = p.RCC.constrain();

    // TRY the other clock configuration
    let clocks = rcc.cfgr.freeze(&mut flash.acr);
    // let clocks = rcc.cfgr.sysclk(64.mhz()).pclk1(32.mhz()).freeze(&mut flash.acr);

    let mut gpiob = p.GPIOB.split(&mut rcc.ahb2);
    let scl = gpiob.pb6.into_af4(&mut gpiob.moder, &mut gpiob.afrl);
    let sda = gpiob.pb7.into_af4(&mut gpiob.moder, &mut gpiob.afrl);

    let i2c = I2c::i2c1(p.I2C1, (scl, sda), 400.khz(), clocks, &mut rcc.apb1r1);

    let mut disp: GraphicsMode<_> = Builder::new().with_i2c_addr(0x3C).connect_i2c(i2c).into();

    disp.init().unwrap();
    disp.flush().unwrap();

    disp.draw(Line::new((8, 16 + 16), (8 + 16, 16 + 16), 1).into_iter());
    disp.draw(Line::new((8, 16 + 16), (8 + 8, 16), 1).into_iter());
    disp.draw(Line::new((8 + 16, 16 + 16), (8 + 8, 16), 1).into_iter());

    disp.draw(Rect::new((48, 16), (48 + 16, 16 + 16), 1u8).into_iter());

    disp.draw(Circle::new((96, 16 + 8), 8, 1u8).into_iter());

    disp.flush().unwrap();
    

    // when you reach this breakpoint you'll be able to inspect the variables `_accel`, `_mag` and
    // `_temp` which contain the accelerometer, compass (magnetometer) and temperature sensor
    // readings
    asm::bkpt();

    loop {}
}

exception!(HardFault, hard_fault);

fn hard_fault(ef: &ExceptionFrame) -> ! {
    panic!("{:#?}", ef);
}

exception!(*, default_handler);

fn default_handler(irqn: i16) {
    panic!("Unhandled exception (IRQn = {})", irqn);
}
