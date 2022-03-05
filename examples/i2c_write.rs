//! Blinks an LED
#![no_std]
#![no_main]

use cortex_m_rt as rt;
use defmt::println;
use hal::{
    i2c::{self, I2c},
    prelude::*,
};
use panic_probe as _;
use rt::{entry, ExceptionFrame};
use stm32l4xx_hal as hal;

#[entry]
fn main() -> ! {
    let dp = hal::stm32::Peripherals::take().unwrap();

    let mut flash = dp.FLASH.constrain();
    let mut rcc = dp.RCC.constrain();
    let mut pwr = dp.PWR.constrain(&mut rcc.apb1r1);

    let clocks = rcc.cfgr.freeze(&mut flash.acr, &mut pwr);

    let mut gpioa = dp.GPIOA.split(&mut rcc.ahb2);

    let mut scl =
        gpioa
            .pa9
            .into_alternate_open_drain(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrh);
    scl.internal_pull_up(&mut gpioa.pupdr, true);

    let mut sda =
        gpioa
            .pa10
            .into_alternate_open_drain(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrh);
    sda.internal_pull_up(&mut gpioa.pupdr, true);

    let mut i2c = I2c::i2c1(
        dp.I2C1,
        (scl, sda),
        i2c::Config::new(100.kHz(), clocks),
        &mut rcc.apb1r1,
    );

    let mut buffer = [0u8; 2];

    const MAX17048_ADDR: u8 = 0x6C;
    // read two bytes starting from version register high byte
    const MAX17048_VERSION_REG: u8 = 0x08;
    i2c.write_read(MAX17048_ADDR, &[MAX17048_VERSION_REG], &mut buffer)
        .unwrap();
    let version: u16 = u16::from_be_bytes(buffer); // (buffer[0] as u16) << 8 | buffer[1] as u16;
    println!("Silicon Version: {}", version).ok();

    // let soc: u16 = (buffer[0] as u16) + (buffer[1] as u16 / 256);  //& 0xFF00
    // let soc: u16 = (buffer[0] as u16) << 8 & 0xFF00 | (buffer[1] as u16) & 0x00FF;
    const MAX17048_SOC_REG: u8 = 0x04;
    i2c.write_read(MAX17048_ADDR, &[MAX17048_SOC_REG], &mut buffer)
        .unwrap();
    let soc: u16 = u16::from_be_bytes(buffer);
    println!("Batt SoC: {}%", soc / 256).ok();

    const MAX17048_VOLT_REG: u8 = 0x02;
    i2c.write_read(MAX17048_ADDR, &[MAX17048_VOLT_REG], &mut buffer)
        .unwrap();
    let vlt: u16 = u16::from_be_bytes(buffer);
    println!("Volt: {}", vlt as f32 * 0.000078125).ok();

    loop {
        continue;
    }
}
