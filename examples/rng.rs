#![no_std]
#![no_main]

use cortex_m_rt::entry;
use defmt::println;
use panic_probe as _;
use stm32l4xx_hal::{
    delay::Delay,
    hal::blocking::rng::Read,
    prelude::*,
    serial::{Config, Serial},
    stm32,
};

#[entry]
fn main() -> ! {
    let core = cortex_m::Peripherals::take().unwrap();
    let device = stm32::Peripherals::take().unwrap();

    let mut flash = device.FLASH.constrain();
    let mut rcc = device.RCC.constrain();
    let mut pwr = device.PWR.constrain(&mut rcc.apb1r1);

    let clocks = rcc
        .cfgr
        .hsi48(true) // needed for RNG
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

    println!("{:?}", defmt::Debug2Format(&clocks));

    let some_time: u32 = 500;
    loop {
        const N: usize = 5;
        let mut random_bytes = [0u8; N];
        rng.read(&mut random_bytes)
            .expect("missing random data for some reason");
        println!("{} random u8 values: {:?}", N, random_bytes);

        timer.delay_ms(some_time);
    }
}
