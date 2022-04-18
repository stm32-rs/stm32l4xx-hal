//! Testing PWM output
#![no_main]
#![no_std]

use cortex_m_rt::entry;
use defmt::println;
use defmt_rtt as _;
use panic_probe as _;
use stm32l4xx_hal::{delay, prelude::*, stm32};

#[entry]
fn main() -> ! {
    let c = cortex_m::Peripherals::take().unwrap();
    let p = stm32::Peripherals::take().unwrap();

    let mut flash = p.FLASH.constrain();
    let mut rcc = p.RCC.constrain();
    let mut pwr = p.PWR.constrain(&mut rcc.apb1r1);

    let clocks = rcc.cfgr.freeze(&mut flash.acr, &mut pwr);

    let mut gpioa = p.GPIOA.split(&mut rcc.ahb2);

    // TIM2
    let c1 = gpioa
        .pa0
        .into_alternate(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrl);
    let c2 = gpioa
        .pa1
        .into_alternate(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrl);
    let c3 = gpioa
        .pa2
        .into_alternate(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrl);
    let c4 = gpioa
        .pa3
        .into_alternate(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrl);

    let mut pwm = p
        .TIM2
        .pwm((c1, c2, c3, c4), 1.kHz(), clocks, &mut rcc.apb1r1)
        .3;

    let max = pwm.get_max_duty();

    pwm.enable();

    let mut timer = delay::Delay::new(c.SYST, clocks);
    let second: u32 = 100;

    // NB: if the pins are LEDs, brightness is not
    //     linear in duty value.
    loop {
        println!("100%");
        pwm.set_duty(max);
        timer.delay_ms(second);

        println!("91%");
        pwm.set_duty(max / 11 * 10);
        timer.delay_ms(second);

        println!("75%");
        pwm.set_duty(3 * max / 4);
        timer.delay_ms(second);

        println!("50%");
        pwm.set_duty(max / 2);
        timer.delay_ms(second);

        println!("25%");
        pwm.set_duty(max / 4);
        timer.delay_ms(second);
    }
}
