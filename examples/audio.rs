#![no_std]
#![no_main]

use panic_rtt_target as _;

use cortex_m_rt::entry;
use rtt_target::rprintln;

use stm32l4xx_hal::{
    delay::Delay,
    gpio::{Alternate, Pin, PushPull, H8, L8},
    pac,
    prelude::*,
    rcc::{MsiFreq, PllConfig},
    sai::{I2SChanConfig, I2SDataSize, I2SDir, I2sUsers, Sai},
    traits::i2s::FullDuplex,
};

type SaiPins = (
    Pin<Alternate<PushPull, 13_u8>, L8, 'E', 2_u8>,
    Pin<Alternate<PushPull, 13_u8>, L8, 'E', 5_u8>,
    Pin<Alternate<PushPull, 13_u8>, L8, 'E', 4_u8>,
    Pin<Alternate<PushPull, 13_u8>, L8, 'E', 6_u8>,
    Option<Pin<Alternate<PushPull, 13_u8>, H8, 'A', 13_u8>>,
);

#[entry]
fn main() -> ! {
    rtt_target::rtt_init_print!();
    rprintln!("Initializing... ");

    let cp = pac::CorePeripherals::take().unwrap();
    let dp = pac::Peripherals::take().unwrap();

    let mut flash = dp.FLASH.constrain();
    let mut rcc = dp.RCC.constrain();
    let mut pwr = dp.PWR.constrain(&mut rcc.apb1r1);

    let clocks = rcc
        .cfgr
        .msi(MsiFreq::RANGE8M)
        .sysclk(80.mhz())
        .hclk(80.mhz())
        .pclk1(80.mhz())
        .pclk2(80.mhz())
        .sai1clk_with_pll(4_016_000.hz(), PllConfig::new(1, 13, Some(25), None, None))
        .freeze(&mut flash.acr, &mut pwr);

    rprintln!(
        "clocks: sysclk: {:?}, hclk: {:?}, pclk1: {:?}, pclk2: {:?}",
        clocks.sysclk().0,
        clocks.hclk().0,
        clocks.pclk1().0,
        clocks.pclk2().0
    );

    let mut delay = Delay::new(cp.SYST, clocks);

    // GPIO
    ///////

    let mut gpioe = dp.GPIOE.split(&mut rcc.ahb2);

    let mclk =
        gpioe
            .pe2
            .into_alternate_push_pull(&mut gpioe.moder, &mut gpioe.otyper, &mut gpioe.afrl);
    let fs =
        gpioe
            .pe4
            .into_alternate_push_pull(&mut gpioe.moder, &mut gpioe.otyper, &mut gpioe.afrl);
    let sck =
        gpioe
            .pe5
            .into_alternate_push_pull(&mut gpioe.moder, &mut gpioe.otyper, &mut gpioe.afrl);
    let fd =
        gpioe
            .pe6
            .into_alternate_push_pull(&mut gpioe.moder, &mut gpioe.otyper, &mut gpioe.afrl);

    let sai_pins: SaiPins = (mclk, sck, fs, fd, None);

    rprintln!("Sai... ");
    let mut sai = Sai::i2s_sai1_ch_a(
        dp.SAI1,
        sai_pins,
        16_000.hz(),
        I2SDataSize::Bits24,
        &mut rcc.apb2,
        &clocks,
        I2sUsers::new(I2SChanConfig::new(I2SDir::Tx)),
    );

    rprintln!("Sai enable... ");
    sai.enable();

    rprintln!("Looping... ");
    loop {
        sai.try_send(0xaa, 0xcc).unwrap();
        delay.delay_ms(5_u32);
    }
}
