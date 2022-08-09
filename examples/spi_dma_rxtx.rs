//! Test the SPI in RX/TX (transfer) DMA mode
#![deny(unsafe_code)]
#![no_main]
#![no_std]

use panic_rtt_target as _;
use rtt_target::rprintln;
use stm32l4xx_hal::{
    dma::TransferDma,
    gpio::{PinState, Speed},
    hal::spi::{Mode, Phase, Polarity},
    prelude::*,
    rcc::MsiFreq,
    spi::Spi,
};

#[rtic::app(device = stm32l4xx_hal::pac, peripherals = true)]
const APP: () = {
    #[init]
    fn init(cx: init::Context) {
        static mut DMA_BUF: [u8; 5] = [0xf0, 0xaa, 0x00, 0xff, 0x0f];

        rtt_target::rtt_init_print!();
        rprintln!("Initializing... ");

        let dp = cx.device;

        let mut flash = dp.FLASH.constrain();
        let mut rcc = dp.RCC.constrain();
        let mut pwr = dp.PWR.constrain(&mut rcc.apb1r1);
        let mut gpiob = dp.GPIOB.split(&mut rcc.ahb2);
        let dma1_channels = dp.DMA1.split(&mut rcc.ahb1);

        //
        // Initialize the clocks to 80 MHz
        //
        rprintln!("  - Clock init");
        let clocks = rcc
            .cfgr
            .msi(MsiFreq::RANGE4M)
            .sysclk(80.MHz())
            .freeze(&mut flash.acr, &mut pwr);

        //
        // Initialize the SPI
        //
        let sck = gpiob
            .pb3
            .into_alternate(&mut gpiob.moder, &mut gpiob.otyper, &mut gpiob.afrl)
            .set_speed(Speed::High);
        let miso = gpiob
            .pb4
            .into_alternate(&mut gpiob.moder, &mut gpiob.otyper, &mut gpiob.afrl)
            .set_speed(Speed::High);
        let mosi = gpiob
            .pb5
            .into_alternate(&mut gpiob.moder, &mut gpiob.otyper, &mut gpiob.afrl)
            .set_speed(Speed::High);
        let mut dummy_cs = gpiob.pb6.into_push_pull_output_in_state(
            &mut gpiob.moder,
            &mut gpiob.otyper,
            PinState::High,
        );
        let spi = Spi::spi1(
            dp.SPI1,
            (sck, miso, mosi),
            Mode {
                phase: Phase::CaptureOnFirstTransition,
                polarity: Polarity::IdleLow,
            },
            100.kHz(),
            clocks,
            &mut rcc.apb2,
        );

        // Create DMA SPI
        let dma_spi = spi.with_rxtx_dma(dma1_channels.2, dma1_channels.3);

        // Check the buffer before using it
        rprintln!("buf pre: 0x{:x?}", &DMA_BUF);

        // Perform transfer and wait for it to finish (blocking), this can also be done using
        // interrupts on the desired DMA channel
        dummy_cs.set_low();
        let transfer = dma_spi.transfer(DMA_BUF);
        let (buf, _dma_spi) = transfer.wait();
        dummy_cs.set_high();

        // Inspect the extracted buffer, if the MISO is connected to VCC or GND it will be all 0 or
        // 1.
        rprintln!("buf post: 0x{:x?}", &buf);
    }

    // Idle function so RTT keeps working
    #[idle]
    fn idle(_cx: idle::Context) -> ! {
        loop {
            continue;
        }
    }
};
