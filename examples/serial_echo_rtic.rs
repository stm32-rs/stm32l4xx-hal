#![no_main]
#![no_std]

extern crate panic_rtt_target;

use nb::block;
use rtt_target::{
    rprint,
    rprintln,
};
use stm32l4xx_hal::{
    prelude::*,
    pac::{
        self,
        USART2,
    },
    serial::{
        self,
        Config,
        Serial,
    },
};

#[rtic::app(device = stm32l4xx_hal::pac)]
const APP: () = {
    struct Resources {
        rx: serial::Rx<USART2>,
        tx: serial::Tx<USART2>,
    }

    #[init]
    fn init(_: init::Context) -> init::LateResources {
        rtt_target::rtt_init_print!();
        rprint!("Initializing... ");

        let p = pac::Peripherals::take().unwrap();

        let mut rcc = p.RCC.constrain();
        let mut flash = p.FLASH.constrain();
        let mut pwr = p.PWR.constrain(&mut rcc.apb1r1);

        let clocks = rcc.cfgr.freeze(&mut flash.acr, &mut pwr);

        let mut gpioa = p.GPIOA.split(&mut rcc.ahb2);

        let tx_pin = gpioa.pa2.into_af7(&mut gpioa.moder, &mut gpioa.afrl);
        let rx_pin = gpioa.pa3.into_af7(&mut gpioa.moder, &mut gpioa.afrl);

        let serial = Serial::usart2(
            p.USART2,
            (tx_pin, rx_pin),
            Config::default()
                .baudrate(115_200.bps()),
            clocks,
            &mut rcc.apb1r1,
        );
        let (tx, rx) = serial.split();

        rprintln!("done.");

        init::LateResources {
            rx,
            tx,
        }
    }

    #[idle(resources = [rx, tx])]
    fn idle(cx: idle::Context) -> ! {
        let rx = cx.resources.rx;
        let tx = cx.resources.tx;

        loop {
            let b = block!(rx.read()).unwrap();
            block!(tx.write(b)).unwrap();
        }
    }
};
