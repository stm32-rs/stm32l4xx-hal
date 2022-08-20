#![no_main]
#![no_std]

extern crate panic_rtt_target;

use heapless::{consts::U8, spsc};
use nb::block;
use rtt_target::{rprint, rprintln};
use stm32l4xx_hal::{
    pac::{self, LPUART1},
    prelude::*,
    serial::{self, Config, Serial},
};

#[rtic::app(device = stm32l4xx_hal::pac)]
const APP: () = {
    struct Resources {
        rx: serial::Rx<LPUART1>,
        tx: serial::Tx<LPUART1>,
        //
        // rx_prod: spsc::Producer<'static, u8, U8>,
        // rx_cons: spsc::Consumer<'static, u8, U8>,
    }

    #[init]
    fn init(_: init::Context) -> init::LateResources {
        static mut RX_QUEUE: spsc::Queue<u8, U8> = spsc::Queue(heapless::i::Queue::new());

        rtt_target::rtt_init_print!();
        rprint!("Initializing... ");

        let p = pac::Peripherals::take().unwrap();

        let mut rcc = p.RCC.constrain();
        let mut flash = p.FLASH.constrain();
        let mut pwr = p.PWR.constrain(&mut rcc.apb1r1);

        let clocks = rcc.cfgr.freeze(&mut flash.acr, &mut pwr);

        let mut gpioa = p.GPIOA.split(&mut rcc.ahb2);
        let mut gpioc = p.GPIOC.split(&mut rcc.ahb2);

        let tx_pin = gpioc
            .pc1
            .into_alternate(&mut gpioc.moder, &mut gpioc.otyper, &mut gpioc.afrl);
        let rx_pin = gpioc
            .pc0
            .into_alternate(&mut gpioc.moder, &mut gpioc.otyper, &mut gpioc.afrl);

        let mut serial = Serial::lpuart1(
            p.LPUART1,
            (tx_pin, rx_pin),
            Config::default().baudrate(115_200.bps()),
            clocks,
            &mut rcc.apb1r2,
        );
        serial.listen(serial::Event::Rxne);

        let (tx, rx) = serial.split();
        let (rx_prod, rx_cons) = RX_QUEUE.split();

        rprintln!("done.");

        init::LateResources {
            rx,
            tx,
            //
            // rx_prod,
            // rx_cons,
        }
    }

    #[idle(resources = [tx])]
    fn idle(cx: idle::Context) -> ! {
        // let rx = cx.resources.rx_cons;
        let tx = cx.resources.tx;

        loop {
            // if let Some(b) = rx.dequeue() {
            //     rprintln!("Echoing '{}'", b as char);
            //     block!(tx.write(b)).unwrap();
            // }
            block!(tx.write('x' as u8)).unwrap();
            cortex_m::asm::delay(1_000_000);
        }
    }

    #[task(binds = LPUART1, resources = [rx])]
    fn usart2(cx: usart2::Context) {
        let rx = cx.resources.rx;
        // let queue = cx.resources.rx_prod;

        let b = match rx.read() {
            Ok(b) => {
                rprintln!("Read: {}", b);
            },
            Err(err) => {
                rprintln!("Error reading from USART: {:?}", err);
                return;
            }
        };
        // match queue.enqueue(b) {
        //     Ok(()) => (),
        //     Err(err) => {
        //         rprintln!("Error adding received byte to queue: {:?}", err);
        //         return;
        //     }
        // }
    }
};
