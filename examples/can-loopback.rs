//! Run the bxCAN peripheral in loopback mode.

#![deny(unsafe_code)]
#![deny(warnings)]
#![no_main]
#![no_std]

use bxcan::{
    filter::Mask32,
    {Frame, StandardId},
};
use panic_halt as _;
use rtic::app;
use rtt_target::{rprintln, rtt_init_print};
use stm32l4xx_hal::{can::Can, prelude::*};

#[app(device = stm32l4xx_hal::stm32, peripherals = true)]
const APP: () = {
    #[init]
    fn init(cx: init::Context) {
        rtt_init_print!();

        let dp = cx.device;

        let mut flash = dp.FLASH.constrain();
        let mut rcc = dp.RCC.constrain();
        let mut pwr = dp.PWR.constrain(&mut rcc.apb1r1);
        let mut gpioa = dp.GPIOA.split(&mut rcc.ahb2);

        // Set the clocks to 80 MHz
        let _clocks = rcc.cfgr.sysclk(80.MHz()).freeze(&mut flash.acr, &mut pwr);

        rprintln!("  - CAN init");

        let can = {
            let rx =
                gpioa
                    .pa11
                    .into_alternate(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrh);
            let tx =
                gpioa
                    .pa12
                    .into_alternate(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrh);

            let can = Can::new(&mut rcc.apb1r1, dp.CAN1, (tx, rx));

            bxcan::Can::builder(can)
        }
        // APB1 (PCLK1): 80 MHz, Bit rate: 100kBit/s, Sample Point 87.5%
        // Value was calculated with http://www.bittiming.can-wiki.info/
        .set_bit_timing(0x001c_0031)
        .set_loopback(true);

        // Enable and wait for bxCAN sync to bus
        let mut can = can.enable();

        // Configure filters so that can frames can be received.
        let mut filters = can.modify_filters();
        filters.enable_bank(0, Mask32::accept_all());

        // Drop filters to leave filter configuraiton mode.
        drop(filters);

        // Send a frame
        let mut test: [u8; 8] = [0; 8];
        let id: u16 = 0x500;

        test[0] = 72;
        test[1] = 1;
        test[2] = 2;
        test[3] = 3;
        test[4] = 4;
        test[5] = 5;
        test[6] = 6;
        test[7] = 7;
        let test_frame = Frame::new_data(StandardId::new(id).unwrap(), test);
        can.transmit(&test_frame).unwrap();

        // Wait for TX to finish
        while !can.is_transmitter_idle() {}

        rprintln!("  - CAN tx complete: {:?}", test_frame);

        // Receive the packet back
        let r = can.receive();

        rprintln!("  - CAN rx {:?}", r);

        assert_eq!(Ok(test_frame), r);
    }

    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop {
            continue;
        }
    }
};
