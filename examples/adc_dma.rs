#![no_main]
#![no_std]

use panic_rtt_target as _;
use rtt_target::{rprintln, rtt_init_print};
use stm32l4xx_hal::{
    adc::{Adc, AdcCommon, DmaMode, SampleTime, Sequence},
    delay::DelayCM,
    dma::{dma1, RxDma, Transfer, W},
    pac::ADC1,
    prelude::*,
};

use rtic::app;

const SEQUENCE_LEN: usize = 3;

#[app(device = stm32l4xx_hal::stm32, peripherals = true, monotonic = rtic::cyccnt::CYCCNT)]
const APP: () = {
    // RTIC app is written in here!

    struct Resources {
        transfer: Option<Transfer<W, &'static mut [u16; SEQUENCE_LEN], RxDma<Adc<ADC1>, dma1::C1>>>,
    }

    #[init]
    fn init(cx: init::Context) -> init::LateResources {
        let MEMORY = {
            static mut MEMORY: [u16; SEQUENCE_LEN] = [0u16; SEQUENCE_LEN];
            unsafe { &mut MEMORY }
        };

        rtt_init_print!();

        rprintln!("Hello from init!");

        let cp = cx.core;
        let mut dcb = cp.DCB;
        let mut dwt = cp.DWT;

        dcb.enable_trace();
        dwt.enable_cycle_counter();

        let pac = cx.device;

        let mut rcc = pac.RCC.constrain();
        let mut flash = pac.FLASH.constrain();
        let mut pwr = pac.PWR.constrain(&mut rcc.apb1r1);
        let dma_channels = pac.DMA1.split(&mut rcc.ahb1);

        //
        // Initialize the clocks
        //
        let clocks = rcc.cfgr.sysclk(80.MHz()).freeze(&mut flash.acr, &mut pwr);

        let mut delay = DelayCM::new(clocks);

        let adc_common = AdcCommon::new(pac.ADC_COMMON, &mut rcc.ahb2);
        let mut adc = Adc::adc1(pac.ADC1, adc_common, &mut rcc.ccipr, &mut delay);

        let mut temp_pin = adc.enable_temperature(&mut delay).unwrap();

        let dma1_channel = dma_channels.1;

        adc.configure_sequence(&mut temp_pin, Sequence::One, SampleTime::Cycles12_5);
        adc.configure_sequence(&mut temp_pin, Sequence::Two, SampleTime::Cycles247_5);
        adc.configure_sequence(&mut temp_pin, Sequence::Three, SampleTime::Cycles640_5);

        // Heapless boxes also work very well as buffers for DMA transfers
        let transfer = Transfer::from_adc(adc, dma1_channel, MEMORY, DmaMode::Oneshot, true);

        init::LateResources {
            transfer: Some(transfer),
        }
    }

    #[idle]
    fn idle(_cx: idle::Context) -> ! {
        loop {
            cortex_m::asm::nop();
        }
    }

    #[task(binds = DMA1_CH1, resources = [transfer])]
    fn dma1_interrupt(cx: dma1_interrupt::Context) {
        let transfer = cx.resources.transfer;
        if let Some(transfer_val) = transfer.take() {
            let (buffer, rx_dma) = transfer_val.wait();
            rprintln!("DMA measurements: {:?}", buffer);
            *transfer = Some(Transfer::from_adc_dma(
                rx_dma,
                buffer,
                DmaMode::Oneshot,
                true,
            ));
        }
    }
};
