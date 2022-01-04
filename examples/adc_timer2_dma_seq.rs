#![no_main]
#![no_std]

use panic_rtt_target as _;
use rtt_target::{rprintln, rtt_init_print};
use stm32l4xx_hal::{
    adc::{DmaMode, SampleTime, Sequence, ADC},
    delay::DelayCM,
    dma::{dma1, RxDma, Transfer, W},
    prelude::*,
    time::Hertz,
    timer::Timer, // Event,
};

use rtic::app;

const SEQUENCE_LEN: usize = 4;

#[app(device = stm32l4xx_hal::stm32, peripherals = true, monotonic = rtic::cyccnt::CYCCNT)]
const APP: () = {
    // RTIC app is written in here!

    struct Resources {
        transfer: Option<Transfer<W, &'static mut [u16; SEQUENCE_LEN], RxDma<ADC, dma1::C1>>>,
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
        let clocks = rcc
            .cfgr
            .sysclk(Hertz(80_000_000))
            .freeze(&mut flash.acr, &mut pwr);

        let mut delay = DelayCM::new(clocks);

        let mut adc = ADC::new(
            pac.ADC1,
            pac.ADC_COMMON,
            &mut rcc.ahb2,
            &mut rcc.ccipr,
            &mut delay,
        );

        let mut temp_pin = adc.enable_temperature(&mut delay);

        let dma1_channel = dma_channels.1;
        // let adc_buffer1_addr = MEMORY.as_ptr();
        let mut gpioc = pac.GPIOC.split(&mut rcc.ahb2);
        let mut pc0 = gpioc.pc0.into_analog(&mut gpioc.moder, &mut gpioc.pupdr);

        adc.configure_sequence(&mut temp_pin, Sequence::One, SampleTime::Cycles12_5);
        adc.configure_sequence(&mut temp_pin, Sequence::Two, SampleTime::Cycles247_5);
        adc.configure_sequence(&mut temp_pin, Sequence::Three, SampleTime::Cycles640_5);
        adc.configure_sequence(&mut pc0, Sequence::Four, SampleTime::Cycles640_5);

        // optional oversampling settings
        adc.set_oversampling_ratio(7);
        adc.set_oversampling_shift(8);
        adc.oversampling_enable();

        adc.set_external_trigger(0b1011, 1_u8); // Timer2_TRGO

        // Heapless boxes also work very well as buffers for DMA transfers
        let transfer = Transfer::from_adc(adc, dma1_channel, MEMORY, DmaMode::ExtTrigger, true);

        // unsafe { NVIC::unmask(stm32l4xx_hal::stm32::Interrupt::TIM2) };
        let _timer = Timer::tim2(pac.TIM2, 1.hz(), clocks, &mut rcc.apb1r1);
        unsafe {
            // get pointer of timer 2
            let tim = &(*stm32l4::stm32l4x6::TIM2::ptr());
            // config master mode selection to TRGO to Compare Pulse of timer2
            tim.cr2.modify(|_, w| w.mms().bits(3_u8));
            tim.dier.write(|w| w.ude().set_bit());
        }

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
                DmaMode::ExtTrigger,
                true,
            ));
        }
    }
};
