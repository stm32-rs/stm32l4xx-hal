#![no_main]
#![no_std]

use panic_rtt_target as _;
use rtt_target::{rprintln, rtt_init_print};
use stm32l4xx_hal::{
    adc::{config, DmaMode, SampleTime, Sequence, ADC},
    delay::DelayCM,
    dma::{dma1, RxDma, Transfer, W},
    timer::{Timer, Event, MasterMode},
    prelude::*,
    pac::{TIM2},
};

use rtic::app;

const SEQUENCE_LEN: usize = 8;

#[app(device = stm32l4xx_hal::stm32, peripherals = true, monotonic = rtic::cyccnt::CYCCNT)]
const APP: () = {
    // RTIC app is written in here!

    struct Resources {
        transfer: Option<Transfer<W, &'static mut [u16; SEQUENCE_LEN], RxDma<ADC, dma1::C1>>>,
        timer: Timer<TIM2>,
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

        let mut adc = ADC::new(
            pac.ADC1,
            pac.ADC_COMMON,
            &mut rcc.ahb2,
            &mut rcc.ccipr,
            &mut delay,
            config::ExternalTriggerConfig(
                config::TriggerMode::RisingEdge,
                config::ExternalTrigger::Tim_2_trgo,
            ),
        );

        let mut timer_adc_trg = Timer::tim2(pac.TIM2, 1.Hz(), clocks, &mut rcc.apb1r1);
        timer_adc_trg.master_mode(MasterMode::Update);
        timer_adc_trg.listen(Event::TimeOut);

        let dma1_channel = dma_channels.1;

        let mut gpioa = pac.GPIOA.split(&mut rcc.ahb2);
        let mut a1 = gpioa.pa0.into_analog(&mut gpioa.moder, &mut gpioa.pupdr);

        adc.configure_sequence(&mut a1, Sequence::One, SampleTime::Cycles12_5);

        // Heapless boxes also work very well as buffers for DMA transfers
        let transfer = Transfer::from_adc(adc, dma1_channel, MEMORY, DmaMode::Oneshot, true, false);

        init::LateResources {
            transfer: Some(transfer),
            timer: timer_adc_trg,
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
                false,
            ));
        }
    }

    #[task(binds = TIM2, resources = [timer])]
    fn tim2_interrupt(cx: tim2_interrupt::Context) {
        let timer = cx.resources.timer;
        timer.clear_interrupt(stm32l4xx_hal::timer::Event::TimeOut);
        rprintln!("TIM2 int - ADC trigger");
    }
};
