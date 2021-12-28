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
        let adc_buffer1_addr = MEMORY.as_ptr();
        let mut gpioc = pac.GPIOC.split(&mut rcc.ahb2);
        let mut pc0 = gpioc.pc0.into_analog(&mut gpioc.moder, &mut gpioc.pupdr);

        adc.configure_sequence(&mut temp_pin, Sequence::One, SampleTime::Cycles12_5);
        adc.configure_sequence(&mut temp_pin, Sequence::Two, SampleTime::Cycles247_5);
        adc.configure_sequence(&mut temp_pin, Sequence::Three, SampleTime::Cycles640_5);
        adc.configure_sequence(&mut pc0, Sequence::Four, SampleTime::Cycles640_5);

        // Heapless boxes also work very well as buffers for DMA transfers
        let transfer = Transfer::from_adc(adc, dma1_channel, MEMORY, DmaMode::Oneshot, true);

        unsafe {
            let adc = &(*stm32l4::stm32l4x6::ADC1::ptr());
            let jdr1_val = adc.jdr1.read().jdata1().bits() as u16;
            let jdr2_val = adc.jdr2.read().jdata2().bits() as u16;
            let jdr3_val = adc.jdr3.read().jdata3().bits() as u16;
            let jdr4_val = adc.jdr4.read().jdata4().bits() as u16;
            let sqr1_val = adc.sqr1.read().l().bits();
            let enabled = adc.cr.read().aden().bit_is_set();
            let inj_end = adc.isr.read().jeoc().bit_is_set(); // injecte adc conversion finished
            let inj_seq_end = adc.isr.read().jeos().bit_is_set(); // injecte sequence conversion finished
            rprintln!("jdr1: {}", jdr1_val);
            rprintln!("jdr2: {}", jdr2_val);
            rprintln!("jdr3: {}", jdr3_val);
            rprintln!("jdr4: {}", jdr4_val);
            rprintln!("inj_end: {}", inj_end);
            rprintln!("inj_seq_end: {}", inj_seq_end);
            adc.isr.modify(|_, w| w.jeos().set_bit());
            rprintln!("DMA measurements_pointer at: {:?}", *adc_buffer1_addr);
            rprintln!("DMA measurements_pointer at: {:?}", adc_buffer1_addr);
            rprintln!("DMA measurements_pointer at: {:?}", adc_buffer1_addr as u32);

            let dma1 = &(*stm32l4::stm32l4x6::DMA1::ptr());
            rprintln!(
                "dma1.cselr.read().c1s():    {}",
                dma1.cselr.read().c1s().bits()
            );
            rprintln!(
                "dma1.ccr1.read().pl():      {}",
                dma1.ccr1.read().pl().bits()
            );
            rprintln!(
                "dma1.ccr1.read().msize():   {}",
                dma1.ccr1.read().msize().bits()
            );
            rprintln!(
                "ddma1.ccr1.read().psize():  {}",
                dma1.ccr1.read().psize().bits()
            );
            rprintln!(
                "dma1.ccr1.read().dir():     {}",
                dma1.ccr1.read().dir().bits()
            );
            rprintln!(
                "dma1.ccr1.read().minc():    {}",
                dma1.ccr1.read().minc().bits()
            );
            rprintln!(
                "dma1.ccr1.read().pinc():    {}",
                dma1.ccr1.read().pinc().bits()
            );
            rprintln!(
                "dma1.ccr1.read().en():      {}",
                dma1.ccr1.read().en().bits()
            );
            rprintln!(
                "dma1.ccr1.read().tcie():    {}",
                dma1.ccr1.read().tcie().bits()
            );
            rprintln!(
                "dma1.ccr1.read().htie():    {}",
                dma1.ccr1.read().htie().bits()
            );
            rprintln!(
                "dma1.ccr1.read().teie():    {}",
                dma1.ccr1.read().teie().bits()
            );

            rprintln!(
                "dma1.isr.read().gif1():     {}",
                dma1.isr.read().gif1().bits()
            );
            rprintln!(
                "dma1.isr.read().tcif1():    {}",
                dma1.isr.read().tcif1().bits()
            );
            rprintln!(
                "dma1.isr.read().htif1():    {}",
                dma1.isr.read().htif1().bits()
            );
            rprintln!(
                "dma1.isr.read().teif1():    {}",
                dma1.isr.read().teif1().bits()
            );
            rprintln!(
                "dma1.cdntr1.read().ndt():    {}",
                dma1.cndtr1.read().ndt().bits()
            );

            // dma1.ccr1.modify(|_, w| w.en().clear_bit());
            // dma1.ccr1.modify(|_, w| w.tcie().set_bit());
            // dma1.ccr1.modify(|_, w| w.teie().set_bit());

            rprintln!(
                "adc.cfgr.read().dmaen():    {}",
                adc.cfgr.read().dmaen().bits()
            );
            rprintln!(
                "adc.cfgr.read().dmacfg():   {}",
                adc.cfgr.read().dmacfg().bits()
            );
            // rprintln!("adc.cfgr.read().dfsdmcfg():    {}", adc.cfgr.read().dfsdmcfg().bits());
            rprintln!(
                "adc.isr.read().ovr():       {}",
                adc.isr.read().ovr().bits()
            );

            rprintln!(
                "adc.ier.read().ovrie():       {}",
                adc.ier.read().ovrie().bits()
            );
            rprintln!(
                "adc.ier.read().adrdyie():       {}",
                adc.ier.read().adrdyie().bits()
            );
            rprintln!(
                "adc.ier.read().eosmpie():       {}",
                adc.ier.read().eosmpie().bits()
            );
            rprintln!(
                "adc.ier.read().eocie():       {}",
                adc.ier.read().eocie().bits()
            );
            rprintln!(
                "adc.ier.read().eosie():       {}",
                adc.ier.read().eosie().bits()
            );
            rprintln!(
                "adc.ier.read().eocie():       {}",
                adc.ier.read().eocie().bits()
            );
            rprintln!(
                "adc.ier.read().jeocie():       {}",
                adc.ier.read().jeocie().bits()
            );
            rprintln!(
                "adc.ier.read().jeosie():       {}",
                adc.ier.read().jeosie().bits()
            );
            rprintln!(
                "adc.ier.read().awd1ie():       {}",
                adc.ier.read().awd1ie().bits()
            );
            rprintln!(
                "adc.ier.read().awd2ie():       {}",
                adc.ier.read().awd2ie().bits()
            );
            rprintln!(
                "adc.ier.read().awd3ie():       {}",
                adc.ier.read().awd3ie().bits()
            );
            rprintln!(
                "adc.ier.read().jqovfie():       {}",
                adc.ier.read().jqovfie().bits()
            );
            // dma1.ccr1.modify(|_, w| w.en().set_bit());
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
                DmaMode::Oneshot,
                true,
            ));
        }
    }
};
