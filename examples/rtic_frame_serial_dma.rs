//! Test the serial interface with the frame DMA engine in RTIC. This will echo frames sent to the
//! board via the debuggers VCP.
//!
//! This is tested on Nucleo-64 STM32L412 over the debuggers VCP.
//!
//! This example only compiles for some targets so it is not part of the CI for now.

#![deny(unsafe_code)]
// #![deny(warnings)]
#![no_main]
#![no_std]

use hal::{
    dma::{self, DMAFrame, FrameReader, FrameSender},
    pac::USART2,
    prelude::*,
    rcc::{ClockSecuritySystem, CrystalBypass, MsiFreq},
    serial::{self, Config, Serial},
};
use heapless::{
    pool,
    pool::singleton::{Box, Pool},
};
use panic_halt as _;
use rtic::app;
use stm32l4xx_hal as hal;
use stm32l4xx_hal::dma::{RxDma, TxDma};
use stm32l4xx_hal::serial::{Rx, Tx};

// The pool gives out `Box<DMAFrame>`s that can hold 8 bytes
pool!(
    #[allow(non_upper_case_globals)]
    SerialDMAPool: DMAFrame<8>
);

#[app(device = stm32l4xx_hal::stm32, peripherals = true)]
const APP: () = {
    struct Resources {
        frame_reader: FrameReader<Box<SerialDMAPool>, RxDma<Rx<USART2>, dma::dma1::C6>, 8>,
        frame_sender: FrameSender<Box<SerialDMAPool>, TxDma<Tx<USART2>, dma::dma1::C7>, 8>,
    }

    #[init]
    fn init(cx: init::Context) -> init::LateResources {
        static mut MEMORY: [u8; 1024] = [0; 1024];

        // increase the capacity of the pool by ~8 blocks
        SerialDMAPool::grow(MEMORY);

        let dp = cx.device;

        let mut flash = dp.FLASH.constrain();
        let mut rcc = dp.RCC.constrain();
        let mut pwr = dp.PWR.constrain(&mut rcc.apb1r1);
        let mut gpioa = dp.GPIOA.split(&mut rcc.ahb2);

        // Set the clocks to 80 MHz
        let clocks = rcc
            .cfgr
            .lse(CrystalBypass::Disable, ClockSecuritySystem::Disable)
            .msi(MsiFreq::RANGE4M)
            .sysclk(80.mhz())
            .freeze(&mut flash.acr, &mut pwr);

        // USART2 pins
        let tx2 = gpioa.pa2.into_af7(&mut gpioa.moder, &mut gpioa.afrl);
        let rx2 = gpioa.pa3.into_af7(&mut gpioa.moder, &mut gpioa.afrl);

        // We will listen for the character `a`, this can be changed to any character such as `\0`
        // if using COBS encoding, or `\n` if using string encoding.
        let mut serial = Serial::usart2(
            dp.USART2,
            (tx2, rx2),
            Config::default()
                .baudrate(115_200.bps())
                .character_match(b'a'),
            clocks,
            &mut rcc.apb1r1,
        );
        serial.listen(serial::Event::CharacterMatch);
        let (serial_tx, serial_rx) = serial.split();

        let channels = dp.DMA1.split(&mut rcc.ahb1);
        let mut dma_ch6 = channels.6;
        let mut dma_ch7 = channels.7;
        dma_ch6.listen(dma::Event::TransferComplete);
        dma_ch7.listen(dma::Event::TransferComplete);

        // Serial frame reader (DMA based), give it a buffer to start reading into
        let fr = if let Some(dma_buf) = SerialDMAPool::alloc() {
            // Set up the first reader frame
            let dma_buf = dma_buf.init(DMAFrame::new());
            serial_rx.with_dma(dma_ch6).frame_reader(dma_buf)
        } else {
            unreachable!()
        };

        // Serial frame sender (DMA based)
        let fs: FrameSender<Box<SerialDMAPool>, _, 8> = serial_tx.with_dma(dma_ch7).frame_sender();

        init::LateResources {
            frame_reader: fr,
            frame_sender: fs,
        }
    }

    /// This task handles the character match interrupt at required by the `FrameReader`
    ///
    /// It will echo the buffer back to the serial.
    #[task(binds = USART2, resources = [frame_reader, frame_sender], priority = 3)]
    fn serial_isr(cx: serial_isr::Context) {
        // Check for character match
        if cx.resources.frame_reader.check_character_match(true) {
            if let Some(dma_buf) = SerialDMAPool::alloc() {
                let dma_buf = dma_buf.init(DMAFrame::new());
                let buf = cx.resources.frame_reader.character_match_interrupt(dma_buf);

                // Echo the buffer back over the serial
                cx.resources.frame_sender.send(buf).ok();
            }
        }
    }

    /// This task handles the RX transfer complete interrupt at required by the `FrameReader`
    ///
    /// In this case we are discarding if a frame gets full as no character match was received
    #[task(binds = DMA1_CH6, resources = [frame_reader], priority = 3)]
    fn serial_rx_dma_isr(cx: serial_rx_dma_isr::Context) {
        if let Some(dma_buf) = SerialDMAPool::alloc() {
            let dma_buf = dma_buf.init(DMAFrame::new());

            // Erroneous packet as it did not fit in a buffer, throw away the buffer
            let _buf = cx
                .resources
                .frame_reader
                .transfer_complete_interrupt(dma_buf);
        }
    }

    /// This task handles the TX transfer complete interrupt at required by the `FrameSender`
    #[task(binds = DMA1_CH7, resources = [frame_sender], priority = 3)]
    fn serial_tx_dma_isr(cx: serial_tx_dma_isr::Context) {
        let fs = cx.resources.frame_sender;

        if let Some(_buf) = fs.transfer_complete_interrupt() {
            // Frame sent, drop the buffer to return it too the pool
        }

        // Send a new buffer
        // fs.send(buffer);
    }
};
