//! Direct Memory Access Multiplexing

#![allow(dead_code)]
#![allow(non_camel_case_types)]

use core::convert::TryFrom;
use core::convert::TryInto;

use crate::dma::{dma1, dma2};

#[cfg(any(
    // feature = "stm32l4p5",
    // feature = "stm32l4q5",
    feature = "stm32l4r5",
    feature = "stm32l4s5",
    // feature = "stm32l4r7",
    // feature = "stm32l4s7",
    feature = "stm32l4r9",
    feature = "stm32l4s9"
))]
use crate::stm32::DMAMUX1 as DMAMUX;

#[non_exhaustive]
#[derive(Debug)]
pub enum Error {
    Invalid,
}

/// Input DMA request line selected
pub enum DmaInput {
    Generator0,
    Generator1,
    Generator2,
    Generator3,
    Adc1,
    Adc2,
    Adc3,
    Dac1Ch1,
    Dac1Ch2,
    Spi1Rx,
    Spi1Tx,
    Spi2Rx,
    Spi2Tx,
    Spi3Rx,
    Spi3Tx,
    I2c1Rx,
    I2c1Tx,
    I2c2Rx,
    I2c2Tx,
    I2c3Rx,
    I2c3Tx,
    I2c4Rx,
    I2c4Tx,
    Usart1Rx,
    Usart1Tx,
    Usart2Rx,
    Usart2Tx,
    Usart3Rx,
    Usart3Tx,
    Uart4Rx,
    Uart4Tx,
    Uart5Rx,
    Uart5Tx,
    LpUart1Rx,
    LpUart1Tx,
    Sai1A,
    Sai1B,
    Sai2A,
    Sai2B,
    QuadSpi,
    OctoSpi1,
    OctoSpi2,
    Tim6Up,
    Tim7Up,
    Tim1Ch1,
    Tim1Ch2,
    Tim1Ch3,
    Tim1Ch4,
    Tim1Up,
    Tim1Trig,
    Tim1Com,
    Tim8Ch1,
    Tim8Ch2,
    Tim8Ch3,
    Tim8Ch4,
    Tim8Up,
    Tim8Trig,
    Tim8Com,
    Tim2Ch1,
    Tim2Ch2,
    Tim2Ch3,
    Tim2Ch4,
    Tim2Up,
    Tim3Ch1,
    Tim3Ch2,
    Tim3Ch3,
    Tim3Ch4,
    Tim3Up,
    Tim3Trig,
    Tim4Ch1,
    Tim4Ch2,
    Tim4Ch3,
    Tim4Ch4,
    Tim4Up,
    Tim5Ch1,
    Tim5Ch2,
    Tim5Ch3,
    Tim5Ch4,
    Tim5Up,
    Tim5Trig,
    Tim15Ch1,
    Tim15Up,
    Tim15Trig,
    Tim15Com,
    Tim16Ch1,
    Tim16Up,
    Tim17Ch1,
    Tim17Up,
    Dfsdm1Flt0,
    Dfsdm1Flt1,
    Dfsdm1Flt2,
    Dfsdm1Flt3,
    Dcmi,
    AesIn,
    AesOut,
    HashIn,
    Swpmi1Rx,
    Swpmi1Tx,
    SdMmc1,
}

#[cfg(any(
    // feature = "stm32l4p5",
    // feature = "stm32l4q5",
    feature = "stm32l4r5",
    feature = "stm32l4s5",
    // feature = "stm32l4r7",
    // feature = "stm32l4s7",
    feature = "stm32l4r9",
    feature = "stm32l4s9"
))]
#[derive(Clone, Copy, Debug, PartialEq)]
#[repr(u8)]
enum DMAREQ_ID_A {
    NONE = 0,
    GENERATOR0 = 1,
    GENERATOR1 = 2,
    GENERATOR2 = 3,
    GENERATOR3 = 4,
    ADC1 = 5,
    DAC1_CH1 = 6,
    DAC1_CH2 = 7,
    TIM6_UP = 8,
    TIM7_UP = 9,
    SPI1_RX = 10,
    SPI1_TX = 11,
    SPI2_RX = 12,
    SPI2_TX = 13,
    SPI3_RX = 14,
    SPI3_TX = 15,
    I2C1_RX = 16,
    I2C1_TX = 17,
    I2C2_RX = 18,
    I2C2_TX = 19,
    I2C3_RX = 20,
    I2C3_TX = 21,
    I2C4_RX = 22,
    I2C4_TX = 23,
    USART1_RX = 24,
    USART1_TX = 25,
    USART2_RX = 26,
    USART2_TX = 27,
    USART3_RX = 28,
    USART3_TX = 29,
    UART4_RX = 30,
    UART4_TX = 31,
    UART5_RX = 32,
    UART5_TX = 33,
    LPUART1_RX = 34,
    LPUART1_TX = 35,
    SAI1_A = 36,
    SAI1_B = 37,
    SAI2_A = 38,
    SAI2_B = 39,
    OCTOSPI1 = 40,
    OCTOSPI2 = 41,
    TIM1_CH1 = 42,
    TIM1_CH2 = 43,
    TIM1_CH3 = 44,
    TIM1_CH4 = 45,
    TIM1_UP = 46,
    TIM1_TRIG = 47,
    TIM1_COM = 48,
    TIM8_CH1 = 49,
    TIM8_CH2 = 50,
    TIM8_CH3 = 51,
    TIM8_CH4 = 52,
    TIM8_UP = 53,
    TIM8_TRIG = 54,
    TIM8_COM = 55,
    TIM2_CH1 = 56,
    TIM2_CH2 = 57,
    TIM2_CH3 = 58,
    TIM2_CH4 = 59,
    TIM2_UP = 60,
    TIM3_CH1 = 61,
    TIM3_CH2 = 62,
    TIM3_CH3 = 63,
    TIM3_CH4 = 64,
    TIM3_UP = 65,
    TIM3_TRIG = 66,
    TIM4_CH1 = 67,
    TIM4_CH2 = 68,
    TIM4_CH3 = 69,
    TIM4_CH4 = 70,
    TIM4_UP = 71,
    TIM5_CH1 = 72,
    TIM5_CH2 = 73,
    TIM5_CH3 = 74,
    TIM5_CH4 = 75,
    TIM5_UP = 76,
    TIM5_TRIG = 77,
    TIM15_CH1 = 78,
    TIM15_UP = 79,
    TIM15_TRIG = 80,
    TIM15_COM = 81,
    TIM16_CH1 = 82,
    TIM16_UP = 83,
    TIM17_CH1 = 84,
    TIM17_UP = 85,
    DFSDM1_FLT0 = 86,
    DFSDM1_FLT1 = 87,
    DFSDM1_FLT2 = 88,
    DFSDM1_FLT3 = 89,
    DCMI = 90,
    AES_IN = 91,
    AES_OUT = 92,
    HASH_IN = 93,
}
#[cfg(any(
    // feature = "stm32l4p5",
    // feature = "stm32l4q5",
    feature = "stm32l4r5",
    feature = "stm32l4s5",
    // feature = "stm32l4r7",
    // feature = "stm32l4s7",
    feature = "stm32l4r9",
    feature = "stm32l4s9"
))]
impl From<DMAREQ_ID_A> for u8 {
    #[inline(always)]
    fn from(variant: DMAREQ_ID_A) -> Self {
        variant as _
    }
}
#[cfg(any(
    // feature = "stm32l4p5",
    // feature = "stm32l4q5",
    feature = "stm32l4r5",
    feature = "stm32l4s5",
    // feature = "stm32l4r7",
    // feature = "stm32l4s7",
    feature = "stm32l4r9",
    feature = "stm32l4s9"
))]
impl TryFrom<DmaInput> for DMAREQ_ID_A {
    type Error = Error;

    #[inline(always)]
    fn try_from(variant: DmaInput) -> Result<Self, Self::Error> {
        let result = match variant {
            DmaInput::Generator0 => Self::GENERATOR0,
            DmaInput::Generator1 => Self::GENERATOR1,
            DmaInput::Generator2 => Self::GENERATOR2,
            DmaInput::Generator3 => Self::GENERATOR3,
            DmaInput::Adc1 => Self::ADC1,
            DmaInput::Dac1Ch1 => Self::DAC1_CH1,
            DmaInput::Dac1Ch2 => Self::DAC1_CH2,
            DmaInput::Tim6Up => Self::TIM6_UP,
            DmaInput::Tim7Up => Self::TIM7_UP,
            DmaInput::Spi1Rx => Self::SPI1_RX,
            DmaInput::Spi1Tx => Self::SPI1_TX,
            DmaInput::Spi2Rx => Self::SPI2_RX,
            DmaInput::Spi2Tx => Self::SPI2_TX,
            DmaInput::Spi3Rx => Self::SPI3_RX,
            DmaInput::Spi3Tx => Self::SPI3_TX,
            DmaInput::I2c1Rx => Self::I2C1_RX,
            DmaInput::I2c1Tx => Self::I2C1_TX,
            DmaInput::I2c2Rx => Self::I2C2_RX,
            DmaInput::I2c2Tx => Self::I2C2_TX,
            DmaInput::I2c3Rx => Self::I2C3_RX,
            DmaInput::I2c3Tx => Self::I2C3_TX,
            DmaInput::I2c4Rx => Self::I2C4_RX,
            DmaInput::I2c4Tx => Self::I2C4_TX,
            DmaInput::Usart1Rx => Self::USART1_RX,
            DmaInput::Usart1Tx => Self::USART1_TX,
            DmaInput::Usart2Rx => Self::USART2_RX,
            DmaInput::Usart2Tx => Self::USART2_TX,
            DmaInput::Usart3Rx => Self::USART3_RX,
            DmaInput::Usart3Tx => Self::USART3_TX,
            DmaInput::Uart4Rx => Self::UART4_RX,
            DmaInput::Uart4Tx => Self::UART4_TX,
            DmaInput::Uart5Rx => Self::UART5_RX,
            DmaInput::Uart5Tx => Self::UART5_TX,
            DmaInput::LpUart1Rx => Self::LPUART1_RX,
            DmaInput::LpUart1Tx => Self::LPUART1_TX,
            DmaInput::Sai1A => Self::SAI1_A,
            DmaInput::Sai1B => Self::SAI1_B,
            DmaInput::Sai2A => Self::SAI2_A,
            DmaInput::Sai2B => Self::SAI2_B,
            DmaInput::OctoSpi1 => Self::OCTOSPI1,
            DmaInput::OctoSpi2 => Self::OCTOSPI2,
            DmaInput::Tim1Ch1 => Self::TIM1_CH1,
            DmaInput::Tim1Ch2 => Self::TIM1_CH2,
            DmaInput::Tim1Ch3 => Self::TIM1_CH3,
            DmaInput::Tim1Ch4 => Self::TIM1_CH4,
            DmaInput::Tim1Up => Self::TIM1_UP,
            DmaInput::Tim1Trig => Self::TIM1_TRIG,
            DmaInput::Tim1Com => Self::TIM1_COM,
            DmaInput::Tim8Ch1 => Self::TIM8_CH1,
            DmaInput::Tim8Ch2 => Self::TIM8_CH2,
            DmaInput::Tim8Ch3 => Self::TIM8_CH3,
            DmaInput::Tim8Ch4 => Self::TIM8_CH4,
            DmaInput::Tim8Up => Self::TIM8_UP,
            DmaInput::Tim8Trig => Self::TIM8_TRIG,
            DmaInput::Tim8Com => Self::TIM8_COM,
            DmaInput::Tim2Ch1 => Self::TIM2_CH1,
            DmaInput::Tim2Ch2 => Self::TIM2_CH2,
            DmaInput::Tim2Ch3 => Self::TIM2_CH3,
            DmaInput::Tim2Ch4 => Self::TIM2_CH4,
            DmaInput::Tim2Up => Self::TIM2_UP,
            DmaInput::Tim3Ch1 => Self::TIM3_CH1,
            DmaInput::Tim3Ch2 => Self::TIM3_CH2,
            DmaInput::Tim3Ch3 => Self::TIM3_CH3,
            DmaInput::Tim3Ch4 => Self::TIM3_CH4,
            DmaInput::Tim3Up => Self::TIM3_UP,
            DmaInput::Tim3Trig => Self::TIM3_TRIG,
            DmaInput::Tim4Ch1 => Self::TIM4_CH1,
            DmaInput::Tim4Ch2 => Self::TIM4_CH2,
            DmaInput::Tim4Ch3 => Self::TIM4_CH3,
            DmaInput::Tim4Ch4 => Self::TIM4_CH4,
            DmaInput::Tim4Up => Self::TIM4_UP,
            DmaInput::Tim5Ch1 => Self::TIM5_CH1,
            DmaInput::Tim5Ch2 => Self::TIM5_CH2,
            DmaInput::Tim5Ch3 => Self::TIM5_CH3,
            DmaInput::Tim5Ch4 => Self::TIM5_CH4,
            DmaInput::Tim5Up => Self::TIM5_UP,
            DmaInput::Tim5Trig => Self::TIM5_TRIG,
            DmaInput::Tim15Ch1 => Self::TIM15_CH1,
            DmaInput::Tim15Up => Self::TIM15_UP,
            DmaInput::Tim15Trig => Self::TIM15_TRIG,
            DmaInput::Tim15Com => Self::TIM15_COM,
            DmaInput::Tim16Ch1 => Self::TIM16_CH1,
            DmaInput::Tim16Up => Self::TIM16_UP,
            DmaInput::Tim17Ch1 => Self::TIM17_CH1,
            DmaInput::Tim17Up => Self::TIM17_UP,
            DmaInput::Dfsdm1Flt0 => Self::DFSDM1_FLT0,
            DmaInput::Dfsdm1Flt1 => Self::DFSDM1_FLT1,
            DmaInput::Dfsdm1Flt2 => Self::DFSDM1_FLT2,
            DmaInput::Dfsdm1Flt3 => Self::DFSDM1_FLT3,
            DmaInput::Dcmi => Self::DCMI,
            DmaInput::AesIn => Self::AES_IN,
            DmaInput::AesOut => Self::AES_OUT,
            DmaInput::HashIn => Self::HASH_IN,
            _ => return Err(Error::Invalid),
        };

        Ok(result)
    }
}

#[cfg(not(any(
    // feature = "stm32l4p5",
    // feature = "stm32l4q5",
    feature = "stm32l4r5",
    feature = "stm32l4s5",
    // feature = "stm32l4r7",
    // feature = "stm32l4s7",
    feature = "stm32l4r9",
    feature = "stm32l4s9"
)))]
macro_rules! cselr {
    ($($DMAX_CY_SEL:ident: {
        $( ($field:ident, $bits:literal, [ $( $input:path ),+ ]), )+
    },)+) => {
        $(
            #[derive(Clone, Copy, Debug, PartialEq)]
            #[repr(u8)]
            enum $DMAX_CY_SEL {
                $(
                    $field = $bits,
                )+
            }
            impl From<$DMAX_CY_SEL> for u8 {
                #[inline(always)]
                fn from(variant: $DMAX_CY_SEL) -> Self {
                    variant as _
                }
            }
            impl TryFrom<DmaInput> for $DMAX_CY_SEL {
                type Error = Error;

                #[inline(always)]
                fn try_from(variant: DmaInput) -> Result<Self, Self::Error> {
                    match variant {
                        $(
                            $(
                                $input => Ok(Self::$field),
                            )+
                        )+
                        _ => Err(Error::Invalid),
                    }
                }
            }
        )+
    };
}

#[cfg(not(any(
    // feature = "stm32l4p5",
    // feature = "stm32l4q5",
    feature = "stm32l4r5",
    feature = "stm32l4s5",
    // feature = "stm32l4r7",
    // feature = "stm32l4s7",
    feature = "stm32l4r9",
    feature = "stm32l4s9"
)))]
cselr! {
    DMA1_C1_SEL: {
        (ADC1, 0b0000, [DmaInput::Adc1]),
        (TIM2_CH3, 0b0100, [DmaInput::Tim2Ch3]),
        (TIM17_CH1_TIM17_UP, 0b0101, [DmaInput::Tim17Ch1, DmaInput::Tim17Up]),
        (TIM4_CH1, 0b0110, [DmaInput::Tim4Ch1]),
    },
    DMA1_C2_SEL: {
        (ADC2, 0b0000, [DmaInput::Adc2]),
        (SPI1_RX, 0b0001, [DmaInput::Spi1Rx]),
        (USART3_TX, 0b0010, [DmaInput::Usart3Tx]),
        (I2C3_TX, 0b0011, [DmaInput::I2c3Tx]),
        (TIM2_UP, 0b0100, [DmaInput::Tim2Up]),
        (TIM3_CH3, 0b0101, [DmaInput::Tim3Ch3]),
        (TIM1_CH1, 0b0111, [DmaInput::Tim1Ch1]),
    },
    DMA1_C3_SEL: {
        (ADC3, 0b0000, [DmaInput::Adc3]),
        (SPI1_TX, 0b0001, [DmaInput::Spi1Tx]),
        (USART3_RX, 0b0010, [DmaInput::Usart3Rx]),
        (I2C3_RX, 0b0011, [DmaInput::I2c3Rx]),
        (TIM16_CH1_TIM16_UP, 0b0100, [DmaInput::Tim16Ch1, DmaInput::Tim16Up]),
        (TIM3_CH4_TIM3_UP, 0b0101, [DmaInput::Tim3Ch4, DmaInput::Tim3Up]),
        (TIM6_UP_DAC_CH1, 0b0110, [DmaInput::Tim6Up, DmaInput::Dac1Ch1]),
        (TIM1_CH2, 0b0111, [DmaInput::Tim1Ch2]),
    },
    DMA1_C4_SEL: {
        (DFSDM1_FLT0, 0b0000, [DmaInput::Dfsdm1Flt0]),
        (SPI2_RX, 0b0001, [DmaInput::Spi2Rx]),
        (USART1_TX, 0b0010, [DmaInput::Usart1Tx]),
        (I2C2_TX, 0b0011, [DmaInput::I2c2Tx]),
        (TIM7_UP_DAC_CH2, 0b0101, [DmaInput::Tim7Up, DmaInput::Dac1Ch2]),
        (TIM4_CH2, 0b0110, [DmaInput::Tim4Ch2]),
        (TIM1_CH4_TIM1_TRIG_TIM1_COM, 0b0111, [DmaInput::Tim1Ch4, DmaInput::Tim1Trig, DmaInput::Tim1Com]),
    },
    DMA1_C5_SEL: {
        (DFSDM1_FLT1, 0b0000, [DmaInput::Dfsdm1Flt1]),
        (SPI2_TX, 0b0001, [DmaInput::Spi2Tx]),
        (USART1_RX, 0b0010, [DmaInput::Usart1Rx]),
        (I2C2_RX, 0b0011, [DmaInput::I2c2Rx]),
        (TIM2_CH1, 0b0100, [DmaInput::Tim2Ch1]),
        (QUADSPI, 0b0101, [DmaInput::QuadSpi]),
        (TIM4_CH3, 0b0110, [DmaInput::Tim4Ch3]),
        (TIM15_CH1_TIM15_UP_TIM15_TRIG_TIM15_COM, 0b0111, [DmaInput::Tim15Ch1, DmaInput::Tim15Up, DmaInput::Tim15Trig, DmaInput::Tim15Com]),
    },
    DMA1_C6_SEL: {
        (DFSDM1_FLT2, 0b0000, [DmaInput::Dfsdm1Flt2]),
        (SAI2_A, 0b0001, [DmaInput::Sai2A]),
        (USART2_RX, 0b0010, [DmaInput::Usart2Rx]),
        (I2C1_TX, 0b0011, [DmaInput::I2c1Tx]),
        (TIM16_CH1_TIM16_UP, 0b0100, [DmaInput::Tim16Ch1, DmaInput::Tim16Up]),
        (TIM3_CH1_TIM3_TRIG, 0b0101, [DmaInput::Tim3Ch1, DmaInput::Tim3Trig]),
        (TIM1_UP, 0b0111, [DmaInput::Tim1Up]),
    },
    DMA1_C7_SEL: {
        (DFSDM1_FLT3, 0b0000, [DmaInput::Dfsdm1Flt3]),
        (SAI2_B, 0b0001, [DmaInput::Sai2B]),
        (USART2_TX, 0b0010, [DmaInput::Usart2Tx]),
        (I2C1_RX, 0b0011, [DmaInput::I2c1Rx]),
        (TIM2_CH2_TIM2_CH4, 0b0100, [DmaInput::Tim2Ch2, DmaInput::Tim2Ch4]),
        (TIM17_CH1_TIM17_UP, 0b0101, [DmaInput::Tim17Ch1, DmaInput::Tim17Up]),
        (TIM4_UP, 0b0110, [DmaInput::Tim4Up]),
        (TIM1_CH3, 0b0111, [DmaInput::Tim1Ch3]),
    },
    DMA2_C1_SEL: {
        (I2C4_RX, 0b0000, [DmaInput::I2c4Rx]),
        (SAI1_A, 0b0001, [DmaInput::Sai1A]),
        (UART5_TX, 0b0010, [DmaInput::Uart5Tx]),
        (SPI3_RX, 0b0011, [DmaInput::Spi3Rx]),
        (SWPMI1_RX, 0b0100, [DmaInput::Swpmi1Rx]),
        (TIM5_CH4_TIM5_TRIG, 0b0101, [DmaInput::Tim5Ch4, DmaInput::Tim5Trig]),
        (AES_IN, 0b0110, [DmaInput::AesIn]),
        (TIM8_CH3_TIM8_UP, 0b0111, [DmaInput::Tim8Ch3, DmaInput::Tim8Up]),
    },
    DMA2_C2_SEL: {
        (I2C4_TX, 0b0000, [DmaInput::I2c4Tx]),
        (SAI1_B, 0b0001, [DmaInput::Sai1B]),
        (UART5_RX, 0b0010, [DmaInput::Uart5Rx]),
        (SPI3_TX, 0b0011, [DmaInput::Spi3Tx]),
        (SWPMI1_TX, 0b0100, [DmaInput::Swpmi1Tx]),
        (TIM5_CH3_TIM5_UP, 0b0101, [DmaInput::Tim5Ch3, DmaInput::Tim5Up]),
        (AES_OUT, 0b0110, [DmaInput::AesOut]),
        (TIM8_CH4_TIM8_TRIG_TIM8_COM, 0b0111, [DmaInput::Tim8Ch4, DmaInput::Tim8Trig, DmaInput::Tim8Com]),
    },
    DMA2_C3_SEL: {
        (ADC1, 0b0000, [DmaInput::Adc1]),
        (SAI2_A, 0b0001, [DmaInput::Sai2A]),
        (UART4_TX, 0b0010, [DmaInput::Uart4Tx]),
        (SPI1_RX, 0b0100, [DmaInput::Spi1Rx]),
        (AES_OUT, 0b0110, [DmaInput::AesOut]),
    },
    DMA2_C4_SEL: {
        (ADC2, 0b0000, [DmaInput::Adc2]),
        (SAI2_B, 0b0001, [DmaInput::Sai2B]),
        (TIM6_UP_DAC_CH1, 0b0011, [DmaInput::Tim6Up, DmaInput::Dac1Ch1]),
        (SPI1_TX, 0b0100, [DmaInput::Spi1Tx]),
        (TIM5_CH2, 0b0101, [DmaInput::Tim5Ch2]),
        (SDMMC1, 0b0111, [DmaInput::SdMmc1]),
    },
    DMA2_C5_SEL: {
        (ADC3, 0b0000, [DmaInput::Adc3]),
        (UART4_RX, 0b0010, [DmaInput::Uart4Rx]),
        (TIM7_UP_DAC_CH2, 0b0011, [DmaInput::Tim7Up, DmaInput::Dac1Ch2]),
        (DCMI, 0b0100, [DmaInput::Dcmi]),
        (TIM5_CH1, 0b0101, [DmaInput::Tim5Ch1]),
        (AES_IN, 0b0110, [DmaInput::AesIn]),
        (SDMMC1, 0b0111, [DmaInput::SdMmc1]),
    },
    DMA2_C6_SEL: {
        (DCMI, 0b0000, [DmaInput::Dcmi]),
        (SAI1_A, 0b0001, [DmaInput::Sai1A]),
        (USART1_TX, 0b0010, [DmaInput::Usart1Tx]),
        (LPUART1_TX, 0b0100, [DmaInput::LpUart1Tx]),
        (I2C1_RX, 0b0101, [DmaInput::I2c1Rx]),
        (TIM8_CH1, 0b0111, [DmaInput::Tim8Ch1]),
    },
    DMA2_C7_SEL: {
        (SAI1_B, 0b0001, [DmaInput::Sai1B]),
        (USART1_RX, 0b0010, [DmaInput::Usart1Rx]),
        (QUADSPI, 0b0011, [DmaInput::QuadSpi]),
        (LPUART1_RX, 0b0100, [DmaInput::LpUart1Rx]),
        (I2C1_TX, 0b0101, [DmaInput::I2c1Tx]),
        (HASH_IN, 0b0110, [DmaInput::HashIn]),
        (TIM8_CH2, 0b0111, [DmaInput::Tim8Ch2]),
    },
}

pub trait DmaMux {
    fn set_request_line(&mut self, request_line: DmaInput) -> Result<(), Error>;
}

macro_rules! dmamux {
    ($($dmaX:ident: { $( $CY:ident: ($cYcr:ident, $cYs:ident, $DMAX_CY_SEL:ident), )+ },)+) => {
        $(
            $(
                impl DmaMux for $dmaX::$CY {
                    #[cfg(any(
                        // feature = "stm32l4p5",
                        // feature = "stm32l4q5",
                        feature = "stm32l4r5",
                        feature = "stm32l4s5",
                        // feature = "stm32l4r7",
                        // feature = "stm32l4s7",
                        feature = "stm32l4r9",
                        feature = "stm32l4s9"
                    ))]
                    #[inline(always)]
                    fn set_request_line(&mut self, request_line: DmaInput) -> Result<(), Error> {
                        let dmareq_id_a: DMAREQ_ID_A = request_line.try_into()?;
                        let mux = unsafe { &(*DMAMUX::ptr()) };
                        unsafe {
                            mux.$cYcr.modify(|_, w| w.dmareq_id().bits(dmareq_id_a.into()));
                        }

                        Ok(())
                    }

                    #[cfg(not(any(
                        // feature = "stm32l4p5",
                        // feature = "stm32l4q5",
                        feature = "stm32l4r5",
                        feature = "stm32l4s5",
                        // feature = "stm32l4r7",
                        // feature = "stm32l4s7",
                        feature = "stm32l4r9",
                        feature = "stm32l4s9"
                    )))]
                    #[inline(always)]
                    fn set_request_line(&mut self, request_line: DmaInput) -> Result<(), Error> {
                        let csel_val: $DMAX_CY_SEL = request_line.try_into()?;
                        self.cselr().modify(|_, w| w.$cYs().bits(csel_val.into()));

                        Ok(())
                    }
                }
            )+
        )+
    };
}

dmamux! {
    dma1: {
        C1: (c0cr, c1s, DMA1_C1_SEL),
        C2: (c1cr, c2s, DMA1_C2_SEL),
        C3: (c2cr, c3s, DMA1_C3_SEL),
        C4: (c3cr, c4s, DMA1_C4_SEL),
        C5: (c4cr, c5s, DMA1_C5_SEL),
        C6: (c5cr, c6s, DMA1_C6_SEL),
        C7: (c6cr, c7s, DMA1_C7_SEL),
    },
    dma2: {
        C1: (c7cr, c1s, DMA2_C1_SEL),
        C2: (c8cr, c2s, DMA2_C2_SEL),
        C3: (c9cr, c3s, DMA2_C3_SEL),
        C4: (c10cr, c4s, DMA2_C4_SEL),
        C5: (c11cr, c5s, DMA2_C5_SEL),
        C6: (c12cr, c6s, DMA2_C6_SEL),
        C7: (c13cr, c7s, DMA2_C7_SEL),
    },
}
