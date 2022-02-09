use crate::{features::*, FeatureGate};

/// list of peripherals to be gated and whether they are present for the selected features
/// see [crate::features::generate_internal_features()] for how to reference these
pub(crate) const PERIPHERAL_FEATURES: &[FeatureGate] = &[
    FeatureGate {
        name: "adc1",
        state: true,
    },
    FeatureGate {
        name: "adc2",
        state: IS_FEATURE_ENABLED_L412
            || IS_FEATURE_ENABLED_L422
            || IS_FEATURE_ENABLED_L471
            || IS_FEATURE_ENABLED_L475
            || IS_FEATURE_ENABLED_L476
            || IS_FEATURE_ENABLED_L485
            || IS_FEATURE_ENABLED_L486
            || IS_FEATURE_ENABLED_L496
            || IS_FEATURE_ENABLED_L4A6
            || IS_FEATURE_ENABLED_L4P5
            || IS_FEATURE_ENABLED_L4Q5,
    },
    FeatureGate {
        name: "adc3",
        state: IS_FEATURE_ENABLED_L471
            || IS_FEATURE_ENABLED_L475
            || IS_FEATURE_ENABLED_L476
            || IS_FEATURE_ENABLED_L485
            || IS_FEATURE_ENABLED_L486
            || IS_FEATURE_ENABLED_L496
            || IS_FEATURE_ENABLED_L4A6,
    },
    FeatureGate {
        name: "dac1",
        state: IS_FEATURE_ENABLED_L431
            || IS_FEATURE_ENABLED_L432
            || IS_FEATURE_ENABLED_L433
            || IS_FEATURE_ENABLED_L442
            || IS_FEATURE_ENABLED_L443
            || IS_FEATURE_ENABLED_L451
            || IS_FEATURE_ENABLED_L452
            || IS_FEATURE_ENABLED_L462
            || IS_FEATURE_ENABLED_L471
            || IS_FEATURE_ENABLED_L475
            || IS_FEATURE_ENABLED_L476
            || IS_FEATURE_ENABLED_L485
            || IS_FEATURE_ENABLED_L486
            || IS_FEATURE_ENABLED_L496
            || IS_FEATURE_ENABLED_L4A6
            || IS_FEATURE_ENABLED_L4P5
            || IS_FEATURE_ENABLED_L4Q5
            || IS_FEATURE_ENABLED_L4R5
            || IS_FEATURE_ENABLED_L4S5
            || IS_FEATURE_ENABLED_L4R7
            || IS_FEATURE_ENABLED_L4S7
            || IS_FEATURE_ENABLED_L4R9
            || IS_FEATURE_ENABLED_L4S9,
    },
    FeatureGate {
        name: "comp1",
        state: true,
    },
    FeatureGate {
        name: "comp2",
        state: !(IS_FEATURE_ENABLED_L412 || IS_FEATURE_ENABLED_L422),
    },
    FeatureGate {
        name: "dfsdm1",
        state: IS_FEATURE_ENABLED_L451
            || IS_FEATURE_ENABLED_L452
            || IS_FEATURE_ENABLED_L462
            || IS_FEATURE_ENABLED_L471
            || IS_FEATURE_ENABLED_L475
            || IS_FEATURE_ENABLED_L476
            || IS_FEATURE_ENABLED_L485
            || IS_FEATURE_ENABLED_L486
            || IS_FEATURE_ENABLED_L496
            || IS_FEATURE_ENABLED_L4A6
            || IS_FEATURE_ENABLED_L4P5
            || IS_FEATURE_ENABLED_L4Q5
            || IS_FEATURE_ENABLED_L4R5
            || IS_FEATURE_ENABLED_L4S5
            || IS_FEATURE_ENABLED_L4R7
            || IS_FEATURE_ENABLED_L4S7
            || IS_FEATURE_ENABLED_L4R9
            || IS_FEATURE_ENABLED_L4S9,
    },
    FeatureGate {
        name: "lcd",
        state: IS_FEATURE_ENABLED_L433
            || IS_FEATURE_ENABLED_L443
            || IS_FEATURE_ENABLED_L476
            || IS_FEATURE_ENABLED_L486
            || IS_FEATURE_ENABLED_L496
            || IS_FEATURE_ENABLED_L4A6,
    },
    FeatureGate {
        name: "aes",
        state: IS_FEATURE_ENABLED_L422
            || IS_FEATURE_ENABLED_L442
            || IS_FEATURE_ENABLED_L443
            || IS_FEATURE_ENABLED_L462
            || IS_FEATURE_ENABLED_L485
            || IS_FEATURE_ENABLED_L486
            || IS_FEATURE_ENABLED_L4A6
            || IS_FEATURE_ENABLED_L4Q5
            || IS_FEATURE_ENABLED_L4S5
            || IS_FEATURE_ENABLED_L4S7
            || IS_FEATURE_ENABLED_L4S9,
    },
    FeatureGate {
        name: "tim1",
        state: true,
    },
    FeatureGate {
        name: "tim2",
        state: true,
    },
    FeatureGate {
        name: "tim3",
        state: /*IS_FEATURE_ENABLED_L451 -- missing PAC support
            || */IS_FEATURE_ENABLED_L452
            || IS_FEATURE_ENABLED_L462
            //|| IS_FEATURE_ENABLED_L471 -- missing PAC support
            || IS_FEATURE_ENABLED_L475
            || IS_FEATURE_ENABLED_L476
            || IS_FEATURE_ENABLED_L485
            || IS_FEATURE_ENABLED_L486
            || IS_FEATURE_ENABLED_L496
            || IS_FEATURE_ENABLED_L4A6
            || IS_FEATURE_ENABLED_L4P5
            || IS_FEATURE_ENABLED_L4Q5
            || IS_FEATURE_ENABLED_L4R5
            || IS_FEATURE_ENABLED_L4S5
            || IS_FEATURE_ENABLED_L4R7
            || IS_FEATURE_ENABLED_L4S7
            || IS_FEATURE_ENABLED_L4R9
            || IS_FEATURE_ENABLED_L4S9,
    },
    FeatureGate {
        name: "tim4",
        state: /*IS_FEATURE_ENABLED_L471 -- missing PAC suport
            || */IS_FEATURE_ENABLED_L475
            || IS_FEATURE_ENABLED_L476
            || IS_FEATURE_ENABLED_L485
            || IS_FEATURE_ENABLED_L486
            || IS_FEATURE_ENABLED_L496
            || IS_FEATURE_ENABLED_L4A6
            || IS_FEATURE_ENABLED_L4P5
            || IS_FEATURE_ENABLED_L4Q5
            || IS_FEATURE_ENABLED_L4R5
            || IS_FEATURE_ENABLED_L4S5
            || IS_FEATURE_ENABLED_L4R7
            || IS_FEATURE_ENABLED_L4S7
            || IS_FEATURE_ENABLED_L4R9
            || IS_FEATURE_ENABLED_L4S9,
    },
    FeatureGate {
        name: "tim5",
        state: /*IS_FEATURE_ENABLED_L471 -- missing PAC suport
            || */IS_FEATURE_ENABLED_L475
            || IS_FEATURE_ENABLED_L476
            || IS_FEATURE_ENABLED_L485
            || IS_FEATURE_ENABLED_L486
            || IS_FEATURE_ENABLED_L496
            || IS_FEATURE_ENABLED_L4A6
            || IS_FEATURE_ENABLED_L4P5
            || IS_FEATURE_ENABLED_L4Q5
            || IS_FEATURE_ENABLED_L4R5
            || IS_FEATURE_ENABLED_L4S5
            || IS_FEATURE_ENABLED_L4R7
            || IS_FEATURE_ENABLED_L4S7
            || IS_FEATURE_ENABLED_L4R9
            || IS_FEATURE_ENABLED_L4S9,
    },
    FeatureGate {
        name: "tim6",
        state: true,
    },
    FeatureGate {
        name: "tim7",
        state: IS_FEATURE_ENABLED_L431
            || IS_FEATURE_ENABLED_L432
            || IS_FEATURE_ENABLED_L433
            || IS_FEATURE_ENABLED_L442
            || IS_FEATURE_ENABLED_L443
            || IS_FEATURE_ENABLED_L471
            || IS_FEATURE_ENABLED_L475
            || IS_FEATURE_ENABLED_L476
            || IS_FEATURE_ENABLED_L485
            || IS_FEATURE_ENABLED_L486
            || IS_FEATURE_ENABLED_L496
            || IS_FEATURE_ENABLED_L4A6
            || IS_FEATURE_ENABLED_L4P5
            || IS_FEATURE_ENABLED_L4Q5
            || IS_FEATURE_ENABLED_L4R5
            || IS_FEATURE_ENABLED_L4S5
            || IS_FEATURE_ENABLED_L4R7
            || IS_FEATURE_ENABLED_L4S7
            || IS_FEATURE_ENABLED_L4R9
            || IS_FEATURE_ENABLED_L4S9,
    },
    FeatureGate {
        name: "tim8",
        state: IS_FEATURE_ENABLED_L471
            || IS_FEATURE_ENABLED_L475
            || IS_FEATURE_ENABLED_L476
            || IS_FEATURE_ENABLED_L485
            || IS_FEATURE_ENABLED_L486
            || IS_FEATURE_ENABLED_L496
            || IS_FEATURE_ENABLED_L4A6
            || IS_FEATURE_ENABLED_L4P5
            || IS_FEATURE_ENABLED_L4Q5
            || IS_FEATURE_ENABLED_L4R5
            || IS_FEATURE_ENABLED_L4S5
            || IS_FEATURE_ENABLED_L4R7
            || IS_FEATURE_ENABLED_L4S7
            || IS_FEATURE_ENABLED_L4R9
            || IS_FEATURE_ENABLED_L4S9,
    },
    FeatureGate {
        name: "tim15",
        state: true,
    },
    FeatureGate {
        name: "tim16",
        state: true,
    },
    FeatureGate {
        name: "tim17",
        state: /*IS_FEATURE_ENABLED_L471 -- missing PAC suport
            || */IS_FEATURE_ENABLED_L475
            || IS_FEATURE_ENABLED_L476
            || IS_FEATURE_ENABLED_L485
            || IS_FEATURE_ENABLED_L486
            || IS_FEATURE_ENABLED_L496
            || IS_FEATURE_ENABLED_L4A6
            || IS_FEATURE_ENABLED_L4P5
            || IS_FEATURE_ENABLED_L4Q5
            || IS_FEATURE_ENABLED_L4R5
            || IS_FEATURE_ENABLED_L4S5
            || IS_FEATURE_ENABLED_L4R7
            || IS_FEATURE_ENABLED_L4S7
            || IS_FEATURE_ENABLED_L4R9
            || IS_FEATURE_ENABLED_L4S9,
    },
    FeatureGate {
        name: "lptim1",
        state: true,
    },
    FeatureGate {
        name: "lptim2",
        state: true,
    },
    FeatureGate {
        name: "i2c1",
        state: true,
    },
    FeatureGate {
        name: "i2c2",
        state: !(IS_FEATURE_ENABLED_L432 || IS_FEATURE_ENABLED_L442),
    },
    FeatureGate {
        name: "i2c3",
        state: true,
    },
    FeatureGate {
        name: "i2c4",
        state: IS_FEATURE_ENABLED_L451
            || IS_FEATURE_ENABLED_L452
            || IS_FEATURE_ENABLED_L462
            || IS_FEATURE_ENABLED_L496
            || IS_FEATURE_ENABLED_L4A6
            || IS_FEATURE_ENABLED_L4P5
            || IS_FEATURE_ENABLED_L4Q5
            || IS_FEATURE_ENABLED_L4R5
            || IS_FEATURE_ENABLED_L4S5
            || IS_FEATURE_ENABLED_L4R7
            || IS_FEATURE_ENABLED_L4S7
            || IS_FEATURE_ENABLED_L4R9
            || IS_FEATURE_ENABLED_L4S9,
    },
    FeatureGate {
        name: "usart1",
        state: true,
    },
    FeatureGate {
        name: "usart2",
        state: true,
    },
    FeatureGate {
        name: "usart3",
        state: !(IS_FEATURE_ENABLED_L432 || IS_FEATURE_ENABLED_L442),
    },
    FeatureGate {
        name: "uart4",
        state: IS_FEATURE_ENABLED_L451
            || IS_FEATURE_ENABLED_L452
            || IS_FEATURE_ENABLED_L462
            || IS_FEATURE_ENABLED_L471
            || IS_FEATURE_ENABLED_L475
            || IS_FEATURE_ENABLED_L476
            || IS_FEATURE_ENABLED_L485
            || IS_FEATURE_ENABLED_L486
            || IS_FEATURE_ENABLED_L496
            || IS_FEATURE_ENABLED_L4A6
            || IS_FEATURE_ENABLED_L4P5
            || IS_FEATURE_ENABLED_L4Q5
            || IS_FEATURE_ENABLED_L4R5
            || IS_FEATURE_ENABLED_L4S5
            || IS_FEATURE_ENABLED_L4R7
            || IS_FEATURE_ENABLED_L4S7
            || IS_FEATURE_ENABLED_L4R9
            || IS_FEATURE_ENABLED_L4S9,
    },
    FeatureGate {
        name: "uart5",
        state: //IS_FEATURE_ENABLED_L471 -- missing PAC support
             IS_FEATURE_ENABLED_L475
            || IS_FEATURE_ENABLED_L476
            || IS_FEATURE_ENABLED_L485
            || IS_FEATURE_ENABLED_L486
            || IS_FEATURE_ENABLED_L496
            || IS_FEATURE_ENABLED_L4A6
            || IS_FEATURE_ENABLED_L4P5
            || IS_FEATURE_ENABLED_L4Q5
            || IS_FEATURE_ENABLED_L4R5
            || IS_FEATURE_ENABLED_L4S5
            || IS_FEATURE_ENABLED_L4R7
            || IS_FEATURE_ENABLED_L4S7
            || IS_FEATURE_ENABLED_L4R9
            || IS_FEATURE_ENABLED_L4S9,
    },
    FeatureGate {
        name: "lpuart1",
        state: true,
    },
    FeatureGate {
        name: "tim16",
        state: true,
    },
    FeatureGate {
        name: "spi1",
        state: true,
    },
    FeatureGate {
        name: "spi2",
        state: !(IS_FEATURE_ENABLED_L432 || IS_FEATURE_ENABLED_L442),
    },
    FeatureGate {
        name: "spi3",
        state: !(IS_FEATURE_ENABLED_L412 || IS_FEATURE_ENABLED_L422),
    },
    FeatureGate {
        name: "qspi",
        state: IS_FEATURE_ENABLED_L412
            || IS_FEATURE_ENABLED_L422
            || IS_FEATURE_ENABLED_L431
            || IS_FEATURE_ENABLED_L432
            //|| IS_FEATURE_ENABLED_L433 -- missing PAC support
            || IS_FEATURE_ENABLED_L442
            //|| IS_FEATURE_ENABLED_L443 -- missing PAC support
            || IS_FEATURE_ENABLED_L451
            || IS_FEATURE_ENABLED_L452
            || IS_FEATURE_ENABLED_L462
            || IS_FEATURE_ENABLED_L471
            || IS_FEATURE_ENABLED_L475
            || IS_FEATURE_ENABLED_L476
            || IS_FEATURE_ENABLED_L485
            || IS_FEATURE_ENABLED_L486
            || IS_FEATURE_ENABLED_L496
            || IS_FEATURE_ENABLED_L4A6,
    },
    FeatureGate {
        name: "sai1",
        state: !(IS_FEATURE_ENABLED_L412 || IS_FEATURE_ENABLED_L422),
    },
    FeatureGate {
        name: "sai2",
        state: IS_FEATURE_ENABLED_L471
            || IS_FEATURE_ENABLED_L475
            || IS_FEATURE_ENABLED_L476
            || IS_FEATURE_ENABLED_L485
            || IS_FEATURE_ENABLED_L486
            || IS_FEATURE_ENABLED_L496
            || IS_FEATURE_ENABLED_L4A6
            || IS_FEATURE_ENABLED_L4P5
            || IS_FEATURE_ENABLED_L4Q5
            || IS_FEATURE_ENABLED_L4R5
            || IS_FEATURE_ENABLED_L4S5
            || IS_FEATURE_ENABLED_L4R7
            || IS_FEATURE_ENABLED_L4S7
            || IS_FEATURE_ENABLED_L4R9
            || IS_FEATURE_ENABLED_L4S9,
    },
    FeatureGate {
        name: "swpmi1",
        state: IS_FEATURE_ENABLED_L431
            || IS_FEATURE_ENABLED_L432
            || IS_FEATURE_ENABLED_L433
            || IS_FEATURE_ENABLED_L442
            || IS_FEATURE_ENABLED_L443
            || IS_FEATURE_ENABLED_L471
            || IS_FEATURE_ENABLED_L475
            || IS_FEATURE_ENABLED_L476
            || IS_FEATURE_ENABLED_L485
            || IS_FEATURE_ENABLED_L486
            || IS_FEATURE_ENABLED_L496
            || IS_FEATURE_ENABLED_L4A6,
    },
    FeatureGate {
        name: "sdmmc1",
        state: !(IS_FEATURE_ENABLED_L412
            || IS_FEATURE_ENABLED_L422
            || IS_FEATURE_ENABLED_L432
            || IS_FEATURE_ENABLED_L442),
    },
    FeatureGate {
        name: "usb_device_fs",
        state: IS_FEATURE_ENABLED_L412
            || IS_FEATURE_ENABLED_L422
            || IS_FEATURE_ENABLED_L432
            || IS_FEATURE_ENABLED_L433
            || IS_FEATURE_ENABLED_L442
            || IS_FEATURE_ENABLED_L443
            || IS_FEATURE_ENABLED_L452
            || IS_FEATURE_ENABLED_L462,
    },
    FeatureGate {
        name: "usb_otg_fs",
        state: IS_FEATURE_ENABLED_L475
            || IS_FEATURE_ENABLED_L476
            || IS_FEATURE_ENABLED_L485
            || IS_FEATURE_ENABLED_L486
            || IS_FEATURE_ENABLED_L496
            || IS_FEATURE_ENABLED_L4A6
            || IS_FEATURE_ENABLED_L4P5
            || IS_FEATURE_ENABLED_L4Q5
            || IS_FEATURE_ENABLED_L4R5
            || IS_FEATURE_ENABLED_L4S5
            || IS_FEATURE_ENABLED_L4R7
            || IS_FEATURE_ENABLED_L4S7
            || IS_FEATURE_ENABLED_L4R9
            || IS_FEATURE_ENABLED_L4S9,
    },
    FeatureGate {
        name: "can1",
        state: !(IS_FEATURE_ENABLED_L412 || IS_FEATURE_ENABLED_L422),
    },
    FeatureGate {
        name: "can2",
        state: IS_FEATURE_ENABLED_L496 || IS_FEATURE_ENABLED_L4A6,
    },
    FeatureGate {
        name: "rtc",
        state: true,
    },
    FeatureGate {
        name: "gpioa",
        state: true,
    },
    FeatureGate {
        name: "gpiob",
        state: true,
    },
    FeatureGate {
        name: "gpioc",
        state: true,
    },
    FeatureGate {
        name: "gpiod",
        state: true,
    },
    FeatureGate {
        name: "gpioe",
        state: true,
    },
    FeatureGate {
        name: "gpiof",
        state: //IS_FEATURE_ENABLED_L471 -- missing PAC support
                IS_FEATURE_ENABLED_L475
            || IS_FEATURE_ENABLED_L476
            || IS_FEATURE_ENABLED_L485
            || IS_FEATURE_ENABLED_L486
            || IS_FEATURE_ENABLED_L496
            || IS_FEATURE_ENABLED_L4A6
            || IS_FEATURE_ENABLED_L4P5
            || IS_FEATURE_ENABLED_L4Q5
            || IS_FEATURE_ENABLED_L4R5
            || IS_FEATURE_ENABLED_L4S5
            || IS_FEATURE_ENABLED_L4R7
            || IS_FEATURE_ENABLED_L4S7
            || IS_FEATURE_ENABLED_L4R9
            || IS_FEATURE_ENABLED_L4S9,
    },
    FeatureGate {
        name: "gpiog",
        state: //IS_FEATURE_ENABLED_L471 -- missing PAC support
                IS_FEATURE_ENABLED_L475
            || IS_FEATURE_ENABLED_L476
            || IS_FEATURE_ENABLED_L485
            || IS_FEATURE_ENABLED_L486
            || IS_FEATURE_ENABLED_L496
            || IS_FEATURE_ENABLED_L4A6
            || IS_FEATURE_ENABLED_L4P5
            || IS_FEATURE_ENABLED_L4Q5
            || IS_FEATURE_ENABLED_L4R5
            || IS_FEATURE_ENABLED_L4S5
            || IS_FEATURE_ENABLED_L4R7
            || IS_FEATURE_ENABLED_L4S7
            || IS_FEATURE_ENABLED_L4R9
            || IS_FEATURE_ENABLED_L4S9,
    },
    FeatureGate {
        name: "gpioh",
        state: true,
    },
    FeatureGate {
        name: "gpioi",
        state: IS_FEATURE_ENABLED_L496
            || IS_FEATURE_ENABLED_L4A6
            || IS_FEATURE_ENABLED_L4P5
            || IS_FEATURE_ENABLED_L4Q5
            || IS_FEATURE_ENABLED_L4R5
            || IS_FEATURE_ENABLED_L4S5
            || IS_FEATURE_ENABLED_L4R7
            || IS_FEATURE_ENABLED_L4S7
            || IS_FEATURE_ENABLED_L4R9
            || IS_FEATURE_ENABLED_L4S9,
    },
    FeatureGate {
        // aka CRS
        name: "clock_recovery_system",
        state: !(IS_FEATURE_ENABLED_L471
            || IS_FEATURE_ENABLED_L475
            || IS_FEATURE_ENABLED_L476
            || IS_FEATURE_ENABLED_L485
            || IS_FEATURE_ENABLED_L486),
    },
    FeatureGate {
        name: "dac1",
        state: !(IS_FEATURE_ENABLED_L412 || IS_FEATURE_ENABLED_L422),
    },
    FeatureGate {
        name: "opamp1",
        state: true,
    },
    FeatureGate {
        name: "opamp2",
        state: IS_FEATURE_ENABLED_L471
            || IS_FEATURE_ENABLED_L475
            || IS_FEATURE_ENABLED_L476
            || IS_FEATURE_ENABLED_L485
            || IS_FEATURE_ENABLED_L486
            || IS_FEATURE_ENABLED_L496
            || IS_FEATURE_ENABLED_L4A6
            || IS_FEATURE_ENABLED_L4P5
            || IS_FEATURE_ENABLED_L4Q5
            || IS_FEATURE_ENABLED_L4R5
            || IS_FEATURE_ENABLED_L4S5
            || IS_FEATURE_ENABLED_L4R7
            || IS_FEATURE_ENABLED_L4S7
            || IS_FEATURE_ENABLED_L4R9
            || IS_FEATURE_ENABLED_L4S9,
    },
    FeatureGate {
        name: "dma1",
        state: true,
    },
    FeatureGate {
        name: "dma2",
        state: true,
    },
    FeatureGate {
        name: "crc",
        state: true,
    },
    FeatureGate {
        // aka touch sense controller
        name: "tsc",
        state: true,
    },
    FeatureGate {
        // aka digital camera memory interface
        name: "dcmi",
        state: IS_FEATURE_ENABLED_L496
            || IS_FEATURE_ENABLED_L4A6
            || IS_FEATURE_ENABLED_L4P5
            || IS_FEATURE_ENABLED_L4Q5
            || IS_FEATURE_ENABLED_L4R5
            || IS_FEATURE_ENABLED_L4S5
            || IS_FEATURE_ENABLED_L4R7
            || IS_FEATURE_ENABLED_L4S7
            || IS_FEATURE_ENABLED_L4R9
            || IS_FEATURE_ENABLED_L4S9,
    },
    FeatureGate {
        name: "dma2d",
        state: IS_FEATURE_ENABLED_L496
            || IS_FEATURE_ENABLED_L4A6
            || IS_FEATURE_ENABLED_L4P5
            || IS_FEATURE_ENABLED_L4Q5
            || IS_FEATURE_ENABLED_L4R5
            || IS_FEATURE_ENABLED_L4S5
            || IS_FEATURE_ENABLED_L4R7
            || IS_FEATURE_ENABLED_L4S7
            || IS_FEATURE_ENABLED_L4R9
            || IS_FEATURE_ENABLED_L4S9,
    },
    FeatureGate {
        name: "hash",
        state: IS_FEATURE_ENABLED_L4A6
            || IS_FEATURE_ENABLED_L4P5
            || IS_FEATURE_ENABLED_L4Q5
            || IS_FEATURE_ENABLED_L4S5
            || IS_FEATURE_ENABLED_L4S7
            || IS_FEATURE_ENABLED_L4S9,
    },
    FeatureGate {
        name: "rng",
        state: true,
    },
    FeatureGate {
        // aka flexible memory controller
        name: "fmc",
        state: true,
    },
    FeatureGate {
        name: "firewall",
        state: true,
    },
    FeatureGate {
        name: "vrefbuf",
        state: true,
    },
    FeatureGate {
        name: "iwdg",
        state: true,
    },
    FeatureGate {
        name: "wwdg",
        state: true,
    },
    FeatureGate {
        name: "dbgmcu",
        state: true,
    },
    FeatureGate {
        // aka LCD-TFT display controller
        name: "lcdt",
        state: IS_FEATURE_ENABLED_L4P5
            || IS_FEATURE_ENABLED_L4Q5
            || IS_FEATURE_ENABLED_L4R7
            || IS_FEATURE_ENABLED_L4R9
            || IS_FEATURE_ENABLED_L4S7
            || IS_FEATURE_ENABLED_L4S9,
    },
    FeatureGate {
        // aka MIPI display serial interface
        name: "dsi",
        state: IS_FEATURE_ENABLED_L4R9 || IS_FEATURE_ENABLED_L4S9,
    },
    FeatureGate {
        name: "dmamux1",
        state: IS_FEATURE_ENABLED_L4P5
            || IS_FEATURE_ENABLED_L4Q5
            || IS_FEATURE_ENABLED_L4R5
            || IS_FEATURE_ENABLED_L4R7
            || IS_FEATURE_ENABLED_L4R9
            || IS_FEATURE_ENABLED_L4S5
            || IS_FEATURE_ENABLED_L4S7
            || IS_FEATURE_ENABLED_L4S9,
    },
    FeatureGate {
        name: "octspi1",
        state: IS_FEATURE_ENABLED_L4P5
            || IS_FEATURE_ENABLED_L4Q5
            || IS_FEATURE_ENABLED_L4R5
            || IS_FEATURE_ENABLED_L4R7
            || IS_FEATURE_ENABLED_L4R9
            || IS_FEATURE_ENABLED_L4S5
            || IS_FEATURE_ENABLED_L4S7
            || IS_FEATURE_ENABLED_L4S9,
    },
    FeatureGate {
        name: "octspi2",
        state: IS_FEATURE_ENABLED_L4P5
            || IS_FEATURE_ENABLED_L4Q5
            || IS_FEATURE_ENABLED_L4R5
            || IS_FEATURE_ENABLED_L4R7
            || IS_FEATURE_ENABLED_L4R9
            || IS_FEATURE_ENABLED_L4S5
            || IS_FEATURE_ENABLED_L4S7
            || IS_FEATURE_ENABLED_L4S9,
    },
    FeatureGate {
        name: "gfxmmu",
        state: IS_FEATURE_ENABLED_L4R7
            || IS_FEATURE_ENABLED_L4R9
            || IS_FEATURE_ENABLED_L4S7
            || IS_FEATURE_ENABLED_L4S9,
    },
    FeatureGate {
        // aka public key accelerator
        name: "pka",
        state: IS_FEATURE_ENABLED_L4Q5,
    },
];

/// list of peripherals variants to be gated and whether they are present for the selected features
/// see [crate::features::generate_internal_features()] for how to reference these
pub(crate) const PERIPHERAL_VARIANTS: &[FeatureGate] = &[
    FeatureGate {
        name: "rtc_type3",
        state: IS_FEATURE_ENABLED_L412
            || IS_FEATURE_ENABLED_L422
            || IS_FEATURE_ENABLED_L4P5
            || IS_FEATURE_ENABLED_L4Q5,
    },
    FeatureGate {
        name: "rtc_type2",
        state: !(IS_FEATURE_ENABLED_L412
            || IS_FEATURE_ENABLED_L422
            || IS_FEATURE_ENABLED_L4P5
            || IS_FEATURE_ENABLED_L4Q5),
    },
];
